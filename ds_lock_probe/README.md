# ds_lock_probe

Probe for the datastore instance-lock shutdown paths: a key stored on startup
must be removed on SIGINT, SIGTERM, and `peppy node stop`. Plain process node,
no hardware.

## The pattern

The node stores a datastore key on startup and removes it from a shutdown hook
registered with `node_runner.on_shutdown(...)`. The peppylib runtime owns every
stop path (it handles SIGINT/SIGTERM itself, acks the in-band
`SHUTDOWN_SERVICE` from `peppy node stop`, and reacts to daemon-liveness loss),
cancels the cancellation token, and then awaits the registered hooks, bounded
by `lifecycle.shutdown_grace_secs`, before `run()` returns. This is the
instance-lock pattern openarm01_arm / openarm01_gripper should use for motor
disable + lock release. The full contract (stop paths, hook ordering, grace
windows, migration from the old spawned-task pattern) is documented in the
shutdown lifecycle guide: https://dev.peppy.bot/advanced_guides/shutdown/

`mode=clean` doubles as a probe of a fourth stop path: it cancels the
cancellation token itself (programmatic cancel) and returns, exiting through
the same graceful sequence.

## History: the leak this node was built to reproduce

Observed on Jetson, peppy v0.10.5, 2026-06-10, with the old pattern (a spawned
task selecting on SIGINT / SIGTERM / the cancellation token):

| trigger | result |
|---|---|
| SIGINT | released |
| SIGTERM | released |
| `peppy node stop` | **leaked**; the shutdown task produced no output at all |

`peppy node stop` sends no unix signal. The daemon sends an in-band
`SHUTDOWN_SERVICE` request, peppylib acked it, cancelled the cancellation
token, and `run()` returned immediately. `main()` returned, the tokio runtime
dropped, and the spawned shutdown task was never polled again, so the
datastore remove never executed. Raising `lifecycle.shutdown_grace_secs`
changed nothing because nothing in the process waited for the cleanup task.

The fix (peppylib `NodeRunner::on_shutdown`): cleanup is registered as a hook
and `run()` awaits the hooks, within the grace window, after the token is
cancelled on every stop path. Spawned tasks observing the token remain the
right tool for stopping in-flight work, but cleanup must live in a hook.

## Expected behavior (post-fix)

Every trigger below must produce `SHUTDOWN_HOOK` / `REMOVE_PRE` /
`REMOVE_POST ok` in the log, and a restart with the same key must succeed:

```bash
cd ds_lock_probe
peppy node sync && peppy node add . && peppy node build ds_lock_probe:v1

# SIGINT / SIGTERM
peppy node run ds_lock_probe:v1 lock_key=demo_lock -i demo-sig
kill -INT <pid of ds_lock_probe>

# node stop
peppy node run ds_lock_probe:v1 lock_key=demo_lock -i demo
peppy node stop demo
peppy node run ds_lock_probe:v1 lock_key=demo_lock -i demo2   # must NOT panic LOCK_HELD

# janitor, for resetting after a (historical) leak without restarting the stack
peppy node run ds_lock_probe:v1 lock_key=demo_lock mode=clean -i demo-clean
```

Logs land in `~/.peppy/logs/run/<instance>.log`.

A run with `SHUTDOWN_HOOK` but no `REMOVE_POST` means the remove was cut off
by the grace window; no `SHUTDOWN_HOOK` at all means the hook never ran (the
original bug).
