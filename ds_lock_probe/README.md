# ds_lock_probe

Minimal repro: a datastore key used as an instance lock is released on
SIGINT / SIGTERM but leaks on `peppy node stop`. Plain process node, no
hardware.

## The pattern

The node stores a datastore key on startup and removes it from a spawned
shutdown task that selects on SIGINT, SIGTERM, and the runtime cancellation
token (the documented graceful-shutdown handle). This is the instance-lock
pattern in openarm01_arm / openarm01_gripper.

## Observed (Jetson, peppy v0.10.5, 2026-06-10)

| trigger | result |
|---|---|
| SIGINT | released (~3 ms after signal) |
| SIGTERM | released (~3 ms after signal) |
| `peppy node stop` | **leaked**; the shutdown task produces no output at all |

After a `node stop`, restarting with the same key panics `LOCK_HELD` by the
dead instance, and the key persists until the core-node stack restarts.

## Why

`peppy node stop` sends no unix signal. The daemon sends an in-band
`SHUTDOWN_SERVICE` request (core-node-internal `services/node/stop.rs`),
peppylib acks it, and `NodeBuilder::run()` cancels the cancellation token and
returns immediately (the `shutdown_rx` select arm in
`peppylib/src/runtime/builder.rs::run_post_setup_services`). `main()` returns,
the tokio runtime drops, and the spawned shutdown task is never polled: the
probe logs show no `SHUTDOWN_TRIGGER` line, so the task did not even reach its
first statement after the select. The datastore remove never executes.

`lifecycle.shutdown_grace_secs` (default 3 s) is not the limiter. The node
exits "gracefully" in under half a second, so the grace window never starts to
matter; raising it changes nothing because nothing in the process waits for
the cleanup task. The window is unusable as designed: the token is the only
in-band shutdown handle a node gets, and cancelling it tears the process down
before any task observing it can run.

Anything else in that cancel arm has the same problem: for the real
arm/gripper nodes, the motor `disable_all()` on `peppy node stop` is equally
unreachable.

## Repro

```bash
cd ds_lock_probe
peppy node sync && peppy node add . && peppy node build ds_lock_probe:v1

# SIGINT / SIGTERM: full TRIGGER/PRE/POST sequence in the log, lock released
peppy node run ds_lock_probe:v1 lock_key=demo_lock -i demo-sig &
kill -INT <pid of ds_lock_probe>

# node stop: no shutdown output, lock leaks
peppy node run ds_lock_probe:v1 lock_key=demo_lock -i demo &
peppy node stop demo
peppy node run ds_lock_probe:v1 lock_key=demo_lock -i demo2   # panics LOCK_HELD by=demo

# reset between runs (no datastore CLI; key otherwise lives until stack restart)
peppy node run ds_lock_probe:v1 lock_key=demo_lock mode=clean -i demo-clean
```

Logs land in `~/.peppy/logs/run/<instance>.log`.

## What would fix it

A peppylib-level shutdown hook: after the `SHUTDOWN_SERVICE` ack cancels the
token, `run()` should wait (bounded by the daemon's grace window) for
registered cleanup before returning, instead of dropping the runtime under
tasks that observe the token. That would make the cancellation token usable
for hardware teardown and lock release on every stop path.
