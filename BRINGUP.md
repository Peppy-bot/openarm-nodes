# OpenArm gravity-comp bring-up runbook

Incrementally bring up gravity / Coriolis / friction compensation on hardware,
validating each step from the logs before the next. The arm node is **re-run with
new params per step** (fast); `srs_model` stays up the whole session. All
compensation is **off by default (fail-safe)** and ramped explicitly here.

> TEMPORARY: this references the bring-up instrumentation and ramp. Remove/update
> for production (see the cleanup notes at the end).

## Conventions

Set once per shell on the Pi:

```sh
export OPENARM=/home/jared/peppy/openarm01_nodes   # this repo
export NODES_HUB=/home/jared/peppy/nodes_hub       # holds srs_model
export CAN=left_follower                            # this arm's CAN interface
```

- Left arm: `arm_id=0`, `base_link=openarm_left_link0`.
- Stop a node: `peppy node stop <instance_id>` (or Ctrl-C its terminal).
- The arm prints diagnostics every `log_period_ms` (default 1000). Lower it
  (e.g. `log_period_ms=200`) to watch a fast step closely — runtime, no rebuild.
- Run each long-lived node in its own terminal.
- The arm's `min_motion_time_s` now **defaults to 5 s** (deliberately slow: the
  URDF velocity limits are high, so a shorter floor would let a move slam). The
  runs below still pass it explicitly for clarity; lower it once tracking is
  trusted.

## 0. Prerequisites

- 64-bit (aarch64) OS, `peppy` installed.
- `capnp --version` ≥ 0.5.2 on PATH (peppygen codegen needs it).
- `git pull` on this branch so the Pi has the latest.
- CAN interface up (same setup that passed the baseline motor-move test).
- Daemon running **in a shell that has capnp**: `peppy service serve` (leave up).
- **Physically support the arm** — in step 2 it is limp and will sag.
- E-stop in reach for every powered step.

## 1. Build the nodes (slow, once)

From each node directory, `peppy node add -sb` syncs, adds, and builds:

```sh
cd $NODES_HUB/srs_model        && peppy node add -sb
cd $OPENARM/openarm01_arm      && peppy node add -sb
cd $OPENARM/openarm01_arm_test && peppy node add -sb
```

(If an interface dependency isn't found, sync against the repos first:
`peppy node sync -r` then `peppy node add -b`.)

> **Build once, then only `run`.** A built node is in stage `Ready`. `peppy node
> run` executes that artifact and does **not** rebuild. Re-running `peppy node
> add` (or `peppy stack launch`, which adds every launch) re-snapshots the source
> and forces a **full recompile from scratch** (no cargo cache across container
> builds). So for the param ramp below we never re-add — we only `run` with new
> args. Don't use the launcher to iterate; it rebuilds every launch.

## 2. Start srs_model (stays up all session) — Terminal A

```sh
peppy node run srs_model:v1 -i srs_left_0 \
  urdf_path=$OPENARM/description/openarm_v10.urdf \
  base_link=openarm_left_link0 \
  --bind-deferred arm@arm_0 --idle-timeout 86400 --max-timeout 86400
```

Compensation is now **bidirectional streaming**: srs_model subscribes to the
arm's `joint_state` and publishes `compensation` back. So srs_model binds to the
arm (`arm@arm_0`), and the arm binds to srs_model (`model@srs_left_0`, step 2) —
a mutual binding. `--bind-deferred` lets you start srs_model first (it records
the binding and waits for `arm_0` to appear); the arm's plain `--bind` is fine
because srs_model is already up by then.

Confirm the startup line `srs_model loaded from base 'openarm_left_link0': arm
base at world [x, y, z] (verify this matches the mounting)`. The world
translation must match where this arm is physically mounted — a near-identity
translation on an off-origin arm means the URDF is missing the `world->base`
mount tree, which silently mis-orients gravity. Leave it running.

`srs_model` is request-driven and prints nothing after startup, so the
`--idle-timeout` clock (which resets only on output) never resets here: the
`86400` above is what keeps the node alive for the session, not a safety margin.
Don't shorten it, or the node exits mid-session while still serving requests.

## What to watch (arm logs, every `log_period_ms`)

- `config: arm_id=.. (left) rate=..Hz scales(..) ...` — confirms this run's params.
- `loop: N Hz (n=..), work avg/max, overruns (budget ..ms)`
- `comp state=.. fresh=.. age=..ms seq=.. scales(g.. c.. f..) max_drift=..rad / q / qdot / gravity / coriolis / friction / tau`
  (`state` is `float` or `trajectory`; `fresh`+`age` are the streamed
  compensation's freshness — `age` should stay well under `stale_timeout_ms`.
  When stale the model feedforward is dropped: float goes limp, a move keeps
  tracking on kp/kd. `tau` is the feedforward actually applied.)
- `track t=.. max_err=..rad / q_des / q / err` (only during a move)

---

## Step 2 — Observe (limp, no feedforward) — Terminal B

```sh
peppy node run openarm01_arm:v1 -i arm_0 \
  arm_id=0 can_interface=$CAN control_rate_hz=100 \
  gravity_scale=0 coriolis_scale=0 friction_scale=0 min_motion_time_s=5.0 \
  --bind model@srs_left_0 --idle-timeout 86400 --max-timeout 86400
```

Arm is **limp** (`tau≈0`, `state=float`). Judge:

- `comp` gravity magnitudes plausible (a few Nm on loaded joints, ~0 at the
  wrist), signs consistent with the pose; `tau≈0`.
- `comp fresh=true` and `age` small (well under `stale_timeout_ms`): the
  compensation stream is live. If `fresh=false` persists, the stream never
  arrived — check srs_model, the arm's `model` binding, and srs_model's `arm`
  binding (it must be subscribed to this arm's `joint_state`).
- `loop` holds ~100 Hz, `overruns=0`.

✅ comp sane + fresh → next. ❌ gravity wild/NaN, or stream never arrives → stop.

## Step 3 — Confirm loop timing

Stay on the step-2 run (limp, 100 Hz) and watch a few windows. Compensation now
**streams** (the arm publishes `joint_state`, srs_model pushes `compensation`),
so the control loop no longer does a per-tick round trip — the old ~2 ms
synchronous poll that capped the rate is gone. 100 Hz is the validated default;
higher rates are no longer poll-bound and can be re-evaluated here once the
stream is trusted (watch `comp age` and `overruns`).

✅ `loop` holds the requested rate, `overruns=0`, `comp age` ≪ `stale_timeout_ms`
→ gravity next.
❌ overruns climb, or `comp age` creeps toward `stale_timeout_ms` (stream not
keeping up) → drop the rate / investigate before energizing.

## Step 4 — Gravity (ramp 0.3 → 1.0)

```sh
peppy node stop arm_0
peppy node run openarm01_arm:v1 -i arm_0 \
  arm_id=0 can_interface=$CAN control_rate_hz=100 \
  gravity_scale=0.3 coriolis_scale=0 friction_scale=0 min_motion_time_s=5.0 \
  --bind model@srs_left_0 --idle-timeout 86400 --max-timeout 86400
```

Judge: each joint **pushes up against gravity** (arm feels lighter / holds
better), `comp max_drift` small, **no joint accelerating away**.

✅ partial hold, nothing runs → re-run with `gravity_scale=1.0` (keep
`min_motion_time_s=5.0`; expect it to roughly float, a little residual wrist sag
from the unmodeled gripper body is normal). ❌ any joint drives *with* gravity or
heads to a limit → **e-stop**, sign error, stop here.

## Step 5 — Trajectory (gravity on) — Terminal C

Leave the arm running at `gravity_scale=1.0`. Then:

```sh
peppy node run openarm01_arm_test:v1 -i arm_test_0 \
  motion_enabled=true \
  --bind arm@arm_0,ik@srs_left_0 --idle-timeout 86400 --max-timeout 86400
```

The tester resolves the IK target once, then **oscillates** between that
workspace pose and near-home, dwelling `dwell_s` (default 2 s) at each end,
**until you `peppy node stop arm_test_0`**. It logs each move + feedback; the arm
logs `track ... max_err`. Each move takes `min_motion_time_s` (5 s, from the arm
run) — deliberately slow.

✅ smooth oscillation reach↔home, `track max_err` small (a few °), each move
reports success. Default gains are the teleop config: 240/3 on the
shoulder/elbow joints (1-4) and lower wrist gains (`kp5/6/7 = 24/31/25`,
`kd = 0.2`). To soften the moves, lower the shoulder/elbow gains (e.g. `kp1=70
kp2=70 kp3=70 kp4=70`) and leave the wrist defaults.
❌ overshoot/oscillation → lower kp or investigate.

## Step 6 — Friction (0.3)

```sh
peppy node stop arm_0
peppy node run openarm01_arm:v1 -i arm_0 \
  arm_id=0 can_interface=$CAN control_rate_hz=100 \
  gravity_scale=1.0 coriolis_scale=0 friction_scale=0.3 min_motion_time_s=5.0 \
  --bind model@srs_left_0 --idle-timeout 86400 --max-timeout 86400
```

✅ at rest, `comp qdot`≈0 stays ≈0 (no buzz). ❌ a joint limit-cycles/buzzes →
friction too high, drop it.

## Step 7 — Coriolis (0.1)

Re-run as step 6 but with `coriolis_scale=0.1`. This is the full openarm teleop
weighting (gravity 1.0, Coriolis 0.1, friction 0.3). Judge: no adverse change.

## Step 8 — Second arm (right)

Right is a mirror chain — repeat step 4's sign check, don't assume.

```sh
peppy node run srs_model:v1 -i srs_right_0 \
  urdf_path=$OPENARM/description/openarm_v10.urdf \
  base_link=openarm_right_link0 \
  --bind-deferred arm@arm_1 --idle-timeout 86400 --max-timeout 86400

peppy node run openarm01_arm:v1 -i arm_1 \
  arm_id=1 can_interface=<right_can> control_rate_hz=100 \
  gravity_scale=0.3 coriolis_scale=0 friction_scale=0 min_motion_time_s=5.0 \
  --bind model@srs_right_0 --idle-timeout 86400 --max-timeout 86400
```

---

## Note on the launcher

`peppy_launcher.json5` encodes the same wiring declaratively, but `peppy stack
launch` re-adds and **recompiles every node on each launch** — so it's only
worth using for a one-shot bring-up, not for the param ramp. Iterate with the
per-node `peppy node run` commands above (no rebuild).

## Safety / abort

- E-stop on any runaway, growing buzz, or a joint heading to a limit.
- Float steps (2, 4, 6, 7) have `kp=kd=0` — nothing holds a bad gravity term but
  the e-stop.
- Change **one** thing per step; never comp + gains together.

## After validation (production cleanup)

- Set the production scales (gravity 1.0, Coriolis 0.1, friction 0.3) explicitly,
  or change the node defaults back from the fail-safe 0.
- Remove the TEMPORARY instrumentation (`LoopStats`, `log_compensation`,
  `log_tracking`, `log_period_ms`, the "compensation acquired/stale/recovered"
  lines); keep the startup config echo.
- Drop the DNC tester commit and this runbook.
