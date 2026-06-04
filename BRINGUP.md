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
export OPENARM=~/path/to/openarm01_nodes   # this repo
export NODES_HUB=~/path/to/nodes_hub       # holds srs_model
export CAN=left_follower                    # this arm's CAN interface
```

- Left arm: `arm_id=0`, `base_link=openarm_left_link0`.
- Stop a node: `peppy node stop <instance_id>` (or Ctrl-C its terminal).
- The arm prints diagnostics every `log_period_ms` (default 1000). Lower it
  (e.g. `log_period_ms=200`) to watch a fast step closely — runtime, no rebuild.
- Run each long-lived node in its own terminal.

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
  --idle-timeout 86400 --max-timeout 86400
```

Confirm it logs the resolved chain/wrist without error. Leave it running.

## What to watch (arm logs, every `log_period_ms`)

- `config: arm_id=.. (left) rate=..Hz scales(..) ...` — confirms this run's params.
- `loop: N Hz (n=..), work avg/max, overruns (budget ..ms)`
- `poll latency: avg/min/max (n=..)`
- `comp ok=.. scales(g.. c.. f..) max_drift=..rad / q / qdot / gravity / coriolis / friction / tau`
- `track t=.. max_err=..rad / q_des / q / err` (only during a move)

---

## Step 2 — Observe (limp, no feedforward) — Terminal B

```sh
peppy node run openarm01_arm:v1 -i arm_0 \
  arm_id=0 can_interface=$CAN control_rate_hz=100 \
  gravity_scale=0 coriolis_scale=0 friction_scale=0 \
  --bind model@srs_left_0 --idle-timeout 86400 --max-timeout 86400
```

Arm is **limp** (`tau≈0`). Judge:

- `comp` gravity magnitudes plausible (a few Nm on loaded joints, ~0 at the
  wrist), signs consistent with the pose; `tau≈0`.
- `poll latency` max small (this sets the step-3 timeouts).
- `loop` holds ~100 Hz, `overruns=0`.

✅ comp sane + low latency → next. ❌ gravity wild/NaN or huge latency → stop.

## Step 3 — 500 Hz (still limp)

Size the timeouts from step-2 latency: `compensation_timeout_ms` must exceed the
observed poll max but stay under the 2 ms cycle; shrink the CAN read too.

```sh
peppy node stop arm_0
peppy node run openarm01_arm:v1 -i arm_0 \
  arm_id=0 can_interface=$CAN control_rate_hz=500 \
  recv_timeout_us=400 compensation_timeout_ms=1 \
  gravity_scale=0 coriolis_scale=0 friction_scale=0 \
  --bind model@srs_left_0 --idle-timeout 86400 --max-timeout 86400
```

✅ `loop` ~500 Hz, `overruns≈0`, `poll latency` max ≪ 2 ms → next.
❌ overruns climb or latency tail blows the budget → drop the rate; that's the
signal the synchronous poll can't sustain 500 Hz (revisit the push/topic model).

> Recommended: do the **first gravity-on (step 4) at 100 Hz**, confirm direction,
> then come back to 500 Hz. A sign error is far easier to catch slow.

## Step 4 — Gravity (ramp 0.3 → 1.0)

```sh
peppy node stop arm_0
peppy node run openarm01_arm:v1 -i arm_0 \
  arm_id=0 can_interface=$CAN control_rate_hz=100 \
  gravity_scale=0.3 coriolis_scale=0 friction_scale=0 \
  --bind model@srs_left_0 --idle-timeout 86400 --max-timeout 86400
```

Judge: each joint **pushes up against gravity** (arm feels lighter / holds
better), `comp max_drift` small, **no joint accelerating away**.

✅ partial hold, nothing runs → re-run with `gravity_scale=1.0` (expect it to
roughly float; a little residual wrist sag from the unmodeled gripper body is
normal). ❌ any joint drives *with* gravity or heads to a limit → **e-stop**,
sign error, stop here.

## Step 5 — Trajectory (gravity on) — Terminal C

Leave the arm running at `gravity_scale=1.0`. Then:

```sh
peppy node run openarm01_arm_test:v1 -i arm_test_0 \
  motion_enabled=true \
  --bind arm@arm_0,ik@srs_left_0 --max-timeout 600
```

The tester logs arm_id, start joints, IK target + solution, goal accepted,
feedback, and result. The arm logs `track ... max_err`.

✅ smooth move to the natural front pose, `track max_err` small (a few °),
result success. Gains default to teleop's 240/3 (validated on this robot). To
soften the first move, add `kp1=70 ... kp7=70` to the arm run.
❌ overshoot/oscillation → lower kp or investigate.

## Step 6 — Friction (0.3)

```sh
peppy node stop arm_0
peppy node run openarm01_arm:v1 -i arm_0 \
  arm_id=0 can_interface=$CAN control_rate_hz=500 \
  recv_timeout_us=400 compensation_timeout_ms=1 \
  gravity_scale=1.0 coriolis_scale=0 friction_scale=0.3 \
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
  base_link=openarm_right_link0 --idle-timeout 86400 --max-timeout 86400

peppy node run openarm01_arm:v1 -i arm_1 \
  arm_id=1 can_interface=<right_can> control_rate_hz=100 \
  gravity_scale=0.3 coriolis_scale=0 friction_scale=0 \
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
- Remove the TEMPORARY instrumentation (`PollStats`, `LoopStats`,
  `log_compensation`, `log_tracking`, `log_period_ms`, the "compensation
  acquired" line); keep the startup config echo.
- Drop the DNC tester commit and this runbook.
