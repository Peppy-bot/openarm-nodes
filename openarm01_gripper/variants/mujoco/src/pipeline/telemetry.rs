// Telemetry pipelines.
//
// TODO: wire via sim_bridge_core::SimBridge. For this gripper, the eight
// typed peppygen topics on the contract come from these raw peppylib
// subscriptions published by robot_initializer:mujoco:
//
//   Subscribe raw                 → Emit typed peppygen
//   ─────────────────────────────────────────────────────────────────────
//   gripper_state_<side>          → gripper_state_<side>
//   ee_pose_<side>                → ee_pose_<side>
//   contact_forces  (filter g1/2) → contact_forces_<side>_finger1
//                                   contact_forces_<side>_finger2
//
// SimBridge handles subscribe + reconnect/backoff + emit_fn dispatch. The
// emit_fn for each pipeline also updates an Arc<Mutex<...>> shared state
// that the move_gripper action handler reads for feedback. Per-side topic
// names are derived from `gripper_id.side_word()`.

use std::sync::Arc;

use peppygen::NodeRunner;
use tracing::info;

use crate::config::GripperId;

pub async fn run(_runner: Arc<NodeRunner>, gripper_id: GripperId) {
    info!(
        "telemetry: scaffold only — pipelines pending sim_bridge_core wiring (gripper_id={})",
        gripper_id.0,
    );
    // Park forever; real pipelines land in the next commit.
    std::future::pending::<()>().await;
}
