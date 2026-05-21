// move_gripper action handler.
//
// TODO: full implementation pending sim_bridge_core shared-state pattern
// (action handler needs read access to the latest gripper_state subscribed
// from robot_initializer's raw peppylib, plus write access to publish raw
// set_ctrl_gripper_<side> back). Outline:
//
//   1. Expose ActionHandle::expose() on move_gripper.
//   2. On each accepted goal:
//        - Compute per-finger ctrl payload: {actuator_values: {
//            "<side>_finger1_ctrl": target,
//            "<side>_finger2_ctrl": target,
//          }}
//        - Publish raw peppylib on `set_ctrl_gripper_<side>` topic.
//        - Read latest gripper_state_<side> from shared state (populated by
//          telemetry pipeline) for feedback.
//        - Stream feedback at requested rate; detect convergence
//          (worst-finger tolerance < POSITION_TOLERANCE_M) or stall
//          (sum-of-motion across 500ms window < epsilon).
//        - On exit: result with success/message/final_positions/action_time.
//
// Carry-over from the bus-era implementation: convergence + stall logic
// stays identical; only the data source (bus.snapshot / bus.write_ctrl)
// is replaced with peppylib raw subscribe + publish.

use std::sync::Arc;

use peppygen::NodeRunner;
use tracing::info;

use crate::config::GripperId;
use crate::state::SharedState;

pub async fn run(
    _runner: Arc<NodeRunner>,
    gripper_id: GripperId,
    _state: Arc<SharedState>,
) {
    info!(
        "move_gripper: scaffold only — handler pending sim_bridge_core wiring (gripper_id={})",
        gripper_id.0,
    );
    // Park forever; real handler lands in the next commit.
    std::future::pending::<()>().await;
}
