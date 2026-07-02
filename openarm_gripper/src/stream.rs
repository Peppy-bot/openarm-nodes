// Always-on gripper_states publisher: emits the measured opening at
// state_rate_hz regardless of mode. Reads the motor's already-cached state (no
// CAN traffic of its own), so it
// never contends with the move control loop for the bus; between moves the
// gripper holds position, so the last cached reading stays correct.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use openarm_can::GripperCan;
use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm_gripper_states::v1::gripper_states;
use peppylib::runtime::CancellationToken;
use tracing::{error, warn};

use crate::geometry;

pub async fn run(
    runner: Arc<NodeRunner>,
    gripper_id: u8,
    state_rate_hz: u32,
    gripper: Arc<Mutex<GripperCan>>,
    token: CancellationToken,
) {
    let publisher = match gripper_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare gripper_states publisher: {e}"),
    };
    let period = Duration::from_micros(1_000_000 / state_rate_hz as u64);
    let mut failing = false;
    loop {
        tokio::select! {
            _ = token.cancelled() => return,
            _ = tokio::time::sleep(period) => {}
        }
        let motor_rad = gripper
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get_state()
            .position;
        let opening = geometry::motor_rad_to_meters(motor_rad);
        // Skip a poisoned sample rather than publishing NaN/Inf to consumers,
        // matching the finiteness guards on the command paths.
        if !opening.is_finite() {
            warn!("gripper_states: skipping non-finite motor sample");
            continue;
        }
        // The v1.0 prismatic gripper does not sense grip force; report 0.
        let force = 0.0;
        let result = async {
            let msg = gripper_states::build_message(gripper_id, opening, force)
                .map_err(|e| e.to_string())?;
            publisher.publish(msg).await.map_err(|e| e.to_string())
        }
        .await;
        match result {
            Ok(()) => failing = false,
            Err(e) if !failing => {
                failing = true;
                warn!("gripper_states publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
    }
}
