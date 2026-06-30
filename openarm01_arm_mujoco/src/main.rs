//! MuJoCo sim follower: republish the hub's governed setpoints for this arm onto
//! the sim's arm_sim_passthrough topic. All motion, trajectory, and collision
//! logic lives in openarm01_backbone; this node only relabels the governed
//! stream for the engine. A held subscription receives every setpoint in order
//! with no re-subscribe gap; a separate task publishes the latest, so neither
//! arm is starved (the same shape the real arm uses).

use peppygen::consumed_topics::hub_arm_governed_setpoints;
use peppygen::emitted_topics::openarm01_arm_sim_passthrough::v1::arm_sim_passthrough;
use peppygen::{NodeBuilder, Parameters, Result};
use tokio::sync::watch;
use tracing::{error, info, warn};

/// Latest desired (positions, velocities) for this arm.
type Setpoint = ([f64; 7], [f64; 7]);

/// Wire arm_id values (matching the hub).
const ARM_ID_LEFT: u8 = 0;
const ARM_ID_RIGHT: u8 = 1;

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let arm_id = params.arm_id;
        assert!(arm_id == ARM_ID_LEFT || arm_id == ARM_ID_RIGHT, "arm_id must be 0 (left) or 1 (right), got {arm_id}");
        info!("starting openarm01_arm_mujoco follower arm_id={arm_id}");

        let (latest_tx, latest_rx) = watch::channel::<Option<Setpoint>>(None);

        // Receive task: one held subscription, looped. Holding the subscription
        // means no re-subscribe gap between messages, so a setpoint for this arm is
        // never dropped while the other arm's message is in flight.
        let rx_runner = node_runner.clone();
        tokio::spawn(async move {
            let mut sub = match hub_arm_governed_setpoints::subscribe(&rx_runner).await {
                Ok(s) => s,
                Err(e) => return error!("governed_setpoints subscribe: {e}"),
            };
            loop {
                let msg = match sub.next().await {
                    Ok(Some((_, msg))) => msg,
                    Ok(None) => return, // subscription closed: node shutting down
                    Err(e) => {
                        error!("governed_setpoints receive: {e}");
                        continue;
                    }
                };
                if msg.arm_id != arm_id {
                    continue;
                }
                // Clear the latest on any non-finite governed setpoint, matching the
                // real arm, so a bad value never reaches the sim engine and the sim
                // holds its last commanded pose.
                let finite = msg.positions.iter().chain(msg.velocities.iter()).all(|v| v.is_finite());
                if !finite {
                    warn!("governed_setpoints: clearing target on non-finite values");
                    let _ = latest_tx.send(None);
                    continue;
                }
                let _ = latest_tx.send(Some((msg.positions, msg.velocities)));
            }
        });

        // Publish task: relabel each new setpoint onto arm_sim_passthrough. No
        // shutdown handler: never publish a zero setpoint on exit, which would
        // command the arm into a self-collision pose.
        tokio::spawn(async move {
            let publisher = match arm_sim_passthrough::declare_publisher(&node_runner).await {
                Ok(p) => p,
                Err(e) => return error!("declare arm_sim_passthrough publisher: {e}"),
            };
            let mut latest_rx = latest_rx;
            let mut failing = false;
            loop {
                if latest_rx.changed().await.is_err() {
                    return; // receive task gone: node shutting down
                }
                let Some((q_des, dq_des)) = *latest_rx.borrow() else { continue };
                let result = async {
                    let payload = arm_sim_passthrough::build_message(arm_id, q_des, dq_des)
                        .map_err(|e| e.to_string())?;
                    publisher.publish(payload).await.map_err(|e| e.to_string())
                }
                .await;
                match result {
                    Ok(()) => failing = false,
                    Err(e) if !failing => {
                        failing = true;
                        warn!("arm_sim_passthrough publish failing, suppressing repeats: {e}");
                    }
                    Err(_) => {}
                }
            }
        });

        Ok(())
    })
}
