//! Isaac sim follower: republish the paired backbone's governed setpoints onto the
//! sim's arm_sim_passthrough topic, and relay the engine's measured state back
//! to the backbone on the pairing, re-emitting it on the generic joint_states
//! contract (and the governed setpoint on joint_commands) so a recorder or
//! monitor binds this follower exactly like the real arm. All motion, trajectory,
//! and collision logic lives in openarm_backbone; this node only relabels the
//! governed stream for the engine and the engine's state for its consumers. A
//! held subscription receives every setpoint in order with no re-subscribe gap; a
//! separate task publishes the latest, so neither arm is starved (the same shape
//! the real arm uses).

use peppygen::consumed_topics::engine_states_arm_states;
use peppygen::emitted_topics::joint_commands::v1::joint_commands;
use peppygen::emitted_topics::joint_states::v1::joint_states;
use peppygen::emitted_topics::openarm_arm_sim_passthrough::v1::arm_sim_passthrough;
use peppygen::pairings::backbone;
use peppygen::{NodeBuilder, Parameters, Result};
use tokio::sync::watch;
use tracing::{error, info, warn};

/// Latest desired (positions, velocities) for this arm.
type Setpoint = ([f64; 7], [f64; 7]);

/// Wire arm_id values (matching the backbone).
const ARM_ID_LEFT: u8 = 0;
const ARM_ID_RIGHT: u8 = 1;

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let arm_id = params.arm_id;
        assert!(
            arm_id == ARM_ID_LEFT || arm_id == ARM_ID_RIGHT,
            "arm_id must be 0 (left) or 1 (right), got {arm_id}"
        );
        info!("starting openarm_arm_isaac follower arm_id={arm_id}");

        let (latest_tx, latest_rx) = watch::channel::<Option<Setpoint>>(None);
        // The relay task re-surfaces the same governed setpoint as joint_commands,
        // held-last, alongside the measured joint_states it publishes.
        let relay_latest = latest_rx.clone();
        // Supervise the follower tasks: if any ever exits, whether a clean Ok(None)
        // on shutdown or an unexpected error/panic, this relabel path is dead, so cancel
        // the node to restart it rather than leaving it healthy but inert.
        let token = node_runner.cancellation_token().clone();

        // Receive task: one held pairing subscription, looped. The slot delivers
        // only the paired backbone's setpoints, so there is no arm_id filter; holding
        // the subscription means no re-subscribe gap between messages.
        let rx_runner = node_runner.clone();
        let receive = tokio::spawn(async move {
            let mut sub = match backbone::arm_setpoints::subscribe(&rx_runner).await {
                Ok(s) => s,
                Err(e) => return error!("arm_setpoints subscribe: {e}"),
            };
            loop {
                let msg = match sub.next().await {
                    Ok(Some((_, msg))) => msg,
                    Ok(None) => return, // subscription closed: node shutting down
                    Err(e) => {
                        error!("arm_setpoints receive: {e}");
                        continue;
                    }
                };
                // Clear the latest on any non-finite governed setpoint, matching the
                // real arm, so a bad value never reaches the sim engine and the sim
                // holds its last commanded pose.
                let finite = msg
                    .positions
                    .iter()
                    .chain(msg.velocities.iter())
                    .all(|v| v.is_finite());
                if !finite {
                    warn!("arm_setpoints: clearing target on non-finite values");
                    let _ = latest_tx.send(None);
                    continue;
                }
                let _ = latest_tx.send(Some((msg.positions, msg.velocities)));
            }
        });

        // Publish task: relabel each new setpoint onto arm_sim_passthrough. No
        // shutdown handler: never publish a zero setpoint on exit, which would
        // command the arm into a self-collision pose.
        let pub_runner = node_runner.clone();
        let publish = tokio::spawn(async move {
            let publisher = match arm_sim_passthrough::declare_publisher(&pub_runner).await {
                Ok(p) => p,
                Err(e) => return error!("declare arm_sim_passthrough publisher: {e}"),
            };
            let mut latest_rx = latest_rx;
            let mut failing = false;
            loop {
                if latest_rx.changed().await.is_err() {
                    return; // receive task gone: node shutting down
                }
                let Some((q_des, dq_des)) = *latest_rx.borrow() else {
                    continue;
                };
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

        // State relay task: this arm's engine measurements (the sim engine's
        // broadcast arm_states, demuxed by arm_id) flow to the paired backbone on
        // the pairing's arm_states (the command loop's state input) and, for
        // observers such as a recorder, on the generic joint_states contract; the
        // governed setpoint the sim is tracking is re-surfaced on joint_commands.
        // A consumer binds this follower exactly like the real arm. Non-finite
        // samples are dropped so no consumer anchors on a bad measurement, and
        // each publish reports independently so one failing never suppresses the
        // others.
        let relay = tokio::spawn(async move {
            let mut sub = match engine_states_arm_states::subscribe(&node_runner).await {
                Ok(s) => s,
                Err(e) => return error!("engine arm_states subscribe: {e}"),
            };
            let observer_pub = match joint_states::declare_publisher(&node_runner).await {
                Ok(p) => p,
                Err(e) => return error!("declare joint_states publisher: {e}"),
            };
            let command_pub = match joint_commands::declare_publisher(&node_runner).await {
                Ok(p) => p,
                Err(e) => return error!("declare joint_commands publisher: {e}"),
            };
            let peer_pub = match backbone::arm_states::declare_publisher(&node_runner).await {
                Ok(p) => p,
                Err(e) => return error!("declare paired arm_states publisher: {e}"),
            };
            let mut observer_failing = false;
            let mut command_failing = false;
            let mut peer_failing = false;
            loop {
                let msg = match sub.next().await {
                    Ok(Some((_, msg))) => msg,
                    Ok(None) => return, // subscription closed: node shutting down
                    Err(e) => {
                        error!("engine arm_states receive: {e}");
                        continue;
                    }
                };
                let finite = msg
                    .positions
                    .iter()
                    .chain(msg.velocities.iter())
                    .all(|v| v.is_finite());
                if msg.arm_id != arm_id || !finite {
                    continue;
                }
                // Measured state on the generic observer contract (no arm_id;
                // consumers identify this arm by its producer binding).
                let observer = async {
                    let m = joint_states::build_message(
                        msg.positions.to_vec(),
                        msg.velocities.to_vec(),
                        Vec::new(),
                    )
                    .map_err(|e| e.to_string())?;
                    observer_pub.publish(m).await.map_err(|e| e.to_string())
                }
                .await;
                match observer {
                    Ok(()) => observer_failing = false,
                    Err(e) if !observer_failing => {
                        observer_failing = true;
                        warn!("joint_states publish failing, suppressing repeats: {e}");
                    }
                    Err(_) => {}
                }
                // The governed setpoint commanded to this arm, held-last, as the
                // action a recorder captures aligned with the measured state.
                let latest_setpoint = *relay_latest.borrow();
                if let Some((q_des, _)) = latest_setpoint {
                    let command = async {
                        let m = joint_commands::build_message(q_des.to_vec())
                            .map_err(|e| e.to_string())?;
                        command_pub.publish(m).await.map_err(|e| e.to_string())
                    }
                    .await;
                    match command {
                        Ok(()) => command_failing = false,
                        Err(e) if !command_failing => {
                            command_failing = true;
                            warn!("joint_commands publish failing, suppressing repeats: {e}");
                        }
                        Err(_) => {}
                    }
                }
                // Measured state to the paired backbone (the command loop's input).
                let peer = async {
                    let m = backbone::arm_states::build_message(msg.positions, msg.velocities)
                        .map_err(|e| e.to_string())?;
                    peer_pub.publish(m).await.map_err(|e| e.to_string())
                }
                .await;
                match peer {
                    Ok(()) => peer_failing = false,
                    Err(e) if !peer_failing => {
                        peer_failing = true;
                        warn!("paired arm_states publish failing, suppressing repeats: {e}");
                    }
                    Err(_) => {}
                }
            }
        });

        // Cancel the node the moment any task stops.
        tokio::spawn(async move {
            tokio::select! {
                _ = receive => {}
                _ = publish => {}
                _ = relay => {}
            }
            token.cancel();
        });

        Ok(())
    })
}
