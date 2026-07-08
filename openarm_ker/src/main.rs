//! openarm_ker: operator entry point driven by the OpenArm KER (Kinematic
//! Equivalent Replica), enactic's motorless bimanual leader arm. A dedicated
//! reader thread speaks the M5Stack's framed protocol (USB vendor mode or
//! serial CDC), maps encoder channels to calibrated joint radians and trigger
//! openings, and the publish tasks stream them like the joint commander does:
//! arms on `arm_joint_commands` (the hub owns governing and safety), grippers
//! on their pairing slots. The thumb button is the engage deadman; while
//! disengaged, stale, or disconnected the node emits nothing and every
//! consumer's stream timeout holds the robot.

mod mapping;
mod protocol;
mod publish;
mod reader;
mod transport;

use std::time::Duration;

use mapping::{Calibration, CalibrationParams};
use openarm_description::HardwareVersion;
use peppygen::{NodeBuilder, Parameters, Result};
use peppylib::datastore::{self, Encoding};
use reader::{EngageMode, ReaderConfig};
use tokio::sync::watch;
use tracing::{info, warn};
use transport::TransportConfig;

const DATASTORE_TIMEOUT: Duration = Duration::from_secs(3);
const LOCK_REMOVE_TIMEOUT: Duration = Duration::from_secs(1);
/// One node instance per KER device: a second reader would fight for the USB
/// claim (or interleave on the serial port) in confusing ways, so fail fast.
const LOCK_KEY: &str = "openarm_ker_instance_lock";

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let token = node_runner.cancellation_token().clone();

        // Parse every parameter up front (parse, don't validate): a bad launch
        // dies here with the reason, before touching the device or the stack.
        let version: HardwareVersion = params
            .hardware_version
            .parse()
            .unwrap_or_else(|e| panic!("{e}"));
        // Rate feeds `Duration::from_micros(1_000_000 / rate)`, so a rate above
        // 1 MHz would round to a 0 µs period; no real deployment approaches that,
        // so just guard against zero.
        assert!(params.command_rate_hz > 0, "command_rate_hz must be > 0");
        assert!(
            params.stale_timeout_s.is_finite() && params.stale_timeout_s > 0.0,
            "stale_timeout_s must be a positive finite number"
        );
        let transport =
            TransportConfig::parse(&params.transport, &params.serial_port, params.serial_baud)
                .unwrap_or_else(|e| panic!("{e}"));
        let engage_mode: EngageMode = params.engage_mode.parse().unwrap_or_else(|e| panic!("{e}"));
        let calibration = Calibration::parse(
            version,
            &CalibrationParams {
                left_channels: &params.left_channels,
                left_signs: &params.left_signs,
                left_offsets_deg: &params.left_offsets_deg,
                right_channels: &params.right_channels,
                right_signs: &params.right_signs,
                right_offsets_deg: &params.right_offsets_deg,
                left_trigger_channel: params.left_trigger_channel,
                left_trigger_closed_deg: params.left_trigger_closed_deg,
                left_trigger_open_deg: params.left_trigger_open_deg,
                right_trigger_channel: params.right_trigger_channel,
                right_trigger_closed_deg: params.right_trigger_closed_deg,
                right_trigger_open_deg: params.right_trigger_open_deg,
            },
        )
        .unwrap_or_else(|e| panic!("calibration: {e}"));

        info!(
            "config: {version} follower, transport {transport:?}, {} Hz, engage {engage_mode:?}, \
             {} channels required",
            params.command_rate_hz,
            calibration.required_channels()
        );

        // Instance lock: crash if another instance is running. Held in the
        // core-node datastore (released from the on_shutdown hook below), so a
        // lock leaked by a hard crash clears with the stack instead of
        // lingering like a /tmp file. get-then-store is not atomic; two
        // simultaneous starts can race (single-writer in practice).
        if let Some(held) = datastore::get(&node_runner, LOCK_KEY, DATASTORE_TIMEOUT).await? {
            panic!("instance lock {LOCK_KEY} held by {}", held.last_modified_by);
        }
        datastore::store(
            &node_runner,
            LOCK_KEY,
            b"locked".to_vec(),
            Encoding::TEXT_PLAIN,
            DATASTORE_TIMEOUT,
        )
        .await?;
        {
            let runner = node_runner.clone();
            node_runner.on_shutdown(async move {
                if let Err(e) = datastore::remove(&runner, LOCK_KEY, LOCK_REMOVE_TIMEOUT).await {
                    warn!("failed to remove lock {LOCK_KEY}: {e}");
                }
            });
        }

        // The reader thread owns the device and keeps the newest calibrated
        // sample on the watch channel; the publish tasks stream it. Returning
        // promptly matters: peppylib registers node_health only after this
        // closure returns, so the device connect must not be awaited here.
        let (sample_tx, sample_rx) = watch::channel(None);
        reader::spawn(
            ReaderConfig {
                transport,
                calibration,
                engage_mode,
                log_raw: params.log_raw,
            },
            sample_tx,
            token.clone(),
        );
        tokio::spawn(publish::run(
            node_runner.clone(),
            sample_rx,
            params.command_rate_hz,
            Duration::from_secs_f64(params.stale_timeout_s),
            token,
        ));
        Ok(())
    })
}
