mod actions;
mod config;
mod drivers;
mod error;
mod pipeline;
mod services;

use std::path::PathBuf;
use std::sync::Arc;

use peppygen::{NodeBuilder, Parameters, Result};
use tokio::signal::unix::{SignalKind, signal};
use tracing::{info, warn};

use crate::config::{GripperId, ResolvedSide};
use crate::drivers::mjdata_bus::MjDataBus;

const BUS_DIR_ENV: &str = "PEPPY_MJDATA_BUS_DIR";

fn main() -> Result<()> {
    tracing_subscriber::fmt().with_max_level(tracing::Level::INFO).init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let gripper_id = GripperId(params.gripper_id);
        info!(
            "starting openarm01_gripper:mujoco instance={} gripper_id={}",
            gripper_id.instance_id(), gripper_id.0
        );

        let bus_dir = std::env::var(BUS_DIR_ENV)
            .or_else(|_| std::env::var("XDG_RUNTIME_DIR").map(|d| format!("{d}/peppy/sim")))
            .expect("set PEPPY_MJDATA_BUS_DIR or XDG_RUNTIME_DIR");
        let bus_path: PathBuf = bus_dir.into();

        // The bus is published by robot_initializer's mujoco variant; if we're
        // started before robot_initializer is ready, retry a few times before
        // giving up. Bounded — surfacing failure beats hanging forever.
        let bus = open_bus_with_retry(&bus_path).await.expect("open mjdata bus");
        let bus = Arc::new(bus);
        info!(
            "bus open: nq={} nv={} nu={} nbody={}",
            bus.meta.dimensions.nq, bus.meta.dimensions.nv,
            bus.meta.dimensions.nu, bus.meta.dimensions.nbody
        );

        let side = ResolvedSide::resolve(&bus, gripper_id).expect("resolve gripper side");
        info!(
            "resolved side: qpos_addrs={:?} ctrl_ids={:?} ee_body={} finger1_geoms={:?} finger2_geoms={:?}",
            side.finger_qpos_addrs, side.finger_ctrl_ids,
            side.ee_body_id, side.finger1_geom_ids, side.finger2_geom_ids
        );
        let side = Arc::new(side);

        // Shutdown task: zero this gripper's ctrl[] on SIGINT/SIGTERM so the sim
        // doesn't keep driving the actuators to the last commanded position after
        // the container goes down. Mirrors the default variant's signal handler.
        {
            let bus = bus.clone();
            let side = side.clone();
            tokio::spawn(async move {
                let mut sigint = signal(SignalKind::interrupt()).expect("sigint");
                let mut sigterm = signal(SignalKind::terminate()).expect("sigterm");
                tokio::select! {
                    _ = sigint.recv() => {},
                    _ = sigterm.recv() => {},
                }
                info!("shutdown: zeroing ctrl for this gripper");
                let zeros: Vec<(usize, f64)> = side
                    .finger_ctrl_ids
                    .iter()
                    .map(|&id| (id, 0.0))
                    .collect();
                bus.write_ctrl(&zeros);
                // peppylib runtime has no clean shutdown path; ctrl is zeroed above
                // so leaving the bus mmap to OS cleanup is safe.
                std::process::exit(0);
            });
        }

        // get_gripper_id service.
        tokio::spawn(services::get_gripper_id::run(node_runner.clone(), gripper_id));

        // move_gripper action.
        tokio::spawn(actions::move_gripper::run(
            node_runner.clone(), bus.clone(), side.clone(),
        ));

        // telemetry (8 topics at 50 Hz).
        tokio::spawn(pipeline::telemetry::run(
            node_runner.clone(), bus.clone(), side.clone(), gripper_id,
        ));

        Ok(())
    })
}

async fn open_bus_with_retry(bus_dir: &std::path::Path) -> Option<MjDataBus> {
    const MAX_ATTEMPTS: u32 = 30;
    for attempt in 1..=MAX_ATTEMPTS {
        match MjDataBus::open(bus_dir) {
            Ok(b) => return Some(b),
            Err(e) => {
                warn!("bus open attempt {attempt}/{MAX_ATTEMPTS} failed: {e}");
                tokio::time::sleep(std::time::Duration::from_secs(1)).await;
            }
        }
    }
    None
}
