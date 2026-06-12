mod actions;
mod startup;

use std::sync::{Arc, Mutex, PoisonError};

use peppygen::{NodeBuilder, Parameters, Result};
use tokio::task::JoinSet;
use tracing::{error, info};

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    NodeBuilder::new().run(|_params: Parameters, node_runner| async move {
        let token = node_runner.cancellation_token().clone();

        // Registry of in-flight per-goal relay tasks. When the token fires,
        // each relay propagates cancel_goal to its downstream arm/gripper goal
        // (which is actively driving hardware) and answers the upstream
        // caller; this hook awaits them all so that cleanup finishes before
        // run() returns, instead of the detached tasks being dropped mid-await
        // when the runtime tears down.
        let in_flight: actions::InFlightGoals = Arc::new(Mutex::new(JoinSet::new()));
        let goals_for_hook = Arc::clone(&in_flight);
        node_runner.on_shutdown(async move {
            // Re-take in a loop: a goal accepted just as the token fired may
            // be registered after the first take.
            loop {
                let mut goals = std::mem::take(
                    &mut *goals_for_hook.lock().unwrap_or_else(PoisonError::into_inner),
                );
                if goals.is_empty() {
                    break;
                }
                while goals.join_next().await.is_some() {}
            }
        });

        // Spawn the startup-and-handlers chain so this setup closure returns
        // immediately and NodeBuilder can register node_health before the
        // daemon's health probe fires (awaiting wait_until_ready here would
        // block for the Isaac USD-load window ~30-60s and trip the probe).
        // JoinSet supervises both handlers so an early exit surfaces.
        tokio::spawn(async move {
            // Gate on the world being ready before exposing any action.
            startup::wait_until_ready(&node_runner, &token).await;

            let mut set = JoinSet::new();
            set.spawn(actions::move_arm_joints::run(
                node_runner.clone(),
                token.clone(),
                Arc::clone(&in_flight),
            ));
            set.spawn(actions::move_gripper::run(
                node_runner.clone(),
                token.clone(),
                in_flight,
            ));
            while let Some(joined) = set.join_next().await {
                match joined {
                    Ok(Ok(())) => info!("backbone action handler exited cleanly"),
                    Ok(Err(e)) => error!(error = %e, "backbone action handler returned Err"),
                    Err(e) if e.is_panic() => {
                        error!(error = %e, "backbone action handler panicked")
                    }
                    Err(e) => error!(error = %e, "backbone action handler join failed"),
                }
            }
        });

        Ok(())
    })
}
