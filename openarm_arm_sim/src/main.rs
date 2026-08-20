//! Binary entry point: tracing to the environment's filter, then the relay's
//! real setup (`openarm_arm_sim::setup`) through the runtime builder.

use peppygen::{NodeBuilder, Result};

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    NodeBuilder::new().run(openarm_arm_sim::setup)
}
