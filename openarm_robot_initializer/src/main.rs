use std::sync::Arc;

use peppygen::{NodeBuilder, NodeRunner, Parameters, Result};

mod service;

fn main() -> Result<()> {
    tracing_subscriber::fmt().init();
    NodeBuilder::new().run(|_args: Parameters, runner: Arc<NodeRunner>| async move {
        let token = runner.cancellation_token().clone();
        tokio::spawn(async move {
            service::run(runner, token).await;
        });
        Ok(())
    })
}
