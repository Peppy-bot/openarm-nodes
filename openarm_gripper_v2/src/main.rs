use peppygen::Result;

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    peppygen::NodeBuilder::new().run(openarm_gripper_v2::setup)?;

    // The runtime has returned, so the shutdown hooks (motor disable, lock
    // release) have already run; exiting non-zero here makes the daemon
    // record a hard CAN fault as failed instead of finished.
    if openarm_gripper_v2::hard_fault_latched() {
        return Err(peppygen::Error::Io(std::io::Error::other(
            "hard fault stopped this node; the log names the failing component",
        )));
    }
    Ok(())
}
