// Minimal repro for the core-node datastore instance-lock leak on
// `peppy node stop`.
//
// Lock flow mirrors openarm01_arm/_gripper: get -> panic if held, store,
// remove from a spawned shutdown task selecting on SIGINT / SIGTERM / the
// runtime cancellation token. SIGINT and SIGTERM release the lock; on
// `peppy node stop` the task never runs and the lock leaks.
//
// Log markers: LOCK_ACQUIRED / LOCK_HELD / SHUTDOWN_TRIGGER <which> /
// REMOVE_PRE / REMOVE_POST. A run with SHUTDOWN_TRIGGER but no REMOVE_POST
// means the remove was cancelled mid-await; no SHUTDOWN_TRIGGER at all means
// the shutdown task was never polled.
//
// mode=clean is a janitor: removes the key and exits, so the repro can be
// re-run without restarting the core-node stack (a leaked key persists until
// then; there is no datastore CLI).
use peppygen::{NodeBuilder, Parameters, Result};
use peppylib::datastore::{self, Encoding};

use std::io::Write;
use std::time::Duration;
use tokio::signal::unix::{SignalKind, signal};

const DATASTORE_TIMEOUT: Duration = Duration::from_secs(3);

fn say(msg: &str) {
    println!("{msg}");
    let _ = std::io::stdout().flush();
}

fn main() -> Result<()> {
    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let lock_key = params.lock_key.clone();

        if params.mode == "clean" {
            let removed = datastore::remove(&node_runner, lock_key.as_str(), DATASTORE_TIMEOUT).await?;
            say(&format!("CLEANED key={lock_key} removed={removed}"));
            std::process::exit(0);
        }

        if let Some(held) = datastore::get(&node_runner, lock_key.as_str(), DATASTORE_TIMEOUT).await? {
            say(&format!("LOCK_HELD key={lock_key} by={}", held.last_modified_by));
            panic!("lock held");
        }
        datastore::store(
            &node_runner,
            lock_key.as_str(),
            b"locked".to_vec(),
            Encoding::TEXT_PLAIN,
            DATASTORE_TIMEOUT,
        )
        .await?;
        say(&format!("LOCK_ACQUIRED key={lock_key}"));

        let cancel = node_runner.cancellation_token().clone();
        tokio::spawn(async move {
            let mut sigint = signal(SignalKind::interrupt()).expect("sigint");
            let mut sigterm = signal(SignalKind::terminate()).expect("sigterm");
            let trigger = tokio::select! {
                _ = sigint.recv() => "sigint",
                _ = sigterm.recv() => "sigterm",
                _ = cancel.cancelled() => "cancel",
            };
            say(&format!("SHUTDOWN_TRIGGER {trigger} key={lock_key}"));
            say("REMOVE_PRE");
            match datastore::remove(&node_runner, lock_key.as_str(), DATASTORE_TIMEOUT).await {
                Ok(removed) => say(&format!("REMOVE_POST ok removed={removed}")),
                Err(e) => say(&format!("REMOVE_POST err={e}")),
            }
            std::process::exit(0);
        });

        Ok(())
    })
}
