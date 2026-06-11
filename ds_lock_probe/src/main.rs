// Probe for the datastore instance-lock shutdown paths: stores a key on
// startup and removes it from a shutdown hook registered with
// `node_runner.on_shutdown`. The peppylib runtime owns every stop path
// (SIGINT, SIGTERM, `peppy node stop`, daemon teardown/loss) and awaits the
// hook — bounded by `lifecycle.shutdown_grace_secs` — before the process
// exits, so the lock is released on all of them.
//
// Historical note: this node started as the minimal repro for the lock LEAK on
// `peppy node stop` — the old pattern (a spawned task selecting on
// SIGINT/SIGTERM/the cancellation token) was never polled on the in-band stop
// because `run()` returned, and dropped the tokio runtime, as soon as the
// token was cancelled. Cleanup now belongs in `on_shutdown`, which `run()`
// awaits on every path.
//
// Log markers: LOCK_ACQUIRED / LOCK_HELD / SHUTDOWN_HOOK / REMOVE_PRE /
// REMOVE_POST. A run with SHUTDOWN_HOOK but no REMOVE_POST means the remove
// was cut off by the grace window; no SHUTDOWN_HOOK at all means the hook
// never ran (the original bug).
//
// mode=clean is a janitor: removes the key and exits, so the probe can be
// re-run after a leak without restarting the core-node stack (there is no
// datastore CLI).
use peppygen::{NodeBuilder, Parameters, Result};
use peppylib::datastore::{self, Encoding};

use std::io::Write;
use std::time::Duration;

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
            // Programmatic cancel: the janitor requests its own shutdown and
            // returns; run() sees the cancelled token and exits cleanly.
            node_runner.cancellation_token().cancel();
            return Ok(());
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

        // Release the lock on shutdown. The runtime awaits this hook (the
        // messenger is still connected) on every stop path before exiting.
        let runner = node_runner.clone();
        node_runner.on_shutdown(async move {
            say(&format!("SHUTDOWN_HOOK key={lock_key}"));
            say("REMOVE_PRE");
            match datastore::remove(&runner, lock_key.as_str(), DATASTORE_TIMEOUT).await {
                Ok(removed) => say(&format!("REMOVE_POST ok removed={removed}")),
                Err(e) => say(&format!("REMOVE_POST err={e}")),
            }
        });

        Ok(())
    })
}
