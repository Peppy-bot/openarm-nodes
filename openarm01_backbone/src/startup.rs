use std::collections::HashMap;
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::consumed_services::{arm_get_arm_id, gripper_get_gripper_id, robot_init_is_ready};
use peppylib::core_node::stack::stack_list;
use peppylib::runtime::CancellationToken;
use tracing::{info, warn};

const IS_READY_POLL_INTERVAL: Duration = Duration::from_millis(500);
const SERVICE_TIMEOUT: Duration = Duration::from_secs(5);
const DISCOVERY_RETRY_INTERVAL: Duration = Duration::from_millis(500);
const DISCOVERY_TIMEOUT: Duration = Duration::from_secs(30);
const ARM_NODE_NAME: &str = "openarm01_arm";
const GRIPPER_NODE_NAME: &str = "openarm01_gripper";

// arm_id / gripper_id -> the peppy instance_id that serves it. Built once at startup from
// get_arm_id / get_gripper_id, then read on every forwarded goal to target the right instance.
// TODO: remove after interface_v1 — launcher bindings replace runtime instance discovery.
pub struct Routing {
    arms: HashMap<u8, String>,
    grippers: HashMap<u8, String>,
}

impl Routing {
    pub fn arm_instance(&self, arm_id: u8) -> Option<&str> {
        self.arms.get(&arm_id).map(String::as_str)
    }

    pub fn gripper_instance(&self, gripper_id: u8) -> Option<&str> {
        self.grippers.get(&gripper_id).map(String::as_str)
    }
}

// Block until robot_initializer reports ready, then map each running arm/gripper instance to its
// side. Best-effort: returns whatever is discoverable (an empty side just means its goals reject).
pub async fn run(runner: &NodeRunner, token: &CancellationToken) -> Routing {
    wait_until_ready(runner, token).await;
    let arms = discover_arms(runner, token).await;
    let grippers = discover_grippers(runner, token).await;
    info!(
        arms = arms.len(),
        grippers = grippers.len(),
        "backbone startup complete"
    );
    Routing { arms, grippers }
}

async fn wait_until_ready(runner: &NodeRunner, token: &CancellationToken) {
    loop {
        match robot_init_is_ready::poll(runner, SERVICE_TIMEOUT, None, None).await {
            Ok(resp) if resp.data.ready => {
                info!("robot_initializer reported ready");
                return;
            }
            Ok(_) => {}
            Err(e) => warn!(error = %e, "is_ready poll failed; retrying"),
        }
        tokio::select! {
            _ = token.cancelled() => return,
            _ = tokio::time::sleep(IS_READY_POLL_INTERVAL) => {}
        }
    }
}

async fn discover_arms(runner: &NodeRunner, token: &CancellationToken) -> HashMap<u8, String> {
    let mut map = HashMap::new();
    for instance_id in running_instances(runner, ARM_NODE_NAME, token).await {
        // TODO: remove after interface_v1 — launcher bindings replace get_arm_id discovery.
        match arm_get_arm_id::poll(runner, SERVICE_TIMEOUT, None, Some(&instance_id)).await {
            Ok(resp) => {
                info!(arm_id = resp.data.arm_id, instance = %instance_id, "discovered arm");
                map.insert(resp.data.arm_id, instance_id);
            }
            Err(e) => warn!(instance = %instance_id, error = %e, "get_arm_id failed; skipping"),
        }
    }
    if map.is_empty() {
        warn!("no arm instances discovered");
    }
    map
}

async fn discover_grippers(runner: &NodeRunner, token: &CancellationToken) -> HashMap<u8, String> {
    let mut map = HashMap::new();
    for instance_id in running_instances(runner, GRIPPER_NODE_NAME, token).await {
        // TODO: remove after interface_v1 — launcher bindings replace get_gripper_id discovery.
        match gripper_get_gripper_id::poll(runner, SERVICE_TIMEOUT, None, Some(&instance_id)).await
        {
            Ok(resp) => {
                info!(gripper_id = resp.data.gripper_id, instance = %instance_id, "discovered gripper");
                map.insert(resp.data.gripper_id, instance_id);
            }
            Err(e) => warn!(instance = %instance_id, error = %e, "get_gripper_id failed; skipping"),
        }
    }
    if map.is_empty() {
        warn!("no gripper instances discovered");
    }
    map
}

// Poll the stack until the node has at least one running instance, or the discovery window closes.
async fn running_instances(
    runner: &NodeRunner,
    node_name: &str,
    token: &CancellationToken,
) -> Vec<String> {
    let deadline = Instant::now() + DISCOVERY_TIMEOUT;
    loop {
        match stack_list(runner, false, None).await {
            Ok(list) => {
                let ids: Vec<String> = list
                    .graph
                    .nodes
                    .iter()
                    .filter(|node| node.name == node_name)
                    .flat_map(|node| node.running_instance_ids())
                    .map(str::to_string)
                    .collect();
                if !ids.is_empty() || Instant::now() >= deadline {
                    return ids;
                }
            }
            Err(e) => {
                warn!(node = node_name, error = %e, "stack_list failed");
                if Instant::now() >= deadline {
                    return Vec::new();
                }
            }
        }
        tokio::select! {
            _ = token.cancelled() => return Vec::new(),
            _ = tokio::time::sleep(DISCOVERY_RETRY_INTERVAL) => {}
        }
    }
}
