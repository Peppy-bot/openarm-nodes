use sim_bridge_core::DaemonState;

/// Read PeppyOS daemon connection info from the standard state file.
///
/// Tries `$HOME/.peppy/daemon_state.json` first, then a local-path fallback
/// for development environments where the daemon runs in-tree.
pub fn read_daemon_state() -> Result<DaemonState, String> {
    let path = std::env::var("HOME")
        .ok()
        .map(|h| std::path::PathBuf::from(h).join(".peppy/daemon_state.json"))
        .filter(|p| p.exists())
        .unwrap_or_else(|| std::path::PathBuf::from(".peppy/daemon_state.json"));

    let raw = std::fs::read_to_string(&path)
        .map_err(|e| format!("read {}: {e}", path.display()))?;

    let v: serde_json::Value =
        serde_json::from_str(&raw).map_err(|e| format!("parse daemon_state.json: {e}"))?;

    Ok(DaemonState {
        core_node_name: v["core_node_name"]
            .as_str()
            .ok_or("daemon_state.json missing 'core_node_name'")?
            .to_string(),
        messaging_port: v["messaging_port"]
            .as_u64()
            .ok_or("daemon_state.json missing 'messaging_port'")? as u16,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_core_node_name_returns_err() {
        let v: serde_json::Value =
            serde_json::from_str(r#"{"messaging_port": 7448}"#).unwrap();
        let result = v["core_node_name"]
            .as_str()
            .ok_or("daemon_state.json missing 'core_node_name'");
        assert!(result.is_err());
    }

    #[test]
    fn missing_messaging_port_returns_err() {
        let v: serde_json::Value =
            serde_json::from_str(r#"{"core_node_name": "peppy"}"#).unwrap();
        let result = v["messaging_port"]
            .as_u64()
            .ok_or("daemon_state.json missing 'messaging_port'");
        assert!(result.is_err());
    }
}
