use std::fs;
use std::path::PathBuf;

use serde::Deserialize;

use crate::types::{BridgeError, Result};

const CONFIG_DIR: &str = "config";
const ENV_CONFIG_DIR: &str = "JOINT_COMMANDER_CONFIG_DIR";
const DEFAULT_STEP_M: f64 = 0.01;
const ENV_RUNTIME_CONFIG: &str = "PEPPY_RUNTIME_CONFIG";
const ENV_ARM_ID: &str = "ARM_ID";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Arm {
    Right = 0,
    Left = 1,
}

impl Arm {
    pub fn from_runtime() -> Option<Self> {
        if let Ok(cfg) = std::env::var(ENV_RUNTIME_CONFIG) {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&cfg) {
                if let Some(id) = val.get("arm_id").and_then(|v| v.as_u64()) {
                    return Self::from_id(id as u8);
                }
            }
        }
        let id: u8 = std::env::var(ENV_ARM_ID).ok()?.trim().parse().ok()?;
        Self::from_id(id)
    }

    pub fn from_id(id: u8) -> Option<Self> {
        match id { 0 => Some(Self::Right), 1 => Some(Self::Left), _ => None }
    }

    pub fn id(self) -> u8 { self as u8 }

    pub fn label(self) -> &'static str {
        match self { Self::Right => "right arm", Self::Left => "left arm" }
    }

    fn config_name(self) -> &'static str {
        match self { Self::Right => "openarm_right", Self::Left => "openarm_left" }
    }
}

#[derive(Deserialize)]
struct JointConfig {
    joints: Vec<String>,
}

pub struct ArmConfig {
    pub arm: Option<Arm>,
    pub label: String,
    pub joint_names: Vec<String>,
    pub default_step: f64,
}

impl ArmConfig {
    pub fn load(arm: Option<Arm>) -> Result<Self> {
        let label = arm.map(|a| a.label().to_owned())
            .unwrap_or_else(|| "full robot".to_owned());
        let joint_names = arm
            .map(|a| load_joints(a.config_name()))
            .transpose()?
            .unwrap_or_default();
        Ok(Self { arm, label, joint_names, default_step: DEFAULT_STEP_M })
    }
}

fn config_base() -> PathBuf {
    std::env::var(ENV_CONFIG_DIR).map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(CONFIG_DIR))
}

fn load_joints(config_name: &str) -> Result<Vec<String>> {
    let path = config_base().join(format!("{config_name}.toml"));
    let text = fs::read_to_string(&path).map_err(|source| BridgeError::Config {
        path: path.display().to_string(),
        source,
    })?;
    toml::from_str::<JointConfig>(&text)
        .map(|c| c.joints)
        .map_err(|e| BridgeError::ConfigParse(e.to_string()))
}
