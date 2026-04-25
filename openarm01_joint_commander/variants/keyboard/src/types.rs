use std::fmt;

#[derive(Debug, Clone, Copy)]
pub enum Axis { X, Y, Z }

#[derive(Debug)]
pub enum Command {
    Nudge { axis: Axis, delta: Option<f64> },
    Goto { x: f64, y: f64, z: f64 },
    SetStep(f64),
    Reset,
    Help,
    Quit,
}

#[derive(Debug, Clone, Copy)]
pub struct CartesianTarget {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl CartesianTarget {
    pub fn zero() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0 }
    }

    pub fn nudge(&mut self, axis: Axis, delta: f64) {
        match axis {
            Axis::X => self.x += delta,
            Axis::Y => self.y += delta,
            Axis::Z => self.z += delta,
        }
    }

    pub fn as_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }
}

#[derive(Debug)]
pub enum BridgeError {
    Config { path: String, source: std::io::Error },
    ConfigParse(String),
}

impl fmt::Display for BridgeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Config { path, source } => write!(f, "config error at '{path}': {source}"),
            Self::ConfigParse(msg) => write!(f, "config parse error: {msg}"),
        }
    }
}

impl std::error::Error for BridgeError {}

pub type Result<T> = std::result::Result<T, BridgeError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nudge_accumulates() {
        let mut t = CartesianTarget::zero();
        t.nudge(Axis::X, 0.1);
        t.nudge(Axis::X, -0.05);
        assert!((t.x - 0.05).abs() < 1e-9);
        assert_eq!(t.y, 0.0);
    }

    #[test]
    fn as_array_order() {
        let t = CartesianTarget { x: 1.0, y: 2.0, z: 3.0 };
        assert_eq!(t.as_array(), [1.0, 2.0, 3.0]);
    }
}
