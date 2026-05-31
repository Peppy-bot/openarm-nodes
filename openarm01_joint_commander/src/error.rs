use std::fmt;
use std::io;

#[derive(Debug)]
pub enum JointCommanderError {
    Terminal(io::Error),
}

impl fmt::Display for JointCommanderError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Terminal(e) => write!(f, "terminal: {e}"),
        }
    }
}

impl std::error::Error for JointCommanderError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Terminal(e) => Some(e),
        }
    }
}

impl From<io::Error> for JointCommanderError {
    fn from(e: io::Error) -> Self {
        Self::Terminal(e)
    }
}

pub type Result<T> = std::result::Result<T, JointCommanderError>;
