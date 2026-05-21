#[derive(Debug)]
pub enum Error {
    InvalidParameter(String),
    Transport(String),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidParameter(m) => write!(f, "invalid parameter: {m}"),
            Self::Transport(m) => write!(f, "transport: {m}"),
        }
    }
}

impl std::error::Error for Error {}

pub type Result<T> = std::result::Result<T, Error>;
