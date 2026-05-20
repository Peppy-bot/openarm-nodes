use crate::drivers::mjdata_bus::BusError;

#[derive(Debug)]
pub enum Error {
    Bus(BusError),
    InvalidParameter(String),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Bus(e) => write!(f, "mjdata bus: {e}"),
            Self::InvalidParameter(m) => write!(f, "invalid parameter: {m}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Bus(e) => Some(e),
            _ => None,
        }
    }
}

impl From<BusError> for Error {
    fn from(e: BusError) -> Self {
        Self::Bus(e)
    }
}

pub type Result<T> = std::result::Result<T, Error>;
