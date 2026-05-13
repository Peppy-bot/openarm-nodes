use std::fmt;

use peppygen::exposed_services::{
    set_brightness, set_contrast, set_exposure, set_gain, set_white_balance,
};
use v4l::Device;
use v4l::control::{Control, Value};
use v4l::v4l_sys::{
    V4L2_CID_AUTO_WHITE_BALANCE, V4L2_CID_BRIGHTNESS, V4L2_CID_CONTRAST,
    V4L2_CID_EXPOSURE_ABSOLUTE, V4L2_CID_EXPOSURE_AUTO, V4L2_CID_GAIN,
    V4L2_CID_WHITE_BALANCE_TEMPERATURE,
    v4l2_exposure_auto_type_V4L2_EXPOSURE_AUTO as V4L2_EXPOSURE_AUTO,
    v4l2_exposure_auto_type_V4L2_EXPOSURE_MANUAL as V4L2_EXPOSURE_MANUAL,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExposureMode {
    Auto,
    Manual,
}

impl ExposureMode {
    fn cid_value(self) -> i64 {
        match self {
            Self::Auto => V4L2_EXPOSURE_AUTO as i64,
            Self::Manual => V4L2_EXPOSURE_MANUAL as i64,
        }
    }
}

impl fmt::Display for ExposureMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Auto => "auto",
            Self::Manual => "manual",
        })
    }
}

impl TryFrom<&str> for ExposureMode {
    type Error = String;

    fn try_from(s: &str) -> Result<Self, Self::Error> {
        match s {
            "auto" => Ok(Self::Auto),
            "manual" => Ok(Self::Manual),
            other => Err(format!(
                "unknown exposure mode '{other}' (expected 'auto' or 'manual')"
            )),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WhiteBalanceMode {
    Auto,
    Manual,
}

impl fmt::Display for WhiteBalanceMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Auto => "auto",
            Self::Manual => "manual",
        })
    }
}

impl TryFrom<&str> for WhiteBalanceMode {
    type Error = String;

    fn try_from(s: &str) -> Result<Self, Self::Error> {
        match s {
            "auto" => Ok(Self::Auto),
            "manual" => Ok(Self::Manual),
            other => Err(format!(
                "unknown white_balance mode '{other}' (expected 'auto' or 'manual')"
            )),
        }
    }
}

pub fn set_exposure(dev: &Device, mode: ExposureMode, value: i32) -> set_exposure::Response {
    if let Err(e) = dev.set_control(Control {
        id: V4L2_CID_EXPOSURE_AUTO,
        value: Value::Integer(mode.cid_value()),
    }) {
        return set_exposure::Response::new(false, format!("set exposure mode: {e}"), value);
    }

    if mode == ExposureMode::Manual
        && let Err(e) = dev.set_control(Control {
            id: V4L2_CID_EXPOSURE_ABSOLUTE,
            value: Value::Integer(value as i64),
        })
    {
        return set_exposure::Response::new(false, format!("set exposure value: {e}"), value);
    }

    set_exposure::Response::new(true, format!("exposure set to {mode}"), value)
}

pub fn set_white_balance(
    dev: &Device,
    mode: WhiteBalanceMode,
    temperature: i32,
) -> set_white_balance::Response {
    let auto = mode == WhiteBalanceMode::Auto;

    if let Err(e) = dev.set_control(Control {
        id: V4L2_CID_AUTO_WHITE_BALANCE,
        value: Value::Boolean(auto),
    }) {
        return set_white_balance::Response::new(
            false,
            format!("set white_balance mode: {e}"),
            temperature,
        );
    }

    if !auto
        && let Err(e) = dev.set_control(Control {
            id: V4L2_CID_WHITE_BALANCE_TEMPERATURE,
            value: Value::Integer(temperature as i64),
        })
    {
        return set_white_balance::Response::new(
            false,
            format!("set white_balance temperature: {e}"),
            temperature,
        );
    }

    set_white_balance::Response::new(true, format!("white_balance set to {mode}"), temperature)
}

pub fn set_gain(dev: &Device, value: i32) -> set_gain::Response {
    match dev.set_control(Control {
        id: V4L2_CID_GAIN,
        value: Value::Integer(value as i64),
    }) {
        Ok(()) => set_gain::Response::new(true, "gain set".into(), value),
        Err(e) => set_gain::Response::new(false, format!("set gain: {e}"), value),
    }
}

pub fn set_brightness(dev: &Device, value: i32) -> set_brightness::Response {
    match dev.set_control(Control {
        id: V4L2_CID_BRIGHTNESS,
        value: Value::Integer(value as i64),
    }) {
        Ok(()) => set_brightness::Response::new(true, "brightness set".into(), value),
        Err(e) => set_brightness::Response::new(false, format!("set brightness: {e}"), value),
    }
}

pub fn set_contrast(dev: &Device, value: i32) -> set_contrast::Response {
    match dev.set_control(Control {
        id: V4L2_CID_CONTRAST,
        value: Value::Integer(value as i64),
    }) {
        Ok(()) => set_contrast::Response::new(true, "contrast set".into(), value),
        Err(e) => set_contrast::Response::new(false, format!("set contrast: {e}"), value),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exposure_mode_parses_known_strings() {
        assert_eq!(ExposureMode::try_from("auto"), Ok(ExposureMode::Auto));
        assert_eq!(ExposureMode::try_from("manual"), Ok(ExposureMode::Manual));
    }

    #[test]
    fn exposure_mode_is_case_sensitive() {
        assert!(ExposureMode::try_from("Auto").is_err());
        assert!(ExposureMode::try_from("AUTO").is_err());
    }

    #[test]
    fn exposure_mode_rejects_unknown() {
        assert!(ExposureMode::try_from("").is_err());
        assert!(ExposureMode::try_from("off").is_err());
    }

    #[test]
    fn exposure_mode_maps_to_v4l2_menu_value() {
        assert_eq!(ExposureMode::Auto.cid_value(), 0);
        assert_eq!(ExposureMode::Manual.cid_value(), 1);
    }

    #[test]
    fn exposure_mode_displays_lowercase() {
        assert_eq!(ExposureMode::Auto.to_string(), "auto");
        assert_eq!(ExposureMode::Manual.to_string(), "manual");
    }

    #[test]
    fn white_balance_mode_displays_lowercase() {
        assert_eq!(WhiteBalanceMode::Auto.to_string(), "auto");
        assert_eq!(WhiteBalanceMode::Manual.to_string(), "manual");
    }

    #[test]
    fn white_balance_mode_parses_known_strings() {
        assert_eq!(WhiteBalanceMode::try_from("auto"), Ok(WhiteBalanceMode::Auto));
        assert_eq!(WhiteBalanceMode::try_from("manual"), Ok(WhiteBalanceMode::Manual));
    }

    #[test]
    fn white_balance_mode_is_case_sensitive() {
        assert!(WhiteBalanceMode::try_from("Auto").is_err());
    }

    #[test]
    fn white_balance_mode_rejects_unknown() {
        assert!(WhiteBalanceMode::try_from("").is_err());
        assert!(WhiteBalanceMode::try_from("incandescent").is_err());
    }
}
