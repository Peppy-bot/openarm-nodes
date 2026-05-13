use peppygen::exposed_services::set_gain;
use v4l::Device;
use v4l::control::{Control, Value};
use v4l::v4l_sys::V4L2_CID_GAIN;

pub fn set_gain(dev: &Device, value: i32) -> set_gain::Response {
    match dev.set_control(Control {
        id: V4L2_CID_GAIN,
        value: Value::Integer(value as i64),
    }) {
        Ok(()) => set_gain::Response::new(true, "gain set".into(), value),
        Err(e) => set_gain::Response::new(false, format!("set gain: {e}"), value),
    }
}
