use std::os::unix::fs::FileTypeExt;

use tracing::info;
use v4l::capability::Flags;
use v4l::video::Capture;
use v4l::video::capture::Parameters as CaptureParams;
use v4l::{Device, Format, FourCC};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CaptureFormat {
    Mjpeg,
    Yuyv,
}

impl CaptureFormat {
    /// Human-readable list of fourccs `from_fourcc` accepts. Update alongside
    /// the match arms below.
    pub const SUPPORTED_FOURCCS: &'static str = "MJPG/YUYV";

    pub fn from_fourcc(fourcc: &FourCC) -> Result<Self, [u8; 4]> {
        match &fourcc.repr {
            b"MJPG" => Ok(Self::Mjpeg),
            b"YUYV" => Ok(Self::Yuyv),
            other => Err(*other),
        }
    }

    pub fn fourcc(self) -> FourCC {
        match self {
            Self::Mjpeg => FourCC::new(b"MJPG"),
            Self::Yuyv => FourCC::new(b"YUYV"),
        }
    }

    pub fn topic_encoding(self) -> &'static str {
        match self {
            Self::Mjpeg => "mjpeg",
            Self::Yuyv => "yuyv",
        }
    }
}

/// Request `requested` at `width`x`height` and return what the driver actually
/// applied. V4L2 may clamp dimensions or substitute a different pixel format
/// during `VIDIOC_S_FMT`, so the caller must use the returned values.
pub fn negotiate_format(
    dev: &Device,
    requested: CaptureFormat,
    width: u32,
    height: u32,
) -> Result<(CaptureFormat, u32, u32), String> {
    let active = dev
        .set_format(&Format::new(width, height, requested.fourcc()))
        .map_err(|e| format!("set V4L2 format: {e}"))?;
    info!("active format:\n{active}");
    let format = CaptureFormat::from_fourcc(&active.fourcc).map_err(|raw| {
        format!(
            "unsupported negotiated fourcc {:?} (expected {})",
            std::str::from_utf8(&raw).unwrap_or("?"),
            CaptureFormat::SUPPORTED_FOURCCS,
        )
    })?;
    Ok((format, active.width, active.height))
}

/// Request `fps` and return what the driver actually applied as a `u8`.
/// V4L2's `interval` is seconds-per-frame; fps is its reciprocal.
pub fn negotiate_fps(dev: &Device, fps: u32) -> Result<u8, String> {
    if fps == 0 {
        return Err("fps must be > 0".into());
    }
    let params = dev
        .set_params(&CaptureParams::with_fps(fps))
        .map_err(|e| format!("set V4L2 params: {e}"))?;
    info!("active params:\n{params}");
    let secs_per_frame = params.interval;
    if secs_per_frame.numerator == 0 || secs_per_frame.denominator == 0 {
        return Err("v4l returned zero interval component".into());
    }
    let active_fps = secs_per_frame.denominator / secs_per_frame.numerator;
    u8::try_from(active_fps).map_err(|_| format!("negotiated fps {active_fps} does not fit in u8"))
}

/// Open `path` after confirming it is a V4L2 capture character device.
///
/// Catches misconfiguration (regular files, directories, metadata-only V4L2
/// nodes) up front rather than failing deeper in the stack with a kernel
/// ioctl error.
pub fn open_validated(path: &str) -> Result<Device, String> {
    let meta = std::fs::metadata(path).map_err(|e| format!("{path}: stat: {e}"))?;
    if !meta.file_type().is_char_device() {
        return Err(format!("{path}: not a character device"));
    }
    let dev = Device::with_path(path).map_err(|e| format!("{path}: open: {e}"))?;
    let caps = dev.query_caps().map_err(|e| format!("{path}: query_caps: {e}"))?;
    if !caps.capabilities.contains(Flags::VIDEO_CAPTURE) {
        return Err(format!(
            "{path}: missing VIDEO_CAPTURE capability (caps={})",
            caps.capabilities
        ));
    }
    Ok(dev)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fourcc_mjpg_is_mjpeg() {
        assert_eq!(
            CaptureFormat::from_fourcc(&FourCC::new(b"MJPG")),
            Ok(CaptureFormat::Mjpeg),
        );
    }

    #[test]
    fn fourcc_yuyv_is_yuyv() {
        assert_eq!(
            CaptureFormat::from_fourcc(&FourCC::new(b"YUYV")),
            Ok(CaptureFormat::Yuyv),
        );
    }

    #[test]
    fn fourcc_other_returns_raw_bytes() {
        assert_eq!(
            CaptureFormat::from_fourcc(&FourCC::new(b"Z16 ")),
            Err(*b"Z16 "),
        );
    }

    #[test]
    fn topic_encoding_strings() {
        assert_eq!(CaptureFormat::Mjpeg.topic_encoding(), "mjpeg");
        assert_eq!(CaptureFormat::Yuyv.topic_encoding(), "yuyv");
    }

    fn expect_err<T>(result: Result<T, String>) -> String {
        match result {
            Ok(_) => panic!("expected open_validated to fail"),
            Err(e) => e,
        }
    }

    #[test]
    fn open_validated_rejects_missing_path() {
        let err = expect_err(open_validated("/dev/this_does_not_exist_xyz"));
        assert!(err.contains("stat:"), "unexpected error: {err}");
    }

    #[test]
    fn open_validated_rejects_directory() {
        let err = expect_err(open_validated("/tmp"));
        assert!(
            err.contains("not a character device"),
            "unexpected error: {err}",
        );
    }

    #[test]
    fn open_validated_rejects_regular_file() {
        let tmp = std::env::temp_dir().join("uvc_camera_open_validated_test");
        std::fs::write(&tmp, b"x").unwrap();
        let result = open_validated(tmp.to_str().unwrap());
        let _ = std::fs::remove_file(&tmp);
        let err = expect_err(result);
        assert!(
            err.contains("not a character device"),
            "unexpected error: {err}",
        );
    }

    #[test]
    fn open_validated_rejects_non_v4l2_char_device() {
        // /dev/null is a char device but VIDIOC_QUERYCAP returns ENOTTY.
        let err = expect_err(open_validated("/dev/null"));
        assert!(
            err.contains("query_caps:") || err.contains("VIDEO_CAPTURE"),
            "unexpected error: {err}",
        );
    }
}
