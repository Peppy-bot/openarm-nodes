mod capture;
mod controls;
mod device;

use std::sync::Arc;

use peppygen::exposed_services::{set_gain, video_stream_info};
use peppygen::{NodeBuilder, NodeRunner, Parameters, Result};
use peppylib::runtime::CancellationToken;
use tracing::{error, info};
use v4l::Device;

use crate::device::{CaptureFormat, negotiate_format, negotiate_fps, open_validated};

fn main() -> Result<()> {
    tracing_subscriber::fmt().with_max_level(tracing::Level::INFO).init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let Parameters { device_path, width, height, fps } = params;
        info!("opening V4L2 device {device_path} {width}x{height}@{fps}fps (requesting Z16)");

        // Open and configure the streaming device.
        let cap_dev = open_validated(&device_path).expect("validate V4L2 device");
        let (capture_format, active_width, active_height) =
            negotiate_format(&cap_dev, CaptureFormat::Z16, width, height)
                .expect("negotiate V4L2 format");
        let active_fps = negotiate_fps(&cap_dev, fps).expect("negotiate V4L2 fps");
        let encoding = capture_format.topic_encoding().to_string();
        info!("publishing encoding={encoding} {active_width}x{active_height}@{active_fps}fps");

        // Streaming and controls use separate fds to avoid VIDIOC contention.
        let ctrl_dev = Arc::new(open_validated(&device_path).expect("open V4L2 control device"));

        // Spawn capture pipeline.
        let cancel = node_runner.cancellation_token().clone();
        spawn_capture(
            node_runner.clone(),
            cap_dev,
            active_width,
            active_height,
            encoding.clone(),
            cancel,
        );
        spawn_video_stream_info(
            node_runner.clone(),
            active_width,
            active_height,
            active_fps,
            encoding,
        );

        // Spawn control service handlers.
        spawn_set_gain(node_runner.clone(), ctrl_dev);

        Ok(())
    })
}

fn spawn_capture(
    runner: Arc<NodeRunner>,
    dev: Device,
    width: u32,
    height: u32,
    encoding: String,
    cancel: CancellationToken,
) {
    let cancel_on_panic = cancel.clone();
    let handle = tokio::task::spawn_blocking(move || {
        capture::run(runner, dev, width, height, encoding, cancel);
    });
    // Watch the blocking task: a silent panic would leave services answering
    // but no frames flowing, so propagate it to the cancellation token.
    tokio::spawn(async move {
        if let Err(e) = handle.await {
            error!("capture task failed: {e}; shutting down");
            cancel_on_panic.cancel();
        }
    });
}

fn spawn_video_stream_info(
    runner: Arc<NodeRunner>,
    width: u32,
    height: u32,
    fps: u8,
    encoding: String,
) {
    tokio::spawn(async move {
        loop {
            let result = video_stream_info::handle_next_request(&runner, |_req| {
                Ok(video_stream_info::Response::new(
                    width,
                    height,
                    fps,
                    encoding.clone(),
                ))
            })
            .await;
            if let Err(e) = result {
                error!("video_stream_info: {e}");
            }
        }
    });
}

fn spawn_set_gain(runner: Arc<NodeRunner>, dev: Arc<Device>) {
    tokio::spawn(async move {
        loop {
            let dev = dev.clone();
            let result = set_gain::handle_next_request(&runner, move |req| {
                Ok(controls::set_gain(&dev, req.data.value))
            })
            .await;
            if let Err(e) = result {
                error!("set_gain: {e}");
            }
        }
    });
}
