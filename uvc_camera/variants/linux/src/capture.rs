use std::io;
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use peppygen::NodeRunner;
use peppygen::emitted_topics::video_stream;
use peppylib::runtime::CancellationToken;
use tokio::runtime::Handle;
use tokio::sync::mpsc;
use tracing::{error, info, warn};
use v4l::buffer::Type;
use v4l::io::traits::CaptureStream;
use v4l::{Device, prelude::MmapStream};

// V4L2 mmap ring depth.
const BUFFER_COUNT: u32 = 4;
// Matches BUFFER_COUNT to keep capture→emit pipeline depth symmetric.
// Worst-case memory at YUYV 640x480: 4 × ~614 KB ≈ 2.4 MB.
const EMIT_CHANNEL_CAPACITY: usize = 4;
const STATUS_LOG_INTERVAL: Duration = Duration::from_secs(3);
// Bounds how long stream.next() can block before we re-check `cancel`.
// Safe for any fps >= 0.5; cancellation latency during shutdown is at most this.
const STREAM_POLL_TIMEOUT: Duration = Duration::from_secs(2);

pub fn run(
    runner: Arc<NodeRunner>,
    dev: Device,
    width: u32,
    height: u32,
    encoding: String,
    cancel: CancellationToken,
) {
    let mut stream =
        MmapStream::with_buffers(&dev, Type::VideoCapture, BUFFER_COUNT).expect("mmap stream");
    stream.set_timeout(STREAM_POLL_TIMEOUT);

    info!("capture stream started");
    let (frame_tx, frame_rx) = mpsc::channel(EMIT_CHANNEL_CAPACITY);
    spawn_emit(runner.clone(), frame_rx, width, height, encoding);

    let mut frame_id: u32 = 0;
    let mut last_log = Instant::now();

    loop {
        if cancel.is_cancelled() {
            break;
        }
        let (buf, meta) = match stream.next() {
            Ok(v) => v,
            Err(e) if e.kind() == io::ErrorKind::TimedOut => continue,
            Err(e) => {
                error!("v4l capture failed: {e}; stopping capture loop");
                cancel.cancel();
                break;
            }
        };
        let stamp = SystemTime::now();
        let used = meta.bytesused as usize;
        if used > buf.len() {
            error!("invalid bytesused={used} > buf_len={}", buf.len());
            continue;
        }
        let frame = buf[..used].to_vec();
        let id = frame_id;
        frame_id = frame_id.wrapping_add(1);

        if last_log.elapsed() >= STATUS_LOG_INTERVAL {
            info!("emitted frame {id}");
            last_log = Instant::now();
        }

        match frame_tx.try_send((stamp, id, frame)) {
            Ok(()) => {}
            Err(mpsc::error::TrySendError::Full(_)) => {
                warn!("emit backlog full, dropping frame {id}");
            }
            Err(mpsc::error::TrySendError::Closed(_)) => {
                error!("emit task closed; stopping capture loop");
                cancel.cancel();
                break;
            }
        }
    }
    info!("capture stream stopped");
}

/// Drain `frame_rx` and publish frames in order on the `video_stream` topic.
///
/// A single dedicated emit task means frames are published in order and the
/// capture loop drops on backpressure (matches sensor_data QoS) rather than
/// blocking the V4L2 reader.
fn spawn_emit(
    runner: Arc<NodeRunner>,
    mut frame_rx: mpsc::Receiver<(SystemTime, u32, Vec<u8>)>,
    width: u32,
    height: u32,
    encoding: String,
) {
    Handle::current().spawn(async move {
        while let Some((stamp, id, frame)) = frame_rx.recv().await {
            let header = video_stream::MessageHeader { stamp, frame_id: id };
            if let Err(e) =
                video_stream::emit(&runner, header, encoding.clone(), width, height, frame).await
            {
                error!("video_stream emit: {e}");
            }
        }
    });
}
