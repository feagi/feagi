//! Video frame reader for sensory input
//!
//! Reads video files frame-by-frame and converts them to ImageFrame format
//! for injection into FEAGI's sensory system.

use anyhow::{Context, Result};
use ffmpeg_next as ffmpeg;
use image::{DynamicImage, ImageBuffer, RgbImage};
use log::{debug, info, warn};
use std::path::Path;

/// Video frame reader that decodes video files frame-by-frame
pub struct VideoReader {
    input_ctx: ffmpeg::format::context::Input,
    decoder: ffmpeg::decoder::Video,
    video_stream_index: usize,
    scaler: ffmpeg::software::scaling::Context,
    frame_count: u64,
    fps: f64,
}

impl VideoReader {
    /// Open a video file for reading
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        // Initialize FFmpeg
        ffmpeg::init().context("Failed to initialize FFmpeg")?;

        // Open input file
        let input_ctx = ffmpeg::format::input(&path)
            .with_context(|| format!("Failed to open video file: {}", path.as_ref().display()))?;

        // Find video stream
        let video_stream = input_ctx
            .streams()
            .best(ffmpeg::media::Type::Video)
            .context("No video stream found")?;

        let video_stream_index = video_stream.index();

        // Get video codec context
        let context_decoder = ffmpeg::codec::context::Context::from_parameters(video_stream.parameters())
            .context("Failed to create codec context")?;

        let decoder = context_decoder
            .decoder()
            .video()
            .context("Failed to create video decoder")?;

        let width = decoder.width();
        let height = decoder.height();
        let fps = video_stream.avg_frame_rate().into();

        info!(
            "Opened video: {}x{} @ {:.2}fps",
            width, height, fps
        );

        // Create scaler to convert frames to RGB24
        let scaler = ffmpeg::software::scaling::Context::get(
            decoder.format(),
            width,
            height,
            ffmpeg::format::Pixel::RGB24,
            width,
            height,
            ffmpeg::software::scaling::Flags::BILINEAR,
        )
        .context("Failed to create video scaler")?;

        Ok(Self {
            input_ctx,
            decoder,
            video_stream_index,
            scaler,
            frame_count: 0,
            fps,
        })
    }

    /// Get the frames per second of the video
    pub fn fps(&self) -> f64 {
        self.fps
    }

    /// Get the total number of frames read so far
    pub fn frame_count(&self) -> u64 {
        self.frame_count
    }

    /// Get video dimensions (width, height)
    pub fn dimensions(&self) -> (u32, u32) {
        (self.decoder.width(), self.decoder.height())
    }

    /// Read the next frame and convert to DynamicImage
    pub fn read_frame(&mut self) -> Result<Option<DynamicImage>> {
        let mut decoded_frame = ffmpeg::frame::Video::empty();

        // Read packets until we get a video frame
        for (stream, packet) in self.input_ctx.packets() {
            if stream.index() == self.video_stream_index {
                self.decoder.send_packet(&packet)
                    .context("Failed to send packet to decoder")?;

                while self.decoder.receive_frame(&mut decoded_frame).is_ok() {
                    // Scale frame to RGB24
                    let mut rgb_frame = ffmpeg::frame::Video::empty();
                    self.scaler.run(&decoded_frame, &mut rgb_frame)
                        .context("Failed to scale frame")?;

                    // Convert to image::DynamicImage
                    let width = rgb_frame.width();
                    let height = rgb_frame.height();
                    let data = rgb_frame.data(0);

                    // Create RGB image from raw data
                    let img: RgbImage = ImageBuffer::from_raw(width, height, data.to_vec())
                        .context("Failed to create image buffer from frame data")?;

                    self.frame_count += 1;

                    if self.frame_count % 100 == 0 {
                        debug!("Processed {} frames", self.frame_count);
                    }

                    return Ok(Some(DynamicImage::ImageRgb8(img)));
                }
            }
        }

        // End of video
        info!("End of video reached. Total frames: {}", self.frame_count);
        Ok(None)
    }

    /// Reset video to beginning (reopen the file)
    pub fn reset(&mut self, path: impl AsRef<Path>) -> Result<()> {
        warn!("Resetting video to beginning...");
        *self = Self::open(path)?;
        Ok(())
    }
}

/// Video loop configuration
pub struct VideoLoopConfig {
    /// Whether to loop the video indefinitely
    pub loop_video: bool,
    /// Path to the video file
    pub video_path: String,
}

impl VideoLoopConfig {
    pub fn new(video_path: String, loop_video: bool) -> Self {
        Self {
            video_path,
            loop_video,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore] // Requires actual video file
    fn test_open_video() {
        let reader = VideoReader::open("test_video.mp4");
        assert!(reader.is_ok() || reader.is_err()); // Just check it doesn't panic
    }
}

