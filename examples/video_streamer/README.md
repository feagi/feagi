# Video Streamer Example

Simple video streaming with `FeagiEngine` and `VideoStreamAgent`. Video is sent to FEAGI only; no local video window.

Covers: basic streaming, generator pattern for motor control, manual agent control, and a sensorimotor loop.

## Requirements

- Python 3.10+
- See `requirements.txt`. Video support requires opencv-python (e.g. install SDK with `[video]` extra).

## Configuration

Config is read from `feagi_configuration.toml`. Lookup order: this folder, then parent `examples/` folder, or set `FEAGI_CONFIG_PATH`. Optional: `FEAGI_VIDEO_PATH` for video file; `FEAGI_AGENT_DESCRIPTOR_B64`, `FEAGI_AUTH_TOKEN_B64` for agent/auth.

By default the example streams from `./assets/rotating_bar.m4v`. Place your video in the `assets/` folder in this directory, or set `FEAGI_VIDEO_PATH` to another path.

## Run

From this folder:

```bash
python example_video_simple.py
```

RGBD paired registration example:

```bash
python example_rgbd_pair.py
```

RealSense true-depth RGBD example:

```bash
FEAGI_DEPTH_BINS=64 FEAGI_RGBD_MAX_DEPTH_M=6.0 python example_rgbd_realsense.py
```

With a custom video path:

```bash
FEAGI_VIDEO_PATH=/path/to/video.mp4 python example_video_simple.py
```

To see neuron activity in Brain Visualizer: load a genome with vision cortical areas before `engine.start()` and connect BV to the same FEAGI instance. If FEAGI logs "Unknown cortical area", use a genome whose vision (IPU) areas match the agent's pipeline (see project docs).

## RGBD Notes

- `example_rgbd_pair.py` registers one physical sensor as sibling `Vision` and
  `DepthMap` units with shared `bundle_id` metadata.
- Replace `get_rgb_frame()` and `get_depth_volume()` with your hardware code
  (for example, RGB + depth streams from RealSense).
- If you want synthetic depth from RGB only, generate it explicitly with
  `brain_output.rgb_frame_to_depth_map_bins(...)` and pass it to
  `write_rgbd_tick(...)`.
- `example_rgbd_realsense.py` uses `pyrealsense2` and true depth frames; install
  RealSense Python bindings in your active virtual environment first.
