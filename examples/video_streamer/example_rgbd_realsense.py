"""
Intel RealSense RGBD streaming example for FEAGI Python SDK.

This example uses true depth frames from RealSense hardware and registers one
physical camera as paired Vision + DepthMap units in FEAGI.

Required environment variables:
- FEAGI_DEPTH_BINS: positive integer depth bin count (Z dimension)
- FEAGI_RGBD_MAX_DEPTH_M: max depth distance in meters for bin projection
"""

import base64
import os
from pathlib import Path

import numpy as np
import toml

from feagi.engine import FeagiEngine
from feagi.pns import brain_output


def _required_env_int(name: str) -> int:
    raw_value = os.environ.get(name)
    if raw_value is None:
        raise RuntimeError(f"Missing required environment variable: {name}")
    value = int(raw_value)
    if value <= 0:
        raise RuntimeError(f"{name} must be a positive integer")
    return value


def _required_env_float(name: str) -> float:
    raw_value = os.environ.get(name)
    if raw_value is None:
        raise RuntimeError(f"Missing required environment variable: {name}")
    value = float(raw_value)
    if value <= 0.0:
        raise RuntimeError(f"{name} must be > 0")
    return value


def _config_path() -> Path:
    env_path = os.environ.get("FEAGI_CONFIG_PATH")
    if env_path:
        return Path(env_path)
    here = Path(__file__).resolve().parent
    local = here / "feagi_configuration.toml"
    if local.is_file():
        return local
    return here.parent / "feagi_configuration.toml"


def _load_network_config(config_path: Path) -> dict:
    with open(config_path, encoding="utf-8") as config_file:
        config = toml.load(config_file)
    api = config.get("api", {})
    agent_cfg = config.get("agent", {})
    ports = config.get("ports", config.get("zmq", {}))
    zmq_cfg = config.get("zmq", {})
    timeouts = config.get("timeouts", {})
    return {
        "feagi_host": agent_cfg.get(
            "advertised_host",
            api.get("advertised_host", "127.0.0.1"),
        ),
        "feagi_api_port": int(api.get("port", 8000)),
        "feagi_registration_port": int(
            agent_cfg.get(
                "registration_port",
                ports.get("zmq_registration_port", 30001),
            )
        ),
        "feagi_sensory_port": int(
            agent_cfg.get("sensory_port", ports.get("zmq_sensory_port", 5558))
        ),
        "feagi_motor_port": int(
            agent_cfg.get("motor_port", ports.get("zmq_motor_port", 5564))
        ),
        "feagi_connection_timeout_ms": int(
            zmq_cfg.get("socket_connect_timeout", 1000)
        ),
        "feagi_registration_retries": int(
            agent_cfg.get("registration_retries", 3)
        ),
        "feagi_http_timeout_s": float(timeouts.get("service_startup", 10.0)),
    }


def _ensure_agent_descriptor() -> str:
    existing = os.environ.get("FEAGI_AGENT_DESCRIPTOR_B64")
    if existing:
        return existing
    manufacturer = b"feagi".ljust(20)[:20]
    agent_name = b"rs_rgbd_example".ljust(20)[:20]
    version = (1).to_bytes(4, "little")
    raw = b"\x00\x00\x00\x00" + manufacturer + agent_name + version
    descriptor = base64.b64encode(raw).decode("ascii")
    os.environ["FEAGI_AGENT_DESCRIPTOR_B64"] = descriptor
    return descriptor


def _ensure_auth_token() -> str:
    existing = os.environ.get("FEAGI_AUTH_TOKEN_B64")
    if existing:
        return existing
    token = base64.b64encode(os.urandom(32)).decode("ascii")
    os.environ["FEAGI_AUTH_TOKEN_B64"] = token
    return token


def _depth_frame_to_level_plane(
    depth_mm_frame: np.ndarray,
    *,
    depth_scale: float,
    depth_levels: int,
    max_depth_m: float,
) -> np.ndarray:
    """
    Project a metric depth frame onto a quantized one-layer plane.

    Each valid pixel becomes ``level / depth_levels`` with ``level`` in
    ``1..=depth_levels``. RealSense reports 0 for no return; those stay 0 so
    they do not fire.
    """
    depth_meters = depth_mm_frame.astype(np.float32) * float(depth_scale)
    normalized = np.clip(depth_meters / float(max_depth_m), 0.0, 1.0)
    level = np.rint(normalized * (depth_levels - 1)) + 1.0
    plane = (level / float(depth_levels)).astype(np.float32)
    plane[depth_mm_frame == 0] = 0.0
    return plane


def main() -> None:
    try:
        import pyrealsense2 as rs
    except ImportError as import_error:
        raise RuntimeError(
            "pyrealsense2 is required for this example. "
            "Install it in your active virtual environment."
        ) from import_error

    depth_levels = _required_env_int("FEAGI_DEPTH_LEVELS")
    max_depth_m = _required_env_float("FEAGI_RGBD_MAX_DEPTH_M")

    config_path = _config_path()
    network = _load_network_config(config_path)
    agent_id = _ensure_agent_descriptor()
    auth_token = _ensure_auth_token()

    pipeline = rs.pipeline()
    rs_config = rs.config()
    rs_config.enable_stream(rs.stream.color)
    rs_config.enable_stream(rs.stream.depth)
    profile = pipeline.start(rs_config)

    device = profile.get_device()
    serial = device.get_info(rs.camera_info.serial_number)
    bundle_id = f"realsense_{serial}"

    align_to_color = rs.align(rs.stream.color)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = float(depth_sensor.get_depth_scale())
    if depth_scale <= 0.0:
        raise RuntimeError("RealSense depth scale must be > 0")

    initial_frames = align_to_color.process(pipeline.wait_for_frames())
    color_frame = initial_frames.get_color_frame()
    depth_frame = initial_frames.get_depth_frame()
    if color_frame is None or depth_frame is None:
        raise RuntimeError(
            "Failed to read initial RealSense color/depth frames"
        )

    color_image = np.asanyarray(color_frame.get_data(), dtype=np.uint8)
    depth_image = np.asanyarray(depth_frame.get_data(), dtype=np.uint16)
    frame_height, frame_width = color_image.shape[:2]
    depth_shape_matches = (
        depth_image.shape[0] == frame_height
        and depth_image.shape[1] == frame_width
    )
    if not depth_shape_matches:
        raise RuntimeError(
            "Aligned depth frame dimensions do not match color frame"
        )

    engine = FeagiEngine()
    engine.load_config(str(config_path))
    engine.start()

    try:
        brain_output.configure(
            agent_id=agent_id,
            feagi_host=network["feagi_host"],
            feagi_registration_port=network["feagi_registration_port"],
            feagi_sensory_port=network["feagi_sensory_port"],
            feagi_motor_port=network["feagi_motor_port"],
            transport="zmq",
            feagi_connection_timeout_ms=network["feagi_connection_timeout_ms"],
            feagi_registration_retries=network["feagi_registration_retries"],
            feagi_heartbeat_interval_s=5.0,
            feagi_api_port=network["feagi_api_port"],
            feagi_http_timeout_s=network["feagi_http_timeout_s"],
            auth_token_b64=auth_token,
        )

        groups = brain_output.register_rgbd_sensor_pair(
            rgb_group=0,
            depth_group=1,
            rgb_resolution_xy=(frame_width, frame_height),
            depth_resolution_xy=(frame_width, frame_height),
            bundle_id=bundle_id,
            bundle_type="rgbd_camera",
            frame_change_handling="absolute",
        )
        brain_output.connect()

        print("RealSense RGBD streaming started. Press Ctrl+C to stop.")
        while True:
            frames = align_to_color.process(pipeline.wait_for_frames())
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if color_frame is None or depth_frame is None:
                continue

            rgb_frame = np.asanyarray(
                color_frame.get_data(),
                dtype=np.uint8,
            )
            depth_mm_frame = np.asanyarray(
                depth_frame.get_data(),
                dtype=np.uint16,
            )
            depth_plane = _depth_frame_to_level_plane(
                depth_mm_frame,
                depth_scale=depth_scale,
                depth_levels=depth_levels,
                max_depth_m=max_depth_m,
            )

            brain_output.write_rgbd_tick(
                rgb_group=groups["Vision"],
                depth_group=groups["DepthMap"],
                channel_index=0,
                frame_rgb=rgb_frame,
                depth_map_xy=depth_plane,
            )
            brain_output.flush_sensory_bytes()
    except KeyboardInterrupt:
        print("Stopping RealSense RGBD streaming.")
    finally:
        brain_output.disconnect()
        engine.stop()
        pipeline.stop()


if __name__ == "__main__":
    main()
