"""
RGBD pair streaming example for FEAGI Python SDK.

Registers one physical RGBD camera as sibling sensory units:
- Vision (RGB frame)
- DepthMap (true depth volume, or explicit synthetic fallback when requested)
"""

import base64
import os
import time
from pathlib import Path

import numpy as np
import toml

from feagi.engine import FeagiEngine
from feagi.pns import brain_output


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
    agent_name = b"rgbd_example".ljust(20)[:20]
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


def get_rgb_frame() -> np.ndarray:
    """
    Replace with real RGB camera capture.

    Expected shape: (H, W, 3), uint8.
    """
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    frame[:, :, 1] = 180
    return frame


def get_depth_volume() -> np.ndarray:
    """
    Replace with real depth frame conversion.

    Expected shape: (H, W, Z), float32 where Z = depth bins and each pixel maps
    to one or more bins according to your sensor semantics.
    """
    depth = np.zeros((240, 320, 64), dtype=np.float32)
    depth[:, :, 12] = 1.0
    return depth


def main() -> None:
    config_path = _config_path()
    network = _load_network_config(config_path)
    agent_id = _ensure_agent_descriptor()
    auth_token = _ensure_auth_token()

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
            rgb_resolution_xy=(320, 240),
            depth_dimensions_xyz=(320, 240, 64),
            bundle_id="front_rgbd",
            bundle_type="rgbd_camera",
            frame_change_handling="absolute",
        )
        brain_output.connect()

        print("RGBD streaming started. Press Ctrl+C to stop.")
        while True:
            rgb_frame = get_rgb_frame()
            depth_volume = get_depth_volume()
            brain_output.write_rgbd_tick(
                rgb_group=groups["Vision"],
                depth_group=groups["DepthMap"],
                channel_index=0,
                frame_rgb=rgb_frame,
                depth_map_xyz=depth_volume,
            )
            brain_output.flush_sensory_bytes()
            time.sleep(1.0 / 15.0)
    except KeyboardInterrupt:
        print("Stopping RGBD streaming.")
    finally:
        brain_output.disconnect()
        engine.stop()


if __name__ == "__main__":
    main()
