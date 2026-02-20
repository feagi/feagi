"""
Simple Video Streaming Example - Using VideoStreamAgent

Clean separation of concerns:
- User manages FEAGI engine lifecycle
- VideoStreamAgent focuses ONLY on video streaming

Video is sent to FEAGI only; no local video window is shown.

To see neuron activity in Brain Visualizer (BV):
1. Load a genome (or connectome) that includes vision cortical areas before
   engine.start(), e.g. engine.load_config(...).load_genome("genome.json").start()
2. Connect BV to the same FEAGI instance (WebSocket visualization port from config).
3. Without a genome, FEAGI has no cortical structure, so no neurons fire and
   BV will show no activity even though frames are being sent.

If FEAGI logs "Unknown cortical area" (e.g. aXN2aQkAAwI=): the genome's vision (IPU)
areas use different cortical IDs than the ones the SDK sends. Use a genome that
defines vision input areas compatible with the agent's segmented-vision pipeline,
or ensure agent registration and genome are aligned (see FEAGI/BDU docs).

Usage:
    python example_video_simple.py
    # Auto-uses first .mov in examples/assets/ (or repo assets/) when available
    FEAGI_VIDEO_PATH=/path/to/video.mp4 python example_video_simple.py

Run from the examples/ directory so feagi_configuration.toml is found.

Requires FEAGI to be running and a valid agent descriptor. Either:
- Set FEAGI_AGENT_DESCRIPTOR_B64 to a base64-encoded 48-byte AgentDescriptor
  (obtained by registering with FEAGI first), or
- Ensure the SDK can register and obtain a descriptor automatically where supported.
"""

import base64
import os
import sys
from pathlib import Path

import toml

from feagi.engine import FeagiEngine
from feagi.agent import VideoStreamAgent

def _discover_default_video_path() -> str:
    example_dir = Path(__file__).resolve().parent
    repo_root = example_dir.parent
    candidate_dirs = (example_dir / "assets", repo_root / "assets")
    for directory in candidate_dirs:
        if directory.is_dir():
            mov_files = sorted(directory.glob("*.mov"))
            if mov_files:
                return str(mov_files[0])
    return ""


# Video path: env override or auto-detected assets default
VIDEO_PATH = os.environ.get("FEAGI_VIDEO_PATH") or _discover_default_video_path()

CONFIG_PATH = Path(__file__).resolve().parent / "feagi_configuration.toml"


def load_connection_params():
    """Build VideoStreamAgent.start() kwargs from feagi_configuration.toml."""
    with open(CONFIG_PATH, encoding="utf-8") as f:
        config = toml.load(f)
    api = config.get("api", {})
    agent_cfg = config.get("agent", {})
    ports = config.get("ports", config.get("zmq", {}))
    zmq_cfg = config.get("zmq", {})
    timeouts = config.get("timeouts", {})
    # Sensory port: agent.sensory_port or zmq_sensory_port
    feagi_port = agent_cfg.get("sensory_port") or ports.get("zmq_sensory_port", 5558)
    registration_port = agent_cfg.get("registration_port")
    connection_timeout_ms = zmq_cfg.get("socket_connect_timeout", 1000)
    return {
        "feagi_host": agent_cfg.get("advertised_host", api.get("advertised_host", "127.0.0.1")),
        "feagi_port": int(feagi_port),
        "api_port": int(api.get("port", 8000)),
        "transport": "zmq",
        "feagi_http_timeout_s": float(timeouts.get("service_startup", 10.0)),
        "heartbeat_interval_s": 15.0,
        "heartbeat_join_timeout_s": 2.0,
        "registration_port": int(registration_port) if registration_port is not None else None,
        "connection_timeout_ms": int(connection_timeout_ms),
        "registration_retries": 3,
    }


CONNECTION_PARAMS = load_connection_params()


def _ensure_agent_descriptor() -> None:
    """
    Use FEAGI_AGENT_DESCRIPTOR_B64 if set and valid; otherwise generate a random
    48-byte descriptor so the example can run without pre-registering with FEAGI.
    """
    b64 = os.environ.get("FEAGI_AGENT_DESCRIPTOR_B64")
    if b64:
        try:
            raw = base64.b64decode(b64, validate=True)
        except Exception:
            print("Error: FEAGI_AGENT_DESCRIPTOR_B64 is not valid base64.")
            sys.exit(1)
        if len(raw) != 48:
            print(
                f"Error: FEAGI_AGENT_DESCRIPTOR_B64 must decode to 48 bytes, got {len(raw)}."
            )
            sys.exit(1)
        return
    # No env set: generate a valid 48-byte descriptor for local/testing use.
    # Layout: [0:4] instance_id, [4:24] manufacturer (ASCII), [24:44] name (ASCII), [44:48] version (u32 le)
    manufacturer = b"feagi".ljust(20)[:20]
    agent_name = b"video_example".ljust(20)[:20]
    version = (1).to_bytes(4, "little")
    raw = b"\x00\x00\x00\x00" + manufacturer + agent_name + version
    assert len(raw) == 48
    os.environ["FEAGI_AGENT_DESCRIPTOR_B64"] = base64.b64encode(raw).decode("ascii")


def _ensure_auth_token() -> None:
    """
    Use FEAGI_AUTH_TOKEN_B64 if set; otherwise set a random 32-byte base64 token
    so the example can run when FEAGI uses permissive (e.g. Dummy) auth.
    """
    if os.environ.get("FEAGI_AUTH_TOKEN_B64"):
        return
    os.environ["FEAGI_AUTH_TOKEN_B64"] = base64.b64encode(os.urandom(32)).decode("ascii")


_ensure_agent_descriptor()
_ensure_auth_token()

print("=" * 60)
print("FEAGI SDK 2.0 - Video Streaming Examples")
print("=" * 60)

# ==============================================================================
# Example 1: Simplest - User controls FEAGI, agent handles video
# ==============================================================================
print("\nExample 1: Basic video streaming")
print("-" * 60)

# 1. User starts FEAGI (load a genome so vision cortical areas exist and BV can show activity)
engine = FeagiEngine()
engine.load_config("feagi_configuration.toml")
# engine.load_genome("path/to/your_genome.json")  # Uncomment and set path to see activity in BV
engine.start()

# 2. Agent handles ONLY video streaming (frames go to FEAGI; no local window)
with VideoStreamAgent(VIDEO_PATH, connection_params=CONNECTION_PARAMS) as agent:
    frames_sent = 0
    for frame_num, _ in agent.stream(max_frames=50, progress_interval=10, pace_by_fps=True):
        frames_sent = frame_num
        if frame_num % 10 == 0:
            print(f"  Frame {frame_num}/50 sent to FEAGI")

# 3. User stops FEAGI
engine.stop()

print(f"Sent {frames_sent} frames.\n")


# ==============================================================================
# Example 2: Generator pattern for motor control
# ==============================================================================
print("\nExample 2: Generator pattern with motor control")
print("-" * 60)

# Start FEAGI (load a genome if you want to see activity in BV)
engine = FeagiEngine()
engine.load_config("feagi_configuration.toml")
engine.start()

# Use agent as generator for frame-by-frame control
with VideoStreamAgent(VIDEO_PATH, connection_params=CONNECTION_PARAMS) as agent:
    for frame_num, frame_data in agent.stream(max_frames=50):
        # Process vision
        if frame_num % 10 == 0:
            print(f"  Frame {frame_num}: shape={frame_data.shape}")
        
        # Here you would send motor commands based on vision
        # Example: brain_output.receive() and process motor commands

engine.stop()

print("Processed frames with custom logic.\n")


# ==============================================================================
# Example 3: Advanced - Manual control
# ==============================================================================
print("\nExample 3: Manual control (no context manager)")
print("-" * 60)

# Start FEAGI (load a genome if you want to see activity in BV)
engine = FeagiEngine()
engine.load_config("feagi_configuration.toml")
engine.start()

# Create agent
agent = VideoStreamAgent(
    video_path=VIDEO_PATH,
    camera_position="center",
    camera_encoding="absolute",
)

# Manually start agent (same params as from config)
agent.start(**CONNECTION_PARAMS)

# Stream with custom settings
frame_count = 0
for frame_num, frame in agent.stream(
    max_frames=50,
    progress_interval=10,
    pace_by_fps=True
):
    frame_count += 1

# Manually stop agent
agent.stop()

# Stop FEAGI
engine.stop()

print(f"Manual control: {frame_count} frames.\n")


# ==============================================================================
# Example 4: Sensorimotor loop (vision + motor control)
# ==============================================================================
print("\nExample 4: Complete sensorimotor loop")
print("-" * 60)

from feagi.pns import brain_output

# Start FEAGI (load a genome if you want to see activity in BV)
engine = FeagiEngine()
engine.load_config("feagi_configuration.toml")
engine.start()

# VideoStreamAgent for vision input
with VideoStreamAgent(VIDEO_PATH, connection_params=CONNECTION_PARAMS) as agent:
    for frame_num, frame_data in agent.stream(max_frames=30):
        # Vision is already sent by agent.stream()
        
        # Receive motor commands from FEAGI
        # motor_data = brain_output.receive()
        
        # Process motor commands
        # Example: control_robot(motor_data)
        
        if frame_num % 10 == 0:
            print(f"  Sensorimotor loop: frame {frame_num}")

engine.stop()

print("Sensorimotor loop complete.\n")

