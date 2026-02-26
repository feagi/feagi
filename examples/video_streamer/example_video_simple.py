"""
Simple Video Streaming Example - Using VideoStreamAgent

Streams all supported media files (video and image) from the assets folder to
FEAGI in an infinite loop. Press Ctrl+C to stop.

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

Place video/image files in ./assets (relative to this script). Supported
extensions (OpenCV): .mp4, .m4v, .avi, .mov, .mkv, .webm, .wmv, .flv,
.jpg, .jpeg, .png, .bmp, .tiff, .tif, .gif. Videos play to end; images are
streamed for 5 seconds. For images the pipeline temporarily drops the
ImageQuickDiff stage (no frame-to-frame diff) so the static image is sent
to FEAGI; the full pipeline is restored for video. Override assets location:
    FEAGI_ASSETS_DIR=/path/to/folder python example_video_simple.py

Config: feagi_configuration.toml in this folder or parent examples/, or set FEAGI_CONFIG_PATH.

Requires FEAGI to be running and a valid agent descriptor. Either:
- Set FEAGI_AGENT_DESCRIPTOR_B64 to a base64-encoded 48-byte AgentDescriptor
  (obtained by registering with FEAGI first), or
- Ensure the SDK can register and obtain a descriptor automatically where supported.
"""

import base64
import os
import signal
import sys
import time
from pathlib import Path

import toml

from feagi.engine import FeagiEngine
from feagi.agent import VideoStreamAgent
from feagi.pns import brain_input

# OpenCV VideoCapture-supported extensions (video and image)
SUPPORTED_EXTENSIONS = (
    ".mp4", ".m4v", ".avi", ".mov", ".mkv", ".webm", ".wmv", ".flv",
    ".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif", ".gif",
)
IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif", ".gif")
IMAGE_STREAM_DURATION_S = 5.0
IMAGE_STREAM_FPS = 30.0


def _assets_dir() -> Path:
    env_dir = os.environ.get("FEAGI_ASSETS_DIR")
    if env_dir:
        return Path(env_dir)
    return Path(__file__).resolve().parent / "assets"


def discover_media_files(assets_dir: Path) -> list[Path]:
    """Return sorted list of supported media files under assets_dir (non-recursive)."""
    if not assets_dir.is_dir():
        return []
    out = []
    for p in assets_dir.iterdir():
        if p.is_file() and p.suffix.lower() in SUPPORTED_EXTENSIONS:
            out.append(p)
    return sorted(out, key=lambda p: p.name)

# Config: this folder, then parent examples/ folder, or FEAGI_CONFIG_PATH
def _config_path() -> Path:
    env_path = os.environ.get("FEAGI_CONFIG_PATH")
    if env_path:
        return Path(env_path)
    here = Path(__file__).resolve().parent
    local = here / "feagi_configuration.toml"
    if local.is_file():
        return local
    return here.parent / "feagi_configuration.toml"


CONFIG_PATH = _config_path()


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
    motor_port = agent_cfg.get("motor_port") or ports.get("zmq_motor_port", 5564)
    registration_port = agent_cfg.get("registration_port")
    connection_timeout_ms = zmq_cfg.get("socket_connect_timeout", 1000)
    return {
        "feagi_host": agent_cfg.get("advertised_host", api.get("advertised_host", "127.0.0.1")),
        "feagi_port": int(feagi_port),
        "motor_port": int(motor_port),
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

ASSETS_DIR = _assets_dir()
MEDIA_FILES = discover_media_files(ASSETS_DIR)

if not MEDIA_FILES:
    print(f"No supported media files found in {ASSETS_DIR}")
    print(f"Supported extensions: {', '.join(SUPPORTED_EXTENSIONS)}")
    sys.exit(1)

print("=" * 60)
print("FEAGI SDK 2.0 - Video Streaming (loop all assets)")
print("=" * 60)
print(f"Assets dir: {ASSETS_DIR}")
print(f"Media files ({len(MEDIA_FILES)}): {[p.name for p in MEDIA_FILES]}")
print("Looping indefinitely. Press Ctrl+C to stop.")
print("-" * 60)

def _is_image(path: Path) -> bool:
    return path.suffix.lower() in IMAGE_EXTENSIONS


def _swap_pipeline_for_image(agent, use_simple: bool, stage_cache: list) -> None:
    """
    Swap segmented-vision pipeline for static image: no ImageQuickDiff so the
    same frame is not zeroed out. Pipeline is [processor, diff, segmentor];
    for image we use [processor, segmentor]. Restore [processor, diff, segmentor]
    for video.
    """
    cache = getattr(brain_input, "_cache", None)
    if cache is None:
        return
    if not hasattr(agent.camera, "group_id") or agent.camera.group_id is None:
        return
    gid = agent.camera.group_id
    ch = agent.camera.channel
    try:
        if use_simple:
            if len(stage_cache) != 3:
                stage_cache[:] = [
                    cache.sensor_segmented_vision_get_single_stage_properties(gid, ch, 0),
                    cache.sensor_segmented_vision_get_single_stage_properties(gid, ch, 1),
                    cache.sensor_segmented_vision_get_single_stage_properties(gid, ch, 2),
                ]
            cache.sensor_segmented_vision_replace_all_stages(gid, ch, [stage_cache[0], stage_cache[2]])
        else:
            if len(stage_cache) == 3:
                cache.sensor_segmented_vision_replace_all_stages(gid, ch, stage_cache)
    except Exception:
        pass


# Allow Ctrl+C to exit even when blocked in Rust (check flag in Python loops).
stop_requested = False


def _handle_sigint(_sig: int, _frame: object) -> None:
    global stop_requested
    stop_requested = True
    print("\nStopping after current file/frame (Ctrl+C again may force exit)...", flush=True)


signal.signal(signal.SIGINT, _handle_sigint)

engine = FeagiEngine()
engine.load_config(str(CONFIG_PATH))
# engine.load_genome("path/to/your_genome.json")  # Uncomment to see activity in BV
engine.start()

# Single agent for the whole run so FEAGI keeps a stable connection and neuron
# activations stay visible. We only change the media source between files.
# Stage cache for pipeline swap when streaming images (no-diff so image is sent).
_stage_cache = []

try:
    with VideoStreamAgent(
        str(MEDIA_FILES[0]),
        connection_params=CONNECTION_PARAMS,
    ) as agent:
        loop_count = 0
        while True:
            if stop_requested:
                break
            for idx, media_path in enumerate(MEDIA_FILES):
                if stop_requested:
                    break
                print(f"\n[{loop_count + 1}] Playing {media_path.name} ({idx + 1}/{len(MEDIA_FILES)})", flush=True)
                agent.video_path = Path(media_path)
                agent._detect_video_properties()
                if _is_image(media_path):
                    _swap_pipeline_for_image(agent, True, _stage_cache)
                    gen = agent.stream(max_frames=None, progress_interval=0, pace_by_fps=False)
                    try:
                        _, frame_rgb = next(gen, (None, None))
                        if frame_rgb is not None:
                            n = int(IMAGE_STREAM_DURATION_S * IMAGE_STREAM_FPS)
                            for _ in range(n):
                                if stop_requested:
                                    break
                                agent.camera.set_frame(frame_rgb)
                                brain_input.send()
                                time.sleep(1.0 / IMAGE_STREAM_FPS)
                            print(f"  Streamed image for {IMAGE_STREAM_DURATION_S}s ({n} frames)", flush=True)
                    finally:
                        gen.close()
                        _swap_pipeline_for_image(agent, False, _stage_cache)
                else:
                    frames_sent = 0
                    for frame_num, _ in agent.stream(
                        max_frames=None,
                        progress_interval=50,
                        pace_by_fps=True,
                    ):
                        if stop_requested:
                            break
                        frames_sent = frame_num
                    print(f"  Sent {frames_sent} frames", flush=True)
            if stop_requested:
                break
            loop_count += 1
except KeyboardInterrupt:
    print("\nStopped by user (Ctrl+C).")
finally:
    engine.stop()
    print("FEAGI stopped.")
