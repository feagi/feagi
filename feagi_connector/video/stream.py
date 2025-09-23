"""
High-level video streaming helpers to FEAGI in <10 lines of user code.

Example (segmented 3x3 vision):

```python
import asyncio
from feagi_connector.video import stream_segmented_camera

asyncio.run(stream_segmented_camera(use_webcam=True))
```
"""

from __future__ import annotations

import asyncio
import logging
from typing import Optional, Tuple, Dict
import time
import random

import cv2

from feagi_connector import (
    FeagiClient,
    MediaSource,
    SegmentedVisionProcessor,
    GazeMotorProcessor,
    get_segmented_3x3_dimensions,
    set_simulation_timestep,
    poll_motor_shm,
)
from pathlib import Path
from feagi_connector.utils.shm import SharedFrameWriter, ShmBytesWriter

logger = logging.getLogger(__name__)


async def stream_segmented_camera(
    use_webcam: bool = True,
    path: Optional[str] = None,
    *,
    host: str = "127.0.0.1",
    rest_port: int = 8000,
    agent_id: Optional[str] = None,
    group_index: int = 0,
    mirror: bool = True,
    sync_fps: bool = True,
) -> None:
    """Open a webcam/video file and stream frames to FEAGI as segmented vision.

    - Auto-discovers center/peripheral dimensions from FEAGI REST
    - Uses SegmentedVisionProcessor to encode frames to FEAGI bytes
    - Publishes bytes via FeagiClient sensory socket
    """

    # FEAGI client: use default ZMQ REST Stream port (5563) for agent registration
    client = FeagiClient(host=host, agent_id=agent_id or None)

    # Check FEAGI readiness first
    logger.info("Checking FEAGI readiness...")
    readiness = await client.check_feagi_readiness()
    
    if not readiness["ready"]:
        reason = readiness.get("reason", "unknown")
        actions = readiness.get("required_actions", [])
        logger.warning(f"🔒 FEAGI not ready: {reason}")
        logger.info(f"Required actions: {', '.join(actions)}")
        client._show_readiness_guidance(reason, actions)
        return
        
    logger.info("✅ FEAGI is ready - establishing connection...")
    
    # Connect in sensory-only mode (skip problematic control stream)
    if not await client.connect_sensory_only():
        logger.error("❌ Failed to connect to FEAGI sensory stream")
        return

    # Register with visualization + video preview capability so FEAGI can expose SHM paths
    # Connection/registration + resources builder
    shm_paths: Dict[str, str] = {}
    center_dims: Tuple[int, int]
    per_dims: Tuple[int, int]
    processor: Optional[SegmentedVisionProcessor] = None
    gaze_motor: Optional[GazeMotorProcessor] = None
    shm_writer: Optional[SharedFrameWriter] = None
    sensory_writer: Optional[ShmBytesWriter] = None
    motor_reader_thread = None
    motor_stop_flag = None

    async def _register_and_connect() -> bool:
        nonlocal client, shm_paths, center_dims, per_dims, processor, gaze_motor, shm_writer, sensory_writer, rest_port, host, agent_id, motor_reader_thread, motor_stop_flag
        # Best-effort disconnect/cleanup
        try:
            await client.disconnect()
        except Exception:
            pass
        client = FeagiClient(host=host, agent_id=agent_id or client.agent_id)
        # Check FEAGI readiness (shorter timeout for reconnects, no guidance)
        if not await client.connect_with_readiness_check(timeout=30.0, show_guidance=False):
            return False
        # Register to obtain SHM paths
        shm_paths = {}
        try:
            reg = await client.rest_client.register_agent(
                agent_id=client.agent_id,
                agent_type="external",
                capabilities={
                    "sensory": True,
                    "motor": True,
                    "visualization": True,
                    "video_stream_raw": True,
                    "video_stream": True,
                },
                metadata={"source": "video_agent"},
            )
            if isinstance(reg, dict) and reg.get("status") == 200:
                client.registered = True
                try:
                    body = reg.get("body", {})
                    msg = body.get("message", "")
                    if isinstance(msg, str) and msg:
                        import json as _json
                        parsed = _json.loads(msg)
                        shm = parsed.get("shared_memory") or {}
                        if isinstance(shm, dict):
                            shm_paths = {str(k): str(v) for k, v in shm.items()}
                except Exception:
                    pass
            try:
                logger.info(f"[SHM] Agent shared memory mappings: {shm_paths}")
            except Exception:
                pass
        except Exception:
            pass
        # Connect streams (sensory-only mode for video agent)
        if not await client.connect_sensory_only():
            return False
        # Dimensions and processor with gaze control
        center_dims, per_dims = get_segmented_3x3_dimensions(host, rest_port)
        processor = SegmentedVisionProcessor(
            cortical_group_index=group_index, 
            center_dims=center_dims, 
            peripheral_dims=per_dims,
            eccentricity=(0.2, 0.2),
            modularity=(0.2, 0.2),
            gaze_position=(0.5, 0.5)
        )
        # Ensure gaze motor processor exists
        if gaze_motor is None:
            try:
                gaze_motor = GazeMotorProcessor(
                    cortical_group_index=group_index,
                    num_channels=10,
                    gaze_resolution=8,
                )
                gaze_motor.register_gaze_motor()
            except Exception:
                gaze_motor = None
        # Setup SHM writer if FEAGI provided a path
        # Close any previous writers without unlink
        if shm_writer is not None:
            try:
                shm_writer.close()
            except Exception:
                pass
        shm_writer = None
        if sensory_writer is not None:
            try:
                sensory_writer.close()
            except Exception:
                pass
        sensory_writer = None
        shm_path = (shm_paths.get("video_stream_raw") or shm_paths.get("video_stream"))
        if isinstance(shm_path, str) and len(shm_path) > 0:
            try:
                shm_writer = SharedFrameWriter(path=shm_path)
            except Exception:
                shm_writer = None
        # Setup sensory SHM writer if FEAGI provided a path
        sensory_shm_path = shm_paths.get("sensory")
        if isinstance(sensory_shm_path, str) and len(sensory_shm_path) > 0:
            try:
                sensory_writer = ShmBytesWriter(Path(sensory_shm_path))
            except Exception:
                sensory_writer = None

        # Setup motor SHM reader (for gaze) if FEAGI provided a path
        motor_shm_path = (shm_paths.get("motor") or shm_paths.get("motor_stream"))
        # Clean up any existing motor polling thread
        try:
            if motor_reader_thread is not None and motor_reader_thread.is_alive():
                if motor_stop_flag is not None:
                    motor_stop_flag.set()
                motor_reader_thread.join(timeout=0.5)
        except Exception:
            pass
        motor_reader_thread = None
        motor_stop_flag = None

        if isinstance(motor_shm_path, str) and len(motor_shm_path) > 0 and gaze_motor is not None:
            try:
                import threading

                motor_stop_flag = threading.Event()

                def on_motor_payload(payload: bytes) -> None:
                    # Decode gaze from FEAGI motor data and update processor
                    try:
                        try:
                            logger.debug(f"[MOTOR] Received payload: {len(payload)} bytes")
                        except Exception:
                            pass
                        result = gaze_motor.process_motor_bytes(payload)
                        if result is not None:
                            gaze_x, gaze_y = result
                            # Log the exact gaze values received from FEAGI
                            # Normalized range [0,1]; approximate grid coordinates based on resolution
                            try:
                                grid_x = int(round(gaze_x * gaze_motor.gaze_resolution))
                                grid_y = int(round(gaze_y * gaze_motor.gaze_resolution))
                            except Exception:
                                grid_x = grid_y = -1
                            logger.info(
                                f"🧭 FEAGI gaze update: gaze_x={gaze_x:.4f}, gaze_y={gaze_y:.4f} (grid≈{grid_x},{grid_y})"
                            )
                            # Apply gaze to vision processor
                            try:
                                if processor is not None:
                                    processor.update_gaze(gaze_x, gaze_y)
                            except Exception:
                                pass
                        else:
                            try:
                                logger.debug("[MOTOR] No gaze decoded from payload")
                            except Exception:
                                pass
                    except Exception:
                        # Ignore malformed payloads but keep polling
                        pass

                motor_reader_thread = threading.Thread(
                    target=poll_motor_shm,
                    args=(Path(motor_shm_path), motor_stop_flag.is_set, on_motor_payload, 0.02),
                    daemon=True,
                )
                motor_reader_thread.start()
                logger.info(f"🔌 Subscribed to FEAGI motor SHM for gaze: {motor_shm_path}")
            except Exception as e:
                logger.debug(f"Motor SHM subscription failed: {e}")
        else:
            logger.info("ℹ️ No motor SHM path provided by FEAGI; gaze updates will not be logged. Ensure agent requests motor capability and FEAGI SHM is enabled.")
        return True

    # Initial registration and connection
    ok = await _register_and_connect()
    if not ok:
        return

    # Capture
    media = MediaSource(use_webcam=use_webcam, path=path, mirror=mirror)
    if not media.open():
        if use_webcam:
            logger.error("❌ Failed to access webcam")
            logger.error("📱 macOS users: Go to System Preferences → Privacy & Security → Camera")
            logger.error("🔓 Enable camera access for Terminal or Python, then restart")
            logger.error("🔍 Tried camera indices 0, 1, 2 - none responded")
        else:
            logger.error(f"❌ Failed to open video file: {path}")
        return
    info = media.info
    
    if use_webcam:
        logger.info(f"📹 Webcam opened successfully: {info.width}x{info.height} @ {info.fps} FPS")
    else:
        logger.info(f"🎬 Video file opened: {path} ({info.width}x{info.height} @ {info.fps} FPS, {info.total_frames} frames)")

    # Match FEAGI timestep to source FPS if available
    if sync_fps and info and info.fps > 0:
        set_simulation_timestep(host, rest_port, 1.0 / float(info.fps))

    # Reconnect backoff state
    backoff = 1.0
    max_backoff = 10.0
    consecutive_failures = 0
    max_consecutive_failures = 5
    last_reconnect_time = 0.0
    min_reconnect_interval = 2.0  # Minimum seconds between reconnects

    try:
        logger.info("🎯 Starting video streaming to FEAGI...")
        while True:
            frame_bgr = media.read()
            if frame_bgr is None:
                await asyncio.sleep(0.01)
                continue

            # Ensure input matches center resolution for segmented pipeline
            try:
                resized = cv2.resize(frame_bgr, center_dims, interpolation=cv2.INTER_LINEAR)
            except Exception as e:
                logger.warning(f"⚠️ Frame resize failed: {e}")
                resized = frame_bgr

            # Encode to bytes and publish (reconnect on failure)
            try:
                assert processor is not None
                sensor_bytes = processor.process_frame(resized)
                if client.sensory_client.socket:
                    client.sensory_client.socket.send(sensor_bytes)
                if sensory_writer is not None:
                    try:
                        sensory_writer.write(sensor_bytes)
                    except Exception:
                        pass
                # Success: reset failure counters
                backoff = 1.0
                consecutive_failures = 0
            except Exception as e:
                consecutive_failures += 1
                current_time = time.time()
                
                # Rate-limit reconnection attempts
                if (consecutive_failures >= max_consecutive_failures or 
                    current_time - last_reconnect_time < min_reconnect_interval):
                    logger.warning(f"⚠️ Frame processing failed ({consecutive_failures} consecutive): {e}")
                    await asyncio.sleep(0.1)
                    continue
                
                logger.warning(f"🔄 Connection issue detected, attempting reconnect... ({consecutive_failures} failures)")
                
                # Attempt bounded backoff reconnect
                delay = backoff + random.uniform(0, 0.25)
                await asyncio.sleep(delay)
                backoff = min(max_backoff, backoff * 2.0)
                last_reconnect_time = current_time
                
                ok = await _register_and_connect()
                if not ok:
                    logger.error("❌ Reconnection failed, retrying...")
                    continue
                else:
                    logger.info("✅ Reconnected successfully")
                    consecutive_failures = 0

            # Optionally write raw RGB frame for BV preview via SHM
            if shm_writer is not None:
                try:
                    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                    shm_writer.write_frame(rgb)
                except Exception:
                    pass

            # Pace by source FPS if available
            if info and info.fps > 0:
                await asyncio.sleep(max(0.0, 1.0 / float(info.fps)))
            else:
                await asyncio.sleep(0.01)
    finally:
        media.release()
        # Do not unlink FEAGI-managed shared memory paths; close mapping only
        try:
            if shm_writer is not None:
                shm_writer.close()
        except Exception:
            pass
        try:
            if sensory_writer is not None:
                sensory_writer.close()
        except Exception:
            pass
        # Stop motor polling thread
        try:
            if motor_reader_thread is not None:
                if motor_stop_flag is not None:
                    motor_stop_flag.set()
                motor_reader_thread.join(timeout=0.5)
        except Exception:
            pass
        await client.disconnect()


