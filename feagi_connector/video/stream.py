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
    log_sensor_area_counts,
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
    
    # Connection/registration + resources builder
    shm_paths: Dict[str, str] = {}
    center_dims: Tuple[int, int]
    per_dims: Tuple[int, int]
    processor: Optional[SegmentedVisionProcessor] = None
    gaze_motor: Optional[GazeMotorProcessor] = None
    shm_writer: Optional[SharedFrameWriter] = None
    feagi_writer: Optional[SharedFrameWriter] = None
    sensory_writer: Optional[ShmBytesWriter] = None
    motor_reader_thread = None
    motor_stop_flag = None

    async def _register_and_connect() -> bool:
        nonlocal client, shm_paths, center_dims, per_dims, processor, gaze_motor, shm_writer, feagi_writer, sensory_writer, rest_port, host, agent_id, motor_reader_thread, motor_stop_flag
        # Best-effort disconnect/cleanup
        try:
            await client.disconnect()
        except Exception:
            pass
        client = FeagiClient(host=host, agent_id=agent_id or client.agent_id)
        # Connect in sensory-only mode (skip problematic control stream)
        if not await client.connect_sensory_only():
            logger.error("❌ Failed to connect to FEAGI sensory stream")
            return False
        # Register to obtain SHM paths
        shm_paths = {}
        try:
            reg = await client.rest_client.register_agent(
                agent_id=client.agent_id,
                agent_type="external",
                capabilities={
                    "video": True,
                    "feagi": True,  # FEAGI processed video with segmentation overlays
                    "sensory": True,
                    "motor": {"enabled": True, "sampling_frequency_hz": "burst", "prefer_shm": True},
                    "visualization": True,
                },
                metadata={"source": "video_agent"},
            )
            if isinstance(reg, dict) and reg.get("status") == 200:
                client.registered = True
                try:
                    body = reg.get("body", {})
                    # Extract SHM paths from transport field (new registration API)
                    transport = body.get("transport", {})
                    if isinstance(transport, dict):
                        shm = transport.get("shm_paths", {})
                        if isinstance(shm, dict):
                            shm_paths = {str(k): str(v) for k, v in shm.items()}
                            logger.info(f"✅ [SHM-PARSE] Extracted SHM paths from transport: {shm_paths}")
                except Exception as e:
                    logger.error(f"❌ [SHM-PARSE] Failed to parse transport/SHM paths: {e}")
            logger.info(f"🔍 [SHM-DEBUG] Agent shared memory mappings: {shm_paths}")
            logger.info(f"🔍 [SHM-DEBUG] Video SHM path: {shm_paths.get('video', 'NOT PROVIDED')}")
            logger.info(f"🔍 [SHM-DEBUG] FEAGI SHM path: {shm_paths.get('feagi', 'NOT PROVIDED')}")
        except Exception:
            pass
        # Dimensions and processor with gaze parameters
        center_dims, per_dims = get_segmented_3x3_dimensions(host, rest_port)
        processor = SegmentedVisionProcessor(
            cortical_group_index=group_index,
            center_dims=center_dims,
            peripheral_dims=per_dims,
            eccentricity=(0.2, 0.2),
            modulation=(0.2, 0.2)
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
        feagi_writer = None
        # Canonical: 'video' for raw preview frames
        shm_path = shm_paths.get("video")
        if isinstance(shm_path, str) and len(shm_path) > 0:
            try:
                shm_writer = SharedFrameWriter(path=shm_path)
                logger.info(f"[SHM-VIDEO-RAW] ✅ SharedFrameWriter created: {shm_path}")
            except Exception as e:
                logger.error(f"[SHM-VIDEO-RAW] ❌ Failed to create SharedFrameWriter: {e}")
                shm_writer = None
        else:
            logger.info(f"[SHM-VIDEO-RAW] No video SHM path provided by FEAGI")
        # Canonical: 'feagi' for processed video with segmentation overlays
        feagi_shm_path = shm_paths.get("feagi")
        if isinstance(feagi_shm_path, str) and len(feagi_shm_path) > 0:
            try:
                feagi_writer = SharedFrameWriter(path=feagi_shm_path)
                logger.info(f"[SHM-VIDEO-FEAGI] ✅ SharedFrameWriter created: {feagi_shm_path}")
            except Exception as e:
                logger.error(f"[SHM-VIDEO-FEAGI] ❌ Failed to create SharedFrameWriter: {e}")
                feagi_writer = None
        else:
            logger.info(f"[SHM-VIDEO-FEAGI] No feagi SHM path provided by FEAGI")
        # Setup sensory SHM writer if FEAGI provided a path
        sensory_shm_path = shm_paths.get("sensory")
        if isinstance(sensory_shm_path, str) and len(sensory_shm_path) > 0:
            try:
                sensory_writer = ShmBytesWriter(Path(sensory_shm_path))
            except Exception:
                sensory_writer = None

        # Setup motor SHM reader (for gaze) if FEAGI provided a path
        motor_shm_path = shm_paths.get("motor")
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
                    # Decode gaze parameters (eccentricity/modulation) from FEAGI motor data and update processor
                    try:
                        try:
                            logger.debug(f"[MOTOR] Received payload: {len(payload)} bytes")
                        except Exception:
                            pass
                        # TEMP: aggressively log all decoded motor data regardless of cortical area
                        try:
                            import feagi_rust_py_libs as frpl  # local import to avoid hard dep at module import time
                            fbs = frpl.data_serialization.FeagiByteStructure(payload)
                            mapped = frpl.data_structures.neurons.xyzp.CorticalMappedXYZPNeuronData.new_from_feagi_byte_structure(fbs)
                            for cid_obj, neuron_arrays in mapped.iter_full():
                                try:
                                    x_coords, y_coords, z_coords, potentials = neuron_arrays
                                    count = min(len(x_coords), len(y_coords), len(z_coords), len(potentials))
                                    cid_str = str(cid_obj)
                                    if count > 0:
                                        try:
                                            xs = [int(v) for v in x_coords]
                                            ys = [int(v) for v in y_coords]
                                            zs = [int(v) for v in z_coords]
                                            ps = [float(v) for v in potentials]
                                        except Exception:
                                            # Best-effort conversion
                                            xs = [int(x_coords[i]) for i in range(count)]
                                            ys = [int(y_coords[i]) for i in range(count)]
                                            zs = [int(z_coords[i]) for i in range(count)]
                                            ps = [float(potentials[i]) for i in range(count)]
                                        logger.info(
                                            f"[MOTOR-DECODE] area='{cid_str}' neurons={count} x={xs} y={ys} z={zs} p={ps}"
                                        )
                                    else:
                                        logger.info(f"[MOTOR-DECODE] area={str(cid_obj)} neurons=0")
                                except Exception:
                                    # Best-effort logging; continue on errors
                                    continue
                        except Exception:
                            pass
                        result = gaze_motor.process_motor_bytes(payload)
                        if result is not None:
                            # Expecting 4 floats: eccentricity (x,y) and modulation (x,y).
                            # Sentinel -1.0 means "not provided".
                            ecc_x = ecc_y = mod_x = mod_y = None
                            try:
                                if isinstance(result, (list, tuple)) and len(result) >= 4:
                                    rx, ry, mx, my = float(result[0]), float(result[1]), float(result[2]), float(result[3])
                                    ecc_x = None if rx < 0.0 else rx
                                    ecc_y = None if ry < 0.0 else ry
                                    mod_x = None if mx < 0.0 else mx
                                    mod_y = None if my < 0.0 else my
                            except Exception:
                                ecc_x = ecc_y = mod_x = mod_y = None

                            # Apply eccentricity/modulation updates when provided
                            try:
                                if processor is not None:
                                    if ecc_x is not None and ecc_y is not None:
                                        processor.update_eccentricity(ecc_x, ecc_y)
                                    if mod_x is not None and mod_y is not None and hasattr(processor, 'update_modulation'):
                                        processor.update_modulation(mod_x, mod_y)
                                    # Log final eccentricity + modulation for visibility
                                    ex, ey = processor.eccentricity
                                    mx, my = processor.modulation
                                    logger.info(f"[VISION] Applied ecc=({ex:.4f},{ey:.4f}) mod=({mx:.4f},{my:.4f})")
                            except Exception:
                                pass
                        else:
                            try:
                                logger.debug("[MOTOR] No gaze parameters decoded from payload")
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
                # Debug: log per-area neuron counts before sending
                try:
                    log_sensor_area_counts(logger, sensor_bytes)
                except Exception:
                    pass
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
                
                # Rate-limit and only reconnect after sustained failures
                if (consecutive_failures < max_consecutive_failures or 
                    (current_time - last_reconnect_time) < min_reconnect_interval):
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

            # Write ORIGINAL raw RGB frame (not resized) for BV preview via SHM
            if shm_writer is not None:
                try:
                    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)  # Use original frame_bgr, not resized!
                    shm_writer.write_frame(rgb)
                    logger.debug(f"[SHM-VIDEO-RAW] ✅ Frame written: {rgb.shape} (original size)")
                except Exception as e:
                    logger.error(f"[SHM-VIDEO-RAW] ❌ Failed to write frame: {e}")
                    import traceback
                    logger.error(f"[SHM-VIDEO-RAW] Traceback: {traceback.format_exc()}")

            # Write FEAGI processed video (segmented mosaic with overlays) for BV FEAGI view via SHM
            if feagi_writer is not None:
                try:
                    from feagi_connector.vision.visualize import build_segmented_mosaic
                    mosaic_bgr = build_segmented_mosaic(sensor_bytes, center_dims, per_dims)
                    mosaic_rgb = cv2.cvtColor(mosaic_bgr, cv2.COLOR_BGR2RGB)
                    feagi_writer.write_frame(mosaic_rgb)
                    # Check if mosaic has any non-zero pixels
                    non_zero = cv2.countNonZero(cv2.cvtColor(mosaic_rgb, cv2.COLOR_RGB2GRAY))
                    logger.debug(f"[SHM-VIDEO-FEAGI] ✅ Mosaic written: {mosaic_rgb.shape}, non-zero pixels: {non_zero}")
                except Exception as e:
                    logger.error(f"[SHM-VIDEO-FEAGI] ❌ Failed to write mosaic: {e}")
                    import traceback
                    logger.error(f"[SHM-VIDEO-FEAGI] Traceback: {traceback.format_exc()}")

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


