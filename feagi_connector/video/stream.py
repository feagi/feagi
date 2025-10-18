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
import numpy as np

try:
    import zmq
except ImportError:
    zmq = None  # ZMQ optional

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
from feagi_connector.utils.shm import SharedFrameWriter
from feagi_connector.utils.latest_only_writer import LatestOnlyWriter

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
    eccentricity: Tuple[float, float] = (0.5, 0.5),
    modulation: Tuple[float, float] = (0.5, 0.5),
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
    sensory_writer: Optional[LatestOnlyWriter] = None
    motor_reader_thread = None
    motor_stop_flag = None
    info = None  # Media info (FPS, dimensions)

    async def _initialize_resources():
        """Initialize processor, SHM writers, and motor reader after registration."""
        nonlocal center_dims, per_dims, processor, gaze_motor, shm_writer, feagi_writer, sensory_writer, motor_reader_thread, motor_stop_flag, shm_paths
        import threading  # Import at function level for motor thread
        
        # Dimensions and processor with gaze parameters
        center_dims, per_dims = get_segmented_3x3_dimensions(host, rest_port)
        processor = SegmentedVisionProcessor(
            cortical_group_index=group_index,
            center_dims=center_dims,
            peripheral_dims=per_dims,
            eccentricity=eccentricity,
            modulation=modulation
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
        if feagi_writer is not None:
            try:
                feagi_writer.close()
            except Exception:
                pass
        feagi_writer = None
        if sensory_writer is not None:
            try:
                sensory_writer.close()
            except Exception:
                pass
        sensory_writer = None
        # Canonical: 'video' for raw preview frames
        shm_path = shm_paths.get("video")
        if isinstance(shm_path, str) and len(shm_path) > 0:
            try:
                shm_writer = SharedFrameWriter(path=shm_path)
            except Exception as e:
                logger.error(f"[SHM-VIDEO-RAW] ❌ Failed to create SharedFrameWriter: {e}")
                shm_writer = None
        # Canonical: 'feagi' for processed video with segmentation overlays
        feagi_shm_path = shm_paths.get("feagi")
        if isinstance(feagi_shm_path, str) and len(feagi_shm_path) > 0:
            try:
                feagi_writer = SharedFrameWriter(path=feagi_shm_path)
            except Exception as e:
                logger.error(f"[SHM-VIDEO-FEAGI] ❌ Failed to create SharedFrameWriter: {e}")
                feagi_writer = None
        
        # Setup sensory SHM writer if FEAGI provided a path
        sensory_shm_path = shm_paths.get("sensory")
        if isinstance(sensory_shm_path, str) and len(sensory_shm_path) > 0:
            try:
                # Use LatestOnlyWriter (compatible with Rust ShmReader)
                sensory_writer = LatestOnlyWriter(Path(sensory_shm_path))
                logger.info(f"✅ [SENSORY-SHM] Created LatestOnlyWriter at {sensory_shm_path}")
            except Exception as e:
                logger.error(f"❌ [SENSORY-SHM] Failed to create LatestOnlyWriter: {e}")
                sensory_writer = None

        # Setup motor SHM reader (for gaze) if FEAGI provided a path
        motor_shm_path = shm_paths.get("motor")
        # Clean up any existing motor polling thread
        try:
            if motor_reader_thread is not None and motor_reader_thread.is_alive():
                if motor_stop_flag is not None:
                    motor_stop_flag.set()
                motor_reader_thread.join(timeout=1.0)
        except Exception:
            pass
        motor_reader_thread = None
        motor_stop_flag = None
        
        if isinstance(motor_shm_path, str) and len(motor_shm_path) > 0:
            try:
                motor_stop_flag = threading.Event()
                def on_motor_payload(payload_bytes: bytes):
                    """Callback for motor payloads (gaze updates)."""
                    try:
                        if gaze_motor and processor:
                            # Decode gaze motor payload
                            decoded = gaze_motor.decode_gaze_motor(payload_bytes)
                            if decoded:
                                ecc_x, ecc_y, mod_x, mod_y = decoded
                            else:
                                ecc_x = ecc_y = mod_x = mod_y = None

                            # Apply eccentricity/modulation updates when provided
                            if processor is not None:
                                if ecc_x is not None and ecc_y is not None:
                                    processor.update_eccentricity(ecc_x, ecc_y)
                                if mod_x is not None and mod_y is not None and hasattr(processor, 'update_modulation'):
                                    processor.update_modulation(mod_x, mod_y)
                                # Log final eccentricity + modulation for visibility
                                ex, ey = processor.eccentricity
                                mx, my = processor.modulation
                                logger.info(f"[VISION] Applied ecc=({ex:.4f},{ey:.4f}) mod=({mx:.4f},{my:.4f})")
                        else:
                            logger.debug("[MOTOR] No gaze parameters decoded from payload")
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
    
    async def _register_and_connect(skip_registration: bool = False) -> bool:
        nonlocal client, shm_paths, center_dims, per_dims, processor, gaze_motor, shm_writer, feagi_writer, sensory_writer, rest_port, host, agent_id, motor_reader_thread, motor_stop_flag, info
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
        
        # SKIP registration if requested (will register later with correct FPS)
        if skip_registration:
            logger.info("⏭️  Skipping initial registration - will register after FPS detection")
            return True
        
        # Register to obtain SHM paths - use direct HTTP if ZMQ times out
        shm_paths = {}
        try:
            # Try ZMQ REST stream first
            # Calculate sensory rate from video FPS
            sensory_rate_hz = info.fps if info and info.fps > 0 else 30.0
            
            capabilities_to_send = {
                "video": True,
                "feagi": True,  # FEAGI processed video with segmentation overlays
                "sensory": {"rate_hz": sensory_rate_hz},  # Tell FEAGI how fast to poll SHM!
                "motor": {"enabled": True, "sampling_frequency_hz": "burst", "prefer_shm": True},
                # Don't register as visualization client - video agent doesn't consume visualization data
            }
            
            logger.warning(f"🎬 [VIDEO-AGENT-REG] Registering with capabilities: {capabilities_to_send}")
            
            reg = await client.rest_client.register_agent(
                agent_id=client.agent_id,
                agent_type="external",
                capabilities=capabilities_to_send,
                metadata={"source": "video_agent"},
            )
            logger.warning(f"🎬 [VIDEO-AGENT-REG] Registration response: {reg}")
            logger.info(f"🔍 [REG-RESPONSE-ZMQ] Registration response: status={reg.get('status')}")
            
            # If ZMQ failed or timed out, use direct HTTP
            if not isinstance(reg, dict) or reg.get("status") != 200:
                logger.warning("⚠️ ZMQ registration failed, trying direct HTTP...")
                import urllib.request
                import json as json_lib
                
                http_payload = {
                    "agent_id": client.agent_id,
                    "agent_type": "external",
                    "capabilities": capabilities_to_send,
                    "agent_version": "1.0.0",
                    "controller_version": "2.0.0",
                    "agent_data_port": 0,
                    "agent_ip": "127.0.0.1",
                    "metadata": {"source": "video_agent"},
                }
                logger.warning(f"🎬 [VIDEO-AGENT-HTTP] HTTP registration payload: {http_payload}")
                http_data = json_lib.dumps(http_payload).encode('utf-8')
                
                http_req = urllib.request.Request(
                    f"http://{host}:{rest_port}/v1/agent/register",
                    data=http_data,
                    headers={'Content-Type': 'application/json'},
                    method='POST'
                )
                
                try:
                    with urllib.request.urlopen(http_req, timeout=10) as http_resp:
                        if http_resp.status == 200:
                            http_body = json_lib.loads(http_resp.read().decode('utf-8'))
                            reg = {"status": 200, "body": http_body}
                            client.registered = True
                            logger.info("✅ [REG-RESPONSE-HTTP] Registration via HTTP successful")
                        else:
                            logger.error(f"❌ [REG-RESPONSE-HTTP] HTTP registration failed: {http_resp.status}")
                except Exception as http_err:
                    logger.error(f"❌ [REG-RESPONSE-HTTP] HTTP request failed: {http_err}")
            
            logger.info(f"🔍 [REG-RESPONSE] Registration response: status={reg.get('status')}")
            if isinstance(reg, dict) and reg.get("status") == 200:
                client.registered = True
                body = reg.get("body", {})
                logger.info(f"🔍 [REG-BODY] Body keys: {list(body.keys())}")
                # Extract SHM paths from transport field (new registration API)
                transport = body.get("transport", {})
                logger.info(f"🔍 [TRANSPORT] Transport type: {type(transport)}, keys: {list(transport.keys()) if isinstance(transport, dict) else 'N/A'}")
                if isinstance(transport, dict):
                    shm = transport.get("shm_paths", {})
                    logger.info(f"🔍 [SHM] SHM type: {type(shm)}, content: {shm}")
                    if isinstance(shm, dict):
                        shm_paths = {str(k): str(v) for k, v in shm.items()}
                        logger.info(f"✅ [SHM-PARSE] Extracted SHM paths from transport: {shm_paths}")
                    else:
                        logger.warning(f"⚠️ [SHM] shm_paths is not a dict: {type(shm)}")
                else:
                    logger.warning(f"⚠️ [TRANSPORT] transport is not a dict: {type(transport)}")
            logger.info(f"🔍 [SHM-FINAL] Final shm_paths variable: {shm_paths}")
            
            # Initialize resources after successful registration
            await _initialize_resources()
        except Exception as e:
            logger.error(f"❌ [REG-EXCEPTION] Exception during registration: {e}")
            import traceback
            logger.error(f"❌ [REG-TRACEBACK] {traceback.format_exc()}")
            return False
        
        return True  # Registration and resource initialization succeeded

    # Initial connection WITHOUT registration (will register after FPS detection)
    ok = await _register_and_connect(skip_registration=True)
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

    # NOW register with correct FPS - this is the ONLY registration!
    if info and info.fps > 0:
        logger.warning(f"🎬 [FIRST-REG] Registering with FEAGI using detected FPS: {info.fps} Hz")
        
        # Register agent with detected video FPS
        sensory_rate_hz = info.fps
        capabilities_to_send = {
            "video": True,
            "feagi": True,
            "sensory": {"rate_hz": sensory_rate_hz},
            "motor": {"enabled": True, "sampling_frequency_hz": "burst", "prefer_shm": True},
            # Don't register as visualization client - video agent doesn't consume visualization data
        }
        
        logger.warning(f"🎬 [VIDEO-AGENT-REG] Registering with capabilities: {capabilities_to_send}")
        
        try:
            reg = await client.rest_client.register_agent(
                agent_id=client.agent_id,
                agent_type="external",
                capabilities=capabilities_to_send,
                metadata={"source": "video_agent"},
            )
            logger.warning(f"🎬 [VIDEO-AGENT-REG] Registration response: {reg}")
            logger.info(f"🔍 [REG-RESPONSE-ZMQ] Registration response: status={reg.get('status')}")
            
            if not isinstance(reg, dict) or reg.get("status") != 200:
                # Try HTTP fallback
                logger.warning("⚠️ ZMQ registration failed, trying direct HTTP...")
                import urllib.request
                import json as json_lib
                http_payload = {
                    "agent_id": client.agent_id,
                    "agent_type": "external",
                    "capabilities": capabilities_to_send,
                    "agent_version": "1.0.0",
                    "controller_version": "2.0.0",
                    "agent_data_port": 0,
                    "agent_ip": "127.0.0.1",
                    "metadata": {"source": "video_agent"},
                }
                logger.warning(f"🎬 [VIDEO-AGENT-HTTP] HTTP registration payload: {http_payload}")
                http_data = json_lib.dumps(http_payload).encode('utf-8')
                http_req = urllib.request.Request(
                    f"http://{host}:{rest_port}/v1/agent/register",
                    data=http_data,
                    headers={'Content-Type': 'application/json'},
                    method='POST'
                )
                with urllib.request.urlopen(http_req, timeout=10) as http_resp:
                    if http_resp.status != 200:
                        logger.error(f"❌ Failed to register with correct FPS: {http_resp.status}")
                        return
                    # Parse HTTP response and update reg
                    http_body = json_lib.loads(http_resp.read().decode('utf-8'))
                    reg = {
                        "status": http_resp.status,
                        "body": http_body
                    }
                    logger.info(f"✅ [REG-RESPONSE-HTTP] Registration via HTTP successful: {http_body.get('success')}")
            
            # Mark as registered
            client.registered = True
            
            # Extract SHM paths from registration response
            if isinstance(reg, dict) and reg.get("status") == 200:
                body = reg.get("body", {})
                logger.info(f"🔍 [REG-BODY] Body keys: {list(body.keys())}")
                transport = body.get("transport", {})
                if isinstance(transport, dict):
                    shm = transport.get("shm_paths", {})
                    if isinstance(shm, dict):
                        shm_paths = {str(k): str(v) for k, v in shm.items()}
                        logger.info(f"✅ [SHM-PARSE] Extracted SHM paths: {shm_paths}")
            
            logger.info(f"✅ Registered with video FPS: {info.fps} Hz")
            
            # Initialize resources (processor, SHM writers, motor reader)
            logger.info("🔧 Initializing resources...")
            await _initialize_resources()
            logger.info(f"✅ Resources initialized successfully (sensory_writer={sensory_writer is not None})")
        except Exception as e:
            logger.error(f"❌ Failed to register with FEAGI: {e}")
            import traceback
            logger.error(f"❌ Traceback: {traceback.format_exc()}")
            return

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

    # Background mosaic processing to avoid blocking main loop
    import threading
    import queue
    mosaic_queue = queue.Queue(maxsize=2)  # Drop old mosaics if we fall behind
    mosaic_results = queue.Queue(maxsize=2)
    mosaic_stop = threading.Event()
    
    def mosaic_worker():
        """Background thread for mosaic building"""
        while not mosaic_stop.is_set():
            try:
                item = mosaic_queue.get(timeout=0.1)
                if item is None:
                    break
                sensor_bytes, center_dims, per_dims, ecc, mod = item
                try:
                    from feagi_connector.vision.visualize import build_segmented_mosaic_with_gaze
                    mosaic_bgr = build_segmented_mosaic_with_gaze(
                        sensor_bytes, center_dims, per_dims, ecc, mod
                    )
                    # Try to put result, but don't block if queue is full (drop old frames)
                    try:
                        mosaic_results.put_nowait(mosaic_bgr)
                    except queue.Full:
                        pass
                except Exception as e:
                    logger.debug(f"[MOSAIC-WORKER] Build failed: {e}")
            except queue.Empty:
                continue
    
    mosaic_thread = threading.Thread(target=mosaic_worker, daemon=True)
    mosaic_thread.start()
    
    try:
        logger.info(f"🎯 Starting video streaming to FEAGI... (sensory_writer={sensory_writer is not None}, client.sensory_client.socket={client.sensory_client.socket is not None if client else None})")
        while True:
            _frame_start = time.perf_counter()  # Track total frame time for pacing
            
            # PROFILING: Frame capture
            _capture_start = time.perf_counter()
            frame_bgr = media.read()
            _capture_time = (time.perf_counter() - _capture_start) * 1000
            
            if frame_bgr is None:
                await asyncio.sleep(0.01)
                continue

            # PROFILING: Frame resize
            _resize_start = time.perf_counter()
            # Ensure input matches center resolution for segmented pipeline
            # TODO: Pass original frame once Rust segmentation is fully implemented
            # Currently using resized frame as workaround
            try:
                resized = cv2.resize(frame_bgr, center_dims, interpolation=cv2.INTER_LINEAR)
                _resize_time = (time.perf_counter() - _resize_start) * 1000
            except Exception as e:
                logger.warning(f"⚠️ Frame resize failed: {e}")
                resized = frame_bgr
                _resize_time = (time.perf_counter() - _resize_start) * 1000
            
            # PROFILING: Registration check/reconnect time
            _reg_check_start = time.perf_counter()
            
            # Encode to bytes and publish (reconnect on failure)
            try:
                # Check if agent is registered before sending data
                if not client.registered:
                    if not hasattr(stream_segmented_camera, '_pause_logged') or not stream_segmented_camera._pause_logged:
                        logger.warning("⏸️  Agent not registered - DATA TRANSMISSION PAUSED until registration completes")
                        stream_segmented_camera._pause_logged = True
                    
                    # Attempt periodic reconnection while paused
                    current_time = time.time()
                    if not hasattr(stream_segmented_camera, '_last_reconnect_attempt') or \
                       (current_time - stream_segmented_camera._last_reconnect_attempt) >= 2.0:
                        logger.info("🔄 Attempting to reconnect...")
                        stream_segmented_camera._last_reconnect_attempt = current_time
                        ok = await _register_and_connect()
                        if ok:
                            logger.info("✅ Reconnected successfully - resuming data transmission")
                            consecutive_failures = 0
                            backoff = 1.0
                            # Don't continue, let it process this frame
                        else:
                            await asyncio.sleep(0.5)
                            continue
                    else:
                        await asyncio.sleep(0.5)  # Sleep longer since we're paused
                        continue
                
                # Agent is registered - log resume if we were paused
                if hasattr(stream_segmented_camera, '_pause_logged') and stream_segmented_camera._pause_logged:
                    logger.info("▶️  Agent registered - DATA TRANSMISSION RESUMED")
                    stream_segmented_camera._pause_logged = False
                
                # PROFILING: End registration check
                _reg_check_time = (time.perf_counter() - _reg_check_start) * 1000
                
                assert processor is not None
                _t0 = time.perf_counter()
                sensor_bytes = processor.process_frame(resized)
                _encode_time = (time.perf_counter() - _t0) * 1000
                
                if len(sensor_bytes) == 0:
                    logger.warning(f"[ENCODE] WARNING: Encoded 0 bytes! Frame shape: {resized.shape if resized is not None else 'None'}")
                
                _sensory_start = time.perf_counter()
                # Prefer SHM over ZMQ (lower latency, no buffering)
                if sensory_writer is not None:
                    try:
                        # Log first write only
                        if not hasattr(sensory_writer, '_first_write_logged'):
                            logger.info(f"✅ [SHM-WRITE] First frame: writing {len(sensor_bytes)} bytes to SHM")
                            sensory_writer._first_write_logged = True
                        sensory_writer.write(sensor_bytes)
                    except Exception as e:
                        logger.error(f"[SHM-SENSORY] ❌ Failed to write: {e}")
                        # Mark as disconnected and re-raise to trigger reconnection
                        client.registered = False
                        raise
                elif client.sensory_client.socket and zmq:
                    # Fall back to ZMQ if SHM not available
                    try:
                        client.sensory_client.socket.send(sensor_bytes, flags=zmq.NOBLOCK)
                    except zmq.Again:
                        logger.warning(f"[ZMQ] ⚠️ Send would block (FEAGI not ready)")
                        # Mark as disconnected and re-raise to trigger reconnection
                        client.registered = False
                        raise
                    except Exception as e:
                        logger.error(f"[ZMQ] ❌ Send failed: {e}")
                        # Mark as disconnected and re-raise to trigger reconnection
                        client.registered = False
                        raise
                elif client.sensory_client.socket:
                    logger.warning("[ZMQ] ⚠️ ZMQ not available, cannot send")
                    # Socket exists but ZMQ not available - connection issue
                    client.registered = False
                    raise ConnectionError("ZMQ not available")
                _sensory_time = (time.perf_counter() - _sensory_start) * 1000
                
                # Submit mosaic building to background thread (non-blocking)
                if feagi_writer is not None:
                    try:
                        # Queue mosaic work (non-blocking, drop if queue full)
                        try:
                            mosaic_queue.put_nowait((
                                sensor_bytes, center_dims, per_dims,
                                processor.eccentricity, processor.modulation
                            ))
                        except queue.Full:
                            pass  # Drop this frame if worker is behind
                        
                        # Check if a completed mosaic is ready (non-blocking)
                        mosaic_bgr = None
                        try:
                            mosaic_bgr = mosaic_results.get_nowait()
                        except queue.Empty:
                            pass  # No mosaic ready yet
                        
                        # Only write if we have a mosaic ready
                        if mosaic_bgr is not None:
                            # WORKAROUND: If mosaic is empty or only has background grid (decoder failed), show the input frame segmented
                            non_zero_before = cv2.countNonZero(cv2.cvtColor(mosaic_bgr, cv2.COLOR_BGR2GRAY))
                            # Blank mosaic with just grid has ~10k pixels, real data should have much more
                            should_fallback = non_zero_before < 15000
                        else:
                            should_fallback = False  # Skip fallback if no mosaic yet
                        
                        if should_fallback:
                            logger.warning(f"[SHM-VIDEO-FEAGI] ⚠️ Mosaic mostly empty ({non_zero_before} pixels, likely decoder bug), using fallback: showing segmented input frame")
                            # Build proper 3x3 segmented mosaic from input frame
                            cw, ch = center_dims
                            pw, ph = per_dims
                            grid = 5
                            mosaic_h = ph + grid + ch + grid + ph
                            mosaic_w = pw + grid + cw + grid + pw
                            # Gray background (128, 128, 128) instead of black
                            mosaic_bgr = np.full((mosaic_h, mosaic_w, 3), 128, dtype=np.uint8)
                            
                            # Extract segments from resized frame using eccentricity/modulation
                            ecc_x, ecc_y = processor.eccentricity
                            mod_x, mod_y = processor.modulation
                            frame_h, frame_w = resized.shape[:2]
                            
                            # Calculate center region
                            center_w_pixels = int(frame_w * mod_x)
                            center_h_pixels = int(frame_h * mod_y)
                            center_x_start = int(frame_w * ecc_x - center_w_pixels / 2)
                            center_y_start = int(frame_h * ecc_y - center_h_pixels / 2)
                            
                            # Extract and place 9 segments
                            # Row 1: Top peripherals
                            top_region = resized[0:center_y_start, :]
                            left_col = top_region[:, 0:center_x_start]
                            top_mid = top_region[:, center_x_start:center_x_start+center_w_pixels]
                            right_col = top_region[:, center_x_start+center_w_pixels:]
                            
                            # Row 2: Middle with center
                            mid_region = resized[center_y_start:center_y_start+center_h_pixels, :]
                            mid_left = mid_region[:, 0:center_x_start]
                            center_img = mid_region[:, center_x_start:center_x_start+center_w_pixels]
                            mid_right = mid_region[:, center_x_start+center_w_pixels:]
                            
                            # Row 3: Bottom peripherals
                            bot_region = resized[center_y_start+center_h_pixels:, :]
                            bot_left = bot_region[:, 0:center_x_start]
                            bot_mid = bot_region[:, center_x_start:center_x_start+center_w_pixels]
                            bot_right = bot_region[:, center_x_start+center_w_pixels:]
                            
                            # Place segments in mosaic grid (all peripherals use pw x ph to preserve aspect ratio)
                            # iic600 (top-left)
                            mosaic_bgr[0:ph, 0:pw] = cv2.resize(left_col, (pw, ph)) if left_col.size > 0 else np.zeros((ph, pw, 3), dtype=np.uint8)
                            # iic700 (top-mid) - centered in allocated space
                            top_mid_resized = cv2.resize(top_mid, (pw, ph)) if top_mid.size > 0 else np.zeros((ph, pw, 3), dtype=np.uint8)
                            x_offset = (cw - pw) // 2
                            mosaic_bgr[0:ph, pw+grid+x_offset:pw+grid+x_offset+pw] = top_mid_resized
                            # iic800 (top-right)
                            mosaic_bgr[0:ph, pw+grid+cw+grid:pw+grid+cw+grid+pw] = cv2.resize(right_col, (pw, ph)) if right_col.size > 0 else np.zeros((ph, pw, 3), dtype=np.uint8)
                            
                            # iic300 (mid-left) - centered in allocated space
                            mid_left_resized = cv2.resize(mid_left, (pw, ph)) if mid_left.size > 0 else np.zeros((ph, pw, 3), dtype=np.uint8)
                            y_offset = (ch - ph) // 2
                            mosaic_bgr[ph+grid+y_offset:ph+grid+y_offset+ph, 0:pw] = mid_left_resized
                            # iic400 (center)
                            mosaic_bgr[ph+grid:ph+grid+ch, pw+grid:pw+grid+cw] = cv2.resize(center_img, (cw, ch)) if center_img.size > 0 else np.zeros((ch, cw, 3), dtype=np.uint8)
                            # iic500 (mid-right) - centered in allocated space
                            mid_right_resized = cv2.resize(mid_right, (pw, ph)) if mid_right.size > 0 else np.zeros((ph, pw, 3), dtype=np.uint8)
                            mosaic_bgr[ph+grid+y_offset:ph+grid+y_offset+ph, pw+grid+cw+grid:pw+grid+cw+grid+pw] = mid_right_resized
                            
                            # iic000 (bot-left)
                            mosaic_bgr[ph+grid+ch+grid:ph+grid+ch+grid+ph, 0:pw] = cv2.resize(bot_left, (pw, ph)) if bot_left.size > 0 else np.zeros((ph, pw, 3), dtype=np.uint8)
                            # iic100 (bot-mid) - centered in allocated space
                            bot_mid_resized = cv2.resize(bot_mid, (pw, ph)) if bot_mid.size > 0 else np.zeros((ph, pw, 3), dtype=np.uint8)
                            mosaic_bgr[ph+grid+ch+grid:ph+grid+ch+grid+ph, pw+grid+x_offset:pw+grid+x_offset+pw] = bot_mid_resized
                            # iic200 (bot-right)
                            mosaic_bgr[ph+grid+ch+grid:ph+grid+ch+grid+ph, pw+grid+cw+grid:pw+grid+cw+grid+pw] = cv2.resize(bot_right, (pw, ph)) if bot_right.size > 0 else np.zeros((ph, pw, 3), dtype=np.uint8)
                            
                            logger.info(f"[SHM-VIDEO-FEAGI] 🔄 Fallback mosaic created: {mosaic_bgr.shape} with 9 segments")
                        
                        # Write mosaic to SHM if we have one ready
                        if mosaic_bgr is not None:
                            _shm_mosaic_start = time.perf_counter()
                            mosaic_rgb = cv2.cvtColor(mosaic_bgr, cv2.COLOR_BGR2RGB)
                            feagi_writer.write_frame(mosaic_rgb)
                            _shm_mosaic_time = (time.perf_counter() - _shm_mosaic_start) * 1000
                        else:
                            _shm_mosaic_time = 0.0
                    except Exception as e:
                        logger.error(f"[SHM-VIDEO-FEAGI] ❌ Failed to write mosaic: {e}")
                        # Only attempt reopen if cooldown period has passed to prevent FD leaks
                        current_time = time.time()
                        if (current_time - feagi_writer_last_reopen) > reopen_cooldown:
                            try:
                                if feagi_writer is not None:
                                    path = getattr(feagi_writer, 'path', None)
                                    feagi_writer.close()
                                    if path:
                                        feagi_writer = SharedFrameWriter(path=path)
                                        feagi_writer_last_reopen = current_time
                                        logger.info("[SHM-VIDEO-FEAGI] Reopened SharedFrameWriter after failure")
                            except Exception as reopen_err:
                                logger.warning(f"[SHM-VIDEO-FEAGI] Reopen failed: {reopen_err}")
                                feagi_writer = None
                
                # Success: reset failure counters
                backoff = 1.0
                consecutive_failures = 0
            except Exception as e:
                consecutive_failures += 1
                current_time = time.time()
                
                # Immediately mark as not registered on ANY failure to pause data transmission
                if client.registered:
                    logger.warning(f"⚠️ Send failure detected - marking agent as disconnected")
                    client.registered = False
                
                # Rate-limit and only reconnect after sustained failures
                if (consecutive_failures < max_consecutive_failures or 
                    (current_time - last_reconnect_time) < min_reconnect_interval):
                    logger.warning(f"⚠️ Frame processing failed ({consecutive_failures} consecutive): {e}")
                    await asyncio.sleep(0.1)
                    continue

                logger.warning(f"🔄 Connection issue detected, pausing data transmission and attempting reconnect... ({consecutive_failures} failures)")
                
                # Attempt bounded backoff reconnect
                delay = backoff + random.uniform(0, 0.25)
                logger.info(f"⏸️  Data transmission PAUSED - waiting {delay:.1f}s before reconnect attempt...")
                await asyncio.sleep(delay)
                backoff = min(max_backoff, backoff * 2.0)
                last_reconnect_time = current_time
                
                ok = await _register_and_connect()
                if not ok:
                    logger.error("❌ Reconnection failed, data transmission remains paused - retrying...")
                    continue
                else:
                    logger.info("✅ Reconnected and registered successfully - resuming data transmission")
                    consecutive_failures = 0

            # Write ORIGINAL raw RGB frame (not resized) for BV preview via SHM
            if shm_writer is not None:
                try:
                    _shm_raw_start = time.perf_counter()
                    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)  # Use original frame_bgr, not resized!
                    shm_writer.write_frame(rgb)
                    _shm_raw_time = (time.perf_counter() - _shm_raw_start) * 1000
                except Exception as e:
                    logger.error(f"[SHM-VIDEO-RAW] ❌ Failed to write frame: {e}")
                    # Only attempt reopen if cooldown period has passed to prevent FD leaks
                    current_time = time.time()
                    if (current_time - shm_writer_last_reopen) > reopen_cooldown:
                        try:
                            if shm_writer is not None:
                                path = getattr(shm_writer, 'path', None)
                                shm_writer.close()
                                if path:
                                    shm_writer = SharedFrameWriter(path=path)
                                    shm_writer_last_reopen = current_time
                                    logger.info("[SHM-VIDEO-RAW] Reopened SharedFrameWriter after failure")
                        except Exception as reopen_err:
                            logger.warning(f"[SHM-VIDEO-RAW] Reopen failed: {reopen_err}")
                            shm_writer = None

            # PROFILING: Log frame processing times every 5 seconds
            _frame_elapsed = (time.perf_counter() - _frame_start) * 1000  # Total frame processing time
            if not hasattr(stream_segmented_camera, '_frame_profile_stats'):
                stream_segmented_camera._frame_profile_stats = {
                    'capture': [], 'resize': [], 'reg_check': [], 'encode': [], 
                    'sensory': [], 'shm_mosaic': [], 'shm_raw': [], 'total': [],
                    'last_log': time.time(), 'count': 0
                }
            stats = stream_segmented_camera._frame_profile_stats
            stats['count'] += 1
            if '_capture_time' in locals():
                stats['capture'].append(_capture_time)
            if '_resize_time' in locals():
                stats['resize'].append(_resize_time)
            if '_reg_check_time' in locals():
                stats['reg_check'].append(_reg_check_time)
            if '_encode_time' in locals():
                stats['encode'].append(_encode_time)
            if '_sensory_time' in locals():
                stats['sensory'].append(_sensory_time)
            if '_shm_mosaic_time' in locals():
                stats['shm_mosaic'].append(_shm_mosaic_time)
            if '_shm_raw_time' in locals():
                stats['shm_raw'].append(_shm_raw_time)
            stats['total'].append(_frame_elapsed)
            
            if time.time() - stats['last_log'] >= 5.0:
                def avg_max(lst):
                    return (sum(lst)/len(lst), max(lst)) if lst else (0, 0)
                cap_avg, cap_max = avg_max(stats['capture'])
                res_avg, res_max = avg_max(stats['resize'])
                reg_avg, reg_max = avg_max(stats['reg_check'])
                enc_avg, enc_max = avg_max(stats['encode'])
                sen_avg, sen_max = avg_max(stats['sensory'])
                shm_mos_avg, shm_mos_max = avg_max(stats['shm_mosaic'])
                shm_raw_avg, shm_raw_max = avg_max(stats['shm_raw'])
                total_avg, total_max = avg_max(stats['total'])
                fps = stats['count'] / 5.0
                
                # Calculate breakdown percentages
                processing_time = cap_avg + res_avg + reg_avg + enc_avg + sen_avg + shm_mos_avg + shm_raw_avg
                sleep_time = total_avg - processing_time if total_avg > processing_time else 0.0
                
                logger.warning(
                    f"⚠️ [VIDEO-AGENT-PROFILE] {stats['count']} frames in 5s ({fps:.1f} Hz):\n"
                    f"  Capture:     avg={cap_avg:.1f}ms  max={cap_max:.1f}ms (video read)\n"
                    f"  Resize:      avg={res_avg:.1f}ms  max={res_max:.1f}ms (cv2.resize)\n"
                    f"  Reg Check:   avg={reg_avg:.1f}ms  max={reg_max:.1f}ms (connection validation)\n"
                    f"  Encode:      avg={enc_avg:.1f}ms  max={enc_max:.1f}ms (Rust encode)\n"
                    f"  Sensory:     avg={sen_avg:.1f}ms  max={sen_max:.1f}ms ⚠️ TO FEAGI\n"
                    f"  SHM Mosaic:  avg={shm_mos_avg:.1f}ms  max={shm_mos_max:.1f}ms (background thread)\n"
                    f"  SHM Raw:     avg={shm_raw_avg:.1f}ms  max={shm_raw_max:.1f}ms\n"
                    f"  Sleep/Other: avg={sleep_time:.1f}ms (frame pacing)\n"
                    f"  Total:       avg={total_avg:.1f}ms  max={total_max:.1f}ms (target: {1000.0/info.fps if info and info.fps > 0 else 0:.1f}ms @ {info.fps if info else 0:.1f} FPS)"
                )
                stream_segmented_camera._frame_profile_stats = {
                    'capture': [], 'resize': [], 'reg_check': [], 'encode': [], 
                    'sensory': [], 'shm_mosaic': [], 'shm_raw': [], 'total': [],
                    'last_log': time.time(), 'count': 0
                }

            # Pace by source FPS, accounting for processing time
            if info and info.fps > 0:
                target_frame_time = 1.0 / float(info.fps)
                sleep_time = max(0.0, target_frame_time - (_frame_elapsed / 1000.0))
                await asyncio.sleep(sleep_time)
            else:
                await asyncio.sleep(0.01)
    finally:
        # Stop mosaic worker thread
        mosaic_stop.set()
        try:
            mosaic_queue.put_nowait(None)  # Signal thread to exit
        except:
            pass
        mosaic_thread.join(timeout=1.0)
        
        media.release()
        # Do not unlink FEAGI-managed shared memory paths; close mapping only
        try:
            if shm_writer is not None:
                shm_writer.close()
        except Exception:
            pass
        try:
            if feagi_writer is not None:
                feagi_writer.close()
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


