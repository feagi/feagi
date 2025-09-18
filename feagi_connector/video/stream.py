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
from typing import Optional, Tuple, Dict

import cv2

from feagi_connector import (
    FeagiClient,
    MediaSource,
    SegmentedVisionProcessor,
    get_segmented_3x3_dimensions,
    set_simulation_timestep,
)
from feagi_connector.utils.shm import SharedFrameWriter


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

    # FEAGI client: use default REST-Stream (ZMQ) port; keep rest_port for HTTP helpers only
    client = FeagiClient(host=host, agent_id=agent_id or None)

    # Wait briefly for FEAGI to become reachable (bounded, no infinite retries)
    async def _wait_for_feagi(max_wait_seconds: float = 30.0) -> bool:
        from feagi_connector.api.command_client import FeagiControlClient
        start = asyncio.get_event_loop().time()
        while (asyncio.get_event_loop().time() - start) < max_wait_seconds:
            try:
                ctrl = FeagiControlClient(host=host, port=5555, timeout=1000)
                ok = await ctrl.connect()
                if ok:
                    return True
            except Exception:
                pass
            await asyncio.sleep(1.0)
        return False

    ready = await _wait_for_feagi()
    if not ready:
        return

    # Register with visualization + video preview capability so FEAGI can expose SHM paths
    # Attempt explicit registration to request SHM mappings for this agent
    shm_paths: Dict[str, str] = {}
    try:
        reg = await client.rest_client.register_agent(
            agent_id=client.agent_id,
            agent_type="external",
            capabilities={
                "sensory": True,
                "motor": False,
                "visualization": True,
                # Helps FEAGI create per-agent SHM mappings for preview
                "video_stream_raw": True,
                # Legacy key for older BV builds
                "video_stream": True,
            },
            metadata={"source": "video_agent"},
        )
        if isinstance(reg, dict) and reg.get("status") == 200:
            client.registered = True
            # Parse shared memory mappings if present in message payload
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
    except Exception:
        pass

    if not await client.connect():
        return

    # Discover dimensions and prepare processor
    center_dims, per_dims = get_segmented_3x3_dimensions(host, rest_port)
    processor = SegmentedVisionProcessor(cortical_group_index=group_index, center_dims=center_dims, peripheral_dims=per_dims)

    # Capture
    media = MediaSource(use_webcam=use_webcam, path=path, mirror=mirror)
    if not media.open():
        return
    info = media.info

    # Optional: publish raw RGB frames for BV if FEAGI provided a SHM path
    shm_writer: Optional[SharedFrameWriter] = None
    shm_path = shm_paths.get("video_stream_raw") or shm_paths.get("video_stream")
    if isinstance(shm_path, str) and len(shm_path) > 0:
        try:
            shm_writer = SharedFrameWriter(path=shm_path)
        except Exception:
            shm_writer = None

    # Match FEAGI timestep to source FPS if available
    if sync_fps and info and info.fps > 0:
        set_simulation_timestep(host, rest_port, 1.0 / float(info.fps))

    try:
        while True:
            frame_bgr = media.read()
            if frame_bgr is None:
                await asyncio.sleep(0.01)
                continue

            # Ensure input matches center resolution for segmented pipeline
            try:
                resized = cv2.resize(frame_bgr, center_dims, interpolation=cv2.INTER_LINEAR)
            except Exception:
                resized = frame_bgr

            # Encode to bytes and publish
            sensor_bytes = processor.process_frame(resized)
            if client.sensory_client.socket:
                client.sensory_client.socket.send(sensor_bytes)

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
        # Do not unlink FEAGI-managed shared memory paths; skip explicit close
        await client.disconnect()


