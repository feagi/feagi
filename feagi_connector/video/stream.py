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
from typing import Optional, Tuple

import cv2

from feagi_connector import (
    FeagiClient,
    MediaSource,
    SegmentedVisionProcessor,
    get_segmented_3x3_dimensions,
    set_simulation_timestep,
)


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

    # FEAGI client
    client = FeagiClient(host=host, agent_id=agent_id or None)
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

            # Pace by source FPS if available
            if info and info.fps > 0:
                await asyncio.sleep(max(0.0, 1.0 / float(info.fps)))
            else:
                await asyncio.sleep(0.01)
    finally:
        media.release()
        await client.disconnect()


