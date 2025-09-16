"""
Vision processing wrappers built on feagi-rust-py-libs.

Abstractions here reduce boilerplate in agents by handling:
- Image properties setup (resolution, color space, memory order)
- Segmented camera registration (center + peripheral)
- Frame conversion (BGR->RGB uint8) and storage/encoding lifecycle
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
from feagi_connector.cache.sensor_cache import SensorCache


def bgr_to_rgb_uint8(frame_bgr: np.ndarray) -> np.ndarray:
    """Convert BGR uint8 image to RGB uint8 without relying on cv2 in the SDK core.

    This avoids a hard OpenCV dependency inside the SDK core modules.
    """
    if frame_bgr is None or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise ValueError("Expected (H,W,3) uint8 BGR frame")
    if frame_bgr.dtype != np.uint8:
        frame_bgr = frame_bgr.astype(np.uint8, copy=False)
    # swap channels BGR -> RGB
    rgb = frame_bgr[..., ::-1].copy()
    return rgb


class SegmentedVisionProcessor:
    """High-level segmented vision pipeline using feagi-rust-py-libs.

    Usage:
        - Construct with FEAGI-derived center/peripheral dimensions
        - Call process_frame(BGR frame) to get encoded neuron bytes
    """

    def __init__(
        self,
        cortical_group_index: int,
        center_dims: Tuple[int, int],
        peripheral_dims: Tuple[int, int],
    ) -> None:
        import feagi_rust_py_libs as frpl  # local import to keep module light

        self._frpl = frpl
        self.group_index = int(cortical_group_index)
        self.center_dims = (int(center_dims[0]), int(center_dims[1]))
        self.per_dims = (int(peripheral_dims[0]), int(peripheral_dims[1]))

        # In-process Rust-backed sensor cache for encoding
        self._cache = SensorCache()
        self._image_properties = None
        self._seg_props = None

        # Establish base input properties with placeholder; updated on first frame
        color_space = self._frpl.data_structures.data.image_descriptors.ColorSpace.Linear
        color_channels = self._frpl.data_structures.data.image_descriptors.ColorChannelLayout.RGB
        self._memory_order = (
            self._frpl.data_structures.data.image_descriptors.MemoryOrderLayout.WidthsHeightsChannels
        )

        # Segmented properties (output sizing)
        cw, ch = self.center_dims
        pw, ph = self.per_dims
        out_center = self._frpl.data_structures.data.image_descriptors.ImageXYResolution(cw, ch)
        out_per = self._frpl.data_structures.data.image_descriptors.ImageXYResolution(pw, ph)
        seg_res = self._frpl.data_structures.data.image_descriptors.SegmentedXYImageResolutions.create_with_same_sized_peripheral(
            out_center, out_per
        )
        self._seg_props = self._frpl.data_structures.data.image_descriptors.SegmentedImageFrameProperties(
            seg_res, color_channels, color_channels, color_space
        )

        # Gaze defaults; can be configured later if needed
        self._gaze = self._frpl.data_structures.data.image_descriptors.GazeProperties((0.5, 0.5), (0.5, 0.5))

        # Registration happens on first process call when input resolution is known

    def _ensure_registered(self, width: int, height: int) -> None:
        if self._image_properties is None:
            cs = self._frpl.data_structures.data.image_descriptors.ColorSpace.Linear
            cc = self._frpl.data_structures.data.image_descriptors.ColorChannelLayout.RGB
            in_res = self._frpl.data_structures.data.image_descriptors.ImageXYResolution(int(width), int(height))
            self._image_properties = self._frpl.data_structures.data.image_descriptors.ImageFrameProperties(in_res, cs, cc)
            self._cache.segmented_image_camera.register(
                self.group_index, 1, True, self._image_properties, self._seg_props, self._gaze
            )

    def process_frame(self, frame_bgr: np.ndarray, resize_to: Optional[Tuple[int, int]] = None) -> bytes:
        import feagi_rust_py_libs as frpl  # ensure availability inside method

        # Convert to RGB and resize if requested (resize is left to caller if they use cv2 elsewhere)
        rgb = bgr_to_rgb_uint8(frame_bgr)
        h, w = rgb.shape[:2]
        self._ensure_registered(w, h)

        color_space = frpl.data_structures.data.image_descriptors.ColorSpace.Linear
        memory_order = self._memory_order
        image_frame = frpl.data_structures.data.ImageFrame.new_from_array(rgb, color_space, memory_order)
        self._cache.segmented_image_camera.store(self.group_index, 0, image_frame)
        self._cache.encode_cached_data_into_bytes()
        return self._cache.get_most_recent_sensor_bytes()

    @property
    def gaze(self):
        return self._cache.segmented_image_camera.gaze



def numpy_to_image_frame(np_rgb_uint8: np.ndarray):
    """Convert a numpy RGB uint8 array (H, W, 3) to an FRPL ImageFrame.

    This is a thin wrapper that standardizes dtype/layout expectations for callers.
    """
    import feagi_rust_py_libs as frpl
    if np_rgb_uint8 is None or np_rgb_uint8.ndim != 3 or np_rgb_uint8.shape[2] != 3:
        raise ValueError("Expected (H,W,3) RGB uint8 array")
    if np_rgb_uint8.dtype != np.uint8:
        np_rgb_uint8 = np_rgb_uint8.astype(np.uint8, copy=False)
    color_space = frpl.data_structures.data.image_descriptors.ColorSpace.Linear
    memory_order = frpl.data_structures.data.image_descriptors.MemoryOrderLayout.WidthsHeightsChannels
    return frpl.data_structures.data.ImageFrame.new_from_array(np_rgb_uint8, color_space, memory_order)

