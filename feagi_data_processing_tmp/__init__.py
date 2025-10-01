"""
Temporary, pure-Python data processing utilities to reduce dependency on
experimental native libraries.

This module provides:
- 3x3 segmented vision generation (gaze-aware)
- Frame and segmented diffs with simple metrics

All functionality is implemented using NumPy only and is OS-agnostic.
"""

from .segmentation import (
    segment_image_3x3,
    build_mosaic_from_segments,
)
from .diff import (
    frame_diff,
    segmented_diff,
)

__all__ = [
    "segment_image_3x3",
    "build_mosaic_from_segments",
    "frame_diff",
    "segmented_diff",
]



