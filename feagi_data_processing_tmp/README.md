# feagi_data_processing_tmp

Temporary, pure-Python (NumPy) fallback utilities to avoid dependency on
experimental native libraries during development.

This module provides:

- 3x3 segmented vision generation (gaze-aware)
- Frame-level and segmented diffs with simple metrics (MSE)

## Install

This module uses only NumPy and ships within the repository. No additional
system packages are required.

## API

- Segmentation
  - `segment_image_3x3(frame_rgb, center_dims, per_dims, gaze=(0.5, 0.5)) -> Dict[str, np.ndarray]`
    - Returns tiles keyed by FEAGI IDs like `iic400`.
  - `build_mosaic_from_segments(tiles, center_dims, per_dims, grid=1) -> np.ndarray`
    - Assemble a visualization mosaic from the tiles.

- Diff
  - `frame_diff(prev_rgb, curr_rgb, threshold=0) -> (diff_rgb, mask, mse)`
  - `segmented_diff(prev_tiles, curr_tiles, threshold=0) -> Dict[str, (diff_rgb, mask, mse)]`

## Notes

- Input frames are RGB uint8 arrays of shape (H, W, 3).
- Resizing uses a small nearest-neighbor implementation to avoid OpenCV.
- All functions are OS-agnostic and have no reliance on environment variables.

## Example

```python
import numpy as np
from feagi_data_processing_tmp import segment_image_3x3, build_mosaic_from_segments, frame_diff

# Create a dummy frame
frame = np.random.randint(0, 255, (360, 640, 3), dtype=np.uint8)

# Segment
tiles = segment_image_3x3(frame, center_dims=(128, 128), per_dims=(64, 64), gaze=(0.5, 0.5))

# Mosaic
mosaic = build_mosaic_from_segments(tiles, center_dims=(128, 128), per_dims=(64, 64))

# Diff with a modified frame
frame2 = np.clip(frame.astype(np.int16) + 5, 0, 255).astype(np.uint8)
_, mask, mse = frame_diff(frame, frame2, threshold=2)
print(mse, mask.sum())
```

## Rationale

This module exists to unblock development while native bindings stabilize.
When the production library is ready, simply replace imports to point back to
it and remove this temporary module.



