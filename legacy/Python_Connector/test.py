import numpy as np
from rust_core_sensiomotor_functions_py import ImageDiff

width = 640
height = 480
channels = 3  # RGB

# Create an ImageDiff instance
image_diff = ImageDiff(width, height, channels)
random_image1 = np.random.randint(0, 255, (height, width, channels), dtype=np.uint8)
difference_out: np.ndarray = image_diff.get_new_delta_from_new_frame(random_image1)



