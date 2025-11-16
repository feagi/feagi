"""
Vision Inputs

Camera and image-based inputs for FEAGI.
"""

from typing import Tuple, Optional, Literal, TYPE_CHECKING
from feagi.pns.inputs.base import BaseInput

if TYPE_CHECKING:
    import numpy as np

# Type hints
CameraEncoding = Literal["absolute", "incremental"]
CameraPosition = Literal[
    "center",
    "top_left", "top_middle", "top_right",
    "middle_left", "middle_right",
    "bottom_left", "bottom_middle", "bottom_right"
]


class Camera(BaseInput):
    """
    Camera input for visual data.
    
    Supports different encoding modes and positions for segmented vision.
    Maps to Rust `sensor_image_camera_*` methods.
    
    Args:
        resolution: (width, height) in pixels
        encoding: "absolute" or "incremental"
        position: Camera position for segmented vision
    
    Example:
        from feagi.pns.inputs import Camera
        from feagi.pns import brain_input
        
        # Register camera
        camera = Camera.register(
            resolution=(1920, 1080),
            encoding="absolute",
            position="center"
        )
        
        # Configure connection
        brain_input.configure(feagi_host="localhost")
        brain_input.connect()
        
        # Main loop
        while True:
            frame = get_camera_frame()  # Your hardware API → (H, W, 3) NumPy
            camera.set_frame(frame)
            brain_input.send()
    """
    
    def __init__(
        self,
        resolution: Tuple[int, int],
        encoding: CameraEncoding = "absolute",
        position: CameraPosition = "center"
    ):
        super().__init__()
        self.resolution = resolution
        self.encoding = encoding
        self.position = position
        self.channel = 0  # Default channel
        
        # Current frame
        self._current_frame: Optional[np.ndarray] = None
        
        # Rust properties (lazy-initialized)
        self._properties = None
    
    @classmethod
    def register(
        cls,
        resolution: Tuple[int, int] = (640, 480),
        encoding: CameraEncoding = "absolute",
        position: CameraPosition = "center"
    ) -> 'Camera':
        """
        Register a new camera input.
        
        Args:
            resolution: (width, height) in pixels
            encoding: "absolute" (full frame) or "incremental" (changes only)
            position: Camera position for segmented vision
        
        Returns:
            Camera instance
        """
        from feagi.pns import brain_input
        
        camera = cls(resolution, encoding, position)
        brain_input.register_input(camera)
        return camera
    
    def set_frame(self, frame):
        """
        Set the current camera frame.
        
        Args:
            frame: NumPy array with shape (H, W, 3) for RGB images
        """
        import numpy as np
        
        if not isinstance(frame, np.ndarray):
            raise TypeError(f"Frame must be a NumPy array, got {type(frame)}")
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError(f"Frame must be (H, W, 3), got shape {frame.shape}")
        
        self._current_frame = frame
    
    def _init_rust_properties(self):
        """Initialize Rust ImageFrameProperties (lazy)"""
        if self._properties is not None:
            return
        
        try:
            import feagi_rust_py_libs as frpl
            
            # Note: Rust expects (height, width)
            self._properties = frpl.io_data.image_descriptors.ImageFrameProperties(
                (self.resolution[1], self.resolution[0]),  # height, width
                frpl.io_data.image_descriptors.ColorSpace.Linear,
                frpl.io_data.image_descriptors.ColorChannelLayout.RGB
            )
        except ImportError as e:
            raise ImportError(
                "feagi_rust_py_libs required for Camera input"
            ) from e
    
    def _register_with_cache(self, cache, group_id: int):
        """Register with Rust IOCache"""
        self._init_rust_properties()
        
        # Build Rust method name
        method_name = f"sensor_image_camera_{self.position}_{self.encoding}_try_register"
        
        # Get method
        if not hasattr(cache, method_name):
            raise AttributeError(
                f"Rust IOCache missing method: {method_name}\n"
                f"encoding={self.encoding}, position={self.position}"
            )
        
        register_method = getattr(cache, method_name)
        
        # Register
        register_method(
            group=group_id,
            number_of_channels=1,
            image_properties=self._properties
        )
    
    def _write_to_cache(self, cache):
        """Write current frame to Rust IOCache"""
        if self._current_frame is None:
            return  # No frame to write yet
        
        try:
            import feagi_rust_py_libs as frpl
            
            # Convert NumPy to Rust ImageFrame
            frame = frpl.io_data.ImageFrame.from_array(
                self._current_frame,
                frpl.io_data.image_descriptors.ColorSpace.Linear,
                frpl.io_data.image_descriptors.MemoryOrderLayout.HeightsWidthsChannels
            )
            
            # Build Rust method name
            method_name = f"sensor_image_camera_{self.position}_{self.encoding}_try_write"
            
            # Get method
            write_method = getattr(cache, method_name)
            
            # Write
            write_method(
                group=self.group_id,
                channel=self.channel,
                data=frame
            )
        except ImportError as e:
            raise ImportError("feagi_rust_py_libs required") from e

