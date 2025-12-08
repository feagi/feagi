"""
Vision Inputs

Camera and image-based inputs for FEAGI.
"""

import logging
from typing import Tuple, Optional, Literal, TYPE_CHECKING
from feagi.pns.inputs.base import BaseInput

logger = logging.getLogger(__name__)

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
        
        # Cortical IDs that will be generated (populated during registration)
        self._cortical_ids: list[str] = []
    
    @classmethod
    def register(
        cls,
        resolution: Tuple[int, int] = (640, 480),
        encoding: CameraEncoding = "absolute",
        position: CameraPosition = "center",
        group_id: Optional[int] = None
    ) -> 'Camera':
        """
        Register a new camera input.
        
        Args:
            resolution: (width, height) in pixels
            encoding: "absolute" (full frame) or "incremental" (changes only)
            position: Camera position for segmented vision
            group_id: Optional cortical group ID. If None, auto-allocates.
        
        Returns:
            Camera instance
        """
        from feagi.pns import brain_input
        
        camera = cls(resolution, encoding, position)
        brain_input.register_input(camera, group_id=group_id)
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
            
            # Create ImageFrameProperties using the correct API
            # Note: The attribute is 'channel_layout', not 'color_channel_layout'
            self._properties = frpl.connector_core.data_types.descriptors.ImageFrameProperties(
                frpl.connector_core.data_types.descriptors.ImageXYResolution(
                    self.resolution[0],  # width
                    self.resolution[1]   # height
                ),
                frpl.connector_core.data_types.descriptors.ColorSpace.Linear,
                frpl.connector_core.data_types.descriptors.ColorChannelLayout.RGB
            )
            
            # Verify the attribute exists (for debugging)
            if not hasattr(self._properties, 'channel_layout'):
                raise AttributeError(
                    f"ImageFrameProperties missing 'channel_layout' attribute. "
                    f"Available attributes: {[attr for attr in dir(self._properties) if not attr.startswith('_')]}"
                )
        except ImportError as e:
            raise ImportError(
                "feagi_rust_py_libs required for Camera input"
            ) from e
        except AttributeError as e:
            # Re-raise with more context
            raise AttributeError(
                f"Failed to initialize ImageFrameProperties: {e}\n"
                f"This may indicate a version mismatch with feagi_rust_py_libs."
            ) from e
    
    def _register_with_cache(self, cache, group_id: int):
        """Register with Rust ConnectorAgent"""
        self._init_rust_properties()
        
        # NOTE: sensor_simple_vision_register does NOT exist in current rust-py-libs API
        # The ImageFrame registration code is commented out in the Rust source.
        # Use sensor_segmented_vision_register instead, which is the available API
        method_name = "sensor_segmented_vision_register"
        
        if not hasattr(cache, method_name):
            raise AttributeError(
                f"ConnectorAgent missing method: {method_name}\n"
                f"The simple_vision registration method is not exposed in rust-py-libs.\n"
                f"Using segmented_vision_register as alternative."
            )
        
        import feagi_rust_py_libs as frpl
        cc_desc = frpl.connector_core.data_types.descriptors
        
        # Create required parameters for segmented_vision_register
        # Following pattern from test.py and examples
        # segmented_vision_register requires: group, number_channels, frame_change_handling,
        # input_image_properties, segmented_image_properties, initial_gaze
        
        # FrameChangeHandling - use Absolute (matches encoding="absolute") or Incremental
        # Note: Enum values are accessed as methods that need to be called
        frame_change_handling_enum = frpl.data_structures.genomic.cortical_area.FrameChangeHandling
        if self.encoding == "absolute":
            frame_change_handling = frame_change_handling_enum.Absolute()
        else:  # incremental
            frame_change_handling = frame_change_handling_enum.Incremental()
        
        # SegmentedImageFrameProperties - create following test.py pattern
        # Create center and peripheral resolutions (using same size for simplicity)
        input_res = self._properties.xy_resolution
        center_res = cc_desc.ImageXYResolution(input_res.width, input_res.height)
        peripheral_res = cc_desc.ImageXYResolution(input_res.width, input_res.height)
        
        # Create segmented resolution using create_with_same_sized_peripheral (from test.py)
        segmented_res = cc_desc.SegmentedXYImageResolutions.create_with_same_sized_peripheral(
            center_res,
            peripheral_res
        )
        
        # Create SegmentedImageFrameProperties (from examples)
        # Get channel_layout attribute - it's 'channel_layout', not 'color_channel_layout'
        # Add safety check in case of version mismatch
        if self._properties is None:
            raise RuntimeError("ImageFrameProperties not initialized. Call _init_rust_properties() first.")
        
        # Try to get channel_layout attribute
        if not hasattr(self._properties, 'channel_layout'):
            available_attrs = [attr for attr in dir(self._properties) if not attr.startswith('_')]
            raise AttributeError(
                f"ImageFrameProperties object has no attribute 'channel_layout'. "
                f"Available attributes: {available_attrs}. "
                f"This may indicate a version mismatch with feagi_rust_py_libs."
            )
        
        channel_layout = self._properties.channel_layout
        
        segmented_properties = cc_desc.SegmentedImageFrameProperties(
            segmented_res,
            channel_layout,  # center_color_channels
            channel_layout,  # peripheral_color_channels
            self._properties.color_space
        )
        
        # GazeProperties - create default centered gaze
        # NOTE: GazeProperties may not be registered in rust-py-libs lib.rs yet
        # If not available, this will raise AttributeError with clear message
        try:
            gaze = cc_desc.GazeProperties.create_default_centered()
        except AttributeError:
            # GazeProperties not exposed - needs to be added to rust-py-libs lib.rs
            raise AttributeError(
                "GazeProperties is not available in rust-py-libs.\n"
                "It exists in the Rust code but needs to be registered in lib.rs:\n"
                "add_python_class!(py, m, \"connector_core.data_types.descriptors\", "
                "feagi_connector_core::data_types::descriptors::PyGazeProperties);"
            )
        
        register_method = getattr(cache, method_name)
        
        # Register with all required parameters
        logger.debug(f"Registering segmented vision with: group={group_id}, frame_change_handling={frame_change_handling}, encoding={self.encoding}")
        register_method(
            group=group_id,
            number_channels=1,
            frame_change_handling=frame_change_handling,
            input_image_properties=self._properties,
            segmented_image_properties=segmented_properties,
            initial_gaze=gaze
        )
        
        # Log registration details
        # NOTE: Segmented vision creates 9 cortical IDs (one per segment) with Absolute encoding
        # FEAGI's registration endpoint will auto-create all 9 areas when it detects segmented_vision
        logger.info(f"✅ Registered segmented vision for group_id={group_id} (encoding={self.encoding}, position={self.position})")
        logger.info(f"   FrameChangeHandling: {frame_change_handling} (will generate 9 cortical IDs for segments)")
        logger.info(f"   FEAGI will auto-create all required cortical areas during agent registration")
    
    def _write_to_cache(self, cache):
        """Write current frame to Rust IOCache"""
        import logging
        logger = logging.getLogger(__name__)
        
        if self._current_frame is None:
            logger.warning("⚠️ [CAMERA] No frame to write yet (_current_frame is None)")
            return  # No frame to write yet
        
        try:
            import feagi_rust_py_libs as frpl
            
            logger.info(f"📤 [CAMERA] Writing frame to cache: shape={self._current_frame.shape}, dtype={self._current_frame.dtype}, group_id={self.group_id}, channel={self.channel}, cortical_area={getattr(self, '_cortical_area', 'N/A')}")
            
            # Convert NumPy to Rust ImageFrame using correct API
            # OpenCV/NumPy arrays are in (height, width, channels) format
            frame = frpl.connector_core.data_types.ImageFrame.new_from_array(
                self._current_frame,
                frpl.connector_core.data_types.descriptors.ColorSpace.Linear,
                frpl.connector_core.data_types.descriptors.MemoryOrderLayout.HeightsWidthsChannels
            )
            logger.info("📤 [CAMERA] ✅ Converted NumPy frame to Rust ImageFrame")
            
            # Use sensor_segmented_vision_write since we registered with sensor_segmented_vision_register
            method_name = "sensor_segmented_vision_write"
            
            logger.info(f"📤 [CAMERA] Using Rust method: {method_name}")
            
            # Get method
            if not hasattr(cache, method_name):
                raise AttributeError(
                    f"ConnectorAgent missing write method: {method_name}. "
                    f"This should exist since we registered with sensor_segmented_vision_register."
                )
            write_method = getattr(cache, method_name)
            
            # Write
            logger.info(f"📤 [CAMERA] Calling {method_name}(group={self.group_id}, channel_index={self.channel})...")
            write_method(
                group=self.group_id,
                channel_index=self.channel,
                data=frame
            )
            logger.info(f"📤 [CAMERA] ✅ Successfully wrote frame to cache (group={self.group_id}, channel_index={self.channel})")
        except ImportError as e:
            logger.error("❌ [CAMERA] feagi_rust_py_libs import failed", exc_info=True)
            raise ImportError("feagi_rust_py_libs required") from e
        except Exception as e:
            logger.error(f"❌ [CAMERA] Error writing frame to cache: {e}", exc_info=True)
            raise

