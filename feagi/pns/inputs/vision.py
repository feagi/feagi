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
        position: CameraPosition = "center",
        central_resolution: Optional[Tuple[int, int]] = None,
        peripheral_resolution: Optional[Tuple[int, int]] = None,
    ):
        super().__init__()
        self.resolution = resolution
        self.encoding = encoding
        self.position = position
        self.channel = 0  # Default channel
        self.central_resolution = central_resolution  # For segmented vision: center segment dimensions
        self.peripheral_resolution = peripheral_resolution  # For segmented vision: peripheral segment dimensions
        
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
        group_id: Optional[int] = None,
        central_resolution: Optional[Tuple[int, int]] = None,
        peripheral_resolution: Optional[Tuple[int, int]] = None,
    ) -> 'Camera':
        """
        Register a new camera input.
        
        Args:
            resolution: (width, height) in pixels - input frame size
            encoding: "absolute" (full frame) or "incremental" (changes only)
            position: Camera position for segmented vision
            group_id: Optional cortical group ID. If None, auto-allocates.
            central_resolution: Optional (width, height) for segmented vision center segment
            peripheral_resolution: Optional (width, height) for segmented vision peripheral segments
        
        Returns:
            Camera instance
        """
        from feagi.pns import brain_input
        
        camera = cls(resolution, encoding, position, central_resolution, peripheral_resolution)
        brain_input.register_input(camera, group_id=group_id)
        return camera
    
    def set_frame(self, frame):
        """
        Set the current camera frame.
        
        Automatically resizes frame to match registered resolution if needed.
        
        Args:
            frame: NumPy array with shape (H, W, 3) for RGB images
        """
        import numpy as np
        
        if not isinstance(frame, np.ndarray):
            raise TypeError(f"Frame must be a NumPy array, got {type(frame)}")
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError(f"Frame must be (H, W, 3), got shape {frame.shape}")
        
        # Validate and resize frame to match registered resolution
        frame_height, frame_width = frame.shape[:2]
        expected_width, expected_height = self.resolution
        
        if frame_width != expected_width or frame_height != expected_height:
            # Resize frame to match registered resolution (required for Rust validation)
            try:
                from PIL import Image
                # Convert NumPy array to PIL Image (H, W, 3) -> PIL expects (W, H)
                pil_image = Image.fromarray(frame)
                # Resize to expected resolution
                pil_image = pil_image.resize((expected_width, expected_height), Image.Resampling.LANCZOS)
                # Convert back to NumPy array
                resized_frame = np.asarray(pil_image, dtype=np.uint8)
                logger.debug(
                    f"📤 [CAMERA] Resized frame from {frame_width}x{frame_height} to {expected_width}x{expected_height}"
                )
                self._current_frame = resized_frame
            except ImportError:
                # Fallback: use OpenCV if available
                try:
                    import cv2
                    resized_frame = cv2.resize(frame, (expected_width, expected_height), interpolation=cv2.INTER_LINEAR)
                    logger.debug(
                        f"📤 [CAMERA] Resized frame from {frame_width}x{frame_height} to {expected_width}x{expected_height} (using OpenCV)"
                    )
                    self._current_frame = resized_frame
                except ImportError:
                    raise ValueError(
                        f"Frame resolution mismatch: got {frame_width}x{frame_height}, "
                        f"expected {expected_width}x{expected_height}. "
                        f"Install PIL (Pillow) or OpenCV for auto-resizing."
                    )
        else:
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
        
        # Use provided resolutions if available, otherwise fall back to FEAGI unit topology defaults
        # Defaults match sensor_cortical_units.rs SegmentedVision unit_default_topology:
        # - Center segment (index 4): channel_dimensions_default: [128, 128, 3] -> 128x128
        # - Peripheral segments (0-3, 5-8): channel_dimensions_default: [32, 32, 1] -> 32x32
        if self.central_resolution:
            center_width, center_height = self.central_resolution
            logger.info(f"📐 Using provided central resolution: {center_width}x{center_height}")
        else:
            center_width, center_height = 128, 128  # From unit topology segment 4 default
            logger.info(f"📐 Using default central resolution: {center_width}x{center_height}")
        
        if self.peripheral_resolution:
            peripheral_width, peripheral_height = self.peripheral_resolution
            logger.info(f"📐 Using provided peripheral resolution: {peripheral_width}x{peripheral_height}")
        else:
            peripheral_width, peripheral_height = 32, 32  # From unit topology segments 0-3, 5-8 default
            logger.info(f"📐 Using default peripheral resolution: {peripheral_width}x{peripheral_height}")
        
        # Create resolutions using correct dimensions
        center_res = cc_desc.ImageXYResolution(center_width, center_height)
        peripheral_res = cc_desc.ImageXYResolution(peripheral_width, peripheral_height)
        
        # Create segmented resolution using create_with_same_sized_peripheral
        segmented_res = cc_desc.SegmentedXYImageResolutions.create_with_same_sized_peripheral(
            center_res,
            peripheral_res
        )
        
        logger.info(f"📐 Segmented vision registration: center={center_width}x{center_height}, peripheral={peripheral_width}x{peripheral_height}")
        
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
        import time
        logger = logging.getLogger(__name__)
        
        if self._current_frame is None:
            logger.debug("⚠️ [CAMERA] No frame to write yet (_current_frame is None)")
            return  # No frame to write yet
        
        t_total_start = time.perf_counter()
        try:
            import feagi_rust_py_libs as frpl
            
            # Convert NumPy to Rust ImageFrame (optimized for performance)
            # NumPy arrays are in (height, width, channels) format
            # This is a zero-copy operation where possible (Rust borrows NumPy memory)
            t_convert_start = time.perf_counter()
            frame = frpl.connector_core.data_types.ImageFrame.new_from_array(
                self._current_frame,
                frpl.connector_core.data_types.descriptors.ColorSpace.Linear,
                frpl.connector_core.data_types.descriptors.MemoryOrderLayout.HeightsWidthsChannels
            )
            t_convert = (time.perf_counter() - t_convert_start) * 1000
            
            # Use sensor_segmented_vision_write (registered during setup)
            method_name = "sensor_segmented_vision_write"
            write_method = getattr(cache, method_name)
            
            # Write frame to Rust cache (all processing happens in Rust)
            t_write_start = time.perf_counter()
            write_method(
                group=self.group_id,
                channel_index=self.channel,
                data=frame
            )
            t_write = (time.perf_counter() - t_write_start) * 1000
            
            # Performance logging removed for hot path
        except ImportError as e:
            logger.error("❌ [CAMERA] feagi_rust_py_libs import failed", exc_info=True)
            raise ImportError("feagi_rust_py_libs required") from e
        except Exception as e:
            logger.error(f"❌ [CAMERA] Error writing frame to cache: {e}", exc_info=True)
            raise

