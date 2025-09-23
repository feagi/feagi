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
    """High-level segmented vision pipeline using feagi-rust-py-libs with gaze control.

    Usage:
        - Construct with FEAGI-derived center/peripheral dimensions
        - Call process_frame(BGR frame) to get encoded neuron bytes
        - Use update_gaze() to dynamically control visual attention
    """

    def __init__(
        self,
        cortical_group_index: int,
        center_dims: Tuple[int, int],
        peripheral_dims: Tuple[int, int],
        eccentricity: Tuple[float, float] = (0.2, 0.2),
        modularity: Tuple[float, float] = (0.2, 0.2),
        gaze_position: Tuple[float, float] = (0.5, 0.5),
        number_of_channels: int = 1,
    ) -> None:
        import feagi_rust_py_libs as frpl  # local import to keep module light

        self._frpl = frpl
        self.group_index = int(cortical_group_index)
        self.center_dims = (int(center_dims[0]), int(center_dims[1]))
        self.per_dims = (int(peripheral_dims[0]), int(peripheral_dims[1]))
        self.number_of_channels = int(number_of_channels)

        # Store gaze parameters
        self._gaze_position = gaze_position
        self._eccentricity_params = eccentricity
        self._modularity_params = modularity

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

        # Setup advanced gaze properties as shown in the sample
        self._setup_gaze_properties()

        # Registration happens on first process call when input resolution is known
        
    def _setup_gaze_properties(self) -> None:
        """Setup gaze properties using the advanced pattern from feagi_rust_py_libs."""
        try:
            # Try the current API path
            eccentricity = self._frpl.data_structures.data.Percentage2D(
                self._frpl.data_structures.data.Percentage.new_from_0_1(self._eccentricity_params[0]),
                self._frpl.data_structures.data.Percentage.new_from_0_1(self._eccentricity_params[1]),
            )
            modularity = self._frpl.data_structures.data.Percentage2D(
                self._frpl.data_structures.data.Percentage.new_from_0_1(self._modularity_params[0]),
                self._frpl.data_structures.data.Percentage.new_from_0_1(self._modularity_params[1]),
            )
        except AttributeError:
            try:
                # Try alternative API paths
                eccentricity = self._frpl.data_structures.Percentage2D(
                    self._frpl.data_structures.Percentage.new_from_0_1(self._eccentricity_params[0]),
                    self._frpl.data_structures.Percentage.new_from_0_1(self._eccentricity_params[1]),
                )
                modularity = self._frpl.data_structures.Percentage2D(
                    self._frpl.data_structures.Percentage.new_from_0_1(self._modularity_params[0]),
                    self._frpl.data_structures.Percentage.new_from_0_1(self._modularity_params[1]),
                )
            except AttributeError:
                # Fallback to simple gaze properties
                self._gaze = self._frpl.data_structures.data.image_descriptors.GazeProperties.create_default_centered()
                return
        
        # Create the advanced gaze properties object
        self._gaze = self._frpl.data_structures.data.image_descriptors.GazeProperties(
            eccentricity, modularity
        )

    def _ensure_registered(self, width: int, height: int) -> None:
        if self._image_properties is None:
            cs = self._frpl.data_structures.data.image_descriptors.ColorSpace.Linear
            cc = self._frpl.data_structures.data.image_descriptors.ColorChannelLayout.RGB
            in_res = self._frpl.data_structures.data.image_descriptors.ImageXYResolution(int(width), int(height))
            self._image_properties = self._frpl.data_structures.data.image_descriptors.ImageFrameProperties(in_res, cs, cc)
            
            # Use image_camera_with_peripheral as shown in the sample
            self._cache.image_camera_with_peripheral.register(
                self.group_index, self.number_of_channels, self._image_properties, self._seg_props, self._gaze
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
        
        # Use image_camera_with_peripheral.store as shown in the sample
        self._cache.image_camera_with_peripheral.store(self.group_index, 0, image_frame)
        self._cache.encode_cached_data_into_bytes()
        return self._cache.get_most_recent_sensor_bytes()

    def update_gaze(self, gaze_x: float, gaze_y: float) -> None:
        """Update the gaze position dynamically.
        
        Args:
            gaze_x: Gaze X position (0.0 to 1.0)
            gaze_y: Gaze Y position (0.0 to 1.0)
        """
        self._gaze_position = (
            max(0.0, min(1.0, gaze_x)),
            max(0.0, min(1.0, gaze_y))
        )
        # Force re-registration with new gaze properties
        self._image_properties = None
        self._setup_gaze_properties()

    @property
    def gaze(self) -> Tuple[float, float]:
        """Get current gaze position."""
        return self._gaze_position
    
    @property
    def gaze_properties(self):
        """Get the gaze properties object for motor integration."""
        return self._gaze



class GazeMotorProcessor:
    """Processor for gaze motor control using FEAGI motor outputs.
    
    Integrates with SegmentedVisionProcessor to provide dynamic gaze control
    based on motor neuron activity from FEAGI.
    """
    
    def __init__(self, cortical_group_index: int, num_channels: int = 10, gaze_resolution: int = 8):
        """Initialize gaze motor processor.
        
        Args:
            cortical_group_index: Cortical group for gaze motors
            num_channels: Number of motor channels 
            gaze_resolution: Resolution for gaze position mapping
        """
        import feagi_rust_py_libs as frpl
        
        self._frpl = frpl
        self.group_index = int(cortical_group_index)
        self.num_channels = int(num_channels)
        self.gaze_resolution = int(gaze_resolution)
        
        self._cache = SensorCache()
        self._registered = False
    
    def register_gaze_motor(self) -> None:
        """Register gaze motor with the system."""
        if not self._registered:
            try:
                # Create the cortical channel count
                num_channels = self._frpl.data_structures.genomic.CorticalChannelCount(self.num_channels)
            except AttributeError:
                # API might have changed, use integer directly
                num_channels = self.num_channels
            
            # Register gaze motor (note: this requires motor cache which isn't available here)
            # This is placeholder - actual registration happens at agent level
            self._registered = True
    
    def create_gaze_neurons(self, gaze_x: float, gaze_y: float, intensity: float = 1.0):
        """Create neuron data for gaze control as shown in the sample.
        
        Args:
            gaze_x: Gaze X position (0.0 to 1.0)
            gaze_y: Gaze Y position (0.0 to 1.0)
            intensity: Neuron firing intensity (0.0 to 1.0)
            
        Returns:
            Mapped neuron data for gaze control
        """
        # Create neurons as shown in the sample
        neurons = self._frpl.data_structures.neurons.xyzp.NeuronXYZPArrays()
        
        # Convert gaze position to coordinates
        x_coord = int(gaze_x * self.gaze_resolution)
        y_coord = int(gaze_y * self.gaze_resolution)
        
        # Create multiple neurons for smooth control (like the sample)
        for i in range(4):  # Sample shows 4 neurons
            neuron = self._frpl.data_structures.neurons.xyzp.NeuronXYZP(x_coord, y_coord, 0, intensity)
            neurons.push(neuron)
        
        # Create mapped neurons
        mapped_neurons = self._frpl.data_structures.neurons.xyzp.CorticalMappedXYZPNeuronData()
        try:
            gaze_cortical_id = self._frpl.data_structures.genomic.CorticalID.new_motor_cortical_area_id(
                self._frpl.data_structures.genomic.MotorCorticalType.Gaze, self.group_index
            )
        except AttributeError:
            # API might have changed, create cortical ID differently
            try:
                gaze_cortical_id = self._frpl.data_structures.genomic.CorticalID.new_from_string(f"gaze{self.group_index}")
            except:
                # Final fallback - create a simple ID
                return neurons
        
        mapped_neurons.insert(gaze_cortical_id, neurons)
        
        return mapped_neurons
    
    def process_motor_bytes(self, motor_bytes: bytes) -> Optional[Tuple[float, float]]:
        """Process motor bytes to extract gaze commands.
        
        Args:
            motor_bytes: Raw motor data bytes from FEAGI
            
        Returns:
            Tuple of (gaze_x, gaze_y) or None if processing failed
        """
        try:
            # Read bytes into motor system (as shown in sample)
            # This would integrate with the agent connector's motor system
            feagi_byte_structure = self._frpl.data_serialization.FeagiByteStructure(motor_bytes)
            mapped_neurons = self._frpl.data_structures.neurons.xyzp.CorticalMappedXYZPNeuronData.new_from_feagi_byte_structure(feagi_byte_structure)
            
            # Extract gaze data - simplified implementation
            total_x = 0.0
            total_y = 0.0 
            total_activation = 0.0
            
            for cid_obj, neuron_arrays in mapped_neurons.iter_full():
                try:
                    x_coords, y_coords, z_coords, potentials = neuron_arrays
                    for x, y, z, p in zip(x_coords, y_coords, z_coords, potentials):
                        # Normalize coordinates to 0-1 range
                        gaze_x = float(x) / self.gaze_resolution
                        gaze_y = float(y) / self.gaze_resolution
                        total_x += gaze_x * p
                        total_y += gaze_y * p
                        total_activation += p
                except Exception:
                    continue
            
            if total_activation > 0:
                return (
                    max(0.0, min(1.0, total_x / total_activation)),
                    max(0.0, min(1.0, total_y / total_activation))
                )
                
            return None
            
        except Exception:
            return None


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


def create_gaze_control_neurons(gaze_x: float, gaze_y: float, intensity: float = 1.0, resolution: int = 8):
    """Create neuron data for gaze control as utility function.
    
    Args:
        gaze_x: Gaze X position (0.0 to 1.0)
        gaze_y: Gaze Y position (0.0 to 1.0)
        intensity: Neuron firing intensity (0.0 to 1.0)
        resolution: Resolution for gaze position mapping
        
    Returns:
        NeuronXYZPArrays object for gaze control
    """
    import feagi_rust_py_libs as frpl
    
    neurons = frpl.data_structures.neurons.xyzp.NeuronXYZPArrays()
    
    # Convert gaze position to coordinates
    x_coord = int(gaze_x * resolution)
    y_coord = int(gaze_y * resolution)
    
    # Create multiple neurons for smoother control 
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            nx = max(0, min(resolution-1, x_coord + dx))
            ny = max(0, min(resolution-1, y_coord + dy))
            
            # Weight based on distance from center
            weight = 1.0 if (dx == 0 and dy == 0) else 0.5
            
            neuron = frpl.data_structures.neurons.xyzp.NeuronXYZP(
                nx, ny, 0, intensity * weight
            )
            neurons.push(neuron)
    
    return neurons

