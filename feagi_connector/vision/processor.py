"""
Vision processing wrappers built on feagi-rust-py-libs.

Abstractions here reduce boilerplate in agents by handling:
- Image properties setup (resolution, color space, memory order)
- Segmented camera registration (center + peripheral)
- Frame conversion (BGR->RGB uint8) and storage/encoding lifecycle
"""

from __future__ import annotations

from typing import Optional, Tuple
import logging

import numpy as np
from feagi_connector.cache.sensor_cache import SensorCache
from feagi_connector.agent_logging.diagnostics import log_sensor_area_counts


# Module logger for detailed vision diagnostics
logger = logging.getLogger("feagi_connector.vision")


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
        modulation: Tuple[float, float] = (0.2, 0.2),
        number_of_channels: int = 1,
    ) -> None:
        import feagi_rust_py_libs as frpl  # local import to keep module light

        self._frpl = frpl
        self.group_index = int(cortical_group_index)
        self.center_dims = (int(center_dims[0]), int(center_dims[1]))
        self.per_dims = (int(peripheral_dims[0]), int(peripheral_dims[1]))
        self.number_of_channels = int(number_of_channels)

        # Store gaze parameters (eccentricity and modulation)
        self._eccentricity_params = eccentricity
        self._modulation_params = modulation

        # In-process Rust-backed sensor cache for encoding
        self._cache = SensorCache()
        self._image_properties = None
        self._seg_props = None

        # Establish base input properties with placeholder; updated on first frame
        # Try new API path first, fallback to old
        # NOTE: Use WidthsHeightsChannels as per segmented_autogaze.py sample
        try:
            color_space = self._frpl.connector_core.data.descriptors.ColorSpace.Linear
            color_channels = self._frpl.connector_core.data.descriptors.ColorChannelLayout.RGB
            self._memory_order = (
                self._frpl.connector_core.data.descriptors.MemoryOrderLayout.WidthsHeightsChannels
            )
        except AttributeError:
            # Fallback to old API path
            color_space = self._frpl.data_structures.data.image_descriptors.ColorSpace.Linear
            color_channels = self._frpl.data_structures.data.image_descriptors.ColorChannelLayout.RGB
            self._memory_order = (
                self._frpl.data_structures.data.image_descriptors.MemoryOrderLayout.WidthsHeightsChannels
            )

        # Segmented properties (output sizing)
        cw, ch = self.center_dims
        pw, ph = self.per_dims
        try:
            # Try new API path
            out_center = self._frpl.connector_core.data.descriptors.ImageXYResolution(cw, ch)
            out_per = self._frpl.connector_core.data.descriptors.ImageXYResolution(pw, ph)
            seg_res = self._frpl.connector_core.data.descriptors.SegmentedXYImageResolutions.create_with_same_sized_peripheral(
                out_center, out_per
            )
            self._seg_props = self._frpl.connector_core.data.descriptors.SegmentedImageFrameProperties(
                seg_res, color_channels, color_channels, color_space
            )
        except AttributeError:
            # Fallback to old API path
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
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(
                f"SegmentedVisionProcessor init: group={self.group_index}, "
                f"center={self.center_dims}, per={self.per_dims}, channels={self.number_of_channels}, "
                f"ecc={self._eccentricity_params}, mod={self._modulation_params}"
            )
        
    def _setup_gaze_properties(self) -> None:
        """Setup gaze properties using the advanced pattern from feagi_rust_py_libs."""
        try:
            eccentricity = self._frpl.connector_core.data.Percentage2D(
                self._frpl.connector_core.data.Percentage.new_from_0_1(self._eccentricity_params[0]),
                self._frpl.connector_core.data.Percentage.new_from_0_1(self._eccentricity_params[1]),
            )
            modulation = self._frpl.connector_core.data.Percentage2D(
                self._frpl.connector_core.data.Percentage.new_from_0_1(self._modulation_params[0]),
                self._frpl.connector_core.data.Percentage.new_from_0_1(self._modulation_params[1]),
            )
        except AttributeError:
            try:
                # Try old data_structures API path
                eccentricity = self._frpl.data_structures.data.Percentage2D(
                    self._frpl.data_structures.data.Percentage.new_from_0_1(self._eccentricity_params[0]),
                    self._frpl.data_structures.data.Percentage.new_from_0_1(self._eccentricity_params[1]),
                )
                modulation = self._frpl.data_structures.data.Percentage2D(
                    self._frpl.data_structures.data.Percentage.new_from_0_1(self._modulation_params[0]),
                    self._frpl.data_structures.data.Percentage.new_from_0_1(self._modulation_params[1]),
                )
            except AttributeError:
                # Final fallback to simple gaze properties
                try:
                    self._gaze = self._frpl.connector_core.data.descriptors.GazeProperties.create_default_centered()
                except AttributeError:
                    self._gaze = self._frpl.data_structures.data.image_descriptors.GazeProperties.create_default_centered()
                return
        
        # Create the advanced gaze properties object
        try:
            # Try new API path
            self._gaze = self._frpl.connector_core.data.descriptors.GazeProperties(
                eccentricity, modulation
            )
        except AttributeError:
            # Try old API path
            self._gaze = self._frpl.data_structures.data.image_descriptors.GazeProperties(
                eccentricity, modulation
            )
        
        if logger.isEnabledFor(logging.DEBUG):
            try:
                ex = float(self._eccentricity_params[0]); ey = float(self._eccentricity_params[1])
                mx = float(self._modulation_params[0]); my = float(self._modulation_params[1])
                logger.debug(f"GazeProperties set: eccentricity=({ex:.3f},{ey:.3f}), modulation=({mx:.3f},{my:.3f})")
            except Exception:
                pass

    def _ensure_registered(self, width: int, height: int) -> None:
        if self._image_properties is None:
            try:
                # Try new API path
                cs = self._frpl.connector_core.data.descriptors.ColorSpace.Linear
                cc = self._frpl.connector_core.data.descriptors.ColorChannelLayout.RGB
                in_res = self._frpl.connector_core.data.descriptors.ImageXYResolution(int(width), int(height))
                self._image_properties = self._frpl.connector_core.data.descriptors.ImageFrameProperties(in_res, cs, cc)
            except AttributeError:
                # Fallback to old API path
                cs = self._frpl.data_structures.data.image_descriptors.ColorSpace.Linear
                cc = self._frpl.data_structures.data.image_descriptors.ColorChannelLayout.RGB
                in_res = self._frpl.data_structures.data.image_descriptors.ImageXYResolution(int(width), int(height))
                self._image_properties = self._frpl.data_structures.data.image_descriptors.ImageFrameProperties(in_res, cs, cc)
            
            # Use image_camera_with_peripheral as shown in the sample
            try:
                self._cache.image_camera_with_peripheral.register(
                    self.group_index, self.number_of_channels, self._image_properties, self._seg_props, self._gaze
                )
                if logger.isEnabledFor(logging.DEBUG):
                    logger.debug(
                        f"Registered segmented camera: in=({width}x{height}), center={self.center_dims}, per={self.per_dims}"
                    )
            except Exception as e:
                # Idempotency guard: ignore duplicate registration errors
                msg = str(e).lower()
                if "already" in msg and "register" in msg:
                    if logger.isEnabledFor(logging.DEBUG):
                        logger.debug("Segmented camera already registered; continuing")
                else:
                    raise

    def process_frame(self, frame_bgr: np.ndarray, resize_to: Optional[Tuple[int, int]] = None) -> bytes:
        import feagi_rust_py_libs as frpl  # ensure availability inside method

        # Convert to RGB and resize if requested (resize is left to caller if they use cv2 elsewhere)
        rgb = bgr_to_rgb_uint8(frame_bgr)
        
        # CRITICAL: Flip image vertically to convert from image coordinates (Y=0 top) 
        # to FEAGI coordinates (Y=0 bottom). The Rust encoder doesn't do this automatically.
        rgb = np.flipud(rgb)
        
        h, w = rgb.shape[:2]
        self._ensure_registered(w, h)

        # Use new API path
        try:
            color_space = frpl.connector_core.data.descriptors.ColorSpace.Linear
        except AttributeError:
            color_space = frpl.data_structures.data.image_descriptors.ColorSpace.Linear
        
        memory_order = self._memory_order
        
        # Create ImageFrame with new API
        try:
            image_frame = frpl.connector_core.data.ImageFrame.new_from_array(rgb, color_space, memory_order)
        except AttributeError:
            image_frame = frpl.data_structures.data.ImageFrame.new_from_array(rgb, color_space, memory_order)
        
        # Use image_camera_with_peripheral.write as shown in the sample
        self._cache.image_camera_with_peripheral.write(self.group_index, 0, image_frame)
        
        self._cache.encode_cached_data_into_bytes()
        
        # Use sensor_get_byte_container() then extract bytes
        byte_container = self._cache.sensor_get_byte_container()
        sensor_bytes = byte_container.copy_out_as_byte_vector()
        
        # Optional compact diagnostics when DEBUG is enabled
        try:
            log_sensor_area_counts(logger, sensor_bytes)
        except Exception:
            pass
        return sensor_bytes

    def update_eccentricity(self, ecc_x: float, ecc_y: float) -> None:
        """Update eccentricity (central region size) dynamically.
        
        Args:
            ecc_x: Horizontal eccentricity (0.0 to 1.0)
            ecc_y: Vertical eccentricity (0.0 to 1.0)
        """
        self._eccentricity_params = (
            max(0.0, min(1.0, float(ecc_x))),
            max(0.0, min(1.0, float(ecc_y)))
        )
        self._setup_gaze_properties()

    def update_modulation(self, mod_x: float, mod_y: float) -> None:
        """Update modulation parameters dynamically.
        
        Args:
            mod_x: Horizontal modulation (0.0 to 1.0)
            mod_y: Vertical modulation (0.0 to 1.0)
        """
        self._modulation_params = (
            max(0.0, min(1.0, float(mod_x))),
            max(0.0, min(1.0, float(mod_y)))
        )
        self._setup_gaze_properties()

    @property
    def eccentricity(self) -> Tuple[float, float]:
        """Get current eccentricity (central region sizing)."""
        return self._eccentricity_params

    @property
    def modulation(self) -> Tuple[float, float]:
        """Get current modulation (peripheral tiling parameters)."""
        return self._modulation_params

    @property
    def gaze_properties(self):
        """Get the gaze properties object for motor integration."""
        return self._gaze
    
    def update_stage_properties(self, input_image_properties, segment_image_properties):
        """Update stage properties dynamically, as shown in segmented_autogaze.py sample.
        
        This allows updating the segmentation pipeline properties at runtime,
        for example to change gaze parameters based on motor feedback.
        
        Args:
            input_image_properties: ImageFrameProperties for input
            segment_image_properties: SegmentedImageFrameProperties for output
        """
        try:
            # Create stage properties using the new API
            stage_props = self._frpl.connector_core.data_pipeline.stage_properties.ImageSegmentorStageProperties(
                input_image_properties, segment_image_properties, self._gaze
            )
        except AttributeError:
            # API might not have stage_properties, skip
            logger.warning("Stage properties update not available in current feagi_rust_py_libs version")
            return
        
        # Update the stage (stage index 0 for the segmentor stage)
        try:
            self._cache.image_camera_with_peripheral.update_stage(
                self.group_index, 0, 0, stage_props
            )
            if logger.isEnabledFor(logging.DEBUG):
                logger.debug(f"Updated stage properties for group {self.group_index}")
        except Exception as e:
            logger.warning(f"Failed to update stage properties: {e}")



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
        neurons = self._frpl.data_structures.neurons_voxels.xyzp.NeuronVoxelXYZPArrays()
        
        # Convert gaze position to coordinates
        x_coord = int(gaze_x * self.gaze_resolution)
        y_coord = int(gaze_y * self.gaze_resolution)
        
        # Create multiple neurons for smooth control (like the sample)
        for i in range(4):  # Sample shows 4 neurons
            neuron = self._frpl.data_structures.neurons_voxels.xyzp.NeuronVoxelXYZP(x_coord, y_coord, 0, intensity)
            neurons.push(neuron)
        
        # Create mapped neurons
        mapped_neurons = self._frpl.data_structures.neurons_voxels.xyzp.CorticalMappedXYZPNeuronVoxels()
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
    
    def process_motor_bytes(self, motor_bytes: bytes) -> Optional[Tuple[float, float, float, float]]:
        """Process motor bytes to extract eccentricity/modulation commands.
        
        This implementation expects the OGaze motor cortical area (e.g., 'ogaz00')
        to encode parameter values across the Z dimension (0..N-1 buckets) with
        potentials as weights, and the X coordinate selecting which parameter:
        X=0 -> eccentricity.x, X=1 -> eccentricity.y, X=2 -> modulation.x, X=3 -> modulation.y.
        
        Args:
            motor_bytes: Raw motor data bytes from FEAGI
            
        Returns:
            Tuple of (ecc_x, ecc_y, mod_x, mod_y) with values in [0,1], or None
            if parameters could not be decoded.
        """
        try:
            # Try new API first (FeagiByteContainer), return None if not available
            if not hasattr(self._frpl.data_serialization, 'FeagiByteContainer'):
                return None
            if not motor_bytes or len(motor_bytes) == 0:
                return None
            try:
                feagi_byte_structure = self._frpl.data_serialization.FeagiByteContainer()
                feagi_byte_structure.load_bytes_and_verify(motor_bytes)
                mapped_neurons = feagi_byte_structure.try_create_new_struct_from_index(0)
            except (KeyboardInterrupt, SystemExit):
                raise
            except:
                import logging
                import sys
                logging.debug(f"Failed to decode motor bytes: {sys.exc_info()[1]}")
                return None

            # Accumulators per param index
            # index -> (weighted_z_sum, activation_sum, z_max)
            accumulators: dict[int, Tuple[float, float, int]] = {}

            def _cid_to_str(cid_obj) -> str:
                try:
                    return str(cid_obj.as_ascii_string())
                except Exception:
                    return str(cid_obj)

            for cid_obj, neuron_arrays in mapped_neurons.iter_full():
                try:
                    cid_str = _cid_to_str(cid_obj).lower()
                    if "ogaz" not in cid_str:
                        # Ignore other motor areas
                        continue
                    x_coords, y_coords, z_coords, potentials = neuron_arrays
                    # Determine z resolution from max z observed
                    try:
                        z_max_val = int(max(z_coords)) if len(z_coords) > 0 else 0
                    except Exception:
                        z_max_val = 0
                    for x, y, z, p in zip(x_coords, y_coords, z_coords, potentials):
                        try:
                            xi = int(x)
                            zi = int(z)
                            pi = float(p)
                        except Exception:
                            continue
                        wsum, asum, zmax = accumulators.get(xi, (0.0, 0.0, 0))
                        wsum += zi * pi
                        asum += pi
                        zmax = max(zmax, z_max_val)
                        accumulators[xi] = (wsum, asum, zmax)
                except Exception:
                    continue

            def _normalize(accum: Tuple[float, float, int]) -> Optional[float]:
                wsum, asum, zmax = accum
                if asum <= 0.0:
                    return None
                denom = float(max(1, zmax))
                val = (wsum / asum) / denom
                return max(0.0, min(1.0, float(val)))

            ecc_x = _normalize(accumulators.get(0, (0.0, 0.0, 0))) if 0 in accumulators else None
            ecc_y = _normalize(accumulators.get(1, (0.0, 0.0, 0))) if 1 in accumulators else None
            mod_x = _normalize(accumulators.get(2, (0.0, 0.0, 0))) if 2 in accumulators else None
            mod_y = _normalize(accumulators.get(3, (0.0, 0.0, 0))) if 3 in accumulators else None

            if any(v is not None for v in (ecc_x, ecc_y, mod_x, mod_y)):
                # Fill missing with existing stored params (kept by caller) or defaults here as None
                # We return None for missing and let caller selectively apply
                # To keep type consistent, replace None with -1 sentinel and let caller check
                def _val_or_sentinel(v):
                    return float(v) if v is not None else -1.0
                return (
                    _val_or_sentinel(ecc_x),
                    _val_or_sentinel(ecc_y),
                    _val_or_sentinel(mod_x),
                    _val_or_sentinel(mod_y),
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
    
    # Try new API path first
    try:
        color_space = frpl.connector_core.data.descriptors.ColorSpace.Linear
        memory_order = frpl.connector_core.data.descriptors.MemoryOrderLayout.WidthsHeightsChannels
        return frpl.connector_core.data.ImageFrame.new_from_array(np_rgb_uint8, color_space, memory_order)
    except AttributeError:
        # Fallback to old API path
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
    
    # Try new API path first
    try:
        neurons = frpl.connector_core.data.neurons.NeuronXYZPArrays()
    except AttributeError:
        neurons = frpl.data_structures.neurons_voxels.xyzp.NeuronVoxelXYZPArrays()
    
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
            
            # Try new API path
            try:
                neuron = frpl.connector_core.data.neurons.NeuronXYZP(
                    nx, ny, 0, intensity * weight
                )
            except AttributeError:
                neuron = frpl.data_structures.neurons_voxels.xyzp.NeuronVoxelXYZP(
                    nx, ny, 0, intensity * weight
                )
            neurons.push(neuron)
    
    return neurons

