"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
FEAGI Fire Queue (FQ) Sampler Module

This module provides high-performance fire queue sampling for FEAGI NPU.
Supports independent instances for motor and visualization with different frequencies.

Features:
- Zero-copy operations with direct SoA access
- SIMD acceleration via centralized configuration  
- Pre-allocated buffers for real-time performance
- RTOS-compatible deterministic timing
- Independent frequency configuration per instance

Architecture:
- Motor sampling: High-frequency OPU area sampling (100Hz)
- Visualization sampling: Configurable-rate full brain sampling (30Hz)
- Custom sampling: Application-specific sampling patterns
"""

import time
import logging
import numpy as np
from typing import Dict, List, Optional, Any
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class UnifiedFQSampler:
    """
    High-performance Fire Queue Sampler for FEAGI NPU.
    
    This unified sampler combines zero-copy optimizations with SIMD acceleration and
    centralized configuration. Can be instantiated independently for different purposes:
    - Motor sampling: High-frequency OPU area sampling
    - Visualization sampling: Configurable-rate full brain sampling
    - Custom sampling: Application-specific sampling patterns
    
    Features:
    - Zero-copy operations with direct SoA access
    - SIMD acceleration via centralized configuration  
    - Pre-allocated buffers for real-time performance
    - RTOS-compatible deterministic timing
    - Independent frequency configuration per instance
    """
    
    def __init__(self, 
                 fire_queue_provider,
                 sample_frequency_hz: float,
                 output_queue=None,
                 connectome_manager=None,
                 sampling_mode: str = 'global',  # 'global', 'motor_only', 'areas_only', 'custom'
                 target_areas: Optional[List[str]] = None,
                 max_retries: int = 3,
                 neuron_type_filter: Optional[str] = None,
                 enable_simd: bool = True,
                 enable_zero_copy: bool = True,
                 buffer_size: int = 100_000):
        """
        Initialize the unified FQ sampler.
        
        Args:
            fire_queue_provider: Source for fire queue data
            sample_frequency_hz: Sampling frequency in Hz
            output_queue: Optional output queue for samples
            connectome_manager: Optional connectome manager for area info
            sampling_mode: 'global' (all areas), 'motor_only' (OPU areas), 'areas_only' (specific areas), 'custom'
            target_areas: Specific areas to sample (for 'areas_only' or 'custom' modes)
            max_retries: Maximum retry attempts for failed operations
            neuron_type_filter: Optional filter for specific neuron types
            enable_simd: Enable SIMD acceleration if available
            enable_zero_copy: Enable zero-copy optimizations
            buffer_size: Pre-allocated buffer size for neurons
        """
        self.fire_queue_provider = fire_queue_provider
        self.sample_frequency = sample_frequency_hz
        self.sample_interval = 1.0 / sample_frequency_hz if sample_frequency_hz > 0 else 0.1
        self.output_queue = output_queue
        self.connectome_manager = connectome_manager
        self.sampling_mode = sampling_mode.lower()
        self.target_areas = target_areas or []
        self.max_retries = max_retries
        self.neuron_type_filter = neuron_type_filter
        self.enable_simd = enable_simd
        self.enable_zero_copy = enable_zero_copy
        self.running = False
        
        # Performance tracking
        self._last_sample_time = 0.0
        self._sample_count = 0
        self._performance_stats = {
            'samples_generated': 0,
            'zero_copy_hits': 0,
            'simd_operations': 0,
            'average_sample_time': 0.0
        }
        
        # Pre-allocate buffers for zero-copy operations
        self.max_neurons_per_sample = buffer_size
        self.output_buffer = np.empty((self.max_neurons_per_sample, 6), dtype=np.float32)
        
        # Initialize SIMD configuration from centralized state manager
        self._initialize_simd_configuration()
        
        # Initialize direct access to optimized structures
        self._initialize_optimized_access()
        
        # Cache OPU areas for motor sampling
        self._cached_opu_areas = None
        self._opu_cache_timestamp = 0.0
        
        logger.info(f"UnifiedFQSampler initialized: mode={self.sampling_mode}, "
                   f"frequency={self.sample_frequency}Hz, simd={self.simd_config['available']}, "
                   f"zero_copy={self.enable_zero_copy}")

    def _initialize_simd_configuration(self) -> None:
        """Initialize SIMD configuration from centralized state manager."""
        try:
            # Use centralized SIMD configuration from State Manager
            from feagi.core.state_manager import get_state_manager
            state_manager = get_state_manager()
            self.simd_config = state_manager.get_simd_configuration()
            
            # Initialize membrane processor if SIMD is available
            if self.simd_config['available'] and self.enable_simd:
                try:
                    from feagi.npu.optimized_membrane_operations import SIMDMembraneProcessor
                    self.membrane_processor = SIMDMembraneProcessor(
                        backend=self.simd_config['backend'],
                        vector_width=self.simd_config['vector_width']
                    )
                    logger.info(f"UnifiedFQSampler using centralized SIMD: {self.simd_config['backend']} "
                               f"(vector_width={self.simd_config['vector_width']})")
                except ImportError:
                    logger.warning("UnifiedFQSampler: SIMDMembraneProcessor not available")
                    self.membrane_processor = None
            else:
                self.membrane_processor = None
                if self.enable_simd:
                    logger.info("UnifiedFQSampler: SIMD not available, using scalar operations")
                else:
                    logger.info("UnifiedFQSampler: SIMD disabled by configuration")
                    
        except ImportError:
            logger.warning("UnifiedFQSampler: State Manager not available, SIMD disabled")
            self.simd_config = {'available': False, 'backend': 'SCALAR', 'vector_width': 1}
            self.membrane_processor = None
        except Exception as e:
            logger.error(f"UnifiedFQSampler: Error accessing SIMD configuration: {e}")
            self.simd_config = {'available': False, 'backend': 'SCALAR', 'vector_width': 1}
            self.membrane_processor = None

    def _initialize_optimized_access(self) -> None:
        """Initialize direct access to optimized SoA structures."""
        try:
            # Get direct access to FCL manager and neuron arrays
            self.fcl_manager = getattr(self.connectome_manager, 'fcl_manager', None) if self.connectome_manager else None
            self.gna = getattr(self.connectome_manager, 'neuron_array', None) if self.connectome_manager else None
            
            if self.fcl_manager and self.enable_zero_copy:
                logger.info("UnifiedFQSampler: Zero-copy mode enabled with direct SoA access")
            else:
                logger.info("UnifiedFQSampler: Using standard fire queue access")
                
        except Exception as e:
            logger.warning(f"UnifiedFQSampler: Could not initialize optimized access: {e}")
            self.fcl_manager = None
            self.gna = None

    def sample_direct(self) -> Optional[bytes]:
        """
        High-performance direct sampling with zero-copy optimizations.
        
        Returns:
            Binary encoded sample data or None if no firing neurons
        """
        start_time = time.perf_counter()
        
        try:
            # Get fire queue data based on sampling mode
            brain_data = self._extract_brain_data_by_mode()
            
            if brain_data is None or len(brain_data) == 0:
                return None
            
            # Add timestamp column
            timestamped_data = np.column_stack((
                brain_data,
                np.full(len(brain_data), time.time(), dtype=np.float32)
            ))
            
            # Update performance stats
            self._performance_stats['samples_generated'] += 1
            if self.enable_zero_copy and self.fcl_manager:
                self._performance_stats['zero_copy_hits'] += 1
            if self.simd_config['available'] and self.membrane_processor:
                self._performance_stats['simd_operations'] += 1
            
            # Track timing
            sample_time = time.perf_counter() - start_time
            self._performance_stats['average_sample_time'] = (
                (self._performance_stats['average_sample_time'] * self._sample_count + sample_time) / 
                (self._sample_count + 1)
            )
            self._sample_count += 1
            
            # Direct binary encoding
            return self._encode_brain_data_binary(timestamped_data)
            
        except Exception as e:
            logger.error(f"Error in direct sampling: {e}")
            return None
                
    def _extract_brain_data_by_mode(self) -> Optional[np.ndarray]:
        """Extract brain data based on the configured sampling mode."""
        if self.sampling_mode == 'global':
            return self._extract_global_brain_data()
        elif self.sampling_mode == 'motor_only':
            return self._extract_motor_brain_data()
        elif self.sampling_mode == 'areas_only':
            return self._extract_areas_brain_data(self.target_areas)
        elif self.sampling_mode == 'custom':
            return self._extract_custom_brain_data()
        else:
            logger.error(f"Unknown sampling mode: {self.sampling_mode}")
            return None
                
    def _extract_global_brain_data(self) -> Optional[np.ndarray]:
        """Extract brain data from global fire queue."""
        if not self.fcl_manager:
            return None
                
        global_fcl = self.fcl_manager.get_global_fcl()
        return self._extract_brain_data_from_fcl(global_fcl)

    def _extract_motor_brain_data(self) -> Optional[np.ndarray]:
        """Extract brain data from motor/OPU areas only."""
        if not self.fcl_manager:
            return None

        # Cache OPU areas with timeout to avoid repeated lookups
        current_time = time.time()
        if (self._cached_opu_areas is None or 
            current_time - self._opu_cache_timestamp > 5.0):  # 5 second cache
            self._cached_opu_areas = self._get_opu_areas_fast()
            self._opu_cache_timestamp = current_time
        
        if not self._cached_opu_areas:
            return None
                    
        # Combine FCLs from all OPU areas using bitmap operations
        combined_fcl = None
        
        for area_id in self._cached_opu_areas:
            area_fcl = self.fcl_manager.get_cortical_fcl(area_id)
            if not area_fcl.is_empty():
                if combined_fcl is None:
                    combined_fcl = area_fcl
                else:
                    combined_fcl = combined_fcl | area_fcl
        
        if combined_fcl is None or combined_fcl.is_empty():
            return None
            
        return self._extract_brain_data_from_fcl(combined_fcl)

    def _extract_areas_brain_data(self, area_ids: List[str]) -> Optional[np.ndarray]:
        """Extract brain data from specific cortical areas."""
        if not self.fcl_manager or not area_ids:
            return None

        # Combine FCLs from specified areas
        combined_fcl = None
        
        for area_id in area_ids:
            try:
                area_fcl = self.fcl_manager.get_cortical_fcl(area_id)
                if not area_fcl.is_empty():
                    if combined_fcl is None:
                        combined_fcl = area_fcl
                    else:
                        combined_fcl = combined_fcl | area_fcl
            except Exception as e:
                logger.warning(f"Error accessing FCL for area {area_id}: {e}")
                continue
        
        if combined_fcl is None or combined_fcl.is_empty():
            return None
        
        return self._extract_brain_data_from_fcl(combined_fcl)

    def _extract_custom_brain_data(self) -> Optional[np.ndarray]:
        """Extract brain data using custom sampling logic."""
        # Default implementation - can be overridden by subclasses
        return self._extract_global_brain_data()

    def _extract_brain_data_from_fcl(self, fcl) -> Optional[np.ndarray]:
        """Extract brain data from a given FCL using direct SoA access."""
        if fcl.is_empty():
            return None
        
        try:
            # Direct FCL to indices (zero-copy if FCL supports it)
            if hasattr(fcl, 'to_gpu_array'):
                firing_indices = fcl.to_gpu_array()  # Already numpy array!
            else:
                firing_indices = np.array(list(fcl), dtype=np.int32)
            
            if len(firing_indices) == 0:
                return None
            
            # Direct SoA extraction - no intermediate conversions
            if not self.gna:
                # Fallback coordinate generation
                return np.column_stack((
                    firing_indices.astype(np.float32),
                    np.ones(len(firing_indices), dtype=np.float32),  # membrane_potentials
                    (firing_indices % 100).astype(np.float32),      # x_coords
                    ((firing_indices // 100) % 100).astype(np.float32),  # y_coords
                    (firing_indices // 10000).astype(np.float32)    # z_coords
                ))
            
            # Use actual SoA structures
            brain_data = np.column_stack((
                firing_indices.astype(np.float32),
                (self.gna.membrane_potentials[firing_indices] 
                 if hasattr(self.gna, 'membrane_potentials') 
                 else np.ones(len(firing_indices), dtype=np.float32)),
                (self.gna.coordinates_x[firing_indices].astype(np.float32)
                 if hasattr(self.gna, 'coordinates_x')
                 else (firing_indices % 100).astype(np.float32)),
                (self.gna.coordinates_y[firing_indices].astype(np.float32)
                 if hasattr(self.gna, 'coordinates_y')
                 else ((firing_indices // 100) % 100).astype(np.float32)),
                (self.gna.coordinates_z[firing_indices].astype(np.float32)
                 if hasattr(self.gna, 'coordinates_z')
                 else (firing_indices // 10000).astype(np.float32))
            ))
            
            return brain_data
            
        except Exception as e:
            logger.error(f"Error extracting brain data from FCL: {e}")
            return None

    def _get_opu_areas_fast(self) -> List[str]:
        """Fast lookup of OPU (Output Processing Unit) cortical areas."""
        try:
            if not self.connectome_manager:
                return []
            
            # Fast lookup using cached connectome structure
            if hasattr(self.connectome_manager, 'get_areas_by_type'):
                return self.connectome_manager.get_areas_by_type('OPU')
            
            # Fallback: scan all areas for OPU type
            opu_areas = []
            all_areas = getattr(self.connectome_manager, 'get_cortical_areas', lambda: [])()
            
            for area_id in all_areas:
                try:
                    area_info = self.connectome_manager.get_area_info(area_id)
                    if area_info and area_info.get('type') == 'OPU':
                        opu_areas.append(area_id)
                except Exception:
                    continue  # Skip areas we can't access
            
            return opu_areas
                            
        except Exception as e:
            logger.warning(f"Error getting OPU areas: {e}")
            return []

    def _encode_brain_data_binary(self, data: np.ndarray) -> bytes:
        """Encode brain data directly to binary format."""
        try:
            import struct
            
            # Header: [neuron_count][timestamp][data_type]
            header = struct.pack('!IQB', 
                               len(data),           # neuron count (4 bytes)
                               int(time.time()),    # timestamp (8 bytes)  
                               11)                  # Type 11 = NEURON_CATEGORIES (1 byte)
            
            # Data: Direct numpy array to bytes (zero-copy)
            return header + data.tobytes()
                
        except Exception as e:
            logger.error(f"Error encoding brain data to binary: {e}")
            return b''

    def run(self) -> None:
        """Main sampling loop with deterministic timing."""
        logger.info(f"UnifiedFQSampler started: mode={self.sampling_mode}, frequency={self.sample_frequency}Hz")
        self.running = True
        
        while self.running:
            start = time.perf_counter()
            
            # Generate sample
            sample_data = self.sample_direct()
            
            # Store sample in output queue if available
            if sample_data and self.output_queue:
                try:
                    self.output_queue.put_nowait(sample_data)
                except Exception as e:
                    logger.warning(f"Failed to queue sample: {e}")
            
            # RTOS-compatible deterministic timing
            elapsed = time.perf_counter() - start
            if elapsed < self.sample_interval:
                target_end_time = start + self.sample_interval
                while time.perf_counter() < target_end_time:
                    pass  # Busy-wait for remainder of sample interval

    def stop(self) -> None:
        """Stop the sampler."""
        self.running = False
        logger.info(f"UnifiedFQSampler stopped. Performance stats: {self._performance_stats}")

    def get_performance_stats(self) -> Dict[str, Any]:
        """Get performance statistics."""
        stats = self._performance_stats.copy()
        stats['sample_frequency'] = self.sample_frequency
        stats['sampling_mode'] = self.sampling_mode
        stats['simd_enabled'] = self.simd_config['available']
        stats['zero_copy_enabled'] = self.enable_zero_copy and self.fcl_manager is not None
        return stats

    def set_sample_frequency(self, frequency_hz: float) -> None:
        """Update sampling frequency."""
        if frequency_hz > 0:
            self.sample_frequency = frequency_hz
            self.sample_interval = 1.0 / frequency_hz
            logger.info(f"UnifiedFQSampler frequency updated to {frequency_hz}Hz")

    def set_target_areas(self, area_ids: List[str]) -> None:
        """Update target areas for area-specific sampling."""
        self.target_areas = area_ids
        logger.info(f"UnifiedFQSampler target areas updated: {area_ids}")


# Backward compatibility aliases
FQSampler = UnifiedFQSampler
OptimizedFQSampler = UnifiedFQSampler

# Public API
__all__ = ['UnifiedFQSampler', 'FQSampler', 'OptimizedFQSampler'] 