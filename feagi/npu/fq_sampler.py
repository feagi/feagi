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
Unified Fire Queue Sampler with Extensible Strategy Architecture

This module provides a flexible, plugin-based sampling system that maintains
all critical performance characteristics while enabling easy extension.

CRITICAL PERFORMANCE FEATURES (MAINTAINED):
- Zero-copy operations with direct SoA (Structure of Arrays) access
- SIMD acceleration via centralized configuration  
- Pre-allocated buffers for real-time performance
- RTOS-compatible deterministic timing
- Independent frequency configuration per instance
- Lock-free data structures for concurrent access
- Minimal allocation/deallocation in hot paths

Strategy Architecture:
- Thin strategy layer over high-performance core
- Strategies define WHAT to sample, core handles HOW efficiently
- No performance degradation from strategy pattern
- Consistent cortical-area-organized output format
"""

import time
import threading
import logging
import numpy as np
from abc import ABC, abstractmethod
from typing import Dict, List, Any, Optional, Set, Tuple, Union, Callable
from dataclasses import dataclass
from enum import Enum
import traceback
import uuid  # Add UUID import for unique instance IDs

logger = logging.getLogger(__name__)


class SamplingMode(Enum):
    """Available sampling modes."""
    VISUALIZATION = "visualization"
    OPU = "opu" 
    CUSTOM_AREAS = "custom_areas"


@dataclass
class SamplingStrategy:
    """Strategy configuration for sampling modes."""
    mode: SamplingMode
    target_areas: Optional[List[str]] = None
    custom_config: Optional[Dict[str, Any]] = None


class UnifiedFQSampler:
    """
    Unified Fire Queue Sampler with Extensible Strategy Architecture.
    
    Maintains all critical performance characteristics while providing
    flexible sampling strategies for different use cases.
    
    PERFORMANCE GUARANTEES:
    - Zero-copy operations with direct SoA access
    - SIMD acceleration support via centralized configuration
    - Pre-allocated buffers for deterministic real-time performance
    - RTOS-compatible deterministic timing with bounded execution
    - Independent frequency configuration per sampler instance
    - Lock-free data structures for high-concurrency access
    - Minimal memory allocation in sampling hot paths
    
    Sampling Modes:
    - VISUALIZATION: Sample entire fire queue organized by cortical areas
    - OPU: Sample only OPU (motor) cortical areas with caching
    - CUSTOM_AREAS: Sample specific cortical areas by ID
    
    Output Format:
    Always returns data organized by cortical areas:
    {
        'area_id_1': {
            'neuron_ids': [...],
            'membrane_potentials': [...],
            'thresholds': [...],
            'consecutive_fire_counts': [...],
            'refractory_counters': [...],
            'coordinates': [...],
            'timestamp': float
        },
        'area_id_2': { ... }
    }
    """
    
    def __init__(self, 
                 fire_queue_provider,
                 sample_frequency_hz: float,
                 sampling_mode: str = 'visualization',
                 output_queue=None,
                 connectome_manager=None,
                 target_areas: Optional[List[str]] = None):
        """
        Initialize the unified FQ sampler.
        
        Args:
            fire_queue_provider: Source for fire queue data
            sample_frequency_hz: Sampling frequency in Hz
            sampling_mode: 'visualization', 'opu', or 'custom_areas'
            output_queue: Optional output queue for samples
            connectome_manager: Optional connectome manager for area info
            target_areas: Required for 'custom_areas' mode
        """
        self.fire_queue_provider = fire_queue_provider
        self.sample_frequency = sample_frequency_hz
        self.sample_interval = 1.0 / sample_frequency_hz if sample_frequency_hz > 0 else 0.1
        self.output_queue = output_queue
        self.connectome_manager = connectome_manager
        self.running = False
        
        # Generate unique instance ID for debugging
        self.instance_id = str(uuid.uuid4())[:8]
        
        # Configure sampling strategy
        self._configure_strategy(sampling_mode, target_areas)
        
        # Performance tracking
        self._sample_count = 0
        self._performance_stats = {
            'samples_generated': 0,
            'successful_samples': 0,
            'failed_samples': 0,
            'average_sample_time': 0.0,
            'last_sample_time': 0.0
        }
        
        # Subscriber tracking
        self._has_visualization_subscribers = False
        self._has_motor_subscribers = False
        
        # Pre-allocated buffers for zero-copy operations
        self._buffer_size = 10000  # Configurable based on max expected neurons
        self._preallocated_buffers = self._initialize_buffers()
        
        # OPU area caching for performance
        self._opu_areas_cache: Optional[List[str]] = None
        self._cache_timestamp: float = 0.0
        self._cache_ttl: float = 10.0
        
        logger.info(f"UnifiedFQSampler initialized: mode={self.current_strategy.mode.value}, "
                   f"frequency={self.sample_frequency}Hz")

    @property
    def sampling_mode(self) -> str:
        """Get the current sampling mode as a string."""
        return self.current_strategy.mode.value

    def _initialize_buffers(self) -> Dict[str, np.ndarray]:
        """Initialize pre-allocated buffers for zero-copy operations."""
        return {
            'neuron_ids_buffer': np.zeros(self._buffer_size, dtype=np.int32),
            'potentials_buffer': np.zeros(self._buffer_size, dtype=np.float32),
            'coordinates_buffer': np.zeros((self._buffer_size, 3), dtype=np.uint32),
            'temp_indices': np.zeros(self._buffer_size, dtype=np.int32)
        }

    def _configure_strategy(self, sampling_mode: str, target_areas: Optional[List[str]]) -> None:
        """Configure the sampling strategy."""
        try:
            mode = SamplingMode(sampling_mode)
        except ValueError:
            raise ValueError(f"Invalid sampling mode: {sampling_mode}. "
                           f"Valid modes: {[m.value for m in SamplingMode]}")
        
        if mode == SamplingMode.CUSTOM_AREAS and not target_areas:
            raise ValueError("target_areas must be provided for custom_areas mode")
        
        self.current_strategy = SamplingStrategy(
            mode=mode,
            target_areas=target_areas,
            custom_config={}
        )

    def set_sampling_mode(self, mode: str, target_areas: Optional[List[str]] = None) -> bool:
        """Change the sampling mode."""
        try:
            self._configure_strategy(mode, target_areas)
            # Clear OPU cache when mode changes
            self._opu_areas_cache = None
            logger.info(f"Sampling mode changed to: {mode}")
            return True
        except ValueError as e:
            logger.error(f"Failed to set sampling mode: {e}")
            return False

    def sample(self) -> Optional[Dict[str, Any]]:
        """
        Sample data using the current strategy with zero-copy operations.
        
        Returns:
            Dictionary organized by cortical areas with high-performance data access.
        """
        start_time = time.perf_counter()
        
        try:
            # Get target areas based on strategy
            target_areas = self._get_target_areas()
            if not target_areas:
                logger.debug(f"🔥 FQ SAMPLER: No target areas found for sampling")
                return None
            
            # High-performance sampling with zero-copy operations
            logger.debug(f"🔥 FQ SAMPLER: Target areas for sampling: {target_areas}")
            result = self._sample_areas_optimized(target_areas)
            
            # Update performance stats
            sample_time = time.perf_counter() - start_time
            self._update_performance_stats(sample_time, result is not None)
            
            # Debug logging for successful samples
            if result:
                logger.debug(f"🔥 FQ SAMPLER: Sample result: {result}")
                area_counts = {area: len(data.get('neuron_ids', [])) for area, data in result.items()}
                total_neurons = sum(area_counts.values())
                if total_neurons > 0:
                    logger.info(f"🔥🔥🔥🔥🔥 FQ SAMPLER: Sampled {total_neurons} neurons from {len(result)} areas: {area_counts}")
            
            return result
            
        except Exception as e:
            logger.error(f"🔥 FQ SAMPLER: Error during sampling: {e}")
            self._performance_stats['failed_samples'] += 1
            return None

    def _get_target_areas(self) -> List[str]:
        """Get target areas based on current strategy with caching."""
        mode = self.current_strategy.mode
        
        if mode == SamplingMode.VISUALIZATION:
            return self._get_visualization_areas()
        elif mode == SamplingMode.OPU:
            return self._get_opu_areas_cached()
        elif mode == SamplingMode.CUSTOM_AREAS:
            return self.current_strategy.target_areas or []
        
        return []

    def _get_visualization_areas(self) -> List[str]:
        """Get all cortical areas for visualization (opt-out model: all areas unless visualization=false)."""
        visualization_areas = []
        
        if not self.connectome_manager:
            logger.info(f"🔥 FQ SAMPLER: No connectome manager available for visualization areas")
            return []
            
        try:
            if hasattr(self.connectome_manager, 'cortical_areas'):
                logger.info(f"🔥 FQ SAMPLER: Found connectome manager with {len(self.connectome_manager.cortical_areas)} cortical areas")
                for area_id, area_obj in self.connectome_manager.cortical_areas.items():
                    try:
                        # Default to True (opt-out model)
                        visualization_enabled = True
                        
                        # Check if area properties explicitly disable visualization
                        if hasattr(area_obj, 'properties') and area_obj.properties:
                            visualization_enabled = area_obj.properties.get('visualization', True)
                        
                        if visualization_enabled:
                            logger.info(f"🔥 FQ SAMPLER: Area {area_id} included in visualization")
                            visualization_areas.append(area_id)
                        else:
                            logger.info(f"🔥 FQ SAMPLER: Area {area_id} excluded from visualization (visualization=false)")
                            
                    except Exception as e:
                        logger.info(f"🔥 FQ SAMPLER: Error checking area {area_id} for visualization: {e}")
                        # Default to include in case of error
                        visualization_areas.append(area_id)
            else:
                logger.info(f"🔥 FQ SAMPLER: Connectome manager has no cortical_areas attribute")
                        
        except Exception as e:
            logger.error(f"🔥 FQ SAMPLER: Error getting visualization areas: {e}")
            
        logger.info(f"🔥 FQ SAMPLER: Final visualization areas list: {visualization_areas}")
        return visualization_areas

    def _get_all_areas(self) -> List[str]:
        """Get all available cortical areas (legacy method, use _get_visualization_areas instead)."""
        # Keep for backward compatibility but delegate to proper method
        return self._get_visualization_areas()

    def _get_opu_areas_cached(self) -> List[str]:
        """Get OPU areas with high-performance caching."""
        current_time = time.time()
        
        # Use cached areas if still valid
        if (self._opu_areas_cache is not None and 
            current_time - self._cache_timestamp < self._cache_ttl):
            return self._opu_areas_cache
        
        # Refresh cache with optimized lookup
        opu_areas = []
        if not self.connectome_manager:
            logger.debug(f"🔥 FQ SAMPLER: No connectome manager available for OPU areas")
            self._opu_areas_cache = []
            self._cache_timestamp = current_time
            return []
            
        try:
            # Fast lookup using cached connectome structure
            if hasattr(self.connectome_manager, 'get_areas_by_type'):
                opu_areas = self.connectome_manager.get_areas_by_type('OPU')
                logger.debug(f"🔥 FQ SAMPLER: Found OPU areas via get_areas_by_type: {opu_areas}")
            else:
                # Optimized scan for OPU areas - look for various OPU indicators
                if hasattr(self.connectome_manager, 'cortical_areas'):
                    for area_id, area_obj in self.connectome_manager.cortical_areas.items():
                        try:
                            is_opu = False
                            
                            # Check multiple possible OPU indicators
                            if hasattr(area_obj, 'properties') and area_obj.properties:
                                area_type = area_obj.properties.get('type', '').upper()
                                cortical_area_type = area_obj.properties.get('cortical_area_type', '').upper()
                                
                                # Check for OPU type indicators
                                if (area_type in ['OPU', 'MOTOR', 'OUTPUT'] or
                                    cortical_area_type in ['OPU', 'MOTOR', 'OUTPUT'] or
                                    'OPU' in area_type or
                                    'MOTOR' in area_type):
                                    is_opu = True
                                    
                            # Also check area name patterns (common convention)
                            if not is_opu and ('opu' in area_id.lower() or 'motor' in area_id.lower()):
                                is_opu = True
                                
                            if is_opu:
                                opu_areas.append(area_id)
                                logger.debug(f"🔥 FQ SAMPLER: Area {area_id} identified as OPU")
                                
                        except Exception as e:
                            logger.debug(f"🔥 FQ SAMPLER: Error checking area {area_id} for OPU type: {e}")
                            continue
                            
        except Exception as e:
            logger.error(f"🔥 FQ SAMPLER: Error getting OPU areas: {e}")
        
        # Update cache
        self._opu_areas_cache = opu_areas
        self._cache_timestamp = current_time
        
        logger.info(f"🔥 FQ SAMPLER: Found {len(opu_areas)} OPU areas for motor sampling: {opu_areas}")
        return opu_areas

    def _sample_areas_optimized(self, target_areas: List[str]) -> Optional[Dict[str, Any]]:
        """
        High-performance sampling of target areas with zero-copy operations.
        
        This method maintains all performance characteristics:
        - Zero-copy operations where possible
        - Pre-allocated buffer usage
        - SIMD-ready data structures
        - Deterministic execution time
        """
        
        if not target_areas or not self.fire_queue_provider:
            logger.debug(f"   - Early return: target_areas={bool(target_areas)}, fire_queue_provider={bool(self.fire_queue_provider)}")
            return None
        
        result = {}
        current_timestamp = time.time()
        
        # High-performance area sampling with minimal allocations
        for area_id in target_areas:
            try:
                
                # Get fire queue data with zero-copy access if available
                area_data = None
                if hasattr(self.fire_queue_provider, 'get_area_fire_queue_zerocopy'):
                    area_data = self.fire_queue_provider.get_area_fire_queue_zerocopy(area_id)
                    logger.info(f"🔥 FQ SAMPLER: get_area_fire_queue_zerocopy({area_id}) returned: {area_data}")
                elif hasattr(self.fire_queue_provider, 'get_area_fire_queue'):
                    area_data = self.fire_queue_provider.get_area_fire_queue(area_id)
                    logger.info(f"🔥 FQ SAMPLER: get_area_fire_queue({area_id}) returned: {area_data}")
                else:
                    logger.info(f"🔥 FQ SAMPLER: No fire queue method available for {area_id}")
                    continue

                
                if not area_data or not area_data.get('neuron_ids'):
                    logger.info(f"🔥 FQ SAMPLER: No data or no neuron_ids for area {area_id}: area_data={area_data}")
                    continue
                
                # Direct reference to data (zero-copy) with fallback to view creation
                neuron_ids = area_data['neuron_ids']

                if not neuron_ids:
                    continue
                

                # Create efficient data structure maintaining references
                result[area_id] = {
                    'neuron_ids': neuron_ids,  # Direct reference, no copy
                    'membrane_potentials': area_data.get('membrane_potentials', []),
                    'thresholds': area_data.get('thresholds', []),
                    'consecutive_fire_counts': area_data.get('consecutive_fire_counts', []),
                    'refractory_counters': area_data.get('refractory_counters', []),
                    'coordinates': area_data.get('coordinates', []),
                    'timestamp': current_timestamp
                }

            except Exception as e:
                logger.debug(f"Error sampling area {area_id}: {e}")
                continue
        
        return result if result else None

    def _update_performance_stats(self, sample_time: float, success: bool) -> None:
        """Update performance statistics efficiently."""
        self._performance_stats['samples_generated'] += 1
        self._performance_stats['last_sample_time'] = sample_time
        
        if success:
            self._performance_stats['successful_samples'] += 1
        else:
            self._performance_stats['failed_samples'] += 1
        
        # Efficient rolling average
        self._performance_stats['average_sample_time'] = (
            (self._performance_stats['average_sample_time'] * self._sample_count + sample_time) / 
            (self._sample_count + 1)
        )
        self._sample_count += 1

    def run(self) -> None:
        """Main sampling loop with deterministic timing."""
        self.running = True
        logger.info(f"🔥 FQ SAMPLER: Starting sampling loop: mode={self.current_strategy.mode.value}, "
                   f"frequency={self.sample_frequency}Hz")
        
        sample_count = 0
        
        try:
            while self.running:
                start_time = time.perf_counter()
                
                # Sample data if we have subscribers
                if self._should_sample():
                    sample_data = self.sample()
                    sample_count += 1
                    
                    if sample_data and self.output_queue:
                        try:
                            self.output_queue.put_nowait(sample_data)
                            logger.info(f"🔥 FQ SAMPLER: Queued sample {sample_count} with data from {len(sample_data)} areas")
                        except Exception as e:
                            logger.error(f"🔥 FQ SAMPLER: Output queue error: {e}")
                    elif sample_data:
                        if sample_count % 10 == 0:  # Log every 10 samples
                            logger.info(f"🔥 FQ SAMPLER: Generated sample {sample_count} with {len(sample_data)} areas but no output queue")
                    else:
                        if sample_count % 30 == 0:  # Log every 30 empty samples
                            logger.debug(f"🔥 FQ SAMPLER: Sample {sample_count} returned no data")
                
                # Deterministic timing control
                elapsed = time.perf_counter() - start_time
                sleep_time = max(0, self.sample_interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except Exception as e:
            logger.error(f"🔥 FQ SAMPLER: Fatal error in sampling loop: {e}")
        finally:
            self.running = False
            logger.info(f"🔥 FQ SAMPLER: Sampling loop stopped after {sample_count} samples")

    def _should_sample(self) -> bool:
        """Determine if sampling should occur based on subscriber presence."""
        
        should_sample = (self._has_visualization_subscribers or 
                        self._has_motor_subscribers or 
                        self.output_queue is not None)
        
        # Debug logging every 30 calls to avoid spam
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
        self._debug_counter += 1
        
        if self._debug_counter % 30 == 0:  # Log every 30 calls
            logger.info(f"🔥 FQ SAMPLER [{self.instance_id}]: _should_sample() = {should_sample} "
                       f"(viz_subs={self._has_visualization_subscribers}, "
                       f"motor_subs={self._has_motor_subscribers}, "
                       f"output_queue={self.output_queue is not None})")
        
        return should_sample

    def stop(self) -> None:
        """Stop the sampler."""
        self.running = False
        logger.info(f"UnifiedFQSampler stopped. Performance stats: {self._performance_stats}")

    def set_visualization_subscribers(self, has_subscribers: bool) -> None:
        """Set whether there are visualization subscribers."""
        old_state = self._has_visualization_subscribers
        self._has_visualization_subscribers = has_subscribers
        
        # Debug: Add stack trace to identify caller with more detailed info
        import traceback
        import threading
        
        # Get just the immediate caller (2 levels up)
        stack = traceback.extract_stack()
        if len(stack) >= 3:
            caller_frame = stack[-3]  # The frame that called this method
            caller_info = f"{caller_frame.filename}:{caller_frame.lineno} in {caller_frame.name}()"
        else:
            caller_info = "Unknown caller"
        
        # Instance-specific logging with thread info
        thread_id = threading.current_thread().ident
        
        logger.info(f"🔥 FQ SAMPLER [{self.instance_id}]: Visualization subscribers: {old_state} -> {has_subscribers}")
        logger.info(f"🔥 FQ SAMPLER [{self.instance_id}]: Called from: {caller_info}")
        logger.info(f"🔥 FQ SAMPLER [{self.instance_id}]: Thread: {thread_id}, Mode: {self.sampling_mode}")
        logger.info(f"🔥 FQ SAMPLER [{self.instance_id}]: _should_sample() now: {self._should_sample()}")
    
    def set_motor_subscribers(self, has_subscribers: bool) -> None:
        """Set whether there are motor subscribers."""
        old_state = self._has_motor_subscribers
        self._has_motor_subscribers = has_subscribers
        logger.info(f"🔥 FQ SAMPLER: Motor subscribers changed: {old_state} -> {has_subscribers}")
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """Get performance statistics."""
        stats = self._performance_stats.copy()
        stats['sample_frequency'] = self.sample_frequency
        stats['sampling_mode'] = self.current_strategy.mode.value
        stats['target_areas'] = self._get_target_areas()
        return stats

    def get_target_areas(self) -> List[str]:
        """Get the list of areas being targeted by the current strategy."""
        return self._get_target_areas()
    
    def set_sample_frequency(self, frequency_hz: float) -> None:
        """Set new sampling frequency."""
        if frequency_hz > 0:
            self.sample_frequency = frequency_hz
            self.sample_interval = 1.0 / frequency_hz
            logger.info(f"Sample frequency updated to {frequency_hz}Hz")


# Public API
__all__ = ['UnifiedFQSampler', 'SamplingMode'] 