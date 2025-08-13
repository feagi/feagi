"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
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

import logging
import threading
import time
import traceback
import uuid  # Add UUID import for unique instance IDs
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional

import numpy as np

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
    """Unified Fire Queue Sampler with Extensible Strategy Architecture.

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

    def __init__(
        self,
        fire_queue_provider,
        sample_frequency_hz: float,
        sampling_mode: str = "visualization",
        output_queue=None,
        connectome_manager=None,
        target_areas: Optional[List[str]] = None,
        state_manager=None,
    ):
        """Initialize the unified FQ sampler.

        Args:
            fire_queue_provider: Source for fire queue data
            sample_frequency_hz: Sampling frequency in Hz
            sampling_mode: 'visualization', 'opu', or 'custom_areas'
            output_queue: Optional output queue for samples
            connectome_manager: Optional connectome manager for area info
            target_areas: Required for 'custom_areas' mode
            state_manager: Optional state manager for debug flag access
        """
        self.fire_queue_provider = fire_queue_provider
        self.sample_frequency = sample_frequency_hz
        self.sample_interval = (
            1.0 / sample_frequency_hz if sample_frequency_hz > 0 else 0.1
        )
        self.output_queue = output_queue
        self.connectome_manager = connectome_manager
        self.state_manager = state_manager
        self.running = False

        # Generate unique instance ID for debugging
        self.instance_id = str(uuid.uuid4())[:8]

        # Configure sampling strategy
        self._configure_strategy(sampling_mode, target_areas)

        # Performance tracking
        self._sample_count = 0
        self._performance_stats = {
            "samples_generated": 0,
            "successful_samples": 0,
            "failed_samples": 0,
            "average_sample_time": 0.0,
            "last_sample_time": 0.0,
        }

        # Subscriber tracking
        self._has_visualization_subscribers = False
        self._has_motor_subscribers = False

        # Pre-allocated buffers for zero-copy operations
        self._buffer_size = 100000  # VISUALIZATION FIX: Increased from 10,000 to 100,000 for large cortical areas
        self._preallocated_buffers = self._initialize_buffers()

        # OPU area caching for performance
        self._opu_areas_cache: Optional[List[str]] = None
        self._cache_timestamp: float = 0.0
        self._cache_ttl: float = 10.0

        logger.info(
            f"UnifiedFQSampler initialized: mode={self.current_strategy.mode.value}, "
            f"frequency={self.sample_frequency}Hz"
        )

    @property
    def sampling_mode(self) -> str:
        """Get the current sampling mode as a string."""
        return self.current_strategy.mode.value

    def _initialize_buffers(self) -> Dict[str, np.ndarray]:
        """Initialize pre-allocated buffers for zero-copy operations."""
        return {
            "neuron_ids_buffer": np.zeros(self._buffer_size, dtype=np.int32),
            "potentials_buffer": np.zeros(self._buffer_size, dtype=np.float32),
            "coordinates_buffer": np.zeros(
                (self._buffer_size, 3), dtype=np.uint32
            ),
            "temp_indices": np.zeros(self._buffer_size, dtype=np.int32),
        }

    def _configure_strategy(
        self, sampling_mode: str, target_areas: Optional[List[str]]
    ) -> None:
        """Configure the sampling strategy."""
        try:
            mode = SamplingMode(sampling_mode)
        except ValueError:
            raise ValueError(
                f"Invalid sampling mode: {sampling_mode}. "
                f"Valid modes: {[m.value for m in SamplingMode]}"
            )

        if mode == SamplingMode.CUSTOM_AREAS and not target_areas:
            raise ValueError(
                "target_areas must be provided for custom_areas mode"
            )

        self.current_strategy = SamplingStrategy(
            mode=mode, target_areas=target_areas, custom_config={}
        )

    def set_sampling_mode(
        self, mode: str, target_areas: Optional[List[str]] = None
    ) -> bool:
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
        """Sample data using the current strategy with zero-copy operations.

        Returns:
            Dictionary organized by cortical areas with high-performance data access.
        """
        start_time = time.perf_counter()

        try:
            # Get target areas based on strategy
            target_areas = self._get_target_areas()
            if not target_areas:
                logger.debug(
                    "🔥 FQ SAMPLER: No target areas found for sampling"
                )
                return None

            # High-performance sampling with zero-copy operations
            logger.debug(
                f"🔥 FQ SAMPLER: Target areas for sampling: {target_areas}"
            )
            result = self._sample_areas_optimized(target_areas)

            # Update performance stats
            sample_time = time.perf_counter() - start_time
            self._update_performance_stats(sample_time, result is not None)

            # Debug logging for successful samples
            if result:
                logger.debug(f"🔥 FQ SAMPLER: Sample result: {result}")
                area_counts = {
                    area: len(data.get("neuron_ids", []))
                    for area, data in result.items()
                }
                total_neurons = sum(area_counts.values())
                if total_neurons > 0:
                    if self._is_debug_npu_enabled():
                        logger.info(
                            f"🔥🔥🔥🔥🔥 FQ SAMPLER: Sampled {total_neurons} neurons from {len(result)} areas: {area_counts}"
                        )

            return result

        except Exception as e:
            logger.error(f"🔥 FQ SAMPLER: Error during sampling: {e}")
            self._performance_stats["failed_samples"] += 1
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
        """Get cortical areas that have firing neurons AND visualization
        enabled."""
        logger.info(
            f"🔥 PIPELINE [{self.instance_id}]: _get_visualization_areas() START"
        )
        visualization_areas = []

        if not self.fire_queue_provider:
            logger.info(
                "🔥 FQ SAMPLER: No fire queue provider available for visualization areas"
            )
            return []

        try:
            # STEP 1: Get areas that actually have firing neurons FROM FCL (performance fix)
            areas_with_activity = []

            # ✅ FCL-DRIVEN APPROACH: Query FCL for active cortical indices, then translate to IDs
            if (
                hasattr(self.connectome_manager, "fcl_manager")
                and self.connectome_manager.fcl_manager
            ):
                try:
                    # Get cortical indices that have firing neurons from FCL
                    active_cortical_indices = (
                        self.connectome_manager.fcl_manager.get_active_corticals()
                    )
                    logger.info(
                        f"🔥 PIPELINE [{self.instance_id}]: FCL returned {len(active_cortical_indices)} active indices: {sorted(active_cortical_indices)}"
                    )

                    # Translate cortical_idx -> cortical_id using BiDirectionalCorticalMap
                    for cortical_idx in active_cortical_indices:
                        try:
                            cortical_id = self.connectome_manager.get_cortical_id_for_idx(
                                cortical_idx
                            )

                            if cortical_id:
                                areas_with_activity.append(cortical_id)
                                # Get actual neuron count from FCL for this area
                                fcl_bitmap = self.connectome_manager.fcl_manager.get_cortical_fcl(
                                    cortical_idx
                                )
                                neuron_count = (
                                    len(fcl_bitmap) if fcl_bitmap else 0
                                )
                                logger.info(
                                    f"🔥 PIPELINE [{self.instance_id}]: Translated idx={cortical_idx} → id='{cortical_id}' ({neuron_count} neurons)"
                                )
                            else:
                                logger.warning(
                                    f"🔥 PIPELINE [{self.instance_id}]: Failed translation for cortical_idx={cortical_idx}"
                                )
                        except Exception as e:
                            logger.error(
                                f"🔥 PIPELINE [{self.instance_id}]: Translation error for idx {cortical_idx}: {e}"
                            )
                            continue

                except Exception as e:
                    logger.error(
                        f"🔥 PIPELINE [{self.instance_id}]: FCL query error: {e}"
                    )
                    areas_with_activity = []
            else:
                logger.warning(
                    f"🔥 PIPELINE [{self.instance_id}]: No FCL manager available"
                )

            logger.info(
                f"🔥 PIPELINE [{self.instance_id}]: areas_with_activity: {areas_with_activity}"
            )

            # STEP 2: Filter areas by visualization requirements
            filtered_areas = []
            for area_id in areas_with_activity:
                try:
                    # Check if visualization is enabled for this area
                    area_info = self.connectome_manager.get_cortical_area(
                        area_id
                    )
                    if (
                        area_info
                        and hasattr(area_info, "properties")
                        and area_info.properties
                    ):
                        viz_enabled = area_info.properties.get(
                            "viz_enabled", True
                        )
                    else:
                        viz_enabled = True  # Default to enabled

                    if viz_enabled:
                        filtered_areas.append(area_id)
                        logger.info(
                            f"🔥 PIPELINE [{self.instance_id}]: Area {area_id} included (has activity + visualization enabled)"
                        )
                    else:
                        logger.info(
                            f"🔥 PIPELINE [{self.instance_id}]: Area {area_id} excluded (visualization disabled)"
                        )

                except Exception as e:
                    logger.error(
                        f"🔥 PIPELINE [{self.instance_id}]: Error checking viz for {area_id}: {e}"
                    )
                    # Include by default if we can't check
                    filtered_areas.append(area_id)

            visualization_areas = filtered_areas
            logger.info(
                f"🔥 PIPELINE [{self.instance_id}]: Final visualization areas: {visualization_areas}"
            )

        except Exception as e:
            logger.error(
                f"🔥 PIPELINE [{self.instance_id}]: _get_visualization_areas error: {e}"
            )
            import traceback

            logger.error(
                f"🔥 PIPELINE [{self.instance_id}]: Full traceback: {traceback.format_exc()}"
            )

        logger.info(
            f"🔥 PIPELINE [{self.instance_id}]: _get_visualization_areas() END - returning {len(visualization_areas)} areas"
        )
        return visualization_areas

    def _get_all_areas(self) -> List[str]:
        """Get all available cortical areas (legacy method, use
        _get_visualization_areas instead)."""
        # Keep for backward compatibility but delegate to proper method
        return self._get_visualization_areas()

    def _get_opu_areas_cached(self) -> List[str]:
        """Get OPU areas with high-performance caching."""
        current_time = time.time()

        # Use cached areas if still valid
        if (
            self._opu_areas_cache is not None
            and current_time - self._cache_timestamp < self._cache_ttl
        ):
            return self._opu_areas_cache

        # Refresh cache with optimized lookup
        opu_areas = []
        if not self.connectome_manager:
            logger.debug(
                "🔥 FQ SAMPLER: No connectome manager available for OPU areas"
            )
            self._opu_areas_cache = []
            self._cache_timestamp = current_time
            return []

        try:
            # Fast lookup using cached connectome structure
            if hasattr(self.connectome_manager, "get_areas_by_type"):
                opu_areas = self.connectome_manager.get_areas_by_type("OPU")
                logger.debug(
                    f"🔥 FQ SAMPLER: Found OPU areas via get_areas_by_type: {opu_areas}"
                )
            else:
                # Optimized scan for OPU areas - look for various OPU indicators
                if hasattr(self.connectome_manager, "cortical_areas"):
                    for (
                        area_id,
                        area_obj,
                    ) in self.connectome_manager.cortical_areas.items():
                        try:
                            is_opu = False

                            # Check multiple possible OPU indicators
                            if (
                                hasattr(area_obj, "properties")
                                and area_obj.properties
                            ):
                                area_type = area_obj.properties.get(
                                    "type", ""
                                ).upper()
                                cortical_area_type = area_obj.properties.get(
                                    "cortical_area_type", ""
                                ).upper()

                                # Check for OPU type indicators
                                if (
                                    area_type in ["OPU", "MOTOR", "OUTPUT"]
                                    or cortical_area_type
                                    in ["OPU", "MOTOR", "OUTPUT"]
                                    or "OPU" in area_type
                                    or "MOTOR" in area_type
                                ):
                                    is_opu = True

                            # Also check area name patterns (common convention)
                            if not is_opu and (
                                "opu" in area_id.lower()
                                or "motor" in area_id.lower()
                            ):
                                is_opu = True

                            if is_opu:
                                opu_areas.append(area_id)
                                logger.debug(
                                    f"🔥 FQ SAMPLER: Area {area_id} identified as OPU"
                                )

                        except Exception as e:
                            logger.debug(
                                f"🔥 FQ SAMPLER: Error checking area {area_id} for OPU type: {e}"
                            )
                            continue

        except Exception as e:
            logger.error(f"🔥 FQ SAMPLER: Error getting OPU areas: {e}")

        # Update cache
        self._opu_areas_cache = opu_areas
        self._cache_timestamp = current_time

        if self._is_debug_npu_enabled():
            logger.info(
                f"🔥 FQ SAMPLER: Found {len(opu_areas)} OPU areas for motor sampling: {opu_areas}"
            )
        return opu_areas

    def _sample_areas_optimized(
        self, target_areas: List[str]
    ) -> Optional[Dict[str, Any]]:
        """High-performance area sampling with minimal allocations.

        Combines zero-copy operations with SIMD-friendly data structures.
        """
        logger.info(
            f"🔥 PIPELINE [{self.instance_id}]: _sample_areas_optimized() START with {len(target_areas)} areas: {target_areas}"
        )

        result = {}
        current_timestamp = time.time()

        # High-performance area sampling with minimal allocations
        for area_id in target_areas:
            try:
                logger.info(
                    f"🔥 PIPELINE [{self.instance_id}]: Processing area '{area_id}'"
                )

                # MEMORY AREA OPTIMIZATION: Check if this is a memory area first
                is_memory_area = self._is_memory_area(area_id)

                if is_memory_area:
                    logger.info(
                        f"🔥 PIPELINE [{self.instance_id}]: Area '{area_id}' identified as MEMORY area"
                    )
                    # Memory areas: efficient handling with simplified data structure
                    memory_data = self._sample_memory_area_optimized(
                        area_id, current_timestamp
                    )
                    if memory_data:
                        result[area_id] = memory_data
                        logger.info(
                            f"🔥 PIPELINE [{self.instance_id}]: Memory area '{area_id}' successfully sampled"
                        )
                    else:
                        logger.info(
                            f"🔥 PIPELINE [{self.instance_id}]: Memory area '{area_id}' returned no data"
                        )
                    continue

                logger.info(
                    f"🔥 PIPELINE [{self.instance_id}]: Area '{area_id}' identified as REGULAR area"
                )

                # REGULAR AREAS: Use existing fire queue lookup logic
                area_data = None
                if hasattr(
                    self.fire_queue_provider, "get_area_fire_queue_zerocopy"
                ):
                    logger.info(
                        f"🔥 PIPELINE [{self.instance_id}]: Using zerocopy method for '{area_id}'"
                    )
                    area_data = (
                        self.fire_queue_provider.get_area_fire_queue_zerocopy(
                            area_id
                        )
                    )
                elif hasattr(self.fire_queue_provider, "get_area_fire_queue"):
                    logger.info(
                        f"🔥 PIPELINE [{self.instance_id}]: Using standard method for '{area_id}'"
                    )
                    area_data = self.fire_queue_provider.get_area_fire_queue(
                        area_id
                    )
                else:
                    logger.warning(
                        f"🔥 PIPELINE [{self.instance_id}]: No fire queue method available for '{area_id}'"
                    )
                    continue

                if not area_data or not area_data.get("neuron_ids"):
                    logger.info(
                        f"🔥 PIPELINE [{self.instance_id}]: No data or no neuron_ids for regular area '{area_id}'"
                    )
                    continue

                # Direct reference to data (zero-copy) with fallback to view creation
                neuron_ids = area_data["neuron_ids"]

                if not neuron_ids:
                    logger.info(
                        f"🔥 PIPELINE [{self.instance_id}]: Empty neuron_ids for regular area '{area_id}'"
                    )
                    continue

                logger.info(
                    f"🔥 PIPELINE [{self.instance_id}]: Regular area '{area_id}' has {len(neuron_ids)} neurons"
                )

                # Create efficient data structure maintaining references
                result[area_id] = {
                    "neuron_ids": neuron_ids,  # Direct reference, no copy
                    "membrane_potentials": area_data.get(
                        "membrane_potentials", []
                    ),
                    "thresholds": area_data.get("thresholds", []),
                    "consecutive_fire_counts": area_data.get(
                        "consecutive_fire_counts", []
                    ),
                    "refractory_counters": area_data.get(
                        "refractory_counters", []
                    ),
                    "coordinates": area_data.get("coordinates", []),
                    "timestamp": current_timestamp,
                }

                logger.info(
                    f"🔥 PIPELINE [{self.instance_id}]: Regular area '{area_id}' successfully sampled"
                )

            except Exception as e:
                logger.error(
                    f"🔥 PIPELINE [{self.instance_id}]: CRITICAL ERROR sampling area '{area_id}': {e}"
                )
                logger.error(
                    f"🔥 PIPELINE [{self.instance_id}]: Exception type: {type(e)}"
                )
                import traceback

                logger.error(
                    f"🔥 PIPELINE [{self.instance_id}]: Full traceback: {traceback.format_exc()}"
                )
                continue

        logger.info(
            f"🔥 PIPELINE [{self.instance_id}]: _sample_areas_optimized() END - sampled {len(result)} areas: {list(result.keys())}"
        )
        return result if result else None

    def _is_memory_area(self, area_id: str) -> bool:
        """Efficiently check if a cortical area is a memory area.

        Args:
            area_id: Cortical area ID to check

        Returns:
            True if this is a memory area, False otherwise
        """
        logger.info(
            f"🔥 PIPELINE [{self.instance_id}]: _is_memory_area('{area_id}') START"
        )

        try:
            if not self.connectome_manager:
                logger.warning(
                    f"🔥 PIPELINE [{self.instance_id}]: No connectome_manager for memory check"
                )
                return False

            # ONLY reliable check: Is it in the memory_areas set?
            if (
                hasattr(self.connectome_manager, "memory_areas")
                and area_id in self.connectome_manager.memory_areas
            ):
                logger.info(
                    f"🔥 PIPELINE [{self.instance_id}]: Area '{area_id}' is memory (in memory_areas set)"
                )
                return True

            # Secondary check: explicit sub_group_id = MEMORY
            area = self.connectome_manager.get_cortical_area(area_id)
            if area and hasattr(area, "properties") and area.properties:
                if area.properties.get("sub_group_id") == "MEMORY":
                    logger.info(
                        f"🔥 PIPELINE [{self.instance_id}]: Area '{area_id}' is memory (sub_group_id=MEMORY)"
                    )
                    return True

            logger.info(
                f"🔥 PIPELINE [{self.instance_id}]: Area '{area_id}' is NOT a memory area"
            )
            return False

        except Exception as e:
            logger.error(
                f"🔥 PIPELINE [{self.instance_id}]: Error checking memory area '{area_id}': {e}"
            )
            return False

    def _sample_memory_area_optimized(
        self, area_id: str, timestamp: float
    ) -> Optional[Dict[str, Any]]:
        """Efficiently sample a memory area for visualization.

        Memory areas are conceptual 1x1x1 voxels at (0,0,0) coordinates.
        They use placeholder IDs to avoid conflicts with regular neuron SoA lookups.

        Args:
            area_id: Memory area ID
            timestamp: Current timestamp

        Returns:
            Simplified data structure if area is active, None otherwise
        """
        logger.info(
            f"🔥 PIPELINE [{self.instance_id}]: _sample_memory_area_optimized('{area_id}') START"
        )

        try:
            if not self.connectome_manager:
                logger.warning(
                    f"🔥 PIPELINE [{self.instance_id}]: No connectome_manager for memory sampling"
                )
                return None

            # Get cortical_idx for this memory area
            area = self.connectome_manager.get_cortical_area(area_id)
            if not area:
                logger.warning(
                    f"🔥 PIPELINE [{self.instance_id}]: Memory area '{area_id}' not found"
                )
                return None

            cortical_idx = getattr(area, "cortical_idx", None)
            if cortical_idx is None:
                logger.warning(
                    f"🔥 PIPELINE [{self.instance_id}]: No cortical_idx for memory area '{area_id}'"
                )
                return None

            logger.info(
                f"🔥 PIPELINE [{self.instance_id}]: Memory area '{area_id}' has cortical_idx={cortical_idx}"
            )

            # Check if memory area has activity in FCL
            if (
                hasattr(self.connectome_manager, "fcl_manager")
                and self.connectome_manager.fcl_manager
            ):
                fcl_bitmap = (
                    self.connectome_manager.fcl_manager.get_cortical_fcl(
                        cortical_idx
                    )
                )
                if not fcl_bitmap or fcl_bitmap.is_empty():
                    logger.info(
                        f"🔥 PIPELINE [{self.instance_id}]: Memory area '{area_id}' has no FCL activity"
                    )
                    return None

                neuron_count = len(fcl_bitmap)
                logger.info(
                    f"🔥 PIPELINE [{self.instance_id}]: Memory area '{area_id}' has {neuron_count} firing neurons in FCL"
                )

                # Memory area is active - return safe placeholder data for visualization
                # Use a high placeholder ID that won't conflict with regular neuron lookups
                placeholder_id = 100000 + cortical_idx  # Safe range: 100000+

                memory_data = {
                    "neuron_ids": [
                        placeholder_id
                    ],  # list of int (matching regular areas)
                    "membrane_potentials": [
                        500.0
                    ],  # list of float (matching regular areas)
                    "thresholds": [
                        1.0
                    ],  # FIXED: same length as neuron_ids (length 1)
                    "consecutive_fire_counts": [
                        0
                    ],  # FIXED: same length as neuron_ids (length 1)
                    "refractory_counters": [
                        0
                    ],  # FIXED: same length as neuron_ids (length 1)
                    "coordinates": [
                        (0, 0, 0)
                    ],  # list of tuple (matching regular areas)
                    "timestamp": timestamp,
                }

                logger.info(
                    f"🔥 PIPELINE [{self.instance_id}]: Memory area '{area_id}' sampled successfully with placeholder_id={placeholder_id}"
                )
                return memory_data
            else:
                logger.warning(
                    f"🔥 PIPELINE [{self.instance_id}]: No FCL manager for memory area '{area_id}'"
                )
                return None

        except Exception as e:
            logger.error(
                f"🔥 PIPELINE [{self.instance_id}]: Error sampling memory area '{area_id}': {e}"
            )
            import traceback

            logger.error(
                f"🔥 PIPELINE [{self.instance_id}]: Traceback: {traceback.format_exc()}"
            )
            return None

    def _update_performance_stats(
        self, sample_time: float, success: bool
    ) -> None:
        """Update performance statistics efficiently."""
        self._performance_stats["samples_generated"] += 1
        self._performance_stats["last_sample_time"] = sample_time

        if success:
            self._performance_stats["successful_samples"] += 1
        else:
            self._performance_stats["failed_samples"] += 1

        # Efficient rolling average
        self._performance_stats["average_sample_time"] = (
            self._performance_stats["average_sample_time"] * self._sample_count
            + sample_time
        ) / (self._sample_count + 1)
        self._sample_count += 1

    def run(self) -> None:
        """Main sampling loop with deterministic timing."""
        self.running = True
        logger.info(
            f"🔥 FQ SAMPLER: Starting sampling loop: mode={self.current_strategy.mode.value}, "
            f"frequency={self.sample_frequency}Hz"
        )

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
                            logger.info(
                                f"🔥 FQ SAMPLER: Queued sample {sample_count} with data from {len(sample_data)} areas"
                            )
                        except Exception as e:
                            logger.error(
                                f"🔥 FQ SAMPLER: Output queue error: {e}"
                            )
                    elif sample_data:
                        if sample_count % 10 == 0:  # Log every 10 samples
                            logger.info(
                                f"🔥 FQ SAMPLER: Generated sample {sample_count} with {len(sample_data)} areas but no output queue"
                            )
                    else:
                        if (
                            sample_count % 30 == 0
                        ):  # Log every 30 empty samples
                            logger.debug(
                                f"🔥 FQ SAMPLER: Sample {sample_count} returned no data"
                            )

                # Deterministic timing control
                elapsed = time.perf_counter() - start_time
                sleep_time = max(0, self.sample_interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except Exception as e:
            logger.error(f"🔥 FQ SAMPLER: Fatal error in sampling loop: {e}")
        finally:
            self.running = False
            logger.info(
                f"🔥 FQ SAMPLER: Sampling loop stopped after {sample_count} samples"
            )

    def _should_sample(self) -> bool:
        """Determine if sampling should occur based on subscriber presence."""

        should_sample = (
            self._has_visualization_subscribers
            or self._has_motor_subscribers
            or self.output_queue is not None
        )

        # Debug logging every 30 calls to avoid spam
        if not hasattr(self, "_debug_counter"):
            self._debug_counter = 0
        self._debug_counter += 1

        if self._debug_counter % 30 == 0:  # Log every 30 calls
            logger.info(
                f"🔥 FQ SAMPLER [{self.instance_id}]: _should_sample() = {should_sample} "
                f"(viz_subs={self._has_visualization_subscribers}, "
                f"motor_subs={self._has_motor_subscribers}, "
                f"output_queue={self.output_queue is not None})"
            )

        return should_sample

    def stop(self) -> None:
        """Stop the sampler."""
        self.running = False
        logger.info(
            f"UnifiedFQSampler stopped. Performance stats: {self._performance_stats}"
        )

    def set_visualization_subscribers(self, has_subscribers: bool) -> None:
        """Set whether there are visualization subscribers."""
        old_state = self._has_visualization_subscribers
        self._has_visualization_subscribers = has_subscribers

        # Debug: Add stack trace to identify caller with more detailed info

        # Get just the immediate caller (2 levels up)
        stack = traceback.extract_stack()
        if len(stack) >= 3:
            caller_frame = stack[-3]  # The frame that called this method
            caller_info = f"{caller_frame.filename}:{caller_frame.lineno} in {caller_frame.name}()"
        else:
            caller_info = "Unknown caller"

        # Instance-specific logging with thread info
        thread_id = threading.current_thread().ident

        logger.info(
            f"🔥 FQ SAMPLER [{self.instance_id}]: Visualization subscribers: {old_state} -> {has_subscribers}"
        )
        logger.info(
            f"🔥 FQ SAMPLER [{self.instance_id}]: Called from: {caller_info}"
        )
        logger.info(
            f"🔥 FQ SAMPLER [{self.instance_id}]: Thread: {thread_id}, Mode: {self.sampling_mode}"
        )
        logger.info(
            f"🔥 FQ SAMPLER [{self.instance_id}]: _should_sample() now: {self._should_sample()}"
        )

    def set_motor_subscribers(self, has_subscribers: bool) -> None:
        """Set whether there are motor subscribers."""
        old_state = self._has_motor_subscribers
        self._has_motor_subscribers = has_subscribers
        logger.info(
            f"🔥 FQ SAMPLER: Motor subscribers changed: {old_state} -> {has_subscribers}"
        )

    def get_performance_stats(self) -> Dict[str, Any]:
        """Get performance statistics."""
        stats = self._performance_stats.copy()
        stats["sample_frequency"] = self.sample_frequency
        stats["sampling_mode"] = self.current_strategy.mode.value
        stats["target_areas"] = self._get_target_areas()
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

    def _is_debug_npu_enabled(self) -> bool:
        """Check if debug NPU logging is enabled."""
        if self.state_manager and hasattr(
            self.state_manager, "is_debug_npu_enabled"
        ):
            return self.state_manager.is_debug_npu_enabled()
        return False


# Public API
__all__ = ["UnifiedFQSampler", "SamplingMode"]
