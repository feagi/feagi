# Copyright 2016-2025 Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""
Memory Processor - CPU-optimized pattern detection and memory neuron lifecycle management.

This processor runs parallel to normal neural processing and handles:
- Temporal pattern extraction from upstream FCL activity
- Memory neuron creation and reactivation based on patterns
- Lifecycle management (aging, long-term memory conversion)
- Performance-optimized operations using RoaringBitmap serialization
"""

import logging
import time
import threading
from typing import Dict, Set, List, Optional, Tuple, Any
from collections import defaultdict, deque
from dataclasses import dataclass

import numpy as np

from feagi.bdu.models.memory_neuron import MemoryNeuronArray, MemoryPatternKey
from feagi.npu.fcl_manager import FCLManager

logger = logging.getLogger(__name__)


@dataclass
class MemoryProcessingStats:
    """Statistics for memory processing performance monitoring."""
    total_patterns_processed: int = 0
    memory_neurons_created: int = 0
    memory_neurons_reactivated: int = 0
    memory_neurons_died: int = 0
    memory_neurons_converted_to_longterm: int = 0
    processing_time_ms: float = 0.0
    pattern_cache_hits: int = 0
    pattern_cache_misses: int = 0


class MemoryProcessor:
    """
    CPU-optimized memory processor for temporal pattern detection and memory neuron lifecycle.
    
    This processor operates independently from GPU neural processing to avoid interference.
    It handles memory area pattern detection, neuron creation, aging, and lifecycle management.
    
    Architecture:
    - Parallel processing: Runs alongside burst engine without blocking
    - CPU-optimized: Uses numpy and bitmap operations 
    - Pattern-based: Uses RoaringBitmap serialization for pattern keys
    - Batch processing: Processes multiple memory areas efficiently
    - LRU caching: Optimizes repeated pattern lookups
    """
    
    def __init__(
        self, 
        memory_neuron_array: MemoryNeuronArray,
        fcl_manager: FCLManager,
        batch_size: int = 100,
        pattern_cache_size: int = 10000
    ):
        """
        Initialize memory processor.
        
        Args:
            memory_neuron_array: Memory neuron storage array
            fcl_manager: FCL manager for temporal pattern extraction
            batch_size: Number of memory areas to process per batch
            pattern_cache_size: LRU cache size for pattern lookups
        """
        self.memory_neuron_array = memory_neuron_array
        self.fcl_manager = fcl_manager
        self.batch_size = batch_size
        
        # Processing control
        self._processing_lock = threading.RLock()
        self._is_processing = False
        self._debug_enabled = False
        
        # Memory area tracking
        self.active_memory_areas: Set[str] = set()
        self.memory_area_properties: Dict[str, Dict[str, Any]] = {}
        
        # Pattern caching for performance optimization
        self._pattern_cache: Dict[MemoryPatternKey, int] = {}
        self._pattern_cache_size = pattern_cache_size
        self._pattern_access_order: deque = deque()
        
        # Performance statistics
        self.stats = MemoryProcessingStats()
        
        # Current burst tracking
        self.current_burst = 0
        
        logger.info(f"MemoryProcessor initialized with batch_size={batch_size}, cache_size={pattern_cache_size}")
    
    def register_memory_area(
        self, 
        cortical_id: str, 
        temporal_depth: int,
        initial_lifespan: int = 9,
        lifespan_growth_rate: float = 1.0,
        longterm_threshold: int = 100,
        upstream_areas: Optional[Set[str]] = None
    ) -> bool:
        """
        Register a memory cortical area for processing.
        
        Args:
            cortical_id: Memory cortical area ID
            temporal_depth: Number of timesteps for pattern recognition
            initial_lifespan: Initial lifespan for new memory neurons
            lifespan_growth_rate: Growth rate for lifespan on reactivation
            longterm_threshold: Threshold for long-term memory conversion
            upstream_areas: Set of upstream cortical areas (for optimization)
            
        Returns:
            True if successful
        """
        with self._processing_lock:
            self.active_memory_areas.add(cortical_id)
            self.memory_area_properties[cortical_id] = {
                "temporal_depth": temporal_depth,
                "initial_lifespan": initial_lifespan,
                "lifespan_growth_rate": lifespan_growth_rate,
                "longterm_threshold": longterm_threshold,
                "upstream_areas": upstream_areas or set()
            }
            
            logger.info(f"Registered memory area {cortical_id} with temporal_depth={temporal_depth}")
            return True
    
    def unregister_memory_area(self, cortical_id: str) -> bool:
        """
        Unregister a memory cortical area.
        
        Args:
            cortical_id: Memory cortical area ID to unregister
            
        Returns:
            True if successful
        """
        with self._processing_lock:
            self.active_memory_areas.discard(cortical_id)
            self.memory_area_properties.pop(cortical_id, None)
            
            # Remove patterns from cache for this area
            self._cleanup_pattern_cache_for_area(cortical_id)
            
            logger.info(f"Unregistered memory area {cortical_id}")
            return True
    
    def update_memory_area_upstream(self, cortical_id: str, upstream_areas: Set[str]) -> None:
        """Update upstream areas for a memory area (for optimization)."""
        if cortical_id in self.memory_area_properties:
            self.memory_area_properties[cortical_id]["upstream_areas"] = upstream_areas
    
    def process_memory_areas_batch(self, current_burst: int) -> Dict[str, Any]:
        """
        Process all active memory areas in batch for current burst.
        
        Args:
            current_burst: Current burst number
            
        Returns:
            Processing results and statistics
        """
        if self._is_processing:
            logger.debug("Memory processing already in progress, skipping")
            return {"skipped": True}
        
        with self._processing_lock:
            self._is_processing = True
            start_time = time.time()
            
            try:
                self.current_burst = current_burst
                
                # Reset burst statistics
                burst_stats = {
                    "patterns_processed": 0,
                    "neurons_created": 0,
                    "neurons_reactivated": 0,
                    "neurons_died": 0,
                    "neurons_converted": 0,
                    "areas_processed": 0
                }
                
                # Process memory areas in batches
                memory_areas_list = list(self.active_memory_areas)
                for i in range(0, len(memory_areas_list), self.batch_size):
                    batch = memory_areas_list[i:i + self.batch_size]
                    batch_results = self._process_memory_area_batch(batch, current_burst)
                    
                    # Aggregate statistics
                    for key in burst_stats:
                        if key in batch_results:
                            burst_stats[key] += batch_results[key]
                
                # Perform aging and lifecycle management
                aging_results = self._perform_aging_and_lifecycle(current_burst)
                burst_stats["neurons_died"] += aging_results["neurons_died"]
                burst_stats["neurons_converted"] += aging_results["neurons_converted"]
                
                # Update global statistics
                self.stats.total_patterns_processed += burst_stats["patterns_processed"]
                self.stats.memory_neurons_created += burst_stats["neurons_created"]
                self.stats.memory_neurons_reactivated += burst_stats["neurons_reactivated"]
                self.stats.memory_neurons_died += burst_stats["neurons_died"]
                self.stats.memory_neurons_converted_to_longterm += burst_stats["neurons_converted"]
                
                processing_time = (time.time() - start_time) * 1000  # Convert to milliseconds
                self.stats.processing_time_ms = processing_time
                
                if self._debug_enabled:
                    logger.debug(f"Memory processing burst {current_burst}: {burst_stats}, time: {processing_time:.2f}ms")
                
                return {
                    "success": True,
                    "burst": current_burst,
                    "stats": burst_stats,
                    "processing_time_ms": processing_time
                }
                
            except Exception as e:
                logger.error(f"Error processing memory areas batch: {e}")
                return {"success": False, "error": str(e)}
                
            finally:
                self._is_processing = False
    
    def _process_memory_area_batch(self, memory_areas: List[str], current_burst: int) -> Dict[str, int]:
        """Process a batch of memory areas."""
        batch_stats = {
            "patterns_processed": 0,
            "neurons_created": 0,
            "neurons_reactivated": 0,
            "areas_processed": 0
        }
        
        for memory_area_id in memory_areas:
            try:
                area_results = self._process_single_memory_area(memory_area_id, current_burst)
                batch_stats["patterns_processed"] += area_results["patterns_processed"]
                batch_stats["neurons_created"] += area_results["neurons_created"]
                batch_stats["neurons_reactivated"] += area_results["neurons_reactivated"]
                batch_stats["areas_processed"] += 1
                
            except Exception as e:
                logger.error(f"Error processing memory area {memory_area_id}: {e}")
        
        return batch_stats
    
    def _process_single_memory_area(self, memory_area_id: str, current_burst: int) -> Dict[str, int]:
        """Process a single memory area for current burst."""
        properties = self.memory_area_properties.get(memory_area_id)
        if not properties:
            return {"patterns_processed": 0, "neurons_created": 0, "neurons_reactivated": 0}
        
        temporal_depth = properties["temporal_depth"]
        upstream_areas = properties["upstream_areas"]
        
        # Extract temporal pattern from upstream FCL activity
        pattern_key = self._extract_temporal_pattern(upstream_areas, temporal_depth, current_burst)
        
        if pattern_key is None:
            # No pattern to process (no upstream activity)
            return {"patterns_processed": 0, "neurons_created": 0, "neurons_reactivated": 0}
        
        # Check if pattern already exists (with caching)
        existing_neuron_idx = self._find_or_cache_pattern(pattern_key)
        
        if existing_neuron_idx is not None:
            # Reactivate existing memory neuron
            success = self.memory_neuron_array.reactivate_memory_neuron(existing_neuron_idx, current_burst)
            neurons_reactivated = 1 if success else 0
            neurons_created = 0
        else:
            # Create new memory neuron
            neuron_idx = self.memory_neuron_array.create_memory_neuron(
                pattern_key=pattern_key,
                cortical_area_id=memory_area_id,
                current_burst=current_burst,
                initial_lifespan=properties["initial_lifespan"],
                lifespan_growth_rate=properties["lifespan_growth_rate"]
            )
            
            neurons_created = 1 if neuron_idx is not None else 0
            neurons_reactivated = 0
            
            # Add to cache if created successfully
            if neuron_idx is not None:
                self._add_to_pattern_cache(pattern_key, neuron_idx)
        
        return {
            "patterns_processed": 1,
            "neurons_created": neurons_created,
            "neurons_reactivated": neurons_reactivated
        }
    
    def _extract_temporal_pattern(
        self, 
        upstream_areas: Set[str], 
        temporal_depth: int, 
        current_burst: int
    ) -> Optional[MemoryPatternKey]:
        """
        Extract temporal pattern from upstream FCL activity using bitmap serialization.
        
        Args:
            upstream_areas: Set of upstream cortical area IDs
            temporal_depth: Number of timesteps to include in pattern
            current_burst: Current burst number
            
        Returns:
            MemoryPatternKey if pattern exists, None if no activity
        """
        if not upstream_areas:
            return None
        
        # Extract FCL bitmaps for last temporal_depth timesteps
        pattern_bitmaps = []
        has_activity = False
        
        for timestep_offset in range(temporal_depth):
            timestep = current_burst - timestep_offset
            if timestep < 0:
                # Use empty bitmap for timesteps before simulation start
                pattern_bitmaps.append(b'')
            else:
                # Get combined FCL bitmap for all upstream areas at this timestep
                combined_bitmap = self.fcl_manager.get_neurons_by_corticals(
                    list(upstream_areas), timestep=timestep
                )
                
                if len(combined_bitmap) > 0:
                    has_activity = True
                
                # Serialize bitmap to bytes for pattern key
                serialized_bitmap = combined_bitmap.serialize()
                pattern_bitmaps.append(serialized_bitmap)
        
        # If no activity in any timestep, return None
        if not has_activity:
            return None
        
        # Create pattern key using bitmap sequence approach (Option 2)
        pattern_data = tuple(pattern_bitmaps)
        source_areas = tuple(sorted(upstream_areas))
        
        return MemoryPatternKey(
            pattern_data=pattern_data,
            temporal_depth=temporal_depth,
            source_cortical_areas=source_areas
        )
    
    def _find_or_cache_pattern(self, pattern_key: MemoryPatternKey) -> Optional[int]:
        """Find neuron index for pattern using cache for performance."""
        # Check cache first
        if pattern_key in self._pattern_cache:
            # Update access order for LRU
            self._pattern_access_order.remove(pattern_key)
            self._pattern_access_order.append(pattern_key)
            self.stats.pattern_cache_hits += 1
            return self._pattern_cache[pattern_key]
        
        # Cache miss - lookup in memory neuron array
        neuron_idx = self.memory_neuron_array.find_memory_neuron_by_pattern(pattern_key)
        self.stats.pattern_cache_misses += 1
        
        # Add to cache if found
        if neuron_idx is not None:
            self._add_to_pattern_cache(pattern_key, neuron_idx)
        
        return neuron_idx
    
    def _add_to_pattern_cache(self, pattern_key: MemoryPatternKey, neuron_idx: int) -> None:
        """Add pattern to cache with LRU eviction."""
        # Remove oldest entry if cache is full
        if len(self._pattern_cache) >= self._pattern_cache_size:
            oldest_key = self._pattern_access_order.popleft()
            del self._pattern_cache[oldest_key]
        
        # Add new entry
        self._pattern_cache[pattern_key] = neuron_idx
        self._pattern_access_order.append(pattern_key)
    
    def _cleanup_pattern_cache_for_area(self, cortical_id: str) -> None:
        """Remove cached patterns for a specific cortical area."""
        keys_to_remove = []
        for pattern_key in self._pattern_cache:
            if cortical_id in pattern_key.source_cortical_areas:
                keys_to_remove.append(pattern_key)
        
        for key in keys_to_remove:
            del self._pattern_cache[key]
            self._pattern_access_order.remove(key)
    
    def _perform_aging_and_lifecycle(self, current_burst: int) -> Dict[str, int]:
        """Perform aging and lifecycle management for all memory neurons."""
        # Age all memory neurons
        died_neurons = self.memory_neuron_array.age_memory_neurons(current_burst)
        
        # Check for long-term memory conversions
        converted_neurons = []
        for area_id, properties in self.memory_area_properties.items():
            longterm_threshold = properties["longterm_threshold"]
            area_conversions = self.memory_neuron_array.check_longterm_conversion(longterm_threshold)
            converted_neurons.extend(area_conversions)
        
        # Remove dead neurons from cache
        for neuron_idx in died_neurons:
            self._remove_neuron_from_cache(neuron_idx)
        
        return {
            "neurons_died": len(died_neurons),
            "neurons_converted": len(converted_neurons)
        }
    
    def _remove_neuron_from_cache(self, neuron_idx: int) -> None:
        """Remove neuron from pattern cache when it dies."""
        keys_to_remove = []
        for pattern_key, cached_idx in self._pattern_cache.items():
            if cached_idx == neuron_idx:
                keys_to_remove.append(pattern_key)
        
        for key in keys_to_remove:
            del self._pattern_cache[key]
            self._pattern_access_order.remove(key)
    
    def get_processing_statistics(self) -> Dict[str, Any]:
        """Get comprehensive processing statistics."""
        neuron_stats = self.memory_neuron_array.get_statistics()
        
        return {
            "memory_processor": {
                "total_patterns_processed": self.stats.total_patterns_processed,
                "memory_neurons_created": self.stats.memory_neurons_created,
                "memory_neurons_reactivated": self.stats.memory_neurons_reactivated,
                "memory_neurons_died": self.stats.memory_neurons_died,
                "memory_neurons_converted_to_longterm": self.stats.memory_neurons_converted_to_longterm,
                "last_processing_time_ms": self.stats.processing_time_ms,
                "pattern_cache_hits": self.stats.pattern_cache_hits,
                "pattern_cache_misses": self.stats.pattern_cache_misses,
                "cache_hit_ratio": self.stats.pattern_cache_hits / max(1, self.stats.pattern_cache_hits + self.stats.pattern_cache_misses),
                "active_memory_areas": len(self.active_memory_areas),
                "pattern_cache_size": len(self._pattern_cache)
            },
            "memory_neuron_array": neuron_stats
        }
    
    def enable_debug(self, enabled: bool = True) -> None:
        """Enable or disable debug logging."""
        self._debug_enabled = enabled
        logger.info(f"Memory processor debug logging {'enabled' if enabled else 'disabled'}")
    
    def get_memory_area_debug_info(self, cortical_id: str) -> Optional[Dict[str, Any]]:
        """Get debug information for a specific memory area."""
        if cortical_id not in self.memory_area_properties:
            return None
        
        properties = self.memory_area_properties[cortical_id]
        active_neurons = self.memory_neuron_array.get_active_neurons_for_area(cortical_id)
        
        neuron_details = []
        for neuron_idx in active_neurons[:10]:  # Limit to first 10 for performance
            neuron_info = self.memory_neuron_array.get_memory_neuron_info(neuron_idx)
            if neuron_info:
                neuron_details.append(neuron_info)
        
        return {
            "cortical_id": cortical_id,
            "properties": properties,
            "active_neuron_count": len(active_neurons),
            "sample_neurons": neuron_details,
            "is_active": cortical_id in self.active_memory_areas
        } 