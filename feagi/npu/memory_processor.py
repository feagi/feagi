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
        pattern_cache_size: int = 10000,
        connectome_manager=None
    ):
        """
        Initialize memory processor.
        
        Args:
            memory_neuron_array: Memory neuron storage array
            fcl_manager: FCL manager for temporal pattern extraction
            batch_size: Number of memory areas to process per batch
            pattern_cache_size: LRU cache size for pattern lookups
            connectome_manager: Direct access to ConnectomeManager (optional)
        """
        self.memory_neuron_array = memory_neuron_array
        self.fcl_manager = fcl_manager
        self.connectome_manager = connectome_manager  # Direct access
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
            
            logger.info(f"🧠 [MEMORY] Registered memory area {cortical_id} with temporal_depth={temporal_depth}")
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
            
            logger.info(f"🧠 [MEMORY] Unregistered memory area {cortical_id}")
            return True
    
    def update_memory_area_upstream(self, cortical_id: str, upstream_areas: Set[str]) -> None:
        """Update upstream areas for a memory area (for optimization)."""
        if cortical_id in self.memory_area_properties:
            self.memory_area_properties[cortical_id]["upstream_areas"] = upstream_areas
    
    def process_memory_areas_batch(self, current_burst: int) -> Dict[str, Any]:
        """
        Process all registered memory areas for current burst cycle.
        
        Args:
            current_burst: Current burst timestep
            
        Returns:
            Processing results and statistics
        """
        if self._is_processing:
            return {"success": False, "error": "Already processing"}
        
        start_time = time.time()
        
        try:
            with self._processing_lock:
                self._is_processing = True
                self.current_burst = current_burst
                
                # ENHANCED DEBUG: Check if NPU debugging is enabled
                npu_debug = self._is_npu_debug_enabled()
                if npu_debug:
                    logger.info(f"🧠 [MEMORY] MEMORY PROCESSING START: Burst {current_burst}")
                    logger.info(f"🧠 [MEMORY] Active memory areas: {list(self.active_memory_areas)}")
                    for area_id in self.active_memory_areas:
                        props = self.memory_area_properties.get(area_id, {})
                        logger.info(f"🧠 [MEMORY] Memory area {area_id}: temporal_depth={props.get('temporal_depth')}, upstream_areas={props.get('upstream_areas')}")
                
                if not self.active_memory_areas:
                    if npu_debug:
                        logger.info(f"🧠 [MEMORY] No active memory areas to process")
                    return {"success": True, "processed_areas": 0, "stats": {}}
                
                # Process memory areas in batches
                memory_areas = list(self.active_memory_areas)
                total_processed = 0
                batch_results = []
                
                for i in range(0, len(memory_areas), self.batch_size):
                    batch = memory_areas[i:i + self.batch_size]
                    if npu_debug:
                        logger.info(f"🧠 [MEMORY] Processing memory batch {i//self.batch_size + 1}: {batch}")
                    
                    batch_result = self._process_memory_area_batch(batch, current_burst)
                    batch_results.append(batch_result)
                    total_processed += len(batch)
                
                # Aggregate results
                total_patterns = sum(result.get("patterns_processed", 0) for result in batch_results)
                total_created = sum(result.get("neurons_created", 0) for result in batch_results)
                total_reactivated = sum(result.get("neurons_reactivated", 0) for result in batch_results)
                
                # Perform aging and lifecycle management
                if npu_debug:
                    logger.info(f"🧠 [MEMORY] Performing memory neuron aging and lifecycle management")
                lifecycle_result = self._perform_aging_and_lifecycle(current_burst)
                
                # Update statistics
                self.stats.total_patterns_processed += total_patterns
                self.stats.memory_neurons_created += total_created
                self.stats.memory_neurons_reactivated += total_reactivated
                self.stats.memory_neurons_died += lifecycle_result.get("neurons_died", 0)
                self.stats.memory_neurons_converted_to_longterm += lifecycle_result.get("neurons_converted", 0)
                self.stats.processing_time_ms = (time.time() - start_time) * 1000
                
                if npu_debug:
                    logger.info(f"🧠 [MEMORY] MEMORY PROCESSING COMPLETE: patterns={total_patterns}, created={total_created}, reactivated={total_reactivated}, died={lifecycle_result.get('neurons_died', 0)}")
                
                return {
                    "success": True,
                    "processed_areas": total_processed,
                    "processing_time_ms": self.stats.processing_time_ms,
                    "stats": {
                        "patterns_processed": total_patterns,
                        "neurons_created": total_created,
                        "neurons_reactivated": total_reactivated,
                        "neurons_died": lifecycle_result.get("neurons_died", 0),
                        "neurons_converted": lifecycle_result.get("neurons_converted", 0)
                    }
                }
                
        except Exception as e:
            logger.error(f"🧠 [MEMORY] Error in memory processing: {e}")
            return {"success": False, "error": str(e)}
        finally:
            self._is_processing = False

    def _is_npu_debug_enabled(self) -> bool:
        """Check if NPU debugging is enabled."""
        try:
            from feagi.core.state_manager import get_state_manager
            state_manager = get_state_manager()
            return state_manager.is_debug_npu_enabled()
        except:
            return False
    
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
                logger.error(f"🧠 [MEMORY] Error processing memory area {memory_area_id}: {e}")
        
        return batch_stats
    
    def _process_single_memory_area(self, memory_area_id: str, current_burst: int) -> Dict[str, int]:
        """
        Process a single memory area for current burst.
        
        Args:
            memory_area_id: ID of the memory cortical area
            current_burst: Current burst number
            
        Returns:
            Dictionary with processing statistics
        """
        stats = {
            'patterns_processed': 0,
            'neurons_created': 0,
            'neurons_reactivated': 0,
            'neurons_died': 0,
            'neurons_converted': 0
        }
        
        npu_debug = self._is_npu_debug_enabled()
        
        if npu_debug:
            logger.info(f"🧠 [MEMORY] Processing memory area {memory_area_id}")
        
        # Get memory area properties
        area_properties = self.memory_area_properties.get(memory_area_id, {})
        temporal_depth = area_properties.get('temporal_depth', 1)
        initial_lifespan = area_properties.get('initial_lifespan', 9)
        lifespan_growth_rate = area_properties.get('lifespan_growth_rate', 1.0)
        longterm_threshold = area_properties.get('longterm_threshold', 100)
        upstream_areas = area_properties.get('upstream_areas', set())
        
        if npu_debug:
            logger.info(f"🧠 [MEMORY] Memory area {memory_area_id}: temporal_depth={temporal_depth}, upstream_areas={upstream_areas}")
        
        # 1. Extract temporal pattern from upstream areas
        temporal_pattern = self._extract_temporal_pattern(upstream_areas, temporal_depth, current_burst)
        
        if temporal_pattern:
            stats['patterns_processed'] = 1
            if npu_debug:
                logger.info(f"🧠 [MEMORY] Pattern detected for {memory_area_id}: creating/reactivating memory neuron")
            
            # 2. Find or create memory neuron for this pattern
            memory_neuron_created = self._find_or_cache_pattern(temporal_pattern)
            
            if memory_neuron_created:
                stats['neurons_created'] = 1
            else:
                stats['neurons_reactivated'] = 1
        else:
            if npu_debug:
                logger.info(f"🧠 [MEMORY] No temporal pattern detected for memory area {memory_area_id} (no upstream activity)")
        
        # 3. Perform aging and lifecycle management for all memory neurons in this area
        if npu_debug:
            logger.info(f"🧠 [MEMORY] Performing memory neuron aging and lifecycle management")
        
        try:
            died_neurons = self.memory_neuron_array.age_memory_neurons(current_burst)
            stats['neurons_died'] = len(died_neurons)
            
            # Check for long-term memory conversion
            converted_neurons = self.memory_neuron_array.check_longterm_conversion(longterm_threshold)
            stats['neurons_converted'] = len(converted_neurons)
            
            if npu_debug and (died_neurons or converted_neurons):
                logger.info(f"🧠 [MEMORY] Memory neuron lifecycle: {len(died_neurons)} died, {len(converted_neurons)} converted to long-term")
                
        except Exception as e:
            if npu_debug:
                logger.error(f"🧠 [MEMORY] Error during memory neuron aging: {e}")
        
        return stats

    def _extract_temporal_pattern(
        self, 
        upstream_areas: Set[str], 
        temporal_depth: int, 
        current_burst: int
    ) -> Optional[MemoryPatternKey]:
        """
        Extract temporal patterns from upstream cortical areas' FCL activity.
        
        Args:
            upstream_areas: Set of upstream cortical area IDs
            temporal_depth: Number of timesteps to include in pattern
            current_burst: Current burst timestep
            
        Returns:
            MemoryPatternKey if pattern found, None if no activity
        """
        try:
            npu_debug = self._is_npu_debug_enabled()
            if npu_debug:
                logger.info(f"🧠 [MEMORY] PATTERN EXTRACTION: upstream_areas={upstream_areas}, temporal_depth={temporal_depth}, current_burst={current_burst}")
            
            if not upstream_areas:
                if npu_debug:
                    logger.info(f"🧠 [MEMORY] No upstream areas - cannot extract pattern")
                return None
            
            pattern_bitmaps = []
            valid_areas = []
            
            # Extract patterns for the specified temporal depth
            for timestep_offset in range(temporal_depth):
                timestep = current_burst - timestep_offset
                if timestep < 0:
                    if npu_debug:
                        logger.info(f"🧠 [MEMORY] Timestep {timestep} is negative - using empty pattern")
                    pattern_bitmaps.append(b'')  # Empty pattern for negative timesteps
                    continue
                
                # Combine firing patterns from all upstream areas for this timestep
                combined_bitmap = None
                areas_with_activity = []
                
                for upstream_area_id in upstream_areas:
                    try:
                        # Get cortical_idx for upstream area
                        cortical_idx = self._get_cortical_idx_for_area(upstream_area_id)
                        if cortical_idx is None:
                            if npu_debug:
                                logger.warning(f"🧠 [MEMORY] Could not get cortical_idx for upstream area {upstream_area_id}")
                            continue
                        
                        # Get FCL bitmap for this area at this timestep
                        area_bitmap = self.fcl_manager.get_cortical_fcl(cortical_idx, timestep)
                        if area_bitmap and len(area_bitmap) > 0:
                            areas_with_activity.append(upstream_area_id)
                            if combined_bitmap is None:
                                combined_bitmap = area_bitmap.copy()
                            else:
                                combined_bitmap.or_update(area_bitmap)
                            
                            if npu_debug:
                                neuron_count = len(area_bitmap)
                                logger.info(f"🧠 [MEMORY] Timestep {timestep}: area {upstream_area_id} has {neuron_count} firing neurons")
                    except Exception as area_error:
                        if npu_debug:
                            logger.warning(f"🧠 [MEMORY] Error processing upstream area {upstream_area_id}: {area_error}")
                        continue
                
                if npu_debug:
                    total_neurons = len(combined_bitmap) if combined_bitmap else 0
                    logger.info(f"🧠 [MEMORY] Timestep {timestep}: combined pattern has {total_neurons} neurons from areas {areas_with_activity}")
                
                # Serialize the combined bitmap for this timestep
                if combined_bitmap and len(combined_bitmap) > 0:
                    pattern_bitmaps.append(combined_bitmap.serialize())
                    if upstream_area_id not in valid_areas:  # Avoid duplicates
                        valid_areas.extend(areas_with_activity)
                else:
                    pattern_bitmaps.append(b'')  # Empty pattern for this timestep
            
            # Check if we have any meaningful pattern
            non_empty_patterns = [p for p in pattern_bitmaps if p]
            if not non_empty_patterns:
                if npu_debug:
                    logger.info(f"🧠 [MEMORY] No meaningful patterns found - all timesteps empty")
                return None
            
            # Create pattern key
            pattern_key = MemoryPatternKey(
                pattern_data=tuple(pattern_bitmaps),
                temporal_depth=temporal_depth,
                source_cortical_areas=tuple(sorted(set(valid_areas)))
            )
            
            if npu_debug:
                logger.info(f"🧠 [MEMORY] PATTERN CREATED: {len(non_empty_patterns)}/{temporal_depth} non-empty timesteps, source_areas={pattern_key.source_cortical_areas}")
            
            return pattern_key
            
        except Exception as e:
            logger.error(f"🧠 [MEMORY] Error extracting temporal pattern: {e}")
            return None
    
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
    
    def _inject_memory_neurons_into_fcl(
        self, 
        memory_area_id: str, 
        memory_neuron_indices: List[int], 
        current_burst: int
    ) -> None:
        """
        Inject active memory neurons into FCL so they appear as firing neurons.
        
        ARCHITECTURE FIX: Instead of using virtual neuron IDs, we need to ensure
        memory areas have real neurons that the FQ Sampler can look up.
        
        Args:
            memory_area_id: ID of the memory cortical area
            memory_neuron_indices: List of memory neuron indices that should fire
            current_burst: Current burst timestep
        """
        try:
            npu_debug = self._is_npu_debug_enabled()
            if npu_debug:
                logger.info(f"🧠 [MEMORY] FCL INJECTION START: area={memory_area_id}, neurons={memory_neuron_indices}, burst={current_burst}")
            
            # CRITICAL FIX: Get actual neuron IDs from memory area instead of creating virtual ones
            # Memory areas should have real neurons that FQ Sampler can look up
            real_neuron_ids = self._get_real_neuron_ids_for_memory_area(memory_area_id, memory_neuron_indices)
            
            if not real_neuron_ids:
                if npu_debug:
                    logger.warning(f"🧠 [MEMORY] No real neuron IDs available for memory area {memory_area_id} - this will cause FQ Sampler issues!")
                return
            
            if npu_debug:
                logger.info(f"🧠 [MEMORY] Using real neuron IDs for FCL injection: {real_neuron_ids}")
            
            # Get cortical_idx for the memory area
            cortical_idx = self._get_cortical_idx_for_memory_area(memory_area_id)
            
            if cortical_idx is not None:
                if npu_debug:
                    logger.info(f"🧠 [MEMORY] Found cortical_idx={cortical_idx} for memory area {memory_area_id}")
                    logger.info(f"�� [MEMORY] Injecting real neuron IDs {real_neuron_ids} into FCL")
                
                # Inject memory neurons into FCL using real neuron IDs
                neurons_by_cortical = {cortical_idx: real_neuron_ids}
                self.fcl_manager.update_fcl(current_burst, neurons_by_cortical)
                
                if npu_debug:
                    logger.info(f"🧠 [MEMORY] FCL INJECTION SUCCESS: {len(real_neuron_ids)} memory neurons injected")
                    
                    # Verify FCL contains our real neurons
                    try:
                        fcl_bitmap = self.fcl_manager.get_cortical_fcl(cortical_idx, current_burst)
                        if fcl_bitmap:
                            fcl_neurons = list(fcl_bitmap)
                            logger.info(f"🧠 [MEMORY] FCL verification: area {memory_area_id} now has {len(fcl_neurons)} neurons in FCL")
                            logger.info(f"🧠 [MEMORY] FCL contains our real IDs: {[nid for nid in real_neuron_ids if nid in fcl_neurons]}")
                            logger.info(f"🧠 [MEMORY] SUCCESS: These are REAL neuron IDs - FQ Sampler should find them!")
                        else:
                            logger.warning(f"🧠 [MEMORY] FCL verification failed: no neurons found for area {memory_area_id}")
                    except Exception as e:
                        logger.warning(f"🧠 [MEMORY] FCL verification error: {e}")
            else:
                logger.warning(f"🧠 [MEMORY] Could not get cortical_idx for memory area {memory_area_id} - FCL injection failed")
            
        except Exception as e:
            logger.error(f"🧠 [MEMORY] Error injecting memory neurons into FCL: {e}")
    
    def _get_real_neuron_ids_for_memory_area(self, memory_area_id: str, memory_neuron_indices: List[int]) -> List[int]:
        """
        Get real neuron IDs for memory neurons in a memory area.
        
        This method ensures memory areas have actual neurons that FQ Sampler can look up.
        If memory area doesn't have enough neurons, we need to create them.
        
        Args:
            memory_area_id: Memory cortical area ID
            memory_neuron_indices: List of memory neuron indices that should fire
            
        Returns:
            List of real neuron IDs that exist in the neuron array
        """
        try:
            # Get access to ConnectomeManager through FCL manager
            if not hasattr(self.fcl_manager, 'connectome_manager'):
                logger.warning(f"🧠 [MEMORY] Cannot access ConnectomeManager for memory area {memory_area_id}")
                return []
            
            connectome_manager = self.fcl_manager.connectome_manager
            
            # Get neurons for this memory area
            memory_area_neurons = connectome_manager.get_neurons_by_cortical_area(memory_area_id)
            
            if not memory_area_neurons:
                logger.warning(f"🧠 [MEMORY] Memory area {memory_area_id} has no neurons! Memory areas need real neurons for FQ Sampler.")
                # TODO: We should create neurons for memory areas during creation
                return []
            
            # Map memory neuron indices to real neuron IDs
            # For now, use the first N neurons from the memory area
            max_needed = max(memory_neuron_indices) + 1 if memory_neuron_indices else 0
            available_neurons = list(memory_area_neurons)
            
            if len(available_neurons) < max_needed:
                logger.warning(f"🧠 [MEMORY] Memory area {memory_area_id} has {len(available_neurons)} neurons but needs {max_needed}")
                # Use what we have, cycling if necessary
                real_neuron_ids = []
                for mem_idx in memory_neuron_indices:
                    if mem_idx < len(available_neurons):
                        real_neuron_ids.append(available_neurons[mem_idx])
                    else:
                        # Cycle through available neurons
                        real_neuron_ids.append(available_neurons[mem_idx % len(available_neurons)])
                return real_neuron_ids
            else:
                # Direct mapping: memory_neuron_index -> available_neurons[index]
                return [available_neurons[mem_idx] for mem_idx in memory_neuron_indices]
            
        except Exception as e:
            logger.error(f"🧠 [MEMORY] Error getting real neuron IDs for memory area {memory_area_id}: {e}")
            return []

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
        logger.info(f"🧠 [MEMORY] Memory processor debug logging {'enabled' if enabled else 'disabled'}")
        
        # Auto-enable when NPU debug is active
        if not enabled and self._is_npu_debug_enabled():
            self._debug_enabled = True
            logger.info("🧠 [MEMORY] Memory processor debug auto-enabled due to --debug-npu flag")
    
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

    def _get_cortical_idx_for_memory_area(self, memory_area_id: str) -> Optional[int]:
        """Get cortical_idx for a memory area."""
        try:
            # We need access to ConnectomeManager to get cortical_idx
            # The FCL manager should have access to this
            if hasattr(self.fcl_manager, 'connectome_manager'):
                connectome_manager = self.fcl_manager.connectome_manager
                if hasattr(connectome_manager, 'cortical_mapping'):
                    return connectome_manager.cortical_mapping.get_idx(memory_area_id)
            
            # Alternative: try to get it from the memory area properties
            area_properties = self.memory_area_properties.get(memory_area_id)
            if area_properties and 'cortical_idx' in area_properties:
                return area_properties['cortical_idx']
            
            logger.warning(f"🧠 [MEMORY] Could not determine cortical_idx for memory area {memory_area_id}")
            return None
            
        except Exception as e:
            logger.error(f"🧠 [MEMORY] Error getting cortical_idx for memory area {memory_area_id}: {e}")
            return None

    def _get_cortical_idx_for_area(self, area_id: str) -> Optional[int]:
        """Get cortical_idx for any cortical area (memory or upstream)."""
        try:
            # Use direct connectome_manager reference first
            if self.connectome_manager:
                connectome_manager = self.connectome_manager
            elif hasattr(self.fcl_manager, 'connectome_manager'):
                connectome_manager = self.fcl_manager.connectome_manager
            else:
                logger.warning(f"🧠 [MEMORY] No connectome_manager available for area {area_id}")
                return None
            
            # Try cortical_areas mapping first
            if hasattr(connectome_manager, 'cortical_areas'):
                area_obj = connectome_manager.cortical_areas.get(area_id)
                if area_obj and hasattr(area_obj, 'cortical_idx'):
                    return area_obj.cortical_idx
            
            # Try cortical_mapping as fallback
            if hasattr(connectome_manager, 'cortical_mapping'):
                return connectome_manager.cortical_mapping.get_idx(area_id)
            
            logger.warning(f"🧠 [MEMORY] Could not determine cortical_idx for area {area_id}")
            return None
            
        except Exception as e:
            logger.error(f"🧠 [MEMORY] Error getting cortical_idx for area {area_id}: {e}")
            return None 