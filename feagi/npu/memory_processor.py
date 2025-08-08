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
import sys # Added for sys.argv

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
        
        # DEBUG: Log initialization state
        logger.info(f"🧠 [MEMORY-INIT] MemoryProcessor initialized with connectome_manager: {connectome_manager is not None}")
        if hasattr(fcl_manager, 'connectome_manager'):
            logger.info(f"🧠 [MEMORY-INIT] FCLManager has connectome_manager: {fcl_manager.connectome_manager is not None}")
        else:
            logger.info(f"🧠 [MEMORY-INIT] FCLManager does NOT have connectome_manager attribute")
        
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
                mem_debug = self._is_mem_debug_enabled()
                if mem_debug:
                    logger.info(f"🧠 [MEMORY] MEMORY PROCESSING START: Burst {current_burst}")
                    logger.info(f"🧠 [MEMORY] Active memory areas: {list(self.active_memory_areas)}")
                    for area_id in self.active_memory_areas:
                        props = self.memory_area_properties.get(area_id, {})
                        logger.info(f"🧠 [MEMORY] Memory area {area_id}: temporal_depth={props.get('temporal_depth')}, upstream_areas={props.get('upstream_areas')}")
                
                if not self.active_memory_areas:
                    if mem_debug:
                        logger.info(f"🧠 [MEMORY] No active memory areas to process")
                    # Still perform global aging/lifecycle to allow short-term memories to expire
                    lifecycle_result = self._perform_aging_and_lifecycle(current_burst)
                    self.stats.memory_neurons_died += lifecycle_result.get("neurons_died", 0)
                    self.stats.memory_neurons_converted_to_longterm += lifecycle_result.get("neurons_converted", 0)
                    if mem_debug:
                        # Dump snapshots for all known memory areas (properties keys)
                        for area_id in list(self.memory_area_properties.keys()):
                            self._debug_log_memory_area_snapshot(area_id)
                    self.stats.processing_time_ms = (time.time() - start_time) * 1000
                    return {
                        "success": True,
                        "processed_areas": 0,
                        "processing_time_ms": self.stats.processing_time_ms,
                        "stats": {
                            "patterns_processed": 0,
                            "neurons_created": 0,
                            "neurons_reactivated": 0,
                            "neurons_died": lifecycle_result.get("neurons_died", 0),
                            "neurons_converted": lifecycle_result.get("neurons_converted", 0),
                        },
                    }
                
                # Process memory areas in batches
                memory_areas = list(self.active_memory_areas)
                total_processed = 0
                batch_results = []
                
                for i in range(0, len(memory_areas), self.batch_size):
                    batch = memory_areas[i:i + self.batch_size]
                    if mem_debug:
                        logger.info(f"🧠 [MEMORY] Processing memory batch {i//self.batch_size + 1}: {batch}")
                    
                    batch_result = self._process_memory_area_batch(batch, current_burst)
                    batch_results.append(batch_result)
                    total_processed += len(batch)
                
                # Aggregate results
                total_patterns = sum(result.get("patterns_processed", 0) for result in batch_results)
                total_created = sum(result.get("neurons_created", 0) for result in batch_results)
                total_reactivated = sum(result.get("neurons_reactivated", 0) for result in batch_results)
                
                # Perform aging and lifecycle management
                if mem_debug:
                    logger.info(f"🧠 [MEMORY] Performing memory neuron aging and lifecycle management")
                lifecycle_result = self._perform_aging_and_lifecycle(current_burst)
                
                if mem_debug:
                    # Dump snapshots after lifecycle
                    for area_id in memory_areas:
                        self._debug_log_memory_area_snapshot(area_id)

                # Update statistics
                self.stats.total_patterns_processed += total_patterns
                self.stats.memory_neurons_created += total_created
                self.stats.memory_neurons_reactivated += total_reactivated
                self.stats.memory_neurons_died += lifecycle_result.get("neurons_died", 0)
                self.stats.memory_neurons_converted_to_longterm += lifecycle_result.get("neurons_converted", 0)
                self.stats.processing_time_ms = (time.time() - start_time) * 1000
                
                if mem_debug:
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

    def _is_mem_debug_enabled(self) -> bool:
        """Check if memory debugging is enabled via --debug-mem flag only."""
        try:
            from feagi.core.state_manager import get_state_manager
            state_manager = get_state_manager()

            # Primary: memory-specific flag in state manager
            if hasattr(state_manager, 'is_mem_debug_enabled') and callable(state_manager.is_mem_debug_enabled):
                if state_manager.is_mem_debug_enabled():
                    return True

            # Secondary: mem_debug from loaded config
            try:
                from feagi.config.toml_loader import get_config_manager
                config_manager = get_config_manager()
                if config_manager:
                    config = config_manager.get_cached_config()
                    if config.get("mem_debug", False):
                        return True
            except Exception:
                pass

        except Exception:
            pass

        # Last resort: explicit CLI flag
        if "--debug-mem" in sys.argv:
            return True

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
        
        mem_debug = self._is_mem_debug_enabled()
        
        if mem_debug:
            logger.info(f"🧠 [MEMORY] Processing memory area {memory_area_id}")
        
        # Get memory area properties
        area_properties = self.memory_area_properties.get(memory_area_id, {})
        temporal_depth = area_properties.get('temporal_depth', 1)
        initial_lifespan = area_properties.get('initial_lifespan', 9)
        lifespan_growth_rate = area_properties.get('lifespan_growth_rate', 1.0)
        longterm_threshold = area_properties.get('longterm_threshold', 100)
        upstream_areas = area_properties.get('upstream_areas', set())
        
        # CRITICAL FIX: If upstream_areas is empty, discover them dynamically
        if not upstream_areas:
            logger.info(f"🔍 [MEMORY] No cached upstream areas for {memory_area_id}, discovering dynamically...")
            upstream_areas = self._discover_upstream_areas(memory_area_id)
            
            # Update the cached properties to avoid repeated discovery
            if upstream_areas:
                self.memory_area_properties[memory_area_id]["upstream_areas"] = upstream_areas
                logger.info(f"🔍 [MEMORY] Updated cached upstream areas for {memory_area_id}: {upstream_areas}")
        
        if mem_debug:
            logger.info(f"🧠 [MEMORY] Memory area {memory_area_id}: temporal_depth={temporal_depth}, upstream_areas={upstream_areas}")
        
        # 1. Extract temporal pattern from upstream areas
        temporal_pattern = self._extract_temporal_pattern(upstream_areas, temporal_depth, current_burst)
        
        if temporal_pattern:
            stats['patterns_processed'] = 1
            if mem_debug:
                logger.info(f"🧠 [MEMORY] Pattern detected for {memory_area_id}: creating/reactivating memory neuron")
                logger.info(f"🔍 [MEMORY] PATTERN DEBUG: {temporal_pattern}")
                logger.info(f"🔍 [MEMORY] Pattern serialized: {temporal_pattern.serialize() if hasattr(temporal_pattern, 'serialize') else 'No serialize method'}")
            
            # 2. Find or create memory neuron for this pattern
            existing_neuron_idx = self._find_or_cache_pattern(temporal_pattern)
            
            if mem_debug:
                if existing_neuron_idx is not None:
                    logger.info(f"🔍 [MEMORY] Pattern lookup result: FOUND existing neuron #{existing_neuron_idx}")
                else:
                    logger.info(f"🔍 [MEMORY] Pattern lookup result: NOT FOUND - will create NEW neuron")
            
            if existing_neuron_idx is not None:
                # EXISTING neuron found - reactivate it (apply additive lifespan growth)
                reactivated = self.memory_neuron_array.reactivate_memory_neuron(existing_neuron_idx, current_burst)
                if reactivated:
                    stats['neurons_reactivated'] = 1
                    if mem_debug:
                        logger.info(f"🌟 [MEMORY] Pattern found: reactivated existing memory neuron {existing_neuron_idx}")
                        logger.info(f"🔄 [MEMORY] EXISTING PATTERN detected - no new neuron created")
            else:
                # NO existing neuron - create a new one
                try:
                    new_neuron_idx = self.memory_neuron_array.create_memory_neuron(
                        pattern_key=temporal_pattern,
                        cortical_area_id=memory_area_id,
                        current_burst=current_burst,
                        initial_lifespan=initial_lifespan,
                        lifespan_growth_rate=lifespan_growth_rate
                    )
                    stats['neurons_created'] = 1
                    if mem_debug:
                        logger.info(f"⭐ ⭐ ⭐ [MEMORY] 🆕 NEW MEMORY NEURON BORN! ⭐ ⭐ ⭐")
                        logger.info(f"⭐ [MEMORY] Created memory neuron #{new_neuron_idx} for area {memory_area_id}")
                        logger.info(f"⭐ [MEMORY] Pattern: {temporal_pattern}")
                        logger.info(f"⭐ [MEMORY] This is a COMPLETELY NEW pattern - neuron count should increase!")
                    
                    # CRITICAL FIX: Update StateManager neuron count
                    self._update_state_manager_neuron_count(increment=1)
                    
                    # Add new neuron to pattern cache
                    self._add_to_pattern_cache(temporal_pattern, new_neuron_idx)
                    
                except Exception as e:
                    logger.error(f"🚨 [MEMORY] CRITICAL ERROR: Failed to create memory neuron for {memory_area_id}: {e}")
                    logger.error(f"🚨 [MEMORY] Exception details: {type(e).__name__}: {str(e)}")
                    import traceback
                    logger.error(f"🚨 [MEMORY] Full traceback: {traceback.format_exc()}")
                    # Don't let memory neuron creation failure block the entire process
            
            # 3. CRITICAL FIX: Inject active memory neurons into FCL for visualization
            # Memory neurons must fire to be visible to FQ Sampler
            if mem_debug:
                logger.info(f"�� [MEMORY] Injecting reactivated memory neurons into FCL for {memory_area_id}")
            
            # For now, inject a single representative neuron (index 0) - proper implementation would 
            # get actual memory neuron indices from the pattern cache
            self._inject_memory_neurons_into_fcl(memory_area_id, [0], current_burst)
        else:
            if mem_debug:
                logger.info(f"🧠 [MEMORY] No temporal pattern detected for memory area {memory_area_id} (no upstream activity)")
        
        # Aging and long-term conversion occur once per burst in batch lifecycle processing
        
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
            mem_debug = self._is_mem_debug_enabled()
            if mem_debug:
                logger.info(f"🧠 [MEMORY] PATTERN EXTRACTION: upstream_areas={upstream_areas}, temporal_depth={temporal_depth}, current_burst={current_burst}")
            
            if not upstream_areas:
                if mem_debug:
                    logger.info(f"🧠 [MEMORY] No upstream areas - cannot extract pattern")
                return None
            
            pattern_bitmaps = []
            valid_areas = []
            
            # Extract patterns for the specified temporal depth
            for timestep_offset in range(temporal_depth):
                timestep = current_burst - timestep_offset
                if timestep < 0:
                    if mem_debug:
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
                            if mem_debug:
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
                            
                            if mem_debug:
                                neuron_count = len(area_bitmap)
                                # Show first 10 neuron IDs for debugging
                                firing_neurons = list(area_bitmap)[:10]
                                more_indicator = "..." if len(area_bitmap) > 10 else ""
                                logger.info(f"🔍 [MEMORY] Timestep {timestep}: area {upstream_area_id} has {neuron_count} firing neurons: {firing_neurons}{more_indicator}")
                                logger.info(f"🧠 [MEMORY] Timestep {timestep}: area {upstream_area_id} has {neuron_count} firing neurons")
                    except Exception as area_error:
                        if mem_debug:
                            logger.warning(f"🧠 [MEMORY] Error processing upstream area {upstream_area_id}: {area_error}")
                        continue
                
                if mem_debug:
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
                if mem_debug:
                    logger.info(f"🧠 [MEMORY] No meaningful patterns found - all timesteps empty")
                return None
            
            # Create pattern key
            pattern_key = MemoryPatternKey(
                pattern_data=tuple(pattern_bitmaps),
                temporal_depth=temporal_depth,
                source_cortical_areas=tuple(sorted(set(valid_areas)))
            )
            
            if mem_debug:
                logger.info(f"🧠 [MEMORY] PATTERN CREATED: {len(non_empty_patterns)}/{temporal_depth} non-empty timesteps, source_areas={pattern_key.source_cortical_areas}")
                logger.info(f"🔍 [MEMORY] Pattern details: temporal_depth={temporal_depth}, pattern_data_length={len(pattern_key.pattern_data)}")
                # Show pattern fingerprint for debugging
                pattern_fingerprint = hash(pattern_key.pattern_data) % 10000  # Last 4 digits of hash
                logger.info(f"🔍 [MEMORY] Pattern fingerprint: #{pattern_fingerprint:04d} (for pattern comparison)")
            
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
    
    def _update_state_manager_neuron_count(self, increment: int) -> None:
        """
        Update StateManager neuron count when memory neurons are created/destroyed.
        
        Args:
            increment: Number of neurons added (positive) or removed (negative)
        """
        try:
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            
            # Get current brain stats
            current_stats = state_manager.get_brain_stats() or {}
            total = int(current_stats.get("neuron_count", 0))
            mem = int(current_stats.get("memory_neuron_count", 0))
            reg = int(current_stats.get("non_memory_neuron_count", max(0, total - mem)))

            # Apply increment to memory count and total
            mem_new = max(0, mem + increment)
            total_new = max(0, total + increment)
            reg_new = max(0, total_new - mem_new)

            updated_stats = current_stats.copy()
            updated_stats["neuron_count"] = total_new
            updated_stats["memory_neuron_count"] = mem_new
            updated_stats["non_memory_neuron_count"] = reg_new
            
            # Update StateManager
            result = state_manager.set_brain_stats(updated_stats)
            
            # DEBUG: Detailed result type analysis
            logger.info(f"🔍 [MEMORY] StateManager result type: {type(result)}")
            logger.info(f"🔍 [MEMORY] StateManager result value: {result}")
            logger.info(f"🔍 [MEMORY] Has is_err attr: {hasattr(result, 'is_err')}")
            if hasattr(result, 'is_err'):
                logger.info(f"🔍 [MEMORY] is_err callable: {callable(getattr(result, 'is_err'))}")
            
            # Handle both boolean and Result return types for compatibility
            if hasattr(result, 'is_ok') and hasattr(result, 'is_err'):
                # Result object (Rust-style)
                logger.info(f"🔍 [MEMORY] Processing as Result object")
                if result.is_err:
                    try:
                        err = result.unwrap_err()
                    except Exception:
                        err = "unknown"
                    logger.error(f"🚨 [MEMORY] FAILED to update StateManager counts: {err}")
                else:
                    logger.info(
                        f"⭐ [MEMORY] ✅ StateManager counts updated: total {total}→{total_new} (Δ{increment:+d}), "
                        f"memory {mem}→{mem_new}, regular {reg}→{reg_new}"
                    )
            elif isinstance(result, bool):
                # Boolean return type
                logger.info(f"🔍 [MEMORY] Processing as boolean: {result}")
                if not result:
                    logger.error(f"🚨 [MEMORY] FAILED to update StateManager neuron count: returned False")
                else:
                    logger.info(
                        f"⭐ [MEMORY] ✅ StateManager counts updated: total {total}→{total_new} (Δ{increment:+d}), "
                        f"memory {mem}→{mem_new}, regular {reg}→{reg_new}"
                    )
            else:
                # Unknown return type, log for debugging but assume failure
                logger.error(f"🚨 [MEMORY] StateManager returned unexpected type: {type(result)} = {result}")
                logger.error(f"🚨 [MEMORY] Cannot verify if neuron count update succeeded!")
                
        except Exception as e:
            logger.error(f"🚨 [MEMORY] CRITICAL ERROR updating StateManager neuron count: {e}")
            import traceback
            logger.error(f"🚨 [MEMORY] StateManager update traceback: {traceback.format_exc()}") 

    # --- Added helper methods for upstream discovery, lifecycle, cortical mapping, and FCL injection ---

    def _discover_upstream_areas(self, memory_cortical_id: str) -> Set[str]:
        """Discover upstream cortical areas for a given memory cortical area.

        This uses the `ConnectomeManager` authoritative mapping via
        `get_upstream_areas_for_memory`. Returns an empty set if unavailable.

        Args:
            memory_cortical_id: Memory cortical area ID

        Returns:
            Set of upstream cortical area IDs.
        """
        try:
            if self.connectome_manager and hasattr(self.connectome_manager, "get_upstream_areas_for_memory"):
                upstream: Set[str] = self.connectome_manager.get_upstream_areas_for_memory(memory_cortical_id)
                return upstream or set()
        except Exception as e:
            logger.error(f"[MEMORY] Error discovering upstream areas for {memory_cortical_id}: {e}")
        return set()

    def _perform_aging_and_lifecycle(self, current_burst: int) -> Dict[str, int]:
        """Run memory neuron aging and long-term conversion lifecycle.

        Ages all active memory neurons and applies long-term conversion checks based
        on configured thresholds per memory area.

        Args:
            current_burst: Current burst number

        Returns:
            Dict with counts: {"neurons_died": int, "neurons_converted": int}
        """
        neurons_died = 0
        neurons_converted: Set[int] = set()

        try:
            mem_debug = self._is_mem_debug_enabled()
            # Pre-aging diagnostics
            try:
                pre_stats = self.memory_neuron_array.get_statistics()
            except Exception:
                pre_stats = {"total_active_neurons": -1}
            try:
                from feagi.core.state_manager import FeagiStateManager
                sm = FeagiStateManager.instance()
                pre_brain = sm.get_brain_stats() or {}
            except Exception:
                pre_brain = {}

            # Age neurons once per burst
            died = self.memory_neuron_array.age_memory_neurons(current_burst)
            neurons_died = len(died)
            if mem_debug:
                sample = died[:10]
                logger.info(
                    f"🧠 [MEMORY] Aging results: died={neurons_died}, sample={sample}, pre_active={pre_stats.get('total_active_neurons')}"
                )
            if neurons_died:
                # Reflect deaths in global stats
                self._update_state_manager_neuron_count(increment=-neurons_died)
                if mem_debug:
                    try:
                        post_brain = sm.get_brain_stats() if sm else {}
                    except Exception:
                        post_brain = {}
                    logger.info(
                        f"🧠 [MEMORY] Brain stats update after deaths: before={pre_brain}, after={post_brain}"
                    )
            elif mem_debug:
                logger.info("🧠 [MEMORY] No neurons died this burst; brain stats unchanged by aging")

            # Apply long-term conversion using configured thresholds per area
            # Note: MemoryNeuronArray operates globally; we invoke conversion check
            # per distinct threshold to approximate area-specific policies.
            thresholds: Set[int] = set()
            for props in self.memory_area_properties.values():
                try:
                    thresholds.add(int(props.get("longterm_threshold", 100)))
                except Exception:
                    thresholds.add(100)

            for threshold in sorted(thresholds):
                converted = self.memory_neuron_array.check_longterm_conversion(longterm_threshold=threshold)
                for idx in converted:
                    neurons_converted.add(idx)
            if mem_debug and neurons_converted:
                logger.info(f"🧠 [MEMORY] Long-term conversions this burst: {len(neurons_converted)} (ids sample: {list(neurons_converted)[:10]})")

        except Exception as e:
            logger.error(f"🧠 [MEMORY] Error in aging/lifecycle processing: {e}")

        return {"neurons_died": neurons_died, "neurons_converted": len(neurons_converted)}

    def _get_cortical_idx_for_area(self, cortical_id: str) -> Optional[int]:
        """Resolve cortical_idx for a given cortical_id via ConnectomeManager.

        Args:
            cortical_id: Cortical area ID

        Returns:
            Integer cortical_idx or None if not found.
        """
        try:
            if self.connectome_manager and hasattr(self.connectome_manager, "get_cortical_idx_for_id"):
                return self.connectome_manager.get_cortical_idx_for_id(cortical_id)
        except Exception as e:
            logger.error(f"🧠 [MEMORY] Error resolving cortical_idx for {cortical_id}: {e}")
        return None

    def _inject_memory_neurons_into_fcl(
        self, memory_area_id: str, neuron_indices: List[int], current_burst: int
    ) -> None:
        """Inject a signal into the FCL to mark a memory area as active.

        This uses a conservative approach that avoids overwriting per-area FCL
        structures. We avoid injecting synthetic neuron IDs to keep FCL semantics
        strict until a dedicated mapping for memory neurons is established.

        Args:
            memory_area_id: Memory cortical area ID
            neuron_indices: Memory neuron indices
            current_burst: Current burst number
        """
        # Intentionally left as a no-op to maintain strict FCL semantics.
        # Memory neuron to FCL integration will use proper ID mapping in future work.
        return 

    def _debug_log_memory_area_snapshot(self, memory_area_id: str, include_inactive: bool = True, limit: int = 0) -> None:
        """
        Log a detailed snapshot of memory neurons for a given area.

        Args:
            memory_area_id: Memory cortical area ID to inspect
            include_inactive: Whether to include inactive neurons (default: True)
            limit: If >0, limit the number of rows logged (0 = no limit)
        """
        try:
            n = self.memory_neuron_array.next_available_index
            if n == 0:
                logger.info(f"[MEMORY] Snapshot for {memory_area_id}: no neurons available")
                return
            rows = []
            arr = self.memory_neuron_array
            for idx in range(n):
                if arr.cortical_area_id[idx] != memory_area_id:
                    continue
                if not include_inactive and not arr.is_active[idx]:
                    continue
                rows.append(
                    (
                        idx,
                        bool(arr.is_active[idx]),
                        int(arr.lifespan_current[idx]),
                        int(arr.lifespan_initial[idx]),
                        float(arr.lifespan_growth_rate[idx]),
                        bool(arr.is_longterm_memory[idx]),
                        int(arr.creation_burst[idx]),
                        int(arr.last_activation_burst[idx]),
                        int(arr.activation_count[idx]),
                    )
                )
            # Sort by lifespan_current ascending (soonest to die first)
            rows.sort(key=lambda r: r[2])
            total = len(rows)
            if total == 0:
                logger.info(f"[MEMORY] Snapshot for {memory_area_id}: no neurons in area yet")
                return
            # Apply limit
            view = rows if limit <= 0 else rows[:limit]
            logger.info(
                f"[MEMORY] Snapshot for {memory_area_id}: total={total} (showing={len(view)}), columns=(idx,active,life,current_init,gr,ltm,created,last_act,acts)"
            )
            for (idx, active, life_cur, life_init, gr, ltm, created, last_act, acts) in view:
                logger.info(
                    f"[MEMORY]   idx={idx:5d} active={int(active)} life={life_cur}/{life_init} gr={gr:.2f} ltm={int(ltm)} created={created} last={last_act} acts={acts}"
                )
            # Summary stats
            active_count = sum(1 for r in rows if r[1])
            longterm_count = sum(1 for r in rows if r[5])
            logger.info(
                f"[MEMORY] Summary for {memory_area_id}: active={active_count}, longterm={longterm_count}, inactive={total - active_count}"
            )
        except Exception as e:
            logger.debug(f"[MEMORY] Snapshot error for {memory_area_id}: {e}") 