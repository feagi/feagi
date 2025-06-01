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
FCL Injection Service for FEAGI Neural Processing Unit.

Handles injection of neuron candidates from special areas (power areas, sensory input,
modulators, etc.) into the Fire Candidate List (FCL) during burst processing. Provides
optimized batch injection and proper timing coordination with the burst engine.

This service implements the unified FCL candidate model:
- Special areas add candidates to FCL (rather than firing directly)
- All candidates (internal synaptic + external special areas) processed together
- Burst engine remains completely area-agnostic

@cursor:critical-path FCL injection affects every burst cycle - performance critical
@cursor:ffi-safe Uses static types and no dynamic allocation in main loops for Rust compatibility
"""

import time
from typing import Dict, List, Optional, Set, Any, Union
from enum import Enum
from dataclasses import dataclass

from feagi.utils.logger import setup_logger
from feagi.npu.special_area_handler import SpecialAreaHandler, CorticalId, NeuronId

logger = setup_logger()


class InjectionTiming(Enum):
    """Enumeration of injection timing phases."""
    PRE_BURST = "pre_burst"
    DURING_BURST = "during_burst" 
    POST_BURST = "post_burst"


@dataclass
class InjectionBatch:
    """A batch of neurons to inject into the FCL."""
    cortical_id: CorticalId
    neuron_ids: List[NeuronId]
    timing: InjectionTiming
    probability: float = 1.0


class FCLInjectionService:
    """
    Service for injecting neuron candidates from special areas into the FCL.
    
    This service coordinates with the SpecialAreaHandler to identify special areas
    (power areas, sensory inputs, modulators, etc.) and adds their neurons as
    candidates to the FCL at appropriate timing during burst processing.
    
    Implements the unified FCL candidate model:
    - External candidates are added to FCL (not fired directly)
    - All FCL candidates processed together by connectome manager
    - Supports extensible special area types through timing phases
    - Burst engine remains completely area-agnostic
    
    Key features:
    - Batch injection for performance
    - Configurable timing (pre/during/post burst)
    - Probabilistic injection support
    - Performance monitoring and statistics
    - Extensible to any special area type
    """
    
    def __init__(self, fcl_manager: Any, special_area_handler: SpecialAreaHandler, 
                 config: Optional[Dict[str, Any]] = None):
        """
        Initialize the FCL injection service.
        
        Args:
            fcl_manager: The FCL manager instance
            special_area_handler: Handler for special areas
            config: Optional configuration parameters
        """
        self.fcl_manager = fcl_manager
        self.special_area_handler = special_area_handler
        self.config = config or {}
        
        # Performance configuration
        self.batch_size = self.config.get('batch_injection_size', 1000)
        self.enable_probabilistic = self.config.get('enable_probabilistic_injection', True)
        self.enable_timing_optimization = self.config.get('enable_timing_optimization', True)
        
        # Pre-computed injection batches for performance
        self._injection_batches: Dict[InjectionTiming, List[InjectionBatch]] = {
            InjectionTiming.PRE_BURST: [],
            InjectionTiming.DURING_BURST: [],
            InjectionTiming.POST_BURST: []
        }
        
        # Statistics
        self.total_injections = 0
        self.total_neurons_injected = 0
        self.injection_timing_stats = {timing: 0 for timing in InjectionTiming}
        self.last_injection_duration = 0.0
        
        # Performance optimization: pre-allocate injection data structures
        self._prepare_injection_batches()
        
        logger.info("FCL Injection Service initialized", status="[CONFIG]")
    
    def _prepare_injection_batches(self) -> None:
        """
        Pre-compute injection batches for all special areas.
        
        This optimization pre-allocates and caches injection data to minimize
        runtime overhead during burst processing.
        """
        # Clear existing batches
        for timing in InjectionTiming:
            self._injection_batches[timing].clear()
        
        # Get all power areas and create injection batches
        power_neurons = self.special_area_handler.get_all_power_neurons()
        logger.info(f"Preparing injection batches for {len(power_neurons)} power areas: {list(power_neurons.keys())}")
        
        for cortical_id, neuron_ids in power_neurons.items():
            logger.debug(f"Processing power area {cortical_id} with {len(neuron_ids)} neurons: {neuron_ids}")
            
            config = self.special_area_handler.get_special_config(cortical_id)
            if not config:
                logger.warning(f"No config found for power area {cortical_id}")
                continue
            if not config.enabled:
                logger.info(f"Power area {cortical_id} is disabled, skipping")
                continue
            
            # Determine timing
            timing_str = config.injection_timing
            try:
                timing = InjectionTiming(timing_str)
                logger.debug(f"Power area {cortical_id} uses {timing_str} timing")
            except ValueError:
                logger.warning(f"Invalid injection timing '{timing_str}' for area {cortical_id}, using PRE_BURST")
                timing = InjectionTiming.PRE_BURST
            
            # Create batches if needed (split large neuron lists)
            if len(neuron_ids) <= self.batch_size:
                # Single batch
                batch = InjectionBatch(
                    cortical_id=cortical_id,
                    neuron_ids=neuron_ids.copy(),
                    timing=timing,
                    probability=config.injection_probability
                )
                self._injection_batches[timing].append(batch)
                logger.info(f"Created single batch for {cortical_id}: {len(neuron_ids)} neurons, timing={timing.value}, prob={config.injection_probability}")
            else:
                # Multiple batches
                batch_count = 0
                for i in range(0, len(neuron_ids), self.batch_size):
                    batch_neurons = neuron_ids[i:i + self.batch_size]
                    batch = InjectionBatch(
                        cortical_id=f"{cortical_id}_batch_{i//self.batch_size}",
                        neuron_ids=batch_neurons,
                        timing=timing,
                        probability=config.injection_probability
                    )
                    self._injection_batches[timing].append(batch)
                    batch_count += 1
                logger.info(f"Created {batch_count} batches for {cortical_id}: {len(neuron_ids)} neurons total")
        
        # Log preparation results
        total_batches = sum(len(batches) for batches in self._injection_batches.values())
        batch_summary = {timing.value: len(batches) for timing, batches in self._injection_batches.items()}
        logger.info(f"Prepared {total_batches} injection batches for {len(power_neurons)} power areas: {batch_summary}", status="[SAVE]")
        
        # Log detailed batch info for debugging
        for timing, batches in self._injection_batches.items():
            if batches:
                area_info = [(batch.cortical_id, len(batch.neuron_ids)) for batch in batches]
                logger.debug(f"{timing.value} batches: {area_info}")
    
    def inject_pre_burst(self, current_timestep: int) -> int:
        """
        Add neuron candidates from special areas to FCL before burst processing.
        
        This phase typically handles power areas, sensory input, and other external
        sources that should be available as firing candidates during the burst.
        
        Args:
            current_timestep: Current simulation timestep
            
        Returns:
            Number of candidates added to FCL
        """
        return self._execute_injection_phase(InjectionTiming.PRE_BURST, current_timestep)
    
    def inject_during_burst(self, current_timestep: int) -> int:
        """
        Add neuron candidates from modulator areas during burst processing.
        
        This phase typically handles modulatory influences that should affect
        ongoing neural computation within the same burst.
        
        Args:
            current_timestep: Current simulation timestep
            
        Returns:
            Number of candidates added to FCL
        """
        return self._execute_injection_phase(InjectionTiming.DURING_BURST, current_timestep)
    
    def inject_post_burst(self, current_timestep: int) -> int:
        """
        Add neuron candidates from special areas after burst processing.
        
        This phase typically handles cleanup, memory consolidation, or other
        post-processing special behaviors.
        
        Args:
            current_timestep: Current simulation timestep
            
        Returns:
            Number of candidates added to FCL
        """
        return self._execute_injection_phase(InjectionTiming.POST_BURST, current_timestep)
    
    def _execute_injection_phase(self, timing: InjectionTiming, current_timestep: int) -> int:
        """
        Execute injection for a specific timing phase.
        
        Args:
            timing: The injection timing phase
            current_timestep: Current simulation timestep
            
        Returns:
            Number of neurons injected
        """
        if not self._injection_batches[timing]:
            logger.debug(f"No injection batches for {timing.value} phase")
            return 0
        
        logger.debug(f"Starting {timing.value} injection phase with {len(self._injection_batches[timing])} batches")
        
        start_time = time.perf_counter()
        total_injected = 0
        
        # Process all batches for this timing
        for batch in self._injection_batches[timing]:
            logger.debug(f"Processing batch for {batch.cortical_id} with {len(batch.neuron_ids)} neurons")
            candidates_added = self._inject_batch(batch, current_timestep)
            total_injected += candidates_added
            if candidates_added > 0:
                logger.info(f"Successfully added {candidates_added} candidates to FCL from {batch.cortical_id}")
        
        # Update statistics
        end_time = time.perf_counter()
        self.last_injection_duration = end_time - start_time
        self.injection_timing_stats[timing] += 1
        self.total_injections += 1
        self.total_neurons_injected += total_injected
        
        if total_injected > 0:
            logger.info(f"FCL INJECTION: Added {total_injected} candidates to FCL in {timing.value} phase ({self.last_injection_duration:.4f}s)")
        else:
            logger.debug(f"No candidates added to FCL in {timing.value} phase")
        
        return total_injected
    
    def _inject_batch(self, batch: InjectionBatch, current_timestep: int) -> int:
        """
        Add a batch of neuron candidates to the FCL.
        
        This method handles the actual addition of candidates from special areas
        to the Fire Candidate List. The candidates will be processed along with
        other FCL entries during the unified burst processing sweep.
        
        Args:
            batch: The injection batch to process
            current_timestep: Current simulation timestep
            
        Returns:
            Number of candidates added to FCL
        """
        if not batch.neuron_ids:
            return 0
        
        # Check probabilistic injection
        if self.enable_probabilistic and batch.probability < 1.0:
            import random
            if random.random() > batch.probability:
                return 0
        
        try:
            # Determine which neurons to inject (could be subset based on targeting)
            neurons_to_inject = batch.neuron_ids
            
            # Extract cortical_id (remove batch suffix if present)
            cortical_id = batch.cortical_id.split('_batch_')[0]
            
            # Add candidates directly to FCL - bypasses membrane potential checks
            # This is the unified FCL candidate model: external sources add candidates
            # that get processed together with internal synaptic propagation
            
            if hasattr(self.fcl_manager, 'add_neurons_to_fcl'):
                # Direct injection - bypasses membrane potential checks
                self.fcl_manager.add_neurons_to_fcl(cortical_id, neurons_to_inject, current_timestep)
                logger.debug(f"FCL candidate addition: Added {len(neurons_to_inject)} candidates from {cortical_id} to FCL")
            elif hasattr(self.fcl_manager, 'add_to_current_fcl'):
                # Fallback: Direct injection into current FCL
                self.fcl_manager.add_to_current_fcl(neurons_to_inject)
                logger.debug(f"FCL candidate addition (fallback): Added {len(neurons_to_inject)} candidates to FCL")
            elif hasattr(self.fcl_manager, 'update_fcl'):
                # Last resort: Force through update_fcl but mark as injected
                neurons_by_cortical = {cortical_id: neurons_to_inject}
                self.fcl_manager.update_fcl(current_timestep, neurons_by_cortical)
                logger.debug(f"FCL candidate addition (forced): Updated FCL with {len(neurons_to_inject)} candidates from {cortical_id}")
            else:
                logger.error("FCL manager does not support any known injection method")
                return 0
            
            # Record injection for special area handler statistics
            self.special_area_handler.record_injection()
            
            return len(neurons_to_inject)
            
        except Exception as e:
            logger.error(f"Error injecting batch {batch.cortical_id}: {e}")
            return 0
    
    def refresh_injection_batches(self) -> None:
        """
        Refresh injection batches when special areas change.
        
        This should be called when the connectome structure changes or
        when special areas are added/removed.
        """
        logger.info("Refreshing injection batches", status="[PROC]")
        self._prepare_injection_batches()
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about injection performance.
        
        Returns:
            Dictionary with injection statistics and performance metrics
        """
        return {
            "total_injections": self.total_injections,
            "total_neurons_injected": self.total_neurons_injected,
            "injection_timing_stats": dict(self.injection_timing_stats),
            "last_injection_duration": self.last_injection_duration,
            "prepared_batches": {
                timing.value: len(batches) 
                for timing, batches in self._injection_batches.items()
            },
            "batch_size": self.batch_size,
            "enable_probabilistic": self.enable_probabilistic
        }
    
    def set_injection_enabled(self, cortical_id: CorticalId, enabled: bool) -> bool:
        """
        Enable or disable injection for a specific cortical area.
        
        Args:
            cortical_id: The cortical area ID
            enabled: Whether to enable or disable injection
            
        Returns:
            True if the setting was applied, False if area not found
        """
        config = self.special_area_handler.get_special_config(cortical_id)
        if config:
            config.enabled = enabled
            # Refresh batches to apply the change
            self._prepare_injection_batches()
            logger.info(f"Injection {'enabled' if enabled else 'disabled'} for area {cortical_id}", status="[SETUP]")
            return True
        return False
    
    def get_power_injection_preview(self) -> Dict[str, Any]:
        """
        Get a preview of what would be injected in the next burst.
        
        Returns:
            Dictionary with preview information for debugging/monitoring
        """
        preview = {
            "pre_burst_neurons": 0,
            "during_burst_neurons": 0,
            "post_burst_neurons": 0,
            "total_batches": 0,
            "areas_involved": set()
        }
        
        for timing, batches in self._injection_batches.items():
            batch_neurons = sum(len(batch.neuron_ids) for batch in batches)
            preview["total_batches"] += len(batches)
            
            if timing == InjectionTiming.PRE_BURST:
                preview["pre_burst_neurons"] = batch_neurons
            elif timing == InjectionTiming.DURING_BURST:
                preview["during_burst_neurons"] = batch_neurons
            elif timing == InjectionTiming.POST_BURST:
                preview["post_burst_neurons"] = batch_neurons
                
            for batch in batches:
                base_id = batch.cortical_id.split('_batch_')[0]
                preview["areas_involved"].add(base_id)
        
        preview["areas_involved"] = list(preview["areas_involved"])
        return preview


# Example usage and testing functions
def example_usage():
    """Example usage of the FCLInjectionService."""
    # This would be used with real FCL manager and special area handler
    # fcl_injection = FCLInjectionService(fcl_manager, special_area_handler)
    # 
    # # In burst engine:
    # fcl_injection.inject_pre_burst(current_timestep)
    # # ... regular burst processing ...
    # fcl_injection.inject_post_burst(current_timestep)
    pass


if __name__ == "__main__":
    example_usage() 