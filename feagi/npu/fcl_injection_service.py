"""
FCL Injection Service for FEAGI Neural Processing Unit.

Handles injection of neurons from special areas (particularly power areas) into the
Fire Candidate List (FCL) during burst processing. Provides optimized batch injection
and proper timing coordination with the burst engine.

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
    Service for injecting neurons from special areas into the FCL.
    
    This service coordinates with the SpecialAreaHandler to identify power areas
    and other special areas, then injects their neurons into the FCL at the
    appropriate timing during burst processing.
    
    Key features:
    - Batch injection for performance
    - Configurable timing (pre/during/post burst)
    - Probabilistic injection support
    - Performance monitoring and statistics
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
        
        logger.info("FCL Injection Service initialized", emoji1="💉")
    
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
        
        for cortical_id, neuron_ids in power_neurons.items():
            config = self.special_area_handler.get_special_config(cortical_id)
            if not config or not config.enabled:
                continue
            
            # Determine timing
            timing_str = config.injection_timing
            try:
                timing = InjectionTiming(timing_str)
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
            else:
                # Multiple batches
                for i in range(0, len(neuron_ids), self.batch_size):
                    batch_neurons = neuron_ids[i:i + self.batch_size]
                    batch = InjectionBatch(
                        cortical_id=f"{cortical_id}_batch_{i//self.batch_size}",
                        neuron_ids=batch_neurons,
                        timing=timing,
                        probability=config.injection_probability
                    )
                    self._injection_batches[timing].append(batch)
        
        # Log preparation results
        total_batches = sum(len(batches) for batches in self._injection_batches.values())
        logger.info(f"Prepared {total_batches} injection batches for {len(power_neurons)} power areas", emoji1="📦")
    
    def inject_pre_burst(self, current_timestep: int) -> int:
        """
        Inject neurons from power areas before burst processing.
        
        Args:
            current_timestep: Current simulation timestep
            
        Returns:
            Number of neurons injected
        """
        return self._execute_injection_phase(InjectionTiming.PRE_BURST, current_timestep)
    
    def inject_during_burst(self, current_timestep: int) -> int:
        """
        Inject neurons from modulator areas during burst processing.
        
        Args:
            current_timestep: Current simulation timestep
            
        Returns:
            Number of neurons injected
        """
        return self._execute_injection_phase(InjectionTiming.DURING_BURST, current_timestep)
    
    def inject_post_burst(self, current_timestep: int) -> int:
        """
        Inject neurons from special areas after burst processing.
        
        Args:
            current_timestep: Current simulation timestep
            
        Returns:
            Number of neurons injected
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
            return 0
        
        start_time = time.perf_counter()
        total_injected = 0
        
        # Process all batches for this timing
        for batch in self._injection_batches[timing]:
            injected = self._inject_batch(batch, current_timestep)
            total_injected += injected
        
        # Update statistics
        end_time = time.perf_counter()
        self.last_injection_duration = end_time - start_time
        self.injection_timing_stats[timing] += 1
        self.total_injections += 1
        self.total_neurons_injected += total_injected
        
        if total_injected > 0:
            logger.debug(f"Injected {total_injected} neurons in {timing.value} phase ({self.last_injection_duration:.4f}s)")
        
        return total_injected
    
    def _inject_batch(self, batch: InjectionBatch, current_timestep: int) -> int:
        """
        Inject a batch of neurons into the FCL.
        
        Args:
            batch: The injection batch to process
            current_timestep: Current simulation timestep
            
        Returns:
            Number of neurons injected
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
            
            # Use update_fcl() to ensure neurons are tracked by cortical area for proper debug output
            if hasattr(self.fcl_manager, 'update_fcl'):
                # Update FCL with neuron mapping to ensure cortical-specific tracking
                neurons_by_cortical = {cortical_id: neurons_to_inject}
                self.fcl_manager.update_fcl(current_timestep, neurons_by_cortical)
            elif hasattr(self.fcl_manager, 'add_to_current_fcl'):
                # Fallback: Direct injection into current FCL (only updates global FCL)
                self.fcl_manager.add_to_current_fcl(neurons_to_inject)
            else:
                logger.warning("FCL manager does not support neuron injection")
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
        logger.info("Refreshing injection batches", emoji1="🔄")
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
            logger.info(f"Injection {'enabled' if enabled else 'disabled'} for area {cortical_id}", emoji1="⚙️")
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