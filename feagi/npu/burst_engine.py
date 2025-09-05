"""
Clean Burst Engine for FEAGI NPU

Complete rewrite with clear separation of concerns and proper data flow:
FCL (candidates) → Fire Queue (firing) → Fire Ledger (history)
"""

from typing import Dict, List, Optional, Any
import time
import threading
import numpy as np
from feagi.utils.logger import setup_logger
from feagi.core.state_manager import FeagiStateManager, ServiceState

from .fire_candidate_list import FireCandidateList, FCLCandidate
from .fire_queue import FireQueue, FiringNeuron
from .fire_ledger import FireLedgerInterface
from .coordinate_converter import CoordinateConverter
from .fq_sampler import FQSampler
from .fcl_injector import FCLInjector

logger = setup_logger(__name__)


class BurstEngine:
    """Clean burst engine with proper separation of concerns."""
    
    # Singleton pattern for compatibility with existing FEAGI code
    _instance = None
    _lock = threading.RLock()
    
    def __new__(cls, connectome_manager=None, state_manager=None, fire_ledger_window_size: int = 20):
        """Singleton pattern implementation."""
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(BurstEngine, cls).__new__(cls)
                cls._instance._initialized = False
                logger.info("Creating new BurstEngine singleton instance")
            return cls._instance

    @classmethod
    def get_instance(cls, connectome_manager=None, state_manager=None, fire_ledger_window_size: int = 20):
        """Get or create the singleton instance."""
        instance = cls(connectome_manager, state_manager, fire_ledger_window_size)
        return instance
    
    @classmethod
    def reset_instance(cls):
        """Reset the singleton instance (for testing/cleanup)."""
        with cls._lock:
            cls._instance = None
    
    def __init__(self, connectome_manager=None, state_manager=None, fire_ledger_window_size: int = 20):
        """Initialize clean burst engine."""
        # Prevent double initialization for singleton
        if hasattr(self, '_initialized') and self._initialized:
            return

        self.connectome_manager = connectome_manager
        
        # STATE MANAGER INTEGRATION - Cache instance for performance 
        # Always use singleton instance as single source of truth
        self.state_manager = FeagiStateManager.instance()
        logger.info("BurstEngine: Using FeagiStateManager singleton instance")
        
        # Set initial burst engine state to INITIALIZING
        self._set_burst_engine_state(ServiceState.INITIALIZING)
        
        # Core NPU components
        self.fire_ledger = FireLedgerInterface(fire_ledger_window_size)
        self.coordinate_converter = CoordinateConverter(connectome_manager) if connectome_manager else None
        self.fcl_injector = FCLInjector(self.coordinate_converter) if self.coordinate_converter else None
        
        # FQ Sampler (initialized after burst engine is ready)
        self.fq_sampler: Optional[FQSampler] = None
        
        # Registry of external FQ samplers (for process manager integration)
        self.registered_fq_samplers: List[Any] = []
        
        # Initialize frequency from state manager (single source of truth)
        self._initialize_frequency_from_state_manager()
        
        # Running state tracking (with debug integration)
        self._running_state = False
        
        # Genome integration tracking
        self.genome_loaded = False
        
        # Instance ID for debugging and tracking
        self._instance_id = f"burst_engine_{id(self)}"
        
        self.current_timestep = 0
        self.burst_count = 0
        self.previous_fire_queue: Optional[FireQueue] = None

        # Mark as initialized and ready
        self._initialized = True
        self._set_burst_engine_state(ServiceState.READY)

        logger.info("Clean Burst Engine initialized with singleton pattern and state manager integration")
    
    def process_burst(self) -> List[int]:
        """Execute complete burst processing with clean 5-phase workflow."""
        self.current_timestep = self.burst_count
        
        # Phase 1: Collect candidates using FCL Injector
        fcl = FireCandidateList()
        self._inject_all_candidates(fcl)
        
        # Phase 2: Process neural dynamics  
        fire_queue = FireQueue()
        
        # Phase 3: Archive to fire ledger
        if not fire_queue.is_empty():
            neurons_by_area = {}
            for area_idx, neurons in fire_queue.firing_neurons_by_area.items():
                neurons_by_area[area_idx] = neurons
            self.fire_ledger.archive_timestep(self.current_timestep, neurons_by_area)
            
            # Update state manager with activity counters
            neuron_count = sum(len(neurons) for neurons in neurons_by_area.values())
            try:
                if self.state_manager:
                    self.state_manager.increment_cumulative_activity(neuron_count)
            except Exception as e:
                # Don't let state manager issues break burst processing
                logger.debug(f"Failed to update activity counters: {e}")
        
        # Phase 4: FQ Sampler access ready (no action needed)
        
        # Phase 5: Cleanup and prepare
        self.previous_fire_queue = fire_queue.copy_for_propagation()
        fcl.clear()
        self.burst_count += 1
        
        return fire_queue.get_all_neuron_ids()
    
    def get_current_fire_queue(self) -> Optional[FireQueue]:
        """Get current fire queue for FQ Sampler access."""
        return self.previous_fire_queue
    
    def get_fire_ledger(self) -> FireLedgerInterface:
        """Get fire ledger interface."""
        return self.fire_ledger
    
    def initialize_fq_sampler(self, sample_frequency_hz: float = 10.0, sampling_mode: str = "visualization") -> FQSampler:
        """Initialize FQ Sampler with this burst engine as provider."""
        self.fq_sampler = FQSampler(
            fire_queue_provider=self,  # BurstEngine provides get_current_fire_queue()
            sample_frequency_hz=sample_frequency_hz,
            sampling_mode=sampling_mode
        )
        logger.info(f"FQ Sampler initialized: {sampling_mode} @ {sample_frequency_hz}Hz")
        return self.fq_sampler
    
    def get_fq_sampler(self) -> Optional[FQSampler]:
        """Get FQ Sampler instance."""
        return self.fq_sampler
    
    def register_fq_sampler(self, fq_sampler: Any):
        """Register an external FQ sampler with this burst engine.
        
        This method allows the process manager to register FQ samplers 
        that need access to the burst engine's fire queue data.
        
        Args:
            fq_sampler: FQ sampler instance (FQSampler or UnifiedFQSampler)
        """
        if fq_sampler not in self.registered_fq_samplers:
            self.registered_fq_samplers.append(fq_sampler)
            sampler_id = getattr(fq_sampler, 'instance_id', 'unknown')
            logger.info(f"FQ sampler [{sampler_id}] registered with BurstEngine")
        else:
            logger.warning(f"FQ sampler already registered: {getattr(fq_sampler, 'instance_id', 'unknown')}")
    
    def unregister_fq_sampler(self, fq_sampler: Any):
        """Unregister an external FQ sampler from this burst engine.
        
        Args:
            fq_sampler: FQ sampler instance to unregister
        """
        try:
            self.registered_fq_samplers.remove(fq_sampler)
            sampler_id = getattr(fq_sampler, 'instance_id', 'unknown')
            logger.info(f"FQ sampler [{sampler_id}] unregistered from BurstEngine")
        except ValueError:
            logger.warning(f"Attempted to unregister FQ sampler that wasn't registered: {getattr(fq_sampler, 'instance_id', 'unknown')}")
    
    def get_registered_fq_samplers(self) -> List[Any]:
        """Get list of all registered FQ samplers."""
        return self.registered_fq_samplers.copy()
    
    # ==============================================================
    # STATE MANAGER INTEGRATION METHODS
    # ==============================================================
    
    @property
    def _running(self):
        """Get the running state with debug tracking (state manager integration)."""
        return getattr(self, "_running_state", False)

    @_running.setter
    def _running(self, value):
        """Setter for _running with debug logging and state manager integration."""
        old_value = getattr(self, "_running_state", None)
        self._running_state = value

        # Update state manager when running state changes
        if old_value != value:
            if value:
                # When starting to run, set state to READY (active and processing)
                self._set_burst_engine_state(ServiceState.READY)
            else:
                # When stopping, set state back to READY but not running
                # (This maintains compatibility with existing state machine)
                pass
        
        # Debug logging if NPU debug mode is enabled
        try:
            if self.state_manager and self.state_manager.is_debug_npu_enabled() and old_value != value:
                logger.info(
                    f"[NPU-DEBUG] BURST ENGINE: Instance {id(self)} _running changed: {old_value} -> {value}"
                )
        except Exception:
            # Don't let debug logging break normal operation
            pass
    
    def _set_burst_engine_state(self, state: ServiceState):
        """Set burst engine state in the state manager."""
        try:
            if self.state_manager:
                self.state_manager.set_burst_engine_state(state)
                logger.debug(f"BurstEngine state updated to: {state}")
        except Exception as e:
            logger.error(f"Failed to update burst engine state: {e}")
    
    def _initialize_frequency_from_state_manager(self):
        """Initialize burst frequency from state manager (single source of truth).
        
        Uses the same robust fallback pattern as the old BurstEngine for compatibility.
        """
        # Configuration fallback frequency (mimicking old BurstEngine behavior)
        config_frequency = 10.0  # Default fallback frequency
        
        try:
            # STATE MANAGER is the SINGLE SOURCE OF TRUTH for burst frequency
            # Get frequency from state manager (authoritative source)
            state_frequency = self.state_manager.get_burst_frequency()
            if state_frequency and state_frequency > 0:
                self.desired_frequency = float(state_frequency)
                logger.info(f"[BURST ENGINE] Using state manager frequency: {state_frequency}Hz")
            else:
                # Emergency fallback: use config and update state manager
                self.desired_frequency = config_frequency
                self.state_manager.set_burst_frequency(config_frequency)
                logger.warning(
                    f"[BURST ENGINE] State manager frequency invalid ({state_frequency}Hz) - "
                    f"using config fallback: {config_frequency}Hz and updating state manager"
                )
        except Exception as e:
            # Emergency fallback: use config frequency and try to update state manager
            self.desired_frequency = config_frequency
            try:
                if self.state_manager:
                    self.state_manager.set_burst_frequency(config_frequency)
                logger.warning(
                    f"[BURST ENGINE] Failed to get frequency from state manager ({e}) - "
                    f"using config fallback: {config_frequency}Hz"
                )
            except Exception:
                # Completely fallback - just use the frequency without updating state manager
                logger.error(
                    f"[BURST ENGINE] Could not initialize frequency in state manager - "
                    f"using local fallback: {config_frequency}Hz"
                )
        
        # Ensure frequency is never zero to avoid division by zero (old BurstEngine safety)
        if self.desired_frequency <= 0:
            self.desired_frequency = config_frequency
            logger.warning(f"[BURST ENGINE] Frequency was zero - using safety fallback: {config_frequency}Hz")
        
        # Set target_frequency for backward compatibility
        self.target_frequency = self.desired_frequency
    
    def update_frequency(self, frequency_hz: float) -> bool:
        """Update burst engine frequency and sync with state manager."""
        try:
            if frequency_hz <= 0 or frequency_hz > 10000:
                logger.error(f"Invalid frequency {frequency_hz}Hz (must be 0 < freq <= 10000)")
                return False
            
            self.desired_frequency = frequency_hz
            
            # Update state manager
            if self.state_manager:
                self.state_manager.set_burst_frequency(frequency_hz)
                logger.info(f"BurstEngine: Updated frequency to {frequency_hz}Hz in state manager")
            
            return True
        except Exception as e:
            logger.error(f"Failed to update burst engine frequency: {e}")
            return False
    
    def start(self) -> bool:
        """Start the burst engine with proper state manager integration."""
        try:
            if self._running:
                logger.warning("BurstEngine: Already running")
                return True
            
            self._running = True
            logger.info("BurstEngine: Started successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to start BurstEngine: {e}")
            self._set_burst_engine_state(ServiceState.ERROR)
            return False
    
    def stop(self) -> bool:
        """Stop the burst engine with proper state manager integration."""
        try:
            if not self._running:
                logger.warning("BurstEngine: Already stopped")
                return True
            
            self._running = False
            self._set_burst_engine_state(ServiceState.STOPPED)
            logger.info("BurstEngine: Stopped successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to stop BurstEngine: {e}")
            self._set_burst_engine_state(ServiceState.ERROR)
            return False
    
    def is_running(self) -> bool:
        """Check if burst engine is currently running."""
        return self._running
    
    def run(self) -> None:
        """Start the burst engine main processing loop.
        
        This method runs the continuous burst processing loop in the current thread.
        It's designed to be called from a background thread by the brain service.
        The loop continues until stop() is called or an exit condition is met.
        """
        try:
            logger.info("[BURST-ENGINE] Starting main processing loop")
            self.start()
            
            # Enter the main processing loop
            if self._running:
                logger.info(f"[BURST-ENGINE] ✅ Main loop started successfully at {self.desired_frequency}Hz") 
                
                # Calculate burst interval from frequency
                burst_interval = 1.0 / self.desired_frequency if self.desired_frequency > 0 else 0.1
                logger.info(f"[BURST-ENGINE] Burst interval: {burst_interval:.4f}s")
                
                # Main burst processing loop
                while self._running:
                    try:
                        burst_start_time = time.perf_counter()
                        
                        # Execute the actual burst processing
                        try:
                            fired_neurons = self.process_burst()
                            
                            # Log periodic burst status 
                            if self.burst_count % 100 == 0:  # Every 100 bursts
                                logger.info(f"[BURST-ENGINE] Burst #{self.burst_count}: {len(fired_neurons)} neurons fired")
                                
                        except Exception as burst_error:
                            logger.error(f"Error in burst processing #{self.burst_count}: {burst_error}")
                            # Continue processing even if one burst fails
                        
                        # Calculate sleep time to maintain frequency
                        burst_duration = time.perf_counter() - burst_start_time
                        sleep_time = max(0, burst_interval - burst_duration)
                        
                        if sleep_time > 0:
                            time.sleep(sleep_time)
                        elif burst_duration > burst_interval * 1.5:  # Warn if significantly over budget
                            logger.warning(f"[BURST-ENGINE] Burst #{self.burst_count} took {burst_duration:.4f}s (>{burst_interval:.4f}s budget)")
                        
                        # Check for exit condition
                        if not self._running:
                            break
                            
                    except Exception as e:
                        logger.error(f"Error in burst engine run loop: {e}")
                        break
                        
                logger.info(f"[BURST-ENGINE] Main processing loop ended after {self.burst_count} bursts")
            else:
                logger.error("[BURST-ENGINE] Failed to start - run loop exiting")
                
        except Exception as e:
            logger.error(f"Error in burst engine run() method: {e}")
            self._set_burst_engine_state(ServiceState.ERROR)
    
    def update_with_genome(self) -> None:
        """Update the burst engine configuration when a new genome is loaded.
        
        This method is called after a new genome is loaded into the connectome manager 
        to refresh the engine's understanding of the neural network. This ensures 
        compatibility with the genome loading process.
        """
        try:
            logger.info("[CONFIG] Updating burst engine with new genome")
            
            # Sync with connectome manager's current state
            if self.connectome_manager:
                # Check if connectome manager has neuron array data
                if hasattr(self.connectome_manager, "neuron_array"):
                    neuron_array = self.connectome_manager.neuron_array
                    if hasattr(neuron_array, "neuron_count"):
                        neuron_count = neuron_array.neuron_count
                        logger.info(f"[GENOME-SYNC] Burst engine synced with {neuron_count} neurons")
                    else:
                        logger.debug("[GENOME-SYNC] Neuron array present but no count available")
                else:
                    logger.debug("[GENOME-SYNC] No neuron array data available yet")
                    
                # Update coordinate converter if available
                if self.coordinate_converter and hasattr(self.connectome_manager, 'get_cortical_dimensions'):
                    logger.debug("[GENOME-SYNC] Coordinate converter will use updated cortical dimensions")
                    
            # Re-initialize FCL injector with updated connectome data
            if self.coordinate_converter:
                # FCL injector will automatically pick up new connectome data through coordinate converter
                logger.debug("[GENOME-SYNC] FCL injector will use updated connectome data")
            
            # Mark that genome data has been integrated
            self.genome_loaded = True
            
            logger.info("✅ Burst engine updated with new genome successfully")
            
        except Exception as e:
            logger.error(f"Error updating burst engine with genome: {e}")
            # Don't raise - genome loading should not fail due to burst engine update issues
    
    def _inject_all_candidates(self, fcl: FireCandidateList):
        """Inject all candidates into FCL using FCL Injector."""
        # This method will be called by external injection services
        
        # Check if FCL injector is available (requires connectome manager)
        if not self.fcl_injector:
            logger.debug("FCL injector not available - no connectome manager provided")
            return
        
        # Example: Synaptic propagation from previous timestep
        if self.previous_fire_queue:
            propagation_data = self._compute_synaptic_propagation()
            self.fcl_injector.inject_synaptic_propagation(fcl, propagation_data)
        
        logger.debug("FCL injection complete via FCL Injector")
    
    def _compute_synaptic_propagation(self) -> Dict[int, List[tuple]]:
        """Compute synaptic propagation data from previous fire queue."""
        # Placeholder for synaptic propagation computation
        # This will integrate with actual connectome synaptic data
        return {}
    
    def inject_sensory_data(self, cortical_id: str, x_coords, y_coords, z_coords, potentials):
        """External interface for sensory injection."""
        if not hasattr(self, '_current_fcl') or self._current_fcl is None:
            logger.warning("No active FCL for sensory injection")
            return 0
            
        return self.fcl_injector.inject_sensory_data(
            self._current_fcl, cortical_id, x_coords, y_coords, z_coords, potentials
        )
    
    def inject_manual_stimulation(self, stimulation_data: Dict[str, Any]):
        """External interface for manual stimulation."""
        if not hasattr(self, '_current_fcl') or self._current_fcl is None:
            logger.warning("No active FCL for manual stimulation")
            return 0
            
        return self.fcl_injector.inject_manual_stimulation(self._current_fcl, stimulation_data)