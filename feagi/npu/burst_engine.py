"""
Clean Burst Engine for FEAGI NPU

Complete rewrite with clear separation of concerns and proper data flow:
FCL (candidates) → Fire Queue (firing) → Fire Ledger (history)
"""

from typing import Dict, List, Optional, Any, Union, Tuple
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
    
    # Lock-free singleton pattern for RTOS compliance
    _instance = None
    
    def __new__(cls, connectome_manager=None, state_manager=None, fire_ledger_window_size: int = 20):
        """Lock-free singleton pattern implementation."""
        if cls._instance is None:
            cls._instance = super(BurstEngine, cls).__new__(cls)
            cls._instance._initialized = False
            pass  # BurstEngine singleton created
        return cls._instance

    @classmethod
    def get_instance(cls, connectome_manager=None, state_manager=None, fire_ledger_window_size: int = 20):
        """Get or create the singleton instance."""
        instance = cls(connectome_manager, state_manager, fire_ledger_window_size)
        return instance
    
    @classmethod
    def reset_instance(cls):
        """Reset the singleton instance (for testing/cleanup)."""
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
        # BurstEngine using FeagiStateManager singleton
        
        # Set initial burst engine state to INITIALIZING
        self._set_burst_engine_state(ServiceState.INITIALIZING)
        
        # Core NPU components
        self.fire_ledger = FireLedgerInterface(fire_ledger_window_size)
        self.coordinate_converter = CoordinateConverter(connectome_manager) if connectome_manager else None
        self.fcl_injector = FCLInjector(self.coordinate_converter) if self.coordinate_converter else None
        
        # Injection service for automatic power and special area injection
        self.injection_service = None
        self.enable_injection = True  # Enable automatic injection by default
        
        # FQ Sampler (initialized after burst engine is ready)
        self.fq_sampler: Optional[FQSampler] = None
        
        # Registry of external FQ samplers (for process manager integration)
        # Pre-allocated for RTOS compliance (max 10 FQ samplers)
        self._max_fq_samplers = 10
        self.registered_fq_samplers: List[Any] = [None] * self._max_fq_samplers
        self._fq_sampler_count = 0
        
        # Initialize frequency from state manager (single source of truth)
        self._initialize_frequency_from_state_manager()
        
        # Running state tracking (with debug integration)
        self._running_state = False
        
        # Genome integration tracking
        self.genome_loaded = False
        
        # Instance ID for debugging and tracking (RTOS-safe)
        self._instance_id = "burst_engine_%d" % id(self)
        
        self.current_timestep = 0
        self.burst_count = 0
        self.previous_fire_queue: Optional[FireQueue] = None

        # Initialize injection service for power areas and special neurons
        self._initialize_injection_service()
        
        # Mark as initialized but keep in INITIALIZING state until burst processing works
        self._initialized = True
        # DON'T set to READY yet - wait until burst processing actually works

        logger.info("BurstEngine initialized with singleton pattern and state manager integration")
    
    def process_burst(self) -> List[int]:
        """Execute complete burst processing with clean 5-phase workflow.
        
        RUST-COMPATIBLE: Deterministic processing with well-defined data flow.
        Uses SoA format internally for SIMD optimization.
        
        Returns:
            List[int]: Neuron IDs that fired in current timestep
        """
        # Process burst - no excessive logging
        
        self.current_timestep = self.burst_count
        
        # NPU Debug logging (enabled with --debug-npu)
        debug_enabled = self.state_manager and self.state_manager.is_debug_npu_enabled()
        
        # Only log critical errors and major state changes
        
        # Phase 1: Collect candidates using FCL Injector
        fcl = FireCandidateList()
        
        self._inject_all_candidates(fcl)
        
        # CRITICAL FIX: Process any remaining accumulated external activations before neural processing
        if self.injection_service and hasattr(self.injection_service, '_pending_external_activations'):
            pending_activations = getattr(self.injection_service, '_pending_external_activations', {})
            if pending_activations:
                # Processing accumulated external activations
                
                # Inject the accumulated data directly into FCL
                late_injection_count = self.injection_service.inject_power_neurons(fcl, self.current_timestep)
                
                # Log successful injection
                if late_injection_count > 0:
                    logger.info("FCL injection: %d neurons from external activations", late_injection_count)

        fcl_candidate_count = fcl.get_total_candidate_count()
        
        # Phase 2: Process neural dynamics - convert FCL candidates to actual firing neurons
        fire_queue = FireQueue()
        
        # Phase 2: Process neural dynamics - convert FCL candidates to actual firing neurons
        try:
            fired_neurons = []
            fcl_candidate_count = fcl.get_total_candidate_count()
            
            if fcl_candidate_count > 0:
                # Process FCL candidates
                # Process FCL candidates for firing evaluation
                
                # Process each cortical area's candidates directly
                total_fired = 0
                for cortical_idx, candidates in fcl.candidates_by_area.items():
                    area_fired = 0
                    
                    # Processing cortical area
                    
                    # Get actual firing thresholds from neuron properties
                    debug_enabled = (self.state_manager and self.state_manager.is_debug_npu_enabled())
                    
                    for candidate in candidates:
                        # Get actual neuron firing threshold from NPU interface
                        firing_threshold = self._get_neuron_firing_threshold(candidate.neuron_id)
                        
                        # Per-neuron firing evaluation
                        
                        if candidate.membrane_potential_delta >= firing_threshold:
                            fired_neurons.append(candidate.neuron_id)
                            area_fired += 1
                    
                    total_fired += area_fired
                    
                    # Area processing complete
                
                # Add fired neurons to fire queue
                if fired_neurons:
                    # Convert fired neuron IDs to FiringNeuron objects
                    
                    # Convert neuron IDs to FiringNeuron objects
                    firing_neurons = self._create_firing_neurons(fired_neurons)
                    # FiringNeuron objects created
                    
                    fire_queue.add_fired_neurons(firing_neurons, self.current_timestep)
                    
                    # Log significant firing activity
                    if total_fired > 0:
                        logger.info("Burst #%d: %d neurons fired", self.burst_count, total_fired)
                    
        except Exception as e:
            # CRITICAL: Set to ERROR state if burst processing fails
            self._set_burst_engine_state(ServiceState.ERROR)
            logger.error("BURST-ENGINE: Critical error in burst processing - marking as ERROR state")
            
            logger.error("Burst processing error: %s", str(e))
            # Continue with empty fire queue to maintain burst cycle
        
        # Phase 3: Archive to fire ledger
        
        if not fire_queue.is_empty():
            neurons_by_area = {}
            for area_idx, neurons in fire_queue.firing_neurons_by_area.items():
                neurons_by_area[area_idx] = neurons
            
            # Archive neurons to fire ledger
                    
            self.fire_ledger.archive_timestep(self.current_timestep, neurons_by_area)
            
            # Update state manager with activity counters
            neuron_count = sum(len(neurons) for neurons in neurons_by_area.values())
            
            try:
                if self.state_manager:
                    self.state_manager.increment_cumulative_activity(neuron_count)
            except Exception:
                # Don't let state manager issues break burst processing
                logger.warning("Failed to update activity counters in state manager")
        # Fire queue was empty - no archiving needed
        
        # Phase 4: FQ Sampler access ready (no action needed)
        # Phase 5: Cleanup and prepare for next burst
            
        self.previous_fire_queue = fire_queue.copy_for_propagation()
        fcl.clear()
        self.burst_count += 1
        
        # Return fired neuron IDs for external systems
        fired_ids = fire_queue.get_all_neuron_ids()
        
        # Set to READY after successful burst processing (if not already READY)
        # This handles cases where the first burst failed but subsequent ones succeed
        current_state = None
        if self.state_manager:
            try:
                current_state = self.state_manager.get_burst_engine_state()
            except Exception:
                pass
        
        if current_state != ServiceState.READY:
            self._set_burst_engine_state(ServiceState.READY)
            if self.burst_count == 1:
                logger.info("Burst engine initialized and ready")
            else:
                logger.info("Burst engine recovered and ready (burst #%d)", self.burst_count)
            
        return fired_ids
    
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
        logger.info("FQ Sampler initialized: %s @ %dHz", sampling_mode, sample_frequency_hz)
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
        # Check if sampler already registered
        sampler_already_registered = False
        for i in range(self._fq_sampler_count):
            if self.registered_fq_samplers[i] == fq_sampler:
                sampler_already_registered = True
                break
        
        if not sampler_already_registered:
            if self._fq_sampler_count < self._max_fq_samplers:
                self.registered_fq_samplers[self._fq_sampler_count] = fq_sampler
                self._fq_sampler_count += 1
                sampler_id = getattr(fq_sampler, 'instance_id', 'unknown')
                logger.info("FQ sampler [%s] registered with BurstEngine", sampler_id)
            else:
                logger.error("Cannot register FQ sampler: maximum limit reached (%d)", self._max_fq_samplers)
        else:
            logger.warning("FQ sampler already registered: %s", getattr(fq_sampler, 'instance_id', 'unknown'))
    
    def unregister_fq_sampler(self, fq_sampler: Any):
        """Unregister an external FQ sampler from this burst engine.
        
        Args:
            fq_sampler: FQ sampler instance to unregister
        """
        # Find and remove sampler using index-based approach
        found_index = -1
        for i in range(self._fq_sampler_count):
            if self.registered_fq_samplers[i] == fq_sampler:
                found_index = i
                break
        
        if found_index >= 0:
            # Shift remaining elements left to fill gap
            for i in range(found_index, self._fq_sampler_count - 1):
                self.registered_fq_samplers[i] = self.registered_fq_samplers[i + 1]
            
            # Clear last element and decrement count
            self.registered_fq_samplers[self._fq_sampler_count - 1] = None
            self._fq_sampler_count -= 1
            
            sampler_id = getattr(fq_sampler, 'instance_id', 'unknown')
            logger.info("FQ sampler [%s] unregistered from BurstEngine", sampler_id)
        else:
            logger.warning("Attempted to unregister FQ sampler that wasn't registered: %s", getattr(fq_sampler, 'instance_id', 'unknown'))
    
    def get_registered_fq_samplers(self) -> List[Any]:
        """Get list of all registered FQ samplers."""
        # Return only active samplers (RTOS-safe: no dynamic allocation)
        return self.registered_fq_samplers[:self._fq_sampler_count]
    
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
                # When starting to run, keep in INITIALIZING state until first burst completes
                # DON'T set to READY here - wait for successful burst processing
                logger.debug("BurstEngine: Started running but waiting for first successful burst to mark as READY")
            else:
                # When stopping, set state to STOPPED
                self._set_burst_engine_state(ServiceState.STOPPED)
        
        # Burst engine running state updated
    
    def _set_burst_engine_state(self, state: ServiceState):
        """Set burst engine state in the state manager."""
        try:
            if self.state_manager:
                self.state_manager.set_burst_engine_state(state)
        except Exception:
            logger.error("Failed to update burst engine state")
    
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
                logger.info("[BURST ENGINE] Using state manager frequency: %dHz", state_frequency)
            else:
                # Emergency fallback: use config and update state manager
                self.desired_frequency = config_frequency
                self.state_manager.set_burst_frequency(config_frequency)
                logger.warning(
                    "[BURST ENGINE] State manager frequency invalid (%sHz) - using config fallback: %dHz and updating state manager", 
                    state_frequency, config_frequency
                )
        except Exception:
            # Emergency fallback: use config frequency and try to update state manager
            self.desired_frequency = config_frequency
            try:
                if self.state_manager:
                    self.state_manager.set_burst_frequency(config_frequency)
                logger.warning("[BURST ENGINE] Failed to get frequency from state manager - using config fallback: %dHz", config_frequency)
            except Exception:
                # Completely fallback - just use the frequency without updating state manager
                logger.error("[BURST ENGINE] Could not initialize frequency in state manager - using local fallback: %dHz", config_frequency)
        
        # Ensure frequency is never zero to avoid division by zero (old BurstEngine safety)
        if self.desired_frequency <= 0:
            self.desired_frequency = config_frequency
            logger.warning("[BURST ENGINE] Frequency was zero - using safety fallback: %dHz", config_frequency)
        
        # Set target_frequency for backward compatibility
        self.target_frequency = self.desired_frequency
    
    def update_frequency(self, frequency_hz: float) -> bool:
        """Update burst engine frequency and sync with state manager."""
        try:
            logger.info("🔄 [API-UPDATE] update_frequency() called with %.2fHz (current: %.2fHz)", 
                       frequency_hz, self.desired_frequency)
            
            if frequency_hz <= 0 or frequency_hz > 10000:
                logger.error("Invalid frequency %dHz (must be 0 < freq <= 10000)", frequency_hz)
                return False
            
            old_frequency = self.desired_frequency
            self.desired_frequency = frequency_hz
            
            # Update state manager
            if self.state_manager:
                self.state_manager.set_burst_frequency(frequency_hz)
                logger.info("✅ [API-UPDATE] Frequency updated: %.2fHz → %.2fHz (dynamic timing will pick up immediately)", 
                           old_frequency, frequency_hz)
            else:
                logger.warning("⚠️  [API-UPDATE] No state manager available for frequency update")
            
            return True
        except Exception as e:
            logger.error("❌ [API-UPDATE] Failed to update burst engine frequency: %s", str(e))
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
        except Exception:
            logger.error("Failed to start BurstEngine")
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
        except Exception:
            logger.error("Failed to stop BurstEngine")
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
            logger.info("🚀 [RUN-DEBUG] Burst engine run() method called - starting main processing loop")
            self.start()
            
            # Enter the main processing loop
            if self._running:
                logger.info("Main loop started at %dHz", self.desired_frequency) 
                
                # CRITICAL: Re-sync frequency from state manager before starting loop
                logger.info("🔄 [STARTUP-DEBUG] About to sync frequency from state manager...")
                try:
                    if self.state_manager:
                        logger.info("🔄 [STARTUP-DEBUG] State manager available, checking frequency...")
                        state_frequency = self.state_manager.get_burst_frequency()
                        logger.info("🔄 [STARTUP-DEBUG] State manager frequency: %.2fHz, Engine frequency: %.2fHz", 
                                   state_frequency or 0.0, self.desired_frequency)
                        if state_frequency and state_frequency > 0:
                            if abs(state_frequency - self.desired_frequency) > 0.001:
                                logger.warning("🚨 [FREQUENCY-SYNC] Frequency mismatch detected! Engine: %.2fHz, State Manager: %.2fHz", 
                                              self.desired_frequency, state_frequency)
                                self.desired_frequency = float(state_frequency)
                                logger.info("🔄 [FREQUENCY-SYNC] Updated engine frequency to match state manager: %.2fHz", 
                                           self.desired_frequency)
                except Exception as e:
                    logger.error("🚨 [FREQUENCY-SYNC] Failed to sync frequency from state manager: %s", str(e))
                
                logger.info("🔄 [FREQUENCY-DEBUG] Starting main loop with frequency: %.2fHz", self.desired_frequency)
                
                # Main burst processing loop
                while self._running:
                    try:
                        # Execute the actual burst processing (RTOS-safe: no timing calls)
                        try:
                            fired_neurons = self.process_burst()
                            
                            # Log periodic burst status 
                            if self.burst_count % 100 == 0:  # Every 100 bursts
                                logger.info("[BURST-ENGINE] Burst #%d: %d neurons fired", self.burst_count, len(fired_neurons))
                                
                        except Exception:
                            logger.error("Error in burst processing #%d", self.burst_count)
                            # Continue processing even if one burst fails
                        
                        # Timing control: Add sleep to prevent runaway CPU usage
                        import time
                        try:
                            # FIXED: Calculate interval dynamically instead of using cached variable
                            current_interval = 1.0 / self.desired_frequency if self.desired_frequency > 0 else 0.1
                            time.sleep(current_interval)
                        except Exception:
                            # Fallback if config unavailable
                            time.sleep(0.1)  # @architecture:acceptable - emergency fallback
                        
                        # Check for exit condition
                        if not self._running:
                            break
                            
                    except Exception:
                        logger.error("Error in burst engine run loop")
                        break
                        
                logger.info("[BURST-ENGINE] Main processing loop ended after %d bursts", self.burst_count)
            else:
                logger.error("[BURST-ENGINE] Failed to start - run loop exiting")
                
        except Exception:
            logger.error("Error in burst engine run() method")
            self._set_burst_engine_state(ServiceState.ERROR)
    
    def update_with_genome(self, connectome_manager=None) -> None:
        """Update the burst engine configuration when a new genome is loaded.
        
        RUST-COMPATIBLE: Reinitializes all memory structures with deterministic sizing
        based on genome neuron count and configuration mode (inference vs design).
        
        This method is called after a new genome is loaded into the connectome manager 
        to refresh the engine's understanding of the neural network. This ensures 
        compatibility with the genome loading process.
        
        Args:
            connectome_manager: Optional connectome manager to use. If not provided,
                               uses the existing one or attempts to get it from the genome service.
        """
        try:
            # Updating burst engine with new genome
            
            # Accept new connectome manager or use existing one
            if connectome_manager:
                self.connectome_manager = connectome_manager
                # ConnectomeManager updated
            elif not self.connectome_manager:
                # Try to get connectome manager from the global/service registry
                try:
                    # Get the connectome manager from the API service or other global reference
                    from feagi.api.core.services.core_api_service import CoreAPIService
                    core_service = CoreAPIService.get_instance()
                    if core_service and hasattr(core_service, 'connectome_manager'):
                        self.connectome_manager = core_service.connectome_manager
                        logger.debug("ConnectomeManager retrieved from CoreAPIService: %s", type(self.connectome_manager).__name__)
                    else:
                        logger.debug("CoreAPIService not available or has no connectome_manager")
                except Exception as e:
                    logger.debug("Failed to get connectome_manager from CoreAPIService: %s", str(e))
            
            logger.info("[CONFIG] Updating burst engine with new genome")
            
            # Sync with connectome manager's current state
            if self.connectome_manager:
                logger.debug("ConnectomeManager available - starting integration")
                
                # Check if connectome manager has neuron array data
                if hasattr(self.connectome_manager, "neuron_array"):
                    neuron_array = self.connectome_manager.neuron_array
                    if hasattr(neuron_array, "neuron_count"):
                        neuron_count = neuron_array.neuron_count
                        logger.debug("Synced with %d neurons from connectome", neuron_count)
                    else:
                        logger.debug("Neuron array present but no count available")
                else:
                    logger.debug("No neuron array data available yet")
                
                # CRITICAL: Re-initialize coordinate converter with connectome manager
                if not self.coordinate_converter:
                    from .coordinate_converter import CoordinateConverter
                    self.coordinate_converter = CoordinateConverter(self.connectome_manager)
                    logger.debug("CoordinateConverter created with ConnectomeManager")
                
                # CRITICAL: Re-initialize FCL injector with connectome data
                if not self.fcl_injector and self.coordinate_converter:
                    from .fcl_injector import FCLInjector
                    self.fcl_injector = FCLInjector(self.coordinate_converter)
                    logger.debug("FCLInjector created with CoordinateConverter")
                elif self.fcl_injector:
                    logger.debug("FCLInjector already exists")
                
                # CRITICAL: Re-initialize injection service for power neurons
                # Always re-initialize to ensure it has the latest fcl_injector
                if self.fcl_injector:
                    self._initialize_injection_service()
                    logger.debug("PowerInjectionService re-initialized with updated FCLInjector")
                elif self.injection_service and hasattr(self.injection_service, 'invalidate_cache'):
                    self.injection_service.invalidate_cache()
                    logger.debug("Power neuron cache invalidated after genome update")
                else:
                    logger.debug("Cannot initialize PowerInjectionService - FCLInjector not available")
                
                # Update coordinate converter with connectome dimensions if available
                if self.coordinate_converter and hasattr(self.connectome_manager, 'get_cortical_dimensions'):
                    logger.debug("CoordinateConverter will use updated cortical dimensions")
                
                # CRITICAL: Calculate and reallocate memory structures based on genome
                self._reinitialize_memory_structures_for_genome()
                
                # Mark that genome data has been integrated
                self.genome_loaded = True
                logger.debug("Genome integration marked complete")
                
            else:
                logger.debug("No connectome manager available after integration attempt")
            
            logger.info("✅ Burst engine updated with new genome successfully")
            
        except Exception as e:
            logger.error("Error updating burst engine with genome: %s", str(e))
            import traceback
            logger.error("Traceback: %s", traceback.format_exc())
            # Don't raise - genome loading should not fail due to burst engine update issues
    
    def force_connectome_integration(self) -> bool:
        """Force integration with connectome manager for debugging purposes.
        
        This method attempts to manually connect the BurstEngine to the connectome
        manager using various discovery methods. Useful for debugging integration issues.
        
        Returns:
            bool: True if integration successful, False otherwise
        """
        try:
            logger.debug("force_connectome_integration() called")
            
            if self.connectome_manager:
                logger.debug("ConnectomeManager already available: %s", type(self.connectome_manager).__name__)
                return True
            
            # Try multiple methods to get connectome manager
            methods_tried = []
            
            # Method 1: Try CoreAPIService
            try:
                from feagi.api.core.services.core_api_service import CoreAPIService
                core_service = CoreAPIService.get_instance()
                if core_service and hasattr(core_service, 'connectome_manager') and core_service.connectome_manager:
                    self.connectome_manager = core_service.connectome_manager
                    logger.debug("ConnectomeManager found via CoreAPIService: %s", type(self.connectome_manager).__name__)
                    self.update_with_genome()
                    return True
                methods_tried.append("CoreAPIService (failed)")
            except Exception as e:
                methods_tried.append("CoreAPIService (error: %s)" % str(e))
            
            # Method 2: Try GenomeService
            try:
                from feagi.api.core.services.genome.genome_service import GenomeService
                # This is harder since GenomeService is not a singleton, but let's try
                methods_tried.append("GenomeService (no singleton pattern)")
            except Exception as e:
                methods_tried.append("GenomeService (error: %s)" % str(e))
            
            # Method 3: Try global registry if available
            try:
                # Check if there's a global connectome manager registry
                import feagi.bdu.connectome_manager as cm_module
                if hasattr(cm_module, '_global_instance'):
                    self.connectome_manager = cm_module._global_instance
                    logger.debug("ConnectomeManager found via global registry")
                    self.update_with_genome()
                    return True
                methods_tried.append("Global registry (not found)")
            except Exception as e:
                methods_tried.append("Global registry (error: %s)" % str(e))
            
            logger.debug("ConnectomeManager not found. Methods tried: %s", ", ".join(methods_tried))
            return False
            
        except Exception as e:
            logger.error("Error in force_connectome_integration: %s", str(e))
            return False
    
    def _inject_all_candidates(self, fcl: FireCandidateList):
        """Inject all candidates into FCL - power neurons, sensory data, and synaptic propagation."""
        
        # NPU Debug logging
        debug_enabled = self.state_manager and self.state_manager.is_debug_npu_enabled()
        periodic_debug = debug_enabled and (self.burst_count % 500 == 0)  # Every 50 bursts
        
        if periodic_debug:
            logger.debug("FCL injection starting...")
            logger.debug("Injection service available: %s", self.injection_service is not None)
            logger.debug("Injection enabled: %s", self.enable_injection)
            logger.debug("FCL injector available: %s", self.fcl_injector is not None)
        
        # 1. CRITICAL: Inject power neurons and special areas EVERY burst
        if self.injection_service and self.enable_injection:
            if periodic_debug:
                logger.debug("Starting power neuron injection...")
                
            try:
                injected_count = self.injection_service.inject_power_neurons(fcl, self.burst_count)
                # Power injection completed
                    
            except Exception as e:
                if debug_enabled:
                    logger.error("Error in power neuron injection: %s", str(e))
                    if periodic_debug:  # Only show traceback periodically
                        import traceback
                        logger.error("Power injection traceback: %s", traceback.format_exc())
                else:
                    logger.error("Error in power neuron injection")
        else:
            if periodic_debug:
                if not self.injection_service:
                    logger.debug("No injection service available - power injection skipped")
                elif not self.enable_injection:
                    logger.debug("Injection disabled - power injection skipped")
        
        # 2. Inject sensory data (if FCL injector available)
        if self.fcl_injector:
            if periodic_debug:
                logger.debug("FCL injector available - checking for sensory/synaptic data...")
                
            # Synaptic propagation from previous timestep
            if self.previous_fire_queue:
                prev_neuron_count = len(self.previous_fire_queue.get_all_neuron_ids()) if self.previous_fire_queue else 0
                
                # Debug logging for NPU debug mode
                debug_enabled = (self.state_manager and self.state_manager.is_debug_npu_enabled())
                # Process synaptic propagation
                
                propagation_data = self._compute_synaptic_propagation()
                if propagation_data:
                    injected_count = self.fcl_injector.inject_synaptic_propagation(fcl, propagation_data)
                    total_targets = sum(len(targets) for targets in propagation_data.values())
                    logger.info("Synaptic propagation: %d candidates injected from %d fired neurons → %d target neurons", 
                               injected_count, prev_neuron_count, total_targets)
                else:
                    logger.info("Synaptic propagation: No synaptic connections found for %d fired neurons", prev_neuron_count)
            # No previous fire queue - first burst or no synaptic propagation
            pass
        else:
            # FCL injector not available - connectome initialization issue
            pass
        
        # FCL injection phase completed
    
    def _compute_synaptic_propagation(self) -> Dict[int, List[tuple]]:
        """Compute synaptic propagation data from previous fire queue.
        
        RUST-COMPATIBLE: Uses vectorized operations and deterministic lookups.
        
        Returns:
            Dict[cortical_idx, List[(target_neuron_id, synaptic_contribution)]]
        """
        if not self.previous_fire_queue or not self.connectome_manager:
            return {}
            
        # Get NPU interface for synapse data access
        npu_interface = getattr(self.connectome_manager, '_npu_interface', None)
        if not npu_interface:
            return {}
            
        synapse_array = getattr(npu_interface, 'synapse_array', None)
        neuron_array = getattr(npu_interface, 'neuron_array', None)
        if not synapse_array or not neuron_array:
            return {}
            
        # Get all fired neuron IDs from previous timestep
        fired_neuron_ids = self.previous_fire_queue.get_all_neuron_ids()
        if not fired_neuron_ids:
            return {}
            
        try:
            propagation_data = {}
            
            # Debug logging
            debug_enabled = (self.state_manager and self.state_manager.is_debug_npu_enabled())
            # Process synaptic propagation for fired neurons
            
            # For each fired neuron, find outgoing synapses
            total_synapses_found = 0
            neurons_with_synapses = 0
            
            for src_neuron_id in fired_neuron_ids:
                # Get outgoing synapses for this source neuron
                synapse_indices = getattr(synapse_array, 'source_neuron_index', {}).get(src_neuron_id, [])
                
                # Neuron has outgoing synapses - process them
                
                if not synapse_indices:
                    continue
                    
                neurons_with_synapses += 1
                total_synapses_found += len(synapse_indices)
                    
                # Convert to numpy array for vectorized operations
                syn_indices = np.array(synapse_indices, dtype=np.int32)
                
                # Filter valid synapses
                valid_mask = getattr(synapse_array, 'valid_mask', None)
                if valid_mask is not None:
                    valid_syn_mask = valid_mask[syn_indices]
                    valid_count = np.sum(valid_syn_mask)
                    if not np.any(valid_syn_mask):
                        continue
                    syn_indices = syn_indices[valid_syn_mask]
                
                # Get target neuron IDs and synaptic properties
                target_neuron_ids = synapse_array.target_neuron_ids[syn_indices].astype(np.int32)
                weights = synapse_array.weights[syn_indices].astype(np.float32)
                
                # Process synaptic targets and weights
                
                # Apply conductance and excitatory/inhibitory type
                conductances = getattr(synapse_array, 'conductances', None)
                syn_types = getattr(synapse_array, 'types', None)
                
                if conductances is not None:
                    conductances_array = conductances[syn_indices].astype(np.float32)
                else:
                    conductances_array = np.ones(len(syn_indices), dtype=np.float32)
                    
                if syn_types is not None:
                    types_array = syn_types[syn_indices].astype(np.int32)
                    # 0 = excitatory (+), 1 = inhibitory (-)
                    sign = np.where(types_array == 0, 1.0, -1.0).astype(np.float32)
                else:
                    sign = np.ones(len(syn_indices), dtype=np.float32)  # Default to excitatory
                
                # Compute synaptic contributions
                synaptic_contributions = weights * conductances_array * sign
                
                # Contributions computed
                
                # Group by target cortical areas
                neuron_to_area = getattr(npu_interface, 'neuron_to_area', {})
                
                targets_by_area = {}
                for target_id, contribution in zip(target_neuron_ids, synaptic_contributions):
                    target_id = int(target_id)
                    contribution = float(contribution)
                    
                    # Get cortical area for target neuron - MUST exist in genome
                    if target_id not in neuron_to_area:
                        raise ValueError(f"Target neuron {target_id} not mapped to any cortical area in genome")
                    cortical_idx = neuron_to_area[target_id]
                    
                    # Track targets by area for debugging
                    if cortical_idx not in targets_by_area:
                        targets_by_area[cortical_idx] = 0
                    targets_by_area[cortical_idx] += 1
                    
                    # Add to propagation data
                    if cortical_idx not in propagation_data:
                        propagation_data[cortical_idx] = []
                    
                    propagation_data[cortical_idx].append((target_id, contribution))
                
                # Group targets by area completed
            
            # Synaptic propagation completed
                           
            return propagation_data
            
        except Exception as e:
            logger.error("Error in synaptic propagation computation: %s", str(e))
            return {}
    
    def _get_neuron_firing_threshold(self, neuron_id: int) -> float:
        """Get the actual firing threshold for a specific neuron.
        
        RUST-COMPATIBLE: 100% deterministic lookup. NO FALLBACKS.
        Raises error if genome data is missing - FEAGI must be deterministic.
        
        Args:
            neuron_id: The neuron ID to get threshold for
            
        Returns:
            Actual firing threshold from neuron properties (from genome)
            
        Raises:
            ValueError: If neuron data is missing from genome/NPU interface
        """
        if not self.connectome_manager:
            raise ValueError(f"Cannot get threshold for neuron {neuron_id}: No connectome manager available")
            
        npu_interface = getattr(self.connectome_manager, '_npu_interface', None)
        if not npu_interface:
            raise ValueError(f"Cannot get threshold for neuron {neuron_id}: No NPU interface in connectome manager")
            
        neuron_array = getattr(npu_interface, 'neuron_array', None)
        if not neuron_array:
            raise ValueError(f"Cannot get threshold for neuron {neuron_id}: No neuron array in NPU interface")
            
        # Get neuron index from ID - MUST exist in genome
        neuron_id_to_index = getattr(neuron_array, 'neuron_id_to_index', None)
        if not neuron_id_to_index:
            raise ValueError(f"Cannot get threshold for neuron {neuron_id}: No neuron ID mapping in genome")
            
        if neuron_id not in neuron_id_to_index:
            raise ValueError(f"Neuron {neuron_id} not found in genome neuron array - missing from neuroembryogenesis")
            
        neuron_index = neuron_id_to_index[neuron_id]
        
        # Get threshold array - MUST exist in genome
        thresholds = getattr(neuron_array, 'thresholds', None)
        if thresholds is None:
            raise ValueError(f"No firing thresholds array in genome neuron data")
            
        if neuron_index >= len(thresholds):
            raise ValueError(f"Neuron {neuron_id} index {neuron_index} out of bounds in thresholds array")
            
        actual_threshold = float(thresholds[neuron_index])
        return actual_threshold
    
    def _create_firing_neurons(self, fired_neuron_ids: List[int]) -> List[FiringNeuron]:
        """Convert fired neuron IDs to FiringNeuron objects with properties."""
        # Use list comprehension to avoid index assignment errors
        firing_neurons = []
        
        try:
            for idx, neuron_id in enumerate(fired_neuron_ids):
                # Get actual neuron properties from genome - ZERO FALLBACKS, STRICT VALIDATION
                # All values MUST come from genome via neuroembryogenesis → connectome → NPU interface
                
                if not self.connectome_manager:
                    raise ValueError(f"Cannot create FiringNeuron for {neuron_id}: No connectome manager available")
                    
                npu_interface = getattr(self.connectome_manager, '_npu_interface', None)
                if not npu_interface:
                    raise ValueError(f"Cannot create FiringNeuron for {neuron_id}: No NPU interface in connectome manager")
                
                # Get cortical area - MUST exist in genome
                neuron_to_area = getattr(npu_interface, 'neuron_to_area', None)
                if not neuron_to_area:
                    raise ValueError(f"Cannot create FiringNeuron for {neuron_id}: No neuron-to-area mapping in genome")
                if neuron_id not in neuron_to_area:
                    raise ValueError(f"Neuron {neuron_id} not mapped to any cortical area in genome")
                cortical_idx = neuron_to_area[neuron_id]
                
                # Get coordinates from neuron array - STRICT VALIDATION, NO FALLBACKS
                neuron_array = getattr(npu_interface, 'neuron_array', None)
                if not neuron_array:
                    raise ValueError(f"Cannot create FiringNeuron for {neuron_id}: No neuron array in NPU interface")
                    
                neuron_id_to_index = getattr(neuron_array, 'neuron_id_to_index', None)
                if not neuron_id_to_index:
                    raise ValueError(f"Cannot create FiringNeuron for {neuron_id}: No neuron ID mapping in genome")
                    
                if neuron_id not in neuron_id_to_index:
                    raise ValueError(f"Neuron {neuron_id} not found in genome neuron array")
                    
                neuron_index = neuron_id_to_index[neuron_id]
                
                # Get coordinates - MUST exist in genome
                coordinates_x = getattr(neuron_array, 'coordinates_x', None)
                coordinates_y = getattr(neuron_array, 'coordinates_y', None) 
                coordinates_z = getattr(neuron_array, 'coordinates_z', None)
                
                if coordinates_x is None or coordinates_y is None or coordinates_z is None:
                    raise ValueError(f"Missing coordinate arrays in neuron array for neuron {neuron_id}")
                    
                if neuron_index >= len(coordinates_x) or neuron_index >= len(coordinates_y) or neuron_index >= len(coordinates_z):
                    raise ValueError(f"Neuron {neuron_id} index {neuron_index} out of bounds in coordinate arrays")
                
                x = int(coordinates_x[neuron_index])
                y = int(coordinates_y[neuron_index])  
                z = int(coordinates_z[neuron_index])
                coordinates = (x, y, z)
                
                # Get membrane potential - MUST exist in genome
                membrane_potentials = getattr(neuron_array, 'membrane_potentials', None)
                if membrane_potentials is None:
                    raise ValueError(f"No membrane potentials array in neuron array for neuron {neuron_id}")
                    
                if neuron_index >= len(membrane_potentials):
                    raise ValueError(f"Neuron {neuron_id} index {neuron_index} out of bounds in membrane potentials array")
                    
                membrane_potential = float(membrane_potentials[neuron_index])
                
                # Get threshold - MUST exist in genome  
                threshold = self._get_neuron_firing_threshold(neuron_id)
                
                # Create FiringNeuron with available data
                firing_neuron = FiringNeuron(
                    neuron_id=neuron_id,
                    cortical_idx=cortical_idx,
                    membrane_potential=membrane_potential,
                    coordinates=coordinates,
                    threshold=threshold,
                    consecutive_fire_count=0,  # Could be tracked in future
                    refractory_counter=0,      # Could be tracked in future
                    timestamp=0.0  # RTOS-safe: no system time calls
                )
                
                # Use append instead of index assignment to avoid range errors
                firing_neurons.append(firing_neuron)
            
            # FiringNeuron objects created successfully
            
        except Exception as e:
            logger.error("Error creating firing neurons: %s", str(e))
            # Return empty list to prevent breaking burst cycle - CRITICAL FIX
            return []
            
        return firing_neurons
    
    def _reinitialize_memory_structures_for_genome(self) -> None:
        """Reinitialize all burst engine memory structures based on genome size and configuration.
        
        RUST-COMPATIBLE: Deterministic memory allocation with pre-calculated sizes.
        Implements inference vs design mode allocation strategy.
        """
        try:
            logger.info("🧠 GENOME-MEMORY: Starting burst engine memory reinitialization")
            
            # Load burst engine configuration from TOML
            burst_config = self._load_burst_engine_config()
            
            # Calculate memory requirements based on genome and mode
            memory_requirements = self._calculate_memory_requirements(burst_config)
            
            # Log allocation strategy
            logger.info(
                f"🧠 GENOME-MEMORY: Mode={burst_config['mode']}, "
                f"Genome neurons={memory_requirements['genome_neurons']}, "
                f"FCL capacity={memory_requirements['fcl_capacity']}, "
                f"FireQueue capacity={memory_requirements['fire_queue_capacity']}"
            )
            
            # Reinitialize data structures with calculated capacities
            self._reallocate_data_structures(memory_requirements)
            
            logger.info("✅ GENOME-MEMORY: Burst engine memory structures reinitialized successfully")
            
        except Exception as e:
            logger.error(f"❌ GENOME-MEMORY: Failed to reinitialize memory structures: {e}")
            # Don't raise - allow burst engine to continue with existing structures
    
    def _load_burst_engine_config(self) -> Dict[str, Any]:
        """Load burst engine configuration from TOML with defaults.
        
        Returns:
            Dictionary with burst engine configuration parameters
        """
        try:
            from feagi.config.toml_loader import load_feagi_config
            config = load_feagi_config()
            
            # Get burst_engine section with defaults
            burst_config = config.get('burst_engine', {})
            
            # Apply defaults for missing values
            defaults = {
                'mode': 'inference',
                'max_supported_neurons': 10000000,
                'design_mode_allocation_percent': 80,
                'fcl_capacity_multiplier': 1.5,
                'fire_queue_capacity_multiplier': 1.2,
                'memory_area_multiplier': 2.0,
                'enable_preallocation': True,
                'enable_capacity_warnings': True
            }
            
            for key, default_value in defaults.items():
                if key not in burst_config:
                    burst_config[key] = default_value
            
            return burst_config
            
        except Exception as e:
            logger.warning(f"Failed to load burst engine config, using defaults: {e}")
            # Return safe defaults
            return {
                'mode': 'inference',
                'max_supported_neurons': 1000000,  # Conservative default
                'design_mode_allocation_percent': 80,
                'fcl_capacity_multiplier': 1.5,
                'fire_queue_capacity_multiplier': 1.2,
                'memory_area_multiplier': 2.0,
                'enable_preallocation': True,
                'enable_capacity_warnings': True
            }
    
    def _calculate_memory_requirements(self, burst_config: Dict[str, Any]) -> Dict[str, int]:
        """Calculate memory requirements based on genome size and configuration mode.
        
        Args:
            burst_config: Burst engine configuration from TOML
            
        Returns:
            Dictionary with calculated memory capacities for each component
        """
        # Get genome neuron count from connectome manager
        genome_neurons = self._get_genome_neuron_count()
        
        if burst_config['mode'] == 'design':
            # Design mode: Use percentage of max supported neurons
            base_neurons = int(
                burst_config['max_supported_neurons'] * 
                (burst_config['design_mode_allocation_percent'] / 100.0)
            )
            logger.info(f"🎨 DESIGN MODE: Allocating for {base_neurons} neurons ({burst_config['design_mode_allocation_percent']}% of {burst_config['max_supported_neurons']})")
        else:
            # Inference mode: Use actual genome neuron count
            base_neurons = max(genome_neurons, 1000)  # Minimum 1000 neurons
            logger.info(f"⚡ INFERENCE MODE: Allocating for {base_neurons} neurons (genome actual)")
        
        # Calculate component capacities with safety multipliers
        fcl_capacity = int(base_neurons * burst_config['fcl_capacity_multiplier'])
        fire_queue_capacity = int(base_neurons * burst_config['fire_queue_capacity_multiplier'])
        memory_area_capacity = int(base_neurons * burst_config['memory_area_multiplier'])
        
        return {
            'genome_neurons': genome_neurons,
            'base_neurons': base_neurons,
            'fcl_capacity': fcl_capacity,
            'fire_queue_capacity': fire_queue_capacity,
            'memory_area_capacity': memory_area_capacity,
            'mode': burst_config['mode']
        }
    
    def _get_genome_neuron_count(self) -> int:
        """Get the actual neuron count from the loaded genome.
        
        Returns:
            Number of neurons in the currently loaded genome
        """
        if not self.connectome_manager:
            return 0
            
        # Try multiple methods to get neuron count
        neuron_count = 0
        
        # Method 1: From neuron array
        if hasattr(self.connectome_manager, 'neuron_array'):
            neuron_array = self.connectome_manager.neuron_array
            if hasattr(neuron_array, 'neuron_count'):
                neuron_count = max(neuron_count, neuron_array.neuron_count)
        
        # Method 2: From cortical areas
        if hasattr(self.connectome_manager, 'cortical_areas'):
            cortical_areas = self.connectome_manager.cortical_areas
            if cortical_areas:
                total_area_neurons = 0
                for area_id, area in cortical_areas.items():
                    if hasattr(area, 'neuron_count'):
                        total_area_neurons += area.neuron_count
                neuron_count = max(neuron_count, total_area_neurons)
        
        # Method 3: From brain statistics (if available)
        if hasattr(self.connectome_manager, 'get_brain_statistics'):
            try:
                stats = self.connectome_manager.get_brain_statistics()
                if stats and 'neuron_count' in stats:
                    neuron_count = max(neuron_count, stats['neuron_count'])
            except Exception:
                pass  # @architecture:acceptable - statistics lookup fallback
        
        logger.debug(f"Genome neuron count determined: {neuron_count}")
        return neuron_count
    
    def _reallocate_data_structures(self, memory_requirements: Dict[str, int]) -> None:
        """Reallocate all data structures with new memory requirements.
        
        RUST-COMPATIBLE: Pre-allocates all arrays with deterministic sizes.
        
        Args:
            memory_requirements: Dictionary with calculated capacities
        """
        # Note: In the current Python implementation, we don't need to explicitly
        # reallocate FCL and FireQueue since they use dynamic containers.
        # However, we log the intended capacities for monitoring and future Rust conversion.
        
        fcl_capacity = memory_requirements['fcl_capacity']
        fire_queue_capacity = memory_requirements['fire_queue_capacity']
        
        logger.info(f"📊 MEMORY ALLOCATION: FCL capacity set to {fcl_capacity:,} candidates")
        logger.info(f"📊 MEMORY ALLOCATION: FireQueue capacity set to {fire_queue_capacity:,} neurons")
        
        # In Rust conversion, this would be:
        # self.fcl_candidates = Vec::with_capacity(fcl_capacity)
        # self.fire_queue_neurons = Vec::with_capacity(fire_queue_capacity)
        
        # For now, we store the capacities for monitoring
        self._fcl_capacity = fcl_capacity
        self._fire_queue_capacity = fire_queue_capacity
        self._memory_requirements = memory_requirements
        
        logger.debug("Memory structure capacities configured for Rust conversion readiness")
    
    def _initialize_injection_service(self):
        """Initialize injection service for power areas and special neuron injection."""
        try:
            if not self.connectome_manager:
                logger.info("No connectome manager - injection service disabled")
                return
            
            # Create simple power injection service
            self.injection_service = PowerInjectionService(
                connectome_manager=self.connectome_manager,
                fcl_injector=self.fcl_injector
            )
            
            logger.info("Power injection service initialized for automatic burst injection")
            
        except Exception:
            logger.error("Failed to initialize injection service")
            self.injection_service = None


class PowerInjectionService:
    """Simple power injection service for constant brain activity."""
    
    def __init__(self, connectome_manager, fcl_injector):
        """Initialize power injection service."""
        self.connectome_manager = connectome_manager
        self.fcl_injector = fcl_injector
        self._power_neurons_cache = None
        self._cache_valid = False
        
        logger.info("PowerInjectionService created")
    
    def _get_neuron_firing_threshold(self, neuron_id: int) -> float:
        """Get the actual firing threshold for a specific neuron.
        
        RUST-COMPATIBLE: 100% deterministic lookup. NO FALLBACKS.
        Raises error if genome data is missing - FEAGI must be deterministic.
        
        Args:
            neuron_id: The neuron ID to get threshold for
            
        Returns:
            Actual firing threshold from neuron properties (from genome)
            
        Raises:
            ValueError: If neuron data is missing from genome/NPU interface
        """
        if not self.connectome_manager:
            raise ValueError(f"Cannot get threshold for neuron {neuron_id}: No connectome manager available")
            
        npu_interface = getattr(self.connectome_manager, '_npu_interface', None)
        if not npu_interface:
            raise ValueError(f"Cannot get threshold for neuron {neuron_id}: No NPU interface in connectome manager")
            
        neuron_array = getattr(npu_interface, 'neuron_array', None)
        if not neuron_array:
            raise ValueError(f"Cannot get threshold for neuron {neuron_id}: No neuron array in NPU interface")
            
        # Get neuron index from ID - MUST exist in genome
        neuron_id_to_index = getattr(neuron_array, 'neuron_id_to_index', None)
        if not neuron_id_to_index:
            raise ValueError(f"Cannot get threshold for neuron {neuron_id}: No neuron ID mapping in genome")
            
        if neuron_id not in neuron_id_to_index:
            raise ValueError(f"Neuron {neuron_id} not found in genome neuron array - missing from neuroembryogenesis")
            
        neuron_index = neuron_id_to_index[neuron_id]
        
        # Get threshold array - MUST exist in genome
        thresholds = getattr(neuron_array, 'thresholds', None)
        if thresholds is None:
            raise ValueError(f"No firing thresholds array in genome neuron data")
            
        if neuron_index >= len(thresholds):
            raise ValueError(f"Neuron {neuron_id} index {neuron_index} out of bounds in thresholds array")
            
        actual_threshold = float(thresholds[neuron_index])
        return actual_threshold
    
    def inject_power_neurons(self, fcl: FireCandidateList, current_timestep: int) -> int:
        """Inject power neurons into FCL every burst for constant brain activity."""
        
        # NPU Debug logging for power injection - use periodic logging to prevent spam
        debug_enabled = (hasattr(self.connectome_manager, 'state_manager') and 
                        self.connectome_manager.state_manager and 
                        self.connectome_manager.state_manager.is_debug_npu_enabled())
        periodic_debug = debug_enabled and (current_timestep % 500 == 0)  # Every 50 bursts
        
        if periodic_debug:
            logger.debug("PowerInjectionService: Starting power neuron injection...")
            logger.debug("PowerInjectionService: Cache valid: %s", self._cache_valid)
            logger.debug("PowerInjectionService: Instance ID: %s", id(self))
            logger.debug("PowerInjectionService: Method entry - about to process power and external activations")
        
        try:
            # Get power neurons (cached for performance)
            power_neurons = self._get_power_neurons()
            
            if periodic_debug:
                logger.debug("PowerInjectionService: Retrieved %d power neurons from cache/detection", len(power_neurons))
            
            # Process power neurons if available
            injected_count = 0
            if not power_neurons:
                # Only log occasionally to avoid spam
                if current_timestep % 1000 == 0:
                    if periodic_debug:
                        logger.debug("PowerInjectionService: No power neurons available at timestep %d", current_timestep)
                    else:
                        logger.debug("No power neurons found at timestep %d", current_timestep)
                # Don't return early - continue to process external activations
            
            # Add power neurons to FCL using SoA format
            if power_neurons:
                if periodic_debug:
                    logger.debug("PowerInjectionService: Converting %d power neurons to SoA format", len(power_neurons))
                
                # Convert to numpy arrays for SoA format
                neuron_ids = np.array(power_neurons, dtype=np.uint32)
                potential_deltas = np.full(len(power_neurons), 75.0, dtype=np.float32)  # High potential for power neurons (above threshold)
                excitatory_mask = np.ones(len(power_neurons), dtype=bool)  # All excitatory
                
                if periodic_debug:
                    logger.debug("PowerInjectionService: SoA arrays created - neuron_ids: %s, potentials: %s", 
                              neuron_ids.shape, potential_deltas.shape)
                
                # Add all power neurons to cortical area 0 (generic power injection)
                injected_count = fcl.add_candidates_soa(
                    cortical_idx=0,  # Generic power area index
                    neuron_ids=neuron_ids,
                    potential_deltas=potential_deltas,
                    excitatory_mask=excitatory_mask
                )
                
                if periodic_debug:
                    logger.debug("PowerInjectionService: FCL.add_candidates_soa returned: %d", injected_count)
            else:
                # No power neurons available
                if periodic_debug:
                    logger.debug("PowerInjectionService: No power neurons to inject")
            
            # Log occasionally (every 100 bursts) to avoid spam
            if current_timestep % 100 == 0:
                if periodic_debug:
                    logger.debug("PowerInjectionService: Periodic status - %d neurons injected at burst %d", injected_count, current_timestep)
                else:
                    logger.info("Power injection: %d neurons injected at burst %d", injected_count, current_timestep)
            
            if periodic_debug:
                logger.debug("PowerInjectionService: Power injection phase complete, proceeding to external activations...")
            
            # CRITICAL: Also process any pending external activations (sensory data)
            external_count = 0
            
            if periodic_debug:
                logger.debug("PowerInjectionService: Reached external activation processing section")
                has_pending_attr = hasattr(self, '_pending_external_activations')
                pending_data = getattr(self, '_pending_external_activations', None)
                logger.debug("PowerInjectionService: Checking external activations - has_attr=%s, data=%s", 
                              has_pending_attr, "None" if pending_data is None else "Data(%d)" % len(pending_data))
            
            if hasattr(self, '_pending_external_activations') and self._pending_external_activations:
                if periodic_debug:
                    logger.debug("PowerInjectionService: Processing %d pending external activations", 
                                  len(self._pending_external_activations))
                
                try:
                    # CRITICAL FIX: Create a copy to avoid "dictionary changed during iteration" race condition
                    pending_copy = dict(self._pending_external_activations)
                    
                    if periodic_debug:
                        logger.debug("PowerInjectionService: Created safe copy with %d pending external activations", 
                                      len(pending_copy))
                    
                    # Process external activations using the same FCL instance
                    for area_id, area_data in pending_copy.items():
                        if periodic_debug:
                            logger.debug("PowerInjectionService: Processing area %s with data type %s", area_id, type(area_data))
                        
                        # Handle both coordinate dict format and neuron ID list format
                        if isinstance(area_data, dict) and 'coordinates_x' in area_data:
                            # COORDINATE DICT FORMAT: {'coordinates_x': [...], 'coordinates_y': [...], ...}
                            coords_x = area_data.get('coordinates_x', np.array([]))
                            coords_y = area_data.get('coordinates_y', np.array([]))  
                            coords_z = area_data.get('coordinates_z', np.array([]))
                            potentials = area_data.get('membrane_potentials', np.array([]))
                            
                            if periodic_debug:
                                logger.debug("PowerInjectionService: Area %s - %d coordinates to convert", area_id, len(coords_x))
                            
                            if len(coords_x) > 0:
                                # Convert coordinates to neuron IDs using coordinate converter
                                if self.fcl_injector and hasattr(self.fcl_injector, 'coordinate_converter'):
                                    converter = self.fcl_injector.coordinate_converter
                                    if periodic_debug:
                                        logger.debug("PowerInjectionService: Coordinate converter available: %s", converter is not None)
                                    
                                    if converter and hasattr(converter, 'convert_soa_to_neuron_ids'):
                                        try:
                                            if periodic_debug:
                                                logger.debug("PowerInjectionService: Calling convert_soa_to_neuron_ids for area %s", area_id)
                                            
                                            neuron_ids, valid_potentials, cortical_idx = converter.convert_soa_to_neuron_ids(
                                                area_id, coords_x, coords_y, coords_z, potentials
                                            )
                                            
                                            if periodic_debug:
                                                logger.debug("PowerInjectionService: Conversion result - %d neuron_ids, cortical_idx=%d", 
                                                              len(neuron_ids) if neuron_ids is not None else 0, cortical_idx)
                                            
                                            if len(neuron_ids) > 0:
                                                # Inject external neurons into FCL
                                                excitatory_mask = np.ones(len(neuron_ids), dtype=bool)
                                                
                                                if periodic_debug:
                                                    logger.debug("PowerInjectionService: Calling FCL.add_candidates_soa with %d neurons", len(neuron_ids))
                                                
                                                ext_injected = fcl.add_candidates_soa(
                                                    cortical_idx=cortical_idx,
                                                    neuron_ids=neuron_ids.astype(np.uint32),
                                                    potential_deltas=valid_potentials.astype(np.float32),
                                                    excitatory_mask=excitatory_mask
                                                )
                                                external_count += ext_injected
                                                
                                                if periodic_debug:
                                                    logger.debug("PowerInjectionService: FCL.add_candidates_soa returned %d for area %s", 
                                                                  ext_injected, area_id)
                                            else:
                                                if periodic_debug:
                                                    logger.debug("PowerInjectionService: No valid neuron_ids generated for area %s", area_id)
                                                
                                        except Exception as convert_err:
                                            if debug_enabled:
                                                logger.error("PowerInjectionService: Error converting external coordinates: %s", str(convert_err))
                                                if periodic_debug:  # Only show traceback periodically
                                                    import traceback
                                                    logger.error("PowerInjectionService: Conversion traceback: %s", traceback.format_exc())
                                    else:
                                        if periodic_debug:
                                            logger.debug("PowerInjectionService: Coordinate converter missing convert_soa_to_neuron_ids method")
                                else:
                                    if periodic_debug:
                                        logger.debug("PowerInjectionService: No coordinate converter available")
                            else:
                                if periodic_debug:
                                    logger.debug("PowerInjectionService: Area %s has no coordinates to process", area_id)
                        elif isinstance(area_data, list) and len(area_data) > 0:
                            # NEURON ID LIST FORMAT: [neuron_id_1, neuron_id_2, ...]
                            if periodic_debug:
                                logger.debug("PowerInjectionService: Processing area %s as neuron ID list with %d neurons", area_id, len(area_data))
                            
                            try:
                                # Convert neuron IDs to numpy array
                                neuron_ids = np.array(area_data, dtype=np.uint32)
                                
                                # Get actual neuron potentials from genome properties, not hardcoded defaults
                                # These neurons were selected by sensory processing - use their actual thresholds
                                potentials = []
                                for neuron_id in neuron_ids:
                                    # Get the actual firing threshold for this specific neuron from genome
                                    actual_threshold = self._get_neuron_firing_threshold(int(neuron_id))
                                    # Use threshold as potential to ensure firing (sensory neurons should activate)
                                    potentials.append(actual_threshold)
                                potentials = np.array(potentials, dtype=np.float32)
                                
                                # Get cortical index for this area
                                if self.fcl_injector and hasattr(self.fcl_injector, 'coordinate_converter'):
                                    converter = self.fcl_injector.coordinate_converter
                                    if hasattr(converter, 'connectome_manager') and hasattr(converter.connectome_manager, 'get_cortical_idx_for_id'):
                                        cortical_idx = converter.connectome_manager.get_cortical_idx_for_id(area_id)
                                        
                                        if cortical_idx is not None:
                                            # Inject neuron ID list directly into FCL
                                            excitatory_mask = np.ones(len(neuron_ids), dtype=bool)
                                            
                                            if periodic_debug:
                                                logger.debug("PowerInjectionService: Injecting %d neuron IDs for area %s (cortical_idx=%d)", len(neuron_ids), area_id, cortical_idx)
                                            
                                            list_injected = fcl.add_candidates_soa(
                                                cortical_idx=cortical_idx,
                                                neuron_ids=neuron_ids,
                                                potential_deltas=potentials,
                                                excitatory_mask=excitatory_mask
                                            )
                                            external_count += list_injected
                                            
                                            if periodic_debug:
                                                logger.debug("PowerInjectionService: FCL.add_candidates_soa returned %d for neuron ID list in area %s", list_injected, area_id)
                                        else:
                                            if periodic_debug:
                                                logger.debug("PowerInjectionService: Could not get cortical_idx for area %s", area_id)
                                    else:
                                        if periodic_debug:
                                            logger.debug("PowerInjectionService: Coordinate converter or connectome manager not available for neuron ID list processing")
                                            
                            except Exception as list_err:
                                if debug_enabled:
                                    logger.error("PowerInjectionService: Error processing neuron ID list for area %s: %s", area_id, str(list_err))
                                            
                        else:
                            if periodic_debug:
                                data_info = f"type={type(area_data)}, len={len(area_data) if hasattr(area_data, '__len__') else 'N/A'}"
                                logger.debug("PowerInjectionService: Area %s data format not supported: %s", area_id, data_info)
                    
                    # Clear processed external activations (but keep the container for new arrivals)
                    self._pending_external_activations.clear()
                    
                    if periodic_debug:
                        logger.debug("PowerInjectionService: Total external neurons injected: %d", external_count)
                        
                except Exception as ext_err:
                    if debug_enabled:
                        logger.error("PowerInjectionService: Error processing external activations: %s", str(ext_err))
            else:
                if periodic_debug:
                    logger.debug("PowerInjectionService: No external activations to process this burst")
            
            total_injected = injected_count + external_count
            
            # Log occasionally (every 100 bursts) to avoid spam
            if current_timestep % 100 == 0:
                if periodic_debug:
                    logger.debug("PowerInjectionService: Combined injection - %d power + %d external = %d total at burst %d", 
                                  injected_count, external_count, total_injected, current_timestep)
                else:
                    logger.info("Combined injection: %d power + %d external = %d total at burst %d", 
                               injected_count, external_count, total_injected, current_timestep)
            
            if periodic_debug:
                logger.debug("PowerInjectionService: inject_power_neurons() completing successfully - total=%d", total_injected)
            
            return total_injected
            
        except Exception as e:
            if debug_enabled:
                logger.error("PowerInjectionService: Exception in inject_power_neurons(): %s", str(e))
                if periodic_debug:  # Only show traceback periodically
                    import traceback
                    logger.error("PowerInjectionService: Traceback: %s", traceback.format_exc())
            else:
                logger.error("Error injecting power neurons: %s", str(e))
            return 0
    
    def inject_external_activations(self, activations: Dict, current_timestep: int, source: str = "external") -> int:
        """Inject external activations (from sensory streams) into the current FCL.
        
        This method is called by the external sensory injection system to inject
        sensory data into the BurstEngine's processing pipeline.
        
        Args:
            activations: Dictionary of activations by cortical area
            current_timestep: Current simulation timestep
            source: Source of the activations (for debugging)
            
        Returns:
            Number of neurons successfully injected
        """
        debug_enabled = (hasattr(self.connectome_manager, 'state_manager') and 
                        self.connectome_manager.state_manager and 
                        self.connectome_manager.state_manager.is_debug_npu_enabled())
        periodic_debug = debug_enabled and (current_timestep % 500 == 0)  # Every 50 bursts
        
        
        try:
            total_injected = 0
            
            # CRITICAL: Accumulate external activations instead of overwriting them
            if not hasattr(self, '_pending_external_activations'):
                self._pending_external_activations = {}
            
            # Merge new activations with existing ones (accumulate, don't overwrite)
            for area_id, area_data in activations.items():
                if periodic_debug:
                    logger.debug("PowerInjectionService: Processing area %s - incoming data type: %s", area_id, type(area_data))
                
                if area_id in self._pending_external_activations:
                    # Merge activation data for this cortical area
                    existing_data = self._pending_external_activations[area_id]
                    
                    if periodic_debug:
                        logger.debug("PowerInjectionService: Merging with existing data type: %s", type(existing_data))
                    
                    if isinstance(existing_data, dict) and isinstance(area_data, dict):
                        # Concatenate coordinate arrays
                        if periodic_debug:
                            logger.debug("PowerInjectionService: Concatenating arrays for area %s", area_id)
                        
                        for key in ['coordinates_x', 'coordinates_y', 'coordinates_z', 'membrane_potentials']:
                            if key in existing_data and key in area_data:
                                try:
                                    existing_data[key] = np.concatenate([existing_data[key], area_data[key]])
                                    if periodic_debug:
                                        logger.debug("PowerInjectionService: Concatenated %s - new length: %d", key, len(existing_data[key]))
                                except Exception as concat_err:
                                    if debug_enabled:
                                        logger.error("PowerInjectionService: Error concatenating %s: %s", key, str(concat_err))
                                    # Fallback: overwrite with new data
                                    existing_data[key] = area_data[key]
                            elif key in area_data:
                                existing_data[key] = area_data[key]
                                if periodic_debug:
                                    logger.debug("PowerInjectionService: Added missing %s key", key)
                        
                        if periodic_debug:
                            final_count = len(existing_data.get('coordinates_x', []))
                            logger.debug("PowerInjectionService: Merge complete - %d total coordinates in area %s", final_count, area_id)
                            logger.debug("PowerInjectionService: Final data type for area %s: %s", area_id, type(self._pending_external_activations[area_id]))
                    else:
                        # If not dict format, just overwrite
                        if periodic_debug:
                            logger.debug("PowerInjectionService: Data types incompatible, overwriting area %s", area_id)
                        self._pending_external_activations[area_id] = area_data
                else:
                    # New cortical area
                    self._pending_external_activations[area_id] = area_data
                    if periodic_debug:
                        coords_count = len(area_data.get('coordinates_x', [])) if isinstance(area_data, dict) else 1
                        logger.debug("PowerInjectionService: Added %d new activations for new area %s", coords_count, area_id)
                        logger.debug("PowerInjectionService: Stored data type for new area %s: %s", area_id, type(self._pending_external_activations[area_id]))
            
            if periodic_debug:
                # Count total accumulated external activations
                total_accumulated = 0
                for area_id, area_data in self._pending_external_activations.items():
                    if isinstance(area_data, dict) and 'coordinates_x' in area_data:
                        total_accumulated += len(area_data['coordinates_x'])
                    elif isinstance(area_data, (list, np.ndarray)):
                        total_accumulated += len(area_data)
                        
                logger.debug("PowerInjectionService: %d total external activations accumulated across %d areas", 
                              total_accumulated, len(self._pending_external_activations))
            
            # For immediate feedback, count what would be injected from the current batch
            for area_id, area_data in activations.items():
                if isinstance(area_data, dict):
                    # Count coordinate arrays length
                    if 'coordinates_x' in area_data:
                        total_injected += len(area_data['coordinates_x'])
                elif isinstance(area_data, (list, np.ndarray)):
                    total_injected += len(area_data)
            
            return total_injected
            
        except Exception as e:
            if debug_enabled:
                logger.error("PowerInjectionService: Error in external activation injection: %s", str(e))
            else:
                logger.error("Error injecting external activations: %s", str(e))
            return 0
    
    def _get_power_neurons(self) -> List[int]:
        """Get neurons from power areas (cached for performance)."""
        if self._cache_valid and self._power_neurons_cache is not None:
            return self._power_neurons_cache
        
        power_neurons = []
        
        # Debug power neuron detection
        debug_enabled = (hasattr(self.connectome_manager, 'state_manager') and 
                        self.connectome_manager.state_manager and 
                        self.connectome_manager.state_manager.is_debug_npu_enabled())
        
        if debug_enabled:
            logger.debug("PowerInjectionService: Starting power neuron detection...")
        
        try:
            # Check if connectome manager has NPU interface
            if hasattr(self.connectome_manager, '_npu_interface') and self.connectome_manager._npu_interface:
                if debug_enabled:
                    logger.debug("PowerInjectionService: Found NPU interface")
            else:
                if debug_enabled:
                    logger.debug("PowerInjectionService: No NPU interface found on connectome_manager")
                    logger.debug("PowerInjectionService: ConnectomeManager attributes: %s", 
                                  [attr for attr in dir(self.connectome_manager) if not attr.startswith('_')])
                return power_neurons  # Return empty list
            
            if hasattr(self.connectome_manager, '_npu_interface') and self.connectome_manager._npu_interface:
                npu_interface = self.connectome_manager._npu_interface
                
                # Look for power areas by checking cortical area names
                if hasattr(npu_interface, 'cortical_areas'):
                    if debug_enabled:
                        logger.debug("PowerInjectionService: Found %d cortical areas to scan", len(npu_interface.cortical_areas))
                        
                        # ENHANCED DEBUG: Show all cortical areas for diagnosis
                        logger.debug("PowerInjectionService: === ALL CORTICAL AREAS ===")
                        for idx, area_data in npu_interface.cortical_areas.items():
                            cortical_id = area_data.get('cortical_id', '')
                            neuron_count = len(npu_interface.get_neurons_by_area(idx)) if hasattr(npu_interface, 'get_neurons_by_area') else 'unknown'
                            logger.debug("PowerInjectionService: cortical_idx=%s, cortical_id='%s', neurons=%s", idx, cortical_id, neuron_count)
                        logger.debug("PowerInjectionService: === END CORTICAL AREAS ===")
                    
                    # First: Check for reserved power area at cortical_idx=1 (old architecture)
                    if 1 in npu_interface.cortical_areas:
                        area_data = npu_interface.cortical_areas[1]
                        cortical_id = area_data.get('cortical_id', '')
                        neurons = npu_interface.get_neurons_by_area(1)
                        power_neurons.extend(neurons)
                        if debug_enabled:
                            logger.debug("PowerInjectionService: Found RESERVED power area at cortical_idx=1 (cortical_id='%s'): %d neurons", cortical_id, len(neurons))
                        else:
                            logger.info("Found reserved power area at cortical_idx=1 ('%s'): %d neurons", cortical_id, len(neurons))
                    
                    # Second: Scan all areas for power naming patterns (fallback)
                    for area_data in npu_interface.cortical_areas.values():
                        cortical_id = area_data.get('cortical_id', '')
                        cortical_idx = area_data.get('cortical_idx')
                        
                        # Skip cortical_idx=1 since we already checked it above
                        if cortical_idx == 1:
                            continue
                        
                        if debug_enabled:
                            logger.debug("PowerInjectionService: Checking area '%s' at cortical_idx=%s", cortical_id, cortical_idx)
                        
                        # Detect power areas (various naming patterns)
                        if (cortical_id == '_power' or 
                            cortical_id == '___pwr' or
                            cortical_id.endswith('_pwr') or 
                            cortical_id.endswith('_power') or
                            '_pwr' in cortical_id or
                            '_power' in cortical_id):
                            
                            if cortical_idx is not None:
                                neurons = npu_interface.get_neurons_by_area(cortical_idx)
                                power_neurons.extend(neurons)
                                if debug_enabled:
                                    logger.debug("PowerInjectionService: Found power area '%s' at cortical_idx=%s: %d neurons", cortical_id, cortical_idx, len(neurons))
                                else:
                                    logger.info("Found power area '%s' at cortical_idx=%s: %d neurons", cortical_id, cortical_idx, len(neurons))
                else:
                    if debug_enabled:
                        logger.debug("PowerInjectionService: NPU interface has no cortical_areas attribute")
            
            # Cache the result
            self._power_neurons_cache = power_neurons
            self._cache_valid = True
            
            if debug_enabled:
                logger.debug("PowerInjectionService: Power detection complete - %d total neurons found", len(power_neurons))
                if not power_neurons:
                    logger.debug("PowerInjectionService: No power areas detected - brain will rely entirely on external stimulation")
                    logger.debug("PowerInjectionService: Consider adding '_power', 'xxx_pwr', or 'xxx_power' cortical area to genome")
            
            if power_neurons:
                logger.info("Power neuron detection complete: %d total power neurons cached", len(power_neurons))
            else:
                logger.warning("No power areas detected - consider adding '_power' area to genome for constant brain activity")
                
        except Exception:
            logger.error("Error detecting power neurons")
            power_neurons = []
            
        return power_neurons
    
    def invalidate_cache(self):
        """Invalidate power neuron cache (call when genome changes)."""
        self._cache_valid = False
        self._power_neurons_cache = None