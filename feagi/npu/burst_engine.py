"""
Clean Burst Engine for FEAGI NPU

Complete rewrite with clear separation of concerns and proper data flow:
FCL (candidates) → Fire Queue (firing) → Fire Ledger (history)
"""

from typing import Dict, List, Optional, Any
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
        
        # Phase 2: Process neural dynamics - convert FCL candidates to actual firing neurons
        fire_queue = FireQueue()
        
        if self.connectome_manager:
            try:
                # Use connectome manager to process membrane potentials vs thresholds
                fired_neuron_ids = self.connectome_manager.update_membrane_potentials(
                    current_timestep=self.current_timestep
                )
                
                if fired_neuron_ids:
                    # Convert fired neuron IDs to FiringNeuron objects and add to fire queue
                    firing_neurons = self._create_firing_neurons(fired_neuron_ids)
                    fire_queue.add_fired_neurons(firing_neurons, self.current_timestep)
                    logger.debug("Phase 2: %d neurons fired at timestep %d", len(fired_neuron_ids), self.current_timestep)
                else:
                    logger.debug("Phase 2: No neurons fired at timestep %d", self.current_timestep)
                    
            except Exception:
                logger.error("Error in neural processing phase")
                # Continue with empty fire queue to maintain burst cycle
        else:
            logger.debug("No connectome manager - skipping neural processing phase")
        
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
            except Exception:
                # Don't let state manager issues break burst processing
                logger.debug("Failed to update activity counters")
        
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
                    "[NPU-DEBUG] BURST ENGINE: Instance %d _running changed: %s -> %s", id(self), old_value, value
                )
        except Exception:
            # Don't let debug logging break normal operation
            pass
    
    def _set_burst_engine_state(self, state: ServiceState):
        """Set burst engine state in the state manager."""
        try:
            if self.state_manager:
                self.state_manager.set_burst_engine_state(state)
                logger.debug("BurstEngine state updated to: %s", state)
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
            if frequency_hz <= 0 or frequency_hz > 10000:
                logger.error("Invalid frequency %dHz (must be 0 < freq <= 10000)", frequency_hz)
                return False
            
            self.desired_frequency = frequency_hz
            
            # Update state manager
            if self.state_manager:
                self.state_manager.set_burst_frequency(frequency_hz)
                logger.info("BurstEngine: Updated frequency to %dHz in state manager", frequency_hz)
            
            return True
        except Exception:
            logger.error("Failed to update burst engine frequency")
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
            logger.info("[BURST-ENGINE] Starting main processing loop")
            self.start()
            
            # Enter the main processing loop
            if self._running:
                logger.info("[BURST-ENGINE] Main loop started successfully at %dHz", self.desired_frequency) 
                
                # Calculate burst interval from frequency
                burst_interval = 1.0 / self.desired_frequency if self.desired_frequency > 0 else 0.1
                logger.info("[BURST-ENGINE] Burst interval: %.4fs", burst_interval)
                
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
                        
                        # RTOS-compliant timing: no blocking sleep operations
                        # Burst engine relies on external scheduler for timing control
                        
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
                        logger.info("[GENOME-SYNC] Burst engine synced with %d neurons", neuron_count)
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
            
            # Invalidate power neuron cache on genome changes
            if self.injection_service and hasattr(self.injection_service, 'invalidate_cache'):
                self.injection_service.invalidate_cache()
                logger.debug("Power neuron cache invalidated after genome update")
            
            # Mark that genome data has been integrated
            self.genome_loaded = True
            
            logger.info("✅ Burst engine updated with new genome successfully")
            
        except Exception:
            logger.error("Error updating burst engine with genome")
            # Don't raise - genome loading should not fail due to burst engine update issues
    
    def _inject_all_candidates(self, fcl: FireCandidateList):
        """Inject all candidates into FCL - power neurons, sensory data, and synaptic propagation."""
        
        # 1. CRITICAL: Inject power neurons and special areas EVERY burst
        if self.injection_service and self.enable_injection:
            try:
                injected_count = self.injection_service.inject_power_neurons(fcl, self.burst_count)
                if injected_count > 0:
                    logger.debug("Power injection: %d neurons added to FCL", injected_count)
            except Exception:
                logger.error("Error in power neuron injection")
        
        # 2. Inject sensory data (if FCL injector available)
        if self.fcl_injector:
            # Synaptic propagation from previous timestep
            if self.previous_fire_queue:
                propagation_data = self._compute_synaptic_propagation()
                self.fcl_injector.inject_synaptic_propagation(fcl, propagation_data)
        else:
            logger.debug("FCL injector not available - no connectome manager provided")
        
        logger.debug("FCL injection complete - power neurons and synaptic propagation processed")
    
    def _compute_synaptic_propagation(self) -> Dict[int, List[tuple]]:
        """Compute synaptic propagation data from previous fire queue."""
        # Placeholder for synaptic propagation computation
        # This will integrate with actual connectome synaptic data
        return {}
    
    def _create_firing_neurons(self, fired_neuron_ids: List[int]) -> List[FiringNeuron]:
        """Convert fired neuron IDs to FiringNeuron objects with properties."""
        # Pre-allocate list for RTOS compliance (no dynamic growth)
        firing_neurons = [None] * len(fired_neuron_ids)
        
        try:
            for idx, neuron_id in enumerate(fired_neuron_ids):
                # Get neuron properties from connectome manager or NPU interface
                cortical_idx = 0  # Default cortical area
                membrane_potential = 1.0  # Default membrane potential
                coordinates = (0, 0, 0)  # Default coordinates
                threshold = 1.0  # Default threshold
                
                # Try to get actual properties from NPU interface
                if (hasattr(self.connectome_manager, '_npu_interface') and 
                    self.connectome_manager._npu_interface):
                    
                    npu_interface = self.connectome_manager._npu_interface
                    
                    # Get cortical area for this neuron
                    if hasattr(npu_interface, 'neuron_to_area') and neuron_id in npu_interface.neuron_to_area:
                        cortical_idx = npu_interface.neuron_to_area[neuron_id]
                    
                    # Get coordinates (simplified - may need more complex lookup)
                    # For now, use neuron_id as coordinate basis
                    x = neuron_id % 10
                    y = (neuron_id // 10) % 10  
                    z = (neuron_id // 100) % 10
                    coordinates = (x, y, z)
                
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
                
                firing_neurons[idx] = firing_neuron
            
            logger.debug("Created %d FiringNeuron objects", len(firing_neurons))
            
        except Exception:
            logger.error("Error creating firing neurons")
            # Return empty list to prevent breaking burst cycle
            
        return firing_neurons
    
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
    
    def inject_power_neurons(self, fcl: FireCandidateList, current_timestep: int) -> int:
        """Inject power neurons into FCL every burst for constant brain activity."""
        try:
            # Get power neurons (cached for performance)
            power_neurons = self._get_power_neurons()
            
            if not power_neurons:
                # Only log occasionally to avoid spam
                if current_timestep % 1000 == 0:
                    logger.debug("No power neurons found at timestep %d", current_timestep)
                return 0
            
            # Add power neurons to FCL using SoA format
            if power_neurons:
                # Convert to numpy arrays for SoA format
                neuron_ids = np.array(power_neurons, dtype=np.uint32)
                potential_deltas = np.full(len(power_neurons), 1.0, dtype=np.float32)  # High potential for power neurons
                excitatory_mask = np.ones(len(power_neurons), dtype=bool)  # All excitatory
                
                # Add all power neurons to cortical area 0 (generic power injection)
                injected_count = fcl.add_candidates_soa(
                    cortical_idx=0,  # Generic power area index
                    neuron_ids=neuron_ids,
                    potential_deltas=potential_deltas,
                    excitatory_mask=excitatory_mask
                )
            else:
                injected_count = 0
            
            # Log occasionally (every 100 bursts) to avoid spam
            if current_timestep % 100 == 0:
                logger.info("Power injection: %d neurons injected at burst %d", injected_count, current_timestep)
            
            return injected_count
            
        except Exception:
            logger.error("Error injecting power neurons")
            return 0
    
    def _get_power_neurons(self) -> List[int]:
        """Get neurons from power areas (cached for performance)."""
        if self._cache_valid and self._power_neurons_cache is not None:
            return self._power_neurons_cache
        
        power_neurons = []
        
        try:
            # Check if connectome manager has NPU interface
            if hasattr(self.connectome_manager, '_npu_interface') and self.connectome_manager._npu_interface:
                npu_interface = self.connectome_manager._npu_interface
                
                # Look for power areas by checking cortical area names
                if hasattr(npu_interface, 'cortical_areas'):
                    for area_data in npu_interface.cortical_areas.values():
                        cortical_id = area_data.get('cortical_id', '')
                        
                        # Detect power areas (various naming patterns)
                        if (cortical_id == '_power' or 
                            cortical_id.endswith('_pwr') or 
                            cortical_id.endswith('_power') or
                            '_pwr' in cortical_id):
                            
                            cortical_idx = area_data.get('cortical_idx')
                            if cortical_idx is not None:
                                neurons = npu_interface.get_neurons_by_area(cortical_idx)
                                power_neurons.extend(neurons)
                                logger.info("Found power area '%s': %d neurons", cortical_id, len(neurons))
            
            # Cache the result
            self._power_neurons_cache = power_neurons
            self._cache_valid = True
            
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