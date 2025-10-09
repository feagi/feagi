"""
Clean Burst Engine for FEAGI NPU

Complete rewrite with clear separation of concerns and proper data flow:
FCL (candidates) → Fire Queue (firing) → Fire Ledger (history)
"""

from typing import Dict, List, Optional, Any, Union, Tuple
import numpy as np
from feagi.utils.logger import setup_logger
from feagi.core.state_manager import FeagiStateManager, ServiceState

from .fire_queue import FireQueue, FiringNeuron
from .fq_sampler import FQSampler

# Rust NPU integration (PRODUCTION PATH - NO FALLBACKS)
from .rust_npu_integration import RustNPUIntegration, RUST_AVAILABLE

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
        if cls._instance is None:
            cls._instance = cls(connectome_manager, state_manager, fire_ledger_window_size)
        return cls._instance
    
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
        
        # Initialize logger
        self.logger = setup_logger(__name__)
        
        # STATE MANAGER INTEGRATION - Cache instance for performance 
        # Always use singleton instance as single source of truth
        self.state_manager = FeagiStateManager.instance()
        # BurstEngine using FeagiStateManager singleton
        
        # Set initial burst engine state to INITIALIZING
        try:
            # Prefer direct state manager call to avoid attribute resolution issues during import
            if self.state_manager:
                self.state_manager.set_burst_engine_state(ServiceState.INITIALIZING.value)
            else:
                # Fallback path should never happen since we cache the singleton above
                pass
        except Exception:
            # Do not block initialization on state publish
            pass
        
        # Core NPU components
        # NOTE: Fire Ledger is now in Rust NPU - no Python Fire Ledger needed!
        
        # Per-area excitability cache for performance optimization (like old NPU)
        self._area_excitability_cache: Dict[int, float] = {}
        self._excitability_cache_dirty = True
        self._rng = np.random.default_rng()  # RNG for excitability (when needed)
        
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
        try:
            state_freq = self.state_manager.get_burst_frequency() if self.state_manager else None
            if state_freq and state_freq > 0:
                self.desired_frequency = float(state_freq)
            else:
                # Use safe default and let state manager be updated by external controller
                self.desired_frequency = 10.0  # @architecture:acceptable - emergency fallback
        except Exception:
            self.desired_frequency = 10.0  # @architecture:acceptable - emergency fallback
        self.target_frequency = self.desired_frequency
        
        # Running state tracking (with debug integration)
        self._running_state = False
        
        # Genome integration tracking
        self.genome_loaded = False
        
        # Instance ID for debugging and tracking (RTOS-safe)
        self._instance_id = "burst_engine_%d" % id(self)
        
        self.current_timestep = 0
        self.burst_count = 0
        self.previous_fire_queue: Optional[FireQueue] = None
        
        # Performance monitoring for actual vs desired frequency
        self._last_burst_time = None
        self._burst_times = []  # Track last N burst times for moving average
        self._burst_times_window = 10  # Use 10 bursts for moving average
        self._performance_log_interval = 100  # Log performance every N bursts
        self._last_performance_log = 0

        # Compatibility adapter for legacy MemoryProcessor access in ConnectomeManager
        try:
            self.memory_processor = _MemoryProcessorAdapter(self)
        except Exception:
            self.memory_processor = None

        # Initialize injection service for power areas and special neurons
        try:
            import inspect
            src_path = inspect.getsourcefile(self.__class__)
            logger.info("[DEBUG] BurstEngine class loaded from: %s", src_path)
        except Exception:
            pass
        if hasattr(self, "_initialize_injection_service"):
            self._initialize_injection_service()
        else:
            logger.warning("[INJECTION] _initialize_injection_service not found on BurstEngine; skipping injection init")
        
        # Initialize Rust NPU (PRODUCTION PATH - NO FALLBACKS)
        self._rust_npu_integration = None
        if not RUST_AVAILABLE:
            raise RuntimeError(
                "🦀 [RUST-NPU] CRITICAL: Rust NPU not available! "
                "FEAGI requires the Rust NPU for production use. "
                "Please build the Rust components: cd feagi-rust && cargo build --release"
            )
        logger.info("🦀 [RUST-NPU] Rust NPU available - will initialize after genome load")
        
        # Mark as initialized but keep in INITIALIZING state until burst processing works
        self._initialized = True
        # DON'T set to READY yet - wait until burst processing actually works

        logger.info("BurstEngine initialized with singleton pattern and state manager integration")
    
    def _initialize_rust_npu(self) -> None:
        """Initialize the complete Rust NPU with connectome data.
        
        PRODUCTION PATH: Direct Rust NPU initialization with no fallbacks.
        This is called on first burst processing.
        
        Raises:
            RuntimeError: If initialization fails (fail fast)
        """
        if self._rust_npu_integration:
            return  # Already initialized
            
        if not self.connectome_manager:
            raise RuntimeError("🦀 [RUST-NPU] Cannot initialize without connectome_manager")
        
        # ✅ NEW ARCHITECTURE: Get existing Rust NPU from NPUInterface
        # NPUInterface already created and populated the Rust NPU during neurogenesis
        if hasattr(self.connectome_manager, '_npu_interface') and self.connectome_manager._npu_interface:
            npu_interface = self.connectome_manager._npu_interface
            
            if hasattr(npu_interface, '_rust_npu_integration') and npu_interface._rust_npu_integration:
                # Use the EXISTING Rust NPU that was populated by neurogenesis
                self._rust_npu_integration = npu_interface._rust_npu_integration
                logger.info("🦀 [RUST-NPU] ✅ Connected to existing Rust NPU from NPUInterface")
                logger.info("🦀 [RUST-NPU] Current state: %d neurons, %d synapses",
                           self._rust_npu_integration.get_neuron_count(),
                           self._rust_npu_integration.get_synapse_count())
                logger.error(f"🦀 [INSTANCE-DEBUG] BurstEngine RustNPU instance ID: {id(self._rust_npu_integration._rust_npu)}")
                logger.error(f"🦀 [INSTANCE-DEBUG] NPUInterface RustNPU instance ID: {id(npu_interface._rust_npu_integration._rust_npu)}")
                return
        
        # Fallback: Create new Rust NPU (should not happen in normal flow)
        logger.warning("🦀 [RUST-NPU] NPUInterface Rust NPU not found - creating new one (unexpected!)")
        
        neuron_capacity = 100_000  # Default
        synapse_capacity = 500_000  # Default
        
        if hasattr(self.connectome_manager, '_npu_interface') and self.connectome_manager._npu_interface:
            npu_interface = self.connectome_manager._npu_interface
            neuron_capacity = npu_interface.max_neurons
            synapse_capacity = npu_interface.max_synapses
        
        self._rust_npu_integration = RustNPUIntegration(
            connectome_manager=self.connectome_manager,
            fire_ledger_window=fire_ledger_window_size,
            neuron_capacity=neuron_capacity,
            synapse_capacity=synapse_capacity
        )
        
        if hasattr(self.connectome_manager, '_npu_interface') and self.connectome_manager._npu_interface:
            self.connectome_manager._npu_interface.set_rust_npu_integration(self._rust_npu_integration)
        
        logger.info("🦀 [RUST-NPU] Created new Rust NPU: %d neurons, %d synapses",
                    self._rust_npu_integration.get_neuron_count(),
                    self._rust_npu_integration.get_synapse_count())
    
    def reinitialize_rust_npu(self) -> None:
        """Rebuild synapse indexes after morphology changes.
        
        NOTE: This does NOT recreate the Rust NPU - it only rebuilds the synapse index
        to pick up newly created synapses. The Rust NPU maintains the single source of truth.
        """
        logger.info("🦀 [RUST-NPU] Rebuilding synapse indexes after morphology change")
        
        if self._rust_npu_integration is None:
            logger.warning("🦀 [RUST-NPU] No Rust NPU integration - skipping index rebuild")
            return
        
        try:
            # Just rebuild the synapse index - synapses are already in the Rust NPU
            self._rust_npu_integration._rust_npu.rebuild_indexes()
            synapse_count = self._rust_npu_integration._rust_npu.get_synapse_count()
            logger.info(f"🦀 [RUST-NPU] ✅ Synapse indexes rebuilt ({synapse_count} synapses)")
        except Exception as e:
            logger.error(f"🦀 [RUST-NPU] Failed to rebuild synapse indexes: {e}")
            logger.exception("Full stack trace:")
            raise
    
    def _old_reinitialize_rust_engine(self) -> bool:
        """Force re-initialization of Rust engine (e.g., after connectome changes).
        
        This should be called after genome loading or synapse modifications.
        """
        logger.info("🦀 [RUST] Force re-initialization requested")
        self._rust_engine = None
        self._rust_engine_initialized = False
        return self._initialize_rust_engine()
    
    def process_burst(self) -> List[int]:
        """Execute burst processing using Rust NPU.
        
        PRODUCTION PATH: All processing happens in Rust for maximum performance.
        
        Returns:
            List[int]: Neuron IDs that fired in current timestep
        """
        # Initialize Rust NPU on first use
        if not self._rust_npu_integration:
            self._initialize_rust_npu()
        
        # Collect all neurons to inject (power + manual stimulation + sensory)
        power_neurons = []
        manual_neurons = []
        
        # 1. Get power neurons
        if self.injection_service and self.enable_injection:
            try:
                power_neurons = self.injection_service._get_power_neurons()
                logger.debug("🦀 Power neurons retrieved: %d neurons - %s", 
                           len(power_neurons),
                           power_neurons[:10] if len(power_neurons) > 10 else power_neurons)
            except Exception as e:
                logger.error("🦀 [POWER-DEBUG] Failed to get power neurons: %s", str(e), exc_info=True)
        else:
            logger.debug("🦀 injection_service=%s, enable_injection=%s", 
                          self.injection_service, self.enable_injection)
        
        # 2. Get manual stimulation / sensory neurons
        if hasattr(self, '_pending_external_activations') and self._pending_external_activations:
            pending = getattr(self, '_pending_external_activations', {})
            for area_id, area_data in pending.items():
                # Convert coordinates to neuron IDs
                if isinstance(area_data, dict) and 'coordinates_x' in area_data:
                    coords_x = area_data.get('coordinates_x', [])
                    coords_y = area_data.get('coordinates_y', [])
                    coords_z = area_data.get('coordinates_z', [])
                    
                    # Use spatial hash to look up neuron IDs
                    if hasattr(self.connectome_manager, '_spatial_hash') and self.connectome_manager._spatial_hash:
                        spatial_hash = self.connectome_manager._spatial_hash
                        for x, y, z in zip(coords_x, coords_y, coords_z):
                            try:
                                neuron_id = spatial_hash.get_neuron_at_coordinate(area_id, int(x), int(y), int(z))
                                if neuron_id is not None:
                                    manual_neurons.append(neuron_id)
                            except Exception as e:
                                logger.warning("Failed to resolve neuron ID for %s[%d,%d,%d]: %s", 
                                             area_id, x, y, z, e)
                    else:
                        logger.warning("Spatial hash not available for coordinate-to-neuron lookup!")
                # Direct neuron ID list
                elif isinstance(area_data, (list, np.ndarray)):
                    manual_neurons.extend([int(nid) for nid in area_data])
            
            # Clear pending activations after collection
            self._pending_external_activations.clear()
            
            logger.info("🦀 [RUST-NPU] Manual stimulation neurons retrieved: %d neurons - %s", 
                       len(manual_neurons),
                       manual_neurons[:10] if len(manual_neurons) > 10 else manual_neurons)
        
        # Combine all injection neurons
        all_injection_neurons = power_neurons + manual_neurons
        
        logger.debug("🦀 Total injection neurons: %d (power=%d, manual=%d) - %s", 
                      len(all_injection_neurons), len(power_neurons), len(manual_neurons),
                      all_injection_neurons[:10] if len(all_injection_neurons) > 10 else all_injection_neurons)
        
        # Process burst in Rust (ALL IN RUST!)
        result = self._rust_npu_integration.process_burst(power_neurons=all_injection_neurons)
        
        logger.debug("🦀 Rust result: neuron_count=%d, fired_neurons=%s", 
                      result.get('neuron_count', 0), 
                      result.get('fired_neurons', [])[:10])
        
        # Update internal state
        self.burst_count = result['burst']
        self.current_timestep += 1
        
        # Create FiringNeuron objects and FireQueue for API/FQ sampler compatibility
        if result['neuron_count'] > 0:
            logger.debug("🦀 Creating FireQueue for %d fired neurons", result['neuron_count'])
            
            # Get Rust NPU for querying live neuron state
            rust_npu = None
            if self._rust_npu_integration and hasattr(self._rust_npu_integration, '_rust_npu'):
                rust_npu = self._rust_npu_integration._rust_npu
            
            # Get actual neuron properties from connectome for proper visualization
            firing_neurons = []
            neurons_by_area = {}  # Track neurons per area for summary
            failed_lookups = 0
            
            for neuron_id in result['fired_neurons']:
                # Try to get actual neuron properties from connectome and Rust NPU
                try:
                    cortical_area = self.connectome_manager.get_cortical_area_for_neuron(neuron_id) if self.connectome_manager else None
                    cortical_idx = self.connectome_manager.get_cortical_idx_for_id(cortical_area) if cortical_area else 0
                    coords = self.connectome_manager.get_neuron_position(neuron_id) if self.connectome_manager else (0, 0, 0)
                    
                    # Query actual neuron state from Rust NPU for accurate visualization
                    membrane_potential = 0.0
                    threshold = 1.0
                    refractory_counter = 0
                    consecutive_fire_count = 0
                    
                    if rust_npu:
                        try:
                            state = rust_npu.get_neuron_state(neuron_id)
                            if state:
                                # state = (cfc, cfc_limit, snooze_period, potential, threshold, refrac_countdown)
                                cfc, _cfc_limit, _snooze, potential, thresh, refrac = state
                                membrane_potential = float(potential)
                                threshold = float(thresh)
                                refractory_counter = int(refrac)
                                consecutive_fire_count = int(cfc)
                        except Exception as e:
                            # Use defaults if query fails
                            logger.debug(f"Failed to get state for neuron {neuron_id}: {e}")
                    
                    # Track neurons by area
                    area_key = cortical_area if cortical_area else "unknown"
                    neurons_by_area[area_key] = neurons_by_area.get(area_key, 0) + 1
                    
                    # Only log first 2 per area to avoid log spam
                    if neurons_by_area[area_key] <= 2:
                        logger.debug(f"🦀 Neuron {neuron_id} -> area={cortical_area}, idx={cortical_idx}")
                except Exception as e:
                    failed_lookups += 1
                    if failed_lookups <= 5:  # Only log first 5 failures
                        logger.warning(f"🦀 [RUST-NPU] Failed to get properties for neuron {neuron_id}: {e}")
                    cortical_idx = 0
                    coords = (0, 0, 0)
                    membrane_potential = 0.0
                    threshold = 1.0
                    refractory_counter = 0
                    consecutive_fire_count = 0
                
                firing_neurons.append(FiringNeuron(
                    neuron_id=neuron_id,
                    membrane_potential=membrane_potential,
                    cortical_idx=cortical_idx,
                    coordinates=coords,
                    threshold=threshold,
                    refractory_counter=refractory_counter,
                    consecutive_fire_count=consecutive_fire_count
                ))
            
            # Log summary (reduced spam)
            logger.debug("🦀 Processed %d neurons: %s (failed_lookups=%d)", 
                          len(firing_neurons), neurons_by_area, failed_lookups)
            
            # Update previous_fire_queue (used by FQ sampler via get_current_fire_queue())
            fire_queue = FireQueue()
            fire_queue.add_fired_neurons(firing_neurons, self.current_timestep)
            self.previous_fire_queue = fire_queue
            
            # NOTE: Fire Ledger archival now happens in Rust NPU's process_burst()!
            # No Python archival needed - Rust handles it automatically.
            
            # Debug: Check what's in the fire queue (reduced spam)
            logger.debug("🦀 FireQueue updated: %d neurons, %d areas", 
                       result['neuron_count'], 
                       len(fire_queue.firing_neurons_by_area))
        else:
            # No neurons fired, but still update the reference
            logger.debug("🦀 No neurons fired (neuron_count=%d), creating empty FireQueue", 
                          result.get('neuron_count', 0))
            self.previous_fire_queue = FireQueue()
            # NOTE: Rust Fire Ledger handles empty timesteps automatically
            
        return result['fired_neurons']
    
    def get_current_fire_queue(self) -> Optional[FireQueue]:
        """Get current fire queue for FQ Sampler access."""
        return self.previous_fire_queue
    
    def initialize_fq_sampler(self, sample_frequency_hz: float = 10.0, sampling_mode: str = "visualization") -> FQSampler:
        """Initialize FQ Sampler with this burst engine as provider."""
        self.fq_sampler = FQSampler(
            fire_queue_provider=self,  # BurstEngine provides get_current_fire_queue()
            sample_frequency_hz=sample_frequency_hz,
            sampling_mode=sampling_mode
        )
        logger.debug("FQ Sampler initialized: %s @ %dHz", sampling_mode, sample_frequency_hz)
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
                logger.debug("FQ sampler [%s] registered with BurstEngine", sampler_id)
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
                logger.debug(f"Setting burst engine state to {state.name} ({state.value})")
                result = self.state_manager.set_burst_engine_state(state.value)
                logger.debug(f"State manager set result: {result}")
                
                # Check if the result indicates an error
                if hasattr(result, 'is_err') and result.is_err:
                    error_msg = result.unwrap_err() if hasattr(result, 'unwrap_err') else "Unknown error"
                    logger.error(f"State manager returned error: {error_msg}")
                    raise Exception(f"State manager error: {error_msg}")
                
                # Verify the state was set
                current_state = self.state_manager.get_burst_engine_state()
                logger.debug(f"Verified state after set: {current_state}")
                if current_state != state.value:
                    logger.error(f"State manager failed to update: expected {state.value}, got {current_state}")
                    raise Exception(f"State verification failed: expected {state.value}, got {current_state}")
            else:
                logger.error("No state manager available for burst engine state update")
                raise Exception("No state manager available")
        except Exception as e:
            logger.error(f"Failed to update burst engine state: {e}")
            import traceback
            logger.error(f"Full traceback: {traceback.format_exc()}")
            raise  # Re-raise to propagate the error
    
    def _notify_sensory_stream_ready(self):
        """Notify the sensory stream that the burst engine is ready."""
        try:
            # Get the ZMQ server instance from the process manager
            from feagi.process_manager import ProcessManager
            process_manager = ProcessManager.get_instance()
            if process_manager and hasattr(process_manager, '_zmq_server'):
                zmq_server = process_manager._zmq_server
                if zmq_server and hasattr(zmq_server, '_sensory') and zmq_server._sensory:
                    # Schedule the sensory stream start in the ZMQ server's event loop
                    import asyncio
                    if zmq_server._loop and zmq_server._loop.is_running():
                        asyncio.run_coroutine_threadsafe(
                            zmq_server._sensory.start_when_burst_engine_ready(),
                            zmq_server._loop
                        )
                        logger.info("🔔 Notified sensory stream that burst engine is ready")
                    else:
                        logger.debug("ZMQ server event loop not available for sensory stream notification")
                else:
                    logger.debug("Sensory stream not available for notification")
            else:
                logger.debug("Process manager or ZMQ server not available for sensory stream notification")
        except Exception as e:
            logger.debug(f"Failed to notify sensory stream: {e}")
    
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
    
    def update_with_genome(self, connectome_manager) -> bool:
        """Update burst engine with genome data and connectome manager.
        
        This method is called after genome loading to ensure the burst engine
        has access to the connectome manager and can process neural dynamics.
        """
        try:
            logger.info("🧬 [GENOME-UPDATE] Updating burst engine with new genome...")
            
            if connectome_manager:
                self.connectome_manager = connectome_manager
                logger.info(f"🧬 [GENOME-UPDATE] Connectome manager updated: {type(connectome_manager)}")
                
                # Initialize injection service if enabled
                if self.enable_injection:
                    try:
                        self.injection_service = PowerInjectionService(connectome_manager)
                        logger.info("🧬 [GENOME-UPDATE] Power injection service initialized")
                    except Exception as injection_error:
                        logger.warning(f"🧬 [GENOME-UPDATE] Failed to initialize injection service: {injection_error}")
                
                logger.info("🧬 [GENOME-UPDATE] ✅ Burst engine updated successfully with genome")
                return True
            else:
                logger.warning("🧬 [GENOME-UPDATE] ⚠️ No connectome manager provided")
                return False
                
        except Exception as e:
            logger.error(f"🧬 [GENOME-UPDATE] ❌ Failed to update burst engine with genome: {e}")
            import traceback
            logger.error(f"🧬 [GENOME-UPDATE] Full traceback: {traceback.format_exc()}")
            return False

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
            
            logger.info("🔧 [START-DEBUG] Checking connectome manager availability...")
            if self.connectome_manager:
                logger.info(f"🔧 [START-DEBUG] Connectome manager available: {type(self.connectome_manager)}")
            else:
                logger.warning("🔧 [START-DEBUG] ⚠️ No connectome manager available - burst engine will run with limited functionality")
            
            logger.info("🔧 [START-DEBUG] Setting burst engine state to INITIALIZING...")
            self._set_burst_engine_state(ServiceState.INITIALIZING)
            
            self._running = True
            
            logger.info("🔧 [START-DEBUG] Setting burst engine state to READY...")
            self._set_burst_engine_state(ServiceState.READY)
            
            logger.info("BurstEngine: Started successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to start BurstEngine: {e}")
            import traceback
            logger.error(f"Full traceback: {traceback.format_exc()}")
            try:
                self._set_burst_engine_state(ServiceState.ERROR)
            except Exception as state_error:
                logger.error(f"Failed to set ERROR state: {state_error}")
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
            
            # Add detailed debug logging for startup
            logger.info("🔧 [RUN-DEBUG] About to call self.start()...")
            try:
                self.start()
                logger.info(f"🔧 [RUN-DEBUG] self.start() completed successfully - _running: {self._running}")
            except Exception as start_error:
                logger.error(f"🚨 [RUN-DEBUG] self.start() failed with error: {start_error}")
                import traceback
                logger.error(f"🚨 [RUN-DEBUG] Full traceback: {traceback.format_exc()}")
                raise  # Re-raise to be caught by outer try-catch
            
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
                import time
                while self._running:
                    try:
                        # Measure actual burst timing
                        burst_start_time = time.perf_counter()
                        
                        # Execute the actual burst processing (RTOS-safe: no timing calls)
                        try:
                            fired_neurons = self.process_burst()
                            burst_process_time = time.perf_counter() - burst_start_time
                            
                            # Track actual burst frequency
                            if self._last_burst_time is not None:
                                actual_burst_interval = burst_start_time - self._last_burst_time
                                self._burst_times.append(actual_burst_interval)
                                
                                # Keep only last N burst times for moving average
                                if len(self._burst_times) > self._burst_times_window:
                                    self._burst_times.pop(0)
                            
                            self._last_burst_time = burst_start_time
                            
                            # Performance logging every N bursts
                            if self.burst_count % self._performance_log_interval == 0 and len(self._burst_times) > 0:
                                # Calculate actual frequency from moving average
                                avg_interval = sum(self._burst_times) / len(self._burst_times)
                                actual_hz = 1.0 / avg_interval if avg_interval > 0 else 0.0
                                
                                # Calculate how much time burst processing took
                                processing_percent = (burst_process_time / avg_interval * 100.0) if avg_interval > 0 else 0.0
                                
                                # Log performance
                                if actual_hz < self.desired_frequency * 0.9:  # If actual < 90% of desired
                                    logger.warning(
                                        "⚠️ [BURST-ENGINE] PERFORMANCE BOTTLENECK: Burst #%d | "
                                        "Desired: %.2f Hz | Actual: %.2f Hz (%.1f%% of target) | "
                                        "Processing time: %.1f ms (%.1f%% of interval) | "
                                        "Neurons fired: %d",
                                        self.burst_count, self.desired_frequency, actual_hz,
                                        (actual_hz / self.desired_frequency * 100.0) if self.desired_frequency > 0 else 0.0,
                                        burst_process_time * 1000.0, processing_percent,
                                        len(fired_neurons)
                                    )
                                else:
                                    logger.info(
                                        "[BURST-ENGINE] Performance: Burst #%d | "
                                        "Desired: %.2f Hz | Actual: %.2f Hz | "
                                        "Processing: %.1f ms (%.1f%% of interval) | "
                                        "Neurons: %d",
                                        self.burst_count, self.desired_frequency, actual_hz,
                                        burst_process_time * 1000.0, processing_percent,
                                        len(fired_neurons)
                                    )
                                
                        except Exception as e:
                            logger.error("Error in burst processing #%d: %s", self.burst_count, str(e), exc_info=True)
                            # Continue processing even if one burst fails
                        
                        # Timing control: Add sleep to prevent runaway CPU usage
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
    
    def _initialize_injection_service(self):
        """Initialize injection service for power areas and special neuron injection."""
        try:
            if not self.connectome_manager:
                logger.info("No connectome manager - injection service disabled")
                return
            
            # Create simple power injection service
            self.injection_service = PowerInjectionService(
                connectome_manager=self.connectome_manager
            )
            
            logger.info("Power injection service initialized for automatic burst injection")
            
        except Exception:
            logger.error("Failed to initialize injection service")
            self.injection_service = None
    
    # ==============================================================
    # CLASS METHODS FOR TEST COMPATIBILITY
    # ==============================================================
    
    @classmethod
    def run_with_fire_queue(cls, fire_queue: Any = None, max_iterations: int = 1) -> Dict[str, Any]:
        """Run burst engine with fire queue (class method for compatibility)."""
        instance = cls.get_instance()
        
        results = {
            'iterations': 0,
            'fired_neurons': [],
            'performance_metrics': {}
        }
        
        for i in range(max_iterations):
            try:
                # Process a single burst
                fired_neurons = instance.process_burst()
                results['fired_neurons'].extend(fired_neurons or [])
                results['iterations'] += 1
                
            except Exception as e:
                logger.error("Error in burst iteration %d: %s", i, str(e))
                break
        
        return results


# ==============================================================
# HELPER CLASSES FOR BURST ENGINE
# ==============================================================


class MemoryRegistrationService:
    """
    Separate service for tracking memory areas and their relationships.
    Used by burst engine and other components for memory pattern detection.
    """
    def __init__(self, burst_engine: BurstEngine):
        self.burst_engine = burst_engine
        self.memory_areas: Dict[str, Dict] = {}  # cortical_id -> metadata
        self.logger = logging.getLogger(__name__)
    
    def register_memory_area(
        self,
        cortical_id: str,
        pattern_type: str = "unknown",
        temporal_depth: int = 1,
        upstream_areas: Optional[set] = None
    ) -> None:
        """Register a cortical area as a memory area."""
        self.memory_areas[cortical_id] = {
            'pattern_type': pattern_type,
            'temporal_depth': temporal_depth,
            'upstream_areas': upstream_areas or set()
        }
        self.logger.debug("Registered memory area: %s (type=%s, depth=%d)", 
                         cortical_id, pattern_type, temporal_depth)
    
    def update_memory_area_upstream(self, cortical_id: str, upstream_areas: set) -> None:
        """Update upstream areas for memory area."""
        if cortical_id in self.memory_areas:
            self.memory_areas[cortical_id]['upstream_areas'] = upstream_areas


class PowerInjectionService:
    """Simple power injection service for constant brain activity."""
    
    def __init__(self, connectome_manager):
        """Initialize power injection service."""
        self.connectome_manager = connectome_manager
        self._power_neurons_cache = None
        self._cache_valid = False
        
        logger.info("PowerInjectionService created")
        
        # Invalidate cache to ensure power neurons get refractory_periods = 0 on first detection
        self.invalidate_cache()
    
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
    
    def inject_power_neurons(self, fcl: Any, current_timestep: int) -> int:
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
                
                # Convert to numpy arrays for SoA format - NO HARDCODED VALUES
                neuron_ids = np.array(power_neurons, dtype=np.uint32)
                
                # Compute delta = max(0, threshold - current_potential) per neuron (deterministic, non-accumulating)
                potential_deltas = []
                npu_interface = self.connectome_manager._npu_interface
                neuron_array = npu_interface.neuron_array if npu_interface else None
                
                if neuron_array:
                    thresholds_arr = getattr(neuron_array, 'thresholds', None)
                    current_pots_arr = getattr(neuron_array, 'membrane_potentials', None)
                    if thresholds_arr is None or current_pots_arr is None:
                        raise ValueError("Cannot inject power neurons: Missing thresholds or membrane_potentials in neuron array")
                    for power_neuron_id in power_neurons:
                        if power_neuron_id not in neuron_array.neuron_id_to_index:
                            raise ValueError(f"Power neuron {power_neuron_id} not found in neuron array")
                        idx = neuron_array.neuron_id_to_index[power_neuron_id]
                        if idx >= len(thresholds_arr) or idx >= len(current_pots_arr):
                            raise ValueError(f"Power neuron index {idx} out of bounds for thresholds or membrane_potentials")
                        threshold = float(thresholds_arr[idx])
                        current_potential = float(current_pots_arr[idx])
                        delta = threshold - current_potential
                        if delta < 0.0:
                            delta = 0.0
                        potential_deltas.append(delta)
                    potential_deltas = np.array(potential_deltas, dtype=np.float32)
                else:
                    raise ValueError("Cannot inject power neurons: No neuron array available from genome")
                
                excitatory_mask = np.ones(len(power_neurons), dtype=bool)  # All excitatory
                
                if periodic_debug:
                    logger.debug("PowerInjectionService: SoA arrays created - neuron_ids: %s, potentials: %s", 
                              neuron_ids.shape, potential_deltas.shape)
                
                # Add all power neurons to cortical area 1 (reserved _power area)
                injected_count = fcl.add_candidates_soa(
                    cortical_idx=1,  # Reserved power area index
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
            
            # Return count of power neurons injected this burst
            if periodic_debug:
                logger.debug("PowerInjectionService: Power injection complete - total=%d", injected_count)
            return injected_count
            
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
        """Buffer external activations deterministically for BurstEngine to inject via FCLInjector."""
        debug_enabled = (hasattr(self.connectome_manager, 'state_manager') and 
                        self.connectome_manager.state_manager and 
                        self.connectome_manager.state_manager.is_debug_npu_enabled())
        periodic_debug = debug_enabled and (current_timestep % 500 == 0)  # Every 50 bursts
        
        
        try:
            # Deterministic buffer for BurstEngine to process; do not merge arrays here
            if not hasattr(self.connectome_manager, 'burst_engine'):
                # Fallback: store locally for BurstEngine to read via reference
                if not hasattr(self, '_pending_external_activations'):
                    self._pending_external_activations = {}
                for area_id, area_data in activations.items():
                    self._pending_external_activations[area_id] = area_data
                return sum(
                    len(v.get('coordinates_x', [])) if isinstance(v, dict) else (len(v) if hasattr(v, '__len__') else 0)
                    for v in activations.values()
                )
            else:
                be = self.connectome_manager.burst_engine
                if not hasattr(be, '_pending_external_activations'):
                    be._pending_external_activations = {}
                for area_id, area_data in activations.items():
                    be._pending_external_activations[area_id] = area_data
                return sum(
                    len(v.get('coordinates_x', [])) if isinstance(v, dict) else (len(v) if hasattr(v, '__len__') else 0)
                    for v in activations.values()
                )
            
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
                # Only use reserved power area at cortical_idx=1
                if hasattr(npu_interface, 'cortical_areas') and 1 in npu_interface.cortical_areas:
                    area_data = npu_interface.cortical_areas[1]
                    cortical_id = area_data.get('cortical_id', '')
                    neurons = npu_interface.get_neurons_by_area(1)
                    if neurons:
                        # SINGLE KNOWN POWER NEURON: choose deterministically (minimum ID)
                        single_power_neuron = int(min(neurons))
                        power_neurons = [single_power_neuron]
                        if debug_enabled:
                            logger.debug(
                                "PowerInjectionService: Using single power neuron %d from cortical_idx=1 (cortical_id='%s')",
                                single_power_neuron,
                                cortical_id,
                            )
                        else:
                            logger.info(
                                "Using single power neuron %d from reserved power area at cortical_idx=1 ('%s')",
                                single_power_neuron,
                                cortical_id,
                            )
                    else:
                        if debug_enabled:
                            logger.debug("PowerInjectionService: Reserved power area at cortical_idx=1 has no neurons")
                else:
                    if debug_enabled:
                        logger.debug("PowerInjectionService: Reserved power area at cortical_idx=1 not found")
            
            # CRITICAL: Set refractory period to 0 for the selected power neuron (so it can fire every burst)
            if power_neurons and hasattr(self.connectome_manager, '_npu_interface'):
                npu_interface = self.connectome_manager._npu_interface
                neuron_array = getattr(npu_interface, 'neuron_array', None)
                
                if neuron_array and hasattr(neuron_array, 'refractory_periods'):
                    power_neuron_id = power_neurons[0]
                    if power_neuron_id in neuron_array.neuron_id_to_index:
                        idx = neuron_array.neuron_id_to_index[power_neuron_id]
                        if idx < len(neuron_array.refractory_periods):
                            neuron_array.refractory_periods[idx] = 0
                            logger.info("Set refractory period to 0 for power neuron %d (enables every-burst firing)", power_neuron_id)
                        elif debug_enabled:
                            logger.debug("PowerInjectionService: Power neuron index %d out of bounds for refractory update", idx)
                    elif debug_enabled:
                        logger.debug("PowerInjectionService: Power neuron %d not found in neuron_id_to_index", power_neuron_id)
            
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
        
        # When cache is invalidated, power neurons will be re-detected and 
        # their refractory periods will be set to 0 on next access
    
    # ==============================================================
    # CLASS METHODS FOR TEST COMPATIBILITY
    # ==============================================================
    
    @classmethod
    def run_with_fire_queue(cls, fire_queue: Any = None, max_iterations: int = 1) -> Dict[str, Any]:
        """Run burst engine with fire queue (class method for compatibility)."""
        instance = cls.get_instance()
        
        results = {
            'iterations': 0,
            'fired_neurons': [],
            'performance_metrics': {}
        }
        
        for i in range(max_iterations):
            try:
                # Process a single burst
                fired_neurons = instance.process_burst()
                results['fired_neurons'].extend(fired_neurons or [])
                results['iterations'] += 1
                
                # If no neurons fired, break early
                if not fired_neurons:
                    break
                    
            except Exception as e:
                logger.warning(f"Burst processing error: {e}")
                break
        
        results['performance_metrics'] = {
            'total_neurons_fired': len(results['fired_neurons']),
            'iterations_completed': results['iterations']
        }
        
        return results