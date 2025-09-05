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
        """Initialize burst frequency from state manager (single source of truth)."""
        try:
            # Get frequency from state manager
            state_frequency = self.state_manager.get_burst_frequency()
            if state_frequency and state_frequency > 0:
                self.desired_frequency = float(state_frequency)
                logger.info(f"BurstEngine: Using state manager frequency: {state_frequency}Hz")
            else:
                # Emergency fallback
                fallback_frequency = 10.0
                self.desired_frequency = fallback_frequency
                self.state_manager.set_burst_frequency(fallback_frequency)
                logger.warning(f"BurstEngine: Invalid state manager frequency ({state_frequency}Hz) - using fallback: {fallback_frequency}Hz")
        except Exception as e:
            # Emergency fallback
            fallback_frequency = 10.0 
            self.desired_frequency = fallback_frequency
            logger.error(f"BurstEngine: Failed to get frequency from state manager ({e}) - using fallback: {fallback_frequency}Hz")
    
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
    
    def _inject_all_candidates(self, fcl: FireCandidateList):
        """Inject all candidates into FCL using FCL Injector."""
        # This method will be called by external injection services
        # For now, it's a placeholder showing the integration point
        
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