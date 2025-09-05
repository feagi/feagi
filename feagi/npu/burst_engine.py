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
        self.state_manager = state_manager
        
        # Core NPU components
        self.fire_ledger = FireLedgerInterface(fire_ledger_window_size)
        self.coordinate_converter = CoordinateConverter(connectome_manager) if connectome_manager else None
        self.fcl_injector = FCLInjector(self.coordinate_converter) if self.coordinate_converter else None
        
        # FQ Sampler (initialized after burst engine is ready)
        self.fq_sampler: Optional[FQSampler] = None
        
        self.current_timestep = 0
        self.burst_count = 0
        self.previous_fire_queue: Optional[FireQueue] = None

        # Mark as initialized
        self._initialized = True

        logger.info("Clean Burst Engine initialized with singleton pattern")
    
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