"""
Burst Engine NPU Integration

This module provides the integration layer between the BurstEngine and the new
NeuralProcessor, replacing the BDU-based neural processing with unified NPU processing.

Copyright 2025 Neuraville Inc.
Licensed under the Apache License, Version 2.0
"""

import logging
from typing import Any, Dict, List, Optional
import time

from feagi.npu.neural_processor import NeuralProcessor
from feagi.core.state_manager import get_state_manager

logger = logging.getLogger(__name__)


class BurstEngineNPUMixin:
    """Mixin to integrate NPU neural processing into BurstEngine.
    
    This mixin replaces the BDU-based neural processing calls with
    unified NPU processing for optimal performance.
    """
    
    def __init__(self, *args, **kwargs):
        """Initialize NPU integration."""
        super().__init__(*args, **kwargs)
        self.npu_processor: Optional[NeuralProcessor] = None
        self._npu_initialized = False
        self._use_npu_processing = False
    
    def initialize_npu_processor(self, 
                                max_neurons: int = 10_000_000,
                                max_synapses: int = 100_000_000,
                                backend: str = "cpu") -> bool:
        """Initialize the NPU neural processor as primary owner.
        
        Args:
            max_neurons: Maximum number of neurons
            max_synapses: Maximum number of synapses
            backend: Computation backend (cpu, cuda, wgpu, rust)
            
        Returns:
            True if successful
        """
        try:
            self.npu_processor = NeuralProcessor(
                max_neurons=max_neurons,
                max_synapses=max_synapses,
                backend=backend
            )
            self._npu_initialized = True
            
            logger.info(f"NPU processor initialized as PRIMARY OWNER: {backend} backend")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize NPU processor: {e}")
            return False
    
    def enable_npu_processing(self) -> bool:
        """Enable NPU-based neural processing.
        
        This loads the brain from BDU into NPU and switches to NPU processing.
        
        Returns:
            True if successful
        """
        if not self._npu_initialized:
            logger.error("NPU processor not initialized")
            return False
        
        if not hasattr(self, 'connectome_manager'):
            logger.error("ConnectomeManager not available")
            return False
        
        try:
            # Load brain from BDU into NPU (one-time transfer)
            success = self.npu_processor.load_brain_from_bdu(self.connectome_manager)
            if success:
                self._use_npu_processing = True
                logger.info("NPU processing enabled - brain loaded from BDU, NPU is now primary owner")
                return True
            else:
                logger.error("Failed to load brain into NPU")
                return False
        except Exception as e:
            logger.error(f"Failed to enable NPU processing: {e}")
            return False
    
    def _process_burst_with_npu(self, current_timestep: int) -> List[int]:
        """Process burst using NPU neural processor.
        
        This replaces the BDU-based _process_burst_with_power_injection method
        with unified NPU processing.
        
        Args:
            current_timestep: Current simulation timestep
            
        Returns:
            List of fired neuron IDs
        """
        state_manager = get_state_manager()
        
        if state_manager.is_debug_npu_enabled():
            logger.info(f"[NPU-DEBUG] BURST ENGINE: Starting NPU burst processing for timestep {current_timestep}")
        
        # 1. Pre-burst injections (power areas, etc.)
        if hasattr(self, 'injection_service') and self.injection_service:
            if state_manager.is_debug_npu_enabled():
                logger.info("[NPU-DEBUG] BURST ENGINE: Pre-burst injections")
            self.injection_service.inject_pre_burst(current_timestep)
        
        # 2. CORE NPU NEURAL PROCESSING - replaces BDU processing
        if state_manager.is_debug_npu_enabled():
            logger.info("[NPU-DEBUG] BURST ENGINE: NPU neural processing")
        
        start_time = time.time()
        fired_neurons = self.npu_processor.process_neural_burst(current_timestep)
        processing_time = (time.time() - start_time) * 1000
        
        if state_manager.is_debug_npu_enabled():
            fired_count = len(fired_neurons) if fired_neurons else 0
            logger.info(f"[NPU-DEBUG] BURST ENGINE: NPU processing complete - {fired_count} neurons fired in {processing_time:.2f}ms")
        
        # 3. During-burst injections
        if hasattr(self, 'injection_service') and self.injection_service:
            if state_manager.is_debug_npu_enabled():
                logger.info("[NPU-DEBUG] BURST ENGINE: During-burst injections")
            self.injection_service.inject_during_burst(current_timestep)
        
        # 4. Post-burst injections
        if hasattr(self, 'injection_service') and self.injection_service:
            if state_manager.is_debug_npu_enabled():
                logger.info("[NPU-DEBUG] BURST ENGINE: Post-burst injections")
            self.injection_service.inject_post_burst(current_timestep)
        
        # 5. Memory processing (if applicable)
        if hasattr(self, 'memory_processor') and self.memory_processor:
            if state_manager.is_debug_npu_enabled():
                logger.info("[NPU-DEBUG] BURST ENGINE: Memory processing")
            # Memory processing logic here
        
        # 6. Update FCL with fired neurons
        if fired_neurons and hasattr(self, 'fcl_manager') and self.fcl_manager:
            # Group neurons by cortical area for FCL update
            neurons_by_cortical = {}
            for neuron_id in fired_neurons:
                # Get cortical index from NPU processor
                if neuron_id in self.npu_processor.neurons.neuron_id_to_index:
                    idx = self.npu_processor.neurons.neuron_id_to_index[neuron_id]
                    cortical_idx = self.npu_processor.neurons.cortical_idxs[idx]
                    
                    if cortical_idx not in neurons_by_cortical:
                        neurons_by_cortical[cortical_idx] = []
                    neurons_by_cortical[cortical_idx].append(neuron_id)
            
            # Update FCL
            if neurons_by_cortical:
                self.fcl_manager.update_fcl(current_timestep, neurons_by_cortical)
        
        return fired_neurons
    
    def get_npu_performance_stats(self) -> Dict[str, Any]:
        """Get NPU performance statistics.
        
        Returns:
            Dictionary with NPU performance metrics
        """
        if not self.npu_processor:
            return {"error": "NPU processor not initialized"}
        
        stats = self.npu_processor.get_performance_stats()
        return {
            "neurons_processed": stats.neurons_processed,
            "neurons_fired": stats.neurons_fired,
            "synapses_propagated": stats.synapses_propagated,
            "processing_time_ms": stats.processing_time_ms,
            "backend_used": stats.backend_used,
            "simd_operations": stats.simd_operations,
            "gpu_operations": stats.gpu_operations
        }


# Monkey patch for backward compatibility during migration
def patch_burst_engine_for_npu():
    """Monkey patch BurstEngine to use NPU processing.
    
    This allows gradual migration without breaking existing code.
    """
    from feagi.npu.burst_engine import BurstEngine
    
    # Store original method
    if not hasattr(BurstEngine, '_original_process_burst_with_power_injection'):
        BurstEngine._original_process_burst_with_power_injection = BurstEngine._process_burst_with_power_injection
    
    def _process_burst_with_npu_fallback(self, current_timestep: int) -> List[int]:
        """NPU processing with fallback to BDU if NPU not available."""
        if hasattr(self, '_use_npu_processing') and self._use_npu_processing and self.npu_processor:
            # Use NPU processing
            return self._process_burst_with_npu(current_timestep)
        else:
            # Fallback to original BDU processing
            return self._original_process_burst_with_power_injection(current_timestep)
    
    # Apply the patch
    BurstEngine._process_burst_with_power_injection = _process_burst_with_npu_fallback
    
    # Add NPU mixin methods
    for method_name in dir(BurstEngineNPUMixin):
        if not method_name.startswith('_') or method_name in ['__init__']:
            method = getattr(BurstEngineNPUMixin, method_name)
            if callable(method):
                setattr(BurstEngine, method_name, method)
    
    logger.info("BurstEngine patched for NPU processing")


# Configuration helper
def configure_npu_burst_engine(burst_engine, 
                              max_neurons: int = 10_000_000,
                              max_synapses: int = 100_000_000,
                              backend: str = "cpu",
                              enable_immediately: bool = True) -> bool:
    """Configure BurstEngine to use NPU processing.
    
    Args:
        burst_engine: BurstEngine instance
        max_neurons: Maximum number of neurons
        max_synapses: Maximum number of synapses
        backend: Computation backend
        enable_immediately: Whether to enable NPU processing immediately
        
    Returns:
        True if successful
    """
    try:
        # Apply NPU patch if not already applied
        if not hasattr(burst_engine, 'initialize_npu_processor'):
            patch_burst_engine_for_npu()
        
        # Initialize NPU processor as primary owner
        success = burst_engine.initialize_npu_processor(
            max_neurons=max_neurons,
            max_synapses=max_synapses,
            backend=backend
        )
        
        if not success:
            return False
        
        # Enable NPU processing if requested
        if enable_immediately:
            success = burst_engine.enable_npu_processing()
            if success:
                logger.info("NPU burst engine configuration complete - NPU is primary owner")
            return success
        
        logger.info("NPU burst engine initialized (not enabled)")
        return True
        
    except Exception as e:
        logger.error(f"Failed to configure NPU burst engine: {e}")
        return False
