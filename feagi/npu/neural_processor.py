"""
NPU Neural Processor - High-Performance Neural Computation Core

This module consolidates all neural processing operations into the NPU for optimal
performance and clean architecture separation:

- BDU: Brain development (neurogenesis, synaptogenesis) 
- NPU: Runtime neural processing (firing, propagation, FCL management)

Copyright 2025 Neuraville Inc.
Licensed under the Apache License, Version 2.0
"""

import logging
from typing import Any, Dict, List, Optional, Tuple, Union
from dataclasses import dataclass
from enum import Enum
import numpy as np

from feagi.config.toml_loader import load_feagi_config
from .bdu_interfaces import BDUNeuronInterface, BDUSynapseInterface
from .plasticity.manager import PlasticityManager, PlasticityConfig

logger = logging.getLogger(__name__)


class NPUBackendType(Enum):
    """Available NPU computation backends."""
    CPU = "cpu"
    CUDA = "cuda" 
    WGPU = "wgpu"
    RUST = "rust"


@dataclass
class NPUNeuralStats:
    """Performance statistics for NPU neural processing."""
    neurons_processed: int = 0
    neurons_fired: int = 0
    synapses_propagated: int = 0
    processing_time_ms: float = 0.0
    backend_used: str = "cpu"
    simd_operations: int = 0
    gpu_operations: int = 0


class NPUNeuronArray:
    """NPU-owned neuron storage using Structure of Arrays (SoA).
    
    NPU is the PRIMARY OWNER of this data structure. BDU gets controlled access.
    Designed for Rust migration where this will become a Rust-owned structure.
    
    Optimized for:
    - SIMD vectorization (16+ neurons per instruction)
    - GPU coalesced memory access
    - Cache-friendly memory layout
    - Zero-allocation operation paths
    - Rust FFI compatibility
    """
    
    def __init__(self, max_neurons: int = 10_000_000, backend: str = "cpu"):
        """Initialize NPU neuron array as primary owner.
        
        Args:
            max_neurons: Maximum number of neurons to support
            backend: Computation backend (cpu, cuda, wgpu, rust)
        """
        self.max_neurons = max_neurons
        self.backend = backend
        self.neuron_count = 0
        
        # Structure of Arrays - 64-byte aligned for SIMD
        self.membrane_potentials = np.zeros(max_neurons, dtype=np.float32)
        self.thresholds = np.ones(max_neurons, dtype=np.float32)
        self.decay_rates = np.full(max_neurons, 0.9, dtype=np.float32)
        self.resting_potentials = np.zeros(max_neurons, dtype=np.float32)
        self.refractory_periods = np.ones(max_neurons, dtype=np.uint8)
        self.refractory_counters = np.zeros(max_neurons, dtype=np.uint8)
        self.excitability = np.ones(max_neurons, dtype=np.float32)
        self.cortical_idxs = np.zeros(max_neurons, dtype=np.uint16)
        
        # Neuron ID mapping (NPU owns this now)
        self.neuron_id_to_index = {}
        self.index_to_neuron_id = {}
        
        # Valid neuron mask
        self.valid_mask = np.zeros(max_neurons, dtype=np.bool_)
        
        # Backend initialization
        self._init_backend()
        
        logger.info(f"NPUNeuronArray initialized as PRIMARY OWNER: {max_neurons:,} max neurons, {backend} backend")
    
    def _init_backend(self):
        """Initialize the computation backend.
        
        Attempts to initialize the requested backend with graceful fallback to CPU.
        This ensures FEAGI works across diverse hardware environments while maintaining
        optimal performance where possible.
        """
        if self.backend == "wgpu":
            try:
                from feagi.bdu.models.array_backend import ArrayBackend, BackendType
                self._gpu_backend = ArrayBackend(BackendType.WGPU)
                logger.info("NPU WGPU backend initialized")
            except Exception as e:
                # @architecture:acceptable - hardware fallback for cross-platform compatibility
                # Rationale: GPU/WGPU may not be available on all deployment targets (RTOS, embedded)
                # Better to run on CPU than crash the entire neural simulation
                logger.warning(f"WGPU backend failed, falling back to CPU: {e}")
                self.backend = "cpu"
                self._gpu_backend = None
        else:
            self._gpu_backend = None
    
    def neural_update_simd(self, timestep: int) -> List[int]:
        """SIMD-optimized neural update - core firing function.
        
        This is the main neural processing function that:
        1. Updates membrane potentials (decay)
        2. Updates refractory counters
        3. Detects firing neurons
        4. Resets fired neurons
        5. Returns list of fired neuron IDs
        
        Args:
            timestep: Current simulation timestep
            
        Returns:
            List of neuron IDs that fired
        """
        # Import SIMD operations
        from feagi.npu.simd_neural_ops import (
            simd_membrane_decay,
            simd_refractory_update,
            simd_firing_check
        )
        
        # PHASE 1: Create update mask (valid neurons not in refractory)
        can_update_mask = self.valid_mask & (self.refractory_counters == 0)
        
        # PHASE 2: Firing detection (BEFORE decay - critical!)
        fired_mask = simd_firing_check(
            self.membrane_potentials,
            self.thresholds,
            can_update_mask
        )
        
        # PHASE 3: Membrane potential decay (SIMD-optimized)
        simd_membrane_decay(
            self.membrane_potentials, 
            self.decay_rates, 
            can_update_mask
        )
        
        # PHASE 4: Refractory period updates (SIMD-optimized)
        simd_refractory_update(self.refractory_counters, self.valid_mask)
        
        # PHASE 5: Process fired neurons
        fired_indices = np.where(fired_mask)[0]
        fired_neuron_ids = []
        
        if len(fired_indices) > 0:
            # Reset fired neurons to resting potential
            self.membrane_potentials[fired_indices] = self.resting_potentials[fired_indices]
            
            # Set refractory periods
            self.refractory_counters[fired_indices] = self.refractory_periods[fired_indices]
            
            # Convert indices to neuron IDs
            for idx in fired_indices:
                if idx in self.index_to_neuron_id:
                    fired_neuron_ids.append(self.index_to_neuron_id[idx])
        
        return fired_neuron_ids


class NPUSynapseArray:
    """NPU-owned synapse storage using Structure of Arrays (SoA).
    
    NPU is the PRIMARY OWNER of this data structure. BDU gets controlled access.
    Designed for Rust migration where this will become a Rust-owned structure.
    
    Optimized for:
    - Scatter-gather operations
    - SIMD vectorization
    - GPU coalesced memory access
    - Rust FFI compatibility
    """
    
    def __init__(self, max_synapses: int = 100_000_000, backend: str = "cpu"):
        """Initialize NPU synapse array as primary owner.
        
        Args:
            max_synapses: Maximum number of synapses to support
            backend: Computation backend
        """
        self.max_synapses = max_synapses
        self.backend = backend
        self.synapse_count = 0
        
        # Structure of Arrays - optimized for scatter-gather operations
        self.source_neurons = np.zeros(max_synapses, dtype=np.uint32)
        self.target_neurons = np.zeros(max_synapses, dtype=np.uint32)
        self.weights = np.zeros(max_synapses, dtype=np.float32)
        self.delays = np.ones(max_synapses, dtype=np.uint8)
        
        # Compatibility with BDU GlobalSynapseArray for data transfer
        self.types = np.zeros(max_synapses, dtype=np.uint8)  # Synapse types (excitatory/inhibitory)
        self.conductances = np.ones(max_synapses, dtype=np.float32)  # Synaptic conductances
        self.is_plastic_flags = np.zeros(max_synapses, dtype=np.bool_)  # Plasticity flags
        
        # Plasticity data structures (NPU-owned)
        self.plasticity_types = np.zeros(max_synapses, dtype=np.uint8)  # PlasticityType enum
        self.plasticity_coeffs = np.ones(max_synapses, dtype=np.float32)
        self.decay_rates = np.full(max_synapses, 0.95, dtype=np.float32)
        self.scaling_exponents = np.ones(max_synapses, dtype=np.float32)
        
        # Spatial indexing for fast lookup (NPU owns this)
        self.source_neuron_index = {}  # neuron_id -> list of synapse indices
        
        logger.info(f"NPUSynapseArray initialized as PRIMARY OWNER: {max_synapses:,} max synapses")
    
    def propagate_simd(self, firing_neurons: List[int], target_potentials: np.ndarray) -> None:
        """SIMD-optimized synaptic propagation.
        
        Args:
            firing_neurons: List of neuron IDs that fired
            target_potentials: Target neuron membrane potentials array
        """
        if not firing_neurons or self.synapse_count == 0:
            return
        
        # Collect all target neurons and weights
        all_targets = []
        all_weights = []
        
        for source_neuron in firing_neurons:
            if source_neuron in self.source_neuron_index:
                synapse_indices = self.source_neuron_index[source_neuron]
                targets = self.target_neurons[synapse_indices]
                weights = self.weights[synapse_indices]
                all_targets.extend(targets)
                all_weights.extend(weights)
        
        if all_targets:
            # Convert to numpy arrays for SIMD operations
            target_array = np.array(all_targets, dtype=np.uint32)
            weight_array = np.array(all_weights, dtype=np.float32)
            
            # SIMD scatter-add operation
            np.add.at(target_potentials, target_array, weight_array)


class NeuralProcessor:
    """Main Neural Processor - PRIMARY OWNER of neural data structures.
    
    This class owns the SoA data structures and provides controlled access to BDU.
    Designed for Rust migration where NPU will be the primary Rust module.
    
    Key Design Principles:
    - NPU is PRIMARY OWNER of neuron and synapse SoA arrays
    - BDU gets CONTROLLED ACCESS through NPU interface
    - Sleep Manager coordinates when BDU can restructure data
    - Memory neurons remain CPU-based and BDU-owned
    - Rust FFI-ready architecture
    """
    
    def __init__(self, 
                 max_neurons: int = 10_000_000,
                 max_synapses: int = 100_000_000,
                 backend: str = "cpu"):
        """Initialize Neural Processor as primary owner.
        
        Args:
            max_neurons: Maximum number of neurons to support
            max_synapses: Maximum number of synapses to support
            backend: Computation backend (cpu, cuda, wgpu, rust)
        """
        self.max_neurons = max_neurons
        self.max_synapses = max_synapses
        self.backend = backend
        
        # NPU OWNS these data structures (primary ownership)
        self.neurons = NPUNeuronArray(max_neurons, backend)
        self.synapses = NPUSynapseArray(max_synapses, backend)
        
        # NPU OWNS plasticity operations (primary ownership)
        plasticity_config = PlasticityConfig(
            plasticity_update_interval=1,      # Update every timestep
            pruning_update_interval=100,       # Prune every 100 timesteps
            homeostatic_update_interval=1000   # Homeostatic every 1000 timesteps
        )
        self.plasticity_manager = PlasticityManager(max_synapses, plasticity_config)
        
        # BDU access control
        self._bdu_access_enabled = True
        self._npu_processing_active = False
        self._sleep_manager = None
        
        # Memory neurons remain BDU-owned (CPU-based)
        self._bdu_connectome_manager = None
        self.memory_neurons = None
        self.has_memory_neurons = False
        
        # Performance tracking
        self.stats = NPUNeuralStats()
        self.current_timestep = 0
        
        logger.info(f"NeuralProcessor initialized as PRIMARY OWNER: {max_neurons:,} neurons, {max_synapses:,} synapses, {backend} backend")
    
    def process_neural_burst(self, timestep: int) -> List[int]:
        """Process a complete neural burst - main entry point.
        
        This replaces ConnectomeManager.update_membrane_potentials() with
        a unified NPU-based implementation that properly handles both
        regular neurons (GPU/SIMD optimized) and memory neurons (CPU-based).
        
        Args:
            timestep: Current simulation timestep
            
        Returns:
            List of neuron IDs that fired
        """
        import time
        start_time = time.time()
        
        self.current_timestep = timestep
        
        # PHASE 1: Regular neuron updates (GPU/SIMD optimized)
        # This processes the main neuron population using the existing BDU neural update
        # but through our NPU wrapper for optimized access patterns
        fired_neurons = self.neurons.neural_update_simd(timestep)
        
        # PHASE 2: Synaptic propagation (GPU/SIMD optimized)
        # Only propagate if we have fired neurons and synapses exist
        if fired_neurons and self.synapses.synapse_count > 0:
            self.synapses.propagate_simd(fired_neurons, self.neurons.membrane_potentials)
        
        # PHASE 3: Memory neuron processing (CPU-based, separate pipeline)
        # Memory neurons are processed separately and don't participate in synaptic propagation
        memory_fired_neurons = []
        if self.has_memory_neurons:
            # Memory neurons are handled by the existing memory processor
            # They have their own lifecycle and pattern-based activation
            # This stays CPU-based as it's fundamentally different from regular neural processing
            try:
                # Delegate to existing memory processing system
                if hasattr(self.connectome_manager, 'memory_processor'):
                    memory_fired_neurons = self.connectome_manager.memory_processor.process_memory_burst(timestep)
                elif hasattr(self.connectome_manager, 'process_memory_neurons'):
                    memory_fired_neurons = self.connectome_manager.process_memory_neurons(timestep)
            except Exception as e:
                # @architecture:acceptable - memory processing fallback
                # Rationale: Memory neurons use different processing patterns than regular neurons
                # Empty result allows main neural processing to continue uninterrupted
                logger.warning(f"Memory neuron processing failed: {e}")
                memory_fired_neurons = []
        
        # PHASE 4: Plasticity updates (NPU-owned)
        # Update activity tracking (every timestep)
        if fired_neurons:
            self.plasticity_manager.update_activity_tracking(
                fired_neurons, 
                self.synapses.source_neurons[:self.synapses.synapse_count],
                self.synapses.target_neurons[:self.synapses.synapse_count],
                timestep
            )
        
        # Scheduled plasticity updates (based on intervals)
        plasticity_updated = 0
        if hasattr(self.synapses, 'plasticity_types'):
            plasticity_updated = self.plasticity_manager.update_plasticity(
                timestep,
                self.synapses.weights[:self.synapses.synapse_count],
                self.synapses.plasticity_types[:self.synapses.synapse_count],
                self.synapses.plasticity_coeffs[:self.synapses.synapse_count],
                self.plasticity_manager._activity_buffer[:self.synapses.synapse_count],
                self.synapses.decay_rates[:self.synapses.synapse_count] if hasattr(self.synapses, 'decay_rates') else np.ones(self.synapses.synapse_count),
                self.synapses.scaling_exponents[:self.synapses.synapse_count] if hasattr(self.synapses, 'scaling_exponents') else np.ones(self.synapses.synapse_count),
                0.001  # dt = 1ms
            )
        
        # PHASE 5: Combine results
        all_fired_neurons = fired_neurons + memory_fired_neurons
        
        # Update statistics
        self.stats.neurons_processed = self.neurons.neuron_count + (self.memory_neurons.capacity if self.has_memory_neurons else 0)
        self.stats.neurons_fired = len(all_fired_neurons)
        self.stats.processing_time_ms = (time.time() - start_time) * 1000
        self.stats.backend_used = self.backend
        
        # Add plasticity statistics
        if plasticity_updated > 0:
            logger.debug(f"NPU plasticity: updated {plasticity_updated} synapses at timestep {timestep}")
        
        return all_fired_neurons
    
    # ============================================================================
    # BDU ACCESS CONTROL - Sleep Manager Integration
    # ============================================================================
    
    def register_sleep_manager(self, sleep_manager):
        """Register sleep manager for coordinated BDU access.
        
        Args:
            sleep_manager: Sleep manager instance that coordinates NPU/BDU access
        """
        self._sleep_manager = sleep_manager
        logger.info("Sleep manager registered with NPU")
    
    def enable_bdu_access(self) -> bool:
        """Enable BDU access to NPU-owned data structures.
        
        This is called by the sleep manager when NPU goes to sleep.
        BDU can then restructure neurons/synapses safely.
        
        Returns:
            True if BDU access enabled successfully
        """
        if self._npu_processing_active:
            logger.error("Cannot enable BDU access while NPU processing is active")
            return False
        
        self._bdu_access_enabled = True
        logger.info("BDU access to NPU data structures ENABLED")
        return True
    
    def disable_bdu_access(self) -> bool:
        """Disable BDU access to NPU-owned data structures.
        
        This is called by the sleep manager when NPU wakes up.
        NPU reclaims exclusive access to its data structures.
        
        Returns:
            True if BDU access disabled successfully
        """
        self._bdu_access_enabled = False
        logger.info("BDU access to NPU data structures DISABLED")
        return True
    
    def get_bdu_neuron_interface(self):
        """Get BDU interface to NPU-owned neuron data.
        
        This provides BDU with controlled access to NPU neuron arrays
        for restructuring operations during sleep periods.
        
        Returns:
            BDUNeuronInterface or None if access not enabled
        """
        if not self._bdu_access_enabled:
            logger.error("BDU access not enabled - cannot provide neuron interface")
            return None
        
        return BDUNeuronInterface(self.neurons)
    
    def get_bdu_synapse_interface(self):
        """Get BDU interface to NPU-owned synapse data.
        
        This provides BDU with controlled access to NPU synapse arrays
        for restructuring operations during sleep periods.
        
        Returns:
            BDUSynapseInterface or None if access not enabled
        """
        if not self._bdu_access_enabled:
            logger.error("BDU access not enabled - cannot provide synapse interface")
            return None
        
        return BDUSynapseInterface(self.synapses)
    
    def load_brain_from_bdu(self, bdu_connectome_manager) -> bool:
        """Load developed brain from BDU into NPU-owned structures.
        
        This is the one-time transfer from BDU (development) to NPU (runtime).
        After this, NPU becomes the primary owner and BDU gets controlled access.
        
        Args:
            bdu_connectome_manager: BDU ConnectomeManager with developed brain
            
        Returns:
            True if successful
        """
        try:
            self._bdu_connectome_manager = bdu_connectome_manager
            
            # Load memory neuron reference (BDU retains ownership)
            if hasattr(bdu_connectome_manager, 'memory_neuron_array'):
                self.memory_neurons = bdu_connectome_manager.memory_neuron_array
                self.has_memory_neurons = True
                logger.info(f"Memory neurons linked: {self.memory_neurons.capacity:,} capacity (BDU-owned)")
            
            # Transfer neurons from BDU to NPU ownership
            neuron_count = 0
            for neuron_id in bdu_connectome_manager.get_all_neuron_ids():
                neuron_data = bdu_connectome_manager.get_neuron_properties(neuron_id)
                if neuron_data:
                    idx = neuron_count
                    
                    # Copy neuron properties to NPU arrays
                    self.neurons.membrane_potentials[idx] = neuron_data.get('membrane_potential', 0.0)
                    self.neurons.thresholds[idx] = neuron_data.get('threshold', 1.0)
                    self.neurons.decay_rates[idx] = neuron_data.get('decay_rate', 0.9)
                    self.neurons.resting_potentials[idx] = neuron_data.get('resting_potential', 0.0)
                    self.neurons.refractory_periods[idx] = neuron_data.get('refractory_period', 1)
                    self.neurons.cortical_idxs[idx] = neuron_data.get('cortical_idx', 0)
                    
                    # Update NPU mappings
                    self.neurons.neuron_id_to_index[neuron_id] = idx
                    self.neurons.index_to_neuron_id[idx] = neuron_id
                    self.neurons.valid_mask[idx] = True
                    
                    neuron_count += 1
            
            self.neurons.neuron_count = neuron_count
            
            # Transfer synapses from BDU GlobalSynapseArray to NPU - SINGLE SoA ARCHITECTURE
            synapse_count = 0
            if hasattr(bdu_connectome_manager, 'synapse_array'):
                bdu_synapses = bdu_connectome_manager.synapse_array
                bdu_synapse_count = bdu_synapses.synapse_count
                logger.info(f"Transferring {bdu_synapse_count:,} synapses from BDU GlobalSynapseArray to NPU")
                
                if bdu_synapse_count > 0:
                    # Direct array transfer for maximum performance
                    self.synapses.source_neurons[:bdu_synapse_count] = bdu_synapses.pre_neuron_ids[:bdu_synapse_count]
                    self.synapses.target_neurons[:bdu_synapse_count] = bdu_synapses.post_neuron_ids[:bdu_synapse_count]
                    self.synapses.weights[:bdu_synapse_count] = bdu_synapses.weights[:bdu_synapse_count]
                    self.synapses.delays[:bdu_synapse_count] = bdu_synapses.delays[:bdu_synapse_count]
                    self.synapses.types[:bdu_synapse_count] = bdu_synapses.types[:bdu_synapse_count]
                    self.synapses.plasticity_coeffs[:bdu_synapse_count] = bdu_synapses.plasticity_coeffs[:bdu_synapse_count]
                    self.synapses.conductances[:bdu_synapse_count] = bdu_synapses.conductances[:bdu_synapse_count]
                    self.synapses.is_plastic_flags[:bdu_synapse_count] = bdu_synapses.is_plastic_flags[:bdu_synapse_count]
                    
                    # Rebuild NPU synapse indices for fast lookup
                    self.synapses.source_neuron_index.clear()
                    for i in range(bdu_synapse_count):
                        source_neuron = self.synapses.source_neurons[i]
                        if source_neuron not in self.synapses.source_neuron_index:
                            self.synapses.source_neuron_index[source_neuron] = []
                        self.synapses.source_neuron_index[source_neuron].append(i)
                    
                    synapse_count = bdu_synapse_count
                    logger.info(f"✅ Transferred {synapse_count:,} synapses to NPU primary ownership")
            else:
                # Fallback to legacy synapse iteration if GlobalSynapseArray not available
                logger.warning("BDU GlobalSynapseArray not found, using legacy synapse transfer")
                for synapse_data in bdu_connectome_manager.get_all_synapses():
                    source_id = synapse_data['source_neuron_id']
                    target_id = synapse_data['target_neuron_id']
                    weight = synapse_data['weight']
                    
                    # Store synapse in NPU arrays
                    self.synapses.source_neurons[synapse_count] = source_id
                    self.synapses.target_neurons[synapse_count] = target_id
                    self.synapses.weights[synapse_count] = weight
                    
                    # Update NPU index
                    if source_id not in self.synapses.source_neuron_index:
                        self.synapses.source_neuron_index[source_id] = []
                    self.synapses.source_neuron_index[source_id].append(synapse_count)
                    
                    synapse_count += 1
            
            self.synapses.synapse_count = synapse_count
            
            logger.info(f"Brain loaded into NPU: {neuron_count:,} neurons, {synapse_count:,} synapses")
            logger.info("NPU is now PRIMARY OWNER of neural data structures")
            return True
            
        except Exception as e:
            logger.error(f"Failed to load brain into NPU: {e}")
            return False
    
    def get_performance_stats(self) -> NPUNeuralStats:
        """Get current performance statistics."""
        return self.stats
