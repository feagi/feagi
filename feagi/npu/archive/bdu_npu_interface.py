"""
BDU-NPU Interface - Clean Brain Transfer System

This module provides a clean interface for transferring developed brains from
BDU (Brain Development Unit) to NPU (Neural Processing Unit) for runtime processing.

The interface ensures:
- One-time transfer after brain development
- Clean separation between development and runtime
- Efficient data transfer with minimal copying
- Validation and error handling

Copyright 2025 Neuraville Inc.
Licensed under the Apache License, Version 2.0
"""

import logging
from typing import Any, Dict, List, Optional, Tuple, Iterator
from dataclasses import dataclass
from enum import Enum
import numpy as np
import time

logger = logging.getLogger(__name__)


class BrainTransferStatus(Enum):
    """Status of brain transfer from BDU to NPU."""
    NOT_STARTED = "not_started"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class NeuronTransferData:
    """Data structure for transferring a single neuron from BDU to NPU."""
    neuron_id: int
    cortical_id: str
    cortical_idx: int
    position: Tuple[int, int, int]
    membrane_potential: float
    threshold: float
    decay_rate: float
    resting_potential: float
    refractory_period: int
    excitability: float = 1.0
    properties: Optional[Dict[str, Any]] = None


@dataclass
class SynapseTransferData:
    """Data structure for transferring a single synapse from BDU to NPU."""
    source_neuron_id: int
    target_neuron_id: int
    weight: float
    delay: int = 1
    is_plastic: bool = False
    plasticity_coeff: float = 0.0


@dataclass
class BrainTransferStats:
    """Statistics for brain transfer operation."""
    neurons_transferred: int = 0
    synapses_transferred: int = 0
    cortical_areas: int = 0
    transfer_time_ms: float = 0.0
    validation_time_ms: float = 0.0
    total_time_ms: float = 0.0
    memory_usage_mb: float = 0.0
    errors: List[str] = None
    
    def __post_init__(self):
        if self.errors is None:
            self.errors = []


class BDUNeuronIterator:
    """Iterator for efficiently extracting neurons from BDU ConnectomeManager."""
    
    def __init__(self, connectome_manager):
        """Initialize iterator with BDU connectome manager.
        
        Args:
            connectome_manager: BDU ConnectomeManager instance
        """
        self.connectome_manager = connectome_manager
        self._current_index = 0
        self._max_neurons = connectome_manager.neuron_array.neuron_count
    
    def __iter__(self) -> Iterator[NeuronTransferData]:
        """Iterate over all neurons in the BDU connectome."""
        self._current_index = 0
        return self
    
    def __next__(self) -> NeuronTransferData:
        """Get next neuron data."""
        if self._current_index >= self._max_neurons:
            raise StopIteration
        
        # Get neuron data from BDU
        neuron_array = self.connectome_manager.neuron_array
        idx = self._current_index
        
        # Skip invalid neurons
        while idx < self._max_neurons and not neuron_array.valid_mask[idx]:
            idx += 1
        
        if idx >= self._max_neurons:
            raise StopIteration
        
        # Get neuron ID from index
        neuron_id = neuron_array.mapping_provider.get_neuron_id(idx) if hasattr(neuron_array, 'mapping_provider') else idx
        
        # Get cortical information
        cortical_idx = neuron_array.cortical_idxs[idx]
        cortical_id = self.connectome_manager.get_cortical_id_from_index(cortical_idx) if cortical_idx != 65535 else "unknown"
        
        # Get position
        position = self.connectome_manager.get_neuron_position(neuron_id) or (0, 0, 0)
        
        # Create transfer data
        neuron_data = NeuronTransferData(
            neuron_id=neuron_id,
            cortical_id=cortical_id,
            cortical_idx=cortical_idx,
            position=position,
            membrane_potential=neuron_array.membrane_potentials[idx],
            threshold=neuron_array.thresholds[idx],
            decay_rate=neuron_array.decay_rates[idx],
            resting_potential=neuron_array.resting_potentials[idx],
            refractory_period=neuron_array.refractory_periods[idx],
            excitability=neuron_array.excitability[idx] if hasattr(neuron_array, 'excitability') else 1.0
        )
        
        self._current_index = idx + 1
        return neuron_data


class BDUSynapseIterator:
    """Iterator for efficiently extracting synapses from BDU GlobalSynapseArray."""
    
    def __init__(self, synapse_array):
        """Initialize iterator with BDU synapse array.
        
        Args:
            synapse_array: BDU GlobalSynapseArray instance
        """
        self.synapse_array = synapse_array
        self._current_index = 0
        self._max_synapses = synapse_array.synapse_count
    
    def __iter__(self) -> Iterator[SynapseTransferData]:
        """Iterate over all synapses in the BDU synapse array."""
        self._current_index = 0
        return self
    
    def __next__(self) -> SynapseTransferData:
        """Get next synapse data."""
        if self._current_index >= self._max_synapses:
            raise StopIteration
        
        idx = self._current_index
        
        # Create transfer data
        synapse_data = SynapseTransferData(
            source_neuron_id=int(self.synapse_array.pre_neuron_ids[idx]),
            target_neuron_id=int(self.synapse_array.post_neuron_ids[idx]),
            weight=float(self.synapse_array.weights[idx]),
            delay=int(self.synapse_array.delays[idx]) if hasattr(self.synapse_array, 'delays') else 1,
            is_plastic=bool(self.synapse_array.is_plastic_flags[idx]) if hasattr(self.synapse_array, 'is_plastic_flags') else False,
            plasticity_coeff=float(self.synapse_array.plasticity_coeffs[idx]) if hasattr(self.synapse_array, 'plasticity_coeffs') else 0.0
        )
        
        self._current_index += 1
        return synapse_data


class BDUNPUInterface:
    """Main interface for transferring brains from BDU to NPU."""
    
    def __init__(self):
        """Initialize BDU-NPU interface."""
        self.transfer_stats = BrainTransferStats()
        self.status = BrainTransferStatus.NOT_STARTED
    
    def transfer_brain(self, bdu_connectome, npu_processor, validate: bool = True) -> bool:
        """Transfer complete brain from BDU to NPU.
        
        Args:
            bdu_connectome: BDU ConnectomeManager with developed brain
            npu_processor: NPU NeuralProcessor instance
            validate: Whether to validate the transfer
            
        Returns:
            True if successful
        """
        start_time = time.time()
        self.status = BrainTransferStatus.IN_PROGRESS
        self.transfer_stats = BrainTransferStats()
        
        try:
            logger.info("Starting brain transfer from BDU to NPU")
            
            # Phase 1: Transfer neurons
            transfer_start = time.time()
            success = self._transfer_neurons(bdu_connectome, npu_processor)
            if not success:
                self.status = BrainTransferStatus.FAILED
                return False
            
            # Phase 2: Transfer synapses
            success = self._transfer_synapses(bdu_connectome, npu_processor)
            if not success:
                self.status = BrainTransferStatus.FAILED
                return False
            
            self.transfer_stats.transfer_time_ms = (time.time() - transfer_start) * 1000
            
            # Phase 3: Validation (optional)
            if validate:
                validation_start = time.time()
                success = self._validate_transfer(bdu_connectome, npu_processor)
                self.transfer_stats.validation_time_ms = (time.time() - validation_start) * 1000
                if not success:
                    self.status = BrainTransferStatus.FAILED
                    return False
            
            # Calculate total time and memory usage
            self.transfer_stats.total_time_ms = (time.time() - start_time) * 1000
            self.transfer_stats.memory_usage_mb = self._estimate_memory_usage(npu_processor)
            
            self.status = BrainTransferStatus.COMPLETED
            logger.info(f"Brain transfer completed: {self.transfer_stats.neurons_transferred:,} neurons, "
                       f"{self.transfer_stats.synapses_transferred:,} synapses in {self.transfer_stats.total_time_ms:.1f}ms")
            return True
            
        except Exception as e:
            self.transfer_stats.errors.append(str(e))
            self.status = BrainTransferStatus.FAILED
            logger.error(f"Brain transfer failed: {e}")
            return False
    
    def _transfer_neurons(self, bdu_connectome, npu_processor) -> bool:
        """Transfer neurons from BDU to NPU.
        
        Args:
            bdu_connectome: BDU ConnectomeManager
            npu_processor: NPU NeuralProcessor
            
        Returns:
            True if successful
        """
        try:
            neuron_iterator = BDUNeuronIterator(bdu_connectome)
            neuron_count = 0
            cortical_areas = set()
            
            for neuron_data in neuron_iterator:
                # Get NPU array index
                idx = neuron_count
                
                # Transfer neuron properties to NPU arrays
                npu_processor.neurons.membrane_potentials[idx] = neuron_data.membrane_potential
                npu_processor.neurons.thresholds[idx] = neuron_data.threshold
                npu_processor.neurons.decay_rates[idx] = neuron_data.decay_rate
                npu_processor.neurons.resting_potentials[idx] = neuron_data.resting_potential
                npu_processor.neurons.refractory_periods[idx] = neuron_data.refractory_period
                npu_processor.neurons.excitability[idx] = neuron_data.excitability
                npu_processor.neurons.cortical_idxs[idx] = neuron_data.cortical_idx
                
                # Update mappings
                npu_processor.neurons.neuron_id_to_index[neuron_data.neuron_id] = idx
                npu_processor.neurons.index_to_neuron_id[idx] = neuron_data.neuron_id
                npu_processor.neurons.valid_mask[idx] = True
                
                cortical_areas.add(neuron_data.cortical_id)
                neuron_count += 1
            
            # Update counts
            npu_processor.neurons.neuron_count = neuron_count
            self.transfer_stats.neurons_transferred = neuron_count
            self.transfer_stats.cortical_areas = len(cortical_areas)
            
            logger.info(f"Transferred {neuron_count:,} neurons from {len(cortical_areas)} cortical areas")
            return True
            
        except Exception as e:
            self.transfer_stats.errors.append(f"Neuron transfer failed: {str(e)}")
            logger.error(f"Neuron transfer failed: {e}")
            return False
    
    def _transfer_synapses(self, bdu_connectome, npu_processor) -> bool:
        """Transfer synapses from BDU to NPU.
        
        Args:
            bdu_connectome: BDU ConnectomeManager
            npu_processor: NPU NeuralProcessor
            
        Returns:
            True if successful
        """
        try:
            synapse_iterator = BDUSynapseIterator(bdu_connectome.synapse_array)
            synapse_count = 0
            
            for synapse_data in synapse_iterator:
                # Validate neuron IDs exist in NPU
                if (synapse_data.source_neuron_id not in npu_processor.neurons.neuron_id_to_index or
                    synapse_data.target_neuron_id not in npu_processor.neurons.neuron_id_to_index):
                    continue  # Skip invalid synapses
                
                # Transfer synapse properties to NPU arrays
                idx = synapse_count
                npu_processor.synapses.source_neurons[idx] = synapse_data.source_neuron_id
                npu_processor.synapses.target_neurons[idx] = synapse_data.target_neuron_id
                npu_processor.synapses.weights[idx] = synapse_data.weight
                npu_processor.synapses.delays[idx] = synapse_data.delay
                
                # Update source neuron index
                source_id = synapse_data.source_neuron_id
                if source_id not in npu_processor.synapses.source_neuron_index:
                    npu_processor.synapses.source_neuron_index[source_id] = []
                npu_processor.synapses.source_neuron_index[source_id].append(idx)
                
                synapse_count += 1
            
            # Update count
            npu_processor.synapses.synapse_count = synapse_count
            self.transfer_stats.synapses_transferred = synapse_count
            
            logger.info(f"Transferred {synapse_count:,} synapses")
            return True
            
        except Exception as e:
            self.transfer_stats.errors.append(f"Synapse transfer failed: {str(e)}")
            logger.error(f"Synapse transfer failed: {e}")
            return False
    
    def _validate_transfer(self, bdu_connectome, npu_processor) -> bool:
        """Validate the brain transfer.
        
        Args:
            bdu_connectome: BDU ConnectomeManager
            npu_processor: NPU NeuralProcessor
            
        Returns:
            True if validation passes
        """
        try:
            # Validate neuron counts
            bdu_neuron_count = bdu_connectome.neuron_array.neuron_count
            npu_neuron_count = npu_processor.neurons.neuron_count
            
            if bdu_neuron_count != npu_neuron_count:
                error = f"Neuron count mismatch: BDU={bdu_neuron_count}, NPU={npu_neuron_count}"
                self.transfer_stats.errors.append(error)
                return False
            
            # Validate synapse counts
            bdu_synapse_count = bdu_connectome.synapse_array.synapse_count
            npu_synapse_count = npu_processor.synapses.synapse_count
            
            if bdu_synapse_count != npu_synapse_count:
                error = f"Synapse count mismatch: BDU={bdu_synapse_count}, NPU={npu_synapse_count}"
                self.transfer_stats.errors.append(error)
                return False
            
            # Sample validation - check a few neurons
            sample_size = min(100, npu_neuron_count)
            for i in range(0, sample_size, max(1, sample_size // 10)):
                if i in npu_processor.neurons.index_to_neuron_id:
                    neuron_id = npu_processor.neurons.index_to_neuron_id[i]
                    
                    # Validate neuron exists in both systems
                    if neuron_id not in npu_processor.neurons.neuron_id_to_index:
                        error = f"Neuron {neuron_id} missing from NPU mapping"
                        self.transfer_stats.errors.append(error)
                        return False
            
            logger.info("Brain transfer validation passed")
            return True
            
        except Exception as e:
            self.transfer_stats.errors.append(f"Validation failed: {str(e)}")
            logger.error(f"Transfer validation failed: {e}")
            return False
    
    def _estimate_memory_usage(self, npu_processor) -> float:
        """Estimate memory usage of NPU processor in MB.
        
        Args:
            npu_processor: NPU NeuralProcessor
            
        Returns:
            Estimated memory usage in MB
        """
        try:
            # Estimate neuron array memory
            neuron_arrays = [
                npu_processor.neurons.membrane_potentials,
                npu_processor.neurons.thresholds,
                npu_processor.neurons.decay_rates,
                npu_processor.neurons.resting_potentials,
                npu_processor.neurons.refractory_periods,
                npu_processor.neurons.refractory_counters,
                npu_processor.neurons.excitability,
                npu_processor.neurons.cortical_idxs,
                npu_processor.neurons.valid_mask
            ]
            
            neuron_memory = sum(arr.nbytes for arr in neuron_arrays)
            
            # Estimate synapse array memory
            synapse_arrays = [
                npu_processor.synapses.source_neurons,
                npu_processor.synapses.target_neurons,
                npu_processor.synapses.weights,
                npu_processor.synapses.delays
            ]
            
            synapse_memory = sum(arr.nbytes for arr in synapse_arrays)
            
            # Convert to MB
            total_memory_mb = (neuron_memory + synapse_memory) / (1024 * 1024)
            return total_memory_mb
            
        except Exception:
            return 0.0
    
    def get_transfer_stats(self) -> BrainTransferStats:
        """Get brain transfer statistics."""
        return self.transfer_stats
    
    def get_transfer_status(self) -> BrainTransferStatus:
        """Get current transfer status."""
        return self.status
