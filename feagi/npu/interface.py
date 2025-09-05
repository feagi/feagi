"""
NPU Interface - Clean Architecture

Simplified interface for NPU data structures that maintains compatibility
while supporting the clean FCL/Fire Queue/Fire Ledger architecture.
"""

from typing import Dict, List, Tuple, Optional, Any, Union
from enum import Enum
from dataclasses import dataclass

from feagi.npu.data_structures import NeuronArray, MemoryNeuronArray, SynapseArray, BackendType
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class OperationResult(Enum):
    """Result codes for NPU operations."""
    SUCCESS = "success"
    AREA_LOCKED = "area_locked"
    CAPACITY_EXCEEDED = "capacity_exceeded" 
    INVALID_INPUT = "invalid_input"
    BACKEND_ERROR = "backend_error"


@dataclass
class BatchOperationResult:
    """Result of a batch operation."""
    result: OperationResult
    successful_count: int
    failed_indices: List[int]
    error_message: Optional[str] = None


class NPUInterface:
    """Simplified NPU interface for clean architecture."""
    
    def __init__(self, max_neurons: int = 100_000, max_synapses: int = 500_000, 
                 max_memory_neurons: int = 50_000, backend: BackendType = BackendType.CPU):
        """Initialize NPU interface with data structures."""
        
        # Initialize data structures
        self.neuron_array = NeuronArray(max_neurons=max_neurons, backend=backend)
        self.memory_neuron_array = MemoryNeuronArray(max_memory_neurons=max_memory_neurons, backend=backend)
        self.synapse_array = SynapseArray(max_synapses=max_synapses, backend=backend)
        
        # State tracking
        self.backend = backend
        self.max_neurons = max_neurons
        self.max_synapses = max_synapses
        self.max_memory_neurons = max_memory_neurons
        
        logger.info(f"NPU Interface initialized: {max_neurons} neurons, {max_synapses} synapses, {backend.value} backend")
    
    def add_neurons_batch(self, neuron_ids: List[int], positions: List[Tuple[int, int, int]],
                         neuron_types: List[int], initial_potentials: List[float],
                         thresholds: List[float], leak_coefficients: List[float],
                         cortical_idx: int) -> BatchOperationResult:
        """Add multiple neurons in batch."""
        try:
            indices = self.neuron_array.add_neurons_batch(
                neuron_ids=neuron_ids,
                positions=positions,
                neuron_types=neuron_types,
                initial_potentials=initial_potentials,
                thresholds=thresholds,
                leak_coefficients=leak_coefficients,
                cortical_idx=cortical_idx
            )
            
            return BatchOperationResult(
                result=OperationResult.SUCCESS,
                successful_count=len(indices),
                failed_indices=[]
            )
            
        except ValueError as e:
            return BatchOperationResult(
                result=OperationResult.CAPACITY_EXCEEDED,
                successful_count=0,
                failed_indices=list(range(len(neuron_ids))),
                error_message=str(e)
            )
        except Exception as e:
            return BatchOperationResult(
                result=OperationResult.BACKEND_ERROR,
                successful_count=0,
                failed_indices=list(range(len(neuron_ids))),
                error_message=str(e)
            )
    
    def get_neuron_property(self, neuron_id: int, property_name: str) -> Any:
        """Get a neuron property value."""
        return self.neuron_array.get_property(neuron_id, property_name)
    
    def set_neuron_property(self, neuron_id: int, property_name: str, value: Union[float, int]):
        """Set a neuron property value."""
        self.neuron_array.set_neuron_property(neuron_id, property_name, value)
    
    def get_firing_neurons_by_cortical_area(self, neuron_ids: List[int]) -> Dict[int, List[int]]:
        """Get firing neurons organized by cortical area."""
        # This is a placeholder - in a real implementation, this would
        # organize neurons by their cortical area indices
        result = {}
        for neuron_id in neuron_ids:
            # Get cortical area for this neuron
            if neuron_id in self.neuron_array.neuron_id_to_index:
                idx = self.neuron_array.neuron_id_to_index[neuron_id]
                cortical_idx = int(self.neuron_array.cortical_idxs[idx])
                
                if cortical_idx not in result:
                    result[cortical_idx] = []
                result[cortical_idx].append(neuron_id)
        
        return result
    
    def process_neural_burst(self, timestep: int) -> List[int]:
        """Process neural burst and return firing neuron IDs."""
        # This is a placeholder for actual neural processing
        # In the clean architecture, this would integrate with
        # the Fire Queue and neural dynamics processing
        return []
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get NPU statistics."""
        return {
            'neuron_count': self.neuron_array.count,
            'synapse_count': self.synapse_array.count,
            'memory_neuron_count': self.memory_neuron_array.count,
            'backend': self.backend.value,
            'capacity_utilization': {
                'neurons': self.neuron_array.count / self.max_neurons,
                'synapses': self.synapse_array.count / self.max_synapses,
                'memory_neurons': self.memory_neuron_array.count / self.max_memory_neurons
            }
        }
