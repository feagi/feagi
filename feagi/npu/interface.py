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
    data: Optional[Dict[str, Any]] = None
    
    @property
    def is_success(self) -> bool:
        """Check if the operation was fully successful."""
        return self.result == OperationResult.SUCCESS
    
    @property
    def has_partial_success(self) -> bool:
        """Check if the operation had partial success."""
        return self.successful_count > 0 and len(self.failed_indices) > 0


@dataclass
class SynapseCreationRequest:
    """Request for creating synapses."""
    source_neuron_ids: List[int]
    target_neuron_ids: List[int]
    weights: List[float]
    delays: Optional[List[int]] = None
    plasticity_types: Optional[List[int]] = None
    plasticity_coefficients: Optional[List[float]] = None


@dataclass
class NeuronCreationRequest:
    """Request for batch neuron creation."""
    cortical_idx: int
    positions: List[Tuple[int, int, int]]
    neuron_types: Optional[List[int]] = None
    initial_potentials: Optional[List[float]] = None
    thresholds: Optional[List[float]] = None
    leak_coefficients: Optional[List[float]] = None
    excitabilities: Optional[List[float]] = None


@dataclass
class NeuronUpdateRequest:
    """Request for updating neuron properties."""
    neuron_ids: List[int]
    property_name: str
    values: List[Union[float, int]]


class NPUInterface:
    """Simplified NPU interface for clean architecture."""
    
    def __init__(self, backend: Optional[BackendType] = None, max_neurons: int = 100_000, 
                 max_synapses: int = 500_000, max_memory_neurons: int = 50_000):
        """Initialize NPU interface with data structures."""
        
        # Use CPU backend as default if not specified
        if backend is None:
            backend = BackendType.CPU
        
        # Initialize data structures
        self.neuron_array = NeuronArray(max_neurons=max_neurons, backend=backend)
        self.memory_neuron_array = MemoryNeuronArray(max_memory_neurons=max_memory_neurons, backend=backend)
        self.synapse_array = SynapseArray(max_synapses=max_synapses, backend=backend)
        
        # State tracking
        self.backend = backend
        self.max_neurons = max_neurons
        self.max_synapses = max_synapses
        self.max_memory_neurons = max_memory_neurons
        
        # Cortical area management for compatibility
        self.cortical_areas: Dict[int, Dict[str, Any]] = {}  # cortical_idx -> area_info
        self.neuron_to_area: Dict[int, int] = {}  # neuron_id -> cortical_idx
        
        logger.info("NPU Interface initialized: %d neurons, %d synapses, %s backend", max_neurons, max_synapses, backend.value)
    
    def create_neurons_batch(self, request: NeuronCreationRequest) -> BatchOperationResult:
        """Create neurons from a NeuronCreationRequest."""
        try:
            # Generate neuron IDs
            num_neurons = len(request.positions)
            neuron_ids = list(range(self.neuron_array.count + 1, self.neuron_array.count + num_neurons + 1))
            
            # Use defaults if not provided
            neuron_types = request.neuron_types or [0] * num_neurons
            initial_potentials = request.initial_potentials or [0.0] * num_neurons
            thresholds = request.thresholds or [1.0] * num_neurons
            leak_coefficients = request.leak_coefficients or [0.1] * num_neurons
            
            indices = self.neuron_array.add_neurons_batch(
                neuron_ids=neuron_ids,
                positions=request.positions,
                neuron_types=neuron_types,
                initial_potentials=initial_potentials,
                thresholds=thresholds,
                leak_coefficients=leak_coefficients,
                cortical_idx=request.cortical_idx
            )
            
            # Update neuron to area mapping
            for neuron_id in neuron_ids:
                self.neuron_to_area[neuron_id] = request.cortical_idx
            
            # Update cortical area neuron count
            if request.cortical_idx in self.cortical_areas:
                self.cortical_areas[request.cortical_idx]["neuron_count"] += len(neuron_ids)
            
            return BatchOperationResult(
                result=OperationResult.SUCCESS,
                successful_count=len(indices),
                failed_indices=[]
            )
            
        except ValueError as e:
            return BatchOperationResult(
                result=OperationResult.CAPACITY_EXCEEDED,
                successful_count=0,
                failed_indices=list(range(len(request.positions))),
                error_message=str(e)
            )
        except Exception as e:
            return BatchOperationResult(
                result=OperationResult.BACKEND_ERROR,
                successful_count=0,
                failed_indices=list(range(len(request.positions))),
                error_message=str(e)
            )

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
            
            # Update neuron to area mapping
            for neuron_id in neuron_ids:
                self.neuron_to_area[neuron_id] = cortical_idx
            
            # Update cortical area neuron count
            if cortical_idx in self.cortical_areas:
                self.cortical_areas[cortical_idx]["neuron_count"] += len(neuron_ids)
            
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
    
    def update_neurons_batch(self, request: NeuronUpdateRequest) -> BatchOperationResult:
        """Update neuron properties from a NeuronUpdateRequest."""
        try:
            updated_count = 0
            for neuron_id, value in zip(request.neuron_ids, request.values):
                if neuron_id in self.neuron_array.neuron_id_to_index:
                    self.set_neuron_property(neuron_id, request.property_name, value)
                    updated_count += 1
            
            return BatchOperationResult(
                result=OperationResult.SUCCESS,
                successful_count=updated_count,
                failed_indices=[]
            )
            
        except Exception as e:
            return BatchOperationResult(
                result=OperationResult.BACKEND_ERROR,
                successful_count=0,
                failed_indices=list(range(len(request.neuron_ids))),
                error_message=str(e)
            )
    
    def create_synapses_batch(self, request: SynapseCreationRequest) -> BatchOperationResult:
        """Create synapses from a SynapseCreationRequest."""
        try:
            # Use defaults if not provided
            delays = request.delays or [1] * len(request.source_neuron_ids)
            plasticity_types = request.plasticity_types or [0] * len(request.source_neuron_ids)
            plasticity_coefficients = request.plasticity_coefficients or [0.0] * len(request.source_neuron_ids)
            
            count = self.synapse_array.add_synapses_batch(
                source_neuron_ids=request.source_neuron_ids,
                target_neuron_ids=request.target_neuron_ids,
                weights=request.weights,
                delays=delays,
                plasticity_types=plasticity_types,
                plasticity_coefficients=plasticity_coefficients
            )
            
            return BatchOperationResult(
                result=OperationResult.SUCCESS,
                successful_count=count,
                failed_indices=[]
            )
            
        except ValueError as e:
            return BatchOperationResult(
                result=OperationResult.CAPACITY_EXCEEDED,
                successful_count=0,
                failed_indices=list(range(len(request.source_neuron_ids))),
                error_message=str(e)
            )
        except Exception as e:
            return BatchOperationResult(
                result=OperationResult.BACKEND_ERROR,
                successful_count=0,
                failed_indices=list(range(len(request.source_neuron_ids))),
                error_message=str(e)
            )
    
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
    
    # === Cortical Area Management (Compatibility Methods) ===
    
    def get_cortical_idx_by_id(self, cortical_id: str) -> Optional[int]:
        """Get cortical_idx by cortical_id string.
        
        Args:
            cortical_id: String identifier (e.g., "_power")
            
        Returns:
            cortical_idx if found, None otherwise
        """
        for cortical_idx, area_info in self.cortical_areas.items():
            if area_info.get("cortical_id") == cortical_id:
                return cortical_idx
        return None
    
    def create_cortical_area(self, cortical_idx: int, dimensions: Tuple[int, int, int], 
                           area_type: str = "regular", cortical_id: str = None) -> OperationResult:
        """Create a new cortical area.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            dimensions: (width, height, depth) dimensions
            area_type: Type of area ("regular" or "memory")
            cortical_id: String identifier for the area
            
        Returns:
            OperationResult indicating success or failure
        """
        if cortical_idx in self.cortical_areas:
            return OperationResult.INVALID_INPUT
            
        self.cortical_areas[cortical_idx] = {
            "cortical_id": cortical_id,
            "dimensions": dimensions,
            "type": area_type,
            "neuron_count": 0,
            "created": True
        }
        
        # Cortical area created
        return OperationResult.SUCCESS
    
    def get_neurons_by_area(self, cortical_idx: int) -> List[int]:
        """Get all neuron IDs in a cortical area."""
        neuron_ids = []
        for neuron_id, area_idx in self.neuron_to_area.items():
            if area_idx == cortical_idx:
                neuron_ids.append(neuron_id)
        return neuron_ids
