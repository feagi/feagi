"""
Fire Queue for FEAGI NPU

Holds neurons that actually fired in the current timestep after membrane potential processing.
This is the clean separation from the Fire Candidate List - only contains confirmed firing neurons.

Key Features:
- Current timestep only (no historical data)
- High-performance access for FQ Sampler
- SoA format compatibility for Rust migration
- Coordinates and neural properties included
- Ready for synaptic propagation to next cycle
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import numpy as np
import time
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


@dataclass
class FiringNeuron:
    """Neuron that fired in current timestep with all relevant properties."""
    neuron_id: int
    cortical_idx: int  
    membrane_potential: float
    coordinates: Tuple[int, int, int]  # (x, y, z)
    threshold: float
    consecutive_fire_count: int = 0
    refractory_counter: int = 0
    timestamp: float = 0.0
    
    def to_dict(self) -> dict:
        """Convert to dictionary for FQ Sampler compatibility."""
        return {
            'neuron_id': self.neuron_id,
            'cortical_idx': self.cortical_idx,
            'membrane_potential': self.membrane_potential,
            'coordinates': self.coordinates,
            'threshold': self.threshold,
            'consecutive_fire_count': self.consecutive_fire_count,
            'refractory_counter': self.refractory_counter,
            'timestamp': self.timestamp
        }


class FireQueue:
    """Current timestep firing neurons - single responsibility for actual firing."""
    
    def __init__(self):
        """Initialize empty fire queue for current timestep."""
        # SoA storage organized by cortical area 
        self.firing_neurons_by_area: Dict[int, List[FiringNeuron]] = {}
        self.current_timestep: int = 0
        self.total_firing_neurons: int = 0
        self.creation_timestamp: float = time.time()
        
    def add_fired_neurons(self, neurons: List[FiringNeuron], timestep: int):
        """Add neurons that actually fired after membrane potential processing.
        
        Args:
            neurons: List of confirmed firing neurons
            timestep: Current timestep
        """
        self.current_timestep = timestep
        
        for neuron in neurons:
            # Organize by cortical area for efficient access
            if neuron.cortical_idx not in self.firing_neurons_by_area:
                self.firing_neurons_by_area[neuron.cortical_idx] = []
                
            # Set firing timestamp
            neuron.timestamp = time.time()
            self.firing_neurons_by_area[neuron.cortical_idx].append(neuron)
            
        self.total_firing_neurons += len(neurons)
        logger.debug(f"Added {len(neurons)} firing neurons to queue for timestep {timestep}")
    
    def add_fired_neurons_soa(self,
                             cortical_idx: int,
                             neuron_ids: np.ndarray,
                             membrane_potentials: np.ndarray,
                             coordinates: np.ndarray,  # Shape: (N, 3)
                             thresholds: np.ndarray,
                             consecutive_fire_counts: np.ndarray = None,
                             refractory_counters: np.ndarray = None,
                             timestep: int = 0):
        """Add firing neurons in SoA format (Rust-friendly).
        
        Args:
            cortical_idx: Cortical area index
            neuron_ids: Array of neuron IDs [i1, i2, ...]
            membrane_potentials: Array of membrane potentials [p1, p2, ...]  
            coordinates: Array of coordinates [[x1,y1,z1], [x2,y2,z2], ...]
            thresholds: Array of firing thresholds [t1, t2, ...]
            consecutive_fire_counts: Optional array of consecutive fire counts
            refractory_counters: Optional array of refractory counters
            timestep: Current timestep
        """
        if len(neuron_ids) == 0:
            return
            
        n_neurons = len(neuron_ids)
        
        # Validate array sizes
        if (len(membrane_potentials) != n_neurons or 
            len(coordinates) != n_neurons or 
            len(thresholds) != n_neurons):
            raise ValueError("All arrays must have same length as neuron_ids")
            
        # Default values for optional arrays
        if consecutive_fire_counts is None:
            consecutive_fire_counts = np.zeros(n_neurons, dtype=np.int32)
        if refractory_counters is None:
            refractory_counters = np.zeros(n_neurons, dtype=np.int32)
            
        # Convert to FiringNeuron objects
        firing_neurons = []
        for i in range(n_neurons):
            neuron = FiringNeuron(
                neuron_id=int(neuron_ids[i]),
                cortical_idx=cortical_idx,
                membrane_potential=float(membrane_potentials[i]),
                coordinates=(int(coordinates[i][0]), int(coordinates[i][1]), int(coordinates[i][2])),
                threshold=float(thresholds[i]),
                consecutive_fire_count=int(consecutive_fire_counts[i]),
                refractory_counter=int(refractory_counters[i]),
                timestamp=time.time()
            )
            firing_neurons.append(neuron)
            
        self.add_fired_neurons(firing_neurons, timestep)
    
    def get_neurons_by_area(self, cortical_idx: int) -> List[FiringNeuron]:
        """Get firing neurons for specific cortical area (FQ Sampler interface)."""
        return self.firing_neurons_by_area.get(cortical_idx, [])
    
    def get_area_fire_queue_dict(self, cortical_idx: int) -> Optional[Dict]:
        """Get fire queue data in dictionary format for FQ Sampler compatibility.
        
        Returns format expected by existing FQ Sampler:
        {
            'neuron_ids': [...],
            'membrane_potentials': [...],
            'coordinates': [...],
            'thresholds': [...],
            'consecutive_fire_counts': [...],
            'refractory_counters': [...],
            'timestamp': float
        }
        """
        neurons = self.firing_neurons_by_area.get(cortical_idx, [])
        if not neurons:
            return None
            
        return {
            'neuron_ids': [n.neuron_id for n in neurons],
            'membrane_potentials': [n.membrane_potential for n in neurons],
            'coordinates': [n.coordinates for n in neurons],
            'thresholds': [n.threshold for n in neurons],
            'consecutive_fire_counts': [n.consecutive_fire_count for n in neurons],
            'refractory_counters': [n.refractory_counter for n in neurons],
            'timestamp': neurons[0].timestamp if neurons else time.time()
        }
    
    def get_area_fire_queue_soa(self, cortical_idx: int) -> Tuple[np.ndarray, ...]:
        """Get fire queue data in SoA format for high-performance processing.
        
        Returns:
            Tuple of (neuron_ids, membrane_potentials, coordinates, thresholds,
                     consecutive_fire_counts, refractory_counters)
        """
        neurons = self.firing_neurons_by_area.get(cortical_idx, [])
        if not neurons:
            return (np.array([]), np.array([]), np.array([]), 
                   np.array([]), np.array([]), np.array([]))
            
        neuron_ids = np.array([n.neuron_id for n in neurons], dtype=np.int32)
        membrane_potentials = np.array([n.membrane_potential for n in neurons], dtype=np.float32)
        coordinates = np.array([n.coordinates for n in neurons], dtype=np.int32)
        thresholds = np.array([n.threshold for n in neurons], dtype=np.float32)
        consecutive_fire_counts = np.array([n.consecutive_fire_count for n in neurons], dtype=np.int32)
        refractory_counters = np.array([n.refractory_counter for n in neurons], dtype=np.int32)
        
        return (neuron_ids, membrane_potentials, coordinates, 
               thresholds, consecutive_fire_counts, refractory_counters)
    
    def get_all_firing_neurons(self) -> List[FiringNeuron]:
        """Get all neurons that fired this timestep."""
        all_neurons = []
        for neurons in self.firing_neurons_by_area.values():
            all_neurons.extend(neurons)
        return all_neurons
    
    def get_all_neuron_ids(self) -> List[int]:
        """Get all firing neuron IDs for synaptic propagation."""
        neuron_ids = []
        for neurons in self.firing_neurons_by_area.values():
            neuron_ids.extend([n.neuron_id for n in neurons])
        return neuron_ids
    
    def get_neurons_for_synaptic_propagation(self) -> Dict[int, List[int]]:
        """Get firing neurons organized by cortical area for synaptic propagation.
        
        Returns:
            Dict mapping cortical_idx -> list of firing neuron IDs
        """
        propagation_data = {}
        for cortical_idx, neurons in self.firing_neurons_by_area.items():
            propagation_data[cortical_idx] = [n.neuron_id for n in neurons]
        return propagation_data
    
    def get_active_areas(self) -> List[int]:
        """Get list of cortical areas with firing neurons."""
        return list(self.firing_neurons_by_area.keys())
    
    def get_statistics(self) -> Dict:
        """Get fire queue statistics for monitoring."""
        area_counts = {area_idx: len(neurons) 
                      for area_idx, neurons in self.firing_neurons_by_area.items()}
        
        return {
            'total_firing_neurons': self.total_firing_neurons,
            'areas_with_firing': len(self.firing_neurons_by_area),
            'neurons_per_area': area_counts,
            'current_timestep': self.current_timestep,
            'age_seconds': time.time() - self.creation_timestamp
        }
    
    def clear(self):
        """Clear fire queue after archival to fire ledger."""
        self.firing_neurons_by_area.clear()
        self.total_firing_neurons = 0
        logger.debug(f"Fire queue cleared after timestep {self.current_timestep}")
    
    def is_empty(self) -> bool:
        """Check if fire queue has any firing neurons."""
        return self.total_firing_neurons == 0
    
    def copy_for_propagation(self):
        """Create a copy for synaptic propagation to next cycle.
        
        Returns a lightweight copy containing only data needed for propagation.
        """
        propagation_copy = FireQueue()
        propagation_copy.firing_neurons_by_area = self.firing_neurons_by_area.copy()
        propagation_copy.current_timestep = self.current_timestep
        propagation_copy.total_firing_neurons = self.total_firing_neurons
        return propagation_copy
