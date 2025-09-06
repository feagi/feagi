"""
FCL Injector for FEAGI NPU

Clean implementation for injecting candidates into Fire Candidate List.
Handles external stimuli injection from:
- Sensory streams (SoA format conversion)
- Power areas (constant brain activity)
- Manual stimulation (direct neuron activation)

Key Features:
- Single responsibility: only injects into FCL
- SoA format support: [x,y,z,p] → [i,p] conversion
- High-performance batch operations
- No timing conflicts or historical data mixing
"""

from typing import Dict, List, Optional, Any, Tuple
import numpy as np
from feagi.utils.logger import setup_logger

from .fire_candidate_list import FireCandidateList, FCLCandidate
from .coordinate_converter import CoordinateConverter

logger = setup_logger(__name__)


class FCLInjector:
    """Clean FCL Injector - adds candidates to Fire Candidate List only."""
    
    def __init__(self, coordinate_converter: CoordinateConverter):
        """Initialize FCL Injector.
        
        Args:
            coordinate_converter: For SoA format conversion
        """
        self.coordinate_converter = coordinate_converter
        self.injection_count = 0
        
        logger.info("FCL Injector initialized")
    
    def inject_sensory_data(self, 
                          fcl: FireCandidateList,
                          cortical_id: str,
                          x_coords: np.ndarray,     # [x1, x2, ...]
                          y_coords: np.ndarray,     # [y1, y2, ...]
                          z_coords: np.ndarray,     # [z1, z2, ...]
                          potentials: np.ndarray    # [p1, p2, ...]
                          ) -> int:
        """Inject sensory stream data in SoA format.
        
        Converts [x,y,z,p] voxel format to [i,p] neuron format and adds to FCL.
        
        Returns:
            Number of candidates actually injected
        """
        if len(x_coords) == 0:
            return 0
            
        # Convert SoA voxel coordinates to neuron IDs
        neuron_ids, valid_potentials, cortical_idx = self.coordinate_converter.convert_soa_to_neuron_ids(
            cortical_id, x_coords, y_coords, z_coords, potentials
        )
        
        if len(neuron_ids) == 0:
            # No valid neurons found for injection
            return 0
        
        # All sensory inputs are excitatory by default
        excitatory_mask = np.ones(len(neuron_ids), dtype=bool)
        
        # Inject into FCL
        added_count = fcl.add_candidates_soa(
            cortical_idx=cortical_idx,
            neuron_ids=neuron_ids,
            potential_deltas=valid_potentials,
            excitatory_mask=excitatory_mask
        )
        
        self.injection_count += added_count
        # Sensory candidates added to FCL
        return added_count
    
    def inject_power_area(self,
                         fcl: FireCandidateList,
                         cortical_idx: int,
                         neuron_ids: List[int],
                         base_potential: float = 1.5) -> int:
        """Inject power area neurons for constant brain activity.
        
        Power areas provide foundational activity every burst cycle.
        
        Args:
            fcl: Fire Candidate List to inject into
            cortical_idx: Cortical area index  
            neuron_ids: List of power neurons to activate
            base_potential: Membrane potential above threshold
            
        Returns:
            Number of candidates injected
        """
        if not neuron_ids:
            return 0
            
        # Convert to numpy arrays
        neuron_ids_array = np.array(neuron_ids, dtype=np.int32)
        potentials = np.full(len(neuron_ids), base_potential, dtype=np.float32)
        excitatory_mask = np.ones(len(neuron_ids), dtype=bool)  # Power areas are excitatory
        
        # Inject into FCL
        added_count = fcl.add_candidates_soa(
            cortical_idx=cortical_idx,
            neuron_ids=neuron_ids_array,
            potential_deltas=potentials,
            excitatory_mask=excitatory_mask
        )
        
        self.injection_count += added_count
        # Power candidates added to FCL
        return added_count
    
    def inject_manual_stimulation(self,
                                fcl: FireCandidateList,
                                stimulation_data: Dict[str, Any]) -> int:
        """Inject manual stimulation data.
        
        Supports both individual neuron stimulation and area-based stimulation.
        
        Args:
            fcl: Fire Candidate List to inject into
            stimulation_data: Dictionary with stimulation parameters
            
        Expected format:
            {
                'cortical_id': 'area_name',
                'neuron_ids': [id1, id2, ...],  # Optional
                'coordinates': [(x,y,z), ...],  # Alternative to neuron_ids
                'potentials': [p1, p2, ...],
                'excitatory': [True, False, ...]  # Optional
            }
            
        Returns:
            Number of candidates injected
        """
        cortical_id = stimulation_data.get('cortical_id')
        if not cortical_id:
            logger.error("Manual stimulation missing cortical_id")
            return 0
        
        # Method 1: Direct neuron IDs
        if 'neuron_ids' in stimulation_data:
            return self._inject_direct_neurons(fcl, stimulation_data)
        
        # Method 2: Coordinate-based stimulation
        elif 'coordinates' in stimulation_data:
            return self._inject_coordinate_stimulation(fcl, stimulation_data)
        
        else:
            logger.error("Manual stimulation must provide either 'neuron_ids' or 'coordinates'")
            return 0
    
    def _inject_direct_neurons(self, fcl: FireCandidateList, data: Dict[str, Any]) -> int:
        """Inject specific neuron IDs directly."""
        neuron_ids = np.array(data['neuron_ids'], dtype=np.int32)
        potentials = np.array(data.get('potentials', [1.0] * len(neuron_ids)), dtype=np.float32)
        excitatory = data.get('excitatory', [True] * len(neuron_ids))
        excitatory_mask = np.array(excitatory, dtype=bool)
        
        # Get cortical index
        cortical_idx = self._get_cortical_idx(data['cortical_id'])
        if cortical_idx is None:
            return 0
        
        added_count = fcl.add_candidates_soa(
            cortical_idx=cortical_idx,
            neuron_ids=neuron_ids,
            potential_deltas=potentials,
            excitatory_mask=excitatory_mask
        )
        
        self.injection_count += added_count
        return added_count
    
    def _inject_coordinate_stimulation(self, fcl: FireCandidateList, data: Dict[str, Any]) -> int:
        """Inject stimulation at specific coordinates."""
        coordinates = data['coordinates']
        potentials = data.get('potentials', [1.0] * len(coordinates))
        
        # Convert coordinates to separate arrays
        x_coords = np.array([c[0] for c in coordinates], dtype=np.int32)
        y_coords = np.array([c[1] for c in coordinates], dtype=np.int32)
        z_coords = np.array([c[2] for c in coordinates], dtype=np.int32)
        potentials_array = np.array(potentials, dtype=np.float32)
        
        # Use coordinate converter
        return self.inject_sensory_data(
            fcl=fcl,
            cortical_id=data['cortical_id'],
            x_coords=x_coords,
            y_coords=y_coords,
            z_coords=z_coords,
            potentials=potentials_array
        )
    
    def inject_synaptic_propagation(self,
                                  fcl: FireCandidateList,
                                  propagation_data: Dict[int, List[Tuple[int, float]]]) -> int:
        """Inject synaptic propagation from previous timestep firing.
        
        Args:
            fcl: Fire Candidate List to inject into
            propagation_data: {cortical_idx: [(target_neuron_id, synaptic_weight), ...]}
            
        Returns:
            Total number of candidates injected
        """
        total_injected = 0
        
        for cortical_idx, connections in propagation_data.items():
            if not connections:
                continue
                
            # Extract target neurons and weights
            target_neurons = [conn[0] for conn in connections]
            synaptic_weights = [conn[1] for conn in connections]
            
            neuron_ids_array = np.array(target_neurons, dtype=np.int32)
            weights_array = np.array(synaptic_weights, dtype=np.float32)
            
            # Determine excitatory/inhibitory based on weight sign
            excitatory_mask = weights_array > 0
            
            added_count = fcl.add_candidates_soa(
                cortical_idx=cortical_idx,
                neuron_ids=neuron_ids_array,
                potential_deltas=np.abs(weights_array),  # Use absolute values
                excitatory_mask=excitatory_mask
            )
            
            total_injected += added_count
        
        self.injection_count += total_injected
        # Synaptic propagation candidates injected
        return total_injected
    
    def _get_cortical_idx(self, cortical_id: str) -> Optional[int]:
        """Get cortical index from cortical ID."""
        # This should integrate with the coordinate converter's mapping
        if hasattr(self.coordinate_converter, '_get_cortical_idx'):
            return self.coordinate_converter._get_cortical_idx(cortical_id)
        return None
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get injection statistics."""
        return {
            'total_injections': self.injection_count,
            'coordinate_converter_stats': self.coordinate_converter.get_cache_stats()
        }
    
    def reset_statistics(self):
        """Reset injection statistics."""
        self.injection_count = 0
        self.coordinate_converter.clear_cache()
    
    # === Batch injection methods ===
    
    def inject_batch(self, fcl: FireCandidateList, injection_batch: List[Dict[str, Any]]) -> int:
        """Inject multiple stimulations in a batch for efficiency."""
        total_injected = 0
        
        for injection in injection_batch:
            injection_type = injection.get('type', 'manual')
            
            if injection_type == 'sensory':
                total_injected += self.inject_sensory_data(fcl, **injection['data'])
            elif injection_type == 'power':
                total_injected += self.inject_power_area(fcl, **injection['data'])
            elif injection_type == 'manual':
                total_injected += self.inject_manual_stimulation(fcl, injection['data'])
            elif injection_type == 'synaptic':
                total_injected += self.inject_synaptic_propagation(fcl, injection['data'])
            else:
                logger.warning(f"Unknown injection type: {injection_type}")
        
        return total_injected
