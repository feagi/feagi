"""
Fire Candidate List (FCL) for FEAGI NPU

Pure pre-burst candidate collector with no historical data or timing complexities.
Responsible for collecting neurons that should undergo membrane potential evaluation.

Key Features:
- Ephemeral storage (cleared after each burst)
- High-performance SoA format: [i1, i2, ...], [p1, p2, ...]
- Excitatory/inhibitory dynamics support
- Rust/RTOS compatible data structures
- Zero mixed responsibilities
"""

from dataclasses import dataclass
from typing import Dict, List, Tuple
import numpy as np
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


@dataclass
class FCLCandidate:
    """Single neuron candidate with membrane potential delta."""
    neuron_id: int
    membrane_potential_delta: float
    is_excitatory: bool = True  # Support for inhibitory candidates


class FireCandidateList:
    """Pure pre-burst candidate collector with no historical data.
    
    This component only collects candidates for the current burst cycle.
    After neural processing, it is immediately cleared.
    """
    
    def __init__(self):
        """Initialize empty FCL for current burst cycle."""
        # SoA storage organized by cortical area for high performance
        self.candidates_by_area: Dict[int, List[FCLCandidate]] = {}
        self.total_candidates: int = 0
        self._membrane_accumulation_enabled: bool = False
        
    def add_candidates_soa(self, 
                          cortical_idx: int,
                          neuron_ids: np.ndarray,         # [i1, i2, ...] 
                          potential_deltas: np.ndarray,   # [p1, p2, ...]
                          excitatory_mask: np.ndarray = None) -> int:
        """Add candidates in SoA format (Rust-friendly).
        
        Args:
            cortical_idx: Cortical area index
            neuron_ids: Array of universal neuron IDs
            potential_deltas: Array of membrane potential deltas
            excitatory_mask: Boolean mask for excitatory (True) vs inhibitory (False)
            
        Returns:
            Number of candidates actually added
        """
        if len(neuron_ids) != len(potential_deltas):
            raise ValueError("neuron_ids and potential_deltas must have same length")
            
        if excitatory_mask is None:
            excitatory_mask = np.ones(len(neuron_ids), dtype=bool)  # Default: all excitatory
            
        if len(excitatory_mask) != len(neuron_ids):
            raise ValueError("excitatory_mask must have same length as neuron_ids")
        
        # Initialize area if needed
        if cortical_idx not in self.candidates_by_area:
            self.candidates_by_area[cortical_idx] = []
            
        area_candidates = self.candidates_by_area[cortical_idx]
        added_count = 0
        
        # Add candidates with excitatory/inhibitory support
        for i in range(len(neuron_ids)):
            candidate = FCLCandidate(
                neuron_id=int(neuron_ids[i]),
                membrane_potential_delta=float(potential_deltas[i]),
                is_excitatory=bool(excitatory_mask[i])
            )
            area_candidates.append(candidate)
            added_count += 1
            
        self.total_candidates += added_count
        
        logger.debug(f"Added {added_count} candidates to cortical area {cortical_idx}")
        return added_count
    
    def add_single_candidate(self,
                           cortical_idx: int, 
                           neuron_id: int,
                           potential_delta: float,
                           is_excitatory: bool = True) -> bool:
        """Add a single candidate (convenience method)."""
        try:
            self.add_candidates_soa(
                cortical_idx,
                np.array([neuron_id], dtype=np.int32),
                np.array([potential_delta], dtype=np.float32),
                np.array([is_excitatory], dtype=bool)
            )
            return True
        except Exception as e:
            logger.error(f"Failed to add single candidate: {e}")
            return False
    
    def get_candidates_by_area(self, cortical_idx: int) -> List[FCLCandidate]:
        """Get all candidates for a specific cortical area."""
        return self.candidates_by_area.get(cortical_idx, [])
    
    def get_all_candidates(self) -> Dict[int, List[FCLCandidate]]:
        """Get all candidates organized by cortical area."""
        return self.candidates_by_area.copy()
    
    def get_candidates_soa(self, cortical_idx: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Get candidates in SoA format for efficient processing.
        
        Returns:
            Tuple of (neuron_ids, potential_deltas, excitatory_mask)
        """
        candidates = self.candidates_by_area.get(cortical_idx, [])
        if not candidates:
            return np.array([]), np.array([]), np.array([])
            
        neuron_ids = np.array([c.neuron_id for c in candidates], dtype=np.int32)
        potential_deltas = np.array([c.membrane_potential_delta for c in candidates], dtype=np.float32)  
        excitatory_mask = np.array([c.is_excitatory for c in candidates], dtype=bool)
        
        return neuron_ids, potential_deltas, excitatory_mask
    
    def process_interactive_dynamics(self, cortical_idx: int) -> int:
        """Process excitatory/inhibitory dynamics within cortical area.
        
        This method handles cancellation of excitatory candidates by inhibitory ones
        and other interactive dynamics that occur during pre-burst processing.
        
        Returns:
            Number of candidates remaining after dynamics processing
        """
        candidates = self.candidates_by_area.get(cortical_idx, [])
        if not candidates:
            return 0
            
        # Group candidates by neuron_id for dynamics processing
        neuron_potentials = {}
        for candidate in candidates:
            if candidate.neuron_id not in neuron_potentials:
                neuron_potentials[candidate.neuron_id] = 0.0
                
            # Apply excitatory/inhibitory contributions
            delta = candidate.membrane_potential_delta
            if not candidate.is_excitatory:
                delta = -abs(delta)  # Ensure inhibitory is negative
                
            neuron_potentials[candidate.neuron_id] += delta
        
        # Create new candidate list with processed dynamics
        processed_candidates = []
        for neuron_id, total_delta in neuron_potentials.items():
            if abs(total_delta) > 1e-6:  # Only keep non-zero contributions
                processed_candidates.append(FCLCandidate(
                    neuron_id=neuron_id,
                    membrane_potential_delta=total_delta,
                    is_excitatory=(total_delta > 0)
                ))
        
        # Update the area with processed candidates
        old_count = len(candidates)
        self.candidates_by_area[cortical_idx] = processed_candidates
        new_count = len(processed_candidates)
        
        # Update total count
        self.total_candidates += (new_count - old_count)
        
        logger.debug(f"Interactive dynamics: {old_count} → {new_count} candidates in area {cortical_idx}")
        return new_count
    
    def get_statistics(self) -> Dict:
        """Get FCL statistics for monitoring."""
        area_counts = {area_idx: len(candidates) 
                      for area_idx, candidates in self.candidates_by_area.items()}
        
        excitatory_count = sum(1 for candidates in self.candidates_by_area.values() 
                              for c in candidates if c.is_excitatory)
        inhibitory_count = self.total_candidates - excitatory_count
        
        return {
            'total_candidates': self.total_candidates,
            'areas_with_candidates': len(self.candidates_by_area),
            'candidates_per_area': area_counts,
            'excitatory_candidates': excitatory_count,
            'inhibitory_candidates': inhibitory_count
        }
    
    def clear(self):
        """Clear all candidates for next burst cycle.
        
        FCL is ephemeral - must be cleared after each burst processing.
        """
        self.candidates_by_area.clear()
        self.total_candidates = 0
        logger.debug("FCL cleared for next burst cycle")
    
    def is_empty(self) -> bool:
        """Check if FCL has any candidates."""
        return self.total_candidates == 0
    
    def enable_membrane_accumulation(self, enabled: bool = True):
        """Enable/disable membrane potential accumulation mode.
        
        When disabled, membrane potentials reset to zero if neuron doesn't fire.
        When enabled, membrane potentials persist across timesteps.
        """
        self._membrane_accumulation_enabled = enabled
        logger.debug(f"Membrane accumulation: {'enabled' if enabled else 'disabled'}")
        
    def is_membrane_accumulation_enabled(self) -> bool:
        """Check if membrane potential accumulation is enabled."""
        return self._membrane_accumulation_enabled
