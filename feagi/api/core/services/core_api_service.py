"""Core API service implementation for FEAGI."""

from typing import Dict, Any, List, Optional, Tuple, Union, Set
from feagi.utils.logger import setup_logger
import os
import json
import tempfile
from datetime import datetime
import time
from pathlib import Path
import sys

import numpy as np
try:
    import pyroaring
    PYROARING_AVAILABLE = True
except ImportError:
    PYROARING_AVAILABLE = False
    pyroaring = None

logger = setup_logger()

from feagi.core.feagi import FEAGI
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis, develop_brain_from_genome
from feagi.bdu.connectome_manager import ConnectomeManager, CorticalArea
from feagi.core.state_manager import FeagiStateManager
from feagi.core.state_manager import ServiceState, SimulationState
from feagi.core.genome_transaction import GenomeTransaction
from feagi.core.state_manager import GenomeState
from feagi.bdu.connectivity import synaptogenesis_rules
try:
    # Try to import these from the new location
    from feagi.evo.genome_validator import genome_validator
    from feagi.evo.genome_editor import save_genome
    from feagi.evo.genome_processor import (
        merge_core_morphologies, 
        genome_morphology_updator, 
        genome_physiology_updator, 
        genome_stat_updator
    )
except ImportError:
    # Fall back to the old location
    from feagi.core.genome.genome_validator import genome_validator
    from feagi.core.genome.genome_editor import save_genome
    from feagi.core.genome.genome_processor import (
        merge_core_morphologies, 
        genome_morphology_updator, 
        genome_physiology_updator, 
        genome_stat_updator
    )

class CoreAPIService:
    """
    Core API Service for FEAGI.
    
    This service manages high-level operations on the FEAGI brain and coordinates
    between the various components.
    
    Naming Convention:
    -----------------
    * cortical_id: 6-character unique identifier from the genome (e.g., "iv00_C") 
      - Used in the genome's blueprint
    * cortical_idx: Auto-incremented integer ID used internally (previously called area_id)
      - These are converted to strings in the API layer and back to ints internally
    """
    
    def __init__(self, connectome_manager: ConnectomeManager, state_manager=None):
        """
        Initialize the Core API service.
        
        Args:
            connectome_manager: ConnectomeManager instance
            state_manager: FeagiStateManager instance
        """
        # Check that connectome_manager is properly initialized
        if not hasattr(connectome_manager, 'fcl_manager') or connectome_manager.fcl_manager is None:
            raise RuntimeError("ConnectomeManager instance does not have an fcl_manager. Did you forget to call initialize_arrays() before passing it to CoreAPIService?")
        
        self._connectome_manager = connectome_manager
        
        # Initialize state manager
        self.state_manager = state_manager or FeagiStateManager.instance()
        # Register as observer
        self.state_manager.register_sync_observer(self)
        
        # Initialize burst engine without requiring a genome - but only if not already initialized
        if not self.state_manager.is_burst_engine_ready():
            self.state_manager.set_burst_engine_state(ServiceState.INITIALIZING)
            self._burst_engine = self._create_burst_engine()
            self.state_manager.set_burst_engine_state(ServiceState.READY)
        else:
            # Burst engine already exists, just reference it
            self._burst_engine = None  # Will be retrieved from state manager when needed
        
        # Other initializations can follow...
        
        self.logger = logger
        self._feagi = FEAGI()
        self._temp_dir = tempfile.mkdtemp(prefix="feagi_")
        self._genome_filename = None
        self._pending_amalgamation = {}
        self._current_genome = None  # Initialize _current_genome to None

        # Cache for frequently accessed data
        self._cortical_areas_cache = None
        self._cortical_areas_cache_timestamp = 0
        
    def _handle_embryogenesis_progress(self, stage, percentage, message):
        """Handle progress updates from the neuroembryogenesis process."""
        self.logger.info(f"{stage} {percentage:.1f}% - {message}", emoji1="  ")
        
    @property
    def feagi(self) -> FEAGI:
        """Get the FEAGI instance."""
        return self._feagi
        
    def get_burst_engine(self):
        """Get the Burst Engine component."""
        # For now, return None as this component isn't fully implemented
        return None
        
    def get_connectome_manager(self) -> ConnectomeManager:
        """
        Get the connectome manager instance.
        
        Returns:
            ConnectomeManager: The connectome manager instance
        """
        return self._connectome_manager
        
    def get_fcl_manager(self):
        """Get the FCL Manager component."""
        return self._connectome_manager.fcl_manager
        
    def get_memory_manager(self):
        """Get the Memory & Learning Manager component."""
        # For now, return None as this component isn't fully implemented
        return None
    
    # Fire queue methods for FQSampler
    
    def get_fire_queue(self) -> Optional[Dict[str, Any]]:
        """
        Get the current global fire queue data.
        
        Returns:
            Dictionary with fire queue data: {
                'neuron_ids': List[int],
                'membrane_potentials': List[float], 
                'thresholds': List[float],
                'consecutive_fire_counts': List[int],
                'refractory_counters': List[int]
            }
        """
        try:
            # Get fire queue from optimized core if available
            if hasattr(self._connectome_manager, 'get_optimized_core'):
                core = self._connectome_manager.get_optimized_core()
                if core and hasattr(core, 'get_fire_queue'):
                    return core.get_fire_queue()
            
            # Fallback: create fire queue from current FCL state
            if hasattr(self._connectome_manager, 'fcl_manager') and self._connectome_manager.fcl_manager:
                fcl_manager = self._connectome_manager.fcl_manager
                global_fcl = fcl_manager.get_global_fcl()
                
                if isinstance(global_fcl, dict):
                    # Aggregate all area FCLs
                    neuron_ids = []
                    for area_id, area_fcl in global_fcl.items():
                        if hasattr(area_fcl, '__iter__'):
                            neuron_ids.extend(list(area_fcl))
                elif hasattr(global_fcl, '__iter__'):
                    neuron_ids = list(global_fcl)
                else:
                    neuron_ids = []
                
                # Create mock fire queue data
                return {
                    'neuron_ids': neuron_ids,
                    'membrane_potentials': [1.0] * len(neuron_ids),  # Default potential
                    'thresholds': [1.0] * len(neuron_ids),  # Default threshold
                    'consecutive_fire_counts': [0] * len(neuron_ids),  # Default count
                    'refractory_counters': [0] * len(neuron_ids)  # Default counter
                }
            
            return None
            
        except Exception as e:
            logger.error(f"Error getting fire queue: {e}")
            return None
    
    def get_area_fire_queue(self, cortical_id: str) -> Optional[Dict[str, Any]]:
        """
        Get fire queue data for a specific cortical area.
        
        Args:
            cortical_id: The cortical area ID
            
        Returns:
            Dictionary with fire queue data for the area
        """
        try:
            # Get fire queue from optimized core if available
            if hasattr(self._connectome_manager, 'get_optimized_core'):
                core = self._connectome_manager.get_optimized_core()
                if core and hasattr(core, 'get_area_fire_queue'):
                    return core.get_area_fire_queue(cortical_id)
            
            # Fallback: get area FCL and convert to fire queue format
            if not self._connectome_manager:
                logger.warning("No connectome manager available for fire queue data")
                return None

            fcl_manager = self._connectome_manager.fcl_manager
            area_fcl = fcl_manager.get_cortical_fcl(cortical_id)
            
            if area_fcl and hasattr(area_fcl, '__iter__'):
                neuron_ids = list(area_fcl)
                
                # Check if we have valid neuron IDs or if we need to generate test data
                if not neuron_ids or all(nid == 0 for nid in neuron_ids):
                    if self._is_test_visualization_mode():
                        neuron_ids = self._generate_synthetic_neural_activity(cortical_id)
                        if neuron_ids:
                            logger.debug(f"Generated synthetic neural activity: {len(neuron_ids)} neurons for {cortical_id}")
                
                if neuron_ids:
                    # Create mock fire queue data
                    fire_queue_data = {
                        'neuron_ids': neuron_ids,
                        'membrane_potentials': [1.0] * len(neuron_ids),  # Default potential
                        'thresholds': [1.0] * len(neuron_ids),  # Default threshold
                        'consecutive_fire_counts': [0] * len(neuron_ids),  # Default count
                        'refractory_counters': [0] * len(neuron_ids)  # Default counter
                    }
                    
                    return fire_queue_data
            else:
                # Check if we're in test visualization mode and generate synthetic activity
                if self._is_test_visualization_mode():
                    neuron_ids = self._generate_synthetic_neural_activity(cortical_id)
                    if neuron_ids:
                        fire_queue_data = {
                            'neuron_ids': neuron_ids,
                            'membrane_potentials': [1.0] * len(neuron_ids),
                            'thresholds': [1.0] * len(neuron_ids),
                            'consecutive_fire_counts': [0] * len(neuron_ids),
                            'refractory_counters': [0] * len(neuron_ids)
                        }
                        return fire_queue_data
            
            return None
            
        except Exception as e:
            logger.error(f"Error getting area fire queue for {cortical_id}: {e}")
            return None

    def _is_test_visualization_mode(self) -> bool:
        """Check if FEAGI is running in test visualization mode."""
        try:
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            return state_manager.get_test_visualization_mode()
        except Exception:
            return False

    def _generate_synthetic_neural_activity(self, cortical_id: str) -> List[int]:
        """Generate synthetic neural activity for test visualization mode."""
        import random
        import time
        
        try:
            # Get the cortical area to determine neuron range
            if not hasattr(self._connectome_manager, 'cortical_areas'):
                return []
                
            # Find the area by cortical_id
            area = None
            for area_key, area_obj in self._connectome_manager.cortical_areas.items():
                if str(area_key) == str(cortical_id) or (hasattr(area_obj, 'cortical_id') and area_obj.cortical_id == cortical_id):
                    area = area_obj
                    break
                    
            if not area:
                logger.warning(f"Could not find area for cortical_id {cortical_id}")
                return []
                
            # Get area dimensions
            dimensions = getattr(area, 'dimensions', (10, 10, 1))
            total_neurons = dimensions[0] * dimensions[1] * dimensions[2]
            
            # Skip areas with zero or very small neuron counts
            if total_neurons <= 0:
                return []
            
            # Generate realistic neuron IDs based on area size
            max_neurons = min(total_neurons, 1000)  # Cap at 1000 for performance
            
            # Skip areas with insufficient neurons for meaningful activity
            if max_neurons < 2:
                return []
            
            # Create a deterministic but varied pattern based on time and cortical_id
            random.seed(int(time.time()) + hash(cortical_id) % 1000)
            
            # Generate 5-15% of neurons firing (realistic for most areas)
            num_firing = random.randint(max(1, max_neurons // 20), max_neurons // 7)
            
            # Generate unique neuron IDs within the area's range
            # Use area-specific offset to ensure unique IDs across areas
            area_offset = hash(cortical_id) % 10000
            neuron_ids = []
            
            for i in range(num_firing):
                # Generate neuron ID within area range
                local_id = random.randint(1, max_neurons)
                global_neuron_id = area_offset + local_id
                neuron_ids.append(global_neuron_id)
                
            # Remove duplicates and sort
            neuron_ids = sorted(list(set(neuron_ids)))
            
            return neuron_ids
            
        except Exception as e:
            logger.error(f"Error generating synthetic neural activity: {e}")
            return []
    
    # Brain state management methods
    
    def get_brain_state(self) -> Dict[str, Any]:
        """
        Get the current brain state.
        
        Returns:
            Dictionary containing the current brain state.
        """
        return self._feagi.get_brain_state()
        
    def save_brain_state(self, path: str) -> bool:
        """
        Save the current brain state to a file.
        
        Args:
            path: Path to save the brain state.
            
        Returns:
            True if successful, False otherwise.
        """
        return self._feagi.save_brain_state(path)
        
    def load_brain_state(self, path: str) -> bool:
        """
        Load a brain state from a file.
        
        Args:
            path: Path to the brain state file.
            
        Returns:
            True if successful, False otherwise.
        """
        return self._feagi.load_brain_state(path)
        
    # Cortical area methods
    
    def get_cortical_areas(self) -> List[Dict[str, Any]]:
        """
        Get all cortical areas.
        
        Returns:
            List of dictionaries containing cortical area information.
        """
        # Check if we can use the cached version
        if self._cortical_areas_cache is not None:
            return self._cortical_areas_cache
            
        result = []
        try:
            if not hasattr(self._connectome_manager, 'cortical_areas') or not self._connectome_manager.cortical_areas:
                # No areas exist at all, just return an empty list
                # This is normal when no genome has been loaded yet
                return []
            
            # Convert all areas to API format
            for cortical_id, area in self._connectome_manager.cortical_areas.items():
                try:
                    # Get neuron count (safely)
                    try:
                        neuron_count = len(self._connectome_manager.get_neurons_by_area(cortical_id))
                    except Exception:
                        neuron_count = 0
                        
                    # Convert to API format
                    result.append({
                        "id": str(cortical_id),  # Convert to string for API consistency
                        "name": area.name,
                        "coordinates": {
                            "x": area.position[0],
                            "y": area.position[1],
                            "z": area.position[2]
                        },
                        "dimensions": {
                            "width": area.dimensions[0],
                            "height": area.dimensions[1],
                            "depth": area.dimensions[2]
                        },
                        "type": area.type,
                        "parameters": area.properties,
                        "neuron_count": neuron_count
                    })
                except Exception as e:
                    self.logger.error(f"Error converting area {cortical_id} to API format: {str(e)}")
                    
        except Exception as e:
            self.logger.error(f"Error retrieving cortical areas: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
        
        # Cache the result
        self._cortical_areas_cache = result
        self._cortical_areas_cache_timestamp = time.time()
        
        return result
        
    def get_cortical_area(self, cortical_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a cortical area by ID.
        
        Args:
            cortical_id: String representation of the cortical_idx.
            
        Returns:
            Dictionary containing cortical area information, or None if not found.
        """
        try:
            # Convert string ID to integer (cortical_idx)
            try:
                cortical_idx = int(cortical_id)
            except ValueError:
                self.logger.error(f"Invalid cortical area ID format: {cortical_id}")
                return None
            
            # Get the area from connectome manager
            area = self._connectome_manager.cortical_areas.get(cortical_idx)
            if not area:
                self.logger.warning(f"Cortical area {cortical_id} not found")
                return None
            
            # Return area information
            neuron_count = len(self._connectome_manager.get_neurons_by_area(cortical_idx))
            
            # Format response
            return {
                "id": str(cortical_idx),
                "name": area.name,
                "coordinates": {
                    "x": area.position[0],
                    "y": area.position[1],
                    "z": area.position[2]
                },
                "dimensions": {
                    "width": area.dimensions[0],
                    "height": area.dimensions[1],
                    "depth": area.dimensions[2]
                },
                "type": area.type,
                "parameters": area.properties,
                "neuron_count": neuron_count
            }
        except Exception as e:
            self.logger.error(f"Error retrieving cortical area: {str(e)}")
            return None
    
    def create_cortical_area(
        self, 
        name: str,
        coordinates: Dict[str, int],
        dimensions: Dict[str, int],
        area_type: str,
        parameters: Dict[str, Any] = None
    ) -> Optional[Dict[str, Any]]:
        """
        Create a new cortical area.
        
        Args:
            name: Name of the cortical area.
            coordinates: 3D coordinates of the cortical area.
            dimensions: Dimensions of the cortical area.
            area_type: Type of the cortical area.
            parameters: Additional parameters for the cortical area.
            
        Returns:
            Dictionary containing the created cortical area information,
            or None if creation failed.
        """
        # In legacy FEAGI, this depends on a genome being loaded first
        if self._current_genome is None:
            self.logger.warning("No genome loaded, cannot create cortical area")
            return None
        
        try:
            # Generate a unique ID for the new area
            # In a real implementation, this might be more sophisticated
            existing_ids = set(self._connectome_manager.cortical_areas.keys())
            new_id = 1
            while new_id in existing_ids:
                new_id += 1
            
            # Convert API format to internal representation
            position = (coordinates["x"], coordinates["y"], coordinates["z"])
            dims = (dimensions["width"], dimensions["height"], dimensions["depth"])
            
            # Create the area in the connectome manager
            area = self._connectome_manager.add_cortical_area(
                name=name,
                dimensions=dims,
                position=position,
                area_type=area_type,
                properties=parameters or {},
                cortical_id=new_id
            )
            
            # Return the created area information
            return {
                "id": str(new_id),
                "name": area.name,
                "coordinates": coordinates,
                "dimensions": dimensions,
                "type": area.type,
                "parameters": area.properties,
                "neuron_count": 0  # New area has no neurons yet
            }
        except Exception as e:
            self.logger.error(f"Failed to create cortical area: {str(e)}")
            return None
    
    def update_cortical_area(
        self,
        cortical_id: str,
        name: Optional[str] = None,
        coordinates: Optional[Dict[str, int]] = None,
        dimensions: Optional[Dict[str, int]] = None,
        area_type: Optional[str] = None,
        parameters: Optional[Dict[str, Any]] = None
    ) -> Optional[Dict[str, Any]]:
        """
        Update an existing cortical area.
        
        Args:
            cortical_id: ID of the cortical area to update.
            name: New name for the cortical area.
            coordinates: New coordinates for the cortical area.
            dimensions: New dimensions for the cortical area.
            area_type: New type for the cortical area.
            parameters: New parameters for the cortical area.
            
        Returns:
            Dictionary containing the updated cortical area information,
            or None if the area doesn't exist.
        """
        # In legacy FEAGI, this depends on a genome being loaded first
        if self._current_genome is None:
            self.logger.warning("No genome loaded, cannot update cortical area")
            return None
        
        try:
            cortical_idx = int(cortical_id)
        except ValueError:
            return None
        
        try:
            area = self._connectome_manager.cortical_areas.get(cortical_idx)
            if not area:
                return None
            
            # Update the area properties
            if name is not None:
                area.name = name
            
            if coordinates is not None:
                area.position = (coordinates["x"], coordinates["y"], coordinates["z"])
            
            if dimensions is not None:
                area.dimensions = (dimensions["width"], dimensions["height"], dimensions["depth"])
            
            if area_type is not None:
                area.type = area_type
            
            if parameters is not None:
                area.properties.update(parameters)
            
            # Return the updated area
            neuron_count = len(self._connectome_manager.get_neurons_by_area(cortical_idx))
            return {
                "cortical_id": cortical_id,
                "cortical_idx": area.cortical_idx,
                "name": area.name,
                "coordinates": {
                    "x": area.position[0],
                    "y": area.position[1],
                    "z": area.position[2]
                },
                "dimensions": {
                    "width": area.dimensions[0],
                    "height": area.dimensions[1],
                    "depth": area.dimensions[2]
                },
                "type": area.type,
                "parameters": area.properties,
                "neuron_count": neuron_count
            }
        except Exception as e:
            self.logger.error(f"Error updating cortical area {cortical_id}: {str(e)}")
            return None
    
    def delete_cortical_area(self, cortical_id: str) -> bool:
        """
        Delete a cortical area.
        
        Args:
            cortical_id: String representation of the cortical_idx to delete.
            
        Returns:
            True if the cortical area was deleted, False otherwise.
        """
        # In legacy FEAGI, this depends on a genome being loaded first
        if self._current_genome is None:
            self.logger.warning("No genome loaded, cannot delete cortical area")
            return False
        
        try:
            cortical_idx = int(cortical_id)
        except ValueError:
            return False
        
        try:
            if cortical_idx not in self._connectome_manager.cortical_areas:
                return False
            
            # Get all neurons in this area
            neurons = self._connectome_manager.get_neurons_by_area(cortical_idx)
            
            # Delete all neurons in the area
            for neuron_id in neurons:
                self._connectome_manager.delete_neuron(neuron_id)
            
            # Remove the area
            del self._connectome_manager.cortical_areas[cortical_idx]
            
            # Clean up any area-specific data structures
            if cortical_idx in self._connectome_manager._occupied_voxels:
                del self._connectome_manager._occupied_voxels[cortical_idx]
            
            if cortical_idx in self._connectome_manager._area_lookup_tables:
                del self._connectome_manager._area_lookup_tables[cortical_idx]
            
            # Remove from area classification sets
            if cortical_idx in self._connectome_manager._small_regular_areas:
                self._connectome_manager._small_regular_areas.remove(cortical_idx)
            
            if cortical_idx in self._connectome_manager._large_regular_areas:
                self._connectome_manager._large_regular_areas.remove(cortical_idx)
            
            if cortical_idx in self._connectome_manager._extreme_dimension_areas:
                self._connectome_manager._extreme_dimension_areas.remove(cortical_idx)
                
            return True
        except Exception as e:
            self.logger.error(f"Error deleting cortical area {cortical_id}: {str(e)}")
            return False
    
    def get_cortical_area_neurons(self, cortical_id: str) -> Optional[List[Dict[str, Any]]]:
        """
        Get neurons for a specific cortical area.
        
        Args:
            cortical_id: ID of the cortical area.
            
        Returns:
            List of dictionaries containing neuron information,
            or None if the area doesn't exist.
        """
        # In legacy FEAGI, this depends on a genome being loaded first
        if self._current_genome is None:
            self.logger.warning("No genome loaded, cannot retrieve cortical area neurons")
            return None
        
        try:
            cortical_idx = int(cortical_id)
        except ValueError:
            return None
        
        try:
            if cortical_idx not in self._connectome_manager.cortical_areas:
                return None
            
            # Get all neurons in this area
            neuron_ids = self._connectome_manager.get_neurons_by_area(cortical_idx)
            result = []
            
            for neuron_id in neuron_ids:
                # Get neuron index for accessing property arrays
                neuron_index = self._connectome_manager._neuron_id_to_index.get(neuron_id)
                if neuron_index is None:
                    continue
                
                # Get neuron position
                position = self._connectome_manager.get_neuron_position(neuron_id)
                
                # Get neuron properties
                membrane_potential = float(self._connectome_manager.membrane_potentials[neuron_index])
                threshold = float(self._connectome_manager.thresholds[neuron_index])
                decay_rate = float(self._connectome_manager.decay_rates[neuron_index])
                
                result.append({
                    "id": str(neuron_id),
                    "position": {
                        "x": position[0],
                        "y": position[1],
                        "z": position[2]
                    },
                    "properties": {
                        "membrane_potential": membrane_potential,
                        "threshold": threshold,
                        "decay_rate": decay_rate
                    }
                })
            
            return result
        except Exception as e:
            self.logger.error(f"Error retrieving neurons for cortical area {cortical_id}: {str(e)}")
            return None
    
    def get_cortical_area_activity(self, cortical_id: str, window: int = 1) -> Optional[Dict[str, Any]]:
        """
        Get activity data for a specific cortical area.
        
        Args:
            cortical_id: ID of the cortical area.
            window: Time window for activity data (in bursts).
            
        Returns:
            Dictionary containing activity data for the cortical area,
            or None if the area doesn't exist.
        """
        # In legacy FEAGI, this depends on a genome being loaded first
        if self._current_genome is None:
            self.logger.warning("No genome loaded, cannot retrieve cortical area activity")
            return None
        
        try:
            cortical_idx = int(cortical_id)
        except ValueError:
            return None
        
        try:
            if cortical_idx not in self._connectome_manager.cortical_areas:
                return None
            
            # Get all neurons in this area
            neuron_ids = self._connectome_manager.get_neurons_by_area(cortical_idx)
            
            # Current timestep
            current_time = self._connectome_manager.current_timestep
            
            # Get neurons that fired within the window
            active_neurons = []
            for neuron_id in neuron_ids:
                neuron_index = self._connectome_manager._neuron_id_to_index.get(neuron_id)
                if neuron_index is None:
                    continue
                
                last_fired = int(self._connectome_manager.last_fired[neuron_index])
                if last_fired > 0 and (current_time - last_fired) <= window:
                    position = self._connectome_manager.get_neuron_position(neuron_id)
                    active_neurons.append({
                        "id": str(neuron_id),
                        "position": {
                            "x": position[0],
                            "y": position[1],
                            "z": position[2]
                        },
                        "last_fired": last_fired
                    })
            
            # Calculate activity summary
            total_neurons = len(neuron_ids)
            active_count = len(active_neurons)
            
            return {
                "total_neurons": total_neurons,
                "active_neurons": active_count,
                "activity_ratio": active_count / total_neurons if total_neurons > 0 else 0,
                "active_details": active_neurons[:100]  # Limit to prevent huge responses
            }
        except Exception as e:
            self.logger.error(f"Error retrieving activity for cortical area {cortical_id}: {str(e)}")
            return None
    
    def get_cortical_area_connectivity(self, cortical_id: str, direction: str = "both") -> Optional[Dict[str, Any]]:
        """
        Get connectivity information for a specific cortical area.
        
        Args:
            cortical_id: ID of the cortical area.
            direction: Connection direction ('incoming', 'outgoing', or 'both').
            
        Returns:
            Dictionary containing connectivity information for the cortical area,
            or None if the area doesn't exist.
        """
        # In legacy FEAGI, this depends on a genome being loaded first
        if self._current_genome is None:
            self.logger.warning("No genome loaded, cannot retrieve cortical area connectivity")
            return None
        
        try:
            cortical_idx = int(cortical_id)
        except ValueError:
            return None
        
        try:
            if cortical_idx not in self._connectome_manager.cortical_areas:
                return None
            
            # Get all neurons in this area
            neuron_ids = self._connectome_manager.get_neurons_by_area(cortical_idx)
            
            # Collect connectivity information
            incoming_connections = set()
            outgoing_connections = set()
            
            if direction in ["incoming", "both"]:
                for neuron_id in neuron_ids:
                    connections = self._connectome_manager.get_incoming_connections(neuron_id)
                    for pre_id, _ in connections:
                        # Skip connections within the same area
                        pre_area = self._connectome_manager._neuron_to_area.get(pre_id)
                        if pre_area is not None and pre_area != cortical_idx:
                            incoming_connections.add(pre_area)
        
            if direction in ["outgoing", "both"]:
                for neuron_id in neuron_ids:
                    connections = self._connectome_manager.get_outgoing_connections(neuron_id)
                    for post_id, _ in connections:
                        # Skip connections within the same area
                        post_area = self._connectome_manager._neuron_to_area.get(post_id)
                        if post_area is not None and post_area != cortical_idx:
                            outgoing_connections.add(post_area)
            
            # Format results
            result = {
                "cortical_id": str(cortical_idx),
                "direction": direction
            }
            
            if direction in ["incoming", "both"]:
                result["incoming_connections"] = [
                    {
                        "cortical_id": str(connected_area),
                        "name": self._connectome_manager.cortical_areas.get(connected_area, CorticalArea(connected_area, "Unknown", "unknown", (0, 0, 0), (0, 0, 0))).name
                    } 
                    for connected_area in incoming_connections
                ]
            
            if direction in ["outgoing", "both"]:
                result["outgoing_connections"] = [
                    {
                        "cortical_id": str(connected_area),
                        "name": self._connectome_manager.cortical_areas.get(connected_area, CorticalArea(connected_area, "Unknown", "unknown", (0, 0, 0), (0, 0, 0))).name
                    }
                    for connected_area in outgoing_connections
                ]
            
            return result
        except Exception as e:
            self.logger.error(f"Error retrieving connectivity for cortical area {cortical_id}: {str(e)}")
            return None
    
    def stimulate_cortical_area(self, cortical_id: str, pattern: str = "random", 
                               intensity: float = 1.0, duration: int = 1,
                               coordinates: Optional[List[Dict[str, int]]] = None) -> Dict[str, Any]:
        """
        Stimulate a cortical area with the specified pattern.
        
        Args:
            cortical_id: ID of the cortical area to stimulate.
            pattern: Stimulation pattern (random, specific, etc.)
            intensity: Stimulation intensity (0.0-1.0)
            duration: Stimulation duration in bursts
            coordinates: Specific coordinates to stimulate
            
        Returns:
            Information about the applied stimulation.
        """
        # This is a placeholder implementation
        self.logger.info(f"stimulate_cortical_area called with cortical_id={cortical_id}, pattern={pattern}")
        return {
            "stimulated_neurons": 100,
            "timestamp": 123456789
        }
        
    def get_burst_engine_config(self) -> Dict[str, Any]:
        """
        Get the burst engine configuration.
        
        Returns:
            Dictionary containing burst engine configuration.
        """
        self.logger.info("get_burst_engine_config called")
        # Delegate to the FEAGI instance to match the legacy API format
        return self._feagi.get_burst_engine_config()
        
    def update_burst_engine_config(self, config: Dict[str, Any]) -> bool:
        """
        Update the burst engine configuration.
        
        Args:
            config: Updated burst engine configuration.
            
        Returns:
            True if successful, False otherwise.
        """
        # This is a placeholder implementation
        self.logger.info(f"update_burst_engine_config called with config={config}")
        return True
        
    def get_burst_engine_stats(self) -> Dict[str, Any]:
        """
        Get statistics from the burst engine.
        
        Returns:
            Dictionary containing burst engine statistics.
        """
        # This is a placeholder implementation
        self.logger.info("get_burst_engine_stats called")
        return {
            "average_burst_time": 8.5,
            "max_burst_time": 12.3,
            "min_burst_time": 7.1,
            "total_bursts": 1000,
            "average_active_neurons": 500,
            "memory_usage": 128.5
        }
    
    def _create_burst_engine(self):
        """
        Create and initialize a BurstEngine instance.
        
        Returns:
            An initialized BurstEngine instance
        """
        from feagi.npu.burst_engine import BurstEngine
        
        # Create the burst engine with our connectome manager
        burst_engine = BurstEngine(
            connectome_manager=self._connectome_manager,
            fcl_manager=self._connectome_manager.fcl_manager,
            config={"desired_frequency_hz": 60.0}  # Reasonable default
        )
        
        return burst_engine

    def on_sync_state_change(self, old_state, new_state, details):
        """React to sync state changes"""
        if new_state == ServiceState.SYNC_COMPLETE:
            # Update any cached data or notify dependent systems
            self.refresh_cached_data()
        
    def begin_transaction(self):
        """Begin a new genome modification transaction"""
        return GenomeTransaction(self.state_manager)
    
    def modify_genome(self, transaction):
        """Apply changes from a genome transaction.
        
        Args:
            transaction: The GenomeTransaction object with recorded changes
            
        Returns:
            bool: Success or failure
        """
        if not self.state_manager:
            logger.error("Cannot modify genome - state manager not initialized", emoji1="❌")
            return False
        
        if not self.genome_is_loaded():
            logger.error("Cannot modify genome - no genome loaded", emoji1="❌")
            return False
        
        # Apply transaction
        success = transaction.commit()
        
        if success:
            # Notify any listeners about genome changes - use LOADED instead of MODIFIED
            self.state_manager.set_genome_state(GenomeState.LOADED)
        
        return success

    def register_genome_change_listener(self, callback):
        """Register a function to be called when the genome changes.
        
        Args:
            callback: Function to call when genome changes
        """
        if self.state_manager:
            return self.state_manager.register_notification_callback("genome", callback)
        return False
    
    def refresh_cached_data(self):
        """Refresh any cached data when state changes occur"""
        # Clear caches so they'll be rebuilt on next access
        self._cortical_areas_cache = None
        self._cortical_areas_cache_timestamp = 0

    def genome_is_loaded(self) -> bool:
        """Check if a genome is currently loaded.
        
        Returns:
            True if a genome is loaded, False otherwise.
        """
        # Check our internal state first
        if self._current_genome is not None:
            return True
        
        # Then check the state manager
        if self.state_manager:
            return self.state_manager.is_genome_loaded()
        
        return False

    def load_essential_genome(self) -> Dict[str, Any]:
        """
        Load the essential genome from the default templates.
            
        Returns:
            Dictionary containing the result of the genome loading process
        """
        try:
            # Multiple possible locations to look for the essential genome
            possible_paths = [
                # Original path
                os.path.join(os.path.dirname(__file__), "../../../../evo/defaults/genome/essential_genome.json"),
                
                # Alternative paths relative to current file
                os.path.join(os.path.dirname(__file__), "../../../../../feagi/evo/defaults/genome/essential_genome.json"),
                os.path.join(os.path.dirname(__file__), "../../../../../evo/defaults/genome/essential_genome.json"),
                
                # Paths relative to working directory
                os.path.join(os.getcwd(), "feagi/evo/defaults/genome/essential_genome.json"),
                os.path.join(os.getcwd(), "feagi_core/feagi/evo/defaults/genome/essential_genome.json"),
                
                # Check FEAGI_HOME environment variable if set
                os.path.join(os.environ.get("FEAGI_HOME", ""), "evo/defaults/genome/essential_genome.json"),
            ]
            
            # Find the first existing path
            essential_path = None
            for path in possible_paths:
                if path and os.path.exists(path):
                    essential_path = path
                    break
                    
            if not essential_path:
                self.logger.error(f"Essential genome template not found in any expected location")
                self.logger.error(f"Checked paths: {possible_paths}")
                return {"success": False, "error": "Essential genome template not found"}
                
            self.logger.info(f"Loading essential genome from {essential_path}")
                
            with open(essential_path, 'r') as f:
                genome_data = json.load(f)
            
            # Set the genome file name
            if self.state_manager:
                self.state_manager.genome_file_name = "essential_genome.json"
            
            # Call the existing load_genome method
            self._genome_filename = "essential_genome.json"
            result = self.load_genome(genome_data, "essential_genome.json")
            
            return result
            
        except Exception as e:
            self.logger.error(f"Failed to load essential genome: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
            return {"success": False, "error": str(e)}

    def load_barebones_genome(self) -> Dict[str, Any]:
        """
        Load the barebones genome from the default templates.
            
        Returns:
            Dictionary containing the result of the genome loading process
        """
        try:
            # Multiple possible locations to look for the barebones genome
            possible_paths = [
                # Original path
                os.path.join(os.path.dirname(__file__), "../../../../evo/defaults/genome/barebones_genome.json"),
                
                # Alternative paths relative to current file
                os.path.join(os.path.dirname(__file__), "../../../../../feagi/evo/defaults/genome/barebones_genome.json"),
                os.path.join(os.path.dirname(__file__), "../../../../../evo/defaults/genome/barebones_genome.json"),
                
                # Paths relative to working directory
                os.path.join(os.getcwd(), "feagi/evo/defaults/genome/barebones_genome.json"),
                os.path.join(os.getcwd(), "feagi_core/feagi/evo/defaults/genome/barebones_genome.json"),
                
                # Check FEAGI_HOME environment variable if set
                os.path.join(os.environ.get("FEAGI_HOME", ""), "evo/defaults/genome/barebones_genome.json"),
            ]
            
            # Find the first existing path
            barebones_path = None
            for path in possible_paths:
                if path and os.path.exists(path):
                    barebones_path = path
                    break
                    
            if not barebones_path:
                self.logger.error(f"Barebones genome template not found in any expected location")
                self.logger.error(f"Checked paths: {possible_paths}")
                return {"success": False, "error": "Barebones genome template not found"}
                
            self.logger.info(f"Loading barebones genome from {barebones_path}")
                
            with open(barebones_path, 'r') as f:
                genome_data = json.load(f)
            
            # Set the genome file name
            if self.state_manager:
                self.state_manager.genome_file_name = "barebones_genome.json"
            
            # Call the existing load_genome method
            self._genome_filename = "barebones_genome.json"
            result = self.load_genome(genome_data, "barebones_genome.json")
            
            return result
            
        except Exception as e:
            self.logger.error(f"Failed to load barebones genome: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
            return {"success": False, "error": str(e)}

    def get_default_genomes(self) -> Dict[str, Any]:
        """
        Get a list of default genome files with their contents.
        
        Returns:
            Dictionary containing default genome files and their contents.
        """
        try:
            # Get the data path where default genomes are stored
            defaults_path = os.path.join(self.get_data_path(), "genome")
            
            # Check if the directory exists
            if not os.path.exists(defaults_path):
                self.logger.warning(f"Default genomes directory not found: {defaults_path}")
                return {}
            
            # Get all .json files in the directory
            default_genomes = {}
            
            for filename in os.listdir(defaults_path):
                if filename.endswith(".json"):
                    file_path = os.path.join(defaults_path, filename)
                    try:
                        with open(file_path, 'r') as f:
                            genome_data = json.load(f)
                            
                            # Store basic metadata about the genome
                            default_genomes[filename] = {
                                "title": genome_data.get("genome_title", "Untitled Genome"),
                                "description": genome_data.get("genome_description", ""),
                                "file_path": file_path
                            }
                    except Exception as e:
                        self.logger.error(f"Error loading default genome {filename}: {str(e)}")
            
            return default_genomes
                
        except Exception as e:
            self.logger.error(f"Error getting default genomes: {str(e)}")
            return {}

    def deploy_genome(self, genome_filepath: str) -> bool:
        """
        Deploy a genome from a file path. This is a more controlled way
        to load a genome with state management and validation.
        
        Args:
            genome_filepath: Path to the genome file to deploy
            
        Returns:
            True if deployment was successful, False otherwise
        """
        try:
            self.logger.info(f"Deploying genome from {genome_filepath}", emoji1="🧬")
            
            # Ensure the file exists
            if not os.path.exists(genome_filepath):
                self.logger.error(f"Genome file not found: {genome_filepath}", emoji1="❌")
                return False
                
            # Update state to LOADING
            if self.state_manager:
                self.state_manager.set_genome_state(GenomeState.LOADING)
                self.state_manager.set_brain_readiness(False)
                
            # Load the genome data
            with open(genome_filepath, 'r') as f:
                genome_data = json.load(f)
                
            # Extract the filename for reference
            filename = os.path.basename(genome_filepath)
                
            # Load the genome using the service
            result = self.load_genome(genome_data, filename=filename)
            
            if not result.get("success", False):
                self.logger.error(f"Failed to load genome: {result.get('error', 'Unknown error')}", emoji1="❌")
                
                # Update state to ERROR
                if self.state_manager:
                    self.state_manager.set_genome_state(GenomeState.ERROR)
                    self.state_manager.set_brain_readiness(False)
                    
                return False
                
            # Update state to LOADED - this is already done in load_genome but we do it again for safety
            if self.state_manager:
                self.state_manager.set_genome_state(GenomeState.LOADED)
                self.state_manager.set_brain_readiness(True)
                
            self.logger.info(f"Genome deployed successfully from {filename}", emoji1="✅")
            return True
            
        except json.JSONDecodeError:
            self.logger.error(f"Invalid JSON in genome file: {genome_filepath}", emoji1="❌")
            
            # Update state to ERROR
            if self.state_manager:
                self.state_manager.set_genome_state(GenomeState.ERROR)
                self.state_manager.set_brain_readiness(False)
                
            return False
        except Exception as e:
            self.logger.error(f"Error deploying genome: {str(e)}", emoji1="❌")
            
            # Update state to ERROR
            if self.state_manager:
                self.state_manager.set_genome_state(GenomeState.ERROR)
                self.state_manager.set_brain_readiness(False)
                
            return False

    def get_plasticity_queue_depth(self) -> int:
        """
        Get the current plasticity queue depth setting.
        
        Returns:
            Current plasticity queue depth value
        """
        try:
            if self._current_genome is None:
                self.logger.warning("No genome loaded, cannot retrieve plasticity queue depth")
                return 0
                
            return self._current_genome.get("physiology", {}).get("plasticity_queue_depth", 0)
        except Exception as e:
            self.logger.error(f"Error getting plasticity queue depth: {str(e)}")
            return 0
            
    def update_plasticity_queue_depth(self, depth: int) -> bool:
        """
        Update the plasticity queue depth setting.
        
        Args:
            depth: New plasticity queue depth value
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if self._current_genome is None:
                self.logger.warning("No genome loaded, cannot update plasticity queue depth")
                return False
                
            if "physiology" not in self._current_genome:
                self._current_genome["physiology"] = {}
                
            self._current_genome["physiology"]["plasticity_queue_depth"] = depth
            
            # Notify any listeners about genome changes
            if self.state_manager:
                self.state_manager.set_genome_state(GenomeState.LOADED)
                
            return True
        except Exception as e:
            self.logger.error(f"Error updating plasticity queue depth: {str(e)}")
            return False
            
    def update_plasticity_config(self, config: Dict[str, Any]) -> bool:
        """
        Update neuroplasticity configuration.
        
        Args:
            config: Dictionary with plasticity configuration parameters
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if self._current_genome is None:
                self.logger.warning("No genome loaded, cannot update plasticity configuration")
                return False
                
            # Ensure physiology section exists
            if "physiology" not in self._current_genome:
                self._current_genome["physiology"] = {}
                
            # Update plasticity settings
            plasticity_settings = self._current_genome["physiology"].get("plasticity", {})
            
            # Merge new configuration with existing settings
            for key, value in config.items():
                plasticity_settings[key] = value
                
            # Save back to genome
            self._current_genome["physiology"]["plasticity"] = plasticity_settings
            
            # Update connectome manager's plasticity settings if available
            if hasattr(self._connectome_manager, "update_plasticity_config"):
                self._connectome_manager.update_plasticity_config(plasticity_settings)
                
            # Notify any listeners about genome changes
            if self.state_manager:
                self.state_manager.set_genome_state(GenomeState.LOADED)
                
            return True
        except Exception as e:
            self.logger.error(f"Error updating plasticity configuration: {str(e)}")
            return False
            
    def enable_area_plasticity(self, cortical_id: str, settings: Optional[Dict[str, Any]] = None) -> bool:
        """
        Enable neuroplasticity for a specific cortical area.
        
        Args:
            cortical_id: ID of the cortical area
            settings: Optional area-specific plasticity settings
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # First verify the area exists
            if not self.get_cortical_area(cortical_id):
                self.logger.warning(f"Cortical area {cortical_id} not found")
                return False
                
            # Apply area-specific plasticity settings if available
            if settings and hasattr(self._connectome_manager, "set_area_plasticity"):
                return self._connectome_manager.set_area_plasticity(cortical_id, True, settings)
                
            # Otherwise just enable plasticity for this area
            if hasattr(self._connectome_manager, "enable_area_plasticity"):
                return self._connectome_manager.enable_area_plasticity(cortical_id)
                
            self.logger.warning("Connectome manager does not support area plasticity operations")
            return False
        except Exception as e:
            self.logger.error(f"Error enabling plasticity for area {cortical_id}: {str(e)}")
            return False
            
    def disable_area_plasticity(self, cortical_id: str) -> bool:
        """
        Disable neuroplasticity for a specific cortical area.
        
        Args:
            cortical_id: ID of the cortical area
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # First verify the area exists
            if not self.get_cortical_area(cortical_id):
                self.logger.warning(f"Cortical area {cortical_id} not found")
                return False
                
            # Disable plasticity for this area
            if hasattr(self._connectome_manager, "disable_area_plasticity"):
                return self._connectome_manager.disable_area_plasticity(cortical_id)
                
            # Alternative method name
            if hasattr(self._connectome_manager, "set_area_plasticity"):
                return self._connectome_manager.set_area_plasticity(cortical_id, False)
                
            self.logger.warning("Connectome manager does not support area plasticity operations")
            return False
        except Exception as e:
            self.logger.error(f"Error disabling plasticity for area {cortical_id}: {str(e)}")
            return False
    
    #----------------------------------------------------------------------
    # Evolution Methods
    #----------------------------------------------------------------------
    
    def get_generations(self) -> Dict[str, Any]:
        """
        Get details about all generations of genomes.
        
        Returns:
            Dictionary containing generation information
        """
        try:
            if not self.state_manager:
                return {}
                
            if hasattr(self.state_manager, 'generation_dict') and self.state_manager.generation_dict:
                return self.state_manager.generation_dict
            else:
                return {}
        except Exception as e:
            self.logger.error(f"Error retrieving generations: {str(e)}")
            return {}
    
    def get_change_register(self) -> Dict[str, Any]:
        """
        Get the evolution change register showing evolutionary history.
        
        Returns:
            Dictionary containing the evolution change register
        """
        try:
            if not self.state_manager:
                return {}
                
            if hasattr(self.state_manager, 'evo_change_register') and self.state_manager.evo_change_register:
                return self.state_manager.evo_change_register
            else:
                return {}
        except Exception as e:
            self.logger.error(f"Error retrieving evolution change register: {str(e)}")
            return {}
            
    def get_membrane_potential_monitoring_status(self, cortical_areas: List[str]) -> List[Tuple[str, bool]]:
        """
        Get the monitoring status of membrane potentials for specified cortical areas.
        
        Args:
            cortical_areas: List of cortical area IDs to check
            
        Returns:
            List of tuples containing (cortical_area, is_monitored)
        """
        try:
            if not self.state_manager:
                return [(area, False) for area in cortical_areas]
                
            # Check if monitoring scope attribute exists
            if not hasattr(self.state_manager, 'neuron_mp_collection_scope'):
                return [(area, False) for area in cortical_areas]
                
            # Check each cortical area
            result = []
            for area in cortical_areas:
                is_monitored = area in self.state_manager.neuron_mp_collection_scope
                result.append((area, is_monitored))
                
            return result
        except Exception as e:
            self.logger.error(f"Error retrieving membrane potential monitoring status: {str(e)}")
            return [(area, False) for area in cortical_areas]
    
    def set_membrane_potential_monitoring(self, cortical_areas: List[str], enabled: bool) -> bool:
        """
        Enable or disable membrane potential monitoring for specified cortical areas.
        
        Args:
            cortical_areas: List of cortical area IDs to update
            enabled: True to enable monitoring, False to disable
            
        Returns:
            True if successful, False otherwise
            
        Raises:
            ValueError: If InfluxDB service is not running
        """
        try:
            if not self.state_manager:
                return False
                
            # Check for InfluxDB service
            influxdb = self.state_manager.get_influxdb()
            if not influxdb:
                raise ValueError("InfluxDB service is not running")
                
            # Test InfluxDB connection
            influx_ready = influxdb.test_influxdb()
            if not influx_ready:
                raise ValueError("InfluxDB service is not responding")
                
            # Ensure neuron_mp_collection_scope attribute exists
            if not hasattr(self.state_manager, 'neuron_mp_collection_scope'):
                self.state_manager.neuron_mp_collection_scope = {}
                
            # Update monitoring status for each area
            for area in cortical_areas:
                # Verify the area exists in the genome
                if not self._current_genome or 'blueprint' not in self._current_genome:
                    self.logger.warning(f"Cannot verify cortical area {area} - no genome loaded")
                    continue
                    
                if area not in self._current_genome['blueprint']:
                    self.logger.warning(f"Cortical area {area} not found in genome blueprint")
                    continue
                    
                # Update the monitoring scope
                if enabled and area not in self.state_manager.neuron_mp_collection_scope:
                    self.state_manager.neuron_mp_collection_scope[area] = {}
                elif not enabled and area in self.state_manager.neuron_mp_collection_scope:
                    self.state_manager.neuron_mp_collection_scope.pop(area)
                    
            return True
        except Exception as e:
            self.logger.error(f"Error setting membrane potential monitoring: {str(e)}")
            if isinstance(e, ValueError):
                raise
            return False
    
    def get_synaptic_potential_monitoring_status(self, cortical_areas: List[str]) -> List[Tuple[str, bool]]:
        """
        Get the monitoring status of synaptic potentials for specified cortical areas.
        
        Args:
            cortical_areas: List of cortical area IDs to check
            
        Returns:
            List of tuples containing (cortical_area, is_monitored)
        """
        try:
            if not self.state_manager:
                return [(area, False) for area in cortical_areas]
                
            # Check if monitoring scope attribute exists
            if not hasattr(self.state_manager, 'neuron_psp_collection_scope'):
                return [(area, False) for area in cortical_areas]
                
            # Check each cortical area
            result = []
            for area in cortical_areas:
                is_monitored = area in self.state_manager.neuron_psp_collection_scope
                result.append((area, is_monitored))
                
            return result
        except Exception as e:
            self.logger.error(f"Error retrieving synaptic potential monitoring status: {str(e)}")
            return [(area, False) for area in cortical_areas]
    
    def set_synaptic_potential_monitoring(self, cortical_areas: List[str], enabled: bool) -> bool:
        """
        Enable or disable synaptic potential monitoring for specified cortical areas.
        
        Args:
            cortical_areas: List of cortical area IDs to update
            enabled: True to enable monitoring, False to disable
            
        Returns:
            True if successful, False otherwise
            
        Raises:
            ValueError: If InfluxDB service is not running
        """
        try:
            if not self.state_manager:
                return False
                
            # Check for InfluxDB service
            influxdb = self.state_manager.get_influxdb()
            if not influxdb:
                raise ValueError("InfluxDB service is not running")
                
            # Test InfluxDB connection
            influx_ready = influxdb.test_influxdb()
            if not influx_ready:
                raise ValueError("InfluxDB service is not responding")
                
            # Ensure neuron_psp_collection_scope attribute exists
            if not hasattr(self.state_manager, 'neuron_psp_collection_scope'):
                self.state_manager.neuron_psp_collection_scope = {}
                
            # Update monitoring status for each area
            for area in cortical_areas:
                # Verify the area exists in the genome
                if not self._current_genome or 'blueprint' not in self._current_genome:
                    self.logger.warning(f"Cannot verify cortical area {area} - no genome loaded")
                    continue
                    
                if area not in self._current_genome['blueprint']:
                    self.logger.warning(f"Cortical area {area} not found in genome blueprint")
                    continue
                    
                # Update the monitoring scope
                if enabled and area not in self.state_manager.neuron_psp_collection_scope:
                    self.state_manager.neuron_psp_collection_scope[area] = {}
                elif not enabled and area in self.state_manager.neuron_psp_collection_scope:
                    self.state_manager.neuron_psp_collection_scope.pop(area)
                    
            return True
        except Exception as e:
            self.logger.error(f"Error setting synaptic potential monitoring: {str(e)}")
            if isinstance(e, ValueError):
                raise
            return False
            
    #----------------------------------------------------------------------
    # Agent Management Methods
    #----------------------------------------------------------------------
    
    def get_agent_list(self) -> Set[str]:
        """
        Get list of registered agents.
        
        Returns:
            Set of agent IDs that are currently registered
        """
        try:
            if not self.state_manager:
                return set()
                
            if hasattr(self.state_manager, 'agent_registry') and self.state_manager.agent_registry:
                return set(self.state_manager.agent_registry.keys())
            else:
                return set()
        except Exception as e:
            self.logger.error(f"Error retrieving agent list: {str(e)}")
            return set()
    
    def get_agent_properties(self, agent_id: str) -> Dict[str, Any]:
        """
        Get properties of a specific agent.
        
        Args:
            agent_id: ID of the agent to query
            
        Returns:
            Dictionary with agent properties
            
        Raises:
            ValueError: If agent is not found
        """
        try:
            if not self.state_manager or not hasattr(self.state_manager, 'agent_registry'):
                raise ValueError("Agent registry not available")
                
            if agent_id not in self.state_manager.agent_registry:
                raise ValueError(f"Agent {agent_id} not found")
                
            agent_info = {}
            agent_registry = self.state_manager.agent_registry[agent_id]
            
            # Copy relevant properties
            agent_info["agent_type"] = agent_registry.get("agent_type", "unknown")
            agent_info["agent_ip"] = agent_registry.get("agent_ip", "")
            agent_info["agent_data_port"] = agent_registry.get("agent_data_port", 0)
            agent_info["agent_router_address"] = agent_registry.get("agent_router_address", "")
            agent_info["agent_version"] = agent_registry.get("agent_version", "")
            agent_info["controller_version"] = agent_registry.get("controller_version", "")
            agent_info["capabilities"] = agent_registry.get("capabilities", {})
            
            return agent_info
        except Exception as e:
            self.logger.error(f"Error retrieving agent properties: {str(e)}")
            if isinstance(e, ValueError):
                raise
            return {}
    
    def _assign_available_port(self) -> Optional[int]:
        """
        Helper method to assign an available port for a new agent.
        
        Returns:
            Available port number, or None if no ports are available
        """
        try:
            if not self.state_manager or not hasattr(self.state_manager, 'agent_registry'):
                return None
                
            ports_used = []
            port_ranges = (40001, 40050)
            
            # Collect already used ports
            for agent_id, agent_info in self.state_manager.agent_registry.items():
                if agent_info.get('agent_type') != 'monitor':
                    ports_used.append(agent_info.get('agent_data_port', 0))
                    
            # Find first available port
            for port in range(port_ranges[0], port_ranges[1]):
                if port not in ports_used:
                    return port
                    
            return None
        except Exception as e:
            self.logger.error(f"Error assigning available port: {str(e)}")
            return None
    
    def register_agent(
        self,
        agent_id: str,
        agent_type: str,
        agent_ip: str,
        agent_data_port: int,
        agent_version: str,
        controller_version: str,
        capabilities: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Register a new agent.
        
        Args:
            agent_id: Unique identifier for the agent
            agent_type: Type of agent (e.g., 'monitor', 'robot')
            agent_ip: IP address of the agent
            agent_data_port: Port for agent data communication
            agent_version: Version of the agent
            controller_version: Version of the controller
            capabilities: Optional dictionary of agent capabilities
            
        Returns:
            Dictionary with agent information including assigned port and router address
            
        Raises:
            ValueError: If agent registration fails
        """
        try:
            if not self.state_manager:
                raise ValueError("State manager not available")
                
            # Ensure agent_registry exists
            if not hasattr(self.state_manager, 'agent_registry'):
                self.state_manager.agent_registry = {}
                
            if not hasattr(self.state_manager, 'host_info'):
                self.state_manager.host_info = {}
                
            if not capabilities:
                capabilities = {}
                
            # Prepare agent info
            agent_info = {
                "agent_id": agent_id,
                "agent_type": agent_type,
                "agent_ip": agent_ip,
                "agent_version": agent_version, 
                "controller_version": controller_version,
                "capabilities": capabilities
            }
            
            # Set up different router addresses based on agent type
            if agent_type == 'monitor':
                agent_router_address = f"tcp://{agent_ip}:{agent_data_port}"
                self.state_manager.brain_activity_pub = True
                self.logger.info("Publication of brain activity turned on!")
            else:
                # Assign a port from available range
                assigned_port = self._assign_available_port()
                if not assigned_port:
                    raise ValueError("No available ports for agent registration")
                agent_data_port = assigned_port
                agent_router_address = f"tcp://*:{agent_data_port}"
                
            # Add port and router address to agent info
            agent_info["agent_data_port"] = agent_data_port
            agent_info["agent_router_address"] = agent_router_address
            
            # Register the agent
            self.state_manager.agent_registry[agent_id] = agent_info
            self.state_manager.host_info[agent_id] = agent_info
            
            # Add default listener field
            if "listener" not in agent_info:
                agent_info["listener"] = None
            
            # If auto PNS area creation is enabled and genome is loaded, update PNS areas
            if (hasattr(self.state_manager, 'auto_pns_area_creation') and 
                self.state_manager.auto_pns_area_creation and self._current_genome):
                # Submit message to API queue
                if hasattr(self._connectome_manager, 'api_message_queue'):
                    message = {'update_pns_areas': capabilities}
                    self._connectome_manager.api_message_queue.put(item=message)
            
            # Update evolution change register
            if hasattr(self.state_manager, 'evo_change_register'):
                self.state_manager.evo_change_register["agent"] += 1
            
            # Return a copy of agent info without the listener field
            result = agent_info.copy()
            if "listener" in result:
                result.pop("listener")
                
            self.logger.info(f"Agent {agent_id} successfully registered")
            return result
        except Exception as e:
            self.logger.error(f"Error registering agent: {str(e)}")
            if isinstance(e, ValueError):
                raise
            return {}
    
    def deregister_agent(self, agent_id: str) -> bool:
        """
        Remove a registered agent.
        
        Args:
            agent_id: ID of the agent to remove
            
        Returns:
            True if successful, False otherwise
            
        Raises:
            ValueError: If agent is not found
        """
        try:
            if not self.state_manager or not hasattr(self.state_manager, 'agent_registry'):
                raise ValueError("Agent registry not available")
                
            if agent_id not in self.state_manager.agent_registry:
                raise ValueError(f"Agent {agent_id} not found")
                
            # Remove the agent from registry
            agent_info = self.state_manager.agent_registry.pop(agent_id)
            
            # Also remove from host info if present
            if hasattr(self.state_manager, 'host_info') and agent_id in self.state_manager.host_info:
                self.state_manager.host_info.pop(agent_id)
                
            self.logger.info(f"Agent {agent_id} successfully deregistered")
            return True
        except Exception as e:
            self.logger.error(f"Error deregistering agent: {str(e)}")
            if isinstance(e, ValueError):
                raise
            return False
    
    def update_robot_controller(self, controller_params: Dict[str, Any]) -> bool:
        """
        Update robot controller parameters.
        
        Args:
            controller_params: Dictionary containing robot controller parameters
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if not self._connectome_manager:
                return False
                
            # Submit parameters to API queue
            if hasattr(self._connectome_manager, 'api_message_queue'):
                message = {'robot_controller': controller_params}
                self._connectome_manager.api_message_queue.put(item=message)
                return True
            else:
                self.logger.error("API message queue not available")
                return False
        except Exception as e:
            self.logger.error(f"Error updating robot controller: {str(e)}")
            return False
    
    def update_robot_model(self, model_params: Dict[str, Any]) -> bool:
        """
        Update robot model parameters.
        
        Args:
            model_params: Dictionary containing robot model parameters
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if not self._connectome_manager:
                return False
                
            # Submit parameters to API queue
            if hasattr(self._connectome_manager, 'api_message_queue'):
                message = {'robot_model': model_params}
                self._connectome_manager.api_message_queue.put(item=message)
                return True
            else:
                self.logger.error("API message queue not available")
                return False
        except Exception as e:
            self.logger.error(f"Error updating robot model: {str(e)}")
            return False
    
    def get_gazebo_robot_files(self) -> Dict[str, List[str]]:
        """
        Get list of available Gazebo robot files.
        
        Returns:
            Dictionary containing list of available robot files
        """
        try:
            import os
            
            # Get the default robots path
            default_robots_path = os.path.join(self.get_data_path(), "robot")
            if not os.path.exists(default_robots_path):
                # Try legacy path
                default_robots_path = "./evo/defaults/robot/"
                if not os.path.exists(default_robots_path):
                    self.logger.warning(f"Robot files directory not found at {default_robots_path}")
                    return {"robots": []}
            
            # List available robot files
            default_robots = os.listdir(default_robots_path)
            return {"robots": default_robots}
        except Exception as e:
            self.logger.error(f"Error retrieving Gazebo robot files: {str(e)}")
            return {"robots": []}
    
    def trigger_manual_stimulation(self, stimulation_payload: Dict[str, Any]) -> bool:
        """
        Trigger a manual stimulation.
        
        Args:
            stimulation_payload: Dictionary containing stimulation parameters
            
        Returns:
            True if successful, False otherwise
            
        Raises:
            ValueError: If connectome is not ready
        """
        try:
            if not self.state_manager or not self.state_manager.is_connectome_ready():
                raise ValueError("Connectome is not ready")
                
            # Submit stimulation to API queue
            if hasattr(self._connectome_manager, 'api_message_queue'):
                message = {'manual_stimulation': stimulation_payload}
                self._connectome_manager.api_message_queue.put(item=message)
                return True
            else:
                self.logger.error("API message queue not available")
                return False
        except Exception as e:
            self.logger.error(f"Error triggering manual stimulation: {str(e)}")
            if isinstance(e, ValueError):
                raise
            return False
    
    def trigger_sustained_stimulation(self, stimulation_payload: Dict[str, Any]) -> bool:
        """
        Trigger a sustained stimulation.
        
        Args:
            stimulation_payload: Dictionary containing stimulation parameters
            
        Returns:
            True if successful, False otherwise
            
        Raises:
            ValueError: If connectome is not ready
        """
        try:
            if not self.state_manager or not self.state_manager.is_connectome_ready():
                raise ValueError("Connectome is not ready")
                
            # Submit stimulation to API queue
            if hasattr(self._connectome_manager, 'api_message_queue'):
                message = {'sustained_stimulation': stimulation_payload}
                self._connectome_manager.api_message_queue.put(item=message)
                return True
            else:
                self.logger.error("API message queue not available")
                return False
        except Exception as e:
            self.logger.error(f"Error triggering sustained stimulation: {str(e)}")
            if isinstance(e, ValueError):
                raise
            return False

    def _get_cortical_id_for_idx(self, cortical_idx: int) -> str:
        """
        Get the 6-character cortical_id string for a given cortical_idx integer.
        
        Args:
            cortical_idx: Integer cortical area identifier
            
        Returns:
            String cortical_id corresponding to this cortical_idx, 
            or a generated placeholder if not found
        """
        # Look up the cortical area in the connectome manager
        area = self._connectome_manager.cortical_areas.get(cortical_idx)
        
        if area and hasattr(area, 'cortical_id') and area.cortical_id:
            # Return the proper cortical_id
            return area.cortical_id
        
        # Generate a temporary placeholder identifier if not found
        # Format: "CID###" where ### is the numeric cortical_idx
        return f"CID{cortical_idx:03d}"
        
    def get_state_manager(self):
        """
        Get the state manager instance.
        
        Returns:
            The state manager instance.
        """
        return self.state_manager

    #----------------------------------------------------------------------
    # Network Management Methods
    #----------------------------------------------------------------------
    
    def get_network_config(self) -> Dict[str, Any]:
        """
        Get the current network configuration.
        
        Returns:
            Dictionary containing network configuration settings
        """
        try:
            if self.state_manager:
                sockets = self.state_manager.parameters.get('Sockets', {})
                if sockets:
                    return sockets
                    
            # Return a minimal default configuration if no configuration is found
            return {}
        except Exception as e:
            self.logger.error(f"Error retrieving network configuration: {str(e)}")
            return {}
            
    def update_network_config(self, network_config: Dict[str, Any]) -> bool:
        """
        Update the network configuration.
        
        Args:
            network_config: Dictionary containing network configuration to update
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if not self.state_manager:
                return False
                
            if 'parameters' not in self.state_manager.__dict__ or not isinstance(self.state_manager.parameters, dict):
                self.state_manager.parameters = {}
                
            if 'Sockets' not in self.state_manager.parameters:
                self.state_manager.parameters['Sockets'] = {}
                
            # Update the existing configuration
            self.state_manager.parameters['Sockets'].update(network_config)
            
            # Submit to message queue for any component that needs to react to changes
            if hasattr(self._connectome_manager, 'api_message_queue'):
                message = {'network_management': network_config}
                self._connectome_manager.api_message_queue.put(item=message)
                
            return True
        except Exception as e:
            self.logger.error(f"Error updating network configuration: {str(e)}")
            return False
    
    #----------------------------------------------------------------------
    # Stimulation Methods
    #----------------------------------------------------------------------
    
    def set_stimulation_script(self, script: str) -> bool:
        """
        Set a stimulation script to be used during simulation.
        
        Args:
            script: Stimulation script in the required format
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if not self.state_manager:
                return False
                
            # Store the script in the state manager
            self.state_manager.stimulation_script = script
            
            return True
        except Exception as e:
            self.logger.error(f"Error setting stimulation script: {str(e)}")
            return False
            
    def reset_stimulation_script(self) -> bool:
        """
        Reset all stimulation scripts.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            if not self._connectome_manager:
                return False
                
            # Submit to message queue for any component that needs to react
            if hasattr(self._connectome_manager, 'api_message_queue'):
                message = {'stimulation_script': {}}
                self._connectome_manager.api_message_queue.put(item=message)
                
            return True
        except Exception as e:
            self.logger.error(f"Error resetting stimulation script: {str(e)}")
            return False
    
    #----------------------------------------------------------------------
    # System Health & Monitoring Methods
    #----------------------------------------------------------------------
    
    def has_pending_amalgamation(self) -> bool:
        """
        Check if there is a pending amalgamation operation.
        
        Returns:
            True if there is a pending amalgamation, False otherwise
        """
        try:
            if not self.state_manager:
                return False
                
            return bool(getattr(self.state_manager, 'pending_amalgamation', False))
        except Exception as e:
            self.logger.error(f"Error checking pending amalgamation: {str(e)}")
            return False
    
    async def get_system_health(self) -> Dict[str, Any]:
        """
        Get comprehensive system health information.
        
        Returns:
            Dictionary containing health metrics for all FEAGI components
        """
        health = {}
        try:
            if not self.state_manager:
                return {"error": "State manager not available"}
                
            # Basic health metrics
            health["burst_engine"] = not getattr(self.state_manager, 'exit_condition', False)
            health["connected_agents"] = getattr(self.state_manager, 'connected_agents', None)
            health["influxdb_availability"] = bool(getattr(self.state_manager, 'influxdb', False))
            
            # Resource limits
            limits = getattr(self.state_manager, 'parameters', {}).get("Limits", {})
            health["neuron_count_max"] = int(limits.get("max_neuron_count", 0))
            health["synapse_count_max"] = int(limits.get("max_synapse_count", 0))
            
            # Genome-related information
            health["latest_changes_saved_externally"] = getattr(self.state_manager, 'changes_saved_externally', False)
            
            # Use the proper state manager method to check if genome is loaded
            if self.state_manager.is_genome_loaded():
                health["fitness"] = getattr(self.state_manager, 'genome_fitness', None)
                health["genome_availability"] = True
                
                # Brain statistics
                brain_stats = getattr(self.state_manager, 'brain_stats', {})
                connectome_neuron_count = brain_stats.get("neuron_count", 0)
                connectome_synapse_count = brain_stats.get("synapse_count", 0)
                
                # Estimate brain size in MB using a formula
                connectome_size = 3E-08 * connectome_neuron_count ** 2 + 0.0011 * connectome_neuron_count + 2.9073
                
                health["cortical_area_count"] = len(getattr(self.state_manager, 'cortical_list', []))
                health["neuron_count"] = connectome_neuron_count
                health["synapse_count"] = connectome_synapse_count
                health["estimated_brain_size_in_MB"] = connectome_size
            else:
                health["genome_availability"] = False
                
            health["genome_validity"] = getattr(self.state_manager, 'genome_validity', None)
            health["brain_readiness"] = self.state_manager.get_brain_readiness()
            
            # Check for pending amalgamation
            if self.has_pending_amalgamation():
                pending = getattr(self.state_manager, 'pending_amalgamation', {})
                health["amalgamation_pending"] = {
                    "initiation_time": pending.get("initiation_time", None),
                    "genome_id": pending.get("genome_id", None),
                    "amalgamation_id": pending.get("amalgamation_id", None),
                    "genome_title": pending.get("genome_title", None),
                    "circuit_size": pending.get("circuit_size", None)
                }
                
            return health
        except Exception as e:
            self.logger.error(f"Error retrieving system health: {str(e)}")
            return {"error": str(e)}
    
    def get_user_preferences(self) -> Dict[str, Any]:
        """
        Get user preferences.
        
        Returns:
            Dictionary with user preferences
        """
        try:
            if self.state_manager and hasattr(self.state_manager, 'user_preferences'):
                return self.state_manager.user_preferences
            
            # Default preferences
            return {
                "adv_mode": False,
                "ui_magnification": 1.0,
                "auto_pns_area_creation": True
            }
        except Exception as e:
            self.logger.error(f"Error getting user preferences: {str(e)}")
            return {}
    
    def update_user_preferences(self, preferences: Dict[str, Any]) -> bool:
        """
        Update user preferences.
        
        Args:
            preferences: Dictionary with preference updates
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if self.state_manager:
                if not hasattr(self.state_manager, 'user_preferences'):
                    self.state_manager.user_preferences = {}
                self.state_manager.user_preferences.update(preferences)
            
            return True
        except Exception as e:
            self.logger.error(f"Error updating user preferences: {str(e)}")
            return False
    
    def get_versions(self) -> Dict[str, Any]:
        """
        Get version information for various components.
        
        Returns:
            Dictionary with version information
        """
        try:
            from feagi.version import __version__
            
            versions = {
                "feagi_core": __version__,
                "python": f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}",
                "timestamp": datetime.now().isoformat()
            }
            
            # Add additional component versions if available
            try:
                import numpy
                versions["numpy"] = numpy.__version__
            except ImportError:
                pass
                
            try:
                import torch
                versions["torch"] = torch.__version__
            except ImportError:
                pass
                
            return versions
        except Exception as e:
            self.logger.error(f"Error getting versions: {str(e)}")
            return {}
    
    def test_influxdb(self) -> Optional[Dict[str, Any]]:
        """
        Test InfluxDB connectivity.
        
        Returns:
            Dictionary with InfluxDB status, or None if not available
        """
        try:
            # Check if InfluxDB configuration exists
            if self.state_manager and hasattr(self.state_manager, 'influxdb_config'):
                return {
                    "status": "connected",
                    "database": self.state_manager.influxdb_config.get('database', 'feagi'),
                    "host": self.state_manager.influxdb_config.get('host', 'localhost'),
                    "port": self.state_manager.influxdb_config.get('port', 8086)
                }
            
            # InfluxDB not configured
            return None
        except Exception as e:
            self.logger.error(f"Error testing InfluxDB: {str(e)}")
            return None
    
    def set_circuit_library_path(self, path: str) -> bool:
        """
        Set the circuit library path.
        
        Args:
            path: Path to circuit library
            
        Returns:
            True if successful, False otherwise
        """
        try:
            import os
            
            if not os.path.exists(path):
                raise ValueError(f"Path does not exist: {path}")
            
            if not os.path.isdir(path):
                raise ValueError(f"Path is not a directory: {path}")
            
            if self.state_manager:
                self.state_manager.circuit_library_path = path
            
            return True
        except Exception as e:
            self.logger.error(f"Error setting circuit library path: {str(e)}")
            return False
    
    def reset_fcl(self) -> bool:
        """
        Reset the Fire Candidate List.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            if hasattr(self._connectome_manager, 'reset_fcl'):
                self._connectome_manager.reset_fcl()
                return True
            else:
                self.logger.warning("Connectome manager does not support FCL reset")
                return False
        except Exception as e:
            self.logger.error(f"Error resetting FCL: {str(e)}")
            return False

    #----------------------------------------------------------------------
    # Helper Methods for Missing Functionality
    #----------------------------------------------------------------------
    
    def get_state_manager(self):
        """Get the state manager instance."""
        return self.state_manager

    #----------------------------------------------------------------------
    # Network Management Methods
    #----------------------------------------------------------------------
    
    def get_neuron_mappings(self) -> Dict[str, Any]:
        """
        Get neuron mappings for connectome visualization.
        
        Returns:
            Dictionary with neuron mapping information
        """
        try:
            if not self._current_genome or 'blueprint' not in self._current_genome:
                return {}
                
            mappings = {}
            
            # Extract mappings from blueprint destination maps
            for blueprint_key, area_data in self._current_genome['blueprint'].items():
                if isinstance(area_data, dict):
                    dstmap = area_data.get('dstmap', {})
                    if dstmap:
                        # Extract cortical_id from blueprint key
                        parts = blueprint_key.split('-')
                        if len(parts) >= 2:
                            source_id = parts[1]
                            mappings[source_id] = list(dstmap.keys())
            
            return mappings
        except Exception as e:
            self.logger.error(f"Error getting neuron mappings: {str(e)}")
            return {}
    
    def get_transforming_areas(self) -> List[str]:
        """
        Get list of cortical areas that are currently transforming.
        
        Returns:
            List of cortical area IDs that are transforming
        """
        try:
            transforming_areas = []
            if not self._current_genome or 'blueprint' not in self._current_genome:
                return transforming_areas
                
            # Group cortical areas by cortical_id to check their transforming property
            cortical_data = {}
            for blueprint_key, area_data in self._current_genome['blueprint'].items():
                parts = blueprint_key.split('-')
                if len(parts) >= 4:
                    cortical_id = parts[1]
                    property_name = parts[3]
                    
                    if cortical_id not in cortical_data:
                        cortical_data[cortical_id] = {}
                    cortical_data[cortical_id][property_name] = area_data
            
            # Check each cortical area's transforming property
            for cortical_id, properties in cortical_data.items():
                if properties.get('transforming', False):
                    transforming_areas.append(cortical_id)
            
            return transforming_areas
        except Exception as e:
            self.logger.error(f"Error getting transforming areas: {str(e)}")
            return []
    
    def get_plasticity_info(self) -> Dict[str, Any]:
        """
        Get neuroplasticity information.
        
        Returns:
            Dictionary with plasticity information
        """
        try:
            if not self._current_genome:
                return {}
                
            physiology = self._current_genome.get('physiology', {})
            plasticity = physiology.get('plasticity', {})
            
            return {
                "plasticity_enabled": plasticity.get('enabled', False),
                "plasticity_queue_depth": physiology.get('plasticity_queue_depth', 0),
                "ltp_enabled": plasticity.get('ltp_enabled', False),
                "ltd_enabled": plasticity.get('ltd_enabled', False),
                "global_plasticity_rate": plasticity.get('global_rate', 0.01)
            }
        except Exception as e:
            self.logger.error(f"Error getting plasticity info: {str(e)}")
            return {}
    
    def get_temp_path(self) -> str:
        """
        Get temporary directory path.
        
        Returns:
            Path to temporary directory
        """
        return self._temp_dir
    
    def save_connectome_snapshot(self, path: str) -> bool:
        """
        Save a snapshot of the current connectome.
        
        Args:
            path: Path to save the snapshot
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if not hasattr(self._connectome_manager, 'save'):
                self.logger.warning("Connectome manager does not support saving")
                return False
                
            self._connectome_manager.save(path)
            return True
        except Exception as e:
            self.logger.error(f"Error saving connectome snapshot: {str(e)}")
            return False
    
    def import_cortical_area(self, cortical_area_data: Dict[str, Any]) -> bool:
        """
        Import a cortical area from data.
        
        Args:
            cortical_area_data: Cortical area data to import
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Extract required fields
            name = cortical_area_data.get('name', 'Imported Area')
            dimensions = tuple(cortical_area_data.get('dimensions', (10, 10, 1)))
            position = tuple(cortical_area_data.get('position', (0, 0, 0)))
            area_type = cortical_area_data.get('area_type', 'custom')
            properties = cortical_area_data.get('properties', {})
            
            # Create the cortical area
            cortical_id = self._connectome_manager.add_cortical_area(
                name=name,
                dimensions=dimensions,
                position=position,
                area_type=area_type,
                properties=properties
            )
            
            return cortical_id is not None
        except Exception as e:
            self.logger.error(f"Error importing cortical area: {str(e)}")
            return False
    
    def get_cortical_area_stats(self, cortical_area: str) -> Optional[Dict[str, Any]]:
        """
        Get statistics for a cortical area.
        
        Args:
            cortical_area: ID of the cortical area
            
        Returns:
            Dictionary with area statistics, or None if not found
        """
        try:
            # Try to find the area
            area = None
            cortical_idx = None
            
            if hasattr(self._connectome_manager, 'cortical_areas'):
                for idx, area_obj in self._connectome_manager.cortical_areas.items():
                    if hasattr(area_obj, 'cortical_id') and area_obj.cortical_id == cortical_area:
                        area = area_obj
                        cortical_idx = idx
                        break
                    elif str(idx) == cortical_area:
                        area = area_obj
                        cortical_idx = idx
                        break
            
            if not area:
                return None
                
            # Get neuron count
            neuron_count = len(self._connectome_manager.get_neurons_by_area(cortical_idx))
            
            # Get basic statistics
            return {
                "cortical_area": cortical_area,
                "neuron_count": neuron_count,
                "dimensions": area.dimensions,
                "position": area.position,
                "area_type": area.area_type,
                "volume": area.volume if hasattr(area, 'volume') else area.dimensions[0] * area.dimensions[1] * area.dimensions[2]
            }
        except Exception as e:
            self.logger.error(f"Error getting cortical area stats: {str(e)}")
            return None
    
    def batch_create_neurons(self, area_id: str, positions: List[Tuple[int, int, int]], 
                           properties: Optional[Dict[str, Any]] = None) -> List[int]:
        """
        Create multiple neurons in batch.
        
        Args:
            area_id: ID of the cortical area
            positions: List of 3D positions for neurons
            properties: Optional properties for all neurons
            
        Returns:
            List of created neuron IDs
        """
        try:
            neuron_ids = []
            
            # Default properties
            default_props = {
                'threshold': 1.0,
                'membrane_potential': 0.0,
                'resting_potential': 0.0,
                'decay_rate': 0.5,
                'refractory_period': 1
            }
            if properties:
                default_props.update(properties)
            
            # Create neurons at each position
            for position in positions:
                try:
                    neuron_id = self._connectome_manager.create_neuron(
                        cortical_id=area_id,
                        position=position,
                        **default_props
                    )
                    neuron_ids.append(neuron_id)
                except Exception as e:
                    self.logger.warning(f"Failed to create neuron at position {position}: {str(e)}")
                    continue
            
            return neuron_ids
        except Exception as e:
            self.logger.error(f"Error in batch neuron creation: {str(e)}")
            return []
    
    def batch_create_synapses(self, connections: List[Tuple[int, int, float]]) -> int:
        """
        Create multiple synapses in batch.
        
        Args:
            connections: List of (pre_neuron_id, post_neuron_id, weight) tuples
            
        Returns:
            Number of successfully created synapses
        """
        try:
            return self._connectome_manager.batch_create_synapses(connections)
        except Exception as e:
            self.logger.error(f"Error in batch synapse creation: {str(e)}")
            return 0
    
    def get_configuration(self) -> Dict[str, Any]:
        """
        Get system configuration.
        
        Returns:
            Dictionary with configuration parameters
        """
        try:
            if self.state_manager and hasattr(self.state_manager, 'parameters'):
                return self.state_manager.parameters
            return {}
        except Exception as e:
            self.logger.error(f"Error getting configuration: {str(e)}")
            return {}

    #----------------------------------------------------------------------
    # Burst Engine Methods
    #----------------------------------------------------------------------
    
    def get_burst_counter(self) -> int:
        """
        Get the current burst counter.
        
        Returns:
            Current burst counter value
        """
        try:
            if self.state_manager:
                return getattr(self.state_manager, 'burst_count', 0)
            return 0
        except Exception as e:
            self.logger.error(f"Error getting burst counter: {str(e)}")
            return 0
    
    def get_fcl_sampler_config(self) -> Dict[str, Any]:
        """
        Get FCL sampler configuration.
        
        Returns:
            Dictionary with FCL sampler configuration
        """
        try:
            # Default configuration
            return {
                "frequency": 60.0,  # Hz
                "consumer": "Visualization"  # Default consumer
            }
        except Exception as e:
            self.logger.error(f"Error getting FCL sampler config: {str(e)}")
            return {"frequency": 60.0, "consumer": "Visualization"}
    
    def update_fcl_sampler_config(self, frequency: float, consumer: str) -> bool:
        """
        Update FCL sampler configuration.
        
        Args:
            frequency: Sampling frequency in Hz
            consumer: Consumer type (Visualization, Motor, etc.)
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Store in state manager if available
            if self.state_manager:
                if not hasattr(self.state_manager, 'fcl_sampler_config'):
                    self.state_manager.fcl_sampler_config = {}
                self.state_manager.fcl_sampler_config['frequency'] = frequency
                self.state_manager.fcl_sampler_config['consumer'] = consumer
            
            return True
        except Exception as e:
            self.logger.error(f"Error updating FCL sampler config: {str(e)}")
            return False
    
    def get_area_fcl_sample_rate(self, area_id: int) -> float:
        """
        Get FCL sample rate for a specific area.
        
        Args:
            area_id: Cortical area ID
            
        Returns:
            Sample rate in Hz
        """
        try:
            if self.state_manager and hasattr(self.state_manager, 'area_fcl_rates'):
                return self.state_manager.area_fcl_rates.get(area_id, 60.0)
            return 60.0  # Default rate
        except Exception as e:
            self.logger.error(f"Error getting area FCL sample rate: {str(e)}")
            return 60.0
    
    def set_area_fcl_sample_rate(self, area_id: int, sample_rate: float) -> bool:
        """
        Set FCL sample rate for a specific area.
        
        Args:
            area_id: Cortical area ID
            sample_rate: Sample rate in Hz
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if self.state_manager:
                if not hasattr(self.state_manager, 'area_fcl_rates'):
                    self.state_manager.area_fcl_rates = {}
                self.state_manager.area_fcl_rates[area_id] = sample_rate
            
            return True
        except Exception as e:
            self.logger.error(f"Error setting area FCL sample rate: {str(e)}")
            return False
    
    def get_membrane_potentials(self, neuron_ids: List[int]) -> Dict[int, float]:
        """
        Get membrane potentials for specific neurons.
        
        Args:
            neuron_ids: List of neuron IDs
            
        Returns:
            Dictionary mapping neuron IDs to membrane potentials
        """
        try:
            potentials = {}
            for neuron_id in neuron_ids:
                try:
                    potential = self._connectome_manager.get_neuron_property(neuron_id, 'membrane_potential')
                    potentials[neuron_id] = potential
                except KeyError:
                    # Neuron doesn't exist
                    continue
                except Exception:
                    # Other error, use default
                    potentials[neuron_id] = 0.0
            
            return potentials
        except Exception as e:
            self.logger.error(f"Error getting membrane potentials: {str(e)}")
            return {}
    
    def update_membrane_potentials(self, potentials: Dict[int, float]) -> bool:
        """
        Update membrane potentials for specific neurons.
        
        Args:
            potentials: Dictionary mapping neuron IDs to new potentials
            
        Returns:
            True if successful, False otherwise
        """
        try:
            for neuron_id, potential in potentials.items():
                try:
                    self._connectome_manager.set_neuron_property(neuron_id, 'membrane_potential', potential)
                except KeyError:
                    # Neuron doesn't exist, skip
                    continue
            
            return True
        except Exception as e:
            self.logger.error(f"Error updating membrane potentials: {str(e)}")
            return False

    #----------------------------------------------------------------------
    # System Visualization Methods  
    #----------------------------------------------------------------------
    
    def get_cortical_area_types(self) -> Dict[str, Any]:
        """
        Get available cortical area types.
        
        Returns:
            Dictionary with cortical area types
        """
        try:
            # Import from FEAGI templates
            from feagi.evo.templates import cortical_types
            return cortical_types
        except ImportError:
            # Fallback types
            return {
                "MEMORY": {"description": "Memory cortical areas"},
                "IPU": {"description": "Input Processing Units"}, 
                "OPU": {"description": "Output Processing Units"},
                "HIDDEN": {"description": "Hidden processing areas"},
                "CUSTOM": {"description": "Custom cortical areas"}
            }
        except Exception as e:
            self.logger.error(f"Error getting cortical area types: {str(e)}")
            return {}

    async def get_simulation_status(self) -> Dict[str, Any]:
        """
        Get the current simulation status.
        
        Returns:
            Dictionary containing simulation status details
        """
        try:
            if not self.state_manager:
                return {
                    "running": False,
                    "paused": False,
                    "burst_count": 0,
                    "burst_frequency": 0,
                    "timestamp": datetime.now().timestamp()
                }
                
            # Get simulation state
            sim_state = self.state_manager.get_simulation_state()
            
            # Check if burst engine is in running state
            burst_engine_state = self.state_manager.get_burst_engine_state()
            is_burst_engine_ready = self.state_manager.is_burst_engine_ready()
            
            # Determine running status
            running = (sim_state == SimulationState.RUNNING) and is_burst_engine_ready
            paused = (sim_state == SimulationState.PAUSED)
            
            return {
                "running": running,
                "paused": paused,
                "burst_count": getattr(self.state_manager, 'burst_count', 0),
                "burst_frequency": self.state_manager.get_burst_frequency(),
                "timestamp": datetime.now().timestamp()
            }
        except Exception as e:
            self.logger.error(f"Error getting simulation status: {str(e)}")
            return {
                "running": False,
                "paused": False,
                "burst_count": 0,
                "burst_frequency": 0,
                "timestamp": datetime.now().timestamp()
            }
    
    async def get_performance_stats(self) -> Dict[str, Any]:
        """
        Get performance statistics for the current simulation.
        
        Returns:
            Dictionary containing performance metrics
        """
        try:
            if not self.state_manager:
                return {
                    "burst_time_ms": 0,
                    "memory_usage_mb": 0,
                    "cpu_usage_percent": 0,
                    "active_neurons": 0,
                    "timestamp": datetime.now().timestamp()
                }
                
            # Get basic stats from state manager
            return {
                "burst_time_ms": getattr(self.state_manager, 'average_burst_time', 0) * 1000,
                "memory_usage_mb": getattr(self.state_manager, 'memory_usage', 0),
                "cpu_usage_percent": getattr(self.state_manager, 'cpu_usage', 0),
                "active_neurons": getattr(self.state_manager, 'active_neurons', 0),
                "timestamp": datetime.now().timestamp()
            }
        except Exception as e:
            self.logger.error(f"Error getting performance stats: {str(e)}")
            return {
                "burst_time_ms": 0,
                "memory_usage_mb": 0,
                "cpu_usage_percent": 0,
                "active_neurons": 0,
                "timestamp": datetime.now().timestamp()
            }

    def get_data_path(self) -> str:
        """
        Get the data directory path.
        
        Returns:
            Path to the data directory
        """
        # Try multiple possible locations
        possible_paths = [
            os.path.join(os.path.dirname(__file__), "../../../../data"),
            os.path.join(os.getcwd(), "data"),
            os.path.join(os.getcwd(), "feagi_core/data"),
            os.environ.get("FEAGI_DATA_PATH", "")
        ]
        
        for path in possible_paths:
            if path and os.path.exists(path):
                return path
                
        # If no path exists, return the first one as default
        return possible_paths[0]

    def load_genome(self, genome_data: Dict[str, Any], filename: str = "genome.json") -> Dict[str, Any]:
        """
        Load a genome and prepare it for use.
        
        Args:
            genome_data: The genome data dictionary to load
            filename: Name of the source file (for reference)
            
        Returns:
            Dictionary with success indicator and additional information
        """
        try:
            self.logger.info(f"Loading genome from {filename}")
            
            # Set brain readiness to False while loading
            if self.state_manager:
                self.state_manager.set_brain_readiness(False)
            
            # Store genome filename 
            self._genome_filename = filename
            
            # Validate genome structure - returns a boolean now
            validation_result = genome_validator(genome_data)
            if not validation_result:
                self.logger.error(f"Invalid genome structure")
                return {"success": False, "error": "Invalid genome structure"}
                
            # Store the current genome
            self._current_genome = genome_data
            
            # Update state manager with genome data but not loaded state yet
            if self.state_manager:
                self.state_manager.genome = genome_data
                self.state_manager.genome_file_name = filename
                # Don't set to LOADED yet - wait until brain development succeeds
                
            # Save genome data to a temporary file
            temp_genome_path = os.path.join(self._temp_dir, "temp_genome.json")
            with open(temp_genome_path, 'w') as f:
                json.dump(genome_data, f)
                
            # Initialize embryogenesis
            embry = NeuroEmbryogenesis(
                connectome_manager=self._connectome_manager,
                progress_callback=self._handle_embryogenesis_progress
            )
            
            # Develop brain from genome using the temporary file
            success, stats = develop_brain_from_genome(
                genome_path=temp_genome_path,
                connectome_manager=self._connectome_manager
            )
            
            if not success:
                self.logger.error(f"Failed to develop brain from genome")
                # Set error state since brain development failed
                if self.state_manager:
                    self.state_manager.set_genome_state(GenomeState.ERROR)
                    self.state_manager.set_brain_readiness(False)
                return {"success": False, "error": "Failed to develop brain from genome"}
                
            # Only set LOADED state after successful brain development
            if self.state_manager:
                self.state_manager.set_genome_state(GenomeState.LOADED)
                self.state_manager.set_brain_readiness(True)
                
            # Get cortical area count
            cortical_areas = self.get_cortical_areas()
            cortical_area_count = len(cortical_areas)
            
            # Log success
            self.logger.info(f"Genome loaded successfully: {cortical_area_count} cortical areas created")
            
            # Return success
            return {
                "success": True, 
                "cortical_area_count": cortical_area_count,
                "cortical_areas": cortical_areas
            }
            
        except Exception as e:
            self.logger.error(f"Error loading genome: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
            
            # Update state manager with error
            if self.state_manager:
                self.state_manager.set_genome_state(GenomeState.ERROR)
                self.state_manager.set_brain_readiness(False)
                
            return {"success": False, "error": str(e)}

    def get_genome(self) -> Dict[str, Any]:
        """
        Get the currently loaded genome data.
        
        Returns:
            The genome data dictionary, or None if no genome is loaded
        """
        if self._current_genome is None:
            logger.warning("No genome has been loaded")
            return None
     
        return self._current_genome
        
    def get_genome_filename(self) -> Optional[str]:
        """
        Get the filename of the currently loaded genome.
        
        Returns:
            The filename of the genome, or None if no genome is loaded
        """
        return self._genome_filename

    def get_genome_file_name(self) -> Dict[str, str]:
        """
        Get the genome file name in the format expected by the REST API.
        
        Returns:
            Dictionary with the genome file name
        """
        if self._genome_filename:
            return {"file_name": self._genome_filename}
        else:
            return {"file_name": "No genome loaded"}

    # Cortical area methods for legacy API compatibility
    
    def get_cortical_area_id_list(self) -> List[str]:
        """
        Get a list of all cortical area IDs (6-character strings) in the current genome.
        
        Returns:
            List of cortical area ID strings
        """
        try:
            if not self._current_genome or 'blueprint' not in self._current_genome:
                return []
                
            # Extract cortical IDs from blueprint keys
            # Blueprint keys have format: {prefix}-{cortical_id}-{suffix}
            # We want the second segment (cortical_id)
            cortical_ids = set()  # Use set to avoid duplicates
            
            for blueprint_key in self._current_genome['blueprint'].keys():
                # Split by dash and take the second segment
                parts = blueprint_key.split('-')
                if len(parts) >= 2:
                    cortical_id = parts[1]
                    cortical_ids.add(cortical_id)
                    
            # Return sorted list of unique cortical IDs
            return sorted(list(cortical_ids))
        except Exception as e:
            self.logger.error(f"Error getting cortical area ID list: {str(e)}")
            return []
    
    def get_cortical_area_index_list(self) -> List[int]:
        """
        Get a list of all cortical area indices (integers) used by the FCL.
        
        Returns:
            List of cortical area indices
        """
        try:
            if not hasattr(self._connectome_manager, 'cortical_areas'):
                return []
                
            return list(self._connectome_manager.cortical_areas.keys())
        except Exception as e:
            self.logger.error(f"Error getting cortical area index list: {str(e)}")
            return []
    
    def get_cortical_area_name_list(self) -> List[str]:
        """
        Get a list of all cortical area names.
        
        Returns:
            List of cortical area names
        """
        try:
            if not hasattr(self._connectome_manager, 'cortical_areas'):
                return []
                
            names = []
            for area in self._connectome_manager.cortical_areas.values():
                if hasattr(area, 'name') and area.name:
                    names.append(area.name)
            
            return names
        except Exception as e:
            self.logger.error(f"Error getting cortical area name list: {str(e)}")
            return []

    def get_cortical_id_name_mapping(self) -> Dict[str, str]:
        """
        Map every cortical area's 6-character cortical_id (from the ConnectomeManager)
        to its human-readable name.
        """
        mapping: Dict[str, str] = {}
        try:
            if not hasattr(self._connectome_manager, "cortical_areas"):
                return mapping

            for cortical_idx, area in self._connectome_manager.cortical_areas.items():
                # Prefer the explicit cortical_id stored on the CorticalArea object (if present)
                cortical_id = getattr(area, "cortical_id", None)
                if not cortical_id:
                    # Fallback: derive a placeholder ID from the integer index
                    cortical_id = self._get_cortical_id_for_idx(cortical_idx)

                name = getattr(area, "name", None)
                if cortical_id and name:
                    mapping[cortical_id] = name

            return mapping
        except Exception as exc:
            self.logger.error(f"Error building cortical_id→name mapping: {exc}")
            return mapping

    def get_cortical_locations_2d(self) -> Dict[str, List[int]]:
        """
        Get 2D locations of all cortical areas.
        
        Returns:
            Dictionary mapping cortical IDs to 2D coordinate arrays [x, y]
        """
        try:
            locations = {}
            if not self._current_genome or 'blueprint' not in self._current_genome:
                return locations
                
            # Group cortical areas by cortical_id to extract their 2D coordinates
            cortical_data = {}
            for blueprint_key, area_data in self._current_genome['blueprint'].items():
                # Parse blueprint key format: {prefix}-{cortical_id}-{property}
                parts = blueprint_key.split('-')
                if len(parts) >= 4:
                    cortical_id = parts[1]
                    property_name = parts[3]
                    
                    if cortical_id not in cortical_data:
                        cortical_data[cortical_id] = {}
                    cortical_data[cortical_id][property_name] = area_data
            
            # Extract 2D coordinates for each cortical area
            for cortical_id, properties in cortical_data.items():
                x_coord = properties.get('2dcorx', 0)
                y_coord = properties.get('2dcory', 0)
                
                # Ensure coordinates are integers
                try:
                    x_coord = int(x_coord) if x_coord is not None else 0
                    y_coord = int(y_coord) if y_coord is not None else 0
                except (ValueError, TypeError):
                    x_coord = 0
                    y_coord = 0
                
                # Return as array [x, y] format
                locations[cortical_id] = [x_coord, y_coord]
            
            return locations
        except Exception as e:
            self.logger.error(f"Error getting cortical 2D locations: {str(e)}")
            return {}

    def get_detailed_cortical_map(self) -> Dict[str, Any]:
        """
        Get detailed cortical mapping information in the format expected by brain visualizer.
        
        Returns:
            Dictionary mapping cortical IDs to their destination connections
        """
        try:
            if not self._current_genome or 'blueprint' not in self._current_genome:
                return {}
                
            cortical_map = {}
            
            # Group cortical areas by cortical_id to extract destination mappings
            cortical_data = {}
            for blueprint_key, area_data in self._current_genome['blueprint'].items():
                # Parse blueprint key format: {prefix}-{cortical_id}-{property}
                parts = blueprint_key.split('-')
                if len(parts) >= 4:
                    cortical_id = parts[1]
                    property_name = parts[3]
                    
                    if cortical_id not in cortical_data:
                        cortical_data[cortical_id] = {}
                    cortical_data[cortical_id][property_name] = area_data
            
            # Extract mapping destinations for each cortical area
            for cortical_id, properties in cortical_data.items():
                cortical_map[cortical_id] = {}
                
                # Get the destination mappings for this cortical area
                dstmap = properties.get('dstmap', {})
                if isinstance(dstmap, dict):
                    for dst_id, mapping_list in dstmap.items():
                        cortical_map[cortical_id][dst_id] = []
                        if isinstance(mapping_list, list):
                            for mapping in mapping_list:
                                # Convert raw array format to structured object format
                                if isinstance(mapping, list) and len(mapping) >= 7:
                                    # Raw format: [morphology_id, morphology_scalar, postSynapticCurrent_multiplier, plasticity_flag, plasticity_constant, ltp_multiplier, ltd_multiplier]
                                    structured_mapping = {
                                        "morphology_id": mapping[0],
                                        "morphology_scalar": mapping[1] if isinstance(mapping[1], list) else [1, 1, 1],
                                        "postSynapticCurrent_multiplier": mapping[2] if len(mapping) > 2 else 1,
                                        "plasticity_flag": mapping[3] if len(mapping) > 3 else False,
                                        "plasticity_constant": mapping[4] if len(mapping) > 4 else 1,
                                        "ltp_multiplier": mapping[5] if len(mapping) > 5 else 1,
                                        "ltd_multiplier": mapping[6] if len(mapping) > 6 else 1
                                    }
                                    cortical_map[cortical_id][dst_id].append(structured_mapping)
                                elif isinstance(mapping, dict):
                                    # Already in correct format, just copy it
                                    cortical_map[cortical_id][dst_id].append(mapping)
                        
            return cortical_map
            
        except Exception as e:
            self.logger.error(f"Error getting detailed cortical map: {str(e)}")
            return {}

    def get_morphology_list(self) -> List[str]:
        """
        Get list of all morphology names (connectivity rules).
        
        Returns:
            List of morphology names
        """
        try:
            if not self._current_genome or 'neuron_morphologies' not in self._current_genome:
                return []
                
            return list(self._current_genome['neuron_morphologies'].keys())
        except Exception as e:
            self.logger.error(f"Error getting morphology list: {str(e)}")
            return []

    def get_connectome_dimensions(self) -> Dict[str, Any]:
        """
        Get connectome dimensions and properties.
        
        Returns:
            Dictionary with connectome dimensions
        """
        try:
            if not self._current_genome:
                return {}
                
            # Get dimensions from genome properties
            genome_properties = self._current_genome.get('genome_properties', {})
            
            return {
                "max_x": genome_properties.get("max_x", 1000),
                "max_y": genome_properties.get("max_y", 1000), 
                "max_z": genome_properties.get("max_z", 100),
                "resolution": genome_properties.get("resolution", 1),
                "total_volume": genome_properties.get("total_volume", 100000000)
            }
        except Exception as e:
            self.logger.error(f"Error getting connectome dimensions: {str(e)}")
            return {}

    def get_genome_counter(self) -> int:
        """
        Get the current genome counter.
        
        Returns:
            Current genome counter value
        """
        try:
            if self.state_manager:
                return self.state_manager.get_genome_counter()
            return 0
        except Exception as e:
            self.logger.error(f"Error getting genome counter: {str(e)}")
            return 0