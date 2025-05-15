"""Core API service implementation for FEAGI."""

from typing import Dict, Any, List, Optional, Tuple, Union
from feagi.utils.logger import setup_logger
import os
import json
import tempfile
from datetime import datetime
import time
from pathlib import Path

import numpy as np

logger = setup_logger()

from feagi.core.feagi import FEAGI
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis, develop_brain_from_genome
from feagi.bdu.connectome_manager import ConnectomeManager, CorticalArea
from feagi.core.state_manager import FeagiStateManager
from feagi.core.state_manager import ServiceState
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
        
        # Initialize burst engine without requiring a genome
        self.state_manager.set_burst_engine_state(ServiceState.INITIALIZING)
        self._burst_engine = self._create_burst_engine()
        self.state_manager.set_burst_engine_state(ServiceState.READY)
        
        # Other initializations can follow...
        
        self.logger = logger
        self._feagi = FEAGI()
        self._temp_dir = tempfile.mkdtemp(prefix="feagi_")
        self._genome_filename = None
        self._pending_amalgamation = {}

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
            for area_id, area in self._connectome_manager.cortical_areas.items():
                try:
                    # Get neuron count (safely)
                    try:
                        neuron_count = len(self._connectome_manager.get_neurons_by_area(area_id))
                    except Exception:
                        neuron_count = 0
                        
                    # Convert to API format
                    result.append({
                        "id": str(area_id),  # Convert to string for API consistency
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
                    self.logger.error(f"Error converting area {area_id} to API format: {str(e)}")
                    
        except Exception as e:
            self.logger.error(f"Error retrieving cortical areas: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
        
        # Cache the result
        self._cortical_areas_cache = result
        self._cortical_areas_cache_timestamp = time.time()
        
        return result
        
    def get_cortical_area(self, area_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a cortical area by ID.
        
        Args:
            area_id: String representation of the cortical_idx.
            
        Returns:
            Dictionary containing cortical area information, or None if not found.
        """
        try:
            # Convert string ID to integer (cortical_idx)
            try:
                cortical_idx = int(area_id)
            except ValueError:
                self.logger.error(f"Invalid cortical area ID format: {area_id}")
                return None
            
            # Get the area from connectome manager
            area = self._connectome_manager.cortical_areas.get(cortical_idx)
            if not area:
                self.logger.warning(f"Cortical area {area_id} not found")
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
                area_id=new_id,
                name=name,
                area_type=area_type,
                dimensions=dims,
                position=position,
                properties=parameters or {}
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
        area_id: str,
        name: Optional[str] = None,
        coordinates: Optional[Dict[str, int]] = None,
        dimensions: Optional[Dict[str, int]] = None,
        area_type: Optional[str] = None,
        parameters: Optional[Dict[str, Any]] = None
    ) -> Optional[Dict[str, Any]]:
        """
        Update an existing cortical area.
        
        Args:
            area_id: ID of the cortical area to update.
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
            area_id_int = int(area_id)
        except ValueError:
            return None
        
        try:
            area = self._connectome_manager.cortical_areas.get(area_id_int)
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
            neuron_count = len(self._connectome_manager.get_neurons_by_area(area_id_int))
            return {
                "id": str(area_id_int),
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
            self.logger.error(f"Error updating cortical area {area_id}: {str(e)}")
            return None
    
    def delete_cortical_area(self, area_id: str) -> bool:
        """
        Delete a cortical area.
        
        Args:
            area_id: String representation of the cortical_idx to delete.
            
        Returns:
            True if the cortical area was deleted, False otherwise.
        """
        # In legacy FEAGI, this depends on a genome being loaded first
        if self._current_genome is None:
            self.logger.warning("No genome loaded, cannot delete cortical area")
            return False
        
        try:
            cortical_idx = int(area_id)
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
            self.logger.error(f"Error deleting cortical area {area_id}: {str(e)}")
            return False
    
    def get_cortical_area_neurons(self, area_id: str) -> Optional[List[Dict[str, Any]]]:
        """
        Get neurons for a specific cortical area.
        
        Args:
            area_id: ID of the cortical area.
            
        Returns:
            List of dictionaries containing neuron information,
            or None if the area doesn't exist.
        """
        # In legacy FEAGI, this depends on a genome being loaded first
        if self._current_genome is None:
            self.logger.warning("No genome loaded, cannot retrieve cortical area neurons")
            return None
        
        try:
            area_id_int = int(area_id)
        except ValueError:
            return None
        
        try:
            if area_id_int not in self._connectome_manager.cortical_areas:
                return None
            
            # Get all neurons in this area
            neuron_ids = self._connectome_manager.get_neurons_by_area(area_id_int)
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
            self.logger.error(f"Error retrieving neurons for cortical area {area_id}: {str(e)}")
            return None
    
    def get_cortical_area_activity(self, area_id: str, window: int = 1) -> Optional[Dict[str, Any]]:
        """
        Get activity data for a specific cortical area.
        
        Args:
            area_id: ID of the cortical area.
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
            area_id_int = int(area_id)
        except ValueError:
            return None
        
        try:
            if area_id_int not in self._connectome_manager.cortical_areas:
                return None
            
            # Get all neurons in this area
            neuron_ids = self._connectome_manager.get_neurons_by_area(area_id_int)
            
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
            self.logger.error(f"Error retrieving activity for cortical area {area_id}: {str(e)}")
            return None
    
    def get_cortical_area_connectivity(self, area_id: str, direction: str = "both") -> Optional[Dict[str, Any]]:
        """
        Get connectivity information for a specific cortical area.
        
        Args:
            area_id: ID of the cortical area.
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
            area_id_int = int(area_id)
        except ValueError:
            return None
        
        try:
            if area_id_int not in self._connectome_manager.cortical_areas:
                return None
            
            # Get all neurons in this area
            neuron_ids = self._connectome_manager.get_neurons_by_area(area_id_int)
            
            # Collect connectivity information
            incoming_connections = set()
            outgoing_connections = set()
            
            if direction in ["incoming", "both"]:
                for neuron_id in neuron_ids:
                    connections = self._connectome_manager.get_incoming_connections(neuron_id)
                    for pre_id, _ in connections:
                        # Skip connections within the same area
                        pre_area = self._connectome_manager._neuron_to_area.get(pre_id)
                        if pre_area is not None and pre_area != area_id_int:
                            incoming_connections.add(pre_area)
            
            if direction in ["outgoing", "both"]:
                for neuron_id in neuron_ids:
                    connections = self._connectome_manager.get_outgoing_connections(neuron_id)
                    for post_id, _ in connections:
                        # Skip connections within the same area
                        post_area = self._connectome_manager._neuron_to_area.get(post_id)
                        if post_area is not None and post_area != area_id_int:
                            outgoing_connections.add(post_area)
            
            # Format results
            result = {
                "area_id": str(area_id_int),
                "direction": direction
            }
            
            if direction in ["incoming", "both"]:
                result["incoming_connections"] = [
                    {
                        "area_id": str(connected_area),
                        "name": self._connectome_manager.cortical_areas.get(connected_area, CorticalArea(connected_area, "Unknown", "unknown", (0, 0, 0), (0, 0, 0))).name
                    } 
                    for connected_area in incoming_connections
                ]
            
            if direction in ["outgoing", "both"]:
                result["outgoing_connections"] = [
                    {
                        "area_id": str(connected_area),
                        "name": self._connectome_manager.cortical_areas.get(connected_area, CorticalArea(connected_area, "Unknown", "unknown", (0, 0, 0), (0, 0, 0))).name
                    }
                    for connected_area in outgoing_connections
                ]
            
            return result
        except Exception as e:
            self.logger.error(f"Error retrieving connectivity for cortical area {area_id}: {str(e)}")
            return None
    
    def stimulate_cortical_area(self, area_id: str, pattern: Dict[str, Any]) -> bool:
        """
        Stimulate a cortical area with a specific pattern.
        
        Args:
            area_id: ID of the cortical area.
            pattern: Stimulation pattern.
            
        Returns:
            True if stimulation was successful, False otherwise.
        """
        # In legacy FEAGI, this depends on a genome being loaded first
        if self._current_genome is None:
            self.logger.warning("No genome loaded, cannot stimulate cortical area")
            return False
        
        try:
            area_id_int = int(area_id)
        except ValueError:
            return False
        
        try:
            if area_id_int not in self._connectome_manager.cortical_areas:
                return False
            
            # Get the neurons in the area
            neuron_ids = self._connectome_manager.get_neurons_by_area(area_id_int)
            
            # Apply the stimulation pattern
            pattern_type = pattern.get("type", "uniform")
            intensity = pattern.get("intensity", 1.0)
            
            successful = False
            
            if pattern_type == "uniform":
                # Apply uniform stimulation to all neurons
                for neuron_id in neuron_ids:
                    neuron_index = self._connectome_manager._neuron_id_to_index.get(neuron_id)
                    if neuron_index is not None:
                        # Add stimulation to membrane potential
                        current_potential = self._connectome_manager.membrane_potentials[neuron_index]
                        self._connectome_manager.membrane_potentials[neuron_index] = current_potential + intensity
                        successful = True
            
            elif pattern_type == "spatial":
                # Apply stimulation based on spatial pattern
                center = pattern.get("center", {"x": 0, "y": 0, "z": 0})
                radius = pattern.get("radius", 5)
                
                for neuron_id in neuron_ids:
                    position = self._connectome_manager.get_neuron_position(neuron_id)
                    
                    # Calculate distance from center
                    dx = position[0] - center["x"]
                    dy = position[1] - center["y"]
                    dz = position[2] - center["z"]
                    distance = (dx*dx + dy*dy + dz*dz) ** 0.5
                    
                    if distance <= radius:
                        # Apply stimulation with falloff based on distance
                        falloff = 1.0 - (distance / radius)
                        neuron_index = self._connectome_manager._neuron_id_to_index.get(neuron_id)
                        if neuron_index is not None:
                            stim_value = intensity * falloff
                            current_potential = self._connectome_manager.membrane_potentials[neuron_index]
                            self._connectome_manager.membrane_potentials[neuron_index] = current_potential + stim_value
                            successful = True
            
            elif pattern_type == "specific":
                # Apply stimulation to specific neurons
                target_positions = pattern.get("positions", [])
                
                for pos in target_positions:
                    if "x" in pos and "y" in pos and "z" in pos:
                        # Find neurons at this position
                        position = (pos["x"], pos["y"], pos["z"])
                        found_neurons = self._connectome_manager.get_neurons_at_position(area_id_int, position)
                        
                        for neuron_id in found_neurons:
                            neuron_index = self._connectome_manager._neuron_id_to_index.get(neuron_id)
                            if neuron_index is not None:
                                stim_value = intensity
                                current_potential = self._connectome_manager.membrane_potentials[neuron_index]
                                self._connectome_manager.membrane_potentials[neuron_index] = current_potential + stim_value
                                successful = True
            
            return successful
        except Exception as e:
            self.logger.error(f"Error stimulating cortical area {area_id}: {str(e)}")
            return False
    
    # Simulation control methods
    
    def start_simulation(self) -> bool:
        """
        Start the simulation.
        
        Returns:
            True if successful, False otherwise.
        """
        return self._feagi.start_simulation()
        
    def stop_simulation(self) -> bool:
        """
        Stop the simulation.
        
        Returns:
            True if successful, False otherwise.
        """
        return self._feagi.stop_simulation()
        
    async def get_simulation_status(self) -> Dict[str, Any]:
        """
        Get the current simulation status.
        
        Returns:
            Dictionary containing the current simulation status.
        """
        # Placeholder implementation
        return {
            "running": False,
            "step": 0,
            "time": 0.0,
            "timestamp": time.time()
        }
        
    async def get_performance_stats(self) -> Dict[str, Any]:
        """
        Get performance statistics for the simulation.
        
        Returns:
            Dictionary containing performance statistics.
        """
        # Placeholder implementation
        return {
            "fps": 0.0,
            "neurons_active": 0,
            "synapses_active": 0,
            "memory_usage": 0.0,
            "cpu_usage": 0.0,
            "timestamp": time.time()
        }
        
    async def get_system_metrics(self) -> Dict[str, Any]:
        """
        Get system metrics.
        
        Returns:
            Dictionary containing system metrics.
        """
        # Placeholder implementation
        return {
            "cpu_usage": 0.0,
            "memory_usage": 0.0,
            "timestamp": time.time()
        }
        
    async def get_brain_structure(self) -> Dict[str, Any]:
        """
        Get the current brain structure.
        
        Returns:
            Dictionary containing the brain structure.
        """
        # Convert cortical areas to structure format
        areas = self.get_cortical_areas()
        
        # Group areas by type
        area_types = {}
        for area in areas:
            area_type = area.get("type", "unknown")
            if area_type not in area_types:
                area_types[area_type] = []
            area_types[area_type].append(area)
        
        return {
            "areas": areas,
            "area_types": area_types,
            "timestamp": time.time()
        }
        
    # Configuration methods
    
    def get_configuration(self) -> Dict[str, Any]:
        """
        Get the current configuration.
        
        Returns:
            Dictionary containing the current configuration.
        """
        return self._feagi.get_configuration()
        
    def update_configuration(self, config: Dict[str, Any]) -> bool:
        """
        Update the configuration.
        
        Args:
            config: Dictionary containing the new configuration.
            
        Returns:
            True if successful, False otherwise.
        """
        return self._feagi.update_configuration(config)
        
    # Path and file management methods
    
    def get_data_path(self) -> str:
        """
        Get the path to the data directory.
        
        Returns:
            Path to the data directory.
        """
        # First look for specific environment variable
        data_path = os.environ.get("FEAGI_DATA_PATH")
        if data_path:
            return data_path
            
        # Then look for the evo/defaults directory in the FEAGI package
        import feagi
        feagi_path = os.path.dirname(os.path.dirname(feagi.__file__))
        evo_defaults_path = os.path.join(feagi_path, "feagi", "evo", "defaults")
        
        # First verify this path exists before returning it
        if os.path.isdir(evo_defaults_path):
            self.logger.info(f"Using data path: {evo_defaults_path}")
            return evo_defaults_path
            
        # If not found through package directory, try to find relative to current file
        current_file_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
        evo_defaults_path = os.path.join(current_file_dir, "evo", "defaults")
        
        if os.path.isdir(evo_defaults_path):
            self.logger.info(f"Using data path: {evo_defaults_path}")
            return evo_defaults_path
            
        # If all else fails, try to search for it
        self.logger.warning(f"Could not find evo/defaults directory, searching...")
        
        def find_evo_defaults(start_path):
            for root, dirs, files in os.walk(start_path):
                if os.path.basename(root) == "defaults" and os.path.basename(os.path.dirname(root)) == "evo":
                    return root
            return None
            
        search_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
        found_path = find_evo_defaults(search_path)
        
        if found_path:
            self.logger.info(f"Found data path by searching: {found_path}")
            return found_path
            
        # Final fallback is to return the directory containing this script
        self.logger.error("Could not find evo/defaults directory, using fallback location")
        return os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
    
    def get_temp_path(self) -> str:
        """
        Get the path to the temporary directory.
        
        Returns:
            Path to the temporary directory.
        """
        return self._temp_dir
    
    # Genome methods
    
    def load_genome(self, genome_data: Dict[str, Any], filename: Optional[str] = None) -> dict:
        """
        Load a genome into FEAGI.
        Returns a dict with success and duration fields.
        """
        state = FeagiStateManager.instance()
        print("setting genome state to loading")
        state.set_genome_state(state=GenomeState.LOADING)
        print("done setting genome state to loading")
        state.set_brain_readiness(False)
        start_time = time.time()
        try:
            self.logger.info(f"Loading genome: {filename}")
            
            # Store the genome filename
            self._genome_filename = filename
            
            # Inject a minimal valid 'brain_regions' structure if missing
            if "brain_regions" not in genome_data:
                genome_data["brain_regions"] = {
                    "root": {
                        "title": "Root Region",
                        "description": "Root region for testing",
                        "parent_region_id": None,
                        "coordinate_2d": [0, 0],
                        "coordinate_3d": [0, 0, 0],
                        "areas": [],
                        "regions": [],
                        "inputs": [],
                        "outputs": []
                    }
                }

            # Validate the genome
            is_valid = genome_validator(genome_data)
            if not is_valid:
                self.logger.error("Invalid genome format")
                return {"success": False, "duration": time.time() - start_time, "error": "Invalid genome format"}
                
            # Process and update the genome
            genome_data = merge_core_morphologies(genome_data)
            genome_data = genome_morphology_updator(genome_data)
            genome_data = genome_physiology_updator(genome_data)
            genome_data = genome_stat_updator(genome_data)
            
            # IMPORTANT: Set the current genome here - this is what makes the genome "loaded"
            self._current_genome = genome_data
            
            # Save the pre-processed genome to a temporary file to load it with neuroembryogenesis
            genome_path = os.path.join(self._temp_dir, filename or "current_genome.json")
            with open(genome_path, 'w') as f:
                json.dump(genome_data, f, indent=2)

            # Clear the connectome manager's state to start fresh
            self._connectome_manager.cortical_areas.clear()
            
            try:
                # Try to develop the brain from genome
                success, stats = develop_brain_from_genome(
                    genome_path=genome_path,
                    connectome_manager=self._connectome_manager
                )
                
                if success:
                    self.logger.info(f"Successfully developed brain from genome: {stats}")
                else:
                    self.logger.warning(f"Brain development completed with warnings: {stats}")
            except Exception as e:
                # Log the error but don't affect genome loaded status
                self.logger.error(f"Error during brain development: {str(e)}")
                import traceback
                self.logger.error(traceback.format_exc())
                
            # Return success - the genome is loaded even if brain development failed
            FeagiStateManager.instance().increment_genome_counter()
            state.set_genome_state(state=GenomeState.LOADED)
            state.set_brain_readiness(True)
            duration = time.time() - start_time
            return {"success": True, "duration": duration}
                
        except Exception as e:
            # Only on catastrophic failure do we reset the genome loaded status
            self.logger.error(f"Error loading genome: {str(e)}")
            state.set_genome_state(state=GenomeState.ERROR)
            import traceback
            self.logger.error(traceback.format_exc())
            self._current_genome = None
            self._genome_filename = None
            state.set_brain_readiness(False)
            duration = time.time() - start_time
            return {"success": False, "duration": duration, "error": str(e)}
    
    def get_genome(self) -> Dict[str, Any]:
        """
        Get the current genome.
        
        Returns:
            Dictionary containing the current genome.
        """
        if self._current_genome is None:
            self.logger.warning("No genome currently loaded")
            return {"genome_title": "No Genome Loaded", "genome_description": "No genome is currently loaded"}
            
        return self._current_genome
    
    def get_genome_filename(self) -> Optional[str]:
        """
        Get the filename of the currently loaded genome.
        
        Returns:
            Filename of the current genome, or None if no genome is loaded.
        """
        # Use the locally stored filename if available
        if self._genome_filename:
            return self._genome_filename
            
        # Delegate to the FEAGI instance as a fallback
        return self._feagi.get_genome_filename()
    
    def get_genome_counter(self) -> int:
        """
        Get the counter for the currently loaded genome.
        
        The counter indicates how many times the genome has been updated.
        
        Returns:
            The genome counter.
        """
        if self._current_genome is None:
            return 0
            
        # Try to get the counter from the genome
        try:
            return self._current_genome.get("stats", {}).get("counter", 1)
        except Exception:
            return 1
    
    def reset_genome(self) -> bool:
        """
        Reset the current genome.
        
        Returns:
            True if successful, False otherwise.
        """
        try:
            # Reset the connectome manager
            if hasattr(self._connectome_manager, 'reset'):
                self._connectome_manager.reset()
                
            # Clear the current genome
            self._current_genome = None
            self._genome_filename = None
            
            return True
        except Exception as e:
            self.logger.error(f"Error resetting genome: {str(e)}")
            return False
    
    def get_region_title(self, region_id: str) -> Optional[str]:
        """
        Get the title of a brain region.
        
        Args:
            region_id: ID of the brain region.
            
        Returns:
            Title of the brain region, or None if not found.
        """
        # This is a placeholder as FEAGI 2.1 does not yet fully implement brain regions
        # We would need to implement this properly when the brain regions feature is completed
        
        # For now, we'll just return the region_id if it exists in the cortical areas
        if self._connectome_manager and hasattr(self._connectome_manager, 'get_area'):
            # Check if there's a cortical area with this ID
            area = self._connectome_manager.get_area(region_id)
            if area:
                return f"Region {region_id}"
        
        return None
    
    def get_genome_from_region(self, region_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a genome from a brain region.
        
        Args:
            region_id: ID of the brain region.
            
        Returns:
            Dictionary containing the genome, or None if the region was not found.
        """
        # This is a placeholder - in a real implementation, we would extract
        # just the relevant parts of the genome for this region
        
        # Check if the region exists
        if self.get_region_title(region_id) is None:
            return None
            
        # For now, just return the whole genome with a modified title
        if self._current_genome:
            genome_copy = self._current_genome.copy()
            genome_copy["genome_title"] = f"Region {region_id} Genome"
            return genome_copy
            
        return None
    
    def has_pending_amalgamation(self) -> bool:
        """
        Check if there is a pending amalgamation.
        
        Returns:
            True if there is a pending amalgamation, False otherwise.
        """
        return len(self._pending_amalgamation) > 0
    
    def initiate_amalgamation(self, amalgamation_id: str, genome_id: str, genome_title: str, genome_payload: dict) -> bool:
        """
        Initialize an amalgamation process with the given parameters.
        
        This method maintains integration with the state manager while providing a service layer.
        
        Args:
            amalgamation_id: Unique identifier for this amalgamation
            genome_id: ID of the genome being amalgamated
            genome_title: Title of the genome being amalgamated
            genome_payload: The genome data payload
        
        Returns:
            bool: True if amalgamation was successfully initiated, False otherwise
        """
        try:
            state_manager = self.get_state_manager()
            
            # Check if there's already a pending amalgamation
            if state_manager.pending_amalgamation and state_manager.pending_amalgamation.get("initiation_time"):
                self.logger.warning(f"An existing amalgamation attempt is already pending")
                return False
                
            # Process the genome payload if needed
            from feagi.evo.genome_processor import genome_2_1_convertor
            processed_genome = genome_2_1_convertor(genome_payload["blueprint"])
            
            # Calculate circuit size
            from feagi.api.rest.routers.v1.genome import circuit_size
            circuit_dimensions = circuit_size(blueprint=processed_genome["blueprint"])
            
            # Store amalgamation data in state manager
            state_manager.pending_amalgamation["genome_id"] = genome_id
            state_manager.pending_amalgamation["genome_title"] = genome_title
            state_manager.pending_amalgamation["genome_payload"] = genome_payload
            state_manager.pending_amalgamation["initiation_time"] = time.time()
            state_manager.pending_amalgamation["amalgamation_id"] = amalgamation_id
            state_manager.pending_amalgamation["circuit_size"] = circuit_dimensions
            
            # Update amalgamation history
            state_manager.amalgamation_history[amalgamation_id] = "pending"
            
            self.logger.info(f"Amalgamation initiated with ID: {amalgamation_id}", emoji1="🧬")
            return True
        except Exception as e:
            self.logger.error(f"Failed to initiate amalgamation: {str(e)}", emoji1="❌")
            return False
    
    def initiate_amalgamation_by_filename(
        self,
        amalgamation_id: str,
        genome_id: str,
        genome_title: str
    ) -> bool:
        """
        Initiate an amalgamation by filename.
        
        Args:
            amalgamation_id: ID for the amalgamation.
            genome_id: ID/filename of the genome.
            genome_title: Title of the genome.
            
        Returns:
            True if successful, False otherwise.
        """
        try:
            # Load the genome from file
            genome_path = os.path.join(self.get_data_path(), "genome", genome_id)
            if not os.path.exists(genome_path):
                self.logger.error(f"Genome file not found: {genome_path}")
                return False
                
            with open(genome_path, 'r') as f:
                genome_payload = json.load(f)
                
            # Call the regular amalgamation method
            return self.initiate_amalgamation(
                amalgamation_id=amalgamation_id,
                genome_id=genome_id,
                genome_title=genome_title,
                genome_payload=genome_payload
            )
        except Exception as e:
            self.logger.error(f"Error initiating amalgamation by filename: {str(e)}")
            return False
    
    def get_amalgamation_history(self) -> Dict[str, str]:
        """
        Get the complete amalgamation history.
        
        Returns:
            Dict mapping amalgamation IDs to their statuses
        """
        try:
            state_manager = self.get_state_manager()
            return state_manager.amalgamation_history
        except Exception as e:
            self.logger.error(f"Error retrieving amalgamation history: {str(e)}", emoji1="❌")
            return {}
    
    def get_amalgamation_status(self, amalgamation_id: str) -> Optional[str]:
        """
        Get the status of a specific amalgamation.
        
        Args:
            amalgamation_id: ID of the amalgamation to check
            
        Returns:
            Status string or None if not found
        """
        try:
            state_manager = self.get_state_manager()
            if amalgamation_id in state_manager.amalgamation_history:
                return state_manager.amalgamation_history[amalgamation_id]
            return None
        except Exception as e:
            self.logger.error(f"Error retrieving amalgamation status: {str(e)}", emoji1="❌")
            return None
    
    def cancel_amalgamation(self, amalgamation_id: str) -> bool:
        """
        Cancel a pending amalgamation.
        
        Args:
            amalgamation_id: ID of the amalgamation to cancel
            
        Returns:
            True if successfully canceled, False otherwise
        """
        try:
            state_manager = self.get_state_manager()
            
            # Update the amalgamation history
            if amalgamation_id in state_manager.amalgamation_history:
                state_manager.amalgamation_history[amalgamation_id] = "canceled"
            
            # Clear pending amalgamation if it matches the ID
            if (state_manager.pending_amalgamation and 
                state_manager.pending_amalgamation.get("amalgamation_id") == amalgamation_id):
                state_manager.pending_amalgamation.clear()
            
            self.logger.info(f"Amalgamation {amalgamation_id} canceled", emoji1="🚫")
            return True
        except Exception as e:
            self.logger.error(f"Error canceling amalgamation: {str(e)}", emoji1="❌")
            return False
    
    def get_cortical_templates(self) -> Dict[str, Any]:
        """
        Get the available cortical templates.
        
        Returns:
            Dictionary containing cortical templates.
        """
        # Placeholder - in a real implementation, we would fetch templates
        # from the connectome manager or a template registry
        
        return {
            "templates": [
                {
                    "name": "Simple Neuron Layer",
                    "description": "A simple layer of neurons",
                    "dimensions": [10, 10, 1]
                },
                {
                    "name": "Sensory Area",
                    "description": "A typical sensory processing area",
                    "dimensions": [10, 10, 5]
                }
            ]
        }
    
    def complete_amalgamation(
        self, 
        amalgamation_id: str, 
        circuit_origin: List[int], 
        brain_region_id: str = "root", 
        rewire_mode: str = "all"
    ) -> bool:
        """
        Complete an amalgamation by applying the circuit to the specified location.
        
        This method maintains tight integration with the state manager while providing
        a service layer for the API.
        
        Args:
            amalgamation_id: ID of the pending amalgamation
            circuit_origin: [x, y, z] coordinates for circuit placement
            brain_region_id: ID of the target brain region
            rewire_mode: Mode for rewiring the circuit ("all", "system", or "none")
            
        Returns:
            bool: True if amalgamation was successfully completed, False otherwise
        """
        try:
            # Get the state manager
            state_manager = self.get_state_manager()
            
            # Verify there's a pending amalgamation with matching ID
            if (not state_manager.pending_amalgamation or 
                not state_manager.pending_amalgamation.get("initiation_time") or
                state_manager.pending_amalgamation.get("amalgamation_id") != amalgamation_id):
                self.logger.warning(f"No matching pending amalgamation found for ID: {amalgamation_id}")
                return False
            
            # Prepare the payload for processing
            payload = {
                "genome_str": state_manager.pending_amalgamation["genome_payload"],
                "circuit_origin": circuit_origin,
                "parent_brain_region": brain_region_id,
                "rewire_mode": rewire_mode
            }
            
            # Process the amalgamation through the appropriate channel
            # In this case, we'll use the API queue mechanism that's already in place
            # This maintains compatibility with existing systems
            from feagi.api.rest.routers.v1.genome import api_queue
            api_queue.put(item={'append_circuit': payload})
            
            # Update amalgamation history
            state_manager.amalgamation_history[amalgamation_id] = "complete"
            
            # Cancel the pending amalgamation
            from feagi.api.rest.routers.v1.genome import cancel_pending_amalgamation
            cancel_pending_amalgamation(amalgamation_id)
            
            self.logger.info(f"Amalgamation completed successfully with ID: {amalgamation_id}", emoji1="✅")
            return True
        except Exception as e:
            self.logger.error(f"Failed to complete amalgamation: {str(e)}", emoji1="❌")
            return False
    
    def get_amalgamation_info(self, amalgamation_id: str) -> Optional[Dict[str, Any]]:
        """
        Get information about an amalgamation.
        
        Args:
            amalgamation_id: ID of the amalgamation.
            
        Returns:
            Dictionary containing amalgamation information, or None if not found.
        """
        # Check if it's a pending amalgamation
        if amalgamation_id in self._pending_amalgamation:
            return {
                "id": amalgamation_id,
                "status": "pending",
                "genome_id": self._pending_amalgamation[amalgamation_id]["genome_id"],
                "genome_title": self._pending_amalgamation[amalgamation_id]["genome_title"]
            }
            
        # Check if it's in the history
        if amalgamation_id in self._amalgamation_history:
            status = self._amalgamation_history[amalgamation_id]
            return {
                "id": amalgamation_id,
                "status": status,
                "genome_id": "unknown",  # We don't store this in the history currently
                "genome_title": "Unknown"  # We don't store this in the history currently
            }
            
        # Not found
        return None
    
    def get_circuit_library(self) -> Dict[str, Any]:
        """
        Get the circuit library.
        
        Returns:
            Dictionary containing the circuit library.
        """
        # Placeholder - in a real implementation, we would fetch actual circuits
        return {
            "circuits": [
                {
                    "name": "Simple Feed-Forward",
                    "description": "A simple feed-forward circuit",
                    "cortical_areas": 2
                },
                {
                    "name": "Recurrent Network",
                    "description": "A recurrent network circuit",
                    "cortical_areas": 3
                }
            ]
        }
    
    def append_circuit(self, circuit_origin: Tuple[int, int, int], circuit_data: Dict[str, Any]) -> bool:
        """
        Append a circuit to the current genome.
        
        Args:
            circuit_origin: Tuple of (x, y, z) coordinates for the circuit origin.
            circuit_data: Dictionary containing the circuit data.
            
        Returns:
            True if successful, False otherwise.
        """
        try:
            # Placeholder - in a real implementation, we would merge the circuit
            # into the current genome and update the connectome
            
            # For now, log that we received the request
            self.logger.info(f"Appending circuit at {circuit_origin}: {circuit_data.get('genome_title', 'Unnamed')}")
            
            return True
        except Exception as e:
            self.logger.error(f"Error appending circuit: {str(e)}")
            return False
    
    def get_cortical_area_types(self) -> Dict[str, List[str]]:
        """
        Get available cortical area types.
        
        Returns:
            Dictionary containing available cortical area types.
        """
        # In legacy FEAGI, this might depend on a genome being loaded
        if self._current_genome is None:
            self.logger.info("No genome loaded, returning default cortical area types")
        
        # Delegate to the FEAGI instance to match the legacy API format
        return self._feagi.get_cortical_area_types()
    
    def get_input_sources(self) -> List[Dict[str, Any]]:
        """
        Get all registered input sources.
        
        Returns:
            List of dictionaries containing input source information.
        """
        # This is a placeholder implementation
        # In a real implementation, this would retrieve input sources from FEAGI
        self.logger.info("get_input_sources called")
        return [
            {
                "id": "camera1",
                "name": "Front Camera",
                "type": "camera",
                "target_area_id": "101",
                "properties": {
                    "resolution": "640x480"
                }
            },
            {
                "id": "microphone1",
                "name": "Microphone",
                "type": "audio",
                "target_area_id": "102",
                "properties": {
                    "sample_rate": 44100
                }
            }
        ]
        
    def get_input_source(self, source_id: str) -> Optional[Dict[str, Any]]:
        """
        Get an input source by ID.
        
        Args:
            source_id: ID of the input source.
            
        Returns:
            Dictionary containing input source information, or None if not found.
        """
        # This is a placeholder implementation
        self.logger.info(f"get_input_source called with source_id={source_id}")
        sources = self.get_input_sources()
        for source in sources:
            if source["id"] == source_id:
                return source
        return None
        
    def register_input_source(self, source_data: Dict[str, Any]) -> str:
        """
        Register a new input source.
        
        Args:
            source_data: Dictionary containing input source information.
            
        Returns:
            ID of the newly registered input source.
        """
        # This is a placeholder implementation
        self.logger.info(f"register_input_source called with source_data={source_data}")
        return "new_source_id"
        
    def update_input_source(self, source_id: str, source_data: Dict[str, Any]) -> bool:
        """
        Update an existing input source.
        
        Args:
            source_id: ID of the input source to update.
            source_data: Updated input source information.
            
        Returns:
            True if successful, False otherwise.
        """
        # This is a placeholder implementation
        self.logger.info(f"update_input_source called with source_id={source_id}, source_data={source_data}")
        return True
        
    def remove_input_source(self, source_id: str) -> bool:
        """
        Remove an input source.
        
        Args:
            source_id: ID of the input source to remove.
            
        Returns:
            True if successful, False otherwise.
        """
        # This is a placeholder implementation
        self.logger.info(f"remove_input_source called with source_id={source_id}")
        return True
        
    def stimulate_cortical_area(self, area_id: str, pattern: str = "random", 
                               intensity: float = 1.0, duration: int = 1,
                               coordinates: Optional[List[Dict[str, int]]] = None) -> Dict[str, Any]:
        """
        Stimulate a cortical area with the specified pattern.
        
        Args:
            area_id: ID of the cortical area to stimulate.
            pattern: Stimulation pattern (random, specific, etc.)
            intensity: Stimulation intensity (0.0-1.0)
            duration: Stimulation duration in bursts
            coordinates: Specific coordinates to stimulate
            
        Returns:
            Information about the applied stimulation.
        """
        # This is a placeholder implementation
        self.logger.info(f"stimulate_cortical_area called with area_id={area_id}, pattern={pattern}")
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

    #----------------------------------------------------------------------
    # Membrane Potential and Neuron Activity Methods
    #----------------------------------------------------------------------
    
    def get_membrane_potentials(self, neuron_ids: List[int]) -> Dict[int, float]:
        """
        Get membrane potentials for the specified neurons.
        
        Args:
            neuron_ids: List of neuron IDs
            
        Returns:
            Dictionary mapping neuron IDs to their membrane potentials
        """
        result = {}
        try:
            for neuron_id in neuron_ids:
                # Use explicit accessor method rather than direct attribute access
                neuron = self._connectome_manager.neurons.get(neuron_id)
                if neuron:
                    result[neuron_id] = neuron.get("membrane_potential", 0.0)
        except Exception as e:
            self.logger.error(f"Error retrieving membrane potentials: {str(e)}")
        
        return result
    
    def update_membrane_potentials(self, neuron_potentials: Dict[int, float]) -> bool:
        """
        Update membrane potentials for multiple neurons in a batch operation.
        
        Args:
            neuron_potentials: Dictionary mapping neuron IDs to new membrane potential values
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Group updates by area for potential batch optimization
            area_neuron_map = {}
            
            for neuron_id, potential in neuron_potentials.items():
                # Get area for this neuron
                try:
                    area_id = self._connectome_manager.get_area_for_neuron(neuron_id)
                    if area_id not in area_neuron_map:
                        area_neuron_map[area_id] = {}
                    area_neuron_map[area_id][neuron_id] = potential
                except KeyError:
                    self.logger.warning(f"Neuron {neuron_id} not found in any area, skipping")
                    continue
            
            # Apply updates
            for area_id, area_neurons in area_neuron_map.items():
                for neuron_id, potential in area_neurons.items():
                    neuron = self._connectome_manager.neurons.get(neuron_id)
                    if neuron:
                        neuron["membrane_potential"] = potential
            
            return True
        except Exception as e:
            self.logger.error(f"Error updating membrane potentials: {str(e)}")
            return False
    
    def batch_create_neurons(self, area_id: str, positions: List[Tuple[int, int, int]], 
                          properties: Optional[Dict[str, Any]] = None) -> List[int]:
        """
        Create multiple neurons in a batch operation.
        
        Args:
            area_id: ID of the cortical area
            positions: List of (x, y, z) positions for the neurons
            properties: Shared properties for all neurons
            
        Returns:
            List of created neuron IDs
        """
        try:
            # Validate the cortical area exists
            if not self._connectome_manager.cortical_areas.get(area_id):
                self.logger.error(f"Cortical area {area_id} not found")
                return []
                
            # Default properties if not provided
            if properties is None:
                properties = {
                    "threshold": 1.0,
                    "refractory_period": 5,
                    "decay_rate": 0.1,
                    "resting_potential": 0.0
                }
            
            # Create neurons in batch
            neuron_ids = []
            for position in positions:
                try:
                    neuron_id = self._connectome_manager.create_neuron(
                        area_id=area_id,
                        position=position,
                        **properties
                    )
                    neuron_ids.append(neuron_id)
                except Exception as e:
                    self.logger.error(f"Error creating neuron at position {position}: {str(e)}")
            
            return neuron_ids
        except Exception as e:
            self.logger.error(f"Error in batch neuron creation: {str(e)}")
            return []
    
    def batch_create_synapses(self, connections: List[Tuple[int, int, float]]) -> int:
        """
        Create multiple synapses in a batch operation.
        
        Args:
            connections: List of (pre_neuron_id, post_neuron_id, weight) tuples
            
        Returns:
            Number of successfully created synapses
        """
        try:
            success_count = 0
            for pre_id, post_id, weight in connections:
                try:
                    self._connectome_manager.create_synapse(
                        pre_neuron_id=pre_id,
                        post_neuron_id=post_id,
                        weight=weight
                    )
                    success_count += 1
                except Exception as e:
                    self.logger.error(f"Error creating synapse from {pre_id} to {post_id}: {str(e)}")
            
            return success_count
        except Exception as e:
            self.logger.error(f"Error in batch synapse creation: {str(e)}")
            return 0
    
    #----------------------------------------------------------------------
    # FCL Sampler Methods
    #----------------------------------------------------------------------
    
    def get_area_fcl_sample_rate(self, area_id: int) -> Optional[float]:
        """
        Get the FCL sample rate for a specific cortical area.
        
        Args:
            area_id: ID of the cortical area
            
        Returns:
            Sample rate value, or None if area not found
            
        Raises:
            KeyError: If cortical area not found
        """
        # Check if the area exists
        area = self._connectome_manager.cortical_areas.get(area_id)
        if area is None:
            raise KeyError(f"Cortical area {area_id} not found")
            
        # Get sample rate from area properties, or use fallback
        rate = area.properties.get('fcl_sample_rate')
        if rate is None:
            # Use the global default if no specific rate is set
            state_manager = FeagiStateManager.instance()
            rate = state_manager.get_fcl_sampler_frequency()
            
        return rate
    
    def set_area_fcl_sample_rate(self, area_id: int, sample_rate: float) -> bool:
        """
        Set the FCL sample rate for a specific cortical area.
        
        Args:
            area_id: ID of the cortical area
            sample_rate: New sample rate value
            
        Returns:
            True if successful, False otherwise
            
        Raises:
            ValueError: If sample rate is invalid
            KeyError: If cortical area not found
        """
        if sample_rate <= 0:
            raise ValueError("Sample rate must be positive")
            
        # Check if the area exists
        area = self._connectome_manager.cortical_areas.get(area_id)
        if area is None:
            raise KeyError(f"Cortical area {area_id} not found")
            
        try:
            # Update the property in the area
            area.properties['fcl_sample_rate'] = sample_rate
            
            # Attempt to notify the process manager if available
            try:
                from feagi.process_manager import get_process_manager
                process_mgr = get_process_manager()
                if process_mgr:
                    process_mgr.update_area_sample_rate(area_id, sample_rate)
            except (ImportError, AttributeError):
                self.logger.warning("Process manager not available for live FCL rate update")
                
            return True
        except Exception as e:
            self.logger.error(f"Error setting FCL sample rate: {str(e)}")
            return False
            
    #----------------------------------------------------------------------
    # Connectome Mapping Methods
    #----------------------------------------------------------------------
    
    def get_neuron_mappings(self) -> Dict[int, List[int]]:
        """
        Get mappings between neurons for visualization.
        
        Returns:
            Dictionary mapping source neuron IDs to lists of target neuron IDs
            
        Note:
            This method is useful for visualizing the connectome with tools 
            like https://csacademy.com/app/graph_editor/
        """
        try:
            mappings = {}
            for neuron_id in self._connectome_manager._neuron_id_to_index.keys():
                mappings[neuron_id] = self._connectome_manager.get_outgoing_connections(neuron_id)
            return mappings
        except Exception as e:
            self.logger.error(f"Error getting neuron mappings: {str(e)}")
            return {}
    
    #----------------------------------------------------------------------
    # Cortical Area Methods
    #----------------------------------------------------------------------
    
    def get_cortical_area_properties(self, cortical_id: str) -> Dict[str, Any]:
        """
        Get properties for a specific cortical area.
        
        Args:
            cortical_id: ID of the cortical area
            
        Returns:
            Dictionary containing the area's properties
            
        Raises:
            KeyError: If cortical area not found
            ValueError: If cortical ID has invalid length
        """
        from feagi.evo.genome_properties import genome_properties
        
        if len(cortical_id) != genome_properties["structure"]["cortical_id_length"]:
            raise ValueError("Cortical ID has invalid length")
            
        if cortical_id not in self._connectome_manager.genome['blueprint']:
            raise KeyError(f"Cortical area {cortical_id} not found")
        
        cortical_data = self._connectome_manager.genome['blueprint'][cortical_id]
        brain_region_id = self._connectome_manager.cortical_area_region_association[cortical_id]
        brain_region_title = ""
        if brain_region_id:
            if brain_region_id in self._connectome_manager.genome["brain_regions"]:
                brain_region_title = self._connectome_manager.genome["brain_regions"][brain_region_id]["title"]

        # Handle missing fields
        if 'mp_charge_accumulation' not in cortical_data:
            cortical_data['mp_charge_accumulation'] = False

        if 'mp_driven_psp' not in cortical_data:
            cortical_data['mp_driven_psp'] = False

        if '2d_coordinate' not in cortical_data:
            cortical_data['2d_coordinate'] = [None, None]

        # Get leak variability or default to 0
        leak_variability = cortical_data.get('leak_variability', 0)
        
        # Check if the area is visible
        cortical_visibility = True
        if cortical_id in self._connectome_manager.cortical_viz_list:
            cortical_visibility = False

        # Get cortical type
        cortical_type = self._connectome_manager.get_cortical_area_type(cortical_id)

        # Get dimensions
        dim_x = cortical_data["block_boundaries"][0]
        dim_y = cortical_data["block_boundaries"][1]
        dim_z = cortical_data["block_boundaries"][2]
        
        # Build properties dictionary
        cortical_properties = {
            "cortical_id": cortical_id,
            "cortical_name": cortical_data['cortical_name'],
            "parent_region_id": brain_region_id,
            "parent_region_title": brain_region_title,
            "cortical_group": cortical_data['group_id'],
            "cortical_sub_group": cortical_data['sub_group_id'],
            "cortical_neuron_per_vox_count": cortical_data['per_voxel_neuron_cnt'],
            "cortical_visibility": cortical_visibility,
            "cortical_synaptic_attractivity": cortical_data['synapse_attractivity'],
            "coordinates_3d": [
                cortical_data["relative_coordinate"][0],
                cortical_data["relative_coordinate"][1],
                cortical_data["relative_coordinate"][2]
            ],
            "coordinates_2d": [
                cortical_data["2d_coordinate"][0],
                cortical_data["2d_coordinate"][1]
            ],
            "cortical_dimensions": [dim_x, dim_y, dim_z],
            "cortical_destinations": self._connectome_manager.get_outgoing_connections(cortical_id),
            "neuron_post_synaptic_potential": cortical_data['postsynaptic_current'],
            "neuron_post_synaptic_potential_max": cortical_data['postsynaptic_current_max'],
            "neuron_fire_threshold": cortical_data['firing_threshold'],
            "neuron_fire_threshold_increment": [
                cortical_data['firing_threshold_increment_x'],
                cortical_data['firing_threshold_increment_y'],
                cortical_data['firing_threshold_increment_z']
            ],
            "neuron_firing_threshold_limit": cortical_data['firing_threshold_limit'],
            "neuron_refractory_period": cortical_data['refractory_period'],
            "neuron_leak_coefficient": cortical_data['leak_coefficient'],
            "neuron_leak_variability": leak_variability,
            "neuron_consecutive_fire_count": cortical_data['consecutive_fire_cnt_max'],
            "neuron_snooze_period": cortical_data['snooze_length'],
            "neuron_degeneracy_coefficient": cortical_data['degeneration'],
            "neuron_psp_uniform_distribution": cortical_data['psp_uniform_distribution'],
            "neuron_mp_charge_accumulation": cortical_data['mp_charge_accumulation'],
            "neuron_mp_driven_psp": cortical_data['mp_driven_psp'],
            "neuron_longterm_mem_threshold": cortical_data['longterm_mem_threshold'],
            "neuron_lifespan_growth_rate": cortical_data['lifespan_growth_rate'],
            "neuron_init_lifespan": cortical_data['init_lifespan'],
            "temporal_depth": cortical_data['temporal_depth'],
            "neuron_excitability": cortical_data['neuron_excitability'],
            "transforming": False
        }
        
        # Add IPU/OPU specific details if applicable
        if cortical_type in ["IPU", "OPU"]:
            dev_count = self._connectome_manager.genome["blueprint"][cortical_id]["dev_count"]
            unit_dim_x = int(dim_x / dev_count)
            unit_dim_y = dim_y
            unit_dim_z = dim_z
            
            cortical_properties["dev_count"] = dev_count
            cortical_properties["cortical_dimensions_per_device"] = [unit_dim_x, unit_dim_y, unit_dim_z]
            
        # Check if transforming
        if cortical_id in self._connectome_manager.transforming_areas:
            cortical_properties["transforming"] = True
            
        return cortical_properties
        
    def get_cortical_area_id_list(self) -> List[str]:
        """
        Get a list of cortical area IDs.
        
        Returns:
            List of cortical area IDs (6-letter strings)
        """
        try:
            # First try CorticalArea objects with cortical_id attribute
            if hasattr(self._connectome_manager, '_areas') and self._connectome_manager._areas:
                # Extract the cortical_id attribute from each CorticalArea object
                result = [area.cortical_id for area in self._connectome_manager._areas.values() 
                        if hasattr(area, 'cortical_id') and area.cortical_id]
                if result:
                    return result
            
            # Fallback: Use the mapping if available
            if hasattr(self._connectome_manager, '_cortical_id_to_idx') and self._connectome_manager._cortical_id_to_idx:
                return list(self._connectome_manager._cortical_id_to_idx.keys())
            
            # Fallback: Try the genome blueprint
            if hasattr(self._connectome_manager, 'genome') and self._connectome_manager.genome and 'blueprint' in self._connectome_manager.genome:
                # Extract unique cortical IDs from the blueprint
                cortical_ids = set()
                for gene_key in self._connectome_manager.genome['blueprint']:
                    if isinstance(gene_key, str) and '-' in gene_key:
                        parts = gene_key.split('-')
                        if len(parts) >= 2:
                            cortical_ids.add(parts[1])
                return list(cortical_ids)

            # Last resort
            return []
        except Exception as e:
            self.logger.error(f"Error getting cortical area ID list: {str(e)}")
            return []
            
    def get_cortical_area_index_list(self) -> List[int]:
        """
        Get a list of cortical area indices used by the FCL.
        
        Returns:
            List of cortical area indices (integers)
        """
        try:
            # Use the _cortical_idx_to_id dict which contains the integer indices as keys
            if hasattr(self._connectome_manager, '_cortical_idx_to_id') and self._connectome_manager._cortical_idx_to_id:
                return list(self._connectome_manager._cortical_idx_to_id.keys())
            
            # Fallback: If we have _areas with integer keys, use those
            if hasattr(self._connectome_manager, '_areas') and self._connectome_manager._areas:
                return list(self._connectome_manager._areas.keys())
                
            # Last resort
            return []
        except Exception as e:
            self.logger.error(f"Error getting cortical area index list: {str(e)}")
            return []

    #----------------------------------------------------------------------
    # Morphology Methods (Connectivity Rules)
    #----------------------------------------------------------------------
    
    def get_morphology_list(self) -> List[str]:
        """
        Get list of all neuron morphologies (connectivity rules).
        
        Note: "Morphology" is the legacy term for connectivity rules in v1 API.
        
        Returns:
            List of morphology names
        """
        if not self.genome_is_loaded():
            return []
            
        morphology_names = set()
        try:
            for morphology in self._current_genome.get('neuron_morphologies', {}):
                morphology_names.add(morphology)
            return sorted(morphology_names)
        except Exception as e:
            self.logger.error(f"Error retrieving morphology list: {str(e)}")
            return []
    
    def get_morphology_properties(self, morphology_name: str) -> Optional[Dict[str, Any]]:
        """
        Get properties of a specific neuron morphology (connectivity rule).
        
        Note: "Morphology" is the legacy term for connectivity rules in v1 API.
        
        Args:
            morphology_name: Name of the morphology/connectivity rule
            
        Returns:
            Dictionary containing morphology properties or None if not found
        """
        if not self.genome_is_loaded():
            return None
            
        try:
            if morphology_name in self._current_genome.get('neuron_morphologies', {}):
                results = self._current_genome['neuron_morphologies'][morphology_name].copy()
                results["morphology_name"] = morphology_name
                return results
            return None
        except Exception as e:
            self.logger.error(f"Error retrieving morphology properties: {str(e)}")
            return None
    
    def get_morphology_usage(self, morphology_name: str) -> List[Tuple[str, str]]:
        """
        Get list of cortical areas where a specific morphology/connectivity rule is used.
        
        Note: "Morphology" is the legacy term for connectivity rules in v1 API.
        
        Args:
            morphology_name: Name of the morphology/connectivity rule to check
            
        Returns:
            List of tuples containing (source_area, target_area) where the morphology is used
        """
        if not self.genome_is_loaded():
            return []
            
        try:
            usage_list = set()
            for cortical_area in self._current_genome.get('blueprint', {}):
                for destination in self._current_genome['blueprint'][cortical_area].get('cortical_mapping_dst', {}):
                    for mapping in self._current_genome['blueprint'][cortical_area]['cortical_mapping_dst'][destination]:
                        if mapping.get("morphology_id") == morphology_name:
                            usage_list.add((cortical_area, destination))
            return list(usage_list)
        except Exception as e:
            self.logger.error(f"Error retrieving morphology usage: {str(e)}")
            return []
    
    def update_morphology(self, name: str, morphology_type: str, parameters: Dict[str, Any]) -> bool:
        """
        Update an existing neuron morphology (connectivity rule).
        
        Note: "Morphology" is the legacy term for connectivity rules in v1 API.
        
        Args:
            name: Name of the morphology/connectivity rule to update
            morphology_type: Type of the rule (vectors, patterns, composite, functions)
            parameters: Rule-specific parameters
            
        Returns:
            True if successful, False otherwise
        """
        if not self.genome_is_loaded():
            return False
            
        try:
            if name not in self._current_genome.get('neuron_morphologies', {}):
                self.logger.warning(f"Morphology {name} not found")
                return False
                
            current_class = self._current_genome['neuron_morphologies'][name].get("class")
            if current_class == "core":
                self.logger.warning(f"Morphology {name} is a core morphology and cannot be modified")
                return False
                
            # Update the morphology
            self._current_genome['neuron_morphologies'][name]["type"] = morphology_type
            self._current_genome['neuron_morphologies'][name]["parameters"] = parameters
            
            # Notify any listeners about genome changes
            if self.state_manager:
                self.state_manager.set_genome_state(GenomeState.LOADED)
                
            return True
        except Exception as e:
            self.logger.error(f"Error updating morphology: {str(e)}")
            return False
    
    def add_morphology(self, name: str, morphology_type: str, parameters: Dict[str, Any]) -> bool:
        """
        Add a new neuron morphology (connectivity rule).
        
        Note: "Morphology" is the legacy term for connectivity rules in v1 API.
        
        Args:
            name: Name of the morphology/connectivity rule to add
            morphology_type: Type of the rule (vectors, patterns, composite, functions)
            parameters: Rule-specific parameters
            
        Returns:
            True if successful, False otherwise
        """
        if not self.genome_is_loaded():
            return False
            
        try:
            if name in self._current_genome.get('neuron_morphologies', {}):
                self.logger.warning(f"Morphology {name} already exists")
                return False
                
            # Make sure neuron_morphologies exists
            if 'neuron_morphologies' not in self._current_genome:
                self._current_genome['neuron_morphologies'] = {}
                
            # Add the morphology
            self._current_genome['neuron_morphologies'][name] = {
                "type": morphology_type,
                "class": "custom",
                "parameters": parameters
            }
            
            # Notify any listeners about genome changes
            if self.state_manager:
                self.state_manager.set_genome_state(GenomeState.LOADED)
                
            return True
        except Exception as e:
            self.logger.error(f"Error adding morphology: {str(e)}")
            return False
    
    def delete_morphology(self, name: str) -> bool:
        """
        Delete a neuron morphology (connectivity rule).
        
        Note: "Morphology" is the legacy term for connectivity rules in v1 API.
        
        Args:
            name: Name of the morphology/connectivity rule to delete
            
        Returns:
            True if successful, False otherwise
        """
        if not self.genome_is_loaded():
            return False
            
        try:
            if name not in self._current_genome.get('neuron_morphologies', {}):
                self.logger.warning(f"Morphology {name} not found")
                return False
                
            morphology = self._current_genome['neuron_morphologies'][name]
            if morphology.get("class") == "core":
                self.logger.warning(f"Morphology {name} is a core morphology and cannot be deleted")
                return False
                
            # Check if morphology is in use
            usage = self.get_morphology_usage(name)
            if usage:
                self.logger.warning(f"Morphology {name} is in use and cannot be deleted")
                return False
                
            # Delete the morphology
            del self._current_genome['neuron_morphologies'][name]
            
            # Notify any listeners about genome changes
            if self.state_manager:
                self.state_manager.set_genome_state(GenomeState.LOADED)
                
            return True
        except Exception as e:
            self.logger.error(f"Error deleting morphology: {str(e)}")
            return False
    
    def get_morphology_functions(self) -> List[str]:
        """
        Get list of available morphology functions (connectivity rule functions).
        
        Note: "Morphology" is the legacy term for connectivity rules in v1 API.
        
        Returns:
            List of morphology function names
        """
        try:
            morphology_list = set()
            for entry in dir(synaptogenesis_rules):
                if str(entry).startswith("syn_"):
                    morphology_list.add(str(entry))
            return list(morphology_list)
        except Exception as e:
            self.logger.error(f"Error retrieving morphology functions: {str(e)}")
            return []
    
    def get_burst_counter(self) -> Optional[int]:
        """
        Get the current burst counter value.
        
        Returns:
            The current burst counter value or None if not available
        """
        try:
            burst_engine = self.get_burst_engine()
            if burst_engine:
                return burst_engine.get_burst_counter()
            return None
        except Exception as e:
            self.logger.error(f"Error retrieving burst counter: {str(e)}", emoji1="❌")
            return None
    
    def get_fcl_sampler_config(self) -> Dict[str, Any]:
        """
        Get the FCL sampler configuration.
        
        Returns:
            Dictionary with frequency and consumer settings
        """
        try:
            state_manager = self.get_state_manager()
            return {
                "frequency": state_manager.get_fcl_sampler_frequency(),
                "consumer": state_manager.get_fcl_sampler_consumer()
            }
        except Exception as e:
            self.logger.error(f"Error retrieving FCL sampler config: {str(e)}", emoji1="❌")
            return {"frequency": 0.0, "consumer": 0}
    
    def update_fcl_sampler_config(self, frequency: float, consumer: int) -> bool:
        """
        Update the FCL sampler configuration.
        
        Args:
            frequency: The sampling frequency
            consumer: The consumer type (0=Visualization, 1=Motor)
            
        Returns:
            True if successful, False otherwise
        """
        try:
            state_manager = self.get_state_manager()
            state_manager.set_fcl_sampler_frequency(frequency)
            state_manager.set_fcl_sampler_consumer(consumer)
            
            # TODO: Notify process manager/FCLSampler to update live config if running
            
            self.logger.info(f"FCL sampler config updated: frequency={frequency}, consumer={consumer}", emoji1="⚙️")
            return True
        except Exception as e:
            self.logger.error(f"Error updating FCL sampler config: {str(e)}", emoji1="❌")
            return False
    
    def get_transforming_areas(self) -> Dict[str, Any]:
        """
        Get a dictionary of transforming cortical areas.
        
        Transforming areas are those undergoing neuroplastic changes.
        
        Returns:
            Dictionary mapping area IDs to their transformation details
        """
        try:
            # First check if the connectome manager has a transforming_areas attribute
            if hasattr(self._connectome_manager, 'transforming_areas'):
                return self._connectome_manager.transforming_areas
                
            # Otherwise get it from the state manager
            state_manager = self.get_state_manager()
            return getattr(state_manager, 'transforming_areas', {})
        except Exception as e:
            self.logger.error(f"Error retrieving transforming areas: {str(e)}", emoji1="❌")
            return {}
    
    def get_plasticity_info(self) -> Dict[str, Any]:
        """
        Get neuroplasticity information for the connectome.
        
        Returns:
            Dictionary containing plasticity information
        """
        try:
            # First check if the connectome manager has a plasticity_dict attribute
            if hasattr(self._connectome_manager, 'plasticity_dict'):
                return self._connectome_manager.plasticity_dict
                
            # Otherwise get it from the state manager
            state_manager = self.get_state_manager()
            return getattr(state_manager, 'plasticity_dict', {})
        except Exception as e:
            self.logger.error(f"Error retrieving plasticity information: {str(e)}", emoji1="❌")
            return {}
    
    def get_connectome_dimensions(self) -> List[int]:
        """
        Get the overall dimensions of the connectome.
        
        Returns:
            List of three integers representing width, height, and depth
        """
        try:
            # First check if the connectome manager has a cortical_dimensions attribute
            if hasattr(self._connectome_manager, 'cortical_dimensions'):
                return self._connectome_manager.cortical_dimensions
                
            # Otherwise get it from the state manager
            state_manager = self.get_state_manager()
            return getattr(state_manager, 'cortical_dimensions', [0, 0, 0])
        except Exception as e:
            self.logger.error(f"Error retrieving connectome dimensions: {str(e)}", emoji1="❌")
            return [0, 0, 0]
    
    def get_cortical_area_stats(self, cortical_area: str) -> Dict[str, Any]:
        """
        Get cumulative statistics for a specific cortical area.
        
        Args:
            cortical_area: ID of the cortical area
            
        Returns:
            Dictionary containing statistics for the cortical area
        """
        try:
            # First check if the connectome manager has a cumulative_stats attribute
            if hasattr(self._connectome_manager, 'cumulative_stats'):
                stats = self._connectome_manager.cumulative_stats
                return stats.get(cortical_area, {})
                
            # Otherwise get it from the state manager
            state_manager = self.get_state_manager()
            stats = getattr(state_manager, 'cumulative_stats', {})
            return stats.get(cortical_area, {})
        except Exception as e:
            self.logger.error(f"Error retrieving cortical area stats: {str(e)}", emoji1="❌")
            return {}
    
    def save_connectome_snapshot(self, path: str) -> bool:
        """
        Save a snapshot of the current connectome to the specified path.
        
        Args:
            path: Path to save the snapshot
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Create a serializable representation of the connectome
            snapshot = {
                "timestamp": datetime.now().isoformat(),
                "cortical_areas": self.get_cortical_areas(),
                "dimensions": self.get_connectome_dimensions(),
                "stats": {
                    "neuron_count": len(self._connectome_manager.neurons),
                    "synapse_count": self._connectome_manager.synapse_count()
                }
            }
            
            # Save to file
            import json
            import os
            
            # Make sure the directory exists
            os.makedirs(os.path.dirname(path), exist_ok=True)
            
            with open(path, 'w') as f:
                json.dump(snapshot, f, indent=2)
                
            self.logger.info(f"Connectome snapshot saved to {path}", emoji1="💾")
            return True
        except Exception as e:
            self.logger.error(f"Error saving connectome snapshot: {str(e)}", emoji1="❌")
            return False
    
    def import_cortical_area(self, cortical_area_data: Dict[str, Any]) -> bool:
        """
        Import a cortical area from serialized data.
        
        Args:
            cortical_area_data: Dictionary containing cortical area data
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Extract the area ID
            area_id = cortical_area_data.get("id")
            if not area_id:
                self.logger.error("Cortical area data missing ID field")
                return False
                
            # Extract the dimensions and position
            try:
                dimensions = (
                    cortical_area_data["dimensions"]["width"],
                    cortical_area_data["dimensions"]["height"],
                    cortical_area_data["dimensions"]["depth"]
                )
                position = (
                    cortical_area_data["coordinates"]["x"],
                    cortical_area_data["coordinates"]["y"],
                    cortical_area_data["coordinates"]["z"]
                )
            except KeyError:
                self.logger.error("Cortical area data missing required dimension or position fields")
                return False
                
            # Create the area
            self._connectome_manager.add_cortical_area(
                area_id=int(area_id) if area_id.isdigit() else area_id,
                name=cortical_area_data.get("name", f"Imported Area {area_id}"),
                area_type=cortical_area_data.get("type", "generic"),
                dimensions=dimensions,
                position=position,
                properties=cortical_area_data.get("parameters", {})
            )
            
            # TODO: Import neurons and synapses
            
            self.logger.info(f"Imported cortical area {area_id}", emoji1="📥")
            return True
        except Exception as e:
            self.logger.error(f"Error importing cortical area: {str(e)}", emoji1="❌")
            return False
    
    def update_cortical_area_properties(
        self, 
        cortical_id: str, 
        properties: Dict[str, Any]
    ) -> bool:
        """
        Update properties of a cortical area.
        
        Args:
            cortical_id: ID of the cortical area to update
            properties: Dictionary of properties to update
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Verify the cortical area exists
            if not self._connectome_manager.genome or cortical_id not in self._connectome_manager.genome.get("blueprint", {}):
                self.logger.warning(f"Cortical area {cortical_id} not found")
                return False
                
            # Check if the area is currently transforming
            if hasattr(self._connectome_manager, 'transforming_areas') and cortical_id in self._connectome_manager.transforming_areas:
                self.logger.warning(f"Cortical area {cortical_id} is currently undergoing transformation")
                return False
                
            # Handle special case for parent region ID
            if "parent_region_id" in properties and properties["parent_region_id"]:
                try:
                    from feagi.bdu.models.brain_region import change_cortical_area_parent
                    change_cortical_area_parent(
                        cortical_area_id=cortical_id, 
                        new_parent_id=properties["parent_region_id"], 
                        connectome=self._connectome_manager
                    )
                except Exception as e:
                    self.logger.error(f"Error changing parent region: {str(e)}")
                    return False
                    
            # Handle neuron count validation
            current_dims = self._connectome_manager.genome["blueprint"][cortical_id]["block_boundaries"]
            current_density = self._connectome_manager.genome["blueprint"][cortical_id]["per_voxel_neuron_cnt"]
            
            updated_dims = properties.get("cortical_dimensions", current_dims)
            updated_density = properties.get("cortical_neuron_per_vox_count", current_density)
            
            current_size = current_dims[0] * current_dims[1] * current_dims[2]
            updated_size = updated_dims[0] * updated_dims[1] * updated_dims[2]
            
            current_neuron_count = current_size * current_density
            updated_neuron_count = updated_size * updated_density
            
            # Check if we would exceed maximum neuron count
            max_neuron_count = int(self._connectome_manager.parameters.get("Limits", {}).get("max_neuron_count", 1000000))
            if self._connectome_manager.brain_stats["neuron_count"] - current_neuron_count + updated_neuron_count > max_neuron_count:
                self.logger.warning(f"Cannot update cortical area as neuron count would exceed {max_neuron_count} threshold")
                return False
                
            # Submit update to message queue for processing
            message = {'update_cortical_properties': {
                'cortical_id': cortical_id,
                **properties
            }}
            
            from feagi.api.rest.commons import api_queue
            api_queue.put(item=message)
            
            self.logger.info(f"Cortical area {cortical_id} update request submitted", emoji1="🧠")
            return True
        except Exception as e:
            self.logger.error(f"Error updating cortical area properties: {str(e)}", emoji1="❌")
            return False
    
    def update_multiple_cortical_properties(self, message) -> bool:
        """
        Update properties for multiple cortical areas at the same time
        
        Args:
            message: UpdateMultipleCorticalProperties instance containing properties to update
                    and list of cortical areas to update
            
        Returns:
            True if successful, False otherwise
            
        Raises:
            ValueError: For validation errors such as mixed area types
        """
        try:
            state_manager = get_state_manager()
            connectome = state_manager.get_connectome()
            
            cortical_id_list = message.cortical_id_list
            
            # Convert Pydantic model to dict for processing
            message_dict = message.dict(exclude_none=True)
            message_dict.pop("cortical_id_list")
            
            # Check to ensure all selected areas are of same type
            type_list = set()
            transforming = False
            for cortical_id in cortical_id_list:
                # Validate the area exists
                if cortical_id not in connectome.genome["blueprint"]:
                    raise ValueError(f"Cortical area {cortical_id} not found")
                    
                type_list.add(connectome.genome["blueprint"][cortical_id]["is_mem_type"])
                if "transforming" in connectome.genome["blueprint"][cortical_id]:
                    if connectome.genome["blueprint"][cortical_id]["transforming"]:
                        transforming = True
                        
            if len(type_list) > 1:
                raise ValueError("Memory and non-memory type cortical areas cannot be edited at the same time")
            
            if transforming:
                from feagi.api.response_templates import generate_response
                raise ValueError("One or more cortical areas are undergoing transformation")
            
            # Prepare multi-edit payload
            multi_edit_payload = []
            
            # Proceed with updates, checking neuron count limits
            for cortical_id in cortical_id_list:
                current_cortical_size = connectome.genome["blueprint"][cortical_id]["block_boundaries"][0] * \
                                       connectome.genome["blueprint"][cortical_id]["block_boundaries"][1] * \
                                       connectome.genome["blueprint"][cortical_id]["block_boundaries"][2]
                updated_cortical_size = current_cortical_size
                
                if message_dict.get("cortical_dimensions"):
                    updated_cortical_size = message.cortical_dimensions[0] * \
                                           message.cortical_dimensions[1] * \
                                           message.cortical_dimensions[2]
                
                current_neuron_density = connectome.genome["blueprint"][cortical_id]["per_voxel_neuron_cnt"]
                updated_neuron_density = current_neuron_density
                
                if message_dict.get("cortical_neuron_per_vox_count"):
                    updated_neuron_density = message.cortical_neuron_per_vox_count
                
                # Handle parent region changes using a special method
                if message_dict.get("parent_region_id"):
                    from feagi.bdu.models.brain_region import change_cortical_area_parent
                    change_cortical_area_parent(
                        cortical_area_id=cortical_id,
                        new_parent_id=message.parent_region_id,
                        connectome=connectome
                    )
                
                # Calculate neuron count change
                current_neuron_count = current_cortical_size * current_neuron_density
                updated_neuron_count = updated_cortical_size * updated_neuron_density
                
                # Check against maximum allowable neuron count
                max_allowable_neuron_count = int(connectome.parameters["Limits"]["max_neuron_count"])
                if connectome.brain_stats["neuron_count"] - current_neuron_count + updated_neuron_count > max_allowable_neuron_count:
                    max_count = max_allowable_neuron_count
                    raise ValueError(
                        f"Cannot update cortical areas as neuron count will exceed {max_count} threshold"
                    )
                
                # Add this cortical area to the payload
                cortical_payload = message_dict.copy()
                cortical_payload["cortical_id"] = cortical_id
                multi_edit_payload.append(cortical_payload)
            
            # Submit updates to message queue
            from feagi.api.rest.commons import api_queue
            message_ = {'update_multiple_cortical_properties': multi_edit_payload}
            api_queue.put(item=message_)
            
            self.logger.info(f"Submitted update request for {len(cortical_id_list)} cortical areas", emoji1="🧠")
            return True
        
        except Exception as e:
            self.logger.error(f"Error updating multiple cortical areas: {str(e)}", emoji1="❌")
            # Re-raise validation errors but catch other exceptions
            if isinstance(e, ValueError):
                raise
            return False

    def delete_multiple_cortical_areas(self, cortical_id_list: list) -> list:
        """
        Delete multiple cortical areas at the same time
        
        Args:
            cortical_id_list: List of cortical area IDs to delete
            
        Returns:
            List of cortical area IDs that were not found, or empty list if all were found
        """
        try:
            state_manager = get_state_manager()
            connectome = state_manager.get_connectome()
            
            from feagi.api.rest.commons import api_queue
            
            not_found = []
            for cortical_id in cortical_id_list:
                if cortical_id in connectome.genome["blueprint"]:
                    message = {'delete_cortical_area': cortical_id}
                    api_queue.put(item=message)
                else:
                    not_found.append(cortical_id)
            
            return not_found
        except Exception as e:
            self.logger.error(f"Error deleting multiple cortical areas: {str(e)}", emoji1="❌")
            return cortical_id_list  # Return the full list to indicate failure