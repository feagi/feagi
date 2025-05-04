"""Core API service implementation for FEAGI."""

from typing import Dict, Any, List, Optional, Tuple, Union
import logging
import os
import json
import tempfile
from datetime import datetime
from time import time
from pathlib import Path

import numpy as np

from feagi.core.feagi import FEAGI
from feagi.bdu.neuroembryogenesis import Neuroembryogenesis, develop_brain_from_genome
from feagi.bdu.connectome_manager import ConnectomeManager, CorticalArea
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
    # Fallback implementations if the imports fail
    def genome_validator(genome):
        """Validate a genome."""
        return True
        
    def save_genome(genome, file_name=''):
        """Save a genome to a file."""
        if file_name:
            with open(file_name, 'w') as f:
                json.dump(genome, f, indent=2)
        return True
        
    def merge_core_morphologies(genome):
        """Merges core morphologies into the genome."""
        return genome
        
    def genome_morphology_updator(genome):
        """Updates morphologies in the genome."""
        return genome
        
    def genome_physiology_updator(genome):
        """Updates physiology in the genome."""
        if "physiology" not in genome:
            genome["physiology"] = {}
        return genome
        
    def genome_stat_updator(genome):
        """Updates stats in the genome."""
        if "stats" not in genome:
            genome["stats"] = {}
        return genome

class CoreAPIService:
    """
    Core API Service for FEAGI.
    
    This class provides the internal API interfaces to FEAGI's core functionality.
    It acts as a bridge between the external interfaces (REST, ZMQ) and the 
    FEAGI core components.
    """
    
    def __init__(self, feagi_instance: Optional[FEAGI] = None):
        """
        Initialize the Core API Service.
        
        Args:
            feagi_instance: An optional FEAGI instance. If not provided,
                            a new instance will be created.
        """
        self.logger = logging.getLogger(__name__)
        self._feagi = feagi_instance or FEAGI()
        self._temp_dir = tempfile.mkdtemp(prefix="feagi_")
        self._genome_filename = None
        self._pending_amalgamation = {}
        self._amalgamation_history = {}
        
        # Initialize the connectome manager
        self._connectome_manager = ConnectomeManager()
        
        # Initialize the neuroembryogenesis module
        self._neuroembryogenesis = Neuroembryogenesis(
            connectome_manager=self._connectome_manager,
            progress_callback=self._handle_embryogenesis_progress
        )
        
        # Current genome state
        self._current_genome = None
        
        # Add a sample cortical area for testing
        if not self._connectome_manager._areas:
            try:
                self._connectome_manager.add_cortical_area(
                    area_id=1,
                    name="Test Area",
                    area_type="interconnect",
                    dimensions=(10, 10, 5),
                    position=(0, 0, 0),
                    properties={"test": True}
                )
            except Exception as e:
                self.logger.warning(f"Failed to create test cortical area: {str(e)}")
        
    def _handle_embryogenesis_progress(self, stage, percentage, message):
        """Handle progress updates from the neuroembryogenesis process."""
        self.logger.info(f"[{stage}] {percentage:.1f}% - {message}")
        
    @property
    def feagi(self) -> FEAGI:
        """Get the FEAGI instance."""
        return self._feagi
        
    def get_burst_engine(self):
        """Get the Burst Engine component."""
        # For now, return None as this component isn't fully implemented
        return None
        
    def get_connectome_manager(self):
        """Get the Connectome Manager component."""
        return self._connectome_manager
        
    def get_fcl_manager(self):
        """Get the FCL Manager component."""
        # For now, return None as this component isn't fully implemented
        return None
        
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
        # In legacy FEAGI, this depends on a genome being loaded first
        if self._current_genome is None:
            self.logger.warning("No genome loaded, cannot retrieve cortical areas")
            return []
        
        result = []
        try:
            for area_id, area in self._connectome_manager._areas.items():
                # Convert from internal representation to API format
                neuron_count = len(self._connectome_manager.get_neurons_by_area(area_id))
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
            self.logger.error(f"Error retrieving cortical areas: {str(e)}")
        
        return result
        
    def get_cortical_area(self, area_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a cortical area by ID.
        
        Args:
            area_id: ID of the cortical area.
            
        Returns:
            Dictionary containing cortical area information, or None if not found.
        """
        # In legacy FEAGI, this depends on a genome being loaded first
        if self._current_genome is None:
            self.logger.warning("No genome loaded, cannot retrieve cortical area")
            return None
        
        try:
            area_id_int = int(area_id)
        except ValueError:
            return None
        
        try:
            area = self._connectome_manager._areas.get(area_id_int)
            if not area:
                return None
            
            # Convert to API format
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
            self.logger.error(f"Error retrieving cortical area {area_id}: {str(e)}")
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
            existing_ids = set(self._connectome_manager._areas.keys())
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
            area = self._connectome_manager._areas.get(area_id_int)
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
            area_id: ID of the cortical area to delete.
            
        Returns:
            True if the cortical area was deleted, False otherwise.
        """
        # In legacy FEAGI, this depends on a genome being loaded first
        if self._current_genome is None:
            self.logger.warning("No genome loaded, cannot delete cortical area")
            return False
        
        try:
            area_id_int = int(area_id)
        except ValueError:
            return False
        
        try:
            if area_id_int not in self._connectome_manager._areas:
                return False
            
            # Get all neurons in this area
            neurons = self._connectome_manager.get_neurons_by_area(area_id_int)
            
            # Delete all neurons in the area
            for neuron_id in neurons:
                self._connectome_manager.delete_neuron(neuron_id)
            
            # Remove the area
            del self._connectome_manager._areas[area_id_int]
            
            # Clean up any area-specific data structures
            if area_id_int in self._connectome_manager._occupied_voxels:
                del self._connectome_manager._occupied_voxels[area_id_int]
            
            if area_id_int in self._connectome_manager._area_lookup_tables:
                del self._connectome_manager._area_lookup_tables[area_id_int]
            
            # Remove from area classification sets
            if area_id_int in self._connectome_manager._small_regular_areas:
                self._connectome_manager._small_regular_areas.remove(area_id_int)
            
            if area_id_int in self._connectome_manager._large_regular_areas:
                self._connectome_manager._large_regular_areas.remove(area_id_int)
            
            if area_id_int in self._connectome_manager._extreme_dimension_areas:
                self._connectome_manager._extreme_dimension_areas.remove(area_id_int)
            
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
            if area_id_int not in self._connectome_manager._areas:
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
            if area_id_int not in self._connectome_manager._areas:
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
            if area_id_int not in self._connectome_manager._areas:
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
                        "name": self._connectome_manager._areas.get(connected_area, CorticalArea(connected_area, "Unknown", "unknown", (0, 0, 0), (0, 0, 0))).name
                    } 
                    for connected_area in incoming_connections
                ]
            
            if direction in ["outgoing", "both"]:
                result["outgoing_connections"] = [
                    {
                        "area_id": str(connected_area),
                        "name": self._connectome_manager._areas.get(connected_area, CorticalArea(connected_area, "Unknown", "unknown", (0, 0, 0), (0, 0, 0))).name
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
            if area_id_int not in self._connectome_manager._areas:
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
            "timestamp": time()
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
            "timestamp": time()
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
            "timestamp": time()
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
            "timestamp": time()
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
            
        # Then use the default location - correct path to the feagi/evo/defaults directory
        feagi_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
        return os.path.join(feagi_path, "evo", "defaults")
    
    def get_temp_path(self) -> str:
        """
        Get the path to the temporary directory.
        
        Returns:
            Path to the temporary directory.
        """
        return self._temp_dir
    
    # Genome methods
    
    def load_genome(self, genome_data: Dict[str, Any], filename: Optional[str] = None) -> bool:
        """
        Load a genome into FEAGI.
        
        Args:
            genome_data: Dictionary containing the genome data.
            filename: Optional filename for the genome.
            
        Returns:
            True if successful, False otherwise.
        """
        try:
            self.logger.info(f"Loading genome: {filename}")
            
            # Store the genome filename
            self._genome_filename = filename
            
            # Validate the genome
            is_valid = genome_validator(genome_data)
            if not is_valid:
                self.logger.error("Invalid genome format")
                return False
                
            # Process and update the genome
            genome_data = merge_core_morphologies(genome_data)
            genome_data = genome_morphology_updator(genome_data)
            genome_data = genome_physiology_updator(genome_data)
            genome_data = genome_stat_updator(genome_data)
            
            # Store the current genome in memory
            self._current_genome = genome_data
            
            # Save the genome to a temporary file to load it with neuroembryogenesis
            genome_path = os.path.join(self._temp_dir, filename or "current_genome.json")
            with open(genome_path, 'w') as f:
                json.dump(genome_data, f, indent=2)
                
            # Attempt to develop the brain from this genome
            # This is a more complex operation that we might want to make optional
            # or run asynchronously in a real implementation
            success, stats = develop_brain_from_genome(
                genome_path=genome_path,
                connectome_manager=self._connectome_manager
            )
            
            if success:
                self.logger.info(f"Successfully developed brain from genome: {stats}")
            else:
                self.logger.warning(f"Brain development completed with warnings: {stats}")
                
            return True
                
        except Exception as e:
            self.logger.error(f"Error loading genome: {str(e)}")
            return False
    
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
        return self._genome_filename
    
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
    
    def initiate_amalgamation(
        self,
        amalgamation_id: str,
        genome_id: str,
        genome_title: str,
        genome_payload: Dict[str, Any]
    ) -> bool:
        """
        Initiate an amalgamation with a genome payload.
        
        Args:
            amalgamation_id: ID for the amalgamation.
            genome_id: ID of the genome.
            genome_title: Title of the genome.
            genome_payload: The genome data.
            
        Returns:
            True if successful, False otherwise.
        """
        try:
            # Add the amalgamation to pending
            self._pending_amalgamation[amalgamation_id] = {
                "id": amalgamation_id,
                "status": "pending",
                "genome_id": genome_id,
                "genome_title": genome_title,
                "payload": genome_payload
            }
            
            # Save the amalgamation genome to a file
            amal_path = os.path.join(self._temp_dir, f"amalgamation_{amalgamation_id}.json")
            with open(amal_path, 'w') as f:
                json.dump(genome_payload, f, indent=2)
                
            return True
        except Exception as e:
            self.logger.error(f"Error initiating amalgamation: {str(e)}")
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
        Get the amalgamation history.
        
        Returns:
            Dictionary of amalgamation IDs to their statuses.
        """
        return self._amalgamation_history
    
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
        circuit_origin: Tuple[int, int, int],
        brain_region_id: str,
        rewire_mode: str
    ) -> bool:
        """
        Complete an amalgamation.
        
        Args:
            amalgamation_id: ID of the amalgamation.
            circuit_origin: Tuple of (x, y, z) coordinates for the circuit origin.
            brain_region_id: ID of the brain region.
            rewire_mode: Mode for rewiring connections.
            
        Returns:
            True if successful, False otherwise.
        """
        try:
            # Check if the amalgamation exists
            if amalgamation_id not in self._pending_amalgamation:
                self.logger.error(f"Amalgamation not found: {amalgamation_id}")
                return False
                
            # Get the amalgamation data
            amalgamation = self._pending_amalgamation[amalgamation_id]
            genome_payload = amalgamation["payload"]
            
            # Apply the amalgamation to the current genome
            # This would be a complex process in a real implementation
            # For now, we'll just update the amalgamation status
            self._amalgamation_history[amalgamation_id] = "completed"
            
            # Remove from pending
            del self._pending_amalgamation[amalgamation_id]
            
            return True
        except Exception as e:
            self.logger.error(f"Error completing amalgamation: {str(e)}")
            
            # Update history with error
            self._amalgamation_history[amalgamation_id] = "failed"
            
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
    
    def cancel_amalgamation(self, amalgamation_id: str) -> bool:
        """
        Cancel a pending amalgamation.
        
        Args:
            amalgamation_id: ID of the amalgamation.
            
        Returns:
            True if successful, False otherwise.
        """
        try:
            # Check if the amalgamation exists
            if amalgamation_id not in self._pending_amalgamation:
                self.logger.error(f"Amalgamation not found: {amalgamation_id}")
                return False
                
            # Update history
            self._amalgamation_history[amalgamation_id] = "cancelled"
            
            # Remove from pending
            del self._pending_amalgamation[amalgamation_id]
            
            return True
        except Exception as e:
            self.logger.error(f"Error cancelling amalgamation: {str(e)}")
            return False
    
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
        
        # In a real implementation, this might come from a configuration file
        # or be derived from the genome
        return {
            "types": [
                "ipu",       # Input Processing Unit
                "opu",       # Output Processing Unit
                "interconnect", # Interconnection
                "memory",    # Memory
                "sensory",   # Sensory processing
                "motor",     # Motor control
                "association", # Association areas
                "prefrontal", # Prefrontal cortex
                "custom"     # Custom area type
            ],
            "parameters": {
                "ipu": ["modality", "input_channels", "mapping"],
                "opu": ["modality", "output_channels", "mapping"],
                "memory": ["capacity", "decay_rate", "association_threshold"],
                "custom": []  # Custom areas can have any parameters
            }
        }
    
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
        # This is a placeholder implementation
        self.logger.info("get_burst_engine_config called")
        return {
            "burst_duration": 10,
            "refractory_period": 5,
            "threshold": 0.5,
            "decay_rate": 0.1,
            "firing_threshold": 0.7,
            "membrane_potential_decay": 0.05
        }
        
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