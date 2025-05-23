"""Cortical Area service for managing FEAGI cortical area operations."""

import time
from typing import Dict, Any, Optional, List, Tuple
from ..shared.base_service import BaseService


class CorticalAreaService(BaseService):
    """
    Cortical Area service handles cortical area operations including
    CRUD operations, activity monitoring, and cortical area management.
    """
    
    def __init__(self, connectome_manager, state_manager=None):
        """Initialize cortical area service."""
        super().__init__(connectome_manager, state_manager)
        # Cache for frequently accessed data
        self._cortical_areas_cache = None
        self._cortical_areas_cache_timestamp = 0

    def get_all_areas(self) -> List[Dict[str, Any]]:
        """Get all cortical areas."""
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

    def get_area(self, cortical_id: str) -> Optional[Dict[str, Any]]:
        """Get a cortical area by ID."""
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

    def create_area(
        self, 
        name: str,
        coordinates: Dict[str, int],
        dimensions: Dict[str, int],
        area_type: str,
        parameters: Dict[str, Any] = None
    ) -> Optional[Dict[str, Any]]:
        """Create a new cortical area."""
        # In legacy FEAGI, this depends on a genome being loaded first
        if not self._validate_genome_loaded():
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

    def update_area(
        self,
        cortical_id: str,
        name: Optional[str] = None,
        coordinates: Optional[Dict[str, int]] = None,
        dimensions: Optional[Dict[str, int]] = None,
        area_type: Optional[str] = None,
        parameters: Optional[Dict[str, Any]] = None
    ) -> Optional[Dict[str, Any]]:
        """Update an existing cortical area."""
        # In legacy FEAGI, this depends on a genome being loaded first
        if not self._validate_genome_loaded():
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

    def delete_area(self, cortical_id: str) -> bool:
        """Delete a cortical area."""
        # In legacy FEAGI, this depends on a genome being loaded first
        if not self._validate_genome_loaded():
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

    def get_area_neurons(self, cortical_id: str) -> Optional[List[Dict[str, Any]]]:
        """Get neurons for a specific cortical area."""
        # In legacy FEAGI, this depends on a genome being loaded first
        if not self._validate_genome_loaded():
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

    def get_area_activity(self, cortical_id: str, window: int = 1) -> Optional[Dict[str, Any]]:
        """Get activity data for a specific cortical area."""
        # In legacy FEAGI, this depends on a genome being loaded first
        if not self._validate_genome_loaded():
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

    def get_area_connectivity(self, cortical_id: str, direction: str = "both") -> Optional[Dict[str, Any]]:
        """Get connectivity information for a specific cortical area."""
        # In legacy FEAGI, this depends on a genome being loaded first
        if not self._validate_genome_loaded():
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
                        "name": self._connectome_manager.cortical_areas.get(connected_area, type('obj', (object,), {"name": "Unknown"})).name
                    } 
                    for connected_area in incoming_connections
                ]
            
            if direction in ["outgoing", "both"]:
                result["outgoing_connections"] = [
                    {
                        "cortical_id": str(connected_area),
                        "name": self._connectome_manager.cortical_areas.get(connected_area, type('obj', (object,), {"name": "Unknown"})).name
                    }
                    for connected_area in outgoing_connections
                ]
            
            return result
        except Exception as e:
            self.logger.error(f"Error retrieving connectivity for cortical area {cortical_id}: {str(e)}")
            return None

    def stimulate_area(self, cortical_id: str, pattern: str = "random", 
                      intensity: float = 1.0, duration: int = 1,
                      coordinates: Optional[List[Dict[str, int]]] = None) -> Dict[str, Any]:
        """Stimulate a cortical area with the specified pattern."""
        # This is a placeholder implementation
        self.logger.info(f"stimulate_cortical_area called with cortical_id={cortical_id}, pattern={pattern}")
        return {
            "stimulated_neurons": 100,
            "timestamp": 123456789
        }

    def get_id_list(self) -> List[str]:
        """Get a list of all cortical area IDs (6-character strings) in the current genome."""
        try:
            current_genome = self._get_current_genome()
            if not current_genome or 'blueprint' not in current_genome:
                return []
                
            # Extract cortical IDs from blueprint keys
            # Blueprint keys have format: {prefix}-{cortical_id}-{suffix}
            # We want the second segment (cortical_id)
            cortical_ids = set()  # Use set to avoid duplicates
            
            for blueprint_key in current_genome['blueprint'].keys():
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

    def get_index_list(self) -> List[int]:
        """Get a list of all cortical area indices (integers) used by the FCL."""
        try:
            if not hasattr(self._connectome_manager, 'cortical_areas'):
                return []
                
            return list(self._connectome_manager.cortical_areas.keys())
        except Exception as e:
            self.logger.error(f"Error getting cortical area index list: {str(e)}")
            return []

    def get_name_list(self) -> List[str]:
        """Get a list of all cortical area names."""
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

    def get_id_name_mapping(self) -> Dict[str, str]:
        """Map every cortical area's 6-character cortical_id to its human-readable name."""
        mapping: Dict[str, str] = {}
        try:
            if not hasattr(self._connectome_manager, "cortical_areas"):
                return mapping

            for cortical_idx, area in self._connectome_manager.cortical_areas.items():
                # Prefer the explicit cortical_id stored on the CorticalArea object (if present)
                cortical_id = getattr(area, "cortical_id", None)
                if not cortical_id:
                    # Fallback: derive a placeholder ID from the integer index
                    cortical_id = f"CID{cortical_idx:03d}"

                name = getattr(area, "name", None)
                if cortical_id and name:
                    mapping[cortical_id] = name

            return mapping
        except Exception as exc:
            self.logger.error(f"Error building cortical_id→name mapping: {exc}")
            return mapping

    def get_2d_locations(self) -> Dict[str, List[int]]:
        """Get 2D locations of all cortical areas."""
        try:
            locations = {}
            current_genome = self._get_current_genome()
            if not current_genome or 'blueprint' not in current_genome:
                return locations
                
            # Group cortical areas by cortical_id to extract their 2D coordinates
            cortical_data = {}
            for blueprint_key, area_data in current_genome['blueprint'].items():
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

    def refresh_cache(self):
        """Refresh cached data when state changes occur."""
        # Clear caches so they'll be rebuilt on next access
        self._cortical_areas_cache = None
        self._cortical_areas_cache_timestamp = 0 