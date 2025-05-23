"""Cortical Area service for managing FEAGI cortical area operations."""

import time
from typing import Dict, Any, Optional, List, Tuple
from ..shared.base_service import BaseService


class CorticalAreaService(BaseService):
    """
    Cortical Area service handles cortical area operations including
    CRUD operations, activity monitoring, and cortical area management.
    
    IMPORTANT: This service maintains the critical distinction between:
    - cortical_id: 6-character string identifier (e.g., "iv00_C", "motor1")
    - cortical_idx: integer index used internally by connectome manager (e.g., 0, 1, 2)
    """
    
    def __init__(self, connectome_manager, state_manager=None):
        """Initialize cortical area service."""
        super().__init__(connectome_manager, state_manager)
        # Cache for frequently accessed data
        self._cortical_areas_cache = None
        self._cortical_areas_cache_timestamp = 0

    def _get_cortical_idx_for_id(self, cortical_id: str) -> Optional[int]:
        """
        Map a cortical_id (6-character string) to its corresponding cortical_idx (integer).
        
        Args:
            cortical_id: 6-character string identifier
            
        Returns:
            Integer index if found, None otherwise
        """
        try:
            if not hasattr(self._connectome_manager, 'cortical_areas'):
                return None
                
            for cortical_idx, area in self._connectome_manager.cortical_areas.items():
                if hasattr(area, 'cortical_id') and area.cortical_id == cortical_id:
                    return cortical_idx
            return None
        except Exception as e:
            self.logger.error(f"Error mapping cortical_id {cortical_id} to cortical_idx: {str(e)}")
            return None

    def _get_cortical_id_for_idx(self, cortical_idx: int) -> Optional[str]:
        """
        Map a cortical_idx (integer) to its corresponding cortical_id (6-character string).
        
        Args:
            cortical_idx: Integer index
            
        Returns:
            6-character string identifier if found, None otherwise
        """
        try:
            if not hasattr(self._connectome_manager, 'cortical_areas'):
                return None
                
            area = self._connectome_manager.cortical_areas.get(cortical_idx)
            if area and hasattr(area, 'cortical_id'):
                return area.cortical_id
            return None
        except Exception as e:
            self.logger.error(f"Error mapping cortical_idx {cortical_idx} to cortical_id: {str(e)}")
            return None

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
            for cortical_idx, area in self._connectome_manager.cortical_areas.items():
                try:
                    # Get neuron count (safely)
                    try:
                        neuron_count = len(self._connectome_manager.get_neurons_by_area(cortical_idx))
                    except Exception:
                        neuron_count = 0
                        
                    # Use cortical_id if available, otherwise fall back to string of cortical_idx
                    area_id = getattr(area, 'cortical_id', str(cortical_idx))
                        
                    # Convert to API format
                    result.append({
                        "id": area_id,  # Use cortical_id (6-char string) or fallback
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
                    self.logger.error(f"Error converting area {cortical_idx} to API format: {str(e)}")
                    
        except Exception as e:
            self.logger.error(f"Error retrieving cortical areas: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
        
        # Cache the result
        self._cortical_areas_cache = result
        self._cortical_areas_cache_timestamp = time.time()
        
        return result

    def get_area(self, cortical_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a cortical area by its cortical_id (6-character string).
        
        Args:
            cortical_id: 6-character string identifier
            
        Returns:
            Area information dictionary or None if not found
        """
        try:
            # Map cortical_id to cortical_idx
            cortical_idx = self._get_cortical_idx_for_id(cortical_id)
            if cortical_idx is None:
                self.logger.warning(f"Cortical area with cortical_id '{cortical_id}' not found")
                return None
            
            # Get the area from connectome manager using the integer index
            area = self._connectome_manager.cortical_areas.get(cortical_idx)
            if not area:
                self.logger.warning(f"Cortical area with cortical_idx {cortical_idx} not found in connectome")
                return None
            
            # Return area information
            neuron_count = len(self._connectome_manager.get_neurons_by_area(cortical_idx))
            
            # Format response
            return {
                "id": cortical_id,  # Return the original cortical_id
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
            self.logger.error(f"Error retrieving cortical area with cortical_id '{cortical_id}': {str(e)}")
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
        """
        Update an existing cortical area.
        
        Args:
            cortical_id: 6-character string identifier
            name: New name (optional)
            coordinates: New coordinates (optional)
            dimensions: New dimensions (optional)
            area_type: New area type (optional)
            parameters: New parameters (optional)
            
        Returns:
            Updated area information or None if not found
        """
        # In legacy FEAGI, this depends on a genome being loaded first
        if not self._validate_genome_loaded():
            self.logger.warning("No genome loaded, cannot update cortical area")
            return None
        
        # Map cortical_id to cortical_idx
        cortical_idx = self._get_cortical_idx_for_id(cortical_id)
        if cortical_idx is None:
            self.logger.warning(f"Cortical area with cortical_id '{cortical_id}' not found for update")
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
                "cortical_id": cortical_id,  # Return original cortical_id
                "cortical_idx": cortical_idx,  # Also include cortical_idx for reference
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
            self.logger.error(f"Error updating cortical area with cortical_id '{cortical_id}': {str(e)}")
            return None

    def delete_area(self, cortical_id: str) -> bool:
        """
        Delete a cortical area.
        
        Args:
            cortical_id: 6-character string identifier
            
        Returns:
            True if successfully deleted, False otherwise
        """
        # In legacy FEAGI, this depends on a genome being loaded first
        if not self._validate_genome_loaded():
            self.logger.warning("No genome loaded, cannot delete cortical area")
            return False
        
        # Map cortical_id to cortical_idx
        cortical_idx = self._get_cortical_idx_for_id(cortical_id)
        if cortical_idx is None:
            self.logger.warning(f"Cortical area with cortical_id '{cortical_id}' not found for deletion")
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
            self.logger.error(f"Error deleting cortical area with cortical_id '{cortical_id}': {str(e)}")
            return False

    def get_area_neurons(self, cortical_id: str) -> Optional[List[Dict[str, Any]]]:
        """
        Get neurons for a specific cortical area.
        
        Args:
            cortical_id: 6-character string identifier
            
        Returns:
            List of neuron information or None if area not found
        """
        # In legacy FEAGI, this depends on a genome being loaded first
        if not self._validate_genome_loaded():
            self.logger.warning("No genome loaded, cannot retrieve cortical area neurons")
            return None
        
        # Map cortical_id to cortical_idx
        cortical_idx = self._get_cortical_idx_for_id(cortical_id)
        if cortical_idx is None:
            self.logger.warning(f"Cortical area with cortical_id '{cortical_id}' not found for neuron retrieval")
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
            self.logger.error(f"Error retrieving neurons for cortical area with cortical_id '{cortical_id}': {str(e)}")
            return None

    def get_area_activity(self, cortical_id: str, window: int = 1) -> Optional[Dict[str, Any]]:
        """
        Get activity data for a specific cortical area.
        
        Args:
            cortical_id: 6-character string identifier
            window: Time window for activity analysis
            
        Returns:
            Activity information or None if area not found
        """
        # In legacy FEAGI, this depends on a genome being loaded first
        if not self._validate_genome_loaded():
            self.logger.warning("No genome loaded, cannot retrieve cortical area activity")
            return None
        
        # Map cortical_id to cortical_idx
        cortical_idx = self._get_cortical_idx_for_id(cortical_id)
        if cortical_idx is None:
            self.logger.warning(f"Cortical area with cortical_id '{cortical_id}' not found for activity retrieval")
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
            self.logger.error(f"Error retrieving activity for cortical area with cortical_id '{cortical_id}': {str(e)}")
            return None

    def get_area_connectivity(self, cortical_id: str, direction: str = "both") -> Optional[Dict[str, Any]]:
        """
        Get connectivity information for a cortical area.
        
        Args:
            cortical_id: 6-character string identifier
            direction: "incoming", "outgoing", or "both"
            
        Returns:
            Connectivity information or None if area not found
        """
        try:
            # Validate genome is loaded
            if not self._validate_genome_loaded():
                raise ValueError("No genome loaded - cannot retrieve cortical area connectivity")
            
            # Map cortical_id to cortical_idx
            cortical_idx = self._get_cortical_idx_for_id(cortical_id)
            if cortical_idx is None:
                raise ValueError(f"Cortical area with cortical_id '{cortical_id}' not found")
            
            # Validate direction parameter
            if direction not in ["incoming", "outgoing", "both"]:
                raise ValueError(f"Invalid direction: {direction}. Must be 'incoming', 'outgoing', or 'both'")
            
            # Check if the cortical area exists
            if not hasattr(self._connectome_manager, 'cortical_areas') or cortical_idx not in self._connectome_manager.cortical_areas:
                raise ValueError(f"Cortical area with cortical_idx {cortical_idx} not found in connectome")
            
            # Get neurons in the area
            neuron_ids = self._connectome_manager.get_neurons_by_area(cortical_idx)
            if not neuron_ids:
                raise ValueError(f"No neurons found in cortical area with cortical_id '{cortical_id}'")
            
            incoming_connections = set()
            outgoing_connections = set()
            
            # Get incoming connections
            if direction in ["incoming", "both"]:
                for neuron_id in neuron_ids:
                    connections = self._connectome_manager.get_incoming_connections(neuron_id)
                    for pre_id, _ in connections:
                        # Skip connections within the same area
                        pre_area_idx = self._connectome_manager._neuron_to_area.get(pre_id)
                        if pre_area_idx is not None and pre_area_idx != cortical_idx:
                            incoming_connections.add(pre_area_idx)
        
            # Get outgoing connections
            if direction in ["outgoing", "both"]:
                for neuron_id in neuron_ids:
                    connections = self._connectome_manager.get_outgoing_connections(neuron_id)
                    for post_id, _ in connections:
                        # Skip connections within the same area
                        post_area_idx = self._connectome_manager._neuron_to_area.get(post_id)
                        if post_area_idx is not None and post_area_idx != cortical_idx:
                            outgoing_connections.add(post_area_idx)
            
            # Format results - only include areas that actually exist
            result = {
                "cortical_id": cortical_id,  # Return original cortical_id
                "direction": direction
            }
            
            if direction in ["incoming", "both"]:
                incoming_list = []
                for connected_area_idx in incoming_connections:
                    area_obj = self._connectome_manager.cortical_areas.get(connected_area_idx)
                    connected_cortical_id = self._get_cortical_id_for_idx(connected_area_idx)
                    if area_obj and hasattr(area_obj, 'name') and area_obj.name and connected_cortical_id:
                        incoming_list.append({
                            "cortical_id": connected_cortical_id,  # Use proper cortical_id
                            "name": area_obj.name
                        })
                    else:
                        # If no valid name or cortical_id, don't include this connection
                        self.logger.warning(f"Skipping connection to area {connected_area_idx} - no valid name or cortical_id")
                result["incoming_connections"] = incoming_list
            
            if direction in ["outgoing", "both"]:
                outgoing_list = []
                for connected_area_idx in outgoing_connections:
                    area_obj = self._connectome_manager.cortical_areas.get(connected_area_idx)
                    connected_cortical_id = self._get_cortical_id_for_idx(connected_area_idx)
                    if area_obj and hasattr(area_obj, 'name') and area_obj.name and connected_cortical_id:
                        outgoing_list.append({
                            "cortical_id": connected_cortical_id,  # Use proper cortical_id
                            "name": area_obj.name
                        })
                    else:
                        # If no valid name or cortical_id, don't include this connection
                        self.logger.warning(f"Skipping connection to area {connected_area_idx} - no valid name or cortical_id")
                result["outgoing_connections"] = outgoing_list
            
            return result
        except Exception as e:
            self.logger.error(f"Error retrieving connectivity for cortical area with cortical_id '{cortical_id}': {str(e)}")
            raise ValueError(f"Failed to retrieve connectivity: {str(e)}")

    def stimulate_area(self, cortical_id: str, pattern: str = "random", 
                      intensity: float = 1.0, duration: int = 1,
                      coordinates: Optional[List[Dict[str, int]]] = None) -> Dict[str, Any]:
        """
        Stimulate a cortical area with the specified pattern.
        
        Args:
            cortical_id: 6-character string identifier
            pattern: Stimulation pattern
            intensity: Stimulation intensity
            duration: Stimulation duration
            coordinates: Optional coordinate list
            
        Returns:
            Stimulation result
        """
        # Validate genome is loaded
        if not self._validate_genome_loaded():
            raise ValueError("No genome loaded - cannot stimulate cortical area")
        
        # This should be implemented properly, not return placeholder data
        raise NotImplementedError("Cortical area stimulation is not yet implemented")

    def get_id_list(self) -> List[str]:
        """
        Get a list of all cortical area IDs (6-character strings) in the current genome.
        
        Returns:
            List of cortical area ID strings
        """
        try:
            # CRITICAL: Validate genome is loaded and state is consistent
            if not self._validate_genome_loaded():
                self.logger.warning("No genome loaded - cannot retrieve cortical area ID list")
                return []
            
            # Validate and sync state if needed
            state_is_consistent = self._validate_state_consistency()
            if not state_is_consistent:
                self.logger.warning("State inconsistency detected in cortical area service, attempting to synchronize")
                sync_success = self._sync_state_if_needed()
                if not sync_success:
                    self.logger.error("Failed to synchronize state in cortical area service")
                    # Try to continue anyway
                else:
                    self.logger.info("State synchronization successful in cortical area service")
            
            # Get genome data from state manager
            genome_data = self._get_current_genome()
            if not genome_data or 'blueprint' not in genome_data:
                self.logger.warning("No valid genome blueprint found")
                return []
                
            # Extract cortical IDs from blueprint keys
            # Blueprint keys have format: {prefix}-{cortical_id}-{suffix}
            # We want the second segment (cortical_id)
            cortical_ids = set()  # Use set to avoid duplicates
            
            for blueprint_key in genome_data['blueprint'].keys():
                # Split by dash and take the second segment
                parts = blueprint_key.split('-')
                if len(parts) >= 2:
                    cortical_id = parts[1]
                    cortical_ids.add(cortical_id)
                    
            # Return sorted list of unique cortical IDs
            result = sorted(list(cortical_ids))
            self.logger.debug(f"Retrieved {len(result)} cortical area IDs from genome blueprint")
            return result
            
        except Exception as e:
            self.logger.error(f"Error getting cortical area ID list: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
            return []

    def get_index_list(self) -> List[int]:
        """Get a list of all cortical area indices (integers) used by the FCL."""
        try:
            # Validate genome is loaded
            if not self._validate_genome_loaded():
                raise ValueError("No genome loaded - cannot retrieve cortical area indices")
            
            if not hasattr(self._connectome_manager, 'cortical_areas'):
                raise ValueError("Connectome manager has no cortical areas")
            
            indices = list(self._connectome_manager.cortical_areas.keys())
            if not indices:
                raise ValueError("No cortical area indices found in connectome")
                
            return sorted(indices)
        except Exception as e:
            self.logger.error(f"Error getting cortical area index list: {str(e)}")
            raise ValueError(f"Failed to retrieve cortical area indices: {str(e)}")

    def get_name_list(self) -> List[str]:
        """Get a list of all cortical area names."""
        try:
            # Validate genome is loaded
            if not self._validate_genome_loaded():
                raise ValueError("No genome loaded - cannot retrieve cortical area names")
            
            if not hasattr(self._connectome_manager, 'cortical_areas'):
                raise ValueError("Connectome manager has no cortical areas")
                
            names = []
            for area in self._connectome_manager.cortical_areas.values():
                if hasattr(area, 'name') and area.name:
                    names.append(area.name)
            
            if not names:
                raise ValueError("No cortical areas with valid names found in connectome")
            
            return sorted(names)
        except Exception as e:
            self.logger.error(f"Error getting cortical area name list: {str(e)}")
            raise ValueError(f"Failed to retrieve cortical area names: {str(e)}")

    def get_id_name_mapping(self) -> Dict[str, str]:
        """Map every cortical area's 6-character cortical_id to its human-readable name."""
        try:
            # Validate genome is loaded
            if not self._validate_genome_loaded():
                raise ValueError("No genome loaded - cannot retrieve cortical area ID mapping")
            
            if not hasattr(self._connectome_manager, "cortical_areas"):
                raise ValueError("Connectome manager has no cortical areas")

            mapping: Dict[str, str] = {}
            for cortical_idx, area in self._connectome_manager.cortical_areas.items():
                # Only include areas that have both a valid cortical_id and name
                cortical_id = getattr(area, "cortical_id", None)
                name = getattr(area, "name", None)
                
                if cortical_id and name:
                    mapping[cortical_id] = name
                else:
                    self.logger.warning(f"Skipping area {cortical_idx} - missing cortical_id or name")

            if not mapping:
                raise ValueError("No cortical areas with valid ID and name mappings found")

            return mapping
        except Exception as exc:
            self.logger.error(f"Error building cortical_id→name mapping: {exc}")
            raise ValueError(f"Failed to retrieve cortical area ID mapping: {str(exc)}")

    def get_2d_locations(self) -> Dict[str, List[int]]:
        """Get 2D locations of all cortical areas."""
        try:
            # Validate genome is loaded
            if not self._validate_genome_loaded():
                raise ValueError("No genome loaded - cannot retrieve cortical area locations")
            
            current_genome = self._get_current_genome()
            if not current_genome or 'blueprint' not in current_genome:
                raise ValueError("No valid genome blueprint found")
                
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
            
            if not cortical_data:
                raise ValueError("No cortical area data found in genome blueprint")
            
            # Extract 2D coordinates for each cortical area
            locations = {}
            for cortical_id, properties in cortical_data.items():
                x_coord = properties.get('2dcorx')
                y_coord = properties.get('2dcory')
                
                # Only include if both coordinates are present and valid
                if x_coord is not None and y_coord is not None:
                    try:
                        x_coord = int(x_coord)
                        y_coord = int(y_coord)
                        locations[cortical_id] = [x_coord, y_coord]
                    except (ValueError, TypeError):
                        self.logger.warning(f"Invalid coordinates for cortical area {cortical_id}: x={x_coord}, y={y_coord}")
                else:
                    self.logger.warning(f"Missing 2D coordinates for cortical area {cortical_id}")
            
            if not locations:
                raise ValueError("No valid 2D coordinates found for any cortical areas")
            
            return locations
        except Exception as e:
            self.logger.error(f"Error getting cortical 2D locations: {str(e)}")
            raise ValueError(f"Failed to retrieve cortical area 2D locations: {str(e)}")

    def refresh_cache(self):
        """Refresh cached data when state changes occur."""
        # Clear caches so they'll be rebuilt on next access
        self._cortical_areas_cache = None
        self._cortical_areas_cache_timestamp = 0 