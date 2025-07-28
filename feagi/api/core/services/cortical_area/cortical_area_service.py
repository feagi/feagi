"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""Cortical Area service for managing FEAGI cortical area operations."""

import time
from typing import Any, Dict, List, Optional

from ..shared.base_service import BaseService


class CorticalAreaService(BaseService):
    """
    Cortical Area service handles cortical area operations including
    CRUD operations, activity monitoring, and cortical area management.

    IMPORTANT: This service maintains the critical distinction between:
    - cortical_id: 6-character string identifier (e.g., "iv00_C", "motor1")
    - cortical_idx: integer index used internally by connectome manager (e.g., 0, 1, 2)
    """

    def __init__(self, connectome_manager, state_manager=None, genome_service=None):
        """Initialize cortical area service."""
        super().__init__(connectome_manager, state_manager)
        self._genome_service = genome_service
        # Cache for frequently accessed data
        self._cortical_areas_cache = None
        self._cortical_areas_cache_timestamp = 0

    def _validate_state_consistency(self) -> bool:
        """
        Validate that the state manager and connectome manager are consistent.

        Returns:
            bool: True if state is consistent, False if inconsistencies are detected
        """
        try:
            # Check if state manager says genome is loaded
            state_manager_loaded = (
                self.state_manager and self.state_manager.is_genome_loaded()
            )

            # Check if connectome actually has cortical areas
            connectome_has_areas = (
                hasattr(self._connectome_manager, "cortical_areas")
                and self._connectome_manager.cortical_areas
                and len(self._connectome_manager.cortical_areas) > 0
            )

            # If both agree, state is consistent
            if state_manager_loaded == connectome_has_areas:
                return True

            # Log the inconsistency for debugging
            self.logger.debug(
                f"State inconsistency detected: state_manager_loaded={state_manager_loaded}, connectome_has_areas={connectome_has_areas}"
            )
            return False

        except Exception as e:
            self.logger.warning(f"Error checking state consistency: {e}")
            return False

    def _sync_state_if_needed(self) -> bool:
        """
        REMOVED: Pre-existing connectome fallback mechanism.

        This method previously attempted to retroactively set genome state to LOADED
        when finding existing cortical areas. This bypassed proper embryogenesis
        and caused neurogenesis corruption by loading connectomes without BiDirectionalCorticalMap
        synchronization.

        ARCHITECTURE COMPLIANCE: All brain development must go through proper neuroembryogenesis.
        No fallbacks that bypass the validated genome loading process are permitted.

        Returns:
            bool: Always False - no synchronization should occur
        """
        self.logger.warning(
            "_sync_state_if_needed() called - this fallback mechanism has been REMOVED"
        )
        self.logger.warning(
            "All brain development must go through proper neuroembryogenesis"
        )
        return False

    def _get_cortical_idx_for_id(self, cortical_id: str) -> Optional[int]:
        """
        Map a cortical_id (6-character string) to its corresponding cortical_idx (integer).
        Uses O(1) BiDirectionalCorticalMap instead of O(N) linear search.

        Args:
            cortical_id: 6-character string identifier

        Returns:
            Integer index if found, None otherwise
        """
        try:
            # Use the connectome manager's O(1) mapping method
            return self._connectome_manager.cortical_mapping.get_idx(cortical_id)
        except Exception as e:
            self.logger.error(
                f"Error mapping cortical_id {cortical_id} to cortical_idx: {str(e)}"
            )
            return None

    def _get_cortical_id_for_idx(self, cortical_idx: int) -> Optional[str]:
        """
        Map a cortical_idx (integer) to its corresponding cortical_id (6-character string).
        Uses O(1) BiDirectionalCorticalMap instead of O(N) linear search.

        Args:
            cortical_idx: Integer index

        Returns:
            6-character string identifier if found, None otherwise
        """
        # Use O(1) lookup from BiDirectionalCorticalMap - no more O(N) disaster!
        return self._connectome_manager.get_cortical_id_for_idx(cortical_idx)

    def get_all_areas(self) -> List[Dict[str, Any]]:
        """Get all cortical areas from ConnectomeManager (single source of truth)."""
        # Check if we can use the cached version
        if self._cortical_areas_cache is not None:
            return self._cortical_areas_cache

        try:
            # ARCHITECTURE COMPLIANCE: Use ConnectomeManager as single source of truth
            result = self._connectome_manager.get_all_cortical_area_properties()
            # Filter out empty dictionaries
            result = [area for area in result if area]
            self.logger.debug(f"Retrieved {len(result)} areas from ConnectomeManager")

        except Exception as e:
            self.logger.error(
                f"Error retrieving cortical areas from ConnectomeManager: {str(e)}"
            )
            import traceback

            self.logger.error(traceback.format_exc())
            result = []

        # Cache the result
        self._cortical_areas_cache = result
        self._cortical_areas_cache_timestamp = time.time()

        return result

    def get_area(self, cortical_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a cortical area by its cortical_id from ConnectomeManager (single source of truth).

        Args:
            cortical_id: 6-character string identifier

        Returns:
            Area information dictionary or None if not found
        """
        try:
            # ARCHITECTURE COMPLIANCE: Use ConnectomeManager as single source of truth
            return self._connectome_manager.get_cortical_area_properties(cortical_id)

        except KeyError:
            self.logger.warning(
                f"Cortical area with cortical_id '{cortical_id}' not found in ConnectomeManager"
            )
            return None
        except Exception as e:
            self.logger.error(
                f"Error retrieving cortical area with cortical_id '{cortical_id}': {str(e)}"
            )
            import traceback

            self.logger.error(traceback.format_exc())
            return None

    def create_area(
        self,
        name: str,
        coordinates: Dict[str, int],
        dimensions: Dict[str, int],
        area_type: str,
        parameters: Dict[str, Any] = None,
    ) -> Optional[Dict[str, Any]]:
        """
        Create a new cortical area.

        ARCHITECTURE COMPLIANCE: WRITE operation routes through GenomeService
        to maintain proper data flow and ensure genome consistency.
        """
        if not self._genome_service:
            self.logger.error("GenomeService not available for WRITE operations")
            return None

        try:
            # ARCHITECTURE COMPLIANCE: Route WRITE operation through GenomeService
            return self._genome_service.create_cortical_area(
                name=name,
                coordinates=coordinates,
                dimensions=dimensions,
                area_type=area_type,
                parameters=parameters,
            )
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
        parameters: Optional[Dict[str, Any]] = None,
    ) -> Optional[Dict[str, Any]]:
        """
        Update an existing cortical area.

        ARCHITECTURE COMPLIANCE: WRITE operation routes through GenomeService
        to maintain proper data flow and ensure genome consistency.

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
        if not self._genome_service:
            self.logger.error("GenomeService not available for WRITE operations")
            return None

        try:
            # ARCHITECTURE COMPLIANCE: Route WRITE operation through GenomeService
            return self._genome_service.update_cortical_area(
                cortical_id=cortical_id,
                name=name,
                coordinates=coordinates,
                dimensions=dimensions,
                area_type=area_type,
                parameters=parameters,
            )
        except Exception as e:
            self.logger.error(
                f"Error updating cortical area with cortical_id '{cortical_id}': {str(e)}"
            )
            return None

    def delete_area(self, cortical_id: str) -> bool:
        """
        Delete a cortical area.

        ARCHITECTURE COMPLIANCE: WRITE operation routes through GenomeService
        to maintain proper data flow and ensure genome consistency.

        Args:
            cortical_id: 6-character string identifier

        Returns:
            True if successfully deleted, False otherwise
        """
        if not self._genome_service:
            self.logger.error("GenomeService not available for WRITE operations")
            return False

        try:
            # ARCHITECTURE COMPLIANCE: Route WRITE operation through GenomeService
            return self._genome_service.delete_cortical_area(cortical_id)
        except Exception as e:
            self.logger.error(
                f"Error deleting cortical area with cortical_id '{cortical_id}': {str(e)}"
            )
            return False

    def get_area_neurons(self, cortical_id: str) -> Optional[List[Dict[str, Any]]]:
        """
        Get neurons for a specific cortical area.

        Args:
            cortical_id: 6-character string identifier

        Returns:
            List of neuron information or None if area not found
        """

        # Map cortical_id to cortical_idx
        cortical_idx = self._get_cortical_idx_for_id(cortical_id)
        if cortical_idx is None:
            self.logger.warning(
                f"Cortical area with cortical_id '{cortical_id}' not found for neuron retrieval"
            )
            return None

        try:
            if cortical_idx not in self._connectome_manager.cortical_areas:
                return None

            # Get all neurons in this area
            neuron_ids = self._connectome_manager.get_neurons_by_area(cortical_id)
            result = []

            for neuron_id in neuron_ids:
                # Get neuron index for accessing property arrays
                neuron_index = self._connectome_manager._neuron_id_to_index.get(
                    neuron_id
                )
                if neuron_index is None:
                    continue

                # Get neuron position
                position = self._connectome_manager.get_neuron_position(neuron_id)

                # Get neuron properties
                membrane_potential = float(
                    self._connectome_manager.membrane_potentials[neuron_index]
                )
                threshold = float(self._connectome_manager.thresholds[neuron_index])
                decay_rate = float(self._connectome_manager.decay_rates[neuron_index])

                result.append(
                    {
                        "id": str(neuron_id),
                        "position": {
                            "x": position[0],
                            "y": position[1],
                            "z": position[2],
                        },
                        "properties": {
                            "membrane_potential": membrane_potential,
                            "threshold": threshold,
                            "decay_rate": decay_rate,
                        },
                    }
                )

            return result
        except Exception as e:
            self.logger.error(
                f"Error retrieving neurons for cortical area with cortical_id '{cortical_id}': {str(e)}"
            )
            return None

    def get_area_activity(
        self, cortical_id: str, window: int = 1
    ) -> Optional[Dict[str, Any]]:
        """
        Get activity data for a specific cortical area.

        Args:
            cortical_id: 6-character string identifier
            window: Time window for activity analysis

        Returns:
            Activity information or None if area not found
        """

        # Map cortical_id to cortical_idx
        cortical_idx = self._get_cortical_idx_for_id(cortical_id)
        if cortical_idx is None:
            self.logger.warning(
                f"Cortical area with cortical_id '{cortical_id}' not found for activity retrieval"
            )
            return None

        try:
            if cortical_idx not in self._connectome_manager.cortical_areas:
                return None

            # Get all neurons in this area
            neuron_ids = self._connectome_manager.get_neurons_by_area(cortical_id)

            # Current timestep
            current_time = self._connectome_manager.current_timestep

            # Get neurons that fired within the window
            active_neurons = []
            for neuron_id in neuron_ids:
                neuron_index = self._connectome_manager._neuron_id_to_index.get(
                    neuron_id
                )
                if neuron_index is None:
                    continue

                last_fired = int(self._connectome_manager.last_fired[neuron_index])
                if last_fired > 0 and (current_time - last_fired) <= window:
                    position = self._connectome_manager.get_neuron_position(neuron_id)
                    active_neurons.append(
                        {
                            "id": str(neuron_id),
                            "position": {
                                "x": position[0],
                                "y": position[1],
                                "z": position[2],
                            },
                            "last_fired": last_fired,
                        }
                    )

            # Calculate activity summary
            total_neurons = len(neuron_ids)
            active_count = len(active_neurons)

            return {
                "total_neurons": total_neurons,
                "active_neurons": active_count,
                "activity_ratio": (
                    active_count / total_neurons if total_neurons > 0 else 0
                ),
                "active_details": active_neurons[
                    :100
                ],  # Limit to prevent huge responses
            }
        except Exception as e:
            self.logger.error(
                f"Error retrieving activity for cortical area with cortical_id '{cortical_id}': {str(e)}"
            )
            return None

    def get_area_connectivity(
        self, cortical_id: str, direction: str = "both"
    ) -> Optional[Dict[str, Any]]:
        """
        Get connectivity information for a cortical area.

        Args:
            cortical_id: 6-character string identifier
            direction: "incoming", "outgoing", or "both"

        Returns:
            Connectivity information or None if area not found
        """
        try:
            # Map cortical_id to cortical_idx
            cortical_idx = self._get_cortical_idx_for_id(cortical_id)
            if cortical_idx is None:
                raise ValueError(
                    f"Cortical area with cortical_id '{cortical_id}' not found"
                )

            # Validate direction parameter
            if direction not in ["incoming", "outgoing", "both"]:
                raise ValueError(
                    f"Invalid direction: {direction}. Must be 'incoming', 'outgoing', or 'both'"
                )

            # Check if the cortical area exists
            if (
                not hasattr(self._connectome_manager, "cortical_areas")
                or cortical_idx not in self._connectome_manager.cortical_areas
            ):
                raise ValueError(
                    f"Cortical area with cortical_idx {cortical_idx} not found in connectome"
                )

            # Get neurons in the area
            neuron_ids = self._connectome_manager.get_neurons_by_area(cortical_id)
            if not neuron_ids:
                raise ValueError(
                    f"No neurons found in cortical area with cortical_id '{cortical_id}'"
                )

            incoming_connections = set()
            outgoing_connections = set()

            # Get incoming connections
            if direction in ["incoming", "both"]:
                for neuron_id in neuron_ids:
                    connections = self._connectome_manager.get_incoming_connections(
                        neuron_id
                    )
                    for pre_id, _ in connections:
                        # Skip connections within the same area
                        pre_area_idx = self._connectome_manager._neuron_to_area.get(
                            pre_id
                        )
                        if pre_area_idx is not None and pre_area_idx != cortical_idx:
                            incoming_connections.add(pre_area_idx)

            # Get outgoing connections
            if direction in ["outgoing", "both"]:
                for neuron_id in neuron_ids:
                    connections = self._connectome_manager.get_outgoing_connections(
                        neuron_id
                    )
                    for post_id, _ in connections:
                        # Skip connections within the same area
                        post_area_idx = self._connectome_manager._neuron_to_area.get(
                            post_id
                        )
                        if post_area_idx is not None and post_area_idx != cortical_idx:
                            outgoing_connections.add(post_area_idx)

            # Format results - only include areas that actually exist
            result = {
                "cortical_id": cortical_id,  # Return original cortical_id
                "direction": direction,
            }

            if direction in ["incoming", "both"]:
                incoming_list = []
                for connected_area_idx in incoming_connections:
                    area_obj = self._connectome_manager.cortical_areas.get(
                        connected_area_idx
                    )
                    connected_cortical_id = self._get_cortical_id_for_idx(
                        connected_area_idx
                    )
                    if (
                        area_obj
                        and hasattr(area_obj, "name")
                        and area_obj.name
                        and connected_cortical_id
                    ):
                        incoming_list.append(
                            {
                                "cortical_id": connected_cortical_id,  # Use proper cortical_id
                                "name": area_obj.name,
                            }
                        )
                    else:
                        # If no valid name or cortical_id, don't include this connection
                        self.logger.warning(
                            f"Skipping connection to area {connected_area_idx} - no valid name or cortical_id"
                        )
                result["incoming_connections"] = incoming_list

            if direction in ["outgoing", "both"]:
                outgoing_list = []
                for connected_area_idx in outgoing_connections:
                    area_obj = self._connectome_manager.cortical_areas.get(
                        connected_area_idx
                    )
                    connected_cortical_id = self._get_cortical_id_for_idx(
                        connected_area_idx
                    )
                    if (
                        area_obj
                        and hasattr(area_obj, "name")
                        and area_obj.name
                        and connected_cortical_id
                    ):
                        outgoing_list.append(
                            {
                                "cortical_id": connected_cortical_id,  # Use proper cortical_id
                                "name": area_obj.name,
                            }
                        )
                    else:
                        # If no valid name or cortical_id, don't include this connection
                        self.logger.warning(
                            f"Skipping connection to area {connected_area_idx} - no valid name or cortical_id"
                        )
                result["outgoing_connections"] = outgoing_list

            return result
        except Exception as e:
            self.logger.error(
                f"Error retrieving connectivity for cortical area with cortical_id '{cortical_id}': {str(e)}"
            )
            raise ValueError(f"Failed to retrieve connectivity: {str(e)}")

    # REMOVED: stimulate_area method - consolidated into unified stimulate_neurons method
    # in core API service. All stimulation now goes through the coordinate-based 
    # unified approach for consistency and performance.

    def get_id_list(self) -> List[str]:
        """
        Get a list of all cortical area IDs (6-character strings).

        Returns:
            List of cortical area ID strings
        """
        try:
            # ARCHITECTURE COMPLIANCE: Use ConnectomeManager as single source of truth
            cortical_ids = self._connectome_manager.get_all_cortical_ids()
            self.logger.debug(
                f"Retrieved {len(cortical_ids)} cortical area IDs from ConnectomeManager"
            )
            return cortical_ids

        except Exception as e:
            self.logger.error(f"Error getting cortical area ID list: {str(e)}")
            import traceback

            self.logger.error(traceback.format_exc())
            return []

    def get_index_list(self) -> List[int]:
        """Get a list of all cortical area indices (integers) used by the FCL."""
        try:
            # ARCHITECTURE COMPLIANCE: Use ConnectomeManager as single source of truth
            indices = self._connectome_manager.get_all_cortical_indices()
            self.logger.debug(
                f"Retrieved {len(indices)} cortical indices from ConnectomeManager"
            )
            return indices

        except Exception as e:
            self.logger.error(f"Error getting cortical area index list: {str(e)}")
            raise ValueError(f"Failed to retrieve cortical area indices: {str(e)}")

    def get_name_list(self) -> List[str]:
        """Get a list of all cortical area names."""
        try:
            # ARCHITECTURE COMPLIANCE: Use ConnectomeManager as single source of truth
            names = self._connectome_manager.get_cortical_area_names()
            return sorted(names)
        except Exception as e:
            self.logger.error(f"Error getting cortical area name list: {str(e)}")
            raise ValueError(f"Failed to retrieve cortical area names: {str(e)}")

    def get_id_name_mapping(self) -> Dict[str, str]:
        """Get mapping of cortical area IDs to names."""
        mapping = {}
        try:
            if not hasattr(self._connectome_manager, "cortical_areas"):
                return mapping

            for cortical_idx, area in self._connectome_manager.cortical_areas.items():
                # Use cortical_id if available, otherwise fall back to string of index
                area_id = getattr(area, "cortical_id", str(cortical_idx))
                mapping[area_id] = area.name

        except Exception as e:
            self.logger.error(f"Error getting ID-name mapping: {str(e)}")

        return mapping

    def get_current_ipu_list(self) -> List[str]:
        """Get list of current IPU (Input Processing Unit) cortical areas."""
        ipu_areas = []
        try:
            if not hasattr(self._connectome_manager, "cortical_areas"):
                return ipu_areas

            for cortical_idx, area in self._connectome_manager.cortical_areas.items():
                # Check if this is an IPU area
                if hasattr(area, "group_id") and area.group_id == "IPU":
                    area_id = getattr(area, "cortical_id", str(cortical_idx))
                    ipu_areas.append(area_id)

        except Exception as e:
            self.logger.error(f"Error getting IPU list: {str(e)}")

        return ipu_areas

    def get_current_opu_list(self) -> List[str]:
        """Get list of current OPU (Output Processing Unit) cortical areas."""
        opu_areas = []
        try:
            if not hasattr(self._connectome_manager, "cortical_areas"):
                return opu_areas

            for cortical_idx, area in self._connectome_manager.cortical_areas.items():
                # Check if this is an OPU area
                if hasattr(area, "group_id") and area.group_id == "OPU":
                    area_id = getattr(area, "cortical_id", str(cortical_idx))
                    opu_areas.append(area_id)

        except Exception as e:
            self.logger.error(f"Error getting OPU list: {str(e)}")

        return opu_areas

    def get_cortical_locations_2d(self) -> Dict[str, List[int]]:
        """Get 2D locations of all cortical areas (fixed method name)."""
        return self.get_2d_locations()

    def get_2d_locations(self) -> Dict[str, List[int]]:
        """Get 2D locations of all cortical areas."""
        try:
            # ARCHITECTURE COMPLIANCE: Use ConnectomeManager as single source of truth
            # Extract 2D coordinates from cortical area properties
            locations = {}
            cortical_ids = self._connectome_manager.get_all_cortical_ids()

            for cortical_id in cortical_ids:
                try:
                    area_props = self._connectome_manager.get_cortical_area_properties(
                        cortical_id
                    )
                    # Check if area has 2D coordinates in its parameters
                    if "parameters" in area_props and area_props["parameters"]:
                        params = area_props["parameters"]
                        x_coord = params.get("2dcorx")
                        y_coord = params.get("2dcory")

                        if x_coord is not None and y_coord is not None:
                            try:
                                x_coord = int(x_coord)
                                y_coord = int(y_coord)
                                locations[cortical_id] = [x_coord, y_coord]
                            except (ValueError, TypeError):
                                self.logger.warning(
                                    f"Invalid 2D coordinates for area {cortical_id}: x={x_coord}, y={y_coord}"
                                )
                except Exception as e:
                    self.logger.warning(
                        f"Could not get 2D location for area {cortical_id}: {e}"
                    )

            return locations
        except Exception as e:
            self.logger.error(f"Error getting cortical 2D locations: {str(e)}")
            raise ValueError(f"Failed to retrieve cortical area 2D locations: {str(e)}")

    def refresh_cache(self):
        """Refresh cached data when state changes occur."""
        # Clear caches so they'll be rebuilt on next access
        self._cortical_areas_cache = None
        self._cortical_areas_cache_timestamp = 0
