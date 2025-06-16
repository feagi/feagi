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

"""Connectome service for managing FEAGI connectome operations."""

from typing import Any, Dict, Optional

from ..shared.base_service import BaseService


class ConnectomeService(BaseService):
    """
    Connectome service handles connectome READ operations following the architectural principle:

    READ Operations: API → Service → ConnectomeManager (direct access)

    Note: This service primarily handles READ operations for connectome analysis.
    Granular synaptic connection modifications are not exposed at the API level
    as they are too low-level for typical use cases.
    """

    # ===== READ OPERATIONS (Direct ConnectomeManager Access) =====

    def get_neuron_connectivity(
        self, neuron_id: str, direction: str = "both"
    ) -> Optional[Dict[str, Any]]:
        """Get connectivity information for a specific neuron."""
        if not self._validate_genome_loaded():
            return None

        try:
            neuron_id_int = int(neuron_id)

            # ARCHITECTURE COMPLIANCE: READ operation uses ConnectomeManager directly
            if neuron_id_int not in self._connectome_manager._neuron_id_to_index:
                return None

            result = {"neuron_id": neuron_id, "direction": direction}

            if direction in ["incoming", "both"]:
                incoming = self._connectome_manager.get_incoming_connections(
                    neuron_id_int
                )
                result["incoming_connections"] = [
                    {"from_neuron": str(pre_id), "weight": float(weight)}
                    for pre_id, weight in incoming
                ]

            if direction in ["outgoing", "both"]:
                outgoing = self._connectome_manager.get_outgoing_connections(
                    neuron_id_int
                )
                result["outgoing_connections"] = [
                    {"to_neuron": str(post_id), "weight": float(weight)}
                    for post_id, weight in outgoing
                ]

            return result
        except Exception as e:
            self.logger.error(f"Error getting neuron connectivity: {str(e)}")
            return None

    def get_connection_stats(self) -> Dict[str, Any]:
        """Get overall connectivity statistics."""
        if not self._validate_genome_loaded():
            return {}

        try:
            # ARCHITECTURE COMPLIANCE: READ operation uses ConnectomeManager directly
            total_neurons = len(self._connectome_manager._neuron_id_to_index)
            total_synapses = sum(
                len(connections)
                for connections in self._connectome_manager._outgoing_connections.values()
            )

            # Calculate average connectivity
            avg_outgoing = total_synapses / total_neurons if total_neurons > 0 else 0

            # Count cortical areas
            total_areas = len(self._connectome_manager.cortical_areas)

            # Calculate network density (simplified)
            max_possible_connections = total_neurons * (total_neurons - 1)
            density = (
                total_synapses / max_possible_connections
                if max_possible_connections > 0
                else 0
            )

            return {
                "total_neurons": total_neurons,
                "total_synapses": total_synapses,
                "total_cortical_areas": total_areas,
                "average_outgoing_connections": avg_outgoing,
                "network_density": density,
            }
        except Exception as e:
            self.logger.error(f"Error getting connection stats: {str(e)}")
            return {}

    def get_connection_matrix(
        self, source_area: str, target_area: str
    ) -> Optional[Dict[str, Any]]:
        """Get connection matrix between two cortical areas.

        Args:
            source_area: ID of the source cortical area
            target_area: ID of the target cortical area

        Returns:
            Dictionary containing connection information between the areas:
            - source_area: Source area ID
            - target_area: Target area ID
            - connection_count: Number of connections
            - total_weight: Sum of all connection weights
            - average_weight: Average connection weight
            - connections: List of individual connections (limited to 100)
        """
        if not self._validate_genome_loaded():
            return None

        try:
            # ARCHITECTURE COMPLIANCE: READ operation uses ConnectomeManager directly
            source_neurons = self._connectome_manager.get_neurons_by_area(source_area)
            target_neurons = self._connectome_manager.get_neurons_by_area(target_area)

            if not source_neurons or not target_neurons:
                return None

            # Build connection matrix
            connections = []
            total_weight = 0.0

            for src_neuron in source_neurons:
                outgoing = self._connectome_manager.get_outgoing_connections(src_neuron)
                for tgt_neuron, weight in outgoing:
                    if tgt_neuron in target_neurons:
                        connections.append(
                            {
                                "source_neuron": str(src_neuron),
                                "target_neuron": str(tgt_neuron),
                                "weight": float(weight),
                            }
                        )
                        total_weight += weight

            return {
                "source_area": source_area,
                "target_area": target_area,
                "connection_count": len(connections),
                "total_weight": total_weight,
                "average_weight": total_weight / len(connections) if connections else 0,
                "connections": connections[:100],  # Limit for response size
            }
        except Exception as e:
            self.logger.error(f"Error getting connection matrix: {str(e)}")
            return None

    def get_area_to_area_connectivity(self) -> Dict[str, Any]:
        """Get connectivity matrix between all cortical areas."""
        if not self._validate_genome_loaded():
            return {}

        try:
            # ARCHITECTURE COMPLIANCE: READ operation uses ConnectomeManager directly
            areas = list(self._connectome_manager.cortical_areas.keys())
            connectivity_matrix = {}

            for src_area in areas:
                connectivity_matrix[str(src_area)] = {}
                src_neurons = self._connectome_manager.get_neurons_by_area(src_area)

                for tgt_area in areas:
                    tgt_neurons = set(
                        self._connectome_manager.get_neurons_by_area(tgt_area)
                    )

                    connection_count = 0
                    total_weight = 0.0

                    for src_neuron in src_neurons:
                        outgoing = self._connectome_manager.get_outgoing_connections(
                            src_neuron
                        )
                        for tgt_neuron, weight in outgoing:
                            if tgt_neuron in tgt_neurons:
                                connection_count += 1
                                total_weight += weight

                    connectivity_matrix[str(src_area)][str(tgt_area)] = {
                        "connection_count": connection_count,
                        "total_weight": total_weight,
                        "average_weight": (
                            total_weight / connection_count
                            if connection_count > 0
                            else 0
                        ),
                    }

            return connectivity_matrix
        except Exception as e:
            self.logger.error(f"Error getting area-to-area connectivity: {str(e)}")
            return {}

    def analyze_network_properties(self) -> Dict[str, Any]:
        """Analyze network properties like clustering, path lengths, etc."""
        if not self._validate_genome_loaded():
            return {}

        try:
            # ARCHITECTURE COMPLIANCE: READ operation uses ConnectomeManager directly
            total_neurons = len(self._connectome_manager._neuron_id_to_index)
            total_connections = sum(
                len(connections)
                for connections in self._connectome_manager._outgoing_connections.values()
            )

            # Calculate degree distribution
            in_degrees = {}
            out_degrees = {}

            for neuron_id in self._connectome_manager._neuron_id_to_index.keys():
                out_degree = len(
                    self._connectome_manager.get_outgoing_connections(neuron_id)
                )
                in_degree = len(
                    self._connectome_manager.get_incoming_connections(neuron_id)
                )

                out_degrees[neuron_id] = out_degree
                in_degrees[neuron_id] = in_degree

            # Calculate statistics
            avg_out_degree = (
                sum(out_degrees.values()) / len(out_degrees) if out_degrees else 0
            )
            avg_in_degree = (
                sum(in_degrees.values()) / len(in_degrees) if in_degrees else 0
            )

            max_out_degree = max(out_degrees.values()) if out_degrees else 0
            max_in_degree = max(in_degrees.values()) if in_degrees else 0

            return {
                "total_neurons": total_neurons,
                "total_connections": total_connections,
                "average_out_degree": avg_out_degree,
                "average_in_degree": avg_in_degree,
                "max_out_degree": max_out_degree,
                "max_in_degree": max_in_degree,
                "network_density": (
                    total_connections / (total_neurons * (total_neurons - 1))
                    if total_neurons > 1
                    else 0
                ),
            }
        except Exception as e:
            self.logger.error(f"Error analyzing network properties: {str(e)}")
            return {}
