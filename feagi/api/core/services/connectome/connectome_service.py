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

from typing import Dict, Any, Optional, List, Tuple
from ..shared.base_service import BaseService


class ConnectomeService(BaseService):
    """
    Connectome service handles connectome operations including
    synapse management, connection analysis, and network structure.
    """
    
    def get_neuron_connectivity(self, neuron_id: str, direction: str = "both") -> Optional[Dict[str, Any]]:
        """Get connectivity information for a specific neuron."""
        if not self._validate_genome_loaded():
            return None
        
        try:
            neuron_id_int = int(neuron_id)
            
            # Check if neuron exists
            if neuron_id_int not in self._connectome_manager._neuron_id_to_index:
                return None
            
            result = {
                "neuron_id": neuron_id,
                "direction": direction
            }
            
            if direction in ["incoming", "both"]:
                incoming = self._connectome_manager.get_incoming_connections(neuron_id_int)
                result["incoming_connections"] = [
                    {
                        "from_neuron": str(pre_id),
                        "weight": float(weight)
                    }
                    for pre_id, weight in incoming
                ]
            
            if direction in ["outgoing", "both"]:
                outgoing = self._connectome_manager.get_outgoing_connections(neuron_id_int)
                result["outgoing_connections"] = [
                    {
                        "to_neuron": str(post_id),
                        "weight": float(weight)
                    }
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
            # Get basic statistics
            total_neurons = len(self._connectome_manager._neuron_id_to_index)
            total_synapses = sum(len(connections) for connections in self._connectome_manager._outgoing_connections.values())
            
            # Calculate average connectivity
            avg_outgoing = total_synapses / total_neurons if total_neurons > 0 else 0
            
            # Count cortical areas
            total_areas = len(self._connectome_manager.cortical_areas)
            
            # Calculate network density (simplified)
            max_possible_connections = total_neurons * (total_neurons - 1)
            density = total_synapses / max_possible_connections if max_possible_connections > 0 else 0
            
            return {
                "total_neurons": total_neurons,
                "total_synapses": total_synapses,
                "total_cortical_areas": total_areas,
                "average_outgoing_connections": avg_outgoing,
                "network_density": density
            }
        except Exception as e:
            self.logger.error(f"Error getting connection stats: {str(e)}")
            return {}

    def get_connection_matrix(self, source_area: str, target_area: str) -> Optional[Dict[str, Any]]:
        """Get connection matrix between two cortical areas."""
        if not self._validate_genome_loaded():
            return None
        
        try:
            source_idx = int(source_area)
            target_idx = int(target_area)
            
            # Get neurons in both areas
            source_neurons = self._connectome_manager.get_neurons_by_area(source_idx)
            target_neurons = self._connectome_manager.get_neurons_by_area(target_idx)
            
            if not source_neurons or not target_neurons:
                return None
            
            # Build connection matrix
            connections = []
            total_weight = 0.0
            
            for src_neuron in source_neurons:
                outgoing = self._connectome_manager.get_outgoing_connections(src_neuron)
                for tgt_neuron, weight in outgoing:
                    if tgt_neuron in target_neurons:
                        connections.append({
                            "source_neuron": str(src_neuron),
                            "target_neuron": str(tgt_neuron),
                            "weight": float(weight)
                        })
                        total_weight += weight
            
            return {
                "source_area": source_area,
                "target_area": target_area,
                "connection_count": len(connections),
                "total_weight": total_weight,
                "average_weight": total_weight / len(connections) if connections else 0,
                "connections": connections[:100]  # Limit for response size
            }
        except Exception as e:
            self.logger.error(f"Error getting connection matrix: {str(e)}")
            return None

    def add_connection(self, source_neuron: str, target_neuron: str, weight: float = 1.0) -> bool:
        """Add a new synaptic connection."""
        if not self._validate_genome_loaded():
            return False
        
        try:
            src_id = int(source_neuron)
            tgt_id = int(target_neuron)
            
            # Check if neurons exist
            if (src_id not in self._connectome_manager._neuron_id_to_index or 
                tgt_id not in self._connectome_manager._neuron_id_to_index):
                return False
            
            # Add the connection
            self._connectome_manager.add_connection(src_id, tgt_id, weight)
            return True
        except Exception as e:
            self.logger.error(f"Error adding connection: {str(e)}")
            return False

    def remove_connection(self, source_neuron: str, target_neuron: str) -> bool:
        """Remove a synaptic connection."""
        if not self._validate_genome_loaded():
            return False
        
        try:
            src_id = int(source_neuron)
            tgt_id = int(target_neuron)
            
            # Remove the connection
            self._connectome_manager.remove_connection(src_id, tgt_id)
            return True
        except Exception as e:
            self.logger.error(f"Error removing connection: {str(e)}")
            return False

    def update_connection_weight(self, source_neuron: str, target_neuron: str, new_weight: float) -> bool:
        """Update the weight of an existing connection."""
        if not self._validate_genome_loaded():
            return False
        
        try:
            src_id = int(source_neuron)
            tgt_id = int(target_neuron)
            
            # Update the weight
            self._connectome_manager.update_connection_weight(src_id, tgt_id, new_weight)
            return True
        except Exception as e:
            self.logger.error(f"Error updating connection weight: {str(e)}")
            return False

    def get_area_to_area_connectivity(self) -> Dict[str, Any]:
        """Get connectivity matrix between all cortical areas."""
        if not self._validate_genome_loaded():
            return {}
        
        try:
            areas = list(self._connectome_manager.cortical_areas.keys())
            connectivity_matrix = {}
            
            for src_area in areas:
                connectivity_matrix[str(src_area)] = {}
                src_neurons = self._connectome_manager.get_neurons_by_area(src_area)
                
                for tgt_area in areas:
                    tgt_neurons = set(self._connectome_manager.get_neurons_by_area(tgt_area))
                    
                    connection_count = 0
                    total_weight = 0.0
                    
                    for src_neuron in src_neurons:
                        outgoing = self._connectome_manager.get_outgoing_connections(src_neuron)
                        for tgt_neuron, weight in outgoing:
                            if tgt_neuron in tgt_neurons:
                                connection_count += 1
                                total_weight += weight
                    
                    connectivity_matrix[str(src_area)][str(tgt_area)] = {
                        "connection_count": connection_count,
                        "total_weight": total_weight,
                        "average_weight": total_weight / connection_count if connection_count > 0 else 0
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
            # Basic network analysis
            total_neurons = len(self._connectome_manager._neuron_id_to_index)
            total_connections = sum(len(connections) for connections in self._connectome_manager._outgoing_connections.values())
            
            # Calculate degree distribution
            in_degrees = {}
            out_degrees = {}
            
            for neuron_id in self._connectome_manager._neuron_id_to_index.keys():
                out_degree = len(self._connectome_manager.get_outgoing_connections(neuron_id))
                in_degree = len(self._connectome_manager.get_incoming_connections(neuron_id))
                
                out_degrees[neuron_id] = out_degree
                in_degrees[neuron_id] = in_degree
            
            # Calculate statistics
            avg_out_degree = sum(out_degrees.values()) / len(out_degrees) if out_degrees else 0
            avg_in_degree = sum(in_degrees.values()) / len(in_degrees) if in_degrees else 0
            
            max_out_degree = max(out_degrees.values()) if out_degrees else 0
            max_in_degree = max(in_degrees.values()) if in_degrees else 0
            
            return {
                "total_neurons": total_neurons,
                "total_connections": total_connections,
                "average_out_degree": avg_out_degree,
                "average_in_degree": avg_in_degree,
                "max_out_degree": max_out_degree,
                "max_in_degree": max_in_degree,
                "network_density": total_connections / (total_neurons * (total_neurons - 1)) if total_neurons > 1 else 0
            }
        except Exception as e:
            self.logger.error(f"Error analyzing network properties: {str(e)}")
            return {} 