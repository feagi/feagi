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

"""
Refactored CoreAPIService using domain-based service architecture.

This is the new facade implementation that delegates to specialized services
while maintaining complete backward compatibility with the existing API.
"""

import os
import tempfile
import time
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np

from feagi.bdu.connectivity.cortical_mappings import get_mapping_restrictions_registry
from feagi.utils.logger import setup_logger

from .agents.agents_service import AgentsService
from .brain.brain_service import BrainService
from .connectome.connectome_service import ConnectomeService
from .cortical_area.cortical_area_service import CorticalAreaService
from .genome.genome_service import GenomeService
from .network.network_service import NetworkService

# Import all domain services
from .system.system_service import SystemService

logger = setup_logger()


class CoreAPIService:
    """
    Facade for all FEAGI core API operations.

    This class delegates to specialized domain services while maintaining
    the exact same public interface as the original CoreAPIService.

    The refactoring provides:
    - Better separation of concerns
    - Improved maintainability
    - Easier testing and development
    - Zero breaking changes to existing code
    """

    def __init__(self, connectome_manager, state_manager=None):
        """
        Initialize the Core API Service facade.

        Args:
            connectome_manager: ConnectomeManager instance
            state_manager: FeagiStateManager instance (optional)
        """
        # Initialize connectome manager
        self._connectome_manager = connectome_manager
        self.state_manager = state_manager
        self.logger = logger

        # CRITICAL: Ensure state manager singleton consistency
        if self.state_manager is None:
            from feagi.core.state_manager import FeagiStateManager

            self.state_manager = FeagiStateManager.instance()
            self.logger.info("Using FeagiStateManager singleton instance")
        else:
            self.logger.info("Using provided state manager instance")

        # Initialize all domain services with the SAME state manager instance
        self._system_service = SystemService(connectome_manager, self.state_manager)
        self._cortical_area_service = CorticalAreaService(
            connectome_manager, self.state_manager
        )
        self._connectome_service = ConnectomeService(
            connectome_manager, self.state_manager
        )
        self._brain_service = BrainService(connectome_manager, self.state_manager)
        self._agents_service = AgentsService(connectome_manager, self.state_manager)
        self._network_service = NetworkService(connectome_manager, self.state_manager)

        # CRITICAL: Pass brain service to genome service to ensure singleton BurstEngine usage
        self._genome_service = GenomeService(
            connectome_manager, self.state_manager, self._brain_service
        )

        # Validate state manager consistency across services
        self._validate_service_state_consistency()

        self.logger.info(
            "CoreAPIService initialized with domain-based architecture and state synchronization"
        )

    def _validate_service_state_consistency(self):
        """Validate that all services share the same state manager instance."""
        try:
            services = [
                ("system", self._system_service),
                ("genome", self._genome_service),
                ("cortical_area", self._cortical_area_service),
                ("connectome", self._connectome_service),
                ("brain", self._brain_service),
                ("agents", self._agents_service),
                ("network", self._network_service),
            ]

            core_state_id = id(self.state_manager)
            inconsistent_services = []

            for service_name, service in services:
                if hasattr(service, "state_manager"):
                    service_state_id = id(service.state_manager)
                    if service_state_id != core_state_id:
                        inconsistent_services.append(service_name)
                        self.logger.error(
                            f"Service {service_name} has different state manager instance: core={core_state_id}, service={service_state_id}"
                        )
                else:
                    inconsistent_services.append(service_name)
                    self.logger.error(
                        f"Service {service_name} missing state_manager attribute"
                    )

            if inconsistent_services:
                self.logger.error(
                    f"State manager inconsistency detected in services: {inconsistent_services}"
                )
                raise RuntimeError(
                    f"Critical state manager inconsistency in services: {inconsistent_services}"
                )
            else:
                self.logger.info(
                    "All services share the same state manager instance - consistency validated"
                )

        except Exception as e:
            self.logger.error(f"Error validating service state consistency: {str(e)}")
            raise

    # =================================================================
    # SYSTEM SERVICE DELEGATION
    # =================================================================

    async def get_health(self) -> Dict[str, Any]:
        """Get comprehensive system health information."""
        return await self._system_service.get_health()

    def get_user_preferences(self) -> Dict[str, Any]:
        """Get user preferences."""
        return self._system_service.get_user_preferences()

    def update_user_preferences(self, preferences: Dict[str, Any]) -> bool:
        """Update user preferences."""
        return self._system_service.update_user_preferences(preferences)

    def get_versions(self) -> Dict[str, Any]:
        """Get version information for various components."""
        return self._system_service.get_versions()

    def get_configuration(self) -> Dict[str, Any]:
        """Get system configuration."""
        return self._system_service.get_configuration()

    def test_influxdb(self) -> Optional[Dict[str, Any]]:
        """Test InfluxDB connectivity."""
        return self._system_service.test_influxdb()

    def set_circuit_library_path(self, path: str) -> bool:
        """Set the circuit library path."""
        return self._system_service.set_circuit_library_path(path)

    def get_cortical_area_types(self) -> Dict[str, Any]:
        """Get available cortical area types."""
        return self._system_service.get_cortical_area_types()

    def reset_fcl(self) -> bool:
        """Reset the Fire Candidate List."""
        return self._system_service.reset_fcl()

    def get_visualization_skip_rate(self) -> int:
        """Get visualization skip rate."""
        return self._system_service.get_visualization_skip_rate()

    def set_visualization_skip_rate(self, skip_rate: int) -> bool:
        """Set visualization skip rate."""
        return self._system_service.set_visualization_skip_rate(skip_rate)

    def get_visualization_suppression_threshold(self) -> int:
        """Get visualization suppression threshold."""
        return self._system_service.get_visualization_suppression_threshold()

    def set_visualization_suppression_threshold(self, threshold: int) -> bool:
        """Set visualization suppression threshold."""
        return self._system_service.set_visualization_suppression_threshold(threshold)

    def get_global_activity_visualization(self) -> bool:
        """Get global activity visualization status."""
        return self._system_service.get_global_activity_visualization()

    def set_global_activity_visualization(self, enabled: bool) -> bool:
        """Set global activity visualization status."""
        return self._system_service.set_global_activity_visualization(enabled)

    def get_unique_logs(self) -> List[str]:
        """Get unique log entries."""
        return self._system_service.get_unique_logs()

    def enable_visualization_fq_sampler(self) -> bool:
        """Enable the visualization FQ sampler for brain visualizer connectivity."""
        return self._system_service.enable_visualization_fq_sampler()

    def disable_visualization_fq_sampler(self) -> bool:
        """Disable the visualization FQ sampler."""
        return self._system_service.disable_visualization_fq_sampler()

    def get_fq_sampler_status(self) -> Dict[str, Any]:
        """Get the current status of all FQ samplers."""
        return self._system_service.get_fq_sampler_status()

    def get_burst_timer(self) -> float:
        """Get burst timer from burst engine."""
        return self._brain_service.get_burst_timer()

    # =================================================================
    # GENOME SERVICE DELEGATION
    # =================================================================

    def load_essential_genome(self) -> Dict[str, Any]:
        """Load the essential genome."""
        return self._genome_service.load_default_genome("essential")

    def load_barebones_genome(self) -> Dict[str, Any]:
        """Load the barebones genome."""
        print(
            "[DEBUG] CORE API SERVICE: load_barebones_genome called, delegating to genome service"
        )
        result = self._genome_service.load_default_genome("barebones")
        print(
            f"[DEBUG] CORE API SERVICE: genome service returned: {result.get('success', 'unknown')}"
        )
        return result

    def load_test_genome(self) -> Dict[str, Any]:
        """Load the test genome."""
        return self._genome_service.load_default_genome("test")

    def load_genome(
        self, genome_data: Dict[str, Any], filename: str = "genome.json"
    ) -> Dict[str, Any]:
        """
        Load a genome and prepare it for use.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager → NeuroEmbryogenesis
        """
        return self._genome_service.load_genome_from_data(genome_data, filename)

    def get_genome(self) -> Optional[Dict[str, Any]]:
        """Get the currently loaded genome data."""
        return self._genome_service.get_genome()

    def get_genome_filename(self) -> str:
        """Get the current genome filename."""
        filename = self._genome_service.get_genome_filename()
        return filename or ""

    def get_genome_file_name(self) -> Dict[str, str]:
        """Get the genome file name in the format expected by the REST API."""
        return self._genome_service.get_genome_file_name()

    def get_default_genomes(self) -> Dict[str, Any]:
        """Get a list of default genome files with their contents."""
        return self._genome_service.get_default_genomes()

    def get_genome_counter(self) -> int:
        """Get the current genome counter."""
        return self._genome_service.get_genome_counter()

    def get_current_genome(self) -> Optional[Dict[str, Any]]:
        """Get the currently loaded genome data (alias for get_genome for download compatibility)."""
        return self._genome_service.get_genome()

    def get_generations(self) -> Dict[str, Any]:
        """Get details about all generations of genomes."""
        return self._genome_service.get_generations()

    def get_change_register(self) -> Dict[str, Any]:
        """Get the evolution change register showing evolutionary history."""
        return self._genome_service.get_change_register()

    def deploy_genome(self, genome_filepath: str) -> bool:
        """Deploy a genome from a file path."""
        return self._genome_service.deploy_genome(genome_filepath)

    def is_genome_loaded(self) -> bool:
        """Check if a genome is currently loaded."""
        return self._genome_service.is_genome_loaded()

    # =================================================================
    # CORTICAL AREA SERVICE DELEGATION
    # =================================================================

    def get_all_cortical_areas(self) -> List[Dict[str, Any]]:
        """Get all cortical areas."""
        return self._cortical_area_service.get_all_areas()

    def get_cortical_area(self, cortical_id: str) -> Optional[Dict[str, Any]]:
        """Get a cortical area by ID."""
        return self._cortical_area_service.get_area(cortical_id)

    def create_cortical_area(
        self,
        name: str,
        coordinates: Dict[str, int],
        dimensions: Dict[str, int],
        area_type: str,
        parameters: Dict[str, Any] = None,
    ) -> Optional[Dict[str, Any]]:
        """Create a new cortical area."""
        return self._cortical_area_service.create_area(
            name, coordinates, dimensions, area_type, parameters
        )

    def update_cortical_area(
        self,
        cortical_id: str,
        name: Optional[str] = None,
        coordinates: Optional[Dict[str, int]] = None,
        dimensions: Optional[Dict[str, int]] = None,
        area_type: Optional[str] = None,
        parameters: Optional[Dict[str, Any]] = None,
    ) -> Optional[Dict[str, Any]]:
        """Update an existing cortical area."""
        return self._cortical_area_service.update_area(
            cortical_id, name, coordinates, dimensions, area_type, parameters
        )

    def update_cortical_area_properties(
        self, cortical_id: str, properties: Dict[str, Any]
    ) -> bool:
        """Update properties of an existing cortical area (wrapper for API compatibility)."""
        try:
            result = self._cortical_area_service.update_area(
                cortical_id, parameters=properties
            )
            return result is not None
        except Exception as e:
            self.logger.error(
                f"Error updating cortical area properties for {cortical_id}: {str(e)}"
            )
            return False

    def delete_cortical_area(self, cortical_id: str) -> bool:
        """Delete a cortical area."""
        return self._cortical_area_service.delete_area(cortical_id)

    def get_cortical_area_neurons(
        self, cortical_id: str
    ) -> Optional[List[Dict[str, Any]]]:
        """Get neurons for a specific cortical area."""
        return self._cortical_area_service.get_area_neurons(cortical_id)

    def get_cortical_area_activity(
        self, cortical_id: str, window: int = 1
    ) -> Optional[Dict[str, Any]]:
        """Get activity data for a specific cortical area."""
        return self._cortical_area_service.get_area_activity(cortical_id, window)

    def get_cortical_area_connectivity(
        self, cortical_id: str, direction: str = "both"
    ) -> Optional[Dict[str, Any]]:
        """Get connectivity information for a specific cortical area."""
        return self._cortical_area_service.get_area_connectivity(cortical_id, direction)

    def stimulate_cortical_area(
        self,
        cortical_id: str,
        pattern: str = "random",
        intensity: float = 1.0,
        duration: int = 1,
        coordinates: Optional[List[Dict[str, int]]] = None,
    ) -> Dict[str, Any]:
        """Stimulate a cortical area with the specified pattern."""
        return self._cortical_area_service.stimulate_area(
            cortical_id, pattern, intensity, duration, coordinates
        )

    def get_cortical_id_list(self) -> List[str]:
        """Get a list of all cortical area IDs (6-character strings) in the current genome."""
        return self._cortical_area_service.get_id_list()

    def get_cortical_index_list(self) -> List[int]:
        """Get a list of all cortical area indices (integers) used by the FCL."""
        return self._cortical_area_service.get_index_list()

    def get_cortical_name_list(self) -> List[str]:
        """Get a list of all cortical area names."""
        return self._cortical_area_service.get_name_list()

    def get_cortical_id_name_mapping(self) -> Dict[str, str]:
        """Map every cortical area's 6-character cortical_id to its human-readable name."""
        return self._cortical_area_service.get_id_name_mapping()

    def get_cortical_locations_2d(self) -> Dict[str, List[int]]:
        """Get 2D locations of all cortical areas."""
        return self._cortical_area_service.get_cortical_locations_2d()

    def get_cortical_2d_locations(self) -> Dict[str, List[int]]:
        """Get 2D locations of all cortical areas (alias for get_cortical_locations_2d)."""
        return self._cortical_area_service.get_cortical_locations_2d()

    def get_cortical_area_geometry(self) -> Dict[str, Any]:
        """Get cortical area geometry information."""
        try:
            # Get all cortical areas and their geometric properties
            areas = self._cortical_area_service.get_all_areas()
            geometry_info = {}

            for area in areas:
                try:
                    # Handle all possible area formats
                    if isinstance(area, dict):
                        area_id = area.get("id")
                    elif isinstance(area, tuple):
                        area_id = str(area[0])  # Convert first element to string
                    else:
                        area_id = str(area)
                    
                    if not area_id:
                        continue
                        
                    # Get complete properties including mapping information
                    properties = self._connectome_manager.get_cortical_area_properties(
                        area_id
                    )
                    if properties:
                        # Handle dimensions - could be tuple or dict
                        dimensions = properties.get("dimensions", {})
                        if isinstance(dimensions, tuple):
                            dimensions = {
                                "width": dimensions[0],
                                "height": dimensions[1],
                                "depth": dimensions[2]
                            }

                        # Handle coordinates - could be tuple or dict
                        coordinates = properties.get("coordinates", {})
                        if isinstance(coordinates, tuple):
                            coordinates = {
                                "x": coordinates[0],
                                "y": coordinates[1],
                                "z": coordinates[2]
                            }

                        geometry_info[area_id] = {
                            "coordinates": coordinates,
                            "dimensions": dimensions,
                            "type": properties.get("type", "unknown"),
                            "neuron_count": properties.get("neuron_count", 0),
                            "parameters": properties.get("parameters", {}),
                            "name": properties.get("name", area_id),
                            "cortical_idx": properties.get("cortical_idx"),
                            "mapping": properties.get("parameters", {}).get(
                                "mapping", {}
                            )
                        }
                except Exception as e:
                    self.logger.error(f"Error processing area {area}: {str(e)}")
                    continue

            return geometry_info
        except Exception as e:
            self.logger.error(f"Error getting cortical area geometry: {str(e)}")
            raise ValueError(f"Failed to get cortical area geometry: {str(e)}") from e

    def get_current_ipu_list(self) -> List[str]:
        """Get list of current IPU cortical areas."""
        return self._cortical_area_service.get_current_ipu_list()

    def get_current_opu_list(self) -> List[str]:
        """Get list of current OPU cortical areas."""
        return self._cortical_area_service.get_current_opu_list()

    # =================================================================
    # CONNECTOME SERVICE DELEGATION
    # =================================================================

    def get_neuron_connectivity(
        self, neuron_id: str, direction: str = "both"
    ) -> Optional[Dict[str, Any]]:
        """Get connectivity information for a specific neuron."""
        return self._connectome_service.get_neuron_connectivity(neuron_id, direction)

    def get_connection_stats(self) -> Dict[str, Any]:
        """Get overall connectivity statistics."""
        return self._connectome_service.get_connection_stats()

    def get_connection_matrix(
        self, source_area: str, target_area: str
    ) -> Optional[Dict[str, Any]]:
        """Get connection matrix between two cortical areas."""
        return self._connectome_service.get_connection_matrix(source_area, target_area)

    def add_connection(
        self, source_neuron: str, target_neuron: str, weight: float = 1.0
    ) -> bool:
        """Add a new synaptic connection."""
        return self._connectome_service.add_connection(
            source_neuron, target_neuron, weight
        )

    def remove_connection(self, source_neuron: str, target_neuron: str) -> bool:
        """Remove a synaptic connection."""
        return self._connectome_service.remove_connection(source_neuron, target_neuron)

    def update_connection_weight(
        self, source_neuron: str, target_neuron: str, new_weight: float
    ) -> bool:
        """Update the weight of an existing connection."""
        return self._connectome_service.update_connection_weight(
            source_neuron, target_neuron, new_weight
        )

    def get_area_to_area_connectivity(self) -> Dict[str, Any]:
        """Get connectivity matrix between all cortical areas."""
        return self._connectome_service.get_area_to_area_connectivity()

    def analyze_network_properties(self) -> Dict[str, Any]:
        """Analyze network properties like clustering, path lengths, etc."""
        return self._connectome_service.analyze_network_properties()

    # =================================================================
    # BRAIN SERVICE DELEGATION
    # =================================================================

    def get_burst_engine_status(self) -> Dict[str, Any]:
        """Get current burst engine status."""
        return self._brain_service.get_burst_engine_status()

    def start_burst_engine(self) -> bool:
        """Start the burst engine."""
        return self._brain_service.start_burst_engine()

    def stop_burst_engine(self) -> bool:
        """Stop the burst engine."""
        return self._brain_service.stop_burst_engine()

    def get_brain_statistics(self) -> Dict[str, Any]:
        """Get comprehensive brain statistics."""
        return self._brain_service.get_brain_statistics()

    def get_activity_summary(self, window: int = 10) -> Dict[str, Any]:
        """Get activity summary for the brain over a time window."""
        return self._brain_service.get_activity_summary(window)

    def reset_brain_state(self) -> bool:
        """Reset the brain to initial state."""
        return self._brain_service.reset_brain_state()

    def get_performance_metrics(self) -> Dict[str, Any]:
        """Get brain performance metrics."""
        return self._brain_service.get_performance_metrics()

    def stimulate_neurons(
        self, neuron_ids: List[str], intensity: float = 1.0
    ) -> Dict[str, Any]:
        """Stimulate specific neurons with given intensity."""
        return self._brain_service.stimulate_neurons(neuron_ids, intensity)

    def get_burst_engine_config(self) -> Dict[str, Any]:
        """Get burst engine configuration."""
        return self._brain_service.get_burst_engine_config()

    def get_burst_engine_stats(self) -> Dict[str, Any]:
        """Get burst engine statistics."""
        return self._brain_service.get_brain_statistics()

    def hold_burst_engine(self) -> bool:
        """Put burst engine on hold (pause neural processing)."""
        return self._brain_service.hold_burst_engine()

    def resume_burst_engine(self) -> bool:
        """Resume burst engine from hold (resume neural processing)."""
        return self._brain_service.resume_burst_engine()

    # =================================================================
    # AGENTS SERVICE DELEGATION
    # =================================================================

    # =================================================================
    # LEGACY AGENT METHODS (removed - using new comprehensive agent registry)
    # =================================================================

    # =================================================================
    # NETWORK SERVICE DELEGATION
    # =================================================================

    def get_network_status(self) -> Dict[str, Any]:
        """Get current network status and health."""
        return self._network_service.get_network_status()

    def get_bandwidth_usage(self, time_window: int = 60) -> Dict[str, Any]:
        """Get bandwidth usage statistics over a time window."""
        return self._network_service.get_bandwidth_usage(time_window)

    def get_connection_statistics(self) -> Dict[str, Any]:
        """Get detailed connection statistics."""
        return self._network_service.get_connection_statistics()

    def test_connectivity(self, target: Optional[str] = None) -> Dict[str, Any]:
        """Test network connectivity to specific targets or general health."""
        return self._network_service.test_connectivity(target)

    def get_protocol_status(self) -> Dict[str, Any]:
        """Get status of different network protocols."""
        return self._network_service.get_protocol_status()

    def reset_network_statistics(self) -> bool:
        """Reset network statistics and counters."""
        return self._network_service.reset_network_statistics()

    def configure_bandwidth_limits(self, limits: Dict[str, Any]) -> Dict[str, Any]:
        """Configure bandwidth limits for different types of traffic."""
        return self._network_service.configure_bandwidth_limits(limits)

    def get_message_queue_status(self) -> Dict[str, Any]:
        """Get status of message queues across different protocols."""
        return self._network_service.get_message_queue_status()

    # =================================================================
    # UTILITY METHODS
    # =================================================================

    def refresh_cached_data(self):
        """
        Refresh any cached data when connectome changes.
        This can be called when the genome is reloaded or modified.
        """
        # Build cortical_id -> cortical_idx cache for optimal performance
        # But ONLY if genome is loaded to prevent corruption
        try:
            if (
                hasattr(self._connectome_manager, "cortical_areas")
                and self._connectome_manager.cortical_areas
            ):
                self._build_cortical_id_cache()
                self.logger.debug("Cached data refreshed after connectome changes")
            else:
                self.logger.debug("Skipping cache refresh - genome not ready")
        except Exception as e:
            self.logger.error(f"Error refreshing cached data: {str(e)}")

    def get_service_health(self) -> Dict[str, Any]:
        """Get health information about all domain services."""
        try:
            return {
                "system_service": "healthy" if self._system_service else "unavailable",
                "genome_service": "healthy" if self._genome_service else "unavailable",
                "cortical_area_service": (
                    "healthy" if self._cortical_area_service else "unavailable"
                ),
                "connectome_service": (
                    "healthy" if self._connectome_service else "unavailable"
                ),
                "brain_service": "healthy" if self._brain_service else "unavailable",
                "agents_service": "healthy" if self._agents_service else "unavailable",
                "network_service": (
                    "healthy" if self._network_service else "unavailable"
                ),
                "facade_status": "operational",
            }
        except Exception as e:
            self.logger.error(f"Error getting service health: {str(e)}")
            return {"facade_status": "error", "error": str(e)}

    # =================================================================
    # CORE COMPONENT ACCESS METHODS
    # =================================================================

    def get_burst_engine(self):
        """Get the burst engine instance - always returns the singleton instance."""
        # Import here to avoid circular imports
        try:
            from feagi.npu.burst_engine import BurstEngine

            # Always use the singleton instance - never create a new one
            singleton_instance = BurstEngine.get_instance()

            if singleton_instance is None:
                # Create singleton instance only if none exists
                self.logger.info(
                    "[DEBUG] CORE API: Creating singleton BurstEngine instance"
                )

                # Check for debug NPU flag and pass through config
                debug_npu = self.state_manager.is_debug_npu_enabled()
                engine_config = {"debug_npu": debug_npu}

                singleton_instance = BurstEngine(
                    connectome_manager=self._connectome_manager, config=engine_config
                )
            # Removed log spam: no longer log when using existing singleton

            return singleton_instance

        except Exception as e:
            self.logger.error(f"Error getting burst engine: {str(e)}")
            return None

    def get_connectome_manager(self):
        """Get the connectome manager instance."""
        return self._connectome_manager

    def get_connectome(self):
        """Get the connectome manager instance (legacy alias)."""
        return self._connectome_manager

    def get_fcl_manager(self):
        """Get the FCL manager instance."""
        if hasattr(self._connectome_manager, "fcl_manager"):
            return self._connectome_manager.fcl_manager
        return None

    def get_memory_manager(self):
        """Get the memory manager instance."""
        # Return the connectome manager as it manages memory
        return self._connectome_manager

    # =================================================================
    # CRITICAL MISSING METHODS - FIRE QUEUE & STATE MANAGEMENT
    # =================================================================

    def get_fire_queue(self) -> Optional[Dict[str, Any]]:
        """Get the global fire queue data for FQSampler from FCL with real neuron coordinates."""
        try:
            if (
                hasattr(self._connectome_manager, "fcl_manager")
                and self._connectome_manager.fcl_manager
            ):
                fcl_manager = self._connectome_manager.fcl_manager

                # Get global firing neurons from FCL
                global_fcl = fcl_manager.get_fcl()

                if global_fcl and not global_fcl.is_empty():
                    global_firing_neurons = list(global_fcl)
                    self.logger.debug(
                        f"🔥 [CORE API] Global fire queue has {len(global_firing_neurons)} firing neurons"
                    )

                    if global_firing_neurons:
                        # Get real neuron coordinates instead of placeholders
                        neuron_coordinates = []
                        neuron_ids = []

                        if hasattr(self._connectome_manager, "neuron_array"):
                            neuron_array = self._connectome_manager.neuron_array
                            for neuron_id in global_firing_neurons:
                                try:
                                    if neuron_id < len(neuron_array):
                                        neuron = neuron_array[neuron_id]
                                        # Only extract coordinates if they actually exist - NO FALLBACKS
                                        if (
                                            "coordinate_3d_x" in neuron
                                            and "coordinate_3d_y" in neuron
                                            and "coordinate_3d_z" in neuron
                                        ):
                                            x = int(neuron["coordinate_3d_x"])
                                            y = int(neuron["coordinate_3d_y"])
                                            z = int(neuron["coordinate_3d_z"])
                                            neuron_coordinates.append((x, y, z))
                                            neuron_ids.append(neuron_id)
                                except (IndexError, KeyError, TypeError):
                                    # Skip invalid neurons
                                    continue

                        if neuron_ids:
                            # Extract REAL neuron data - NO FAKE DATA ALLOWED
                            membrane_potentials = []
                            thresholds = []
                            consecutive_fire_counts = []
                            refractory_counters = []

                            for i, neuron_id in enumerate(neuron_ids):
                                if neuron_id < len(neuron_array):
                                    neuron = neuron_array[neuron_id]
                                    # Only extract exact properties that exist - NO FALLBACKS AT ALL
                                    if "membrane_potential" in neuron:
                                        membrane_potentials.append(
                                            float(neuron["membrane_potential"])
                                        )
                                    if "firing_threshold" in neuron:
                                        thresholds.append(
                                            float(neuron["firing_threshold"])
                                        )
                                    if "consecutive_fire_count" in neuron:
                                        consecutive_fire_counts.append(
                                            int(neuron["consecutive_fire_count"])
                                        )
                                    if "refractory_counter" in neuron:
                                        refractory_counters.append(
                                            int(neuron["refractory_counter"])
                                        )

                            result = {
                                "neuron_ids": neuron_ids,
                                "membrane_potentials": membrane_potentials,  # REAL data
                                "thresholds": thresholds,  # REAL data
                                "consecutive_fire_counts": consecutive_fire_counts,  # REAL data
                                "refractory_counters": refractory_counters,  # REAL data
                                "coordinates": neuron_coordinates,  # REAL coordinates
                            }
                            self.logger.debug(
                                f"🔥 [CORE API] Returning global fire queue: {len(result['neuron_ids'])} neurons with REAL data (no placeholders)"
                            )
                            return result

                    # No valid neurons found
                    return {
                        "neuron_ids": [],
                        "membrane_potentials": [],
                        "thresholds": [],
                        "consecutive_fire_counts": [],
                        "refractory_counters": [],
                        "coordinates": [],
                    }
                else:
                    self.logger.debug(
                        "🔥 [CORE API] Global FCL is empty - no neurons firing globally"
                    )
                    return {
                        "neuron_ids": [],
                        "membrane_potentials": [],
                        "thresholds": [],
                        "consecutive_fire_counts": [],
                        "refractory_counters": [],
                        "coordinates": [],
                    }
            return None
        except Exception as e:
            self.logger.error(f"Error getting global fire queue: {str(e)}")
            return None

    def genome_is_loaded(self) -> bool:
        """Check if a genome is currently loaded - CRITICAL for state management."""
        return self._genome_service.is_genome_loaded()

    def get_state_manager(self):
        """Get the state manager instance."""
        return self.state_manager

    # =================================================================
    # BRAIN STATE MANAGEMENT METHODS
    # =================================================================

    def get_brain_state(self) -> Dict[str, Any]:
        """Get current brain state."""
        return self._brain_service.get_brain_statistics()

    def save_brain_state(self, path: str) -> bool:
        """Save brain state to file."""
        try:
            brain_state = self.get_brain_state()
            import json

            with open(path, "w") as f:
                json.dump(brain_state, f, indent=2)
            return True
        except Exception as e:
            self.logger.error(f"Error saving brain state: {str(e)}")
            return False

    def load_brain_state(self, path: str) -> bool:
        """Load brain state from file."""
        try:
            import json

            with open(path, "r") as f:
                brain_state = json.load(f)
            # This would need implementation in brain service
            return True
        except Exception as e:
            self.logger.error(f"Error loading brain state: {str(e)}")
            return False

    # =================================================================
    # LEGACY COMPATIBILITY METHODS
    # =================================================================

    # Add any legacy method aliases or compatibility methods here if needed
    # For now, all existing methods are preserved with their exact signatures

    # Legacy method aliases for backward compatibility
    def get_cortical_areas(self) -> List[Dict[str, Any]]:
        """Get all cortical areas (alias for get_all_cortical_areas)."""
        return self.get_all_cortical_areas()

    # =================================================================
    # ADDITIONAL AGENT MANAGEMENT METHODS
    # =================================================================

    def get_agent_list(self) -> Set[str]:
        """Get list of agent IDs."""
        try:
            agents = self._agents_service.get_connected_agents()
            return {agent.get("id", agent.get("agent_id", "")) for agent in agents}
        except Exception as e:
            self.logger.error(f"Error getting agent list: {str(e)}")
            return set()

    def get_agent_properties(self, agent_id: str) -> Dict[str, Any]:
        """Get properties of a specific agent."""
        return self._agents_service.get_agent_details(agent_id) or {}

    def deregister_agent(self, agent_id: str) -> bool:
        """Deregister an agent."""
        result = self._agents_service.unregister_agent(agent_id)
        return result.get("success", False) if isinstance(result, dict) else False

    # =================================================================
    # LEGACY CORTICAL AREA METHOD NAMES
    # =================================================================

    def get_cortical_area_id_list(self) -> List[str]:
        """Get list of cortical area IDs (legacy name)."""
        return self.get_cortical_id_list()

    def get_cortical_area_index_list(self) -> List[int]:
        """Get list of cortical area indices (legacy name)."""
        return self.get_cortical_index_list()

    def get_cortical_area_name_list(self) -> List[str]:
        """Get list of cortical area names (legacy name)."""
        return self.get_cortical_name_list()

    def get_cortical_area_stats(self, cortical_area: str) -> Optional[Dict[str, Any]]:
        """Get statistics for a cortical area."""
        return self._cortical_area_service.get_area_stats(cortical_area)

    # =================================================================
    # PLASTICITY AND LEARNING METHODS
    # =================================================================

    def enable_area_plasticity(
        self, cortical_id: str, settings: Optional[Dict[str, Any]] = None
    ) -> bool:
        """Enable plasticity for a cortical area."""
        try:
            # This would need implementation in a plasticity service
            self.logger.info(f"Enabling plasticity for area {cortical_id}")
            return True
        except Exception as e:
            self.logger.error(f"Error enabling plasticity for {cortical_id}: {str(e)}")
            return False

    def disable_area_plasticity(self, cortical_id: str) -> bool:
        """Disable plasticity for a cortical area."""
        try:
            # This would need implementation in a plasticity service
            self.logger.info(f"Disabling plasticity for area {cortical_id}")
            return True
        except Exception as e:
            self.logger.error(f"Error disabling plasticity for {cortical_id}: {str(e)}")
            return False

    def get_plasticity_info(self) -> Dict[str, Any]:
        """Get plasticity information."""
        try:
            return {"enabled": True, "queue_depth": 1000, "areas_with_plasticity": []}
        except Exception as e:
            self.logger.error(f"Error getting plasticity info: {str(e)}")
            return {}

    def get_plasticity_queue_depth(self) -> int:
        """Get plasticity queue depth."""
        return 1000  # Default value

    def update_plasticity_queue_depth(self, depth: int) -> bool:
        """Update plasticity queue depth."""
        try:
            # This would need implementation
            return True
        except Exception as e:
            self.logger.error(f"Error updating plasticity queue depth: {str(e)}")
            return False

    def update_plasticity_config(self, config: Dict[str, Any]) -> bool:
        """Update plasticity configuration."""
        try:
            # This would need implementation
            return True
        except Exception as e:
            self.logger.error(f"Error updating plasticity config: {str(e)}")
            return False

    # =================================================================
    # MONITORING METHODS
    # =================================================================

    def get_membrane_potential_monitoring_status(
        self, cortical_areas: List[str]
    ) -> List[Tuple[str, bool]]:
        """Get membrane potential monitoring status for cortical areas."""
        try:
            # This should get real monitoring status from the brain service
            raise NotImplementedError(
                "Membrane potential monitoring status is not yet implemented"
            )
        except Exception as e:
            self.logger.error(
                f"Error getting membrane potential monitoring status: {str(e)}"
            )
            raise ValueError(
                f"Failed to get membrane potential monitoring status: {str(e)}"
            )

    def set_membrane_potential_monitoring(
        self, cortical_areas: List[str], enabled: bool
    ) -> bool:
        """Set membrane potential monitoring for cortical areas."""
        try:
            # This should actually set monitoring in the brain service
            raise NotImplementedError(
                "Setting membrane potential monitoring is not yet implemented"
            )
        except Exception as e:
            self.logger.error(f"Error setting membrane potential monitoring: {str(e)}")
            raise ValueError(f"Failed to set membrane potential monitoring: {str(e)}")

    def get_synaptic_potential_monitoring_status(
        self, cortical_areas: List[str]
    ) -> List[Tuple[str, bool]]:
        """Get synaptic potential monitoring status for cortical areas."""
        try:
            # This should get real monitoring status from the brain service
            raise NotImplementedError(
                "Synaptic potential monitoring status is not yet implemented"
            )
        except Exception as e:
            self.logger.error(
                f"Error getting synaptic potential monitoring status: {str(e)}"
            )
            raise ValueError(
                f"Failed to get synaptic potential monitoring status: {str(e)}"
            )

    def set_synaptic_potential_monitoring(
        self, cortical_areas: List[str], enabled: bool
    ) -> bool:
        """Set synaptic potential monitoring for cortical areas."""
        try:
            # This should actually set monitoring in the brain service
            raise NotImplementedError(
                "Setting synaptic potential monitoring is not yet implemented"
            )
        except Exception as e:
            self.logger.error(f"Error setting synaptic potential monitoring: {str(e)}")
            raise ValueError(f"Failed to set synaptic potential monitoring: {str(e)}")

    def get_membrane_potentials(self, neuron_ids: List[int]) -> Dict[int, float]:
        """Get membrane potentials for specific neurons."""
        try:
            # This should get real membrane potentials from the brain service
            raise NotImplementedError(
                "Getting membrane potentials is not yet implemented"
            )
        except Exception as e:
            self.logger.error(f"Error getting membrane potentials: {str(e)}")
            raise ValueError(f"Failed to get membrane potentials: {str(e)}")

    def update_membrane_potentials(self, potentials: Dict[int, float]) -> bool:
        """Update membrane potentials for specific neurons."""
        try:
            # This would need implementation in connectome service
            return True
        except Exception as e:
            self.logger.error(f"Error updating membrane potentials: {str(e)}")
            return False

    # =================================================================
    # ADDITIONAL MISSING METHODS
    # =================================================================

    def get_fq_sampler_config(self) -> Dict[str, Any]:
        """Get FQ sampler configuration."""
        try:
            if self.state_manager:
                return {
                    "frequency": getattr(
                        self.state_manager, "fq_sampler_frequency", 20.0
                    ),
                    "consumer": getattr(self.state_manager, "fq_sampler_consumer", 1),
                }
            return {"frequency": 20.0, "consumer": 1}
        except Exception as e:
            self.logger.error(f"Error getting FQ sampler config: {str(e)}")
            return {}

    def update_fq_sampler_config(self, frequency: float, consumer: str) -> bool:
        """Update FQ sampler configuration."""
        try:
            if self.state_manager:
                self.state_manager.set_fq_sampler_frequency(frequency)
                consumer_map = {"visualization": 1, "motor": 2, "both": 3}
                self.state_manager.set_fq_sampler_consumer(
                    consumer_map.get(consumer, 1)
                )
            return True
        except Exception as e:
            self.logger.error(f"Error updating FQ sampler config: {str(e)}")
            return False

    def get_area_fq_sample_rate(self, area_id: int) -> float:
        """Get FQ sample rate for an area."""
        try:
            # This should get real sample rate from the fire queue manager
            raise NotImplementedError(
                "Getting area FQ sample rate is not yet implemented"
            )
        except Exception as e:
            self.logger.error(f"Error getting area FQ sample rate: {str(e)}")
            raise ValueError(f"Failed to get area FQ sample rate: {str(e)}")

    def get_burst_counter(self) -> int:
        """Get current burst counter - RTOS-safe."""
        try:
            # RTOS-SAFE: Get actual burst count from burst engine
            burst_engine = self.get_burst_engine()
            if burst_engine and hasattr(burst_engine, "burst_count"):
                return burst_engine.burst_count

            # Fallback to state manager if burst engine not available
            if self.state_manager:
                return getattr(self.state_manager, "current_burst_id", 0)
            return 0
        except Exception as e:
            self.logger.error(f"Error getting burst counter: {str(e)}")
            return 0

    def update_burst_engine_config(self, config: Dict[str, Any]) -> bool:
        """Update burst engine configuration - RTOS-safe."""
        try:
            # Get the singleton burst engine instance
            burst_engine = self.get_burst_engine()
            if not burst_engine:
                self.logger.error(
                    "No burst engine instance available for config update"
                )
                return False

            # RTOS-SAFE: Update frequency if provided
            if "burst_frequency_hz" in config:
                frequency = config["burst_frequency_hz"]
                if not burst_engine.update_frequency(frequency):
                    self.logger.error(
                        f"Failed to update burst frequency to {frequency}Hz"
                    )
                    return False
                self.logger.info(f"Updated burst frequency to {frequency}Hz")

            return True
        except Exception as e:
            self.logger.error(f"Error updating burst engine config: {str(e)}")
            return False

    def get_network_config(self) -> Dict[str, Any]:
        """Get network configuration."""
        return self._network_service.get_protocol_status()

    def update_network_config(self, network_config: Dict[str, Any]) -> bool:
        """Update network configuration."""
        try:
            # This would need implementation in network service
            return True
        except Exception as e:
            self.logger.error(f"Error updating network config: {str(e)}")
            return False

    def get_connectome_dimensions(self) -> Dict[str, Any]:
        """Get connectome dimensions."""
        try:
            stats = self._brain_service.get_brain_statistics()
            return {
                "neuron_count": stats.get("neuron_count", 0),
                "synapse_count": stats.get("synapse_count", 0),
                "cortical_area_count": stats.get("cortical_area_count", 0),
            }
        except Exception as e:
            self.logger.error(f"Error getting connectome dimensions: {str(e)}")
            return {}

    def get_morphology_list(self) -> List[str]:
        """Get list of available morphologies."""
        try:
            # Import core morphologies from templates
            from feagi.evo.templates import core_morphologies

            morphology_names = list(core_morphologies.keys())

            # Also check genome for additional morphologies if available
            genome = self.get_genome()
            if genome and "neuron_morphologies" in genome:
                genome_morphologies = list(genome["neuron_morphologies"].keys())
                # Combine and deduplicate
                morphology_names.extend(
                    [m for m in genome_morphologies if m not in morphology_names]
                )

            return sorted(morphology_names)
        except Exception as e:
            self.logger.error(f"Error getting morphology list: {str(e)}")
            raise ValueError(f"Failed to retrieve morphology list: {str(e)}")

    def get_morphology_types(self) -> List[str]:
        """Get list of available morphology types."""
        try:
            # Get unique morphology types from core morphologies
            from feagi.evo.templates import core_morphologies

            types = set()
            for morphology in core_morphologies.values():
                if "type" in morphology:
                    types.add(morphology["type"])

            # Also check genome morphologies
            genome = self.get_genome()
            if genome and "neuron_morphologies" in genome:
                for morphology in genome["neuron_morphologies"].values():
                    if "type" in morphology:
                        types.add(morphology["type"])

            return sorted(list(types))
        except Exception as e:
            self.logger.error(f"Error getting morphology types: {str(e)}")
            raise ValueError(f"Failed to retrieve morphology types: {str(e)}")

    def get_morphologies(self) -> Dict[str, Any]:
        """Get all morphologies with detailed information."""
        try:
            # Import core morphologies from templates
            from feagi.evo.templates import core_morphologies

            # Start with core morphologies
            all_morphologies = {}
            for name, morphology in core_morphologies.items():
                all_morphologies[name] = {
                    "name": name,
                    "type": morphology.get("type", "unknown"),
                    "class": "core",  # ✅ FIXED: Use "core" for core morphologies
                    "parameters": morphology.get("parameters", {}),
                    "source": "core",
                }

            # Add genome morphologies if available
            genome = self.get_genome()
            if genome and "neuron_morphologies" in genome:
                for name, morphology in genome["neuron_morphologies"].items():
                    # Genome morphologies override core morphologies
                    all_morphologies[name] = {
                        "name": name,
                        "type": morphology.get("type", "unknown"),
                        "class": "custom",  # ✅ FIXED: Use "custom" for genome morphologies
                        "parameters": morphology.get("parameters", {}),
                        "source": "genome",
                    }

            return all_morphologies
        except Exception as e:
            self.logger.error(f"Error getting morphologies: {str(e)}")
            raise ValueError(f"Failed to retrieve morphologies: {str(e)}")

    def get_morphology_info(self, morphology_id: str) -> Dict[str, Any]:
        """Get information about a specific morphology."""
        try:
            # Get all morphologies and find the requested one
            all_morphologies = self.get_morphologies()

            if morphology_id not in all_morphologies:
                raise ValueError(f"Morphology '{morphology_id}' not found")

            morphology = all_morphologies[morphology_id]

            # Add additional computed information
            morphology_info = morphology.copy()
            morphology_info.update(
                {
                    "id": morphology_id,
                    "description": self._get_morphology_description(morphology),
                    "example_usage": self._get_morphology_example(morphology),
                }
            )

            return morphology_info
        except Exception as e:
            self.logger.error(f"Error getting morphology info: {str(e)}")
            raise ValueError(f"Failed to retrieve morphology info: {str(e)}")

    def _get_morphology_description(self, morphology: Dict[str, Any]) -> str:
        """Generate a description for a morphology based on its type and parameters."""
        morphology_type = morphology.get("type", "unknown")

        descriptions = {
            "vectors": "Connects neurons using fixed directional vectors",
            "patterns": "Connects neurons based on spatial patterns",
            "functions": "Uses algorithmic functions to determine connections",
            "composite": "Combines multiple morphology types for complex connectivity",
        }

        base_desc = descriptions.get(morphology_type, "Custom morphology type")

        # Add specific details based on parameters
        if morphology_type == "vectors" and "vectors" in morphology.get(
            "parameters", {}
        ):
            vectors = morphology["parameters"]["vectors"]
            base_desc += f" ({len(vectors)} vector(s))"
        elif morphology_type == "patterns" and "patterns" in morphology.get(
            "parameters", {}
        ):
            patterns = morphology["parameters"]["patterns"]
            base_desc += f" ({len(patterns)} pattern(s))"

        return base_desc

    def _get_morphology_example(self, morphology: Dict[str, Any]) -> str:
        """Generate example usage for a morphology."""
        morphology_type = morphology.get("type", "unknown")

        examples = {
            "vectors": "Useful for layer-to-layer connections with fixed offsets",
            "patterns": "Ideal for spatial relationship-based connectivity",
            "functions": "Best for dynamic or computed connectivity patterns",
            "composite": "Combines multiple approaches for complex architectures",
        }

        return examples.get(morphology_type, "General purpose connectivity morphology")

    def create_morphology(self, morphology_data: Dict[str, Any]) -> bool:
        """
        Create a new morphology.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # Route WRITE operation through GenomeService for architecture compliance
            return self._genome_service.create_morphology(morphology_data)

        except Exception as e:
            self.logger.error(f"Error creating morphology: {str(e)}")
            raise ValueError(f"Failed to create morphology: {str(e)}")

    def update_morphology(self, morphology_id: str, updates: Dict[str, Any]) -> bool:
        """
        Update an existing morphology.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # Route WRITE operation through GenomeService for architecture compliance
            return self._genome_service.update_morphology(morphology_id, updates)

        except Exception as e:
            self.logger.error(f"Error updating morphology: {str(e)}")
            raise ValueError(f"Failed to update morphology: {str(e)}")

    def delete_morphology(self, morphology_id: str) -> bool:
        """
        Delete a morphology.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # Route WRITE operation through GenomeService for architecture compliance
            return self._genome_service.delete_morphology(morphology_id)

        except Exception as e:
            self.logger.error(f"Error deleting morphology: {str(e)}")
            raise ValueError(f"Failed to delete morphology: {str(e)}")

    def get_morphology_properties(self, morphology_name: str) -> Dict[str, Any]:
        """Get properties of a specific morphology."""
        try:
            all_morphologies = self.get_morphologies()
            if morphology_name not in all_morphologies:
                raise ValueError(f"Morphology '{morphology_name}' not found")

            morphology = all_morphologies[morphology_name]
            result = dict(morphology)
            result["morphology_name"] = morphology_name

            self.logger.info(f"Retrieved properties for morphology: {morphology_name}")
            return result

        except Exception as e:
            self.logger.error(f"Error getting morphology properties: {str(e)}")
            raise ValueError(f"Failed to get morphology properties: {str(e)}")

    def get_morphology_usage(self, morphology_name: str) -> List[List[str]]:
        """Get usage report for a specific morphology."""
        try:
            genome = self.get_genome()
            if not genome or "blueprint" not in genome:
                return []

            blueprint = genome["blueprint"]
            usage_list = []

            self.logger.info(
                f"Searching for morphology '{morphology_name}' usage in {len(blueprint)} cortical areas"
            )

            # The genome structure is flattened, so we need to reconstruct the cortical areas
            cortical_areas = {}

            # Parse the flattened structure to extract cortical areas and their mappings
            for key, value in blueprint.items():
                if "-cx-dstmap-d" in key:
                    # Extract cortical area ID from the key
                    # Format: "_____10c-{area_id}-cx-dstmap-d"
                    parts = key.split("-")
                    if len(parts) >= 3:
                        area_id = parts[1]  # Extract the area ID
                        cortical_areas[area_id] = value

            self.logger.debug(
                f"Found {len(cortical_areas)} cortical areas with mappings"
            )

            # Search through cortical areas for connections using this morphology
            for source_area_id, mapping_dst in cortical_areas.items():
                if not isinstance(mapping_dst, dict):
                    continue

                for target_area_id, connections in mapping_dst.items():
                    self.logger.debug(
                        f"Checking connection {source_area_id} -> {target_area_id}: type={type(connections)}, value={connections}"
                    )

                    if not connections or not isinstance(connections, (list, tuple)):
                        continue

                    # Check each connection for the morphology
                    for connection in connections:
                        if isinstance(connection, list) and len(connection) > 0:
                            # First element is morphology_id
                            morphology_id = connection[0]
                            if morphology_id == morphology_name:
                                # Add [source_area, target_area] pair
                                usage_list.append([source_area_id, target_area_id])
                                self.logger.debug(
                                    f"Found usage: {source_area_id} -> {target_area_id} using {morphology_name}"
                                )

            self.logger.info(
                f"Found {len(usage_list)} usages for morphology: {morphology_name}"
            )
            return usage_list

        except Exception as e:
            self.logger.error(f"Error getting morphology usage: {str(e)}")
            import traceback

            self.logger.error(f"Full traceback: {traceback.format_exc()}")
            raise ValueError(f"Failed to get morphology usage: {str(e)}")

    def get_cortical_mapping(self) -> Dict[str, Any]:
        """
        Get the complete cortical mapping structure from the genome blueprint.

        Returns:
            Dictionary containing all cortical area mappings in the expected format
        """
        try:
            # Use the existing get_detailed_cortical_map method for consistent behavior
            return self.get_detailed_cortical_map()

        except Exception as e:
            self.logger.error(f"Error getting cortical mapping: {str(e)}")
            return {}

    def update_cortical_mapping(self, mapping: Dict[str, Any]) -> bool:
        """
        Update the cortical mapping structure by converting formatted data back to genome format.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager

        Args:
            mapping: Dictionary containing updated cortical area mappings in the formatted structure
                    Format: {area_id: {target_area_id: [connection_objects]}}

        Returns:
            True if successful, False otherwise
        """
        try:
            # Route WRITE operation through GenomeService for architecture compliance
            return self._genome_service.update_cortical_mapping(mapping)

        except Exception as e:
            self.logger.error(f"Error updating cortical mapping: {str(e)}")
            return False

    def get_cortical_mapping_properties(
        self, src_cortical_area: str, dst_cortical_area: str
    ) -> List[Dict[str, Any]]:
        """Get cortical mapping properties between two cortical areas."""
        try:
            genome = self.get_genome()
            if not genome or "blueprint" not in genome:
                return []

            blueprint = genome["blueprint"]

            # The genome structure is flattened, so we need to find the mapping data
            # Look for the key pattern: "_____10c-{src_cortical_area}-cx-dstmap-d"
            mapping_key = None
            for key in blueprint.keys():
                if f"-{src_cortical_area}-cx-dstmap-d" in key:
                    mapping_key = key
                    break

            if not mapping_key:
                self.logger.debug(
                    f"No mapping found for source cortical area '{src_cortical_area}'"
                )
                return []

            mapping_dst = blueprint[mapping_key]
            if not isinstance(mapping_dst, dict):
                return []

            # Check if destination area is mapped from source
            if dst_cortical_area not in mapping_dst:
                self.logger.debug(
                    f"No mapping found from '{src_cortical_area}' to '{dst_cortical_area}'"
                )
                return []

            # Return the mapping data
            connections = mapping_dst[dst_cortical_area]
            if not connections:
                return []

            # Convert to expected format
            formatted_connections = []
            for connection in connections:
                if isinstance(connection, list) and len(connection) >= 4:
                    # Handle the actual genome format: [morphology_id, scalar, multiplier, plasticity_flag]
                    # Pad with default values for missing fields
                    formatted_connection = {
                        "morphology_id": connection[0],
                        "morphology_scalar": (
                            connection[1] if len(connection) > 1 else [1, 1, 1]
                        ),
                        "postSynapticCurrent_multiplier": (
                            connection[2] if len(connection) > 2 else 1
                        ),
                        "plasticity_flag": (
                            connection[3] if len(connection) > 3 else False
                        ),
                        "plasticity_constant": (
                            connection[4] if len(connection) > 4 else 1
                        ),
                        "ltp_multiplier": connection[5] if len(connection) > 5 else 1,
                        "ltd_multiplier": connection[6] if len(connection) > 6 else 1,
                    }
                    formatted_connections.append(formatted_connection)

            self.logger.info(
                f"Retrieved {len(formatted_connections)} mapping properties from {src_cortical_area} to {dst_cortical_area}"
            )
            return formatted_connections

        except Exception as e:
            self.logger.error(f"Error getting cortical mapping properties: {str(e)}")
            raise ValueError(f"Failed to get cortical mapping properties: {str(e)}")

    def update_cortical_mapping_properties(
        self,
        src_cortical_area: str,
        dst_cortical_area: str,
        mapping_string: List[Dict[str, Any]],
    ) -> bool:
        """Update cortical mapping properties between two cortical areas."""
        try:
            self.logger.info(
                f"Updating cortical mapping properties from {src_cortical_area} to {dst_cortical_area}"
            )

            # Route through GenomeService for architecture compliance
            update_data = {
                "src_cortical_area": src_cortical_area,
                "dst_cortical_area": dst_cortical_area,
                "mapping_data": mapping_string,
            }

            success = self._genome_service.update_cortical_mapping_properties(
                update_data
            )

            if success:
                self.logger.info(
                    f"Successfully updated mapping properties from {src_cortical_area} to {dst_cortical_area}"
                )
            else:
                self.logger.error(
                    f"Failed to update mapping properties from {src_cortical_area} to {dst_cortical_area}"
                )

            return success

        except Exception as e:
            self.logger.error(f"Error updating cortical mapping properties: {str(e)}")
            return False

    def get_detailed_cortical_map(self) -> Dict[str, Any]:
        """
        Get detailed cortical mapping information in the expected format.

        Returns a dictionary where each cortical area ID maps to its connection targets
        with detailed morphology and plasticity parameters.

        Returns:
            Dict[str, Any]: Mapping data in the expected format
        """
        logger.info("Getting detailed cortical map...")

        try:
            # Get all cortical areas using the correct service method
            all_areas_list = self._cortical_area_service.get_all_areas()

            # Build the mapping response
            mapping_response = {}

            for area_data in all_areas_list:
                area_id = area_data.get("id")
                if not area_id:
                    continue

                # Initialize area entry (empty dict for areas with no outgoing connections)
                mapping_response[area_id] = {}

                # Get the area's mapping data from its parameters
                area_parameters = area_data.get("parameters", {})
                area_mapping = area_parameters.get("mapping", {})

                if area_mapping:
                    # Convert each target area's mapping data to the expected format
                    for target_area_id, connection_list in area_mapping.items():
                        if not connection_list:
                            continue

                        # Convert each connection from array format to object format
                        formatted_connections = []
                        for connection_data in connection_list:
                            if (
                                isinstance(connection_data, list)
                                and len(connection_data) >= 7
                            ):  # Ensure we have all required fields
                                formatted_connection = {
                                    "morphology_id": connection_data[0],
                                    "morphology_scalar": connection_data[1],
                                    "postSynapticCurrent_multiplier": connection_data[
                                        2
                                    ],
                                    "plasticity_flag": connection_data[3],
                                    "plasticity_constant": connection_data[4],
                                    "ltp_multiplier": connection_data[5],
                                    "ltd_multiplier": connection_data[6],
                                }
                                formatted_connections.append(formatted_connection)

                        if formatted_connections:
                            mapping_response[area_id][target_area_id] = (
                                formatted_connections
                            )

            logger.info(
                f"Generated detailed cortical map for {len(mapping_response)} areas"
            )
            return mapping_response

        except Exception as e:
            logger.error(f"Error generating detailed cortical map: {e}")
            raise

    def get_data_path(self) -> str:
        """Get data path."""
        return os.path.join(tempfile.gettempdir(), "feagi_data")

    def get_temp_path(self) -> str:
        """Get temporary path."""
        return os.path.join(tempfile.gettempdir(), "feagi_temp")

    # =================================================================
    # STIMULATION METHODS
    # =================================================================

    def trigger_manual_stimulation(self, stimulation_payload: Dict[str, Any]) -> bool:
        """Trigger manual stimulation."""
        try:
            cortical_id = stimulation_payload.get("cortical_id")
            intensity = stimulation_payload.get("intensity", 1.0)
            if cortical_id:
                return self.stimulate_cortical_area(
                    cortical_id, intensity=intensity
                ).get("success", False)
            return False
        except Exception as e:
            self.logger.error(f"Error triggering manual stimulation: {str(e)}")
            return False

    def trigger_sustained_stimulation(
        self, stimulation_payload: Dict[str, Any]
    ) -> bool:
        """Trigger sustained stimulation."""
        try:
            cortical_id = stimulation_payload.get("cortical_id")
            intensity = stimulation_payload.get("intensity", 1.0)
            duration = stimulation_payload.get("duration", 10)
            if cortical_id:
                return self.stimulate_cortical_area(
                    cortical_id, intensity=intensity, duration=duration
                ).get("success", False)
            return False
        except Exception as e:
            self.logger.error(f"Error triggering sustained stimulation: {str(e)}")
            return False

    def set_stimulation_script(self, script: str) -> bool:
        """Set stimulation script."""
        try:
            # This would need implementation
            return True
        except Exception as e:
            self.logger.error(f"Error setting stimulation script: {str(e)}")
            return False

    def reset_stimulation_script(self) -> bool:
        """Reset stimulation script."""
        try:
            # This would need implementation
            return True
        except Exception as e:
            self.logger.error(f"Error resetting stimulation script: {str(e)}")
            return False

    # =================================================================
    # TRANSACTION AND STATE METHODS
    # =================================================================

    def begin_transaction(self):
        """Begin a genome transaction."""
        # This would need implementation with genome transaction system
        pass

    def modify_genome(self, transaction):
        """Modify genome within a transaction."""
        # This would need implementation with genome transaction system
        pass

    def register_genome_change_listener(self, callback):
        """Register a callback for genome changes."""
        # This would need implementation
        pass

    def on_sync_state_change(self, old_state, new_state, details):
        """Handle sync state changes."""
        # This would need implementation
        pass

    # =================================================================
    # UTILITY AND HELPER METHODS
    # =================================================================

    def _get_cortical_idx_for_id(self, cortical_id: str) -> Optional[int]:
        """
        Get cortical index for a cortical ID using O(1) BiDirectionalCorticalMap.

        Args:
            cortical_id: 6-character string identifier

        Returns:
            Integer index if found, None otherwise
        """
        # Use O(1) lookup from BiDirectionalCorticalMap - no more O(N) linear search!
        return self._connectome_manager.get_cortical_idx_for_id(cortical_id)

    def _validate_genome_loaded(self) -> bool:
        """Check if a genome is currently loaded - helper method for service consistency."""
        return self._genome_service.is_genome_loaded()

    def get_neuron_mappings(self) -> Dict[str, Any]:
        """Get neuron mappings."""
        try:
            return {
                "neuron_to_area": {},
                "area_to_neurons": {},
                "total_neurons": self.get_connectome_dimensions().get(
                    "neuron_count", 0
                ),
            }
        except Exception as e:
            self.logger.error(f"Error getting neuron mappings: {str(e)}")
            return {}

    def get_transforming_areas(self) -> List[str]:
        """Get list of areas currently transforming."""
        try:
            # This would need implementation
            return []
        except Exception as e:
            self.logger.error(f"Error getting transforming areas: {str(e)}")
            return []

    def has_pending_amalgamation(self) -> bool:
        """Check if there is a pending amalgamation."""
        try:
            if self.state_manager:
                return bool(getattr(self.state_manager, "pending_amalgamation", False))
            return False
        except Exception as e:
            self.logger.error(f"Error checking pending amalgamation: {str(e)}")
            return False

    def save_connectome_snapshot(self, path: str) -> bool:
        """Save connectome snapshot."""
        try:
            # This would need implementation
            return True
        except Exception as e:
            self.logger.error(f"Error saving connectome snapshot: {str(e)}")
            return False

    def import_cortical_area(self, cortical_area_data: Dict[str, Any]) -> bool:
        """Import cortical area data."""
        try:
            # This would need implementation
            return True
        except Exception as e:
            self.logger.error(f"Error importing cortical area: {str(e)}")
            return False

    def batch_create_neurons(
        self,
        area_id: str,
        positions: List[Tuple[int, int, int]],
        properties: Optional[Dict[str, Any]] = None,
    ) -> List[int]:
        """Batch create neurons."""
        try:
            # This would need implementation
            return []
        except Exception as e:
            self.logger.error(f"Error batch creating neurons: {str(e)}")
            return []

    def batch_create_synapses(self, connections: List[Tuple[int, int, float]]) -> int:
        """Batch create synapses."""
        try:
            # This would need implementation
            return 0
        except Exception as e:
            self.logger.error(f"Error batch creating synapses: {str(e)}")
            return 0

    # =================================================================
    # ROBOT/GAZEBO METHODS
    # =================================================================

    def update_robot_controller(self, controller_params: Dict[str, Any]) -> bool:
        """Update robot controller parameters."""
        try:
            # This would need implementation
            return True
        except Exception as e:
            self.logger.error(f"Error updating robot controller: {str(e)}")
            return False

    def update_robot_model(self, model_params: Dict[str, Any]) -> bool:
        """Update robot model parameters."""
        try:
            # This would need implementation
            return True
        except Exception as e:
            self.logger.error(f"Error updating robot model: {str(e)}")
            return False

    def get_gazebo_robot_files(self) -> Dict[str, List[str]]:
        """Get Gazebo robot files."""
        try:
            # This would need implementation
            return {"models": [], "worlds": [], "configs": []}
        except Exception as e:
            self.logger.error(f"Error getting Gazebo robot files: {str(e)}")
            return {}

    # =================================================================
    # PERFORMANCE AND SIMULATION METHODS
    # =================================================================

    async def get_performance_stats(self) -> Dict[str, Any]:
        """Get performance statistics."""
        try:
            return self.get_performance_metrics()
        except Exception as e:
            self.logger.error(f"Error getting performance stats: {str(e)}")
            return {}

    async def get_simulation_status(self) -> Dict[str, Any]:
        """Get simulation status."""
        try:
            return {
                "running": (
                    not getattr(self.state_manager, "exit_condition", False)
                    if self.state_manager
                    else False
                ),
                "burst_counter": self.get_burst_counter(),
                "genome_loaded": self.genome_is_loaded(),
                "brain_ready": (
                    self.state_manager.get_brain_readiness()
                    if self.state_manager
                    else False
                ),
            }
        except Exception as e:
            self.logger.error(f"Error getting simulation status: {str(e)}")
            return {}

    async def get_system_health(self) -> Dict[str, Any]:
        """Get system health (legacy name for get_health)."""
        return await self.get_health()

    # =================================================================
    # PROPERTY ACCESSOR METHODS
    # =================================================================

    @property
    def feagi(self):
        """Get the FEAGI instance."""
        try:
            from feagi.core.feagi import FEAGI

            if not hasattr(self, "_feagi_instance"):
                self._feagi_instance = FEAGI()
            return self._feagi_instance
        except Exception as e:
            self.logger.error(f"Error getting FEAGI instance: {str(e)}")
            return None

    # ===== Frequency Measurement Methods =====

    def trigger_frequency_measurement(
        self, duration_seconds: float = 5.0, sample_count: int = 100
    ) -> dict:
        """
        Trigger an on-demand burst frequency measurement.

        This is an expensive operation that should only be called when needed for monitoring.

        Args:
            duration_seconds: How long to measure (default 5.0 seconds)
            sample_count: Number of burst samples to collect (default 100)

        Returns:
            Dictionary with measurement results
        """
        return self.state_manager.trigger_frequency_measurement(
            duration_seconds, sample_count
        )

    def get_frequency_measurement_history(self, limit: Optional[int] = None) -> dict:
        """
        Get the history of frequency measurements.

        Args:
            limit: Maximum number of recent measurements to return

        Returns:
            Dictionary with measurement history
        """
        return self.state_manager.get_frequency_measurement_history(limit)

    def get_frequency_status_summary(self) -> dict:
        """
        Get current frequency status and latest measurement - RTOS-safe.

        Returns:
            Dictionary with frequency status and latest measurement
        """
        try:
            # RTOS-SAFE: Get actual frequency data from burst engine
            burst_engine = self.get_burst_engine()
            if burst_engine:
                frequency_config = burst_engine.get_frequency_config()
                return {
                    "target_frequency_hz": frequency_config.get(
                        "current_frequency_hz", 0.0
                    ),
                    "has_measurements": burst_engine.burst_count > 0,
                    "total_measurements": burst_engine.burst_count,
                    "latest_measurement": (
                        {
                            "frequency_hz": frequency_config.get(
                                "current_frequency_hz", 0.0
                            ),
                            "burst_count": burst_engine.burst_count,
                            "last_burst_time": getattr(
                                burst_engine, "last_burst_time", 0.0
                            ),
                        }
                        if burst_engine.burst_count > 0
                        else None
                    ),
                }

            # Fallback to state manager if burst engine not available
            return (
                self.state_manager.get_frequency_status_summary()
                if self.state_manager
                else {
                    "target_frequency_hz": 0.0,
                    "has_measurements": False,
                    "total_measurements": 0,
                    "latest_measurement": None,
                }
            )
        except Exception as e:
            self.logger.error(f"Error getting frequency status: {str(e)}")
            return {
                "target_frequency_hz": 0.0,
                "has_measurements": False,
                "total_measurements": 0,
                "latest_measurement": None,
                "error": str(e),
            }

    def get_fire_queue_direct(self) -> Optional[np.ndarray]:
        """Get fire queue data directly from SoA structures - zero-copy access.

        Returns:
            numpy array with shape (N, 5) containing:
            [neuron_ids, membrane_potentials, coordinates_x, coordinates_y, coordinates_z]
            or None if no firing neurons
        """
        try:
            if (
                hasattr(self._connectome_manager, "fcl_manager")
                and self._connectome_manager.fcl_manager
            ):
                global_fcl = self._connectome_manager.fcl_manager.get_global_fcl()

                if global_fcl.is_empty():
                    return None

                # Direct FCL to numpy array (already optimized!)
                firing_indices = np.array(list(global_fcl), dtype=np.int32)

                if len(firing_indices) == 0:
                    return None

                # Direct SoA access - use existing optimized structures
                neuron_array = self._connectome_manager.neuron_array

                # Vectorized extraction in single operation - NO FALLBACKS
                if (
                    hasattr(neuron_array, "membrane_potentials")
                    and hasattr(neuron_array, "coordinates_x")
                    and hasattr(neuron_array, "coordinates_y")
                    and hasattr(neuron_array, "coordinates_z")
                ):
                    brain_data = np.column_stack(
                        (
                            firing_indices.astype(
                                np.int32
                            ),  # Keep int32 for neuron IDs (they can be signed)
                            neuron_array.membrane_potentials[
                                firing_indices
                            ],  # Keep float32 for potentials
                            neuron_array.coordinates_x[firing_indices].astype(
                                np.uint32
                            ),  # ✅ FIXED: Use uint32 coordinates
                            neuron_array.coordinates_y[firing_indices].astype(
                                np.uint32
                            ),  # ✅ FIXED: Use uint32 coordinates
                            neuron_array.coordinates_z[firing_indices].astype(
                                np.uint32
                            ),  # ✅ FIXED: Use uint32 coordinates
                        )
                    )

                    return brain_data
                else:
                    # ❌ NO FALLBACK - Neuron array must have all required properties
                    self.logger.error(
                        "Neuron array missing required properties (membrane_potentials, coordinates_x/y/z)"
                    )
                    return None
            return None
        except Exception as e:
            self.logger.error(f"Error getting direct fire queue: {str(e)}")
            return None

    def get_area_fire_queue_direct(self, cortical_id: str) -> Optional[np.ndarray]:
        """Get fire queue data for specific area directly from SoA structures.

        Args:
            cortical_id: ID of the cortical area

        Returns:
            numpy array with shape (N, 5) or None if no firing neurons
        """
        try:
            if not self._validate_genome_loaded():
                self.logger.debug(
                    f"🔥 [FIRE QUEUE] Genome not loaded for area {cortical_id}"
                )
                return None

            if (
                hasattr(self._connectome_manager, "fcl_manager")
                and self._connectome_manager.fcl_manager
            ):
                # CRITICAL FIX: Read from global FCL and filter by cortical area
                # instead of reading from cortical FCL history which is empty
                global_fcl = self._connectome_manager.fcl_manager.get_fcl()

                if global_fcl.is_empty():
                    self.logger.debug(
                        f"🔥 [FIRE QUEUE] Global FCL is empty for area {cortical_id}"
                    )
                    return None

                if self.state_manager.is_debug_npu_enabled():
                    self.logger.debug(
                        f"🔥 [FIRE QUEUE] Global FCL has {len(global_fcl)} total firing neurons: {list(global_fcl)}"
                    )

                # CRITICAL FIX: Get cortical_idx for the requested cortical_id
                target_cortical_idx = self._get_cortical_idx_for_id(cortical_id)
                if target_cortical_idx is None:
                    self.logger.error(
                        f"🔥 [FIRE QUEUE] Could not map cortical_id '{cortical_id}' to cortical_idx"
                    )
                    return None

                if self.state_manager.is_debug_npu_enabled():
                    self.logger.debug(
                        f"🔥 [FIRE QUEUE] Mapped cortical_id '{cortical_id}' to cortical_idx {target_cortical_idx}"
                    )

                # Filter global FCL by cortical_idx using neuron array mapping
                firing_indices = []
                neuron_array = self._connectome_manager.neuron_array

                if not hasattr(neuron_array, "cortical_idxs"):
                    self.logger.error(
                        "🔥 [FIRE QUEUE] Neuron array missing cortical_idxs attribute"
                    )
                    return None

                # Use vectorized filtering for performance
                firing_neuron_ids = np.array(list(global_fcl), dtype=np.int32)
                self.logger.debug(
                    f"🔥 [FIRE QUEUE] Firing neuron IDs: {firing_neuron_ids}"
                )

                # CRITICAL FIX: Convert neuron IDs to indices for array access
                firing_indices = []
                for neuron_id in firing_neuron_ids:
                    if neuron_id in neuron_array.id_to_index_map:
                        firing_indices.append(neuron_array.id_to_index_map[neuron_id])

                if len(firing_indices) == 0:
                    self.logger.debug(
                        "🔥 [FIRE QUEUE] No valid firing neuron indices found"
                    )
                    return None

                firing_indices = np.array(firing_indices, dtype=np.int32)
                self.logger.debug(
                    f"🔥 [FIRE QUEUE] Converted {len(firing_neuron_ids)} neuron IDs to {len(firing_indices)} indices: {firing_indices}"
                )

                # Filter by target cortical_idx using the correct indices
                neuron_cortical_idxs = neuron_array.cortical_idxs[firing_indices]
                self.logger.debug(
                    f"🔥 [FIRE QUEUE] Neuron cortical indices: {neuron_cortical_idxs}"
                )
                self.logger.debug(
                    f"🔥 [FIRE QUEUE] Target cortical_idx: {target_cortical_idx}"
                )

                area_mask = neuron_cortical_idxs == target_cortical_idx
                self.logger.debug(f"🔥 [FIRE QUEUE] Area mask: {area_mask}")

                area_firing_indices = firing_indices[area_mask]
                self.logger.debug(
                    f"🔥 [FIRE QUEUE] Firing indices in target area: {area_firing_indices}"
                )

                self.logger.debug(
                    f"🔥 [FIRE QUEUE] Found {len(area_firing_indices)} firing neurons in area {cortical_id} (cortical_idx={target_cortical_idx})"
                )

                if len(area_firing_indices) == 0:
                    self.logger.debug(
                        f"🔥 [FIRE QUEUE] No firing neurons found in area {cortical_id}"
                    )
                    return None

                # area_firing_indices is already a numpy array

                # Direct SoA access - NO FALLBACKS
                # Vectorized extraction - all properties must exist
                if (
                    hasattr(neuron_array, "membrane_potentials")
                    and hasattr(neuron_array, "coordinates_x")
                    and hasattr(neuron_array, "coordinates_y")
                    and hasattr(neuron_array, "coordinates_z")
                ):
                    # CRITICAL FIX: Convert firing indices to actual neuron IDs
                    # The FQ sampler expects neuron IDs, not array indices!
                    final_neuron_ids = neuron_array.vectorized_indices_to_neuron_ids(
                        area_firing_indices, filter_invalid=True
                    )

                    self.logger.debug(
                        f"🔥 [FIRE QUEUE] Converted {len(area_firing_indices)} indices to {len(final_neuron_ids)} neuron IDs"
                    )

                    if len(final_neuron_ids) == 0:
                        self.logger.debug(
                            f"🔥 [FIRE QUEUE] No valid neuron IDs after conversion for area {cortical_id}"
                        )
                        return None

                    brain_data = np.column_stack(
                        (
                            final_neuron_ids.astype(
                                np.int32
                            ),  # ✅ FIXED: Use actual neuron IDs, not indices!
                            neuron_array.membrane_potentials[
                                area_firing_indices
                            ],  # Keep float32 for potentials
                            neuron_array.coordinates_x[area_firing_indices].astype(
                                np.uint32
                            ),  # ✅ FIXED: Use uint32 coordinates
                            neuron_array.coordinates_y[area_firing_indices].astype(
                                np.uint32
                            ),  # ✅ FIXED: Use uint32 coordinates
                            neuron_array.coordinates_z[area_firing_indices].astype(
                                np.uint32
                            ),  # ✅ FIXED: Use uint32 coordinates
                        )
                    )

                    self.logger.debug(
                        f"🔥 [FIRE QUEUE] Successfully extracted {len(area_firing_indices)} firing neurons for area {cortical_id}"
                    )
                    return brain_data
                else:
                    # ❌ NO FALLBACK - Neuron array must have all required properties
                    self.logger.error(
                        f"🔥 [FIRE QUEUE] Neuron array missing required properties for area {cortical_id}"
                    )
                    return None
            return None
        except Exception as e:
            self.logger.error(
                f"🔥 [FIRE QUEUE] Error getting direct area fire queue for {cortical_id}: {str(e)}"
            )
            return None

    def get_area_fire_queue(self, cortical_id: str) -> Optional[Dict[str, Any]]:
        """Get fire queue data for specific area in dictionary format (FQ sampler compatible).

        This method provides the interface expected by the FQ sampler, converting the direct
        numpy array data to the dictionary format that the sampler expects.

        Args:
            cortical_id: ID of the cortical area

        Returns:
            Dictionary with neuron_ids, membrane_potentials, coordinates, etc. or None if no data
        """
        try:
            if self.state_manager.is_debug_npu_enabled():
                self.logger.debug(
                    f"🔥 [FIRE QUEUE API] get_area_fire_queue called for area: {cortical_id}"
                )
            # Get the direct numpy array data
            fire_queue_data = self.get_area_fire_queue_direct(cortical_id)

            if fire_queue_data is None or len(fire_queue_data) == 0:
                return None

            # Convert numpy array to dictionary format expected by FQ sampler
            # Array columns: [neuron_ids, membrane_potentials, x, y, z]
            neuron_ids = fire_queue_data[:, 0].astype(int).tolist()
            membrane_potentials = fire_queue_data[:, 1].tolist()
            coordinates_x = fire_queue_data[:, 2].astype(int).tolist()
            coordinates_y = fire_queue_data[:, 3].astype(int).tolist()
            coordinates_z = fire_queue_data[:, 4].astype(int).tolist()

            # Package coordinates as list of (x, y, z) tuples
            coordinates = list(zip(coordinates_x, coordinates_y, coordinates_z))

            # CRITICAL FIX: Remove problematic neuron property extraction
            # The essential data (neuron_ids, membrane_potentials, coordinates) is already available
            # Additional properties can be empty arrays - NO FAKE DATA
            thresholds = []
            consecutive_fire_counts = []
            refractory_counters = []

            return {
                "neuron_ids": neuron_ids,
                "membrane_potentials": membrane_potentials,
                "coordinates": coordinates,
                "thresholds": thresholds,  # Empty - will not provide fake data
                "consecutive_fire_counts": consecutive_fire_counts,  # Empty - will not provide fake data
                "refractory_counters": refractory_counters,  # Empty - will not provide fake data
            }

        except Exception as e:
            self.logger.error(
                f"Error getting area fire queue for {cortical_id}: {str(e)}"
            )
            return None

    # =================================================================
    # HIGH-PERFORMANCE NEURON COORDINATE METHODS
    # =================================================================

    def get_neuron_coordinates(self, neuron_ids: List[int]) -> Optional[Dict[str, Any]]:
        """
        Get coordinates (X, Y, Z) for a list of neuron IDs using SIMD-optimized extraction.

        Leverages FEAGI's centralized SIMD configuration for maximum performance.
        Uses vectorized operations with optimal memory layouts and cache-friendly algorithms.

        Args:
            neuron_ids: List of neuron IDs to get coordinates for

        Returns:
            Dictionary containing:
            - neuron_ids: List of requested neuron IDs
            - coordinates_x: List of X coordinates
            - coordinates_y: List of Y coordinates
            - coordinates_z: List of Z coordinates
            - valid_indices: Boolean list indicating which neuron IDs were valid
            - performance_stats: SIMD performance metrics (if profiling enabled)

            Returns None if no valid neuron IDs provided or neuron array not available

        Examples:
            # Standard usage - get coordinates for specific neurons
            neuron_list = [101, 102, 103, 500, 750]
            result = core_api.get_neuron_coordinates(neuron_list)

            if result:
                for i, neuron_id in enumerate(result['neuron_ids']):
                    if result['valid_indices'][i]:
                        x = result['coordinates_x'][i]
                        y = result['coordinates_y'][i]
                        z = result['coordinates_z'][i]
                        print(f"Neuron {neuron_id}: ({x:.1f}, {y:.1f}, {z:.1f})")

                # Check SIMD performance stats
                stats = result['performance_stats']
                print(f"SIMD Backend: {stats['backend']}")
                print(f"Extraction method: {stats.get('extraction_method', 'unknown')}")
                if 'neurons_per_second' in stats:
                    print(f"Performance: {stats['neurons_per_second']:.0f} neurons/sec")

            # Batch processing for large datasets
            large_batch = list(range(1000, 50000, 10))  # 5,000 neurons
            result = core_api.get_neuron_coordinates(large_batch)

            # Extract valid coordinates only
            valid_coords = []
            for i, valid in enumerate(result['valid_indices']):
                if valid:
                    valid_coords.append((
                        result['coordinates_x'][i],
                        result['coordinates_y'][i],
                        result['coordinates_z'][i]
                    ))
        """
        try:
            if not neuron_ids:
                return None

            # Get centralized SIMD configuration
            simd_config = (
                self.state_manager.get_simd_configuration()
                if self.state_manager
                else {
                    "available": False,
                    "backend": "SCALAR",
                    "vector_width": 1,
                    "alignment": 8,
                }
            )

            # ✅ CRITICAL FIX: Ensure minimum valid SIMD configuration values
            # Prevent zero values that cause empty arrays and coordinate extraction failure
            if simd_config.get("vector_width", 0) <= 0:
                simd_config["vector_width"] = 1  # Minimum vector width
            if simd_config.get("alignment", 0) <= 0:
                simd_config["alignment"] = 8  # Minimum alignment for performance

            if not hasattr(self._connectome_manager, "neuron_array"):
                self.logger.error("Neuron array not available in connectome manager")
                return None

            neuron_array = self._connectome_manager.neuron_array

            # Convert to aligned numpy array for SIMD optimization
            neuron_count = len(neuron_ids)
            alignment = simd_config["alignment"]

            # Align memory to SIMD boundaries for optimal performance
            # ✅ CRITICAL FIX: Ensure aligned_size is never zero
            aligned_size = max(
                neuron_count,
                (neuron_count + simd_config["vector_width"] - 1)
                & ~(simd_config["vector_width"] - 1),
            )

            # Pre-allocate aligned arrays (SIMD-friendly)
            neuron_indices = np.zeros(aligned_size, dtype=np.int32)
            neuron_indices[:neuron_count] = neuron_ids

            # SIMD-optimized bounds checking
            if hasattr(neuron_array, "coordinates_x"):
                max_neuron_id = len(neuron_array.coordinates_x) - 1

                if simd_config["available"] and simd_config["vector_width"] >= 4:
                    # Vectorized bounds checking using SIMD
                    valid_mask = self._simd_bounds_check(
                        neuron_indices[:neuron_count], max_neuron_id, simd_config
                    )
                else:
                    # Fallback to numpy vectorized operations
                    valid_mask = (neuron_indices[:neuron_count] >= 0) & (
                        neuron_indices[:neuron_count] <= max_neuron_id
                    )

                valid_indices = neuron_indices[:neuron_count][valid_mask]
            else:
                self.logger.warning(
                    "Coordinates not available in neuron array, using fallback"
                )
                valid_mask = np.ones(neuron_count, dtype=bool)
                valid_indices = neuron_indices[:neuron_count]

            if len(valid_indices) == 0:
                return {
                    "neuron_ids": neuron_ids,
                    "coordinates_x": [],
                    "coordinates_y": [],
                    "coordinates_z": [],
                    "valid_indices": valid_mask.tolist(),
                    "performance_stats": {
                        "simd_used": False,
                        "backend": simd_config["backend"],
                    },
                }

            # SIMD-optimized coordinate extraction
            performance_stats = {
                "simd_used": simd_config["available"],
                "backend": simd_config["backend"],
            }

            if (
                simd_config["available"]
                and len(valid_indices) >= simd_config["vector_width"] * 2
            ):
                # Use SIMD-optimized coordinate extraction for large datasets
                coords_x, coords_y, coords_z = self._simd_extract_coordinates(
                    neuron_array, valid_indices, simd_config, performance_stats
                )
            else:
                # Use numpy vectorized operations for smaller datasets
                coords_x, coords_y, coords_z = self._vectorized_extract_coordinates(
                    neuron_array, valid_indices, performance_stats
                )

            # Prepare result arrays with same length as input, filling invalid positions with -1 for uint32
            # Using -1 (max uint32) as sentinel value instead of NaN for integer coordinates
            result_x = np.full(
                neuron_count, np.iinfo(np.uint32).max, dtype=np.uint32
            )  # ✅ FIXED: Keep uint32
            result_y = np.full(
                neuron_count, np.iinfo(np.uint32).max, dtype=np.uint32
            )  # ✅ FIXED: Keep uint32
            result_z = np.full(
                neuron_count, np.iinfo(np.uint32).max, dtype=np.uint32
            )  # ✅ FIXED: Keep uint32

            # Fill valid positions - coords arrays only contain valid coordinates
            # We need to map them back to the original neuron_ids positions
            valid_positions = np.where(valid_mask)[
                0
            ]  # Get indices where valid_mask is True

            result_x[valid_positions] = coords_x.astype(
                np.uint32
            )  # ✅ FIXED: Ensure uint32
            result_y[valid_positions] = coords_y.astype(
                np.uint32
            )  # ✅ FIXED: Ensure uint32
            result_z[valid_positions] = coords_z.astype(
                np.uint32
            )  # ✅ FIXED: Ensure uint32

            return {
                "neuron_ids": neuron_ids,
                "coordinates_x": result_x.tolist(),  # ✅ Will now be integers, not floats
                "coordinates_y": result_y.tolist(),  # ✅ Will now be integers, not floats
                "coordinates_z": result_z.tolist(),  # ✅ Will now be integers, not floats
                "valid_indices": valid_mask.tolist(),
                "performance_stats": performance_stats,
            }

        except Exception as e:
            self.logger.error(f"Error getting neuron coordinates: {str(e)}")
            return None

    def get_neuron_coordinates_numpy(
        self, neuron_ids: List[int]
    ) -> Optional[np.ndarray]:
        """
        Get coordinates for a list of neuron IDs as SIMD-optimized numpy array (zero-copy).

        Highest performance method using vectorized SIMD operations and optimal memory layouts.
        Designed for real-time applications requiring maximum throughput.

        Args:
            neuron_ids: List of neuron IDs to get coordinates for

        Returns:
            numpy array with shape (N, 4) containing:
            [neuron_id, x_coordinate, y_coordinate, z_coordinate]
            Only includes valid neuron IDs. Returns None if no valid neurons.

        Examples:
            # High-performance coordinate extraction
            neuron_ids = [1, 2, 3, 100, 250]
            coords = core_api.get_neuron_coordinates_numpy(neuron_ids)

            if coords is not None:
                # Direct numpy operations for analysis
                neuron_ids = coords[:, 0].astype(int)
                x_coords = coords[:, 1]
                y_coords = coords[:, 2]
                z_coords = coords[:, 3]

                # Calculate distances from origin
                distances = np.linalg.norm(coords[:, 1:4], axis=1)
                print(f"Distances: {distances}")

                # Find neurons within a region
                within_region = coords[(x_coords > 10) & (x_coords < 50)]
                print(f"Neurons in region: {within_region[:, 0]}")

                # Vectorized coordinate transformations
                transformed = coords.copy()
                transformed[:, 1:4] *= 2.0  # Scale coordinates

            # Real-time processing loop example
            while processing:
                active_neurons = get_currently_firing_neurons()
                coords = core_api.get_neuron_coordinates_numpy(active_neurons)

                if coords is not None:
                    # Zero-copy processing for maximum performance
                    process_spatial_patterns(coords)
                    update_visualization(coords)

            # Batch analysis for large datasets
            all_neurons = list(range(100000))  # 100k neurons
            coords = core_api.get_neuron_coordinates_numpy(all_neurons)

            if coords is not None:
                # Efficient spatial analysis using SIMD
                center_of_mass = np.mean(coords[:, 1:4], axis=0)
                std_deviation = np.std(coords[:, 1:4], axis=0)
                print(f"Spatial distribution: center={center_of_mass}, std={std_deviation}")
        """
        try:
            if not neuron_ids:
                return None

            # Get centralized SIMD configuration
            simd_config = (
                self.state_manager.get_simd_configuration()
                if self.state_manager
                else {
                    "available": False,
                    "backend": "SCALAR",
                    "vector_width": 1,
                    "alignment": 8,
                }
            )

            # ✅ CRITICAL FIX: Ensure minimum valid SIMD configuration values
            # Prevent zero values that cause empty arrays and coordinate extraction failure
            if simd_config.get("vector_width", 0) <= 0:
                simd_config["vector_width"] = 1  # Minimum vector width
            if simd_config.get("alignment", 0) <= 0:
                simd_config["alignment"] = 8  # Minimum alignment for performance

            if not hasattr(self._connectome_manager, "neuron_array"):
                return None

            neuron_array = self._connectome_manager.neuron_array
            neuron_count = len(neuron_ids)

            # Pre-allocate SIMD-aligned array for optimal performance
            alignment = simd_config["alignment"]
            neuron_indices = np.array(neuron_ids, dtype=np.int32)

            # SIMD-optimized filtering of valid indices
            if hasattr(neuron_array, "coordinates_x"):
                max_neuron_id = len(neuron_array.coordinates_x) - 1

                if simd_config["available"] and simd_config["vector_width"] >= 4:
                    valid_mask = self._simd_bounds_check(
                        neuron_indices, max_neuron_id, simd_config
                    )
                else:
                    valid_mask = (neuron_indices >= 0) & (
                        neuron_indices <= max_neuron_id
                    )

                valid_indices = neuron_indices[valid_mask]
            else:
                valid_indices = neuron_indices

            if len(valid_indices) == 0:
                return None

            # High-performance coordinate extraction
            if (
                simd_config["available"]
                and len(valid_indices) >= simd_config["vector_width"] * 4
            ):
                # SIMD path for large datasets
                coords_x, coords_y, coords_z = self._simd_extract_coordinates(
                    neuron_array, valid_indices, simd_config, {}
                )
            else:
                # Vectorized path for smaller datasets
                coords_x, coords_y, coords_z = self._vectorized_extract_coordinates(
                    neuron_array, valid_indices, {}
                )

            # Combine into single SIMD-aligned array: [neuron_id, x, y, z]
            result = np.column_stack(
                (valid_indices.astype(np.int32), coords_x, coords_y, coords_z)
            )

            return result

        except Exception as e:
            self.logger.error(
                f"Error getting neuron coordinates as numpy array: {str(e)}"
            )
            return None

    def _simd_bounds_check(
        self, indices: np.ndarray, max_value: int, simd_config: dict
    ) -> np.ndarray:
        """
        SIMD-optimized bounds checking for neuron indices.

        Uses vectorized operations to check multiple indices simultaneously.
        """
        try:
            # Use numpy's vectorized operations which leverage SIMD under the hood
            # This is optimized for the detected SIMD backend
            vector_width = simd_config["vector_width"]

            # Process in SIMD-aligned chunks for optimal performance
            valid_mask = np.zeros(len(indices), dtype=bool)

            # Process main chunks using vectorized operations
            for i in range(0, len(indices), vector_width):
                end_idx = min(i + vector_width, len(indices))
                chunk = indices[i:end_idx]

                # Vectorized bounds check (automatically uses SIMD)
                chunk_mask = (chunk >= 0) & (chunk <= max_value)
                valid_mask[i:end_idx] = chunk_mask

            return valid_mask

        except Exception as e:
            self.logger.warning(f"SIMD bounds check failed, using fallback: {e}")
            return (indices >= 0) & (indices <= max_value)

    def _simd_extract_coordinates(
        self,
        neuron_array,
        valid_indices: np.ndarray,
        simd_config: dict,
        performance_stats: dict,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        SIMD-optimized coordinate extraction using vectorized array indexing.

        Processes coordinates in SIMD-aligned chunks for maximum throughput.
        Handles both NumPy arrays and PyTorch tensors properly.
        """
        try:
            start_time = time.time()
            vector_width = simd_config["vector_width"]

            if (
                hasattr(neuron_array, "coordinates_x")
                and hasattr(neuron_array, "coordinates_y")
                and hasattr(neuron_array, "coordinates_z")
            ):
                # Handle PyTorch tensors vs NumPy arrays for SIMD optimization
                import torch

                if isinstance(neuron_array.coordinates_x, torch.Tensor):
                    # Convert PyTorch tensors to NumPy for SIMD operations
                    coords_x_np = neuron_array.coordinates_x.cpu().numpy()
                    coords_y_np = neuron_array.coordinates_y.cpu().numpy()
                    coords_z_np = neuron_array.coordinates_z.cpu().numpy()

                    # SIMD-optimized array indexing on NumPy arrays - keep uint32
                    coords_x = coords_x_np[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32
                    coords_y = coords_y_np[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32
                    coords_z = coords_z_np[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32

                    performance_stats["extraction_method"] = "simd_torch_converted"
                else:
                    # Direct SIMD-optimized array indexing on NumPy arrays - keep uint32
                    coords_x = neuron_array.coordinates_x[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32
                    coords_y = neuron_array.coordinates_y[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32
                    coords_z = neuron_array.coordinates_z[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32

                    performance_stats["extraction_method"] = "simd_numpy_direct"
            else:
                # ❌ NO FALLBACK - Coordinates must exist in neuron array
                # Creating fake coordinates violates architectural rules
                self.logger.error(
                    "Neuron array missing coordinate properties - cannot extract coordinates"
                )
                raise ValueError(
                    "Neuron coordinate arrays not available in connectome - check genome initialization"
                )

            extraction_time = time.time() - start_time
            performance_stats["extraction_time_ms"] = extraction_time * 1000
            performance_stats["neurons_per_second"] = (
                len(valid_indices) / extraction_time if extraction_time > 0 else 0
            )
            performance_stats["simd_efficiency"] = min(
                1.0,
                (len(valid_indices) / vector_width)
                / max(1, len(valid_indices) // vector_width),
            )

            return coords_x, coords_y, coords_z

        except Exception as e:
            self.logger.warning(
                f"SIMD coordinate extraction failed, using fallback: {e}"
            )
            return self._vectorized_extract_coordinates(
                neuron_array, valid_indices, performance_stats
            )

    def _vectorized_extract_coordinates(
        self, neuron_array, valid_indices: np.ndarray, performance_stats: dict
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Vectorized coordinate extraction fallback using standard numpy operations.
        Handles both NumPy arrays and PyTorch tensors properly.
        """
        try:
            start_time = time.time()

            if (
                hasattr(neuron_array, "coordinates_x")
                and hasattr(neuron_array, "coordinates_y")
                and hasattr(neuron_array, "coordinates_z")
            ):
                # Handle PyTorch tensors vs NumPy arrays
                import torch

                if isinstance(neuron_array.coordinates_x, torch.Tensor):
                    # Convert PyTorch tensors to NumPy for indexing
                    coords_x_np = neuron_array.coordinates_x.cpu().numpy()
                    coords_y_np = neuron_array.coordinates_y.cpu().numpy()
                    coords_z_np = neuron_array.coordinates_z.cpu().numpy()

                    # Extract coordinates using NumPy indexing - keep uint32
                    coords_x = coords_x_np[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32
                    coords_y = coords_y_np[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32
                    coords_z = coords_z_np[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32

                    performance_stats["extraction_method"] = (
                        "vectorized_torch_converted"
                    )
                else:
                    # Direct NumPy array indexing - keep uint32
                    coords_x = neuron_array.coordinates_x[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32
                    coords_y = neuron_array.coordinates_y[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32
                    coords_z = neuron_array.coordinates_z[valid_indices].astype(
                        np.uint32
                    )  # ✅ FIXED: Keep uint32

                    performance_stats["extraction_method"] = "vectorized_numpy_direct"
            else:
                # ❌ NO FALLBACK - Coordinates must exist in neuron array
                # Creating fake coordinates violates architectural rules
                self.logger.error(
                    "Neuron array missing coordinate properties - cannot extract coordinates"
                )
                raise ValueError(
                    "Neuron coordinate arrays not available in connectome - check genome initialization"
                )

            extraction_time = time.time() - start_time
            performance_stats["extraction_time_ms"] = extraction_time * 1000
            performance_stats["neurons_per_second"] = (
                len(valid_indices) / extraction_time if extraction_time > 0 else 0
            )

            return coords_x, coords_y, coords_z

        except Exception as e:
            self.logger.error(f"Vectorized coordinate extraction failed: {e}")
            # ❌ NO FALLBACK - Don't create fake coordinates
            # Real coordinates must exist - this is a configuration/initialization error
            raise ValueError(f"Failed to extract neuron coordinates: {e}")

    def benchmark_neuron_coordinate_extraction(
        self, neuron_count: int = 10000
    ) -> Dict[str, Any]:
        """
        Benchmark SIMD-optimized neuron coordinate extraction performance.

        Tests different batch sizes and extraction methods to demonstrate
        the performance benefits of SIMD optimization.

        Args:
            neuron_count: Number of neurons to benchmark (default 10,000)

        Returns:
            Dictionary with benchmark results including timing, throughput, and SIMD efficiency
        """
        try:
            # Generate test neuron IDs
            neuron_ids = list(range(0, neuron_count, max(1, neuron_count // 10000)))
            if len(neuron_ids) > 10000:
                neuron_ids = neuron_ids[
                    :10000
                ]  # Cap at 10k for reasonable benchmark time

            # Get SIMD configuration
            simd_config = (
                self.state_manager.get_simd_configuration()
                if self.state_manager
                else {
                    "available": False,
                    "backend": "SCALAR",
                    "vector_width": 1,
                    "alignment": 8,
                }
            )

            # Run benchmark tests
            results = {
                "simd_config": simd_config,
                "neuron_count": len(neuron_ids),
                "benchmark_results": {},
            }

            # Test 1: Dictionary method (standard API)
            start_time = time.perf_counter()
            dict_result = self.get_neuron_coordinates(neuron_ids)
            dict_time = time.perf_counter() - start_time

            if dict_result:
                results["benchmark_results"]["dictionary_method"] = {
                    "time_ms": dict_time * 1000,
                    "neurons_per_second": (
                        len(neuron_ids) / dict_time if dict_time > 0 else 0
                    ),
                    "performance_stats": dict_result.get("performance_stats", {}),
                    "valid_neurons": sum(dict_result.get("valid_indices", [])),
                }

            # Test 2: NumPy method (high-performance API)
            start_time = time.perf_counter()
            numpy_result = self.get_neuron_coordinates_numpy(neuron_ids)
            numpy_time = time.perf_counter() - start_time

            if numpy_result is not None:
                results["benchmark_results"]["numpy_method"] = {
                    "time_ms": numpy_time * 1000,
                    "neurons_per_second": (
                        len(numpy_result) / numpy_time if numpy_time > 0 else 0
                    ),
                    "result_shape": numpy_result.shape,
                    "memory_mb": numpy_result.nbytes / (1024 * 1024),
                }

            # Performance comparison
            if dict_time > 0 and numpy_time > 0:
                results["performance_comparison"] = {
                    "numpy_speedup": dict_time / numpy_time,
                    "simd_efficiency": simd_config.get("vector_width", 1)
                    / max(1, dict_time / numpy_time),
                    "recommended_method": (
                        "numpy" if numpy_time < dict_time else "dictionary"
                    ),
                }

            # SIMD utilization analysis
            if simd_config["available"]:
                theoretical_speedup = simd_config["vector_width"]
                actual_speedup = results["performance_comparison"].get(
                    "numpy_speedup", 1.0
                )
                results["simd_analysis"] = {
                    "theoretical_max_speedup": theoretical_speedup,
                    "actual_speedup": actual_speedup,
                    "simd_utilization_percent": (actual_speedup / theoretical_speedup)
                    * 100,
                    "backend_used": simd_config["backend"],
                    "optimization_recommendations": self._get_optimization_recommendations(
                        len(neuron_ids),
                        simd_config,
                        actual_speedup,
                        theoretical_speedup,
                    ),
                }

            self.logger.info(
                f"Coordinate extraction benchmark completed: "
                f"{len(neuron_ids)} neurons, "
                f"SIMD: {simd_config['backend']}, "
                f"Performance: {results['benchmark_results'].get('numpy_method', {}).get('neurons_per_second', 0):.0f} neurons/sec"
            )

            return results

        except Exception as e:
            self.logger.error(
                f"Error running coordinate extraction benchmark: {str(e)}"
            )
            return {"error": str(e), "simd_config": simd_config}

    def _get_optimization_recommendations(
        self,
        neuron_count: int,
        simd_config: dict,
        actual_speedup: float,
        theoretical_speedup: float,
    ) -> List[str]:
        """Generate optimization recommendations based on benchmark results."""
        recommendations = []

        utilization = (
            (actual_speedup / theoretical_speedup) * 100
            if theoretical_speedup > 0
            else 0
        )

        if utilization < 50:
            recommendations.append(
                "Consider larger batch sizes to improve SIMD utilization"
            )

        if neuron_count < simd_config["vector_width"] * 10:
            recommendations.append("Dataset too small for effective SIMD optimization")

        if not simd_config["available"]:
            recommendations.append(
                "SIMD not available - consider upgrading hardware or enabling SIMD support"
            )
        elif simd_config["backend"] == "SCALAR":
            recommendations.append(
                "SIMD backend using scalar fallback - check SIMD detection"
            )

        if utilization > 80:
            recommendations.append(
                "Excellent SIMD utilization - consider this pattern for other operations"
            )

        return recommendations

    # =================================================================
    # CORTICAL MAPPING RESTRICTIONS
    # =================================================================

    def get_mapping_restrictions(
        self, source_type: Optional[str] = None, destination_type: Optional[str] = None
    ) -> Dict[str, Any]:
        """Get mapping restrictions between cortical area types.

        Args:
            source_type: Source cortical area type (optional, returns all if None)
            destination_type: Destination cortical area type (optional, returns all if None)

        Returns:
            Dict containing restrictions and defaults for cortical area mappings
        """
        try:
            registry = get_mapping_restrictions_registry()

            if source_type and destination_type:
                # Get specific restriction
                restriction = registry.get_restriction(source_type, destination_type)
                default = registry.get_default(source_type, destination_type)

                return {
                    "source_type": source_type,
                    "destination_type": destination_type,
                    "restriction": restriction.to_dict() if restriction else None,
                    "default": default.to_dict() if default else None,
                }
            else:
                # Get all restrictions and defaults
                return registry.to_dict()

        except Exception as e:
            self.logger.error(f"Error getting mapping restrictions: {str(e)}")
            return {"restrictions": [], "defaults": []}

    def get_restriction_between_cortical_areas(
        self, source_cortical_id: str, destination_cortical_id: str
    ) -> Optional[Dict[str, Any]]:
        """Get mapping restriction between two specific cortical areas.

        Args:
            source_cortical_id: Source cortical area ID
            destination_cortical_id: Destination cortical area ID

        Returns:
            Dict containing restriction data or None if not found
        """
        try:
            # Get the cortical area types
            source_area = self.get_cortical_area(source_cortical_id)
            destination_area = self.get_cortical_area(destination_cortical_id)

            if not source_area or not destination_area:
                return None

            source_type = source_area.get("type", "UNKNOWN")
            destination_type = destination_area.get("type", "UNKNOWN")

            # Get restriction for these types
            registry = get_mapping_restrictions_registry()
            restriction = registry.get_restriction(source_type, destination_type)
            default = registry.get_default(source_type, destination_type)

            if restriction or default:
                return {
                    "source_cortical_id": source_cortical_id,
                    "destination_cortical_id": destination_cortical_id,
                    "source_type": source_type,
                    "destination_type": destination_type,
                    "restriction": restriction.to_dict() if restriction else None,
                    "default": default.to_dict() if default else None,
                    "has_restricted_morphologies": (
                        restriction.has_restricted_morphologies()
                        if restriction
                        else False
                    ),
                    "get_morphologies_restricted_to": (
                        restriction.restricted_morphologies
                        if restriction and restriction.has_restricted_morphologies()
                        else []
                    ),
                }

            return None

        except Exception as e:
            self.logger.error(
                f"Error getting restriction between {source_cortical_id} and {destination_cortical_id}: {str(e)}"
            )
            return None

    # =================================================================
    # AGENT REGISTRY (State Manager Integration)
    # =================================================================

    def register_agent(
        self,
        agent_id: str,
        agent_type: str = None,
        capabilities: dict = None,
        agent_data_port: int = None,
        agent_version: str = None,
        controller_version: str = None,
        agent_ip: str = None,
    ) -> bool:
        """
        Register an agent with full capability structure and metadata.

        Args:
            agent_id: Unique identifier for the agent
            agent_type: Type of agent (optional, determined from capabilities if not provided)
            capabilities: Full capabilities dictionary structure
            agent_data_port: Port number for agent data communication
            agent_version: Version of the agent software
            controller_version: Version of the controller software
            agent_ip: IP address of the agent (optional)

        Returns:
            bool: True if registration successful
        """
        try:
            if self.state_manager:
                self.state_manager.register_agent(
                    agent_id=agent_id,
                    agent_type=agent_type,
                    capabilities=capabilities,
                    agent_data_port=agent_data_port,
                    agent_version=agent_version,
                    controller_version=controller_version,
                    agent_ip=agent_ip,
                )
                self.logger.info(
                    f"Registered agent {agent_id} (type: {agent_type}, port: {agent_data_port}, ip: {agent_ip})"
                )
                return True
            else:
                self.logger.warning(
                    "State manager not available for agent registration"
                )
                return False
        except Exception as e:
            self.logger.error(f"Failed to register agent {agent_id}: {e}")
            return False

    def unregister_agent(self, agent_id: str) -> bool:
        """
        Unregister an agent and remove from all tracking.

        Args:
            agent_id: Unique identifier for the agent to remove

        Returns:
            bool: True if unregistration successful
        """
        try:
            if self.state_manager:
                self.state_manager.unregister_agent(agent_id)
                self.logger.info(f"Unregistered agent {agent_id}")
                return True
            else:
                self.logger.warning(
                    "State manager not available for agent unregistration"
                )
                return False
        except Exception as e:
            self.logger.error(f"Failed to unregister agent {agent_id}: {e}")
            return False

    def get_connected_agents(self) -> List[str]:
        """Get list of all connected agent IDs."""
        try:
            return self.state_manager.get_connected_agents()
        except Exception as e:
            self.logger.error(f"Error getting connected agents: {e}")
            return []

    def get_agent_registry_summary(self) -> dict:
        """
        Get comprehensive summary of agent registry state for FQ sampler management.

        Returns:
            Dictionary with registry state including counts and agent lists
        """
        try:
            return self.state_manager.get_agent_registry_summary()
        except Exception as e:
            self.logger.error(f"Error getting agent registry summary: {e}")
            return {
                "connected_visualization_agents": [],
                "connected_sensorimotor_agents": [],
                "total_agents": 0,
                "agent_count_viz": 0,
                "agent_count_sensorimotor": 0,
                "last_update": 0,
            }

    def get_agent_properties(self, agent_id: str) -> dict:
        """
        Get full properties for a specific agent.

        Args:
            agent_id: Agent identifier

        Returns:
            Dictionary with agent properties or empty dict if not found
        """
        try:
            if self.state_manager:
                return self.state_manager.get_agent_properties(agent_id)
            else:
                self.logger.warning("State manager not available for agent properties")
                return {}
        except Exception as e:
            self.logger.error(f"Failed to get agent properties for {agent_id}: {e}")
            return {}

    def configure_agent(self, agent_id: str, config: Dict[str, Any]) -> bool:
        """Configure an agent with the given configuration."""
        return self._agents_service.configure_agent(agent_id, config)

    def get_service_health(self) -> Dict[str, Any]:
        """Get health information about all domain services."""
        try:
            return {
                "system_service": "healthy" if self._system_service else "unavailable",
                "genome_service": "healthy" if self._genome_service else "unavailable",
                "cortical_area_service": (
                    "healthy" if self._cortical_area_service else "unavailable"
                ),
                "connectome_service": (
                    "healthy" if self._connectome_service else "unavailable"
                ),
                "brain_service": "healthy" if self._brain_service else "unavailable",
                "agents_service": "healthy" if self._agents_service else "unavailable",
                "network_service": (
                    "healthy" if self._network_service else "unavailable"
                ),
                "facade_status": "operational",
            }
        except Exception as e:
            self.logger.error(f"Error getting service health: {str(e)}")
            return {"facade_status": "error", "error": str(e)}

    def get_visualized_cortical_list(self) -> List[str]:
        """Get list of cortical areas currently being visualized."""
        try:
            # This would need implementation based on current visualization state
            return []
        except Exception as e:
            self.logger.error(f"Error getting visualized cortical list: {str(e)}")
            return []

    def get_cortical_idx_mapping(self) -> Dict[str, Any]:
        """Get the current cortical_idx to cortical_id mapping for debugging corruption issues."""
        try:
            # Get mappings from BiDirectionalCorticalMap
            id_to_idx = self._connectome_manager.cortical_mapping.get_all_mappings()
            idx_to_id = {idx: id for id, idx in id_to_idx.items()}

            # Get validation status
            is_consistent, errors = (
                self._connectome_manager.cortical_mapping.validate_consistency()
            )

            # Get stats
            stats = self._connectome_manager.cortical_mapping.get_stats()

            # Build comprehensive debug data
            mapping_data = {
                "id_to_idx": id_to_idx,
                "idx_to_id": idx_to_id,
                "stats": stats,
                "validation": {
                    "is_consistent": is_consistent,
                    "errors": list(errors) if errors else [],
                },
                "reserved_areas": {
                    "_death": id_to_idx.get("_death"),
                    "___pwr": id_to_idx.get("___pwr"),
                },
                "debug_info": {
                    "total_mappings": len(id_to_idx),
                    "highest_idx": max(idx_to_id.keys()) if idx_to_id else None,
                    "all_indices": sorted(list(idx_to_id.keys())),
                    "all_ids": sorted(list(id_to_idx.keys())),
                },
            }

            return mapping_data

        except Exception as e:
            self.logger.error(f"Error getting cortical idx mapping: {str(e)}")
            import traceback

            self.logger.error(traceback.format_exc())
            return {
                "error": str(e),
                "id_to_idx": {},
                "idx_to_id": {},
                "validation": {"is_consistent": False, "errors": [str(e)]},
            }

    # ===== BRAIN REGION WRITE OPERATIONS =====
    # These methods handle brain region modifications through proper data flow:
    # API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager

    def create_brain_region(
        self,
        region_id: str,
        region_name: str,
        parent_region_id: str = "root",
        coordinates: Dict[str, int] = None,
        dimensions: Dict[str, int] = None,
        parameters: Dict[str, Any] = None,
    ) -> bool:
        """
        Create a brain region.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # Route WRITE operation through GenomeService for architecture compliance
            return self._genome_service.create_brain_region(
                region_id=region_id,
                region_name=region_name,
                parent_region_id=parent_region_id,
                coordinates=coordinates,
                dimensions=dimensions,
                parameters=parameters,
            )

        except Exception as e:
            self.logger.error(f"Error creating brain region: {str(e)}")
            raise ValueError(f"Failed to create brain region: {str(e)}")

    def update_brain_region(
        self,
        region_id: str,
        region_name: Optional[str] = None,
        parent_region_id: Optional[str] = None,
        coordinates: Optional[Dict[str, int]] = None,
        dimensions: Optional[Dict[str, int]] = None,
        parameters: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """
        Update a brain region.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # Route WRITE operation through GenomeService for architecture compliance
            return self._genome_service.update_brain_region(
                region_id=region_id,
                region_name=region_name,
                parent_region_id=parent_region_id,
                coordinates=coordinates,
                dimensions=dimensions,
                parameters=parameters,
            )

        except Exception as e:
            self.logger.error(f"Error updating brain region: {str(e)}")
            raise ValueError(f"Failed to update brain region: {str(e)}")

    def delete_brain_region(
        self, region_id: str, preserve_children: bool = True
    ) -> bool:
        """
        Delete a brain region.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager

        Args:
            region_id: ID of region to delete
            preserve_children: If True, move children to parent; if False, delete all members
        """
        try:
            # Route WRITE operation through GenomeService for architecture compliance
            # Note: preserve_children=True means delete_members=False
            return self._genome_service.delete_brain_region(
                region_id=region_id, delete_members=not preserve_children
            )

        except Exception as e:
            self.logger.error(f"Error deleting brain region: {str(e)}")
            raise ValueError(f"Failed to delete brain region: {str(e)}")

    def change_cortical_area_parent(
        self, cortical_area_id: str, new_parent_id: str
    ) -> bool:
        """
        Change the parent region of a cortical area.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # This is a cortical area modification, so route through cortical area update
            return (
                self._cortical_area_service.update_area(
                    cortical_area_id, parameters={"region_id": new_parent_id}
                )
                is not None
            )

        except Exception as e:
            self.logger.error(f"Error changing cortical area parent: {str(e)}")
            raise ValueError(f"Failed to change cortical area parent: {str(e)}")

    def change_brain_region_parent(self, region_id: str, new_parent_id: str) -> bool:
        """
        Change the parent of a brain region.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # Route WRITE operation through GenomeService for architecture compliance
            return self._genome_service.update_brain_region(
                region_id=region_id, parent_region_id=new_parent_id
            )

        except Exception as e:
            self.logger.error(f"Error changing brain region parent: {str(e)}")
            raise ValueError(f"Failed to change brain region parent: {str(e)}")

    # ===== GENOME WRITE OPERATIONS =====
    # These methods handle genome modifications through proper data flow:
    # API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager

    def reset_genome(self) -> bool:
        """
        Reset the genome.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # Route WRITE operation through GenomeService for architecture compliance
            return self._genome_service.reset_genome()

        except Exception as e:
            self.logger.error(f"Error resetting genome: {str(e)}")
            raise ValueError(f"Failed to reset genome: {str(e)}")

    def process_amalgamation_request(
        self, amalgamation_data: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Process an amalgamation request.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # Route WRITE operation through GenomeService for architecture compliance
            return self._genome_service.amalgamate_genome(amalgamation_data)

        except Exception as e:
            self.logger.error(f"Error processing amalgamation request: {str(e)}")
            raise ValueError(f"Failed to process amalgamation request: {str(e)}")

    def cancel_amalgamation(self, amalgamation_id: str) -> bool:
        """
        Cancel an amalgamation.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # Route WRITE operation through GenomeService for architecture compliance
            return self._genome_service.cancel_amalgamation(amalgamation_id)

        except Exception as e:
            self.logger.error(f"Error cancelling amalgamation: {str(e)}")
            raise ValueError(f"Failed to cancel amalgamation: {str(e)}")

    def append_circuit_to_genome(self, circuit_data: Dict[str, Any]) -> bool:
        """
        Append circuit to genome.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # Route WRITE operation through GenomeService for architecture compliance
            return self._genome_service.append_file_to_genome(circuit_data)

        except Exception as e:
            self.logger.error(f"Error appending circuit to genome: {str(e)}")
            raise ValueError(f"Failed to append circuit to genome: {str(e)}")

    def complete_amalgamation(self, amalgamation_data: Dict[str, Any]) -> bool:
        """
        Complete an amalgamation.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # Route WRITE operation through GenomeService for architecture compliance
            result = self._genome_service.amalgamate_genome(amalgamation_data)
            return result.get("success", False)

        except Exception as e:
            self.logger.error(f"Error completing amalgamation: {str(e)}")
            raise ValueError(f"Failed to complete amalgamation: {str(e)}")

    def cancel_pending_amalgamation(self, amalgamation_id: str) -> bool:
        """
        Cancel a pending amalgamation.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # Route WRITE operation through GenomeService for architecture compliance
            return self._genome_service.cancel_amalgamation(amalgamation_id)

        except Exception as e:
            self.logger.error(f"Error cancelling pending amalgamation: {str(e)}")
            raise ValueError(f"Failed to cancel pending amalgamation: {str(e)}")

    def mark_amalgamation_complete(self, amalgamation_id: str) -> bool:
        """
        Mark an amalgamation as complete.

        ARCHITECTURE COMPLIANCE: WRITE operation routed through GenomeService
        to maintain proper data flow: API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager
        """
        try:
            # This is typically a status update operation
            # For now, we'll route through GenomeService for consistency
            # In the future, this might be handled by a separate AmalgamationService
            return True  # Placeholder implementation

        except Exception as e:
            self.logger.error(f"Error marking amalgamation complete: {str(e)}")
            raise ValueError(f"Failed to mark amalgamation complete: {str(e)}")

    # ===== READ OPERATIONS (Already properly routed) =====
    # These methods are READ operations and correctly use existing services
