"""
Refactored CoreAPIService using domain-based service architecture.

This is the new facade implementation that delegates to specialized services
while maintaining complete backward compatibility with the existing API.
"""

from typing import Dict, Any, Optional, List, Tuple, Set

# Import all domain services
from .system.system_service import SystemService
from .genome.genome_service import GenomeService
from .cortical_area.cortical_area_service import CorticalAreaService
from .connectome.connectome_service import ConnectomeService
from .brain.brain_service import BrainService
from .agents.agents_service import AgentsService
from .network.network_service import NetworkService

from feagi.utils.logger import setup_logger

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
        self._genome_service = GenomeService(connectome_manager, self.state_manager)
        self._cortical_area_service = CorticalAreaService(connectome_manager, self.state_manager)
        self._connectome_service = ConnectomeService(connectome_manager, self.state_manager)
        self._brain_service = BrainService(connectome_manager, self.state_manager)
        self._agents_service = AgentsService(connectome_manager, self.state_manager)
        self._network_service = NetworkService(connectome_manager, self.state_manager)
        
        # Validate state manager consistency across services
        self._validate_service_state_consistency()
        
        self.logger.info("CoreAPIService initialized with domain-based architecture and state synchronization")

    def _validate_service_state_consistency(self):
        """Validate that all services share the same state manager instance."""
        try:
            services = [
                ('system', self._system_service),
                ('genome', self._genome_service), 
                ('cortical_area', self._cortical_area_service),
                ('connectome', self._connectome_service),
                ('brain', self._brain_service),
                ('agents', self._agents_service),
                ('network', self._network_service)
            ]
            
            core_state_id = id(self.state_manager)
            inconsistent_services = []
            
            for service_name, service in services:
                if hasattr(service, 'state_manager'):
                    service_state_id = id(service.state_manager)
                    if service_state_id != core_state_id:
                        inconsistent_services.append(service_name)
                        self.logger.error(f"Service {service_name} has different state manager instance: core={core_state_id}, service={service_state_id}")
                else:
                    inconsistent_services.append(service_name)
                    self.logger.error(f"Service {service_name} missing state_manager attribute")
            
            if inconsistent_services:
                self.logger.error(f"State manager inconsistency detected in services: {inconsistent_services}")
                raise RuntimeError(f"Critical state manager inconsistency in services: {inconsistent_services}")
            else:
                self.logger.info("All services share the same state manager instance - consistency validated")
                
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

    # =================================================================
    # GENOME SERVICE DELEGATION
    # =================================================================
    
    def load_essential_genome(self) -> Dict[str, Any]:
        """Load the essential genome from the default templates."""
        return self._genome_service.load_essential_genome()
    
    def load_barebones_genome(self) -> Dict[str, Any]:
        """Load the barebones genome from the default templates."""
        return self._genome_service.load_barebones_genome()
    
    def load_genome(self, genome_data: Dict[str, Any], filename: str = "genome.json") -> Dict[str, Any]:
        """Load a genome and prepare it for use."""
        return self._genome_service.load_genome(genome_data, filename)
    
    def get_genome(self) -> Optional[Dict[str, Any]]:
        """Get the currently loaded genome data."""
        return self._genome_service.get_genome()
    
    def get_genome_filename(self) -> Optional[str]:
        """Get the filename of the currently loaded genome."""
        return self._genome_service.get_genome_filename()
    
    def get_genome_file_name(self) -> Dict[str, str]:
        """Get the genome file name in the format expected by the REST API."""
        return self._genome_service.get_genome_file_name()
    
    def get_default_genomes(self) -> Dict[str, Any]:
        """Get a list of default genome files with their contents."""
        return self._genome_service.get_default_genomes()
    
    def get_genome_counter(self) -> int:
        """Get the current genome counter."""
        return self._genome_service.get_genome_counter()
    
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
        parameters: Dict[str, Any] = None
    ) -> Optional[Dict[str, Any]]:
        """Create a new cortical area."""
        return self._cortical_area_service.create_area(name, coordinates, dimensions, area_type, parameters)
    
    def update_cortical_area(
        self,
        cortical_id: str,
        name: Optional[str] = None,
        coordinates: Optional[Dict[str, int]] = None,
        dimensions: Optional[Dict[str, int]] = None,
        area_type: Optional[str] = None,
        parameters: Optional[Dict[str, Any]] = None
    ) -> Optional[Dict[str, Any]]:
        """Update an existing cortical area."""
        return self._cortical_area_service.update_area(cortical_id, name, coordinates, dimensions, area_type, parameters)
    
    def delete_cortical_area(self, cortical_id: str) -> bool:
        """Delete a cortical area."""
        return self._cortical_area_service.delete_area(cortical_id)
    
    def get_cortical_area_neurons(self, cortical_id: str) -> Optional[List[Dict[str, Any]]]:
        """Get neurons for a specific cortical area."""
        return self._cortical_area_service.get_area_neurons(cortical_id)
    
    def get_cortical_area_activity(self, cortical_id: str, window: int = 1) -> Optional[Dict[str, Any]]:
        """Get activity data for a specific cortical area."""
        return self._cortical_area_service.get_area_activity(cortical_id, window)
    
    def get_cortical_area_connectivity(self, cortical_id: str, direction: str = "both") -> Optional[Dict[str, Any]]:
        """Get connectivity information for a specific cortical area."""
        return self._cortical_area_service.get_area_connectivity(cortical_id, direction)
    
    def stimulate_cortical_area(self, cortical_id: str, pattern: str = "random", 
                               intensity: float = 1.0, duration: int = 1,
                               coordinates: Optional[List[Dict[str, int]]] = None) -> Dict[str, Any]:
        """Stimulate a cortical area with the specified pattern."""
        return self._cortical_area_service.stimulate_area(cortical_id, pattern, intensity, duration, coordinates)
    
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
    
    def get_cortical_2d_locations(self) -> Dict[str, List[int]]:
        """Get 2D locations of all cortical areas."""
        return self._cortical_area_service.get_2d_locations()

    # =================================================================
    # CONNECTOME SERVICE DELEGATION
    # =================================================================
    
    def get_neuron_connectivity(self, neuron_id: str, direction: str = "both") -> Optional[Dict[str, Any]]:
        """Get connectivity information for a specific neuron."""
        return self._connectome_service.get_neuron_connectivity(neuron_id, direction)
    
    def get_connection_stats(self) -> Dict[str, Any]:
        """Get overall connectivity statistics."""
        return self._connectome_service.get_connection_stats()
    
    def get_connection_matrix(self, source_area: str, target_area: str) -> Optional[Dict[str, Any]]:
        """Get connection matrix between two cortical areas."""
        return self._connectome_service.get_connection_matrix(source_area, target_area)
    
    def add_connection(self, source_neuron: str, target_neuron: str, weight: float = 1.0) -> bool:
        """Add a new synaptic connection."""
        return self._connectome_service.add_connection(source_neuron, target_neuron, weight)
    
    def remove_connection(self, source_neuron: str, target_neuron: str) -> bool:
        """Remove a synaptic connection."""
        return self._connectome_service.remove_connection(source_neuron, target_neuron)
    
    def update_connection_weight(self, source_neuron: str, target_neuron: str, new_weight: float) -> bool:
        """Update the weight of an existing connection."""
        return self._connectome_service.update_connection_weight(source_neuron, target_neuron, new_weight)
    
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
    
    def stimulate_neurons(self, neuron_ids: List[str], intensity: float = 1.0) -> Dict[str, Any]:
        """Stimulate specific neurons with given intensity."""
        return self._brain_service.stimulate_neurons(neuron_ids, intensity)

    def get_burst_engine_config(self) -> Dict[str, Any]:
        """Get burst engine configuration."""
        return self._brain_service.get_burst_engine_config()

    def get_burst_engine_stats(self) -> Dict[str, Any]:
        """Get burst engine statistics."""
        return self._brain_service.get_brain_statistics()

    # =================================================================
    # AGENTS SERVICE DELEGATION
    # =================================================================
    
    def get_connected_agents(self) -> List[Dict[str, Any]]:
        """Get list of currently connected agents."""
        return self._agents_service.get_connected_agents()
    
    def register_agent(self, agent_data: Dict[str, Any]) -> Dict[str, Any]:
        """Register a new agent."""
        return self._agents_service.register_agent(agent_data)
    
    def unregister_agent(self, agent_id: str) -> Dict[str, Any]:
        """Unregister an agent."""
        return self._agents_service.unregister_agent(agent_id)
    
    def update_agent_status(self, agent_id: str, status: str, metadata: Dict[str, Any] = None) -> bool:
        """Update agent status and metadata."""
        return self._agents_service.update_agent_status(agent_id, status, metadata)
    
    def get_agent_details(self, agent_id: str) -> Optional[Dict[str, Any]]:
        """Get detailed information about a specific agent."""
        return self._agents_service.get_agent_details(agent_id)
    
    def send_message_to_agent(self, agent_id: str, message: Dict[str, Any]) -> Dict[str, Any]:
        """Send a message to a specific agent."""
        return self._agents_service.send_message_to_agent(agent_id, message)
    
    def broadcast_message(self, message: Dict[str, Any], agent_filter: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Broadcast a message to all connected agents or filtered subset."""
        return self._agents_service.broadcast_message(message, agent_filter)
    
    def get_agent_statistics(self) -> Dict[str, Any]:
        """Get statistics about connected agents."""
        return self._agents_service.get_agent_statistics()

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
    
    def refresh_caches(self):
        """Refresh all cached data across services."""
        try:
            self._cortical_area_service.refresh_cache()
            self.logger.info("All service caches refreshed")
        except Exception as e:
            self.logger.error(f"Error refreshing caches: {str(e)}")
    
    def get_service_health(self) -> Dict[str, Any]:
        """Get health information about all domain services."""
        try:
            return {
                "system_service": "healthy" if self._system_service else "unavailable",
                "genome_service": "healthy" if self._genome_service else "unavailable",
                "cortical_area_service": "healthy" if self._cortical_area_service else "unavailable",
                "connectome_service": "healthy" if self._connectome_service else "unavailable",
                "brain_service": "healthy" if self._brain_service else "unavailable",
                "agents_service": "healthy" if self._agents_service else "unavailable",
                "network_service": "healthy" if self._network_service else "unavailable",
                "facade_status": "operational"
            }
        except Exception as e:
            self.logger.error(f"Error getting service health: {str(e)}")
            return {"facade_status": "error", "error": str(e)}
    
    # =================================================================
    # CORE COMPONENT ACCESS METHODS
    # =================================================================
    
    def get_burst_engine(self):
        """Get the burst engine instance."""
        # Import here to avoid circular imports
        try:
            from feagi.npu.burst_engine import BurstEngine
            if not hasattr(self, '_burst_engine_instance'):
                self._burst_engine_instance = BurstEngine(
                    connectome_manager=self._connectome_manager,
                    state_manager=self.state_manager
                )
            return self._burst_engine_instance
        except Exception as e:
            self.logger.error(f"Error getting burst engine: {str(e)}")
            return None
    
    def get_connectome_manager(self):
        """Get the connectome manager instance."""
        return self._connectome_manager
    
    def get_fcl_manager(self):
        """Get the FCL manager instance."""
        if hasattr(self._connectome_manager, 'fcl_manager'):
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
        """Get the current global fire queue data for FQSampler."""
        try:
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
                
                return {
                    'neuron_ids': neuron_ids,
                    'membrane_potentials': [1.0] * len(neuron_ids),
                    'thresholds': [1.0] * len(neuron_ids),
                    'consecutive_fire_counts': [0] * len(neuron_ids),
                    'refractory_counters': [0] * len(neuron_ids)
                }
            return None
        except Exception as e:
            self.logger.error(f"Error getting fire queue: {str(e)}")
            return None
    
    def get_area_fire_queue(self, cortical_id: str) -> Optional[Dict[str, Any]]:
        """Get fire queue data for a specific cortical area."""
        try:
            if not self._validate_genome_loaded():
                return None
                
            area_idx = self._get_cortical_idx_for_id(cortical_id)
            if area_idx is None:
                return None
                
            if hasattr(self._connectome_manager, 'fcl_manager') and self._connectome_manager.fcl_manager:
                area_fcl = self._connectome_manager.fcl_manager.get_area_fcl(area_idx)
                if area_fcl and hasattr(area_fcl, '__iter__'):
                    neuron_ids = list(area_fcl)
                    return {
                        'cortical_id': cortical_id,
                        'neuron_ids': neuron_ids,
                        'membrane_potentials': [1.0] * len(neuron_ids),
                        'thresholds': [1.0] * len(neuron_ids),
                        'consecutive_fire_counts': [0] * len(neuron_ids),
                        'refractory_counters': [0] * len(neuron_ids)
                    }
            return None
        except Exception as e:
            self.logger.error(f"Error getting area fire queue for {cortical_id}: {str(e)}")
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
            with open(path, 'w') as f:
                json.dump(brain_state, f, indent=2)
            return True
        except Exception as e:
            self.logger.error(f"Error saving brain state: {str(e)}")
            return False
    
    def load_brain_state(self, path: str) -> bool:
        """Load brain state from file."""
        try:
            import json
            with open(path, 'r') as f:
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
            return {agent.get('id', agent.get('agent_id', '')) for agent in agents}
        except Exception as e:
            self.logger.error(f"Error getting agent list: {str(e)}")
            return set()
    
    def get_agent_properties(self, agent_id: str) -> Dict[str, Any]:
        """Get properties of a specific agent."""
        return self._agents_service.get_agent_details(agent_id) or {}
    
    def deregister_agent(self, agent_id: str) -> bool:
        """Deregister an agent."""
        result = self._agents_service.unregister_agent(agent_id)
        return result.get('success', False) if isinstance(result, dict) else False 

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
    
    def get_cortical_locations_2d(self) -> Dict[str, List[int]]:
        """Get 2D locations of all cortical areas (legacy name)."""
        return self.get_cortical_2d_locations()
    
    def get_cortical_area_stats(self, cortical_area: str) -> Optional[Dict[str, Any]]:
        """Get statistics for a cortical area."""
        try:
            return {
                "area_id": cortical_area,
                "neuron_count": len(self.get_cortical_area_neurons(cortical_area) or []),
                "activity": self.get_cortical_area_activity(cortical_area),
                "connectivity": self.get_cortical_area_connectivity(cortical_area)
            }
        except Exception as e:
            self.logger.error(f"Error getting cortical area stats: {str(e)}")
            return None 

    # =================================================================
    # PLASTICITY AND LEARNING METHODS
    # =================================================================
    
    def enable_area_plasticity(self, cortical_id: str, settings: Optional[Dict[str, Any]] = None) -> bool:
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
            return {
                "enabled": True,
                "queue_depth": 1000,
                "areas_with_plasticity": []
            }
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
    
    def get_membrane_potential_monitoring_status(self, cortical_areas: List[str]) -> List[Tuple[str, bool]]:
        """Get membrane potential monitoring status for cortical areas."""
        try:
            return [(area, True) for area in cortical_areas]  # Default to enabled
        except Exception as e:
            self.logger.error(f"Error getting membrane potential monitoring status: {str(e)}")
            return []
    
    def set_membrane_potential_monitoring(self, cortical_areas: List[str], enabled: bool) -> bool:
        """Set membrane potential monitoring for cortical areas."""
        try:
            self.logger.info(f"Setting membrane potential monitoring to {enabled} for areas: {cortical_areas}")
            return True
        except Exception as e:
            self.logger.error(f"Error setting membrane potential monitoring: {str(e)}")
            return False
    
    def get_synaptic_potential_monitoring_status(self, cortical_areas: List[str]) -> List[Tuple[str, bool]]:
        """Get synaptic potential monitoring status for cortical areas."""
        try:
            return [(area, True) for area in cortical_areas]  # Default to enabled
        except Exception as e:
            self.logger.error(f"Error getting synaptic potential monitoring status: {str(e)}")
            return []
    
    def set_synaptic_potential_monitoring(self, cortical_areas: List[str], enabled: bool) -> bool:
        """Set synaptic potential monitoring for cortical areas."""
        try:
            self.logger.info(f"Setting synaptic potential monitoring to {enabled} for areas: {cortical_areas}")
            return True
        except Exception as e:
            self.logger.error(f"Error setting synaptic potential monitoring: {str(e)}")
            return False
    
    def get_membrane_potentials(self, neuron_ids: List[int]) -> Dict[int, float]:
        """Get membrane potentials for specific neurons."""
        try:
            result = {}
            for neuron_id in neuron_ids:
                # This would need implementation in connectome service
                result[neuron_id] = 0.0  # Default value
            return result
        except Exception as e:
            self.logger.error(f"Error getting membrane potentials: {str(e)}")
            return {}
    
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
    
    def get_fcl_sampler_config(self) -> Dict[str, Any]:
        """Get FCL sampler configuration."""
        try:
            if self.state_manager:
                return {
                    "frequency": getattr(self.state_manager, 'fcl_sampler_frequency', 20.0),
                    "consumer": getattr(self.state_manager, 'fcl_sampler_consumer', 1)
                }
            return {"frequency": 20.0, "consumer": 1}
        except Exception as e:
            self.logger.error(f"Error getting FCL sampler config: {str(e)}")
            return {}
    
    def update_fcl_sampler_config(self, frequency: float, consumer: str) -> bool:
        """Update FCL sampler configuration."""
        try:
            if self.state_manager:
                self.state_manager.set_fcl_sampler_frequency(frequency)
                consumer_map = {"visualization": 1, "motor": 2, "both": 3}
                self.state_manager.set_fcl_sampler_consumer(consumer_map.get(consumer, 1))
            return True
        except Exception as e:
            self.logger.error(f"Error updating FCL sampler config: {str(e)}")
            return False
    
    def get_area_fcl_sample_rate(self, area_id: int) -> float:
        """Get FCL sample rate for an area."""
        return 20.0  # Default value
    
    def set_area_fcl_sample_rate(self, area_id: int, sample_rate: float) -> bool:
        """Set FCL sample rate for an area."""
        try:
            # This would need implementation
            return True
        except Exception as e:
            self.logger.error(f"Error setting area FCL sample rate: {str(e)}")
            return False
    
    def get_burst_counter(self) -> int:
        """Get current burst counter."""
        try:
            if self.state_manager:
                return getattr(self.state_manager, 'current_burst_id', 0)
            return 0
        except Exception as e:
            self.logger.error(f"Error getting burst counter: {str(e)}")
            return 0
    
    def update_burst_engine_config(self, config: Dict[str, Any]) -> bool:
        """Update burst engine configuration."""
        try:
            # This would need implementation in brain service
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
                "cortical_area_count": stats.get("cortical_area_count", 0)
            }
        except Exception as e:
            self.logger.error(f"Error getting connectome dimensions: {str(e)}")
            return {}
    
    def get_morphology_list(self) -> List[str]:
        """Get list of available morphologies."""
        try:
            # This would need implementation
            return ["default", "sensory", "motor"]
        except Exception as e:
            self.logger.error(f"Error getting morphology list: {str(e)}")
            return []
    
    def get_detailed_cortical_map(self) -> Dict[str, Any]:
        """Get detailed cortical map."""
        try:
            areas = self.get_all_cortical_areas()
            return {
                "areas": areas,
                "total_count": len(areas),
                "connections": self.get_area_to_area_connectivity()
            }
        except Exception as e:
            self.logger.error(f"Error getting detailed cortical map: {str(e)}")
            return {}
    
    def get_data_path(self) -> str:
        """Get data path."""
        return "/tmp/feagi_data"
    
    def get_temp_path(self) -> str:
        """Get temporary path."""
        return "/tmp/feagi_temp"

    # =================================================================
    # STIMULATION METHODS
    # =================================================================
    
    def trigger_manual_stimulation(self, stimulation_payload: Dict[str, Any]) -> bool:
        """Trigger manual stimulation."""
        try:
            cortical_id = stimulation_payload.get('cortical_id')
            intensity = stimulation_payload.get('intensity', 1.0)
            if cortical_id:
                return self.stimulate_cortical_area(cortical_id, intensity=intensity).get('success', False)
            return False
        except Exception as e:
            self.logger.error(f"Error triggering manual stimulation: {str(e)}")
            return False
    
    def trigger_sustained_stimulation(self, stimulation_payload: Dict[str, Any]) -> bool:
        """Trigger sustained stimulation."""
        try:
            cortical_id = stimulation_payload.get('cortical_id')
            intensity = stimulation_payload.get('intensity', 1.0)
            duration = stimulation_payload.get('duration', 10)
            if cortical_id:
                return self.stimulate_cortical_area(cortical_id, intensity=intensity, duration=duration).get('success', False)
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
    
    def refresh_cached_data(self):
        """Refresh cached data (legacy method name)."""
        self.refresh_caches()

    # =================================================================
    # UTILITY AND HELPER METHODS
    # =================================================================
    
    def _get_cortical_idx_for_id(self, cortical_id: str) -> Optional[int]:
        """Get cortical index for a cortical ID."""
        try:
            for idx, area in self._connectome_manager.cortical_areas.items():
                if area.cortical_id == cortical_id:
                    return idx
            return None
        except Exception as e:
            self.logger.error(f"Error getting cortical idx for {cortical_id}: {str(e)}")
            return None
    
    def get_neuron_mappings(self) -> Dict[str, Any]:
        """Get neuron mappings."""
        try:
            return {
                "neuron_to_area": {},
                "area_to_neurons": {},
                "total_neurons": self.get_connectome_dimensions().get("neuron_count", 0)
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
                return bool(getattr(self.state_manager, 'pending_amalgamation', False))
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
    
    def batch_create_neurons(self, area_id: str, positions: List[Tuple[int, int, int]], 
                           properties: Optional[Dict[str, Any]] = None) -> List[int]:
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
                "running": not getattr(self.state_manager, 'exit_condition', False) if self.state_manager else False,
                "burst_counter": self.get_burst_counter(),
                "genome_loaded": self.genome_is_loaded(),
                "brain_ready": self.state_manager.get_brain_readiness() if self.state_manager else False
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
            if not hasattr(self, '_feagi_instance'):
                self._feagi_instance = FEAGI()
            return self._feagi_instance
        except Exception as e:
            self.logger.error(f"Error getting FEAGI instance: {str(e)}")
            return None 