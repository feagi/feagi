"""
Refactored CoreAPIService using domain-based service architecture.

This is the new facade implementation that delegates to specialized services
while maintaining complete backward compatibility with the existing API.
"""

from typing import Dict, Any, Optional, List, Tuple

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
        
        # Initialize all domain services
        self._system_service = SystemService(connectome_manager, state_manager)
        self._genome_service = GenomeService(connectome_manager, state_manager)
        self._cortical_area_service = CorticalAreaService(connectome_manager, state_manager)
        self._connectome_service = ConnectomeService(connectome_manager, state_manager)
        self._brain_service = BrainService(connectome_manager, state_manager)
        self._agents_service = AgentsService(connectome_manager, state_manager)
        self._network_service = NetworkService(connectome_manager, state_manager)
        
        self.logger.info("CoreAPIService initialized with domain-based architecture")

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
    # LEGACY COMPATIBILITY METHODS
    # =================================================================
    
    # Add any legacy method aliases or compatibility methods here if needed
    # For now, all existing methods are preserved with their exact signatures 

    # Legacy method aliases for backward compatibility
    def get_cortical_areas(self) -> List[Dict[str, Any]]:
        """Get all cortical areas (alias for get_all_cortical_areas)."""
        return self.get_all_cortical_areas() 