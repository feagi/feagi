"""Core API service implementation for FEAGI."""

from typing import Dict, Any, List, Optional
import logging

from feagi.core.feagi import FEAGI

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
        
    @property
    def feagi(self) -> FEAGI:
        """Get the FEAGI instance."""
        return self._feagi
        
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
        return self._feagi.get_cortical_areas()
        
    def get_cortical_area(self, area_id: str) -> Dict[str, Any]:
        """
        Get a cortical area by ID.
        
        Args:
            area_id: ID of the cortical area.
            
        Returns:
            Dictionary containing cortical area information.
        """
        return self._feagi.get_cortical_area(area_id)
        
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
        
    def get_simulation_status(self) -> Dict[str, Any]:
        """
        Get the current simulation status.
        
        Returns:
            Dictionary containing the simulation status.
        """
        return self._feagi.get_simulation_status()
        
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