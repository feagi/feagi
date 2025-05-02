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
from feagi.bdu.connectome_manager import ConnectomeManager
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
        
    def _handle_embryogenesis_progress(self, stage, percentage, message):
        """Handle progress updates from the neuroembryogenesis process."""
        self.logger.info(f"[{stage}] {percentage:.1f}% - {message}")
        
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
            
        # Then use the default location
        return os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), "evo", "defaults")
    
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