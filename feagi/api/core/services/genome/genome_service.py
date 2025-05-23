"""Genome service for managing FEAGI genome operations."""

import os
import json
import tempfile
from typing import Dict, Any, Optional, List
from ..shared.base_service import BaseService


class GenomeService(BaseService):
    """
    Genome service handles genome loading, saving, validation,
    and genome-related operations.
    """
    
    def __init__(self, connectome_manager, state_manager=None):
        """Initialize genome service."""
        super().__init__(connectome_manager, state_manager)
        self._current_genome = None
        self._genome_filename = None
        self._temp_dir = tempfile.mkdtemp(prefix="feagi_")

    def load_essential_genome(self) -> Dict[str, Any]:
        """Load the essential genome from the default templates."""
        try:
            # Multiple possible locations to look for the essential genome
            possible_paths = [
                # Original path
                os.path.join(os.path.dirname(__file__), "../../../../evo/defaults/genome/essential_genome.json"),
                
                # Alternative paths relative to current file
                os.path.join(os.path.dirname(__file__), "../../../../../feagi/evo/defaults/genome/essential_genome.json"),
                os.path.join(os.path.dirname(__file__), "../../../../../evo/defaults/genome/essential_genome.json"),
                
                # Paths relative to working directory
                os.path.join(os.getcwd(), "feagi/evo/defaults/genome/essential_genome.json"),
                os.path.join(os.getcwd(), "feagi_core/feagi/evo/defaults/genome/essential_genome.json"),
                
                # Check FEAGI_HOME environment variable if set
                os.path.join(os.environ.get("FEAGI_HOME", ""), "evo/defaults/genome/essential_genome.json"),
            ]
            
            # Find the first existing path
            essential_path = None
            for path in possible_paths:
                if path and os.path.exists(path):
                    essential_path = path
                    break
                    
            if not essential_path:
                self.logger.error(f"Essential genome template not found in any expected location")
                self.logger.error(f"Checked paths: {possible_paths}")
                return {"success": False, "error": "Essential genome template not found"}
                
            self.logger.info(f"Loading essential genome from {essential_path}")
                
            with open(essential_path, 'r') as f:
                genome_data = json.load(f)
            
            # Set the genome file name
            if self.state_manager:
                self.state_manager.genome_file_name = "essential_genome.json"
            
            # Call the existing load_genome method
            self._genome_filename = "essential_genome.json"
            result = self.load_genome(genome_data, "essential_genome.json")
            
            return result
            
        except Exception as e:
            self.logger.error(f"Failed to load essential genome: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
            return {"success": False, "error": str(e)}

    def load_barebones_genome(self) -> Dict[str, Any]:
        """Load the barebones genome from the default templates."""
        try:
            # Multiple possible locations to look for the barebones genome
            possible_paths = [
                # Original path
                os.path.join(os.path.dirname(__file__), "../../../../evo/defaults/genome/barebones_genome.json"),
                
                # Alternative paths relative to current file
                os.path.join(os.path.dirname(__file__), "../../../../../feagi/evo/defaults/genome/barebones_genome.json"),
                os.path.join(os.path.dirname(__file__), "../../../../../evo/defaults/genome/barebones_genome.json"),
                
                # Paths relative to working directory
                os.path.join(os.getcwd(), "feagi/evo/defaults/genome/barebones_genome.json"),
                os.path.join(os.getcwd(), "feagi_core/feagi/evo/defaults/genome/barebones_genome.json"),
                
                # Check FEAGI_HOME environment variable if set
                os.path.join(os.environ.get("FEAGI_HOME", ""), "evo/defaults/genome/barebones_genome.json"),
            ]
            
            # Find the first existing path
            barebones_path = None
            for path in possible_paths:
                if path and os.path.exists(path):
                    barebones_path = path
                    break
                    
            if not barebones_path:
                self.logger.error(f"Barebones genome template not found in any expected location")
                self.logger.error(f"Checked paths: {possible_paths}")
                return {"success": False, "error": "Barebones genome template not found"}
                
            self.logger.info(f"Loading barebones genome from {barebones_path}")
                
            with open(barebones_path, 'r') as f:
                genome_data = json.load(f)
            
            # Set the genome file name
            if self.state_manager:
                self.state_manager.genome_file_name = "barebones_genome.json"
            
            # Call the existing load_genome method
            self._genome_filename = "barebones_genome.json"
            result = self.load_genome(genome_data, "barebones_genome.json")
            
            return result
            
        except Exception as e:
            self.logger.error(f"Failed to load barebones genome: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
            return {"success": False, "error": str(e)}

    def load_genome(self, genome_data: Dict[str, Any], filename: str = "genome.json") -> Dict[str, Any]:
        """Load a genome and prepare it for use."""
        try:
            from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis, develop_brain_from_genome
            try:
                # Try to import these from the new location
                from feagi.evo.genome_validator import genome_validator
            except ImportError:
                # Fall back to the old location
                from feagi.core.genome.genome_validator import genome_validator
            
            self.logger.info(f"Loading genome from {filename}")
            
            # Set brain readiness to False while loading
            if self.state_manager:
                self.state_manager.set_brain_readiness(False)
            
            # Store genome filename 
            self._genome_filename = filename
            
            # Validate genome structure - returns a boolean now
            validation_result = genome_validator(genome_data)
            if not validation_result:
                self.logger.error(f"Invalid genome structure")
                return {"success": False, "error": "Invalid genome structure"}
                
            # Store the current genome
            self._current_genome = genome_data
            
            # Update state manager with genome data but not loaded state yet
            if self.state_manager:
                self.state_manager.genome = genome_data
                self.state_manager.genome_file_name = filename
                # Don't set to LOADED yet - wait until brain development succeeds
                
            # Save genome data to a temporary file
            temp_genome_path = os.path.join(self._temp_dir, "temp_genome.json")
            with open(temp_genome_path, 'w') as f:
                json.dump(genome_data, f)
                
            # Initialize embryogenesis
            embry = NeuroEmbryogenesis(
                connectome_manager=self._connectome_manager,
                progress_callback=self._handle_embryogenesis_progress
            )
            
            # Develop brain from genome using the temporary file
            success, stats = develop_brain_from_genome(
                genome_path=temp_genome_path,
                connectome_manager=self._connectome_manager
            )
            
            if not success:
                self.logger.error(f"Failed to develop brain from genome")
                # Set error state since brain development failed
                if self.state_manager:
                    from feagi.core.state_manager import GenomeState
                    self.state_manager.set_genome_state(GenomeState.ERROR)
                    self.state_manager.set_brain_readiness(False)
                return {"success": False, "error": "Failed to develop brain from genome"}
                
            # Only set LOADED state after successful brain development
            if self.state_manager:
                from feagi.core.state_manager import GenomeState
                self.state_manager.set_genome_state(GenomeState.LOADED)
                self.state_manager.set_brain_readiness(True)
                
            # Get cortical area count from connectome manager
            cortical_area_count = len(getattr(self._connectome_manager, 'cortical_areas', {}))
            
            # Log success
            self.logger.info(f"Genome loaded successfully: {cortical_area_count} cortical areas created")
            
            # Return success
            return {
                "success": True, 
                "cortical_area_count": cortical_area_count
            }
            
        except Exception as e:
            self.logger.error(f"Error loading genome: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
            
            # Update state manager with error
            if self.state_manager:
                from feagi.core.state_manager import GenomeState
                self.state_manager.set_genome_state(GenomeState.ERROR)
                self.state_manager.set_brain_readiness(False)
                
            return {"success": False, "error": str(e)}

    def _handle_embryogenesis_progress(self, stage, percentage, message):
        """Handle progress updates from the neuroembryogenesis process."""
        self.logger.info(f"{stage} {percentage:.1f}% - {message}", emoji1="  ")

    def get_genome(self) -> Optional[Dict[str, Any]]:
        """Get the currently loaded genome data."""
        if self._current_genome is None:
            self.logger.warning("No genome has been loaded")
            return None
     
        return self._current_genome
        
    def get_genome_filename(self) -> Optional[str]:
        """Get the filename of the currently loaded genome."""
        return self._genome_filename

    def get_genome_file_name(self) -> Dict[str, str]:
        """Get the genome file name in the format expected by the REST API."""
        if self._genome_filename:
            return {"file_name": self._genome_filename}
        else:
            return {"file_name": "No genome loaded"}

    def get_default_genomes(self) -> Dict[str, Any]:
        """Get a list of default genome files with their contents."""
        try:
            # Get the data path where default genomes are stored
            defaults_path = os.path.join(self._get_data_path(), "genome")
            
            # Check if the directory exists
            if not os.path.exists(defaults_path):
                self.logger.warning(f"Default genomes directory not found: {defaults_path}")
                return {}
            
            # Get all .json files in the directory
            default_genomes = {}
            
            for filename in os.listdir(defaults_path):
                if filename.endswith(".json"):
                    file_path = os.path.join(defaults_path, filename)
                    try:
                        with open(file_path, 'r') as f:
                            genome_data = json.load(f)
                            
                            # Store basic metadata about the genome
                            default_genomes[filename] = {
                                "title": genome_data.get("genome_title", "Untitled Genome"),
                                "description": genome_data.get("genome_description", ""),
                                "file_path": file_path
                            }
                    except Exception as e:
                        self.logger.error(f"Error loading default genome {filename}: {str(e)}")
            
            return default_genomes
                
        except Exception as e:
            self.logger.error(f"Error getting default genomes: {str(e)}")
            return {}

    def _get_data_path(self) -> str:
        """Get the data directory path."""
        # Try multiple possible locations
        possible_paths = [
            os.path.join(os.path.dirname(__file__), "../../../../data"),
            os.path.join(os.getcwd(), "data"),
            os.path.join(os.getcwd(), "feagi_core/data"),
            os.environ.get("FEAGI_DATA_PATH", "")
        ]
        
        for path in possible_paths:
            if path and os.path.exists(path):
                return path
                
        # If no path exists, return the first one as default
        return possible_paths[0]

    def get_genome_counter(self) -> int:
        """Get the current genome counter."""
        try:
            if self.state_manager:
                return self.state_manager.get_genome_counter()
            return 0
        except Exception as e:
            self.logger.error(f"Error getting genome counter: {str(e)}")
            return 0

    def get_generations(self) -> Dict[str, Any]:
        """Get details about all generations of genomes."""
        try:
            if not self.state_manager:
                return {}
                
            if hasattr(self.state_manager, 'generation_dict') and self.state_manager.generation_dict:
                return self.state_manager.generation_dict
            else:
                return {}
        except Exception as e:
            self.logger.error(f"Error retrieving generations: {str(e)}")
            return {}
    
    def get_change_register(self) -> Dict[str, Any]:
        """Get the evolution change register showing evolutionary history."""
        try:
            if not self.state_manager:
                return {}
                
            if hasattr(self.state_manager, 'evo_change_register') and self.state_manager.evo_change_register:
                return self.state_manager.evo_change_register
            else:
                return {}
        except Exception as e:
            self.logger.error(f"Error retrieving evolution change register: {str(e)}")
            return {}

    def deploy_genome(self, genome_filepath: str) -> bool:
        """Deploy a genome from a file path."""
        try:
            self.logger.info(f"Deploying genome from {genome_filepath}", emoji1="🧬")
            
            # Ensure the file exists
            if not os.path.exists(genome_filepath):
                self.logger.error(f"Genome file not found: {genome_filepath}", emoji1="❌")
                return False
                
            # Update state to LOADING
            if self.state_manager:
                from feagi.core.state_manager import GenomeState
                self.state_manager.set_genome_state(GenomeState.LOADING)
                self.state_manager.set_brain_readiness(False)
                
            # Load the genome data
            with open(genome_filepath, 'r') as f:
                genome_data = json.load(f)
                
            # Extract the filename for reference
            filename = os.path.basename(genome_filepath)
                
            # Load the genome using the service
            result = self.load_genome(genome_data, filename=filename)
            
            if not result.get("success", False):
                self.logger.error(f"Failed to load genome: {result.get('error', 'Unknown error')}", emoji1="❌")
                
                # Update state to ERROR
                if self.state_manager:
                    from feagi.core.state_manager import GenomeState
                    self.state_manager.set_genome_state(GenomeState.ERROR)
                    self.state_manager.set_brain_readiness(False)
                    
                return False
                
            # Update state to LOADED - this is already done in load_genome but we do it again for safety
            if self.state_manager:
                from feagi.core.state_manager import GenomeState
                self.state_manager.set_genome_state(GenomeState.LOADED)
                self.state_manager.set_brain_readiness(True)
                
            self.logger.info(f"Genome deployed successfully from {filename}", emoji1="✅")
            return True
            
        except json.JSONDecodeError:
            self.logger.error(f"Invalid JSON in genome file: {genome_filepath}", emoji1="❌")
            
            # Update state to ERROR
            if self.state_manager:
                from feagi.core.state_manager import GenomeState
                self.state_manager.set_genome_state(GenomeState.ERROR)
                self.state_manager.set_brain_readiness(False)
                
            return False
        except Exception as e:
            self.logger.error(f"Error deploying genome: {str(e)}", emoji1="❌")
            
            # Update state to ERROR
            if self.state_manager:
                from feagi.core.state_manager import GenomeState
                self.state_manager.set_genome_state(GenomeState.ERROR)
                self.state_manager.set_brain_readiness(False)
                
            return False

    def is_genome_loaded(self) -> bool:
        """Check if a genome is currently loaded."""
        # Check our internal state first
        if self._current_genome is not None:
            return True
        
        # Then check the state manager
        if self.state_manager:
            return self.state_manager.is_genome_loaded()
        
        return False 