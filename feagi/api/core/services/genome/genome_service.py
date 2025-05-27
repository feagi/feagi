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
    
    def __init__(self, connectome_manager, state_manager=None, brain_service=None):
        """Initialize genome service."""
        super().__init__(connectome_manager, state_manager)
        self._current_genome = None
        self._genome_filename = None
        self._temp_dir = tempfile.mkdtemp(prefix="feagi_")
        self._brain_service = brain_service  # Reference to existing brain service
        
        print(f"🔥 GENOME SERVICE: Initialized with brain_service: {brain_service is not None}")
        if brain_service:
            self.logger.info("🔥 GENOME SERVICE: Using provided brain service instance")
        else:
            self.logger.info("🔥 GENOME SERVICE: No brain service provided, will create when needed")

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
            
            # STEP 1: START BURST ENGINE FIRST (design requirement - but pause it during genome load)
            self.logger.info("Step 1: Starting burst engine before genome load")
            if self.state_manager:
                try:
                    from feagi.api.core.services.brain.brain_service import BrainService
                    from feagi.core.state_manager import ServiceState
                    
                    # Check current burst engine state
                    current_state = self.state_manager.get_burst_engine_state()
                    burst_was_running = current_state in [ServiceState.READY, ServiceState.ON_HOLD]
                    
                    if burst_was_running:
                        # Stop the existing burst engine to prevent resource interference during genome loading
                        self.logger.info("🔥 GENOME SERVICE: Stopping burst engine for genome loading")
                        if self._brain_service:
                            stop_success = self._brain_service.stop_burst_engine()
                        else:
                            brain_service = BrainService(self._connectome_manager, self.state_manager)
                            stop_success = brain_service.stop_burst_engine()
                        
                        if stop_success:
                            self.logger.info("✅ Burst engine stopped successfully for genome loading")
                        else:
                            self.logger.warning("⚠️ Failed to cleanly stop burst engine - proceeding anyway")
                    else:
                        self.logger.info(f"✅ Burst engine not running (state: {current_state.name}) - proceeding with genome loading")
                        
                except Exception as engine_error:
                    self.logger.error(f"CRITICAL: Error starting/pausing burst engine: {str(engine_error)}")
                    return {"success": False, "error": f"Failed to start burst engine: {str(engine_error)}"}
            
            # STEP 2: PROCEED WITH GENOME LOADING (with burst engine safely paused)
            self.logger.info("Step 2: Proceeding with genome loading")
            
            # STEP 3: PREPARE CONNECTOME FOR NEW GENOME
            self.logger.info("Step 3: Preparing connectome for new genome")
            
            # Use the new ConnectomeManager method to handle all preparation
            preparation_result = self._connectome_manager.prepare_for_new_genome(genome_data, save_current_state=True)
            
            if not preparation_result["success"]:
                self.logger.error(f"Failed to prepare connectome: {preparation_result.get('error', 'Unknown error')}")
                if self.state_manager:
                    from feagi.core.state_manager import GenomeState
                    self.state_manager.set_genome_state(GenomeState.ERROR)
                    self.state_manager.set_brain_readiness(False)
                    self.state_manager.genome_validity = False
                return {"success": False, "error": f"Failed to prepare connectome: {preparation_result.get('error', 'Unknown error')}"}
            
            self.logger.info("✅ Connectome prepared successfully for new genome")
            
            # Log summary of preparation
            memory_req = preparation_result["memory_requirements"]
            capacity_res = preparation_result["capacity_results"]
            self.logger.info(f"Genome analysis: {memory_req['cortical_areas']} areas, "
                           f"{memory_req['estimated_neurons']} neurons, {memory_req['estimated_synapses']} synapses")
            self.logger.info(f"Memory allocation: {capacity_res['max_neurons']} neurons, "
                           f"{capacity_res['max_synapses']} synapses {'(reallocated)' if capacity_res['reallocated'] else '(existing)'}")
            
            # STEP 4: VALIDATE AND PREPARE GENOME DATA 
            self.logger.info("Step 4: Validating genome structure")
            
            # Set brain readiness to False while loading
            if self.state_manager:
                from feagi.core.state_manager import GenomeState
                self.state_manager.set_genome_state(GenomeState.LOADING)
                self.state_manager.set_brain_readiness(False)
                # Clear all brain stats during loading
                self.state_manager.brain_stats = {}
                self.state_manager.cortical_list = []
                self.state_manager.genome_validity = None
            
            # Store genome filename 
            self._genome_filename = filename
            
            # Validate genome structure - returns a boolean now
            validation_result = genome_validator(genome_data)
            if not validation_result:
                self.logger.error(f"Invalid genome structure")
                if self.state_manager:
                    from feagi.core.state_manager import GenomeState
                    self.state_manager.set_genome_state(GenomeState.ERROR)
                    self.state_manager.set_brain_readiness(False)
                    self.state_manager.genome_validity = False
                return {"success": False, "error": "Invalid genome structure"}
                
            # Store the current genome
            self._current_genome = genome_data
            
            # Update state manager with genome data but not loaded state yet
            if self.state_manager:
                self.state_manager.genome = genome_data
                self.state_manager.genome_file_name = filename
                # Don't set to LOADED yet - wait until brain development succeeds
            
            # STEP 5: RUN BRAIN EMBRYOGENESIS
            self.logger.info("Step 5: Running brain embryogenesis")
                
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
                    self.state_manager.genome_validity = False
                return {"success": False, "error": "Failed to develop brain from genome"}
                
            # CRITICAL: Complete state manager update after successful brain development
            if self.state_manager:
                from feagi.core.state_manager import GenomeState
                
                # Update state manager with comprehensive brain statistics for health checks
                try:
                    # Get statistics from connectome manager
                    cortical_area_count = len(getattr(self._connectome_manager, 'cortical_areas', {}))
                    
                    # Calculate neuron and synapse counts if methods exist
                    total_neurons = 0
                    total_synapses = 0
                    
                    if hasattr(self._connectome_manager, 'get_total_neuron_count'):
                        total_neurons = self._connectome_manager.get_total_neuron_count()
                    elif hasattr(self._connectome_manager, 'cortical_areas'):
                        # Fallback: count neurons in all cortical areas
                        for area_idx in self._connectome_manager.cortical_areas:
                            try:
                                if hasattr(self._connectome_manager, 'get_neurons_by_area'):
                                    area_neurons = self._connectome_manager.get_neurons_by_area(area_idx)
                                    total_neurons += len(area_neurons) if area_neurons else 0
                            except Exception:
                                pass
                    
                    if hasattr(self._connectome_manager, 'get_total_synapse_count'):
                        total_synapses = self._connectome_manager.get_total_synapse_count()
                    
                    # Update state manager with brain statistics (CRITICAL for health check)
                    self.state_manager.brain_stats = {
                        "neuron_count": total_neurons,
                        "synapse_count": total_synapses,
                        "cortical_area_count": cortical_area_count
                    }
                    
                    # Create cortical list for health check compatibility (CRITICAL)
                    cortical_ids = []
                    if hasattr(self._connectome_manager, 'cortical_areas'):
                        for area_idx, area in self._connectome_manager.cortical_areas.items():
                            # Try to get cortical_id from area object, fallback to string representation
                            if hasattr(area, 'cortical_id') and area.cortical_id:
                                cortical_ids.append(area.cortical_id)
                            else:
                                cortical_ids.append(f"CID{area_idx:03d}")
                    self.state_manager.cortical_list = cortical_ids
                    
                    # Set genome validity to True on successful load (CRITICAL)
                    self.state_manager.genome_validity = True
                    
                    # Set brain readiness and genome state (CRITICAL)
                    self.state_manager.set_brain_readiness(True)
                    self.state_manager.set_genome_state(GenomeState.LOADED)
                    
                    # Ensure connected_agents is initialized if not already set
                    if not hasattr(self.state_manager, 'connected_agents') or self.state_manager.connected_agents is None:
                        self.state_manager.connected_agents = 0  # Count of connected agents, not a list
                    
                    # Ensure changes_saved_externally is initialized
                    if not hasattr(self.state_manager, 'changes_saved_externally'):
                        self.state_manager.changes_saved_externally = False
                    
                    # Ensure exit_condition is properly set (for burst engine status)
                    if not hasattr(self.state_manager, 'exit_condition'):
                        self.state_manager.exit_condition = False
                    
                    self.logger.info(f"State manager fully synchronized: {total_neurons} neurons, {total_synapses} synapses, {cortical_area_count} cortical areas")
                    
                except Exception as stats_error:
                    self.logger.error(f"CRITICAL: Error updating state manager with brain statistics: {str(stats_error)}")
                    # Set error state since this is critical for health checks
                    self.state_manager.set_genome_state(GenomeState.ERROR)
                    self.state_manager.set_brain_readiness(False)
                    self.state_manager.genome_validity = False
                    return {"success": False, "error": f"Failed to update state manager: {str(stats_error)}"}
                
                # STEP 6: RESTART BURST ENGINE AFTER SUCCESSFUL GENOME LOADING
                try:
                    self.logger.info("Step 6: Restarting burst engine after successful genome loading")
                    
                    # Restart the burst engine now that genome loading is complete
                    if self._brain_service:
                        restart_success = self._brain_service.start_burst_engine()
                    else:
                        brain_service = BrainService(self._connectome_manager, self.state_manager)
                        restart_success = brain_service.start_burst_engine()
                    
                    if restart_success:
                        self.logger.info("✅ Burst engine restarted and ready for neural processing")
                    else:
                        self.logger.warning("⚠️ Failed to restart burst engine - you may need to start it manually")
                        
                except Exception as burst_error:
                    # Don't fail the genome loading if burst engine restart fails
                    # Just log the error and continue
                    self.logger.warning(f"Failed to restart burst engine: {str(burst_error)}")
                    self.logger.warning("You may need to start the burst engine manually")
                
            # Get cortical area count from connectome manager for return value
            cortical_area_count = len(getattr(self._connectome_manager, 'cortical_areas', {}))
            
            # Log success
            self.logger.info(f"Genome loaded successfully: {cortical_area_count} cortical areas created")
            
            # Return success with detailed information
            return {
                "success": True, 
                "cortical_area_count": cortical_area_count,
                "message": "Genome loaded and state manager fully synchronized"
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
                self.state_manager.genome_validity = False
                
            return {"success": False, "error": str(e)}

    def _calculate_genome_memory_requirements(self, genome_data: Dict[str, Any]) -> Dict[str, int]:
        """
        Calculate memory requirements from genome data.
        
        This method analyzes the genome to determine how much memory the ConnectomeManager
        needs to allocate for neurons, synapses, and cortical areas.
        
        Args:
            genome_data: The genome dictionary
            
        Returns:
            Dictionary with estimated memory requirements
        """
        try:
            requirements = {
                "cortical_areas": 0,
                "estimated_neurons": 0,
                "estimated_synapses": 0,
                "max_dimensions": [0, 0, 0]
            }
            
            if "blueprint" not in genome_data:
                self.logger.warning("No blueprint found in genome - using minimal requirements")
                return {
                    "cortical_areas": 1,
                    "estimated_neurons": 1000,
                    "estimated_synapses": 10000,
                    "max_dimensions": [10, 10, 10]
                }
            
            # Extract cortical area information from blueprint
            cortical_areas = {}
            
            # Parse blueprint keys to group by cortical_id
            # Blueprint format: "_____10c-{cortical_id}-{prop_category}-{property_name}-{type}"
            for blueprint_key, value in genome_data["blueprint"].items():
                parts = blueprint_key.split('-')
                if len(parts) >= 4:
                    cortical_id = parts[1]
                    prop_category = parts[2]  # 'cx' for cortical, 'nx' for neuron
                    property_name = parts[3]
                    
                    if cortical_id not in cortical_areas:
                        cortical_areas[cortical_id] = {}
                    
                    # Map blueprint property names to our expected names
                    if prop_category == "cx":  # Cortical properties
                        if property_name == "___bbx":
                            cortical_areas[cortical_id]["dimx"] = value
                        elif property_name == "___bby":
                            cortical_areas[cortical_id]["dimy"] = value
                        elif property_name == "___bbz":
                            cortical_areas[cortical_id]["dimz"] = value
                        elif property_name == "_n_cnt":
                            cortical_areas[cortical_id]["neurons_per_voxel"] = value
            
            requirements["cortical_areas"] = len(cortical_areas)
            
            # Calculate neuron and synapse requirements for each cortical area
            total_neurons = 0
            total_synapses = 0
            
            for cortical_id, properties in cortical_areas.items():
                # Get dimensions (default to 1x1x1 if missing)
                dim_x = int(properties.get("dimx", 1))
                dim_y = int(properties.get("dimy", 1))
                dim_z = int(properties.get("dimz", 1))
                
                # Update max dimensions
                requirements["max_dimensions"][0] = max(requirements["max_dimensions"][0], dim_x)
                requirements["max_dimensions"][1] = max(requirements["max_dimensions"][1], dim_y)
                requirements["max_dimensions"][2] = max(requirements["max_dimensions"][2], dim_z)
                
                # Calculate voxel count
                voxel_count = dim_x * dim_y * dim_z
                
                # Get neurons per voxel (default to 1)
                neurons_per_voxel = int(properties.get("neurons_per_voxel", 1))
                
                # Calculate neurons for this area
                area_neurons = voxel_count * neurons_per_voxel
                total_neurons += area_neurons
                
                # Estimate synapses (conservative estimate: avg 10 synapses per neuron)
                # This will be refined based on actual morphology connections
                estimated_synapses_per_neuron = 10
                area_synapses = area_neurons * estimated_synapses_per_neuron
                total_synapses += area_synapses
                
                self.logger.debug(f"Area {cortical_id}: {dim_x}x{dim_y}x{dim_z} = {voxel_count} voxels, "
                                f"{area_neurons} neurons, ~{area_synapses} synapses")
            
            requirements["estimated_neurons"] = total_neurons
            requirements["estimated_synapses"] = total_synapses
            
            # Add morphology-based synapse estimates
            if "neuron_morphologies" in genome_data:
                morphology_multiplier = len(genome_data["neuron_morphologies"]) * 1.2
                requirements["estimated_synapses"] = int(requirements["estimated_synapses"] * morphology_multiplier)
            
            # Ensure minimum reasonable values
            requirements["estimated_neurons"] = max(requirements["estimated_neurons"], 100)
            requirements["estimated_synapses"] = max(requirements["estimated_synapses"], 1000)
            
            self.logger.info(f"Genome analysis complete: {requirements['cortical_areas']} areas, "
                           f"~{requirements['estimated_neurons']} neurons, "
                           f"~{requirements['estimated_synapses']} synapses")
            
            return requirements
            
        except Exception as e:
            self.logger.error(f"Error calculating genome memory requirements: {str(e)}")
            # Return safe defaults on error
            return {
                "cortical_areas": 10,
                "estimated_neurons": 10000,
                "estimated_synapses": 100000,
                "max_dimensions": [20, 20, 20]
            }

    def _reset_connectome_singleton(self) -> bool:
        """
        Reset the ConnectomeManager singleton to allow creation of a fresh instance.
        
        This method forces the singleton pattern to release the current instance
        so that a new one can be created with proper memory allocation for the new genome.
        
        Returns:
            True if reset was successful, False otherwise
        """
        try:
            from feagi.bdu.connectome_manager import ConnectomeManager
            
            # Reset the singleton state using the proper method
            if hasattr(ConnectomeManager, '_instance') and ConnectomeManager._instance is not None:
                self.logger.info("Resetting ConnectomeManager singleton for fresh genome load")
                
                # Use the proper reset method
                ConnectomeManager.reset_singleton()
                
                self.logger.info("✅ ConnectomeManager singleton reset successfully")
                return True
            else:
                self.logger.info("No ConnectomeManager singleton to reset")
                return True
                
        except Exception as e:
            self.logger.error(f"Error resetting ConnectomeManager singleton: {str(e)}")
            return False

    def _save_current_connectome_state(self) -> bool:
        """
        Save the current connectome state to a file before replacing with new genome.
        
        This method serializes the current brain state so it can be restored later
        or analyzed for comparison with the new genome.
        
        Returns:
            True if save was successful, False otherwise
        """
        try:
            import json
            import datetime
            
            if not hasattr(self._connectome_manager, 'cortical_areas') or not self._connectome_manager.cortical_areas:
                self.logger.info("No connectome state to save - skipping backup")
                return True
            
            # Generate timestamp for unique filename
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_filename = f"connectome_backup_{timestamp}.json"
            backup_path = os.path.join(self._temp_dir, backup_filename)
            
            # Collect connectome state data
            connectome_state = {
                "metadata": {
                    "timestamp": timestamp,
                    "genome_filename": self._genome_filename,
                    "cortical_area_count": len(self._connectome_manager.cortical_areas)
                },
                "cortical_areas": {},
                "statistics": {}
            }
            
            # Save cortical area information
            for area_idx, area in self._connectome_manager.cortical_areas.items():
                try:
                    # Get neuron count safely
                    neuron_count = 0
                    if hasattr(self._connectome_manager, 'get_neurons_by_area'):
                        neurons = self._connectome_manager.get_neurons_by_area(area_idx)
                        neuron_count = len(neurons) if neurons else 0
                    
                    connectome_state["cortical_areas"][str(area_idx)] = {
                        "name": getattr(area, 'name', f"Area_{area_idx}"),
                        "cortical_id": getattr(area, 'cortical_id', f"ID_{area_idx}"),
                        "dimensions": getattr(area, 'dimensions', [1, 1, 1]),
                        "position": getattr(area, 'position', [0, 0, 0]),
                        "area_type": getattr(area, 'area_type', 'unknown'),
                        "neuron_count": neuron_count
                    }
                except Exception as area_error:
                    self.logger.warning(f"Error saving area {area_idx}: {area_error}")
            
            # Save overall statistics
            try:
                total_neurons = sum(
                    area_data["neuron_count"] 
                    for area_data in connectome_state["cortical_areas"].values()
                )
                connectome_state["statistics"] = {
                    "total_neurons": total_neurons,
                    "total_areas": len(connectome_state["cortical_areas"])
                }
            except Exception:
                connectome_state["statistics"] = {"total_neurons": 0, "total_areas": 0}
            
            # Write to file
            with open(backup_path, 'w') as f:
                json.dump(connectome_state, f, indent=2)
            
            self.logger.info(f"✅ Connectome state saved to: {backup_filename}")
            self.logger.info(f"Backup contains {connectome_state['statistics']['total_areas']} areas, "
                           f"{connectome_state['statistics']['total_neurons']} neurons")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to save connectome state: {str(e)}")
            return False

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