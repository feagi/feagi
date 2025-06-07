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
from pathlib import Path
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
        
        print(f"[DEBUG] GENOME SERVICE: Initialized with brain_service: {brain_service is not None}")
        if brain_service:
            self.logger.info("[DEBUG] GENOME SERVICE: Using provided brain service instance")
        else:
            self.logger.info("[DEBUG] GENOME SERVICE: No brain service provided, will create when needed")

    def load_genome(self, genome_data: Dict[str, Any], filename: str = "genome.json") -> Dict[str, Any]:
        """Load a genome and prepare it for use."""
        try:
            # ARCHITECTURE: Only import NeuroEmbryogenesis - no old develop_brain_from_genome
            from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
            try:
                # Try to import these from the new location
                from feagi.evo.genome_validator import genome_validator_with_errors
            except ImportError:
                # Fallback to the old location
                from feagi.core.genome.genome_validator import genome_validator
            
            self.logger.info(f"Loading genome from {filename}")
            
            # STEP 1: START BURST ENGINE FIRST (new design requirement)
            self.logger.info("Step 1: Starting burst engine before genome load")
            if self.state_manager:
                try:
                    from feagi.api.core.services.brain.brain_service import BrainService
                    from feagi.core.state_manager import ServiceState
                    
                    # Check current burst engine state
                    current_state = self.state_manager.get_burst_engine_state()
                    self.logger.info(f"Current burst engine state: {current_state}", status="[CHECK]")
                    
                    if current_state != ServiceState.READY:
                        self.logger.info("Starting burst engine before genome load", status="[START]")
                        brain_service = BrainService(self._connectome_manager, self.state_manager)
                        
                        start_success = brain_service.start_burst_engine()
                        if start_success:
                            self.logger.info("Burst engine started successfully", status="[OK]")
                        else:
                            self.logger.error("Failed to start burst engine", status="[ERR]")
                            # Continue with genome loading even if burst engine fails to start
                    else:
                        self.logger.info("Burst engine already running", status="[OK]")
                        
                except Exception as burst_start_error:
                    self.logger.error(f"Error starting burst engine: {str(burst_start_error)}", status="[ERR]")
                    # Continue with genome loading even if burst engine start fails
                    
            # Set brain readiness to False while loading
            if self.state_manager:
                from feagi.core.state_manager import GenomeState
                self.state_manager.set_genome_state(GenomeState.LOADING)
                self.state_manager.set_brain_readiness(False)
                # Clear all brain stats during loading
                self.state_manager.brain_stats = {}
                self.state_manager.cortical_list = []
                self.state_manager.genome_validity = None

            # CRITICAL: Preserve old genome data BEFORE setting new values for comparison
            old_genome_data = self._current_genome
            old_genome_filename = self._genome_filename
            
            # Store genome filename 
            self._genome_filename = filename
            
            # Load FEAGI configuration to check genome settings
            try:
                from feagi.config.toml_loader import load_feagi_config, get_genome_config
                config = load_feagi_config()
                genome_config = get_genome_config(config)
                allow_auto_recovery = genome_config.auto_recovery_on_validation_failure
            except Exception as e:
                self.logger.warning(f"Could not load FEAGI configuration, defaulting to allow auto-recovery: {e}")
                allow_auto_recovery = True  # Default to allow auto-recovery if config fails
            
            # Validate the genome with detailed error reporting
            try:
                from feagi.evo.genome_validator import genome_validator_with_errors
            except ImportError:
                # Fallback to basic validator if detailed validator not available
                try:
                    from feagi.core.genome.genome_validator import genome_validator
                    validation_result = {
                        "valid": genome_validator(genome_data),
                        "errors": [],
                        "error_summary": "Basic validation failed" if not genome_validator(genome_data) else None
                    }
                except ImportError as e:
                    self.logger.error(f"Error loading genome: {e}")
                    return {"success": False, "error": f"Genome validator not available: {e}"}
            else:
                # Check if auto-recovery is enabled to determine which validation to use
                if allow_auto_recovery:
                    # Use silent validation for initial check to avoid logging errors that will be fixed
                    from feagi.evo.genome_validator import genome_validator_with_errors_silent
                    validation_result = genome_validator_with_errors_silent(genome_data)
                else:
                    # Auto-recovery disabled - use regular validation (log all errors)
                    validation_result = genome_validator_with_errors(genome_data)
            
            # Initialize auto_recovery_details at the start
            auto_recovery_details = {
                "recovery_performed": False,
                "recovery_reason": "No auto-recovery attempted",
                "validation_warnings": []
            }
            
            # Handle validation failures based on configuration
            if not validation_result["valid"]:
                error_msg = validation_result.get("error_summary", "Invalid genome structure")
                
                # Log specific errors if available
                specific_errors = validation_result.get("errors", [])
                
                # Check if auto-recovery is allowed
                if not allow_auto_recovery:
                    # Only log errors when auto-recovery is disabled (they won't be fixed)
                    self.logger.error(f"Genome validation failed: {error_msg}")
                    if specific_errors:
                        self.logger.error("Specific validation errors:")
                        for error in specific_errors:
                            self.logger.error(f"  - {error}")
                    
                    self.logger.error("Auto-recovery is disabled in configuration - failing genome load")
                    return {
                        "success": False,
                        "error": error_msg,
                        "validation_errors": specific_errors,
                        "message": "Genome validation failed and auto-recovery is disabled"
                    }
                else:
                    # Auto-recovery is enabled - suppress initial validation error logging
                    # Just log that we're attempting auto-recovery instead of the detailed errors
                    self.logger.info(f"Genome validation found {len(specific_errors)} issues - attempting auto-recovery")
                    self.logger.info("Auto-recovery enabled - attempting to sanitize invalid morphologies")
                    
                    # Attempt to sanitize invalid morphologies
                    try:
                        from feagi.evo.genome_validator import sanitize_invalid_morphologies
                        sanitization_result = sanitize_invalid_morphologies(genome_data)
                        
                        # Use the sanitized genome
                        genome_data = sanitization_result["genome"]
                        removed_morphologies = sanitization_result["removed_morphologies"]
                        fixed_references = sanitization_result["fixed_references"]
                        recovery_summary = sanitization_result["recovery_summary"]
                        
                        self.logger.info(f"Auto-recovery completed: {recovery_summary}")
                        if removed_morphologies:
                            self.logger.info(f"Removed invalid morphologies: {', '.join(removed_morphologies)}")
                        if fixed_references:
                            self.logger.info(f"Fixed {len(fixed_references)} blueprint references")
                        
                        # Re-validate after sanitization
                        try:
                            post_sanitization_result = genome_validator_with_errors(genome_data)
                            if post_sanitization_result["valid"]:
                                self.logger.info("Genome validation passed after auto-recovery sanitization")
                                validation_result = post_sanitization_result  # Update validation result
                                if self.state_manager:
                                    self.state_manager.genome_validity = True
                            else:
                                # NOW log the errors since auto-recovery couldn't fix them
                                remaining_error_msg = post_sanitization_result.get("error_summary", "Unknown validation issues")
                                remaining_errors = post_sanitization_result.get("errors", [])
                                
                                self.logger.error(f"Genome still has validation issues after auto-recovery: {remaining_error_msg}")
                                if remaining_errors:
                                    self.logger.error("Remaining validation errors after auto-recovery:")
                                    for error in remaining_errors:
                                        self.logger.error(f"  - {error}")
                                
                                validation_result = post_sanitization_result  # Update with new validation result
                                if self.state_manager:
                                    self.state_manager.genome_validity = False
                        except Exception as revalidation_error:
                            self.logger.warning(f"Could not re-validate after sanitization: {revalidation_error}")
                            # Assume it's still invalid but continue
                            if self.state_manager:
                                self.state_manager.genome_validity = False
                        
                        # Store auto-recovery details for inclusion in response (CRITICAL - don't overwrite later!)
                        auto_recovery_details = {
                            "recovery_performed": True,
                            "removed_morphologies": removed_morphologies,
                            "fixed_references": fixed_references,
                            "recovery_summary": recovery_summary,
                            "original_errors": specific_errors,
                            "validation_warnings": sanitization_result.get("validation_warnings", [])
                        }
                        
                    except Exception as sanitization_error:
                        self.logger.error(f"Auto-recovery sanitization failed: {sanitization_error}")
                        # Now log the original errors since auto-recovery failed
                        self.logger.error(f"Original genome validation failed: {error_msg}")
                        if specific_errors:
                            self.logger.error("Original validation errors:")
                            for error in specific_errors:
                                self.logger.error(f"  - {error}")
                        
                        # Fall back to original approach - mark as invalid but continue
                        if self.state_manager:
                            self.state_manager.genome_validity = False
                        auto_recovery_details = {
                            "recovery_performed": False,
                            "recovery_error": str(sanitization_error),
                            "original_errors": specific_errors
                        }
            else:
                # Validation passed initially - keep the original auto_recovery_details (no changes needed)
                if self.state_manager:
                    self.state_manager.genome_validity = True
            
            # Store the current genome
            self._current_genome = genome_data
            
            # ARCHITECTURE IMPROVEMENT: Stage sanitized genome in state manager FIRST
            # This ensures connectome manager always builds from single source of truth
            if self.state_manager:
                self.state_manager.genome = genome_data
                self.state_manager.genome_file_name = filename
                self.state_manager.genome_validity = True if validation_result.get("valid") else False
                # Set to STAGING state while brain development is in progress
                from feagi.core.state_manager import GenomeState
                self.state_manager.set_genome_state(GenomeState.LOADING)
                
                self.logger.info("Sanitized genome staged in state manager as single source of truth")
                
            # CRITICAL: Prepare connectome for new genome loading (clear existing brain data)
            self.logger.info("Preparing connectome for new genome loading...")
            preparation_result = self._connectome_manager.prepare_for_new_genome(genome_data, save_current_state=True)
            if not preparation_result.get("success", False):
                self.logger.error(f"Failed to prepare connectome for new genome")
                if self.state_manager:
                    from feagi.core.state_manager import GenomeState
                    self.state_manager.set_genome_state(GenomeState.ERROR)
                    self.state_manager.set_brain_readiness(False)
                    self.state_manager.genome_validity = False
                return {"success": False, "error": "Failed to prepare connectome for new genome"}
            
            self.logger.info(f"[OK] Connectome preparation complete: {preparation_result.get('message', 'Ready for genome loading')}")
                
            # ARCHITECTURE IMPROVEMENT: Build brain from state manager's genome (not temp file)
            # This ensures connectome manager always uses the sanitized genome from state manager
            self.logger.info("Building brain from state manager's sanitized genome...")
            
            # Initialize embryogenesis
            embry = NeuroEmbryogenesis(
                connectome_manager=self._connectome_manager,
                progress_callback=self._handle_embryogenesis_progress
            )
            
            # CRITICAL: Develop brain from state manager's genome (single source of truth)
            # No more temp files - connectome manager reads directly from state manager
            try:
                success = embry.develop_brain_from_genome_data(genome_data)
                
                if not success:
                    error_msg = embry.error or "Unknown error during brain development"
                    self.logger.error(f"Failed to develop brain from genome: {error_msg}")
                    
                    # Set error state since brain development failed
                    if self.state_manager:
                        from feagi.core.state_manager import GenomeState
                        self.state_manager.set_genome_state(GenomeState.ERROR)
                        self.state_manager.set_brain_readiness(False)
                        self.state_manager.genome_validity = False
                    return {"success": False, "error": f"Failed to develop brain from genome: {error_msg}"}
                
                # Get development statistics
                stats = embry.get_development_statistics()
                self.logger.info(f"Brain development completed: {stats.get('total_neurons', 0)} neurons, {stats.get('total_synapses', 0)} synapses")
                
            except Exception as dev_error:
                self.logger.error(f"Exception during brain development: {str(dev_error)}")
                if self.state_manager:
                    from feagi.core.state_manager import GenomeState
                    self.state_manager.set_genome_state(GenomeState.ERROR)
                    self.state_manager.set_brain_readiness(False)
                    self.state_manager.genome_validity = False
                return {"success": False, "error": f"Exception during brain development: {str(dev_error)}"}
            
            # CRITICAL: Update burst engine with new genome - this reinitializes injection service
            self.logger.info("Updating burst engine with newly developed brain...")
            try:
                from feagi.npu.burst_engine import BurstEngine
                burst_engine = BurstEngine.get_instance()
                if burst_engine:
                    burst_engine.update_with_genome()
                    self.logger.info("[OK] Burst engine updated with new genome - injection service reinitialized")
                else:
                    self.logger.warning("No burst engine instance found - injection service may not be available")
            except Exception as e:
                self.logger.warning(f"Error updating burst engine with genome: {str(e)}")
                # Don't fail the genome loading because of this
            
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
                    
                    # Set genome validity based on earlier validation results
                    # If validation failed earlier, keep genome_validity as False
                    # If validation passed and brain development succeeded, set to True
                    if not hasattr(self.state_manager, 'genome_validity') or self.state_manager.genome_validity is None:
                        # No previous validation state, set to True since brain development succeeded
                        self.state_manager.genome_validity = True
                    # If genome_validity is already False from validation failure, keep it False
                    # If genome_validity is already True from validation success, keep it True
                    
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
                
                # Automatically start the burst engine if it's not already running
                try:
                    # Check if we have access to the brain service through the core API
                    # We need to import at runtime to avoid circular imports
                    if hasattr(self.state_manager, 'get_burst_engine_state'):
                        from feagi.core.state_manager import ServiceState
                        burst_state = self.state_manager.get_burst_engine_state()
                        
                        if burst_state != ServiceState.READY:
                            self.logger.info("Burst engine not running, starting automatically after genome load")
                            
                            # Properly start the burst engine through the brain service
                            try:
                                if self._brain_service:
                                    print(f"[DEBUG] GENOME SERVICE: Using existing brain service for auto-start")
                                    success = self._brain_service.start_burst_engine()
                                else:
                                    print(f"[DEBUG] GENOME SERVICE: Creating temporary brain service for auto-start")
                                    from feagi.api.core.services.brain.brain_service import BrainService
                                    brain_service = BrainService(self._connectome_manager, self.state_manager)
                                    success = brain_service.start_burst_engine()
                                    
                                if success:
                                    self.logger.info("Burst engine started automatically")
                                else:
                                    self.logger.warning("Failed to start burst engine automatically")
                            except Exception as start_error:
                                self.logger.warning(f"Error starting burst engine: {str(start_error)}")
                                # Fallback: Set exit condition to False (legacy method)
                                self.state_manager.exit_condition = False
                                self.logger.info("Used fallback method to start burst engine")
                        else:
                            self.logger.info("Burst engine already running")
                    else:
                        # Fallback method - directly set exit_condition to False and set burst engine state
                        self.logger.info("Starting burst engine using fallback method")
                        self.state_manager.exit_condition = False
                        # Also set the burst engine state to READY
                        self.state_manager.set_burst_engine_state(ServiceState.READY)
                        
                except Exception as burst_error:
                    # Don't fail the genome loading if burst engine auto-start fails
                    # Just log the error and continue
                    self.logger.warning(f"Failed to auto-start burst engine: {str(burst_error)}")
                    self.logger.warning("You may need to start the burst engine manually")
                
            # Get cortical area count from connectome manager for return value
            cortical_area_count = len(getattr(self._connectome_manager, 'cortical_areas', {}))
            
            # CRITICAL: Only increment genome counter for ACTUALLY NEW genomes
            if self.state_manager:
                old_genome_counter = self.state_manager.get_genome_counter()
                
                # Check if this is genuinely a NEW genome (different from what we had before)
                is_new_genome = False
                if old_genome_data is None or old_genome_filename != filename:
                    # Definitely new - no previous genome or different filename
                    is_new_genome = True
                    self.logger.info(f"NEW genome detected: filename changed from '{old_genome_filename}' to '{filename}'")
                else:
                    # Same filename - check if genome data actually changed
                    import hashlib
                    new_hash = hashlib.md5(json.dumps(genome_data, sort_keys=True).encode()).hexdigest()
                    old_hash = hashlib.md5(json.dumps(old_genome_data, sort_keys=True).encode()).hexdigest()
                    if new_hash != old_hash:
                        is_new_genome = True
                        self.logger.info(f"NEW genome detected: data changed (hash: {old_hash[:8]} → {new_hash[:8]})")
                    else:
                        self.logger.info(f"[RELOAD] SAME genome being reloaded: '{filename}' with identical data")
                
                # Only increment counter and update timestamp for genuinely new genomes
                if is_new_genome:
                    self.state_manager.increment_genome_counter()
                    current_genome_number = self.state_manager.get_genome_counter()
                    self.logger.info(f"[OK] Genome counter incremented to {current_genome_number}")
                    
                    # Update timestamp to signal change to downstream clients
                    import time
                    new_genome_timestamp = int(time.time() * 1000)  # milliseconds
                    self.state_manager.set_genome_timestamp(new_genome_timestamp)
                    self.logger.info(f"[OK] Genome timestamp updated to {new_genome_timestamp} (signals NEW genome to clients)")
                else:
                    current_genome_number = old_genome_counter
                    self.logger.info(f"[SKIP] Genome counter NOT incremented (same genome reloaded)")
                    self.logger.info(f"[SKIP] Genome timestamp NOT updated (prevents reload loop)")
            
            # Log success
            if self.state_manager and hasattr(self.state_manager, 'genome_validity') and not self.state_manager.genome_validity:
                self.logger.info(f"Genome loaded successfully but marked as INVALID due to validation failures: {cortical_area_count} cortical areas created")
            else:
                self.logger.info(f"Genome loaded successfully: {cortical_area_count} cortical areas created")
            
            # Return success with detailed information including validation status
            result = {
                "success": True, 
                "cortical_area_count": cortical_area_count,
                "message": "Genome loaded and state manager fully synchronized"
            }
            
            # Include validation errors in response if validation failed but loading succeeded
            if not validation_result["valid"]:
                result["validation_errors"] = validation_result.get("errors", [])
                result["genome_validity"] = False
                
                # Check if auto-recovery was performed
                if auto_recovery_details.get("recovery_performed", False):
                    result["message"] = f"Genome loaded with auto-recovery: {auto_recovery_details.get('recovery_summary', 'Auto-recovery performed')}"
                    result["auto_recovery_performed"] = True
                    result["removed_morphologies"] = auto_recovery_details.get("removed_morphologies", [])
                    result["fixed_references"] = auto_recovery_details.get("fixed_references", [])
                    # Include validation warnings from auto-recovery
                    result["validation_warnings"] = auto_recovery_details.get("validation_warnings", [])
                else:
                    result["message"] = f"Genome loaded but marked as invalid due to validation failures: {validation_result.get('error_summary', 'Validation failed')}"
                    result["auto_recovery_performed"] = False
                    result["validation_warnings"] = []
            else:
                result["genome_validity"] = True
                result["auto_recovery_performed"] = False
                # Even if validation passed, include warnings from auto-recovery if any corrections were made
                if auto_recovery_details.get("recovery_performed", False):
                    result["validation_warnings"] = auto_recovery_details.get("validation_warnings", [])
                else:
                    result["validation_warnings"] = []
            
            # Include auto-recovery details in response
            result["auto_recovery_details"] = auto_recovery_details
            
            return result
            
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

    def _handle_embryogenesis_progress(self, stage, percentage, message):
        """Handle progress updates from the neuroembryogenesis process."""
        self.logger.info(f"{stage} {percentage:.1f}% - {message}", status="[PROC]")

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
            defaults_path = Path(self._get_data_path()) / "genome"
            
            # Check if the directory exists
            if not defaults_path.exists():
                self.logger.warning(f"Default genomes directory not found: {defaults_path}")
                return {}
            
            # Get all .json files in the directory
            default_genomes = {}
            
            for file_path in defaults_path.glob("*.json"):
                try:
                    with file_path.open('r') as f:
                        genome_data = json.load(f)
                        
                        # Store basic metadata about the genome
                        default_genomes[file_path.name] = {
                            "title": genome_data.get("genome_title", "Untitled Genome"),
                            "description": genome_data.get("genome_description", ""),
                            "file_path": str(file_path)
                        }
                except Exception as e:
                    self.logger.error(f"Error loading default genome {file_path.name}: {str(e)}")
            
            return default_genomes
                
        except Exception as e:
            self.logger.error(f"Error getting default genomes: {str(e)}")
            return {}

    def _get_data_path(self) -> str:
        """Get the data directory path."""
        # Try multiple possible locations
        current_dir = Path(__file__).parent
        cwd = Path.cwd()
        
        possible_paths = [
            current_dir / "../../../../data",
            cwd / "data",
            cwd / "feagi_core/data",
            Path(os.environ.get("FEAGI_DATA_PATH", "")) if os.environ.get("FEAGI_DATA_PATH") else None
        ]
        
        for path in possible_paths:
            if path and path.exists():
                return str(path)
                
        # If no path exists, return the first one as default
        return str(possible_paths[0])

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
            self.logger.info(f"Deploying genome from {genome_filepath}", status="[DNA]")
            
            # Ensure the file exists
            genome_path = Path(genome_filepath)
            if not genome_path.exists():
                self.logger.error(f"Genome file not found: {genome_filepath}", status="[ERR]")
                return False
                
            # Update state to LOADING
            if self.state_manager:
                from feagi.core.state_manager import GenomeState
                self.state_manager.set_genome_state(GenomeState.LOADING)
                self.state_manager.set_brain_readiness(False)
                
            # Load the genome data
            with genome_path.open('r') as f:
                genome_data = json.load(f)
                
            # Extract the filename for reference
            filename = genome_path.name
                
            # Load the genome using the service
            result = self.load_genome(genome_data, filename=filename)
            
            if not result.get("success", False):
                self.logger.error(f"Failed to load genome: {result.get('error', 'Unknown error')}", status="[ERR]")
                
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
                
            self.logger.info(f"Genome deployed successfully from {filename}", status="[OK]")
            return True
            
        except json.JSONDecodeError:
            self.logger.error(f"Invalid JSON in genome file: {genome_filepath}", status="[ERR]")
            
            # Update state to ERROR
            if self.state_manager:
                from feagi.core.state_manager import GenomeState
                self.state_manager.set_genome_state(GenomeState.ERROR)
                self.state_manager.set_brain_readiness(False)
                
            return False
        except Exception as e:
            self.logger.error(f"Error deploying genome: {str(e)}", status="[ERR]")
            
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

    def load_default_genome(self, genome_name: str) -> Dict[str, Any]:
        """
        Load a genome from the default templates directory.
        
        Args:
            genome_name: Name of the genome (e.g., 'essential', 'test', 'barebones')
        
        Returns:
            Dict containing success status and error information
        """
        try:
            # Normalize genome name
            genome_name = genome_name.replace('.json', '')
            genome_filename = f"{genome_name}_genome.json"
            
            # Find genome file using clean path resolution
            genome_path = self._find_default_genome_path(genome_filename)
            if not genome_path:
                return {"success": False, "error": f"Default genome '{genome_name}' not found"}
                
            self.logger.info(f"Loading {genome_name} genome from {genome_path}")
                
            # Load and process genome
            with genome_path.open('r') as f:
                genome_data = json.load(f)
            
            # Update state manager
            if self.state_manager:
                self.state_manager.genome_file_name = genome_filename
            
            # Load genome through the main pipeline
            self._genome_filename = genome_filename
            return self.load_genome(genome_data, genome_filename)
            
        except Exception as e:
            self.logger.error(f"Failed to load {genome_name} genome: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
            return {"success": False, "error": str(e)}

    def _find_default_genome_path(self, filename: str) -> Optional[Path]:
        """Clean path resolution for default genome files."""
        current_file = Path(__file__)
        current_dir = current_file.parent
        cwd = Path.cwd()
        
        # Search paths in order of preference
        search_paths = [
            current_dir / "../../../../evo/defaults/genome",  # Relative to this file
            cwd / "feagi/evo/defaults/genome",                # From working directory  
            cwd / "feagi_core/feagi/evo/defaults/genome",     # From project root
        ]
        
        # Add FEAGI_HOME if set
        feagi_home = os.getenv("FEAGI_HOME")
        if feagi_home:
            search_paths.append(Path(feagi_home) / "evo/defaults/genome")
        
        # Return first existing file
        for path in search_paths:
            if path.exists():
                genome_file = path / filename
                if genome_file.exists():
                    return genome_file
                    
        return None 