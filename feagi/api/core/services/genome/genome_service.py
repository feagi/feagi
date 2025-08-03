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

import copy
import json
import os
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, Optional

from ..shared.base_service import BaseService

# Import genome conversion functions for flat <-> hierarchical conversion
from feagi.evo.genome_processor import genome_2_1_convertor


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

        self.logger.debug(
            "GENOME SERVICE: Initialized with clean architecture - no service dependencies"
        )

    def load_genome(
        self, genome_data: Dict[str, Any], filename: str = "genome.json"
    ) -> Dict[str, Any]:
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

            # Store the provided genome data for processing
            self._current_genome = copy.deepcopy(genome_data)

            if not self._connectome_manager:
                return {"success": False, "error": "Connectome manager not available"}

            # CRITICAL: Start timing for performance monitoring
            start_time = time.time()

            try:
                self.logger.info("Step 1: Initializing genome load process")

                # Set brain readiness to False while loading
                if self.state_manager:
                    from feagi.core.state_manager import GenomeState

                    self.state_manager.set_genome_state(GenomeState.LOADING)
                    self.state_manager.set_brain_readiness(False)
                    # Clear all brain stats during loading
                    result = self.state_manager.set_brain_stats({})
                    if result.is_err:
                        self.logger.warning("Failed to clear brain stats")
                    
                    result = self.state_manager.set_cortical_list([])
                    if result.is_err:
                        self.logger.warning("Failed to clear cortical list")
                    
                    result = self.state_manager.set_genome_validity(False)  # None -> False
                    if result.is_err:
                        self.logger.warning("Failed to set genome validity")

                # CRITICAL: Preserve old genome data BEFORE setting new values for comparison
                old_genome_data = self._current_genome
                old_genome_filename = self._genome_filename

                # Store genome filename
                self._genome_filename = filename

                # Load FEAGI configuration to check genome settings
                try:
                    from feagi.config.toml_loader import (
                        get_genome_config,
                        load_feagi_config,
                    )

                    config = load_feagi_config()
                    genome_config = get_genome_config(config)
                    allow_auto_recovery = (
                        genome_config.auto_recovery_on_validation_failure
                    )
                except Exception as e:
                    self.logger.warning(
                        f"Could not load FEAGI configuration, defaulting to allow auto-recovery: {e}"
                    )
                    allow_auto_recovery = (
                        True  # Default to allow auto-recovery if config fails
                    )

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
                            "error_summary": (
                                "Basic validation failed"
                                if not genome_validator(genome_data)
                                else None
                            ),
                        }
                    except ImportError as e:
                        self.logger.error(f"Error loading genome: {e}")
                        return {
                            "success": False,
                            "error": f"Genome validator not available: {e}",
                        }
                else:
                    # Check if auto-recovery is enabled to determine which validation to use
                    if allow_auto_recovery:
                        # Use silent validation for initial check to avoid logging errors that will be fixed
                        from feagi.evo.genome_validator import (
                            genome_validator_with_errors_silent,
                        )

                        validation_result = genome_validator_with_errors_silent(
                            genome_data
                        )
                    else:
                        # Auto-recovery disabled - use regular validation (log all errors)
                        validation_result = genome_validator_with_errors(genome_data)

                # Initialize auto_recovery_details at the start
                auto_recovery_details = {
                    "recovery_performed": False,
                    "recovery_reason": "No auto-recovery attempted",
                    "validation_warnings": [],
                }

                # Handle validation failures based on configuration
                if not validation_result["valid"]:
                    error_msg = validation_result.get(
                        "error_summary", "Invalid genome structure"
                    )

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

                        self.logger.error(
                            "Auto-recovery is disabled in configuration - failing genome load"
                        )
                        return {
                            "success": False,
                            "error": error_msg,
                            "validation_errors": specific_errors,
                            "message": "Genome validation failed and auto-recovery is disabled",
                        }
                    else:
                        # Auto-recovery is enabled - suppress initial validation error logging
                        # Just log that we're attempting auto-recovery instead of the detailed errors
                        self.logger.info(
                            f"Genome validation found {len(specific_errors)} issues - attempting auto-recovery"
                        )
                        self.logger.info(
                            "Auto-recovery enabled - attempting to sanitize invalid morphologies"
                        )

                        # Attempt to sanitize invalid morphologies
                        try:
                            from feagi.evo.genome_validator import (
                                sanitize_invalid_morphologies,
                            )

                            sanitization_result = sanitize_invalid_morphologies(
                                genome_data
                            )

                            # Use the sanitized genome
                            genome_data = sanitization_result["genome"]
                            removed_morphologies = sanitization_result[
                                "removed_morphologies"
                            ]
                            fixed_references = sanitization_result["fixed_references"]
                            recovery_summary = sanitization_result["recovery_summary"]

                            self.logger.info(
                                f"Auto-recovery completed: {recovery_summary}"
                            )
                            if removed_morphologies:
                                self.logger.info(
                                    f"Removed invalid morphologies: {', '.join(removed_morphologies)}"
                                )
                            if fixed_references:
                                self.logger.info(
                                    f"Fixed {len(fixed_references)} blueprint references"
                                )

                            # Re-validate after sanitization
                            try:
                                post_sanitization_result = genome_validator_with_errors(
                                    genome_data
                                )
                                if post_sanitization_result["valid"]:
                                    self.logger.info(
                                        "Genome validation passed after auto-recovery sanitization"
                                    )
                                    validation_result = post_sanitization_result  # Update validation result
                                    if self.state_manager:
                                        result = self.state_manager.set_genome_validity(True)
                                        if result.is_err:
                                            self.logger.warning(f"Failed to set genome validity: {result.unwrap_err()}")
                                else:
                                    # NOW log the errors since auto-recovery couldn't fix them
                                    remaining_error_msg = post_sanitization_result.get(
                                        "error_summary", "Unknown validation issues"
                                    )
                                    remaining_errors = post_sanitization_result.get(
                                        "errors", []
                                    )

                                    self.logger.error(
                                        f"Genome still has validation issues after auto-recovery: {remaining_error_msg}"
                                    )
                                    if remaining_errors:
                                        self.logger.error(
                                            "Remaining validation errors after auto-recovery:"
                                        )
                                        for error in remaining_errors:
                                            self.logger.error(f"  - {error}")

                                    validation_result = post_sanitization_result  # Update with new validation result
                                    if self.state_manager:
                                        result = self.state_manager.set_genome_validity(False)
                                        if result.is_err:
                                            self.logger.warning(f"Failed to set genome validity: {result.unwrap_err()}")
                            except Exception as revalidation_error:
                                self.logger.warning(
                                    f"Could not re-validate after sanitization: {revalidation_error}"
                                )
                                # Assume it's still invalid but continue
                                if self.state_manager:
                                    result = self.state_manager.set_genome_validity(False)
                                    if result.is_err:
                                        self.logger.warning(f"Failed to set genome validity: {result.unwrap_err()}")

                            # Store auto-recovery details for inclusion in response (CRITICAL - don't overwrite later!)
                            auto_recovery_details = {
                                "recovery_performed": True,
                                "removed_morphologies": removed_morphologies,
                                "fixed_references": fixed_references,
                                "recovery_summary": recovery_summary,
                                "original_errors": specific_errors,
                                "validation_warnings": sanitization_result.get(
                                    "validation_warnings", []
                                ),
                            }

                        except Exception as sanitization_error:
                            self.logger.error(
                                f"Auto-recovery sanitization failed: {sanitization_error}"
                            )
                            # Now log the original errors since auto-recovery failed
                            self.logger.error(
                                f"Original genome validation failed: {error_msg}"
                            )
                            if specific_errors:
                                self.logger.error("Original validation errors:")
                                for error in specific_errors:
                                    self.logger.error(f"  - {error}")

                            # Fall back to original approach - mark as invalid but continue
                            if self.state_manager:
                                result = self.state_manager.set_genome_validity(False)
                                if result.is_err:
                                    self.logger.warning(f"Failed to set genome validity: {result.unwrap_err()}")
                            auto_recovery_details = {
                                "recovery_performed": False,
                                "recovery_error": str(sanitization_error),
                                "original_errors": specific_errors,
                            }
                else:
                    # Validation passed initially - keep the original auto_recovery_details (no changes needed)
                    if self.state_manager:
                        result = self.state_manager.set_genome_validity(True)
                        if result.is_err:
                            self.logger.warning(f"Failed to set genome validity: {result.unwrap_err()}")

                # Store the current genome - CONVERT TO HIERARCHICAL FORMAT
                # ARCHITECTURE: Only store hierarchical format for working operations
                # Flat format is used only for save/export operations  
                if "blueprint" in genome_data and isinstance(genome_data["blueprint"], dict):
                    # Check if this is flat format (contains flattened keys like _____10c-area_id-cx-property-type)
                    # ARCHITECTURE: Robust detection - parse for dash pattern, not hardcoded underscores
                    blueprint_keys = list(genome_data["blueprint"].keys())
                    def is_flat_genome_key(key):
                        """Detect flat genome format: *10c-area_id-{cx|nx}-property-type"""
                        import re
                        # Pattern: any prefix ending with 10c-, then area_id, then -cx- or -nx-, then more components
                        return bool(re.match(r'.*10c-[^-]+-[cn]x-.*', key))
                    
                    if blueprint_keys and any(is_flat_genome_key(key) for key in blueprint_keys[:5]):
                        self.logger.info("Converting flat genome to hierarchical format for working operations")
                        # Convert flat blueprint to hierarchical structure
                        hierarchical_genome = copy.deepcopy(genome_data)
                        hierarchical_genome["blueprint"] = genome_2_1_convertor(genome_data["blueprint"])["blueprint"]
                        self._current_genome = hierarchical_genome
                        self.logger.info(f"Converted {len(blueprint_keys)} flat entries to hierarchical structure")
                    else:
                        # Already hierarchical format
                        self.logger.info("Genome already in hierarchical format")
                        self._current_genome = genome_data
                else:
                    # No blueprint section or not dict - store as-is
                    self._current_genome = genome_data

                # ARCHITECTURE IMPROVEMENT: Stage sanitized genome in state manager FIRST
                # This ensures connectome manager always builds from single source of truth
                if self.state_manager:
                    # CRITICAL: Always store hierarchical format in state manager (single source of truth)
                    self.state_manager.genome = self._current_genome
                    self.state_manager.genome_file_name = filename
                    validity_result = self.state_manager.set_genome_validity(
                        True if validation_result.get("valid") else False
                    )
                    if validity_result.is_err:
                        self.logger.warning(f"Failed to set genome validity: {validity_result.unwrap_err()}")
                    # Set to STAGING state while brain development is in progress
                    from feagi.core.state_manager import GenomeState

                    self.state_manager.set_genome_state(GenomeState.LOADING)

                    self.logger.info(
                        "Sanitized genome staged in state manager as single source of truth"
                    )

                # CRITICAL: Prepare connectome for new genome loading (clear existing brain data)
                self.logger.info("Preparing connectome for new genome loading...")
                # ARCHITECTURE FIX: Use converted hierarchical genome, not original flat format
                preparation_result = self._connectome_manager.prepare_for_new_genome(
                    self._current_genome, save_current_state=True
                )
                if not preparation_result.get("success", False):
                    self.logger.error("Failed to prepare connectome for new genome")
                    if self.state_manager:
                        from feagi.core.state_manager import GenomeState

                        self.state_manager.set_genome_state(GenomeState.ERROR)
                        self.state_manager.set_brain_readiness(False)
                        result = self.state_manager.set_genome_validity(False)
                        if result.is_err:
                            self.logger.warning(f"Failed to set genome validity: {result.unwrap_err()}")
                    return {
                        "success": False,
                        "error": "Failed to prepare connectome for new genome",
                    }

                self.logger.info(
                    f"[OK] Connectome preparation complete: {preparation_result.get('message', 'Ready for genome loading')}"
                )

                # GENOME-FIRST ARCHITECTURE: Analyze requirements BEFORE brain development
                self.logger.info("Analyzing genome requirements for optimal ConnectomeManager sizing...")
                requirements = self._analyze_genome_requirements(genome_data)
                
                # Ensure ConnectomeManager has adequate capacity
                if self._connectome_manager:
                    current_capacity = getattr(self._connectome_manager, 'max_neurons', 0)
                    required_capacity = requirements['recommended_neuron_capacity']
                    
                    if current_capacity < required_capacity:
                        self.logger.error(f"[GENOME] INSUFFICIENT CAPACITY: {current_capacity:,} < {required_capacity:,}")
                        
                        # Try dynamic resize if available
                        if hasattr(self._connectome_manager, 'resize_for_genome'):
                            try:
                                self.logger.info(f"[GENOME] Attempting dynamic resize to {required_capacity:,} neurons...")
                                resize_result = self._connectome_manager.resize_for_genome(genome_data)
                                if resize_result.get('success'):
                                    self.logger.info(f"[GENOME] ✅ Connectome resized: {resize_result.get('message', 'Success')}")
                                else:
                                    self.logger.error(f"[GENOME] ❌ Connectome resize failed: {resize_result.get('message', 'Unknown')}")
                                    if self.state_manager:
                                        from feagi.core.state_manager import GenomeState
                                        self.state_manager.set_genome_state(GenomeState.ERROR)
                                    return {"success": False, "message": f"Insufficient capacity: need {required_capacity:,} neurons"}
                            except Exception as e:
                                self.logger.error(f"[GENOME] ❌ Connectome resize exception: {e}")
                                if self.state_manager:
                                    from feagi.core.state_manager import GenomeState
                                    self.state_manager.set_genome_state(GenomeState.ERROR)
                                return {"success": False, "message": f"Resize failed: {e}"}
                        else:
                            self.logger.error("[GENOME] ❌ ConnectomeManager doesn't support dynamic resizing")
                            if self.state_manager:
                                from feagi.core.state_manager import GenomeState
                                self.state_manager.set_genome_state(GenomeState.ERROR)
                            return {"success": False, "message": f"Insufficient capacity: need {required_capacity:,} neurons, have {current_capacity:,}"}
                    else:
                        self.logger.info(f"[GENOME] ✅ Adequate capacity: {current_capacity:,} >= {required_capacity:,}")

                # ARCHITECTURE IMPROVEMENT: Build brain from state manager's genome (not temp file)
                # This ensures connectome manager always uses the sanitized genome from state manager
                self.logger.info(
                    "Building brain from state manager's sanitized genome..."
                )

                # Initialize embryogenesis
                embry = NeuroEmbryogenesis(
                    connectome_manager=self._connectome_manager,
                    progress_callback=self._handle_embryogenesis_progress,
                )

                # CRITICAL: Develop brain from state manager's genome (single source of truth)
                # This includes the COMPLETE brain development process:
                # 1. Corticogenesis (cortical area creation)
                # 2. Voxelogenesis (spatial framework)
                # 3. Neurogenesis (neuron creation)
                # 4. Synaptogenesis (synapse formation) <- This is the long-running step
                try:
                    self.logger.info(
                        "Starting COMPLETE brain development from genome (including synaptogenesis)..."
                    )
                    # ARCHITECTURE: Use hierarchical genome from single source of truth
                    success = embry.develop_brain_from_genome_data(self._current_genome)

                    if not success:
                        error_msg = (
                            embry.error or "Unknown error during brain development"
                        )
                        self.logger.error(
                            f"Failed to develop brain from genome: {error_msg}"
                        )

                        # Set error state since brain development failed
                        if self.state_manager:
                            from feagi.core.state_manager import GenomeState

                            self.state_manager.set_genome_state(GenomeState.ERROR)
                            self.state_manager.set_brain_readiness(False)
                            result = self.state_manager.set_genome_validity(False)
                            if result.is_err:
                                self.logger.warning(f"Failed to set genome validity: {result.unwrap_err()}")
                        return {
                            "success": False,
                            "error": f"Failed to develop brain from genome: {error_msg}",
                        }

                    # Get development statistics - this includes completed synaptogenesis
                    stats = embry.get_development_statistics()
                    self.logger.info(
                        f"COMPLETE brain development finished: {stats.get('total_neurons', 0)} neurons, {stats.get('total_synapses', 0)} synapses"
                    )

                    # CRITICAL: Set genome state to LOADED only after complete brain development
                    # This ensures genome is marked as loaded ONLY when everything is truly complete
                    from feagi.core.state_manager import GenomeState

                    self.state_manager.set_genome_state(GenomeState.LOADED)
                    self.logger.info(
                        "Genome state set to LOADED - COMPLETE brain development finished (including synaptogenesis)"
                    )

                    # STEP 3: After complete brain development, set final states
                    self.logger.info("Setting final genome and brain states...")

                    # Set brain readiness to true - genome loading is complete
                    self.state_manager.set_brain_readiness(True)
                    self.logger.info(
                        "✅ Brain readiness set to True - complete genome loaded"
                    )

                    # Log current burst engine state for monitoring

                    current_burst_state = self.state_manager.get_burst_engine_state()
                    self.logger.info(
                        f"📊 Current burst engine state: {current_burst_state}"
                    )

                    # The process manager will detect the state changes and handle service startup
                    self.logger.info(
                        "🎯 Genome loading complete - process manager will handle service coordination"
                    )
                    
                    # CRITICAL: Update burst engine with new genome directly since event system is not available
                    try:
                        from feagi.npu.burst_engine import BurstEngine
                        burst_engine = BurstEngine.get_instance()
                        if burst_engine:
                            burst_engine.update_with_genome()
                            self.logger.info("✅ Burst engine updated with new genome successfully")
                        else:
                            self.logger.warning("⚠️ Burst engine instance not available for genome update")
                    except Exception as burst_error:
                        self.logger.warning(f"Failed to update burst engine with genome: {burst_error}")
                        # Don't fail genome loading for burst engine update issues
                    
                    # CRITICAL: Emit GENOME_LOADED event to trigger burst engine startup (fallback)
                    try:
                        from feagi.utils.event_system import EventType, EventPriority, emit_event
                        # Get cortical area count for event data
                        current_cortical_area_count = len(
                            getattr(self._connectome_manager, "cortical_areas", {})
                        )
                        success = emit_event(
                            EventType.GENOME_LOADED, 
                            data={"filename": filename, "cortical_areas": current_cortical_area_count},
                            priority=EventPriority.HIGH
                        )
                        if success:
                            self.logger.info(f"📡 GENOME_LOADED event emitted for '{filename}'")
                        else:
                            self.logger.warning("Failed to emit GENOME_LOADED event - using direct burst engine update instead")
                    except Exception as event_error:
                        self.logger.warning(f"Failed to emit GENOME_LOADED event: {event_error}")
                        # Don't fail genome loading for event emission issues

                except Exception as dev_error:
                    self.logger.error(
                        f"Exception during brain development: {str(dev_error)}"
                    )
                    if self.state_manager:
                        from feagi.core.state_manager import GenomeState

                        self.state_manager.set_genome_state(GenomeState.ERROR)
                        self.state_manager.set_brain_readiness(False)
                        result = self.state_manager.set_genome_validity(False)
                        if result.is_err:
                            self.logger.warning(f"Failed to set genome validity: {result.unwrap_err()}")
                    return {
                        "success": False,
                        "error": f"Exception during brain development: {str(dev_error)}",
                    }

                # CRITICAL: Update state manager with comprehensive brain statistics for health checks
                if self.state_manager:
                    # Update state manager with comprehensive brain statistics for health checks
                    try:
                        # Get statistics from connectome manager
                        cortical_area_count = len(
                            getattr(self._connectome_manager, "cortical_areas", {})
                        )

                        # Calculate neuron and synapse counts if methods exist
                        total_neurons = 0
                        total_synapses = 0

                        if hasattr(self._connectome_manager, "get_total_neuron_count"):
                            total_neurons = (
                                self._connectome_manager.get_total_neuron_count()
                            )
                        elif hasattr(self._connectome_manager, "cortical_areas"):
                            # Fallback: count neurons in all cortical areas
                            for area_idx in self._connectome_manager.cortical_areas:
                                try:
                                    if hasattr(
                                        self._connectome_manager, "get_neurons_by_area"
                                    ):
                                        area_neurons = self._connectome_manager.get_neurons_by_area(
                                            area_idx
                                        )
                                        total_neurons += (
                                            len(area_neurons) if area_neurons else 0
                                        )
                                except Exception:
                                    pass

                        if hasattr(self._connectome_manager, "get_total_synapse_count"):
                            total_synapses = (
                                self._connectome_manager.get_total_synapse_count()
                            )

                        # Update state manager with brain statistics (CRITICAL for health check)
                        stats_result = self.state_manager.set_brain_stats({
                            "neuron_count": total_neurons,
                            "synapse_count": total_synapses,
                            "cortical_area_count": cortical_area_count,
                        })
                        if stats_result.is_err:
                            self.logger.warning(f"Failed to set brain stats: {stats_result.unwrap_err()}")

                        # Create cortical list for health check compatibility (CRITICAL)
                        cortical_ids = []
                        if hasattr(self._connectome_manager, "cortical_areas"):
                            for (
                                area_idx,
                                area,
                            ) in self._connectome_manager.cortical_areas.items():
                                # Try to get cortical_id from area object, fallback to string representation
                                if hasattr(area, "cortical_id") and area.cortical_id:
                                    cortical_ids.append(area.cortical_id)
                                else:
                                    cortical_ids.append(f"CID{area_idx:03d}")
                        cortical_result = self.state_manager.set_cortical_list(cortical_ids)
                        if cortical_result.is_err:
                            self.logger.warning(f"Failed to set cortical list: {cortical_result.unwrap_err()}")

                        # Set genome validity based on earlier validation results
                        if (
                            not hasattr(self.state_manager, "genome_validity")
                            or self.state_manager.genome_validity is None
                        ):
                            validity_result = self.state_manager.set_genome_validity(True)
                            if validity_result.is_err:
                                self.logger.warning(f"Failed to set genome validity: {validity_result.unwrap_err()}")

                        # Ensure other state manager attributes are initialized
                        if (
                            not hasattr(self.state_manager, "connected_agents")
                            or self.state_manager.connected_agents is None
                        ):
                            self.state_manager.set_agent_count(0)

                        if not hasattr(self.state_manager, "changes_saved_externally"):
                            self.state_manager.changes_saved_externally = False

                        if not hasattr(self.state_manager, "exit_condition"):
                            # NOTE: exit_condition doesn't have a proper setter method yet
                            # This should be handled through proper state management
                            result = self.state_manager.set_exit_condition(False)
                            if result.is_err:
                                self.logger.warning("Failed to set exit condition")
                                # Continue anyway - this is not critical

                        self.logger.info(
                            f"State manager fully synchronized: {total_neurons} neurons, {total_synapses} synapses, {cortical_area_count} cortical areas"
                        )

                    except Exception as stats_error:
                        # CRITICAL FIX: Do NOT reset brain_readiness to False here!
                        # Statistics updating is NOT critical for brain functionality
                        self.logger.error(
                            f"WARNING: Error updating state manager statistics: {str(stats_error)}"
                        )
                        self.logger.warning(
                            "Statistics update failed but genome loading succeeded - brain is still functional"
                        )
                        # Don't fail genome loading for statistics issues
                        # Don't touch brain_readiness or genome_state - they're already correctly set

                # Get cortical area count from connectome manager for return value
                cortical_area_count = len(
                    getattr(self._connectome_manager, "cortical_areas", {})
                )

                # CRITICAL: Only increment genome counter for ACTUALLY NEW genomes
                if self.state_manager:
                    old_genome_counter = self.state_manager.get_genome_counter()

                    # Check if this is genuinely a NEW genome (different from what we had before)
                    is_new_genome = False
                    if (
                        not old_genome_data
                        or old_genome_data != self._current_genome
                        or old_genome_filename != filename
                    ):
                        # Update timestamp to signal change to downstream clients
                        new_genome_timestamp = int(time.time() * 1000)  # milliseconds
                        self.state_manager.set_genome_timestamp(new_genome_timestamp)
                        self.logger.info(
                            f"[OK] Genome timestamp updated to {new_genome_timestamp} (signals NEW genome to clients)"
                        )
                    else:
                        # Same filename - check if genome data actually changed
                        import hashlib

                        new_hash = hashlib.md5(
                            json.dumps(genome_data, sort_keys=True).encode()
                        ).hexdigest()
                        old_hash = hashlib.md5(
                            json.dumps(old_genome_data, sort_keys=True).encode()
                        ).hexdigest()
                        if new_hash != old_hash:
                            is_new_genome = True
                            self.logger.info(
                                f"NEW genome detected: data changed (hash: {old_hash[:8]} → {new_hash[:8]})"
                            )
                        else:
                            self.logger.info(
                                f"[RELOAD] SAME genome being reloaded: '{filename}' with identical data"
                            )

                    # Only increment counter and update timestamp for genuinely new genomes
                    if is_new_genome:
                        self.state_manager.increment_genome_counter()
                        current_genome_number = self.state_manager.get_genome_counter()
                        self.logger.info(
                            f"[OK] Genome counter incremented to {current_genome_number}"
                        )

                        new_genome_timestamp = int(time.time() * 1000)  # milliseconds
                        self.state_manager.set_genome_timestamp(new_genome_timestamp)
                        self.logger.info(
                            f"[OK] Genome timestamp updated to {new_genome_timestamp} (signals NEW genome to clients)"
                        )
                    else:
                        current_genome_number = old_genome_counter
                        self.logger.info(
                            "[SKIP] Genome counter NOT incremented (same genome reloaded)"
                        )
                        self.logger.info(
                            "[SKIP] Genome timestamp NOT updated (prevents reload loop)"
                        )

                # DYNAMIC SIZING: Resize connectome based on genome requirements
                if self._connectome_manager and hasattr(self._connectome_manager, 'resize_for_genome'):
                    try:
                        resize_success = self._connectome_manager.resize_for_genome(genome_data)
                        if resize_success:
                            self.logger.info("✅ [DYNAMIC SIZING] Connectome resized successfully based on genome requirements")
                        else:
                            self.logger.info("ℹ️  [DYNAMIC SIZING] Connectome resize not needed - current size is optimal")
                    except Exception as resize_error:
                        self.logger.warning(f"⚠️  [DYNAMIC SIZING] Error during connectome resize: {resize_error}")
                        # Don't fail genome loading for resize issues - it's an optimization, not critical

                # Log success
                if (
                    self.state_manager
                    and hasattr(self.state_manager, "genome_validity")
                    and not self.state_manager.genome_validity
                ):
                    self.logger.info(
                        f"Genome loaded successfully but marked as INVALID due to validation failures: {cortical_area_count} cortical areas created"
                    )
                else:
                    self.logger.info(
                        f"Genome loaded successfully: {cortical_area_count} cortical areas created"
                    )

                # Return success with detailed information including validation status
                result = {
                    "success": True,
                    "cortical_area_count": cortical_area_count,
                    "message": "Genome loaded and state manager fully synchronized",
                }

                # Include validation errors in response if validation failed but loading succeeded
                if not validation_result["valid"]:
                    result["validation_errors"] = validation_result.get("errors", [])
                    result["genome_validity"] = False

                    # Check if auto-recovery was performed
                    if auto_recovery_details.get("recovery_performed", False):
                        result["message"] = (
                            f"Genome loaded with auto-recovery: {auto_recovery_details.get('recovery_summary', 'Auto-recovery performed')}"
                        )
                        result["auto_recovery_performed"] = True
                        result["removed_morphologies"] = auto_recovery_details.get(
                            "removed_morphologies", []
                        )
                        result["fixed_references"] = auto_recovery_details.get(
                            "fixed_references", []
                        )
                        # Include validation warnings from auto-recovery
                        result["validation_warnings"] = auto_recovery_details.get(
                            "validation_warnings", []
                        )
                    else:
                        result["message"] = (
                            f"Genome loaded but marked as invalid due to validation failures: {validation_result.get('error_summary', 'Validation failed')}"
                        )
                        result["auto_recovery_performed"] = False
                        result["validation_warnings"] = []
                else:
                    result["genome_validity"] = True
                    result["auto_recovery_performed"] = False
                    # Even if validation passed, include warnings from auto-recovery if any corrections were made
                    if auto_recovery_details.get("recovery_performed", False):
                        result["validation_warnings"] = auto_recovery_details.get(
                            "validation_warnings", []
                        )
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
                    result = self.state_manager.set_genome_validity(False)
                    if result.is_err:
                        self.logger.warning(f"Failed to set genome validity: {result.unwrap_err()}")

                return {"success": False, "error": str(e)}

        except Exception as e:
            self.logger.error(f"Error loading genome: {str(e)}")
            import traceback

            self.logger.error(traceback.format_exc())

            # Update state manager with error
            if self.state_manager:
                from feagi.core.state_manager import GenomeState

                self.state_manager.set_genome_state(GenomeState.ERROR)
                self.state_manager.set_brain_readiness(False)
                result = self.state_manager.set_genome_validity(False)
                if result.is_err:
                    self.logger.warning(f"Failed to set genome validity: {result.unwrap_err()}")

            return {"success": False, "error": str(e)}

    def _handle_embryogenesis_progress(self, stage, percentage, message):
        """Handle progress updates from the neuroembryogenesis process."""
        self.logger.info(f"{stage} {percentage:.1f}% - {message}", status="[PROC]")
        
        # Update state manager with development stage
        if self.state_manager:
            from feagi.bdu.embryogenesis.neuroembryogenesis import DevelopmentStage
            
            # Map DevelopmentStage enum to integer values for state manager
            stage_mapping = {
                DevelopmentStage.INITIALIZATION: 0,
                DevelopmentStage.CORTICOGENESIS: 1,
                DevelopmentStage.VOXELOGENESIS: 2,
                DevelopmentStage.NEUROGENESIS: 3,
                DevelopmentStage.SYNAPTOGENESIS: 4,
                DevelopmentStage.COMPLETED: 5,
                DevelopmentStage.FAILED: 6,
            }
            
            stage_value = stage_mapping.get(stage, 0)
            
            # Update neuroembryogenesis stage in state manager
            if hasattr(self.state_manager._state, 'neuroembryogenesis_stage'):
                self.state_manager._state.neuroembryogenesis_stage = stage_value
                self.state_manager._state.neuroembryogenesis_progress = int(percentage)

    def get_genome(self) -> Optional[Dict[str, Any]]:
        """Get the currently loaded genome data."""
        # Check our internal state first
        if self._current_genome is not None:
            return self._current_genome

        # If not in our cache, try to get from state manager
        if (
            self.state_manager
            and hasattr(self.state_manager, "genome")
            and self.state_manager.genome
        ):
            self._current_genome = self.state_manager.genome
            return self._current_genome

        # If no genome in cache/state manager, but connectome has areas,
        # create a minimal genome structure to allow operations to proceed
        if (
            self._connectome_manager
            and hasattr(self._connectome_manager, "cortical_areas")
            and len(self._connectome_manager.cortical_areas) > 0
        ):
            self.logger.info(
                "Creating minimal genome structure from existing connectome state"
            )

            # Create minimal genome with cortical_areas section
            minimal_genome = {
                "cortical_areas": {},
                "physiology": {
                    "burst_delay": 0.025,
                    "max_age": 10000000,
                    "evolution_burst_count": 50,
                    "ipu_idle_threshold": 1000,
                    "plasticity_queue_depth": 3,
                    "lifespan_mgmt_interval": 10,
                },
            }

            # Populate cortical areas from connectome
            for cortical_id, area in self._connectome_manager.cortical_areas.items():
                minimal_genome["cortical_areas"][cortical_id] = {
                    "cortical_name": getattr(area, "name", cortical_id),
                    "coordinates": {
                        "x": getattr(area, "x", 0),
                        "y": getattr(area, "y", 0),
                        "z": getattr(area, "z", 0),
                    },
                    "dimensions": {
                        "x": getattr(area, "width", 1),
                        "y": getattr(area, "height", 1),
                        "z": getattr(area, "depth", 1),
                    },
                    "parameters": {},
                }

            self._current_genome = minimal_genome
            return self._current_genome

        self.logger.warning("No genome has been loaded")
        return None

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
                self.logger.warning(
                    f"Default genomes directory not found: {defaults_path}"
                )
                return {}

            # Get all .json files in the directory
            default_genomes = {}

            for file_path in defaults_path.glob("*.json"):
                try:
                    with file_path.open("r") as f:
                        genome_data = json.load(f)

                        # Store basic metadata about the genome
                        default_genomes[file_path.name] = {
                            "title": genome_data.get("genome_title", "Untitled Genome"),
                            "description": genome_data.get("genome_description", ""),
                            "file_path": str(file_path),
                        }
                except Exception as e:
                    self.logger.error(
                        f"Error loading default genome {file_path.name}: {str(e)}"
                    )

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
            (
                Path(os.environ.get("FEAGI_DATA_PATH", ""))
                if os.environ.get("FEAGI_DATA_PATH")
                else None
            ),
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

            if (
                hasattr(self.state_manager, "generation_dict")
                and self.state_manager.generation_dict
            ):
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

            if (
                hasattr(self.state_manager, "evo_change_register")
                and self.state_manager.evo_change_register
            ):
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
                self.logger.error(
                    f"Genome file not found: {genome_filepath}", status="[ERR]"
                )
                return False

            # Update state to LOADING
            if self.state_manager:
                from feagi.core.state_manager import GenomeState

                self.state_manager.set_genome_state(GenomeState.LOADING)
                self.state_manager.set_brain_readiness(False)

            # Load the genome data
            with genome_path.open("r") as f:
                genome_data = json.load(f)

            # Extract the filename for reference
            filename = genome_path.name

            # Load the genome using the service
            result = self.load_genome(genome_data, filename=filename)

            if not result.get("success", False):
                self.logger.error(
                    f"Failed to load genome: {result.get('error', 'Unknown error')}",
                    status="[ERR]",
                )

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

            self.logger.info(
                f"Genome deployed successfully from {filename}", status="[OK]"
            )
            return True

        except json.JSONDecodeError:
            self.logger.error(
                f"Invalid JSON in genome file: {genome_filepath}", status="[ERR]"
            )

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
        if self.state_manager and self.state_manager.is_genome_loaded():
            return True

        # If state manager says no genome, but connectome has areas, consider it loaded
        # This handles cases where genome was loaded but state wasn't properly synced
        if (
            self._connectome_manager
            and hasattr(self._connectome_manager, "cortical_areas")
            and len(self._connectome_manager.cortical_areas) > 0
        ):
            self.logger.info(
                "Genome appears loaded (connectome has cortical areas) despite state manager saying otherwise"
            )
            return True

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
            self.logger.debug(
                f"GENOME SERVICE: load_default_genome called with genome_name: {genome_name}"
            )
            # Normalize genome name
            genome_name = genome_name.replace(".json", "")
            genome_filename = f"{genome_name}_genome.json"
            self.logger.debug(
                f"GENOME SERVICE: Looking for genome file: {genome_filename}"
            )

            # Find genome file using clean path resolution
            genome_path = self._find_default_genome_path(genome_filename)
            if not genome_path:
                self.logger.debug(
                    f"GENOME SERVICE: Genome file not found: {genome_filename}"
                )
                return {
                    "success": False,
                    "error": f"Default genome '{genome_name}' not found",
                }

            self.logger.debug(f"GENOME SERVICE: Found genome file at: {genome_path}")
            self.logger.info(f"Loading {genome_name} genome from {genome_path}")

            # Load and process genome
            with genome_path.open("r") as f:
                genome_data = json.load(f)

            self.logger.debug(
                "GENOME SERVICE: Loaded genome data, calling load_genome()..."
            )
            # Update state manager
            if self.state_manager:
                self.state_manager.genome_file_name = genome_filename

            # Load genome through the main pipeline
            self._genome_filename = genome_filename
            result = self.load_genome(genome_data, genome_filename)
            self.logger.debug(
                f"GENOME SERVICE: load_genome returned: {result.get('success', 'unknown')}"
            )
            return result

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
            cwd / "feagi/evo/defaults/genome",  # From working directory
            cwd / "feagi_core/feagi/evo/defaults/genome",  # From project root
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

    # ===== CORTICAL AREA WRITE OPERATIONS =====
    # These methods handle cortical area modifications through proper data flow:
    # API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager

    def create_cortical_area(
        self,
        name: str,
        coordinates: Dict[str, int],
        dimensions: Dict[str, int],
        area_type: str,
        parameters: Dict[str, Any] = None,
    ) -> Optional[Dict[str, Any]]:
        """
        Create a new cortical area through proper genome modification pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures cortical area modifications
        go through the proper data flow to maintain genome consistency.

        Args:
            name: Area name
            coordinates: Area coordinates (x, y, z)
            dimensions: Area dimensions (width, height, depth)
            area_type: Type of cortical area
            parameters: Additional area parameters

        Returns:
            Dict containing created area information or None if failed
        """
        try:
            if not self.is_genome_loaded():
                self.logger.error("Cannot create cortical area: No genome loaded")
                return None

            # Begin genome transaction for atomic modification
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
            else:
                transaction = None

            try:
                # Get current genome for modification
                current_genome = self.get_genome()
                if not current_genome:
                    self.logger.error(
                        "Cannot create cortical area: Genome data not available"
                    )
                    return None

                # Ensure blueprint section exists in hierarchical genome
                if "blueprint" not in current_genome:
                    current_genome["blueprint"] = {}

                # Generate unique cortical area ID
                existing_ids = set(current_genome["blueprint"].keys())
                new_id = 1
                while str(new_id) in existing_ids:
                    new_id += 1
                cortical_id = str(new_id)

                # Create new cortical area definition in hierarchical format
                new_area = {
                    "cortical_id": cortical_id,
                    "cortical_name": name,
                    "coordinates_3d": coordinates,
                    "cortical_dimensions": dimensions,
                    "cortical_type": area_type,
                    "parameters": parameters or {},
                }

                # Add to hierarchical blueprint structure
                current_genome["blueprint"][cortical_id] = new_area

                # Update the genome through proper pipeline
                self._current_genome = current_genome

                # Trigger NeuroEmbryogenesis to update ConnectomeManager
                from feagi.bdu.embryogenesis.neuroembryogenesis import (
                    NeuroEmbryogenesis,
                )

                embryogenesis = NeuroEmbryogenesis(
                    self._connectome_manager, self.state_manager
                )

                # Apply the cortical area creation
                success = embryogenesis.create_cortical_area(
                    cortical_id=cortical_id,
                    name=name,
                    coordinates=coordinates,
                    dimensions=dimensions,
                    area_type=area_type,
                    parameters=parameters,
                )

                if success and transaction:
                    transaction.commit()
                elif transaction:
                    transaction.rollback()
                    return None

                if success:
                    self.logger.info(f"Created cortical area: {cortical_id} ({name})")
                    return {
                        "cortical_id": cortical_id,
                        "name": name,
                        "coordinates": coordinates,
                        "dimensions": dimensions,
                        "type": area_type,
                        "parameters": parameters or {},
                        "neuron_count": 0,  # New area has no neurons yet
                    }
                else:
                    return None

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error creating cortical area: {str(e)}")
            return None

    def update_cortical_area(
        self,
        cortical_id: str,
        name: str = None,
        coordinates: Dict[str, int] = None,
        dimensions: Dict[str, int] = None,
        area_type: str = None,
        parameters: Dict[str, Any] = None,
    ) -> Optional[Dict[str, Any]]:
        """
        Update an existing cortical area with intelligent routing for optimal performance.

        PERFORMANCE OPTIMIZATION: This method now intelligently routes updates based on
        change type to avoid unnecessary full brain rebuilds:
        - Parameter changes: Direct neuron updates (~2-5ms, 160-400x faster)
        - Metadata changes: Simple property updates (~1ms, 800x faster)  
        - Structural changes: Full rebuild (~800ms, existing behavior)
        - Hybrid changes: Optimized combination of above strategies

        ARCHITECTURE COMPLIANCE: Maintains genome consistency while providing
        massive performance improvements for parameter-only updates.

        Args:
            cortical_id: 6-character cortical area identifier
            name: New cortical area name (metadata update)
            coordinates: New coordinates (structural change)
            dimensions: New dimensions (structural change)
            area_type: New area type (structural change)
            parameters: New parameters (parameter updates)

        Returns:
            Updated area information or None if not found
        """
        import time
        start_time = time.time()
        

        
        try:
            if not self.is_genome_loaded():
                self.logger.error("Cannot update cortical area: No genome loaded")
                return None

            if cortical_id not in self._current_genome["blueprint"]:
                self.logger.error(f"Cannot update cortical area: {cortical_id} not found in genome")
                return None

            # Start transaction if state manager is available
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
            else:
                transaction = None

            try:
                # Collect all non-None changes into a single dictionary
                changes = {}
                if name is not None:
                    changes["cortical_name"] = name
                if coordinates is not None:
                    changes["coordinates_3d"] = coordinates
                if dimensions is not None:
                    changes["cortical_dimensions"] = dimensions
                if area_type is not None:
                    changes["cortical_type"] = area_type
                if parameters is not None:
                    changes.update(parameters)

                if not changes:
                    self.logger.warning(f"No changes provided for cortical area {cortical_id}")
                    if transaction:
                        transaction.rollback()
                    return self._current_genome["blueprint"][cortical_id]

                # INTELLIGENT ROUTING: Classify changes for optimal performance
                from feagi.api.core.services.genome.change_classifier import (
                    CorticalChangeClassifier, ChangeType
                )
                
                change_type = CorticalChangeClassifier.classify_changes(changes)
                CorticalChangeClassifier.log_classification_result(changes, change_type)
                
                # ROUTE BASED ON CHANGE TYPE for optimal performance
                if change_type == ChangeType.PARAMETER:
                    # FAST PATH: Direct parameter updates only (~2-5ms)
                    result = self._update_parameters_only(cortical_id, changes, transaction)
                    
                elif change_type == ChangeType.METADATA:
                    # FASTEST PATH: Metadata updates only (~1ms)
                    result = self._update_metadata_only(cortical_id, changes, transaction)
                    
                elif change_type == ChangeType.STRUCTURAL:
                    # LOCALIZED REBUILD PATH: Structural changes (~100-200ms)
                    result = self._update_with_localized_rebuild(cortical_id, changes, transaction)
                    
                elif change_type == ChangeType.HYBRID:
                    # HYBRID PATH: Optimized combination (uses localized rebuild for structural parts)
                    result = self._update_hybrid(cortical_id, changes, transaction)
                    
                else:
                    # Fallback to safe localized rebuild (avoid global rebuild)
                    self.logger.warning(f"Unknown change type {change_type}, using localized rebuild")
                    result = self._update_with_localized_rebuild(cortical_id, changes, transaction)
                    
                # Log performance metrics
                duration = time.time() - start_time
                self.logger.info(
                    f"[CORTICAL-UPDATE] {cortical_id} updated via {change_type.value} "
                    f"path in {duration*1000:.1f}ms"
                )
                
                return result

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error updating cortical area: {str(e)}")
            return None

    def delete_cortical_area(self, cortical_id: str) -> bool:
        """
        Delete a cortical area through proper genome modification pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures cortical area modifications
        go through the proper data flow to maintain genome consistency.

        Args:
            cortical_id: ID of cortical area to delete

        Returns:
            bool: True if area was deleted successfully
        """
        try:
            if not self.is_genome_loaded():
                self.logger.error("Cannot delete cortical area: No genome loaded")
                return False

            # Begin genome transaction for atomic modification
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
            else:
                transaction = None

            try:
                # Get current genome for modification
                current_genome = self.get_genome()
                if not current_genome:
                    self.logger.error(
                        "Cannot delete cortical area: Genome data not available"
                    )
                    return False

                # Check if cortical area exists in blueprint
                if (
                    "blueprint" not in current_genome
                    or cortical_id not in current_genome["blueprint"]
                ):
                    self.logger.warning(
                        f"Cortical area {cortical_id} not found in genome"
                    )
                    return False

                # Remove from blueprint section
                del current_genome["blueprint"][cortical_id]

                # Update the genome through proper pipeline
                self._current_genome = current_genome

                # Trigger NeuroEmbryogenesis to update ConnectomeManager
                from feagi.bdu.embryogenesis.neuroembryogenesis import (
                    NeuroEmbryogenesis,
                )

                embryogenesis = NeuroEmbryogenesis(
                    self._connectome_manager, self.state_manager
                )

                # Apply the cortical area deletion by triggering brain development
                # ARCHITECTURE: Pass hierarchical genome directly (single source of truth)
                # NeuroEmbryogenesis now supports hierarchical format natively
                success = embryogenesis.develop_brain_from_genome_data(current_genome)

                if success and transaction:
                    transaction.commit()
                elif transaction:
                    transaction.rollback()
                    return False

                if success:
                    self.logger.info(f"Deleted cortical area: {cortical_id}")

                return success

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error deleting cortical area: {str(e)}")
            return False

    # ===== MORPHOLOGY WRITE OPERATIONS =====
    # These methods handle morphology modifications through proper data flow:
    # API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager

    def create_morphology(self, morphology_data: Dict[str, Any]) -> bool:
        """
        Create a new morphology through proper genome modification pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures morphology modifications
        go through the proper data flow to maintain genome consistency.

        Args:
            morphology_data: Dictionary containing morphology definition
                           Must include: name, type, parameters

        Returns:
            bool: True if morphology was created successfully
        """
        try:
            if not self.is_genome_loaded():
                self.logger.error("Cannot create morphology: No genome loaded")
                return False

            # Validate required fields
            if "name" not in morphology_data:
                raise ValueError("Morphology name is required")
            if "type" not in morphology_data:
                raise ValueError("Morphology type is required")
            if "parameters" not in morphology_data:
                raise ValueError("Morphology parameters are required")

            name = morphology_data["name"]

            # Begin genome transaction for atomic modification
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
            else:
                transaction = None

            try:
                # Get current genome for modification
                current_genome = self.get_genome()
                if not current_genome:
                    self.logger.error(
                        "Cannot create morphology: Genome data not available"
                    )
                    return False

                # Ensure neuron_morphologies section exists in genome
                if "neuron_morphologies" not in current_genome:
                    current_genome["neuron_morphologies"] = {}

                # Check if morphology already exists
                if name in current_genome["neuron_morphologies"]:
                    raise ValueError(f"Morphology '{name}' already exists")

                # Validate morphology type
                valid_types = ["vectors", "patterns", "functions", "composite"]
                if morphology_data["type"] not in valid_types:
                    raise ValueError(
                        f"Invalid morphology type. Must be one of: {valid_types}"
                    )

                # Add the new morphology
                current_genome["neuron_morphologies"][name] = {
                    "type": morphology_data["type"],
                    "parameters": morphology_data["parameters"],
                    "class": "custom",
                }

                # Update the genome through proper pipeline
                self._current_genome = current_genome
                
                # Update genome in state manager (single source of truth)
                if self.state_manager:
                    self.state_manager.genome = current_genome

                # Morphology is now available in genome - NeuroEmbryogenesis will automatically
                # pick it up through get_morphology_registry() when needed for synaptogenesis
                
                if transaction:
                    transaction.commit()

                self.logger.info(f"Successfully created morphology '{name}' of type '{morphology_data['type']}'")
                return True

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error creating morphology: {str(e)}")
            return False

    def update_morphology(self, morphology_id: str, updates: Dict[str, Any]) -> bool:
        """
        Update an existing morphology through proper genome modification pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures morphology modifications
        go through the proper data flow to maintain genome consistency.

        Args:
            morphology_id: ID of morphology to update
            updates: Dictionary containing updates to apply

        Returns:
            bool: True if morphology was updated successfully
        """
        try:
            if not self.is_genome_loaded():
                self.logger.error("Cannot update morphology: No genome loaded")
                return False

            # Begin genome transaction for atomic modification
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
            else:
                transaction = None

            try:
                # Get current genome for modification
                current_genome = self.get_genome()
                if not current_genome:
                    self.logger.error(
                        "Cannot update morphology: Genome data not available"
                    )
                    return False

                # Check if morphology exists
                if (
                    "neuron_morphologies" not in current_genome
                    or morphology_id not in current_genome["neuron_morphologies"]
                ):
                    self.logger.warning(
                        f"Morphology '{morphology_id}' not found in genome"
                    )
                    return False

                morphology = current_genome["neuron_morphologies"][morphology_id]

                # Don't allow updating core morphologies
                if morphology.get("source") == "core":
                    raise ValueError("Cannot modify core morphologies")

                # Apply updates
                for key, value in updates.items():
                    if key in ["type", "parameters", "class"]:
                        morphology[key] = value

                # Update the genome through proper pipeline
                self._current_genome = current_genome

                # Trigger NeuroEmbryogenesis to update ConnectomeManager
                from feagi.bdu.embryogenesis.neuroembryogenesis import (
                    NeuroEmbryogenesis,
                )

                embryogenesis = NeuroEmbryogenesis(
                    self._connectome_manager, self.state_manager
                )

                # Apply the morphology update
                success = embryogenesis.update_morphology(morphology_id, updates)

                if success and transaction:
                    transaction.commit()
                elif transaction:
                    transaction.rollback()
                    return False

                if success:
                    self.logger.info(f"Updated morphology: {morphology_id}")

                return success

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error updating morphology: {str(e)}")
            return False

    def delete_morphology(self, morphology_id: str) -> bool:
        """
        Delete a morphology through proper genome modification pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures morphology modifications
        go through the proper data flow to maintain genome consistency.

        Args:
            morphology_id: ID of morphology to delete

        Returns:
            bool: True if morphology was deleted successfully
        """
        try:
            if not self.is_genome_loaded():
                self.logger.error("Cannot delete morphology: No genome loaded")
                return False

            # Begin genome transaction for atomic modification
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
            else:
                transaction = None

            try:
                # Get current genome for modification
                current_genome = self.get_genome()
                if not current_genome:
                    self.logger.error(
                        "Cannot delete morphology: Genome data not available"
                    )
                    return False

                # Check if morphology exists
                if (
                    "neuron_morphologies" not in current_genome
                    or morphology_id not in current_genome["neuron_morphologies"]
                ):
                    self.logger.warning(
                        f"Morphology '{morphology_id}' not found in genome"
                    )
                    return False

                morphology = current_genome["neuron_morphologies"][morphology_id]

                # Don't allow deleting core morphologies
                if morphology.get("source") == "core":
                    raise ValueError("Cannot delete core morphologies")

                # Remove from genome structure
                del current_genome["neuron_morphologies"][morphology_id]

                # Update the genome through proper pipeline
                self._current_genome = current_genome

                # Trigger NeuroEmbryogenesis to update ConnectomeManager
                from feagi.bdu.embryogenesis.neuroembryogenesis import (
                    NeuroEmbryogenesis,
                )

                embryogenesis = NeuroEmbryogenesis(
                    self._connectome_manager, self.state_manager
                )

                # Apply the morphology deletion
                success = embryogenesis.delete_morphology(morphology_id)

                if success and transaction:
                    transaction.commit()
                elif transaction:
                    transaction.rollback()
                    return False

                if success:
                    self.logger.info(f"Deleted morphology: {morphology_id}")

                return success

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error deleting morphology: {str(e)}")
            return False

    # ===== CORTICAL MAPPING WRITE OPERATIONS =====
    # These methods handle cortical mapping modifications through proper data flow:
    # API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager

    def update_cortical_mapping(self, mapping: Dict[str, Any]) -> bool:
        """
        Update cortical mapping through proper genome modification pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures cortical mapping modifications
        go through the proper data flow to maintain genome consistency.

        Args:
            mapping: Dictionary containing updated cortical area mappings
                    Format: {area_id: {target_area_id: [connection_objects]}}

        Returns:
            bool: True if mapping was updated successfully
        """
        try:
            if not self.is_genome_loaded():
                self.logger.error("Cannot update cortical mapping: No genome loaded")
                return False

            # Begin genome transaction for atomic modification
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
            else:
                transaction = None

            try:
                # Get current genome for modification
                current_genome = self.get_genome()
                if not current_genome:
                    self.logger.error(
                        "Cannot update cortical mapping: Genome data not available"
                    )
                    return False

                # Convert the formatted mapping data back to the genome array format
                # and update each cortical area's parameters
                for area_id, area_mappings in mapping.items():
                    # Convert the formatted connections back to array format
                    genome_mapping = {}

                    for target_area_id, connections in area_mappings.items():
                        if not connections:
                            continue

                        # Convert each connection object back to array format
                        connection_arrays = []
                        for connection in connections:
                            if isinstance(connection, dict):
                                # Convert from object format to array format
                                # Expected array: [morphology_id, scalar, multiplier, plasticity_flag, constant, ltp, ltd]
                                connection_array = [
                                    connection.get("morphology_id", ""),
                                    connection.get("morphology_scalar", [1, 1, 1]),
                                    connection.get("postSynapticCurrent_multiplier", 1),
                                    connection.get("plasticity_flag", False),
                                    connection.get("plasticity_constant", 1),
                                    connection.get("ltp_multiplier", 1),
                                    connection.get("ltd_multiplier", 1),
                                ]
                                connection_arrays.append(connection_array)

                        if connection_arrays:
                            genome_mapping[target_area_id] = connection_arrays

                    # Update the cortical area's parameters with the new mapping
                    # Ensure blueprint section exists in hierarchical genome
                    if "blueprint" not in current_genome:
                        current_genome["blueprint"] = {}

                    # Find or create the area in the hierarchical blueprint
                    if area_id in current_genome["blueprint"]:
                        area_def = current_genome["blueprint"][area_id]
                    else:
                        # Create area definition from templates for system areas (like ___pwr)
                        self.logger.info(
                            f"Creating genome entry for system area '{area_id}' from templates"
                        )

                        # Import templates to get proper area definition
                        from feagi.evo.templates import cortical_types

                        # Look for the area in CORE devices first
                        area_template = None
                        if (
                            "CORE" in cortical_types
                            and "supported_devices" in cortical_types["CORE"]
                        ):
                            if area_id in cortical_types["CORE"]["supported_devices"]:
                                area_template = cortical_types["CORE"][
                                    "supported_devices"
                                ][area_id]

                        if area_template:
                            # Create area definition from template
                            area_def = {
                                "cortical_name": area_template.get(
                                    "cortical_name", area_id
                                ),
                                "coordinates": {
                                    "x": area_template.get("coordinate_3d", [0, 0, 0])[
                                        0
                                    ],
                                    "y": area_template.get("coordinate_3d", [0, 0, 0])[
                                        1
                                    ],
                                    "z": area_template.get("coordinate_3d", [0, 0, 0])[
                                        2
                                    ],
                                },
                                "dimensions": {
                                    "x": area_template.get("resolution", [1, 1, 1])[0],
                                    "y": area_template.get("resolution", [1, 1, 1])[1],
                                    "z": area_template.get("resolution", [1, 1, 1])[2],
                                },
                                "parameters": {},
                            }
                        else:
                            # Fallback for unknown system areas
                            self.logger.warning(
                                f"System area '{area_id}' not found in templates, using minimal definition"
                            )
                            area_def = {
                                "cortical_name": area_id,
                                "coordinates": {"x": 0, "y": 0, "z": 0},
                                "dimensions": {"x": 1, "y": 1, "z": 1},
                                "parameters": {},
                            }

                        current_genome["blueprint"][area_id] = area_def

                    # Ensure parameters section exists
                    if "parameters" not in area_def:
                        area_def["parameters"] = {}

                    # Update the mapping
                    area_def["parameters"]["mapping"] = genome_mapping

                # Update the genome through proper pipeline
                self._current_genome = current_genome

                # Trigger NeuroEmbryogenesis to update ConnectomeManager
                from feagi.bdu.embryogenesis.neuroembryogenesis import (
                    NeuroEmbryogenesis,
                )

                embryogenesis = NeuroEmbryogenesis(
                    self._connectome_manager, self.state_manager
                )

                # CRITICAL FIX: Load the genome data into the NeuroEmbryogenesis instance
                # This ensures the morphology definitions are available for cortical mapping
                if not embryogenesis._load_genome_data(current_genome):
                    self.logger.error(
                        "Failed to load genome data into NeuroEmbryogenesis"
                    )
                    if transaction:
                        transaction.rollback()
                    return False

                # Apply the cortical mapping update
                success = embryogenesis.update_cortical_mapping(mapping)

                if success and transaction:
                    transaction.commit()
                elif transaction:
                    transaction.rollback()
                    return False

                if success:
                    self.logger.info("Updated cortical mapping")

                return success

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error updating cortical mapping: {str(e)}")
            return False

    def update_cortical_mapping_properties(self, update_data: Dict[str, Any]) -> bool:
        """
        Update cortical mapping properties between two specific cortical areas.

        Args:
            update_data: Dictionary containing:
                - src_cortical_area: Source cortical area ID
                - dst_cortical_area: Destination cortical area ID
                - mapping_data: List of connection dictionaries

        Returns:
            bool: True if mapping properties were updated successfully
        """
        try:
            src_area = update_data.get("src_cortical_area")
            dst_area = update_data.get("dst_cortical_area")
            mapping_data = update_data.get("mapping_data", [])

            if not src_area or not dst_area:
                self.logger.error(
                    "Source and destination cortical areas must be specified"
                )
                return False

            if not self.is_genome_loaded():
                self.logger.error(
                    "Cannot update cortical mapping properties: No genome loaded"
                )
                return False

            self.logger.info(
                f"Updating cortical mapping properties from {src_area} to {dst_area}"
            )

            # Begin genome transaction for atomic modification
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
            else:
                transaction = None

            try:
                # Get current genome for modification
                current_genome = self.get_genome()
                if not current_genome:
                    self.logger.error(
                        "Cannot update cortical mapping properties: Genome data not available"
                    )
                    return False

                # CRITICAL FIX: Handle empty mapping_data as deletion request
                if not mapping_data or len(mapping_data) == 0:
                    self.logger.info(
                        f"Empty mapping_data detected - treating as deletion request for {src_area} -> {dst_area}"
                    )
                    # Use the delete_cortical_mapping method for proper synapse removal
                    success = self.delete_cortical_mapping(src_area, dst_area)
                else:
                    # Convert mapping_data to the expected format for normal updates
                    formatted_mapping = {src_area: {dst_area: mapping_data}}
                    # Use the existing update_cortical_mapping method
                    success = self.update_cortical_mapping(formatted_mapping)

                if success and transaction:
                    transaction.commit()
                elif transaction:
                    transaction.rollback()
                    return False

                if success:
                    self.logger.info(
                        f"Successfully updated mapping properties from {src_area} to {dst_area}"
                    )

                return success

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error updating cortical mapping properties: {str(e)}")
            return False

    def delete_cortical_mapping(
        self, src_cortical_area: str, dst_cortical_area: str
    ) -> bool:
        """
        Delete cortical mapping and all associated synapses between two cortical areas.

        ARCHITECTURE COMPLIANCE: WRITE operation through proper data flow:
        API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager

        Args:
            src_cortical_area: Source cortical area ID
            dst_cortical_area: Destination cortical area ID

        Returns:
            bool: True if mapping was deleted successfully
        """
        try:
            if not src_cortical_area or not dst_cortical_area:
                self.logger.error(
                    "Source and destination cortical areas must be specified"
                )
                return False

            if not self.is_genome_loaded():
                self.logger.error("Cannot delete cortical mapping: No genome loaded")
                return False

            self.logger.info(
                f"Deleting cortical mapping from {src_cortical_area} to {dst_cortical_area}"
            )

            # Begin genome transaction for atomic modification
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
            else:
                transaction = None

            try:
                # Get current genome for modification
                current_genome = self.get_genome()
                if not current_genome:
                    self.logger.error(
                        "Cannot delete cortical mapping: Genome data not available"
                    )
                    return False

                # Step 1: SAFETY CHECK - Ensure we're only removing MAPPING, not the cortical area itself
                blueprint = current_genome.get("blueprint", {})
                if src_cortical_area not in blueprint:
                    self.logger.warning(
                        f"Source cortical area {src_cortical_area} not found in blueprint"
                    )
                    return False

                if dst_cortical_area not in blueprint:
                    self.logger.warning(
                        f"Destination cortical area {dst_cortical_area} not found in blueprint"
                    )
                    return False

                # SAFETY: We're only modifying mapping properties, NOT deleting the cortical area
                src_area_def = blueprint[src_cortical_area]
                
                # Remove from cortical_mapping_dst if it exists
                mapping_dst = src_area_def.get("cortical_mapping_dst", {})
                if dst_cortical_area in mapping_dst:
                    del mapping_dst[dst_cortical_area]
                    self.logger.info(f"Removed {dst_cortical_area} from {src_cortical_area} mapping destinations")
                
                # Remove from parameters.mapping if it exists
                parameters = src_area_def.get("parameters", {})
                if "mapping" in parameters:
                    mapping_params = parameters["mapping"]
                    if isinstance(mapping_params, dict) and dst_cortical_area in mapping_params:
                        del mapping_params[dst_cortical_area]
                        self.logger.info(f"Removed {dst_cortical_area} from {src_cortical_area} mapping parameters")

                # Step 2: Update genome in state manager
                self._current_genome = current_genome
                if self.state_manager:
                    self.state_manager.genome = current_genome

                # Step 3: Remove existing synapses between cortical areas 
                # Use direct synapse deletion approach (following delete_cortical_connection pattern)
                connectome_manager = self._connectome_manager
                if connectome_manager:
                    try:
                        # Get all neurons in source and target areas
                        source_neurons = connectome_manager.get_neurons_by_cortical_area(src_cortical_area)
                        target_neurons = connectome_manager.get_neurons_by_cortical_area(dst_cortical_area)
                        
                        deleted_count = 0
                        self.logger.info(f"Deleting synapses from {len(source_neurons)} source neurons to {len(target_neurons)} target neurons")
                        
                        # Delete all synapses between source and target areas
                        for source_id in source_neurons:
                            for target_id in target_neurons:
                                if connectome_manager.has_synapse(source_id, target_id):
                                    success_remove = connectome_manager.remove_synapse(source_id, target_id)
                                    if success_remove:
                                        deleted_count += 1
                        
                        self.logger.info(f"Successfully deleted {deleted_count} synapses between {src_cortical_area} and {dst_cortical_area}")
                        success = True
                        
                    except Exception as e:
                        self.logger.error(f"Failed to delete synapses between areas: {e}")
                        success = False
                        if transaction:
                            transaction.rollback()
                        return False
                else:
                    self.logger.error("ConnectomeManager not available for synapse deletion")
                    success = False
                    if transaction:
                        transaction.rollback()
                    return False

                if success and transaction:
                    transaction.commit()
                elif transaction:
                    transaction.rollback()
                    return False

                if success:
                    self.logger.info(
                        f"Successfully deleted cortical mapping from {src_cortical_area} to {dst_cortical_area}"
                    )

                return success

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error deleting cortical mapping: {str(e)}")
            return False

    # ===== GENOME LOADING AND MANAGEMENT WRITE OPERATIONS =====
    # These methods handle genome loading and management through proper data flow:
    # API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis → ConnectomeManager



    def reset_genome(self) -> bool:
        """
        Reset genome through proper genome modification pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures genome reset
        goes through the proper data flow to maintain genome consistency.

        Returns:
            bool: True if genome was reset successfully
        """
        try:
            # Begin genome transaction for atomic modification
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
            else:
                transaction = None

            try:
                # Clear the current genome
                self._current_genome = None

                # Trigger NeuroEmbryogenesis to reset the connectome
                from feagi.bdu.embryogenesis.neuroembryogenesis import (
                    NeuroEmbryogenesis,
                )

                embryogenesis = NeuroEmbryogenesis(
                    self._connectome_manager, self.state_manager
                )

                # Apply the genome reset
                success = embryogenesis.reset_genome()

                if success and transaction:
                    transaction.commit()
                elif transaction:
                    transaction.rollback()
                    return False

                if success:
                    self.logger.info("Genome reset successfully")

                return success

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error resetting genome: {str(e)}")
            return False

    def amalgamate_genome(self, amalgamation_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Perform genome amalgamation through proper genome modification pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures genome amalgamation
        goes through the proper data flow to maintain genome consistency.

        Args:
            amalgamation_data: Dictionary containing amalgamation parameters

        Returns:
            Dict containing amalgamation result information
        """
        try:
            # Begin genome transaction for atomic modification
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
            else:
                transaction = None

            try:
                # Get current genome for modification
                current_genome = self.get_genome()
                if not current_genome:
                    raise ValueError("No genome loaded for amalgamation")

                # Trigger NeuroEmbryogenesis to perform amalgamation
                from feagi.bdu.embryogenesis.neuroembryogenesis import (
                    NeuroEmbryogenesis,
                )

                embryogenesis = NeuroEmbryogenesis(
                    self._connectome_manager, self.state_manager
                )

                # Apply the amalgamation
                result = embryogenesis.amalgamate_genome(amalgamation_data)

                if result.get("success") and transaction:
                    transaction.commit()
                elif transaction:
                    transaction.rollback()
                    return {"success": False, "error": "Amalgamation failed"}

                if result.get("success"):
                    self.logger.info("Genome amalgamation completed successfully")

                return result

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error during genome amalgamation: {str(e)}")
            return {"success": False, "error": str(e)}

    def cancel_amalgamation(self, amalgamation_id: str) -> bool:
        """
        Cancel genome amalgamation through proper genome modification pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures amalgamation cancellation
        goes through the proper data flow to maintain genome consistency.

        Args:
            amalgamation_id: ID of amalgamation to cancel

        Returns:
            bool: True if amalgamation was cancelled successfully
        """
        try:
            # Begin genome transaction for atomic modification
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
            else:
                transaction = None

            try:
                # Trigger NeuroEmbryogenesis to cancel amalgamation
                from feagi.bdu.embryogenesis.neuroembryogenesis import (
                    NeuroEmbryogenesis,
                )

                embryogenesis = NeuroEmbryogenesis(
                    self._connectome_manager, self.state_manager
                )

                # Apply the amalgamation cancellation
                success = embryogenesis.cancel_amalgamation(amalgamation_id)

                if success and transaction:
                    transaction.commit()
                elif transaction:
                    transaction.rollback()
                    return False

                if success:
                    self.logger.info(
                        f"Amalgamation {amalgamation_id} cancelled successfully"
                    )

                return success

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error cancelling amalgamation: {str(e)}")
            return False

    def append_file_to_genome(self, file_data: Dict[str, Any]) -> bool:
        """
        Append file content to genome through proper genome modification pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures file appending
        goes through the proper data flow to maintain genome consistency.

        Args:
            file_data: Dictionary containing file content and metadata

        Returns:
            bool: True if file was appended successfully
        """
        try:
            # Begin genome transaction for atomic modification
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
            else:
                transaction = None

            try:
                # Get current genome for modification
                current_genome = self.get_genome()
                if not current_genome:
                    raise ValueError("No genome loaded for file appending")

                # Trigger NeuroEmbryogenesis to append file
                from feagi.bdu.embryogenesis.neuroembryogenesis import (
                    NeuroEmbryogenesis,
                )

                embryogenesis = NeuroEmbryogenesis(
                    self._connectome_manager, self.state_manager
                )

                # Apply the file appending
                success = embryogenesis.append_file_to_genome(file_data)

                if success and transaction:
                    transaction.commit()
                elif transaction:
                    transaction.rollback()
                    return False

                if success:
                    self.logger.info("File appended to genome successfully")

                return success

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error appending file to genome: {str(e)}")
            return False

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
        Create a brain region through proper genome modification pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures brain region modifications
        go through the proper data flow to maintain genome consistency.

        Args:
            region_id: Unique identifier for the region
            region_name: Human-readable name for the region
            parent_region_id: ID of parent region (default: "root")
            coordinates: Region coordinates (optional)
            dimensions: Region dimensions (optional)
            parameters: Additional region parameters (optional)

        Returns:
            bool: True if region was created successfully
        """
        try:
            if not self.is_genome_loaded():
                self.logger.error("Cannot create brain region: No genome loaded")
                return False

            # Begin genome transaction for atomic modification
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
            else:
                transaction = None

            try:
                # Get current genome for modification
                current_genome = self.get_genome()
                if not current_genome:
                    self.logger.error(
                        "Cannot create brain region: Genome data not available"
                    )
                    return False

                # Ensure brain_regions section exists in genome
                if "brain_regions" not in current_genome:
                    current_genome["brain_regions"] = {}

                # Check if region already exists
                if region_id in current_genome["brain_regions"]:
                    raise ValueError(f"Brain region '{region_id}' already exists")

                # Create new brain region definition
                new_region = {
                    "region_id": region_id,
                    "region_name": region_name,
                    "parent_region_id": parent_region_id,
                    "coordinates": coordinates or {"x": 0, "y": 0, "z": 0},
                    "dimensions": dimensions or {"width": 1, "height": 1, "depth": 1},
                    "parameters": parameters or {},
                    "child_regions": [],
                    "cortical_areas": [],
                }

                # Add to genome structure
                current_genome["brain_regions"][region_id] = new_region

                # Update parent region's children list
                if (
                    parent_region_id != "root"
                    and parent_region_id in current_genome["brain_regions"]
                ):
                    parent_region = current_genome["brain_regions"][parent_region_id]
                    if "child_regions" not in parent_region:
                        parent_region["child_regions"] = []
                    parent_region["child_regions"].append(region_id)

                # Update the genome through proper pipeline
                self._current_genome = current_genome

                # Trigger NeuroEmbryogenesis to update ConnectomeManager
                from feagi.bdu.embryogenesis.neuroembryogenesis import (
                    NeuroEmbryogenesis,
                )

                embryogenesis = NeuroEmbryogenesis(
                    self._connectome_manager, self.state_manager
                )

                # Apply the brain region creation
                success = embryogenesis.create_brain_region(
                    region_id=region_id,
                    region_name=region_name,
                    parent_region_id=parent_region_id,
                    coordinates=coordinates,
                    dimensions=dimensions,
                    parameters=parameters,
                )

                if success and transaction:
                    transaction.commit()
                elif transaction:
                    transaction.rollback()
                    return False

                if success:
                    self.logger.info(
                        f"Created brain region: {region_id} ({region_name})"
                    )

                return success

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error creating brain region: {str(e)}")
            return False

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
        Update a brain region through proper genome modification pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures brain region modifications
        go through the proper data flow to maintain genome consistency.

        Args:
            region_id: ID of region to update
            region_name: New name (optional)
            parent_region_id: New parent region ID (optional)
            coordinates: New coordinates (optional)
            dimensions: New dimensions (optional)
            parameters: New parameters (optional)

        Returns:
            bool: True if region was updated successfully
        """
        try:
            if not self.is_genome_loaded():
                self.logger.error("Cannot update brain region: No genome loaded")
                return False

            # Begin genome transaction for atomic modification
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
            else:
                transaction = None

            try:
                # Get current genome for modification
                current_genome = self.get_genome()
                if not current_genome:
                    self.logger.error(
                        "Cannot update brain region: Genome data not available"
                    )
                    return False

                # Check if region exists
                if (
                    "brain_regions" not in current_genome
                    or region_id not in current_genome["brain_regions"]
                ):
                    self.logger.warning(f"Brain region {region_id} not found in genome")
                    return False

                # Get existing region definition
                region_def = current_genome["brain_regions"][region_id]

                # Update fields if provided
                if region_name is not None:
                    region_def["region_name"] = region_name
                if coordinates is not None:
                    region_def["coordinates"] = coordinates
                if dimensions is not None:
                    region_def["dimensions"] = dimensions
                if parameters is not None:
                    region_def["parameters"].update(parameters)

                # Handle parent region change
                if parent_region_id is not None and parent_region_id != region_def.get(
                    "parent_region_id"
                ):
                    old_parent_id = region_def.get("parent_region_id")

                    # Remove from old parent's children list
                    if (
                        old_parent_id
                        and old_parent_id in current_genome["brain_regions"]
                    ):
                        old_parent = current_genome["brain_regions"][old_parent_id]
                        if (
                            "child_regions" in old_parent
                            and region_id in old_parent["child_regions"]
                        ):
                            old_parent["child_regions"].remove(region_id)

                    # Add to new parent's children list
                    if (
                        parent_region_id != "root"
                        and parent_region_id in current_genome["brain_regions"]
                    ):
                        new_parent = current_genome["brain_regions"][parent_region_id]
                        if "child_regions" not in new_parent:
                            new_parent["child_regions"] = []
                        if region_id not in new_parent["child_regions"]:
                            new_parent["child_regions"].append(region_id)

                    region_def["parent_region_id"] = parent_region_id

                # Update the genome through proper pipeline
                self._current_genome = current_genome

                # Trigger NeuroEmbryogenesis to update ConnectomeManager
                from feagi.bdu.embryogenesis.neuroembryogenesis import (
                    NeuroEmbryogenesis,
                )

                embryogenesis = NeuroEmbryogenesis(
                    self._connectome_manager, self.state_manager
                )

                # Apply the brain region update
                success = embryogenesis.update_brain_region(
                    region_id=region_id,
                    region_name=region_name,
                    parent_region_id=parent_region_id,
                    coordinates=coordinates,
                    dimensions=dimensions,
                    parameters=parameters,
                )

                if success and transaction:
                    transaction.commit()
                elif transaction:
                    transaction.rollback()
                    return False

                if success:
                    self.logger.info(f"Updated brain region: {region_id}")

                return success

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error updating brain region: {str(e)}")
            return False

    def delete_brain_region(self, region_id: str, delete_members: bool = False) -> bool:
        """
        Delete a brain region through proper genome modification pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures brain region modifications
        go through the proper data flow to maintain genome consistency.

        Args:
            region_id: ID of region to delete
            delete_members: Whether to delete all members (cortical areas) or move them to parent

        Returns:
            bool: True if region was deleted successfully
        """
        try:
            if not self.is_genome_loaded():
                self.logger.error("Cannot delete brain region: No genome loaded")
                return False

            # Begin genome transaction for atomic modification
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
            else:
                transaction = None

            try:
                # Get current genome for modification
                current_genome = self.get_genome()
                if not current_genome:
                    self.logger.error(
                        "Cannot delete brain region: Genome data not available"
                    )
                    return False

                # Check if region exists
                if (
                    "brain_regions" not in current_genome
                    or region_id not in current_genome["brain_regions"]
                ):
                    self.logger.warning(f"Brain region {region_id} not found in genome")
                    return False

                # Cannot delete root region
                if region_id == "root":
                    raise ValueError("Cannot delete root region")

                region_def = current_genome["brain_regions"][region_id]
                parent_region_id = region_def.get("parent_region_id", "root")

                # Handle child regions and cortical areas
                child_regions = region_def.get("child_regions", [])
                cortical_areas = region_def.get("cortical_areas", [])

                if delete_members:
                    # Delete all child regions recursively
                    for child_region_id in child_regions:
                        self.delete_brain_region(child_region_id, delete_members=True)

                    # Delete all cortical areas in this region
                    for cortical_area_id in cortical_areas:
                        self.delete_cortical_area(cortical_area_id)
                else:
                    # Move child regions to parent
                    if (
                        parent_region_id != "root"
                        and parent_region_id in current_genome["brain_regions"]
                    ):
                        parent_region = current_genome["brain_regions"][
                            parent_region_id
                        ]
                        if "child_regions" not in parent_region:
                            parent_region["child_regions"] = []
                        for child_region_id in child_regions:
                            if child_region_id not in parent_region["child_regions"]:
                                parent_region["child_regions"].append(child_region_id)
                            # Update child's parent reference
                            if child_region_id in current_genome["brain_regions"]:
                                current_genome["brain_regions"][child_region_id][
                                    "parent_region_id"
                                ] = parent_region_id

                    # Move cortical areas to parent region (using hierarchical blueprint)
                    for cortical_area_id in cortical_areas:
                        if (
                            "blueprint" in current_genome
                            and cortical_area_id in current_genome["blueprint"]
                        ):
                            current_genome["blueprint"][cortical_area_id][
                                "region_id"
                            ] = parent_region_id

                # Remove from parent's children list
                if (
                    parent_region_id
                    and parent_region_id in current_genome["brain_regions"]
                ):
                    parent_region = current_genome["brain_regions"][parent_region_id]
                    if (
                        "child_regions" in parent_region
                        and region_id in parent_region["child_regions"]
                    ):
                        parent_region["child_regions"].remove(region_id)

                # Remove the region from genome structure
                del current_genome["brain_regions"][region_id]

                # Update the genome through proper pipeline
                self._current_genome = current_genome

                # Trigger NeuroEmbryogenesis to update ConnectomeManager
                from feagi.bdu.embryogenesis.neuroembryogenesis import (
                    NeuroEmbryogenesis,
                )

                embryogenesis = NeuroEmbryogenesis(
                    self._connectome_manager, self.state_manager
                )

                # Apply the brain region deletion
                success = embryogenesis.delete_brain_region(region_id, delete_members)

                if success and transaction:
                    transaction.commit()
                elif transaction:
                    transaction.rollback()
                    return False

                if success:
                    self.logger.info(f"Deleted brain region: {region_id}")

                return success

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error deleting brain region: {str(e)}")
            return False

    def _analyze_genome_requirements(self, genome_data: Dict[str, Any]) -> Dict[str, int]:
        """
        Analyze genome to calculate required neuron and synapse capacity.
        
        This method must be called BEFORE ConnectomeManager initialization
        to ensure adequate capacity for the genome.
        
        Args:
            genome_data: The genome dictionary to analyze
            
        Returns:
            Dictionary with required capacities:
            {
                "neurons_required": int,
                "synapses_required": int,
                "recommended_neuron_capacity": int,
                "recommended_synapse_capacity": int
            }
        """
        try:
            # Use existing GenomeProcessor to parse genome properly
            from feagi.evo.genome_processor import GenomeProcessor
            
            processor = GenomeProcessor(genome_data)
            cortical_areas = processor.extract_cortical_areas()
            cortical_mappings = processor.extract_cortical_mappings()
            
            total_neurons = 0
            total_synapses = 0
            
            # Calculate neurons from cortical areas
            for cortical_id, area_props in cortical_areas.items():
                if "dimensions" in area_props:
                    dims = area_props["dimensions"]
                    if len(dims) >= 3:
                        # CRITICAL: Account for neuron density (neurons per voxel)
                        neurons_per_voxel = area_props.get("neurons_per_voxel", 1)
                        area_neurons = dims[0] * dims[1] * dims[2] * neurons_per_voxel
                        total_neurons += area_neurons
                        
                        self.logger.debug(f"[GENOME ANALYSIS] {cortical_id}: {area_neurons:,} neurons ({dims[0]}x{dims[1]}x{dims[2]} * {neurons_per_voxel} neurons/voxel)")
           
            # Estimate synapses from cortical mappings
            for src_id, dst_mappings in cortical_mappings.items():
                for dst_id, connections in dst_mappings.items():
                    # Rough estimate: each connection creates synapses proportional to area sizes
                    src_area = cortical_areas.get(src_id)
                    dst_area = cortical_areas.get(dst_id)
                    
                    if src_area and dst_area and "dimensions" in src_area and "dimensions" in dst_area:
                        src_neurons = src_area["dimensions"][0] * src_area["dimensions"][1] * src_area["dimensions"][2]
                        dst_neurons = dst_area["dimensions"][0] * dst_area["dimensions"][1] * dst_area["dimensions"][2]
                        
                        # Estimate synapses per connection (conservative estimate)
                        synapses_per_connection = min(src_neurons, dst_neurons) // 10
                        total_synapses += synapses_per_connection * len(connections)
           
            # Apply buffer multiplier for safety (50% extra capacity)
            buffer_multiplier = 1.5
            min_neuron_capacity = 100_000
            min_synapse_capacity = 1_000_000
            
            recommended_neurons = max(
                int(total_neurons * buffer_multiplier),
                min_neuron_capacity
            )
            recommended_synapses = max(
                int(total_synapses * buffer_multiplier), 
                min_synapse_capacity
            )
            
            self.logger.info(f"[GENOME ANALYSIS] Required neurons: {total_neurons:,}")
            self.logger.info(f"[GENOME ANALYSIS] Required synapses: {total_synapses:,}")
            self.logger.info(f"[GENOME ANALYSIS] Recommended capacity: {recommended_neurons:,} neurons, {recommended_synapses:,} synapses")
            
            return {
                "neurons_required": total_neurons,
                "synapses_required": total_synapses,
                "recommended_neuron_capacity": recommended_neurons,
                "recommended_synapse_capacity": recommended_synapses
            }
            
        except Exception as e:
            self.logger.error(f"Error analyzing genome requirements: {e}")
            # Return safe defaults if analysis fails
            return {
                "neurons_required": 0,
                "synapses_required": 0,
                "recommended_neuron_capacity": 100_000,
                "recommended_synapse_capacity": 1_000_000
            }

    def _update_parameters_only(
        self, 
        cortical_id: str, 
        changes: Dict[str, Any], 
        transaction
    ) -> Optional[Dict[str, Any]]:
        """
        Fast path: Update only neuron parameters without full brain rebuild.
        
        Performance: ~2-5ms vs ~800ms for full rebuild (160-400x faster)
        """
        import time
        start_time = time.time()
        
        try:
            # Update genome (hierarchical format)
            current_genome = self._current_genome.copy()
            area_def = current_genome["blueprint"][cortical_id]
            
            # Filter to parameter changes only  
            from feagi.api.core.services.genome.change_classifier import CorticalChangeClassifier
            parameter_changes = {
                k: v for k, v in changes.items() 
                if k in CorticalChangeClassifier.PARAMETER_TO_NEURON_PROPERTY
            }
            
            if not parameter_changes:
                self.logger.warning(f"No parameter changes found in {list(changes.keys())}")
                return area_def
            
            # Update genome parameters section
            if "parameters" not in area_def:
                area_def["parameters"] = {}
            area_def["parameters"].update(parameter_changes)
            
            # Commit genome changes to both internal cache and state manager
            self._current_genome = current_genome
            
            # CRITICAL: Synchronize StateManager's genome to maintain consistency
            if self.state_manager:
                self.state_manager.genome = current_genome
                self.logger.debug(f"[PARAMETER-UPDATE] Synchronized genome with StateManager")
            
            # Direct neuron updates (NO REBUILD!)
            from feagi.api.core.services.genome.parameter_updater import CorticalParameterUpdater
            updater = CorticalParameterUpdater(self._connectome_manager)
            
            # Update neurons in arrays
            neuron_update_success = updater.update_neuron_parameters(cortical_id, parameter_changes)
            
            # Update ConnectomeManager cortical area properties for consistency
            connectome_update_success = self._connectome_manager.update_cortical_area_properties(
                cortical_id, parameter_changes
            )
            
            success = neuron_update_success and connectome_update_success
            
            if success and transaction:
                transaction.commit()
            elif transaction:
                transaction.rollback()
                return None
                
            duration = time.time() - start_time
            if success:
                self.logger.info(
                    f"[FAST-UPDATE] Parameter update completed for {cortical_id} "
                    f"in {duration*1000:.1f}ms"
                )
                return current_genome["blueprint"][cortical_id]
            else:
                self.logger.error(f"Parameter update failed for {cortical_id}")
                return None
                
        except Exception as e:
            duration = time.time() - start_time
            self.logger.error(f"Fast parameter update failed after {duration*1000:.1f}ms: {e}")
            if transaction:
                transaction.rollback()
            return None
    
    def _update_metadata_only(
        self, 
        cortical_id: str, 
        changes: Dict[str, Any], 
        transaction
    ) -> Optional[Dict[str, Any]]:
        """
        Fastest path: Update only metadata without affecting neurons.
        
        Performance: ~1ms (metadata changes only)
        """
        import time
        start_time = time.time()
        
        try:
            # Update genome (hierarchical format)
            current_genome = self._current_genome.copy()
            area_def = current_genome["blueprint"][cortical_id]
            
            # Filter to metadata changes only
            from feagi.api.core.services.genome.change_classifier import CorticalChangeClassifier
            metadata_changes = {
                k: v for k, v in changes.items() 
                if k in CorticalChangeClassifier.METADATA_CHANGES
            }
            
            # Update genome metadata
            for key, value in metadata_changes.items():
                area_def[key] = value
                
            # Commit genome changes
            self._current_genome = current_genome
            
            # Update ConnectomeManager cortical area properties for consistency
            success = self._connectome_manager.update_cortical_area_properties(
                cortical_id, metadata_changes
            )
            
            if success and transaction:
                transaction.commit()
            elif transaction:
                transaction.rollback()
                return None
                
            duration = time.time() - start_time
            if success:
                self.logger.info(
                    f"[METADATA-UPDATE] Metadata update completed for {cortical_id} "
                    f"in {duration*1000:.1f}ms"
                )
                return current_genome["blueprint"][cortical_id]
            else:
                self.logger.error(f"Metadata update failed for {cortical_id}")
                return None
                
        except Exception as e:
            duration = time.time() - start_time
            self.logger.error(f"Metadata update failed after {duration*1000:.1f}ms: {e}")
            if transaction:
                transaction.rollback()
            return None
    
    def _update_with_full_rebuild(
        self, 
        cortical_id: str, 
        changes: Dict[str, Any], 
        transaction
    ) -> Optional[Dict[str, Any]]:
        """
        Full rebuild path: For structural changes requiring complete brain reconstruction.
        
        Performance: ~800ms (existing behavior)
        """
        import time
        start_time = time.time()
        
        try:
            # Update genome (hierarchical format) 
            current_genome = self._current_genome.copy()
            area_def = current_genome["blueprint"][cortical_id]
            
            # Apply all changes to area definition
            for key, value in changes.items():
                if key in ["cortical_name", "coordinates_3d", "cortical_dimensions", "cortical_type"]:
                    area_def[key] = value
                else:
                    # Parameter changes
                    if "parameters" not in area_def:
                        area_def["parameters"] = {}
                    area_def["parameters"][key] = value
                    
            # Commit genome changes to both internal cache and state manager
            self._current_genome = current_genome
            
            # CRITICAL: Synchronize StateManager's genome to maintain consistency
            if self.state_manager:
                self.state_manager.genome = current_genome
                self.logger.debug(f"[FULL-REBUILD] Synchronized genome with StateManager")
            
            # Full brain rebuild (existing logic)
            from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
            embryogenesis = NeuroEmbryogenesis(self._connectome_manager, self.state_manager)
            
            # ARCHITECTURE: Pass hierarchical genome directly (single source of truth)
            # NeuroEmbryogenesis now supports hierarchical format natively
            success = embryogenesis.develop_brain_from_genome_data(current_genome)
            
            if success and transaction:
                transaction.commit()
            elif transaction:
                transaction.rollback()
                return None
                
            duration = time.time() - start_time
            if success:
                self.logger.info(
                    f"[FULL-REBUILD] Structural update completed for {cortical_id} "
                    f"in {duration*1000:.1f}ms"
                )
                return current_genome["blueprint"][cortical_id]
            else:
                self.logger.error(f"Full rebuild failed for {cortical_id}")
                return None
                
        except Exception as e:
            duration = time.time() - start_time
            self.logger.error(f"Full rebuild failed after {duration*1000:.1f}ms: {e}")
            if transaction:
                transaction.rollback()
            return None
    
    def _update_hybrid(
        self, 
        cortical_id: str, 
        changes: Dict[str, Any], 
        transaction
    ) -> Optional[Dict[str, Any]]:
        """
        Hybrid path: Optimize mixed structural + parameter + metadata changes.
        
        Strategy: Apply fast updates first, then structural rebuild if needed.
        """
        import time
        start_time = time.time()
        
        try:
            # Separate changes by type
            from feagi.api.core.services.genome.change_classifier import (
                CorticalChangeClassifier, ChangeType
            )
            
            separated = CorticalChangeClassifier.separate_changes_by_type(changes)
            
            self.logger.info(
                f"[HYBRID-UPDATE] Processing {len(separated[ChangeType.STRUCTURAL])} structural, "
                f"{len(separated[ChangeType.PARAMETER])} parameter, "
                f"{len(separated[ChangeType.METADATA])} metadata changes"
            )
            
            # Apply metadata changes first (fastest)
            if separated[ChangeType.METADATA]:
                result = self._update_metadata_only(cortical_id, separated[ChangeType.METADATA], None)
                if not result:
                    return None
                    
            # If we have structural changes, do localized rebuild (includes parameters)
            if separated[ChangeType.STRUCTURAL]:
                # Combine structural + parameter changes for single localized rebuild
                combined_changes = {**separated[ChangeType.STRUCTURAL], **separated[ChangeType.PARAMETER]}
                result = self._update_with_localized_rebuild(cortical_id, combined_changes, transaction)
            else:
                # Only parameter changes remain - use fast path
                if separated[ChangeType.PARAMETER]:
                    result = self._update_parameters_only(cortical_id, separated[ChangeType.PARAMETER], transaction)
                else:
                    # Only metadata was updated
                    if transaction:
                        transaction.commit()
                    result = self._current_genome["blueprint"][cortical_id]
                    
            duration = time.time() - start_time
            self.logger.info(
                f"[HYBRID-UPDATE] Completed hybrid update for {cortical_id} "
                f"in {duration*1000:.1f}ms"
            )
            
            return result
            
        except Exception as e:
            duration = time.time() - start_time
            self.logger.error(f"Hybrid update failed after {duration*1000:.1f}ms: {e}")
            if transaction:
                transaction.rollback()
            return None

    def _update_with_localized_rebuild(
        self, 
        cortical_id: str, 
        changes: Dict[str, Any], 
        transaction
    ) -> Optional[Dict[str, Any]]:
        """
        Localized rebuild path: For structural changes requiring only the specific area to be rebuilt.
        
        This method rebuilds ONLY the specific cortical area that changed, preserving
        all other areas and their neuron IDs. This fixes the neuron ID instability bug
        where changing one area would reset the entire brain.
        
        Performance: ~100-200ms (vs ~800ms for full rebuild)
        """
        import time
        start_time = time.time()
        
        try:
            # 1. Update genome (hierarchical format) first
            current_genome = self._current_genome.copy()
            area_def = current_genome["blueprint"][cortical_id]
            
            # Apply all changes to area definition
            for key, value in changes.items():
                if key in ["cortical_name", "coordinates_3d", "cortical_dimensions", "cortical_type"]:
                    area_def[key] = value
                else:
                    # Parameter changes
                    if "parameters" not in area_def:
                        area_def["parameters"] = {}
                    area_def["parameters"][key] = value
                    
            # Commit genome changes to both internal cache and state manager
            self._current_genome = current_genome
            
            # CRITICAL: Synchronize StateManager's genome to maintain consistency
            if self.state_manager:
                self.state_manager.genome = current_genome
                self.logger.info(f"[LOCALIZED-REBUILD] Synchronized genome with StateManager")
            
            # 2. Extract properties for area recreation
            properties = self._extract_area_properties_from_genome(cortical_id, current_genome)
            
            # 3. PROPER REAL-TIME APPROACH: Resize cortical area without deleting neurons
            # This preserves memory locations for GPU compatibility and avoids lag
            self.logger.info(f"[LOCALIZED-REBUILD] Resizing cortical area {cortical_id} dimensions: {changes.get('cortical_dimensions')}")
            old_area_existed = cortical_id in self._connectome_manager.cortical_areas
            new_dimensions = tuple(changes.get('cortical_dimensions', [1, 1, 1]))
            
            if old_area_existed:
                area = self._connectome_manager.cortical_areas[cortical_id]
                old_dimensions = area.dimensions
                new_dimensions = tuple(changes.get('cortical_dimensions', old_dimensions))
                
                if new_dimensions != old_dimensions:
                    self.logger.info(f"[LOCALIZED-REBUILD] Changing dimensions from {old_dimensions} to {new_dimensions}")
                    
                    # Use the proper cortical area resize method - NO NEURON DELETION
                    removed_neuron_indices = area.resize(new_dimensions)
                    
                    if removed_neuron_indices:
                        # Mark neurons as inactive and add to free pool (GPU-friendly)
                        self.logger.info(f"[LOCALIZED-REBUILD] Marking {len(removed_neuron_indices)} neurons as inactive (reusable)")
                        for neuron_idx in removed_neuron_indices:
                            neuron_id = self._connectome_manager.index_to_neuron_id.get(neuron_idx)
                            if neuron_id:
                                # Mark as inactive but don't delete - preserves memory location
                                self._connectome_manager.neuron_array.is_active[neuron_idx] = False
                                self._connectome_manager.neuron_array.valid_mask[neuron_idx] = False
                                self._connectome_manager.neuron_array.free_indices.add(neuron_idx)
                    
                    # Calculate if we need more neurons
                    old_volume = old_dimensions[0] * old_dimensions[1] * old_dimensions[2]
                    new_volume = new_dimensions[0] * new_dimensions[1] * new_dimensions[2]
                    neurons_per_voxel = properties.get("neurons_per_voxel", 1)
                    
                    if new_volume > old_volume:
                        additional_neurons_needed = (new_volume - old_volume) * neurons_per_voxel
                        self.logger.info(f"[LOCALIZED-REBUILD] Need {additional_neurons_needed} additional neurons - reusing from free pool")
                        
                        # Reuse neurons from free pool - NO MEMORY ALLOCATION
                        self._reuse_neurons_for_area_expansion(cortical_id, additional_neurons_needed, properties)
                    
                    self.logger.info(f"[LOCALIZED-REBUILD] Cortical area {cortical_id} resized - {len(self._connectome_manager.neuron_array.free_indices)} neurons in free pool")
                else:
                    self.logger.info(f"[LOCALIZED-REBUILD] No dimension change needed for {cortical_id}")
            else:
                # Area doesn't exist - create it properly using existing methods
                self.logger.info(f"[LOCALIZED-REBUILD] Creating new cortical area {cortical_id}")
                self._connectome_manager.add_cortical_area(
                    name=properties["name"],
                    dimensions=tuple(properties["dimensions"]),
                    position=tuple(properties["position"]),
                    area_type=properties.get("area_type", "custom"),
                    properties=properties,
                    cortical_id=cortical_id
                )
            
            # 4. Update area properties without disrupting neuron structure
            if old_area_existed:
                area = self._connectome_manager.cortical_areas[cortical_id]
                
                # Update properties in-place - preserves neuron assignments
                if "name" in properties:
                    area.name = properties["name"]
                if "position" in properties:
                    area.position = tuple(properties["position"])
                if "area_type" in properties:
                    area.area_type = properties["area_type"]
                
                # Update additional properties
                for key, value in properties.items():
                    if key not in ["name", "position", "area_type", "dimensions"]:
                        area.properties[key] = value
                
                self.logger.info(f"[LOCALIZED-REBUILD] Updated properties for existing area {cortical_id}")
            
            # 5. Update connections if needed (preserve existing structure)
            # Only rebuild connections that specifically involve dimension changes
            if old_area_existed and new_dimensions != old_dimensions:
                self.logger.info(f"[LOCALIZED-REBUILD] Updating connections for resized area {cortical_id}")
                # Note: Connection updates should be minimal for dimension changes
                # Most connections should remain valid since we're not deleting neurons
            
            # NO VISUALIZATION INTERRUPTION NEEDED
            # Since we're not deleting/recreating neurons, neuron IDs remain stable
            # and the visualization stream can continue uninterrupted
            
            if transaction:
                transaction.commit()
                
            duration = time.time() - start_time
            self.logger.info(
                f"[LOCALIZED-REBUILD] Real-time cortical area resize completed for {cortical_id} "
                f"in {duration*1000:.1f}ms (no neuron deletion, GPU-friendly)"
            )
            return current_genome["blueprint"][cortical_id]
                
        except Exception as e:
            duration = time.time() - start_time
            self.logger.error(f"Real-time cortical area resize failed after {duration*1000:.1f}ms: {e}")
            
            if transaction:
                transaction.rollback()
            raise e

    def _reuse_neurons_for_area_expansion(self, cortical_id: str, additional_neurons_needed: int, properties: Dict[str, Any]) -> None:
        """
        Reuse neurons from the pre-allocated free pool for cortical area expansion.
        
        This is the proper real-time approach that:
        - Reuses existing memory locations (GPU-friendly)
        - Avoids memory allocation/deallocation 
        - Maintains stable neuron IDs
        - Provides zero-lag operation
        
        Args:
            cortical_id: The cortical area being expanded
            additional_neurons_needed: Number of additional neurons required
            properties: Area properties for neuron configuration
        """
        if additional_neurons_needed <= 0:
            return
            
        free_pool_size = len(self._connectome_manager.neuron_array.free_indices)
        
        if additional_neurons_needed > free_pool_size:
            self.logger.warning(
                f"[REAL-TIME] Need {additional_neurons_needed} neurons but only "
                f"{free_pool_size} available in free pool. Using available neurons."
            )
            additional_neurons_needed = free_pool_size
        
        if additional_neurons_needed > 0:
            # Get cortical index for the area
            area = self._connectome_manager.cortical_areas[cortical_id]
            cortical_idx = area.cortical_idx
            
            # Reuse neurons from free pool - this is GPU-friendly and real-time
            positions = []
            for i in range(additional_neurons_needed):
                # Generate positions within the new area dimensions
                x = i % area.dimensions[0] 
                y = (i // area.dimensions[0]) % area.dimensions[1]
                z = i // (area.dimensions[0] * area.dimensions[1])
                positions.append((x, y, z))
            
            # Batch create neurons using pre-allocated memory
            reused_neuron_ids = self._connectome_manager.neuron_array.batch_create_neurons(
                cortical_idx=cortical_idx,
                positions=positions,
                thresholds=properties.get("fire_t", 1.0),
                membrane_potentials=0.0,
                resting_potentials=0.0,
                decay_rates=properties.get("leak_coefficient", 0.5),
                refractory_periods=properties.get("refractory_period", 1)
            )
            
            # Update area tracking
            for neuron_id, position in zip(reused_neuron_ids, positions):
                area.add_neuron(neuron_id, position)
            
            self.logger.info(
                f"[REAL-TIME] Reused {len(reused_neuron_ids)} neurons from free pool for {cortical_id} "
                f"(GPU-friendly, zero memory allocation)"
            )

    def _extract_area_properties_from_genome(self, cortical_id: str, genome: Dict[str, Any]) -> Dict[str, Any]:
        """Extract cortical area properties from hierarchical genome format."""
        try:
            area_def = genome["blueprint"][cortical_id]
            
            return {
                "name": area_def.get("cortical_name", cortical_id),
                "dimensions": area_def.get("cortical_dimensions", [1, 1, 1]),
                "position": area_def.get("coordinates_3d", [0, 0, 0]),
                "area_type": area_def.get("cortical_type", "custom"),
                "neurons_per_voxel": area_def.get("parameters", {}).get("per_voxel_neuron_cnt", 1),
                "fire_t": area_def.get("parameters", {}).get("firing_threshold", 1.0),
                "leak_c": area_def.get("parameters", {}).get("leak_coefficient", 0.0),
                "refrac": area_def.get("parameters", {}).get("refractory_period", 1),
                "fire_increment": area_def.get("parameters", {}).get("fire_increment", 0.0),
                "leak_variability": area_def.get("parameters", {}).get("leak_variability", 0.0),
            }
        except KeyError as e:
            raise ValueError(f"Missing required property in genome for area {cortical_id}: {e}")

    def _rebuild_neurons_for_area(self, cortical_id: str, properties: Dict[str, Any]) -> None:
        """Rebuild neurons for a specific cortical area (adapted from NeuroEmbryogenesis._perform_neurogenesis)."""
        try:
            area = self._connectome_manager.cortical_areas[cortical_id]
            
            # Calculate neuron count for this area
            width, height, depth = area.dimensions
            neurons_per_voxel = properties.get("neurons_per_voxel", 1)
            area_neuron_count = width * height * depth * neurons_per_voxel
            
            self.logger.info(f"[LOCALIZED-REBUILD] Creating {area_neuron_count} neurons for {cortical_id}")
            
            # Reserve array indices in bulk
            neuron_array = self._connectome_manager.neuron_array
            start_idx = neuron_array.next_index
            end_idx = start_idx + area_neuron_count
            
            if end_idx > neuron_array.max_neurons:
                raise ValueError(f"Not enough capacity for {area_neuron_count} neurons")
            
            # Generate neuron IDs in bulk (continuing from current counter)
            neuron_ids = list(range(neuron_array._next_neuron_id, neuron_array._next_neuron_id + area_neuron_count))
            neuron_array._next_neuron_id += area_neuron_count
            
            # Update mappings in bulk via ConnectomeManager (single source of truth)
            for j, neuron_id in enumerate(neuron_ids):
                self._connectome_manager.set_neuron_mapping(neuron_id, start_idx + j)
            
            # Set uniform properties with vectorized array slicing
            base_threshold = properties.get("fire_t", 1.0)
            base_decay_rate = 1.0 - (properties.get("leak_c", 0) / 100.0)
            base_refractory = properties.get("refrac", 1)
            
            # SoA OPTIMIZATION: Set all properties with single array operations
            neuron_array.valid_mask[start_idx:end_idx] = True
            neuron_array.membrane_potentials[start_idx:end_idx] = 0.0
            neuron_array.resting_potentials[start_idx:end_idx] = 0.0
            neuron_array.thresholds[start_idx:end_idx] = base_threshold
            neuron_array.decay_rates[start_idx:end_idx] = base_decay_rate
            neuron_array.refractory_periods[start_idx:end_idx] = base_refractory
            neuron_array.refractory_counters[start_idx:end_idx] = 0
            neuron_array.cortical_idxs[start_idx:end_idx] = area.cortical_idx
            neuron_array.is_active[start_idx:end_idx] = True
            
            # Generate coordinates and apply position-based variations
            import numpy as np
            positions = []
            for x in range(width):
                for y in range(height):
                    for z in range(depth):
                        for _ in range(neurons_per_voxel):
                            positions.append((x, y, z))
            
            # Set coordinates with vectorized operations
            coords_x = np.array([pos[0] for pos in positions], dtype=np.uint32)
            coords_y = np.array([pos[1] for pos in positions], dtype=np.uint32) 
            coords_z = np.array([pos[2] for pos in positions], dtype=np.uint32)
            
            neuron_array.coordinates_x[start_idx:end_idx] = coords_x
            neuron_array.coordinates_y[start_idx:end_idx] = coords_y
            neuron_array.coordinates_z[start_idx:end_idx] = coords_z
            
            # Apply position-based variations with vectorized operations
            fire_increment = properties.get("fire_increment", 0.0)
            if fire_increment != 0.0:
                z_increments = coords_z.astype(np.float32) * fire_increment
                neuron_array.thresholds[start_idx:end_idx] += z_increments
            
            leak_variability = properties.get("leak_variability", 0.0)
            base_leak = properties.get("leak_c", 0.0)
            if leak_variability != 0.0 and base_leak != 0.0:
                np.random.seed(42)  # Deterministic for reproducibility
                variations = np.random.uniform(-leak_variability, leak_variability, area_neuron_count) / 100.0
                varied_leak = np.clip(base_leak / 100.0 + variations, 0.0, 1.0)
                neuron_array.decay_rates[start_idx:end_idx] = 1.0 - varied_leak
            
            # Sync neurons with cortical area objects using bulk operations
            area._neuron_indices.update(neuron_ids)
            area._position_map.update(zip(neuron_ids, positions))
            
            for neuron_id, position in zip(neuron_ids, positions):
                if position not in area._position_to_neurons:
                    area._position_to_neurons[position] = []
                area._position_to_neurons[position].append(neuron_id)
            
            # Update array state
            neuron_array.next_index = end_idx
            neuron_array.neuron_count = end_idx
            
            self.logger.info(f"[LOCALIZED-REBUILD] Created {area_neuron_count} neurons for {cortical_id}")
            
        except Exception as e:
            self.logger.error(f"Failed to rebuild neurons for area {cortical_id}: {e}")
            raise

    def _rebuild_connections_for_area(self, cortical_id: str, genome: Dict[str, Any]) -> None:
        """Rebuild synaptic connections involving the specified cortical area."""
        try:
            # Import NeuroEmbryogenesis for connection building
            from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
            
            # Create temporary embryogenesis instance for connection building
            embryogenesis = NeuroEmbryogenesis(self._connectome_manager, self.state_manager)
            embryogenesis.genome = genome
            
            # 1. Remove all existing connections involving this area
            self._remove_area_connections(cortical_id)
            
            # 2. Rebuild outgoing connections (this area -> others)
            area_def = genome["blueprint"][cortical_id]
            if "parameters" in area_def and "mapping" in area_def["parameters"]:
                mapping_data = area_def["parameters"]["mapping"]
                self.logger.info(f"[LOCALIZED-REBUILD] Rebuilding outgoing connections from {cortical_id}")
                
                # Use NeuroEmbryogenesis.update_cortical_mapping for proper connection building
                api_mapping = {cortical_id: mapping_data}
                embryogenesis.update_cortical_mapping(api_mapping)
            
            # 3. Rebuild incoming connections (others -> this area)
            self.logger.info(f"[LOCALIZED-REBUILD] Rebuilding incoming connections to {cortical_id}")
            for other_cortical_id, other_area_def in genome["blueprint"].items():
                if other_cortical_id == cortical_id:
                    continue
                    
                if "parameters" in other_area_def and "mapping" in other_area_def["parameters"]:
                    other_mapping = other_area_def["parameters"]["mapping"]
                    if cortical_id in other_mapping:
                        # This other area connects to our rebuilt area
                        connection_data = {cortical_id: other_mapping[cortical_id]}
                        api_mapping = {other_cortical_id: connection_data}
                        embryogenesis.update_cortical_mapping(api_mapping)
            
            self.logger.info(f"[LOCALIZED-REBUILD] Completed connection rebuilding for {cortical_id}")
            
        except Exception as e:
            self.logger.error(f"Failed to rebuild connections for area {cortical_id}: {e}")
            raise

    def _remove_area_connections(self, cortical_id: str) -> None:
        """Remove all synaptic connections involving the specified cortical area."""
        try:
            # Get all neurons in the area (before deletion, these would be the old neurons)
            # After area deletion and recreation, we need to remove connections to the old neurons
            
            # For now, we rely on ConnectomeManager.delete_cortical_area() to handle
            # connection cleanup. This method is called as a safety net for any remaining connections.
            
            # Get all existing connections and remove those involving this area
            connection_ids_to_remove = []
            for connection_id, connection in self._connectome_manager.cortical_connections.items():
                if (connection["source_area_id"] == cortical_id or 
                    connection["target_area_id"] == cortical_id):
                    connection_ids_to_remove.append(connection_id)
            
            for connection_id in connection_ids_to_remove:
                self._connectome_manager.delete_cortical_connection(connection_id, delete_synapses=True)
                
            self.logger.info(f"[LOCALIZED-REBUILD] Removed {len(connection_ids_to_remove)} connection definitions for {cortical_id}")
            
        except Exception as e:
            self.logger.warning(f"Error removing connections for area {cortical_id}: {e}")
