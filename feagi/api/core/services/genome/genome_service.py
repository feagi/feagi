"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
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
from typing import Any, Dict, List, Optional, Tuple

# Import genome conversion functions for flat <-> hierarchical conversion
from feagi.evo.genome_processor import genome_2_1_convertor

from ..shared.base_service import BaseService


class GenomeService(BaseService):
    """Genome service handles genome loading, saving, validation, and genome-
    related operations."""

    def __init__(
        self, connectome_manager, state_manager=None, core_api_service=None
    ):
        """Initialize genome service."""
        super().__init__(connectome_manager, state_manager)
        self._current_genome = None
        self._genome_filename = None
        self._temp_dir = tempfile.mkdtemp(prefix="feagi_")
        self._core_api_service = core_api_service

        self.logger.debug(
            "GENOME SERVICE: Initialized with clean architecture"
        )

    def _ensure_core_areas_in_blueprint(self, genome: Dict[str, Any]) -> None:
        """Ensure core cortical areas ('_death', '_power') exist in the genome blueprint.

        This uses FEAGI templates as the single source of truth to create minimal
        hierarchical definitions when missing. This keeps genome and connectome
        mapping consistent without introducing hardcoded values.

        Args:
            genome: The working genome dictionary (hierarchical format expected)
        """
        try:
            if not isinstance(genome, dict) or "blueprint" not in genome:
                return

            blueprint = genome["blueprint"]
            if not isinstance(blueprint, dict):
                return

            from feagi.evo.templates import cortical_types

            core_devices = (
                cortical_types.get("CORE", {}).get("supported_devices", {})
            )

            for area_id in ("_death", "_power"):
                if area_id in blueprint:
                    continue

                area_template = core_devices.get(area_id, {})

                # Build minimal hierarchical area definition
                coord3d = area_template.get("coordinate_3d", [0, 0, 0])
                res = area_template.get("resolution", [1, 1, 1])

                area_def = {
                    "cortical_name": area_template.get(
                        "cortical_name", area_id
                    ),
                    "coordinates": {"x": coord3d[0], "y": coord3d[1], "z": coord3d[2]},
                    "dimensions": {"x": res[0], "y": res[1], "z": res[2]},
                    "parameters": {},
                }

                blueprint[area_id] = area_def
                self.logger.info(
                    f"Added core cortical area '{area_id}' to genome blueprint from templates"
                )
        except Exception as e:
            # Non-fatal: keep loader deterministic while logging context
            self.logger.warning(
                f"Could not ensure core areas in genome blueprint: {e}"
            )

    def _clear_connectome_manager_data(self) -> None:
        """Clear connectome manager's internal data structures to prevent stale data.
        
        This method immediately clears the connectome manager's neuron arrays, cortical areas,
        and other data structures to ensure health checks don't report stale data during
        genome loading.
        
        ARCHITECTURE COMPLIANCE: This follows the same pattern used in snapshot restoration
        to ensure clean state transitions.
        """
        if not self._connectome_manager:
            self.logger.debug("No connectome manager to clear")
            return
            
        try:
            self.logger.info("Clearing connectome manager data structures")
            cm = self._connectome_manager
            
            # Clear cortical areas and mapping state
            if hasattr(cm, "cortical_areas") and isinstance(cm.cortical_areas, dict):
                cm.cortical_areas.clear()
                self.logger.debug("Cleared cortical areas")
                
            if hasattr(cm, "cortical_mapping") and hasattr(cm.cortical_mapping, "clear"):
                cm.cortical_mapping.clear()
                self.logger.debug("Cleared cortical mapping")
                
            if hasattr(cm, "cortical_connections"):
                cm.cortical_connections = {}
                self.logger.debug("Cleared cortical connections")
            
            # Clear id/index mappings
            if hasattr(cm, "_neuron_id_to_index_map"):
                cm._neuron_id_to_index_map.clear()
            if hasattr(cm, "_index_to_neuron_id_map"):
                cm._index_to_neuron_id_map.clear()
            
            # Reset neuron arrays to empty state
            try:
                if hasattr(cm, "_npu_interface") and cm._npu_interface:
                    npu = cm._npu_interface
                    
                    # Clear regular neuron array
                    if hasattr(npu, "neuron_array") and npu.neuron_array:
                        na = npu.neuron_array
                        if hasattr(na, "count"):
                            na.count = 0
                        if hasattr(na, "next_index"):
                            na.next_index = 0
                        if hasattr(na, "neuron_count"):
                            na.neuron_count = 0
                        if hasattr(na, "free_indices"):
                            na.free_indices = set()
                        if hasattr(na, "valid_mask"):
                            na.valid_mask[:] = False
                        if hasattr(na, "is_active"):
                            na.is_active[:] = False
                        self.logger.debug("Cleared regular neuron array")
                    
                    # Clear memory neuron array
                    if hasattr(npu, "memory_neuron_array") and npu.memory_neuron_array:
                        mna = npu.memory_neuron_array
                        if hasattr(mna, "count"):
                            mna.count = 0
                        if hasattr(mna, "next_index"):
                            mna.next_index = 0
                        if hasattr(mna, "neuron_count"):
                            mna.neuron_count = 0
                        if hasattr(mna, "free_indices"):
                            mna.free_indices = set()
                        if hasattr(mna, "valid_mask"):
                            mna.valid_mask[:] = False
                        if hasattr(mna, "is_active"):
                            mna.is_active[:] = False
                        self.logger.debug("Cleared memory neuron array")
                        
            except Exception as e:
                self.logger.debug(f"Could not clear NPU arrays: {e}")
            
            # Clear synapse arrays
            try:
                sa = getattr(cm, "synapse_array", None)
                if sa is not None:
                    for attr in ("next_slot", "count"):
                        if hasattr(sa, attr):
                            setattr(sa, attr, 0)
                    for attr in ("free_slots",):
                        if hasattr(sa, attr):
                            setattr(sa, attr, set())
                    self.logger.debug("Cleared synapse array")
            except Exception as e:
                self.logger.debug(f"Could not clear synapse array: {e}")
            
            # Clear FCL caches
            try:
                fclm = getattr(cm, "fcl_manager", None)
                if fclm is not None and hasattr(fclm, "clear_all_fcl_history"):
                    fclm.clear_all_fcl_history()
                    self.logger.debug("Cleared FCL history")
            except Exception as e:
                self.logger.debug(f"Could not clear FCL history: {e}")
            
            self.logger.info("✅ Connectome manager data cleared successfully")
            
        except Exception as e:
            self.logger.warning(f"Error clearing connectome manager data: {e}")
            # Non-fatal: continue with genome loading even if clearing fails

    def _clear_state_for_genome_loading(self) -> None:
        """Clear all state manager entries for genome loading while preserving genome counter.
        
        This function ensures a clean state when loading a new genome by resetting all
        brain-related statistics, development state, and connection data while preserving
        the genome counter for version tracking.
        
        ARCHITECTURE COMPLIANCE: This method follows the centralized state management
        pattern and ensures deterministic state clearing without hardcoded values.
        """
        if not self.state_manager:
            self.logger.warning("State manager not available for state clearing")
            return
            
        try:
            # Preserve genome counter before clearing
            preserved_genome_counter = self.state_manager.get_genome_counter()
            
            self.logger.info("Clearing state manager entries for genome loading")
            
            # Core genome and brain state
            self.state_manager.set_brain_readiness(False)
            self.state_manager.set_genome_validity(False)
            
            # Clear brain statistics and counts
            self.logger.info("Clearing brain stats in state manager")
            result = self.state_manager.set_brain_stats({
                "neuron_count": 0,
                "synapse_count": 0,
                "cortical_area_count": 0,
                "memory_neuron_count": 0,
                "non_memory_neuron_count": 0
            })
            if result.is_err:
                self.logger.warning("Failed to clear brain stats")
            else:
                self.logger.info("✅ Brain stats cleared: all counts set to 0")
            
            result = self.state_manager.set_cortical_list([])
            if result.is_err:
                self.logger.warning("Failed to clear cortical list")
            else:
                self.logger.info("✅ Cortical list cleared")
                
            # Reset connectome state to MISSING
            from feagi.core.state_manager import ConnectomeState
            result = self.state_manager.set_connectome_state(ConnectomeState.MISSING)
            if result.is_err:
                self.logger.warning("Failed to reset connectome state")
            
            # Clear all statistical counters
            self.state_manager.set_neuron_count(0)
            self.state_manager.set_synapse_count(0)
            
            # CRITICAL: Clear connectome manager data immediately to prevent health check
            # from showing stale data during genome loading
            self._clear_connectome_manager_data()
            
            # Clear agent/connection data
            result = self.state_manager.set_connected_agents({})
            if result.is_err:
                self.logger.warning("Failed to clear connected agents")
                
            result = self.state_manager.set_agent_count(0)
            if result.is_err:
                self.logger.warning("Failed to reset agent count")
            
            # Reset development/embryogenesis state
            if hasattr(self.state_manager, 'set_neuroembryogenesis_stage'):
                try:
                    self.state_manager.set_neuroembryogenesis_stage(0)  # INITIALIZATION
                except Exception as e:
                    self.logger.debug(f"Could not reset neuroembryogenesis stage: {e}")
                    
            if hasattr(self.state_manager, 'set_neuroembryogenesis_progress'):
                try:
                    self.state_manager.set_neuroembryogenesis_progress(0)
                except Exception as e:
                    self.logger.debug(f"Could not reset neuroembryogenesis progress: {e}")
            
            # Clear external state tracking
            result = self.state_manager.set_changes_saved_externally(False)
            if result.is_err:
                self.logger.warning("Failed to reset changes_saved_externally")
                
            # Reset simulation state to STOPPED
            from feagi.core.state_manager import SimulationState
            try:
                self.state_manager.set_simulation_state(SimulationState.STOPPED)
            except Exception as e:
                self.logger.debug(f"Could not reset simulation state: {e}")
            
            # Clear amalgamation state (legacy compatibility)
            if hasattr(self.state_manager._state, 'pending_amalgamation'):
                try:
                    self.state_manager._state.pending_amalgamation = {}
                except Exception as e:
                    self.logger.debug(f"Could not clear pending_amalgamation: {e}")
                    
            if hasattr(self.state_manager._state, 'amalgamation_history'):
                try:
                    self.state_manager._state.amalgamation_history = {}
                except Exception as e:
                    self.logger.debug(f"Could not clear amalgamation_history: {e}")
            
            # Update genome timestamp but preserve counter
            import time
            current_timestamp = int(time.time() * 1000)
            result = self.state_manager.set_genome_timestamp(current_timestamp)
            if result.is_err:
                self.logger.warning("Failed to update genome timestamp")
                
            # Restore preserved genome counter
            if hasattr(self.state_manager._state, 'genome_counter'):
                try:
                    self.state_manager._state.genome_counter = preserved_genome_counter
                    self.logger.debug(f"Preserved genome counter: {preserved_genome_counter}")
                except Exception as e:
                    self.logger.warning(f"Could not restore genome counter: {e}")
            
            self.logger.info("✅ State manager cleared successfully for genome loading")
            
        except Exception as e:
            self.logger.error(f"Error clearing state for genome loading: {e}")
            # Non-fatal: continue with genome loading even if state clearing fails

    def load_genome(
        self, genome_data: Dict[str, Any], filename: str = "genome.json"
    ) -> Dict[str, Any]:
        """Load a genome and prepare it for use."""
        try:
            #  ARCHITECTURE: Only import NeuroEmbryogenesis - no old
            #  develop_brain_from_genome
            from feagi.bdu.embryogenesis.neuroembryogenesis import (
                NeuroEmbryogenesis,
            )

            try:
                # Try to import these from the new location
                from feagi.evo.genome_validator import (
                    genome_validator_with_errors,
                )
            except ImportError:
                # Fallback to the old location
                from feagi.core.genome.genome_validator import genome_validator

            self.logger.info(f"Loading genome from {filename}")

            # Store the provided genome data for processing
            self._current_genome = copy.deepcopy(genome_data)

            if not self._connectome_manager:
                return {
                    "success": False,
                    "error": "Connectome manager not available",
                }

            # CRITICAL: Start timing for performance monitoring

            try:
                self.logger.info("Step 1: Initializing genome load process")

                # Set genome loading state and clear all state manager entries
                if self.state_manager:
                    from feagi.core.state_manager import GenomeState

                    self.state_manager.set_genome_state(GenomeState.LOADING)
                    
                    # Comprehensive state clearing for genome loading
                    self._clear_state_for_genome_loading()

                #  CRITICAL: Preserve old genome data BEFORE setting new values
                #  for comparison
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
                    from feagi.evo.genome_validator import (
                        genome_validator_with_errors,
                    )
                except ImportError:
                    #  Fallback to basic validator if detailed validator not
                    #  available
                    try:
                        from feagi.core.genome.genome_validator import (
                            genome_validator,
                        )

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
                    #  Check if auto-recovery is enabled to determine which
                    #  validation to use
                    if allow_auto_recovery:
                        #  Use silent validation for initial check to avoid
                        #  logging errors that will be fixed
                        from feagi.evo.genome_validator import (
                            genome_validator_with_errors_silent,
                        )

                        validation_result = (
                            genome_validator_with_errors_silent(genome_data)
                        )
                    else:
                        #  Auto-recovery disabled - use regular validation (log
                        #  all errors)
                        validation_result = genome_validator_with_errors(
                            genome_data
                        )

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
                        #  Only log errors when auto-recovery is disabled (they
                        #  won't be fixed)
                        self.logger.error(
                            f"Genome validation failed: {error_msg}"
                        )
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
                        #  Auto-recovery is enabled - suppress initial
                        #  validation error logging
                        #  Just log that we're attempting auto-recovery instead
                        #  of the detailed errors
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

                            sanitization_result = (
                                sanitize_invalid_morphologies(genome_data)
                            )

                            # Use the sanitized genome
                            genome_data = sanitization_result["genome"]
                            removed_morphologies = sanitization_result[
                                "removed_morphologies"
                            ]
                            fixed_references = sanitization_result[
                                "fixed_references"
                            ]
                            recovery_summary = sanitization_result[
                                "recovery_summary"
                            ]

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
                                post_sanitization_result = (
                                    genome_validator_with_errors(genome_data)
                                )
                                if post_sanitization_result["valid"]:
                                    self.logger.info(
                                        "Genome validation passed after auto-recovery sanitization"
                                    )
                                    validation_result = post_sanitization_result  # Update validation result
                                    if self.state_manager:
                                        result = self.state_manager.set_genome_validity(
                                            True
                                        )
                                        if result.is_err:
                                            self.logger.warning(
                                                f"Failed to set genome validity: {result.unwrap_err()}"
                                            )
                                else:
                                    #  NOW log the errors since auto-recovery
                                    #  couldn't fix them
                                    remaining_error_msg = (
                                        post_sanitization_result.get(
                                            "error_summary",
                                            "Unknown validation issues",
                                        )
                                    )
                                    remaining_errors = (
                                        post_sanitization_result.get(
                                            "errors", []
                                        )
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
                                        result = self.state_manager.set_genome_validity(
                                            False
                                        )
                                        if result.is_err:
                                            self.logger.warning(
                                                f"Failed to set genome validity: {result.unwrap_err()}"
                                            )
                            except Exception as revalidation_error:
                                self.logger.warning(
                                    f"Could not re-validate after sanitization: {revalidation_error}"
                                )
                                # Assume it's still invalid but continue
                                if self.state_manager:
                                    result = (
                                        self.state_manager.set_genome_validity(
                                            False
                                        )
                                    )
                                    if result.is_err:
                                        self.logger.warning(
                                            f"Failed to set genome validity: {result.unwrap_err()}"
                                        )

                            #  Store auto-recovery details for inclusion in
                            #  response (CRITICAL - don't overwrite later!)
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
                            #  Now log the original errors since auto-recovery
                            #  failed
                            self.logger.error(
                                f"Original genome validation failed: {error_msg}"
                            )
                            if specific_errors:
                                self.logger.error(
                                    "Original validation errors:"
                                )
                                for error in specific_errors:
                                    self.logger.error(f"  - {error}")

                            #  Fall back to original approach - mark as invalid
                            #  but continue
                            if self.state_manager:
                                result = (
                                    self.state_manager.set_genome_validity(
                                        False
                                    )
                                )
                                if result.is_err:
                                    self.logger.warning(
                                        f"Failed to set genome validity: {result.unwrap_err()}"
                                    )
                            auto_recovery_details = {
                                "recovery_performed": False,
                                "recovery_error": str(sanitization_error),
                                "original_errors": specific_errors,
                            }
                else:
                    #  Validation passed initially - keep the original
                    #  auto_recovery_details (no changes needed)
                    if self.state_manager:
                        result = self.state_manager.set_genome_validity(True)
                        if result.is_err:
                            self.logger.warning(
                                f"Failed to set genome validity: {result.unwrap_err()}"
                            )

                # Store the current genome - CONVERT TO HIERARCHICAL FORMAT
                #  ARCHITECTURE: Only store hierarchical format for working
                #  operations
                # Flat format is used only for save/export operations
                if "blueprint" in genome_data and isinstance(
                    genome_data["blueprint"], dict
                ):
                    #  Check if this is flat format (contains flattened keys
                    #  like _____10c-area_id-cx-property-type)
                    #  ARCHITECTURE: Robust detection - parse for dash pattern,
                    #  not hardcoded underscores
                    blueprint_keys = list(genome_data["blueprint"].keys())

                    def is_flat_genome_key(key):
                        """Detect flat genome format: *10c-area_id-{cx|nx}-property-type"""
                        import re

                        #  Pattern: any prefix ending with 10c-, then area_id,
                        #  then -cx- or -nx-, then more components
                        return bool(re.match(r".*10c-[^-]+-[cn]x-.*", key))

                    if blueprint_keys and any(
                        is_flat_genome_key(key) for key in blueprint_keys[:5]
                    ):
                        self.logger.info(
                            "Converting flat genome to hierarchical format for working operations"
                        )
                        # Convert flat blueprint to hierarchical structure
                        hierarchical_genome = copy.deepcopy(genome_data)
                        hierarchical_genome["blueprint"] = (
                            genome_2_1_convertor(genome_data["blueprint"])[
                                "blueprint"
                            ]
                        )
                        self._current_genome = hierarchical_genome
                        self.logger.info(
                            f"Converted {len(blueprint_keys)} flat entries to hierarchical structure"
                        )
                    else:
                        # Already hierarchical format
                        self.logger.info(
                            "Genome already in hierarchical format"
                        )
                        self._current_genome = genome_data
                else:
                    # No blueprint section or not dict - store as-is
                    self._current_genome = genome_data

                # Ensure core areas are present in the hierarchical blueprint
                self._ensure_core_areas_in_blueprint(self._current_genome)

                #  ARCHITECTURE IMPROVEMENT: Stage sanitized genome in state
                #  manager FIRST
                #  This ensures connectome manager always builds from single
                #  source of truth
                if self.state_manager:
                    #  CRITICAL: Always store hierarchical format in state
                    #  manager (single source of truth)
                    self.state_manager.genome = self._current_genome
                    self.state_manager.genome_file_name = filename
                    validity_result = self.state_manager.set_genome_validity(
                        True if validation_result.get("valid") else False
                    )
                    if validity_result.is_err:
                        self.logger.warning(
                            f"Failed to set genome validity: {validity_result.unwrap_err()}"
                        )
                    #  Set to STAGING state while brain development is in
                    #  progress
                    from feagi.core.state_manager import GenomeState

                    self.state_manager.set_genome_state(GenomeState.LOADING)

                    self.logger.info(
                        "Sanitized genome staged in state manager as single source of truth"
                    )

                #  CRITICAL: Prepare connectome for new genome loading (clear
                #  existing brain data)
                self.logger.info(
                    "Preparing connectome for new genome loading..."
                )
                #  ARCHITECTURE FIX: Use converted hierarchical genome, not
                #  original flat format
                preparation_result = (
                    self._connectome_manager.prepare_for_new_genome(
                        self._current_genome, save_current_state=True
                    )
                )
                if not preparation_result.get("success", False):
                    self.logger.error(
                        "Failed to prepare connectome for new genome"
                    )
                    if self.state_manager:
                        from feagi.core.state_manager import GenomeState

                        self.state_manager.set_genome_state(GenomeState.ERROR)
                        self.state_manager.set_brain_readiness(False)
                        result = self.state_manager.set_genome_validity(False)
                        if result.is_err:
                            self.logger.warning(
                                f"Failed to set genome validity: {result.unwrap_err()}"
                            )
                    return {
                        "success": False,
                        "error": "Failed to prepare connectome for new genome",
                    }

                self.logger.info(
                    f"[OK] Connectome preparation complete: {preparation_result.get('message', 'Ready for genome loading')}"
                )

                #  GENOME-FIRST ARCHITECTURE: Analyze requirements BEFORE brain
                #  development
                self.logger.info(
                    "Analyzing genome requirements for optimal ConnectomeManager sizing..."
                )
                requirements = self._analyze_genome_requirements(genome_data)

                # Ensure ConnectomeManager has adequate capacity
                if self._connectome_manager:
                    current_capacity = getattr(
                        self._connectome_manager, "max_neurons", 0
                    )
                    required_capacity = requirements[
                        "recommended_neuron_capacity"
                    ]

                    if current_capacity < required_capacity:
                        self.logger.error(
                            f"[GENOME] INSUFFICIENT CAPACITY: {current_capacity:,} < {required_capacity:,}"
                        )

                        # Try dynamic resize if available
                        if hasattr(
                            self._connectome_manager, "resize_for_genome"
                        ):
                            try:
                                self.logger.info(
                                    f"[GENOME] Attempting dynamic resize to {required_capacity:,} neurons..."
                                )
                                resize_result = (
                                    self._connectome_manager.resize_for_genome(
                                        genome_data
                                    )
                                )
                                if resize_result.get("success"):
                                    self.logger.info(
                                        f"[GENOME] ✅ Connectome resized: {resize_result.get('message', 'Success')}"
                                    )
                                else:
                                    self.logger.error(
                                        f"[GENOME] ❌ Connectome resize failed: {resize_result.get('message', 'Unknown')}"
                                    )
                                    if self.state_manager:
                                        from feagi.core.state_manager import (
                                            GenomeState,
                                        )

                                        self.state_manager.set_genome_state(
                                            GenomeState.ERROR
                                        )
                                    return {
                                        "success": False,
                                        "message": f"Insufficient capacity: need {required_capacity:,} neurons",
                                    }
                            except Exception as e:
                                self.logger.error(
                                    f"[GENOME] ❌ Connectome resize exception: {e}"
                                )
                                if self.state_manager:
                                    from feagi.core.state_manager import (
                                        GenomeState,
                                    )

                                    self.state_manager.set_genome_state(
                                        GenomeState.ERROR
                                    )
                                return {
                                    "success": False,
                                    "message": f"Resize failed: {e}",
                                }
                        else:
                            self.logger.error(
                                "[GENOME] ❌ ConnectomeManager doesn't support dynamic resizing"
                            )
                            if self.state_manager:
                                from feagi.core.state_manager import (
                                    GenomeState,
                                )

                                self.state_manager.set_genome_state(
                                    GenomeState.ERROR
                                )
                            return {
                                "success": False,
                                "message": f"Insufficient capacity: need {required_capacity:,} neurons, have {current_capacity:,}",
                            }
                    else:
                        self.logger.info(
                            f"[GENOME] ✅ Adequate capacity: {current_capacity:,} >= {required_capacity:,}"
                        )

                #  ARCHITECTURE IMPROVEMENT: Build brain from state manager's
                #  genome (not temp file)
                #  This ensures connectome manager always uses the sanitized
                #  genome from state manager
                self.logger.info(
                    "Building brain from state manager's sanitized genome..."
                )

                # Initialize embryogenesis
                embry = NeuroEmbryogenesis(
                    connectome_manager=self._connectome_manager,
                    progress_callback=self._handle_embryogenesis_progress,
                )

                #  CRITICAL: Develop brain from state manager's genome (single
                #  source of truth)
                # This includes the COMPLETE brain development process:
                # 1. Corticogenesis (cortical area creation)
                # 2. Voxelogenesis (spatial framework)
                # 3. Neurogenesis (neuron creation)
                #  4. Synaptogenesis (synapse formation) <- This is the
                #  long-running step
                try:
                    self.logger.info(
                        "Starting COMPLETE brain development from genome (including synaptogenesis)..."
                    )
                    #  ARCHITECTURE: Use hierarchical genome from single source
                    #  of truth
                    success = embry.develop_brain_from_genome_data(
                        self._current_genome
                    )

                    if not success:
                        error_msg = (
                            embry.error
                            or "Unknown error during brain development"
                        )
                        self.logger.error(
                            f"Failed to develop brain from genome: {error_msg}"
                        )

                        # Set error state since brain development failed
                        if self.state_manager:
                            from feagi.core.state_manager import GenomeState

                            self.state_manager.set_genome_state(
                                GenomeState.ERROR
                            )
                            self.state_manager.set_brain_readiness(False)
                            result = self.state_manager.set_genome_validity(
                                False
                            )
                            if result.is_err:
                                self.logger.warning(
                                    f"Failed to set genome validity: {result.unwrap_err()}"
                                )
                        return {
                            "success": False,
                            "error": f"Failed to develop brain from genome: {error_msg}",
                        }

                    #  Get development statistics - this includes completed
                    #  synaptogenesis
                    stats = embry.get_development_statistics()
                    self.logger.info(
                        f"COMPLETE brain development finished: {stats.get('total_neurons', 0)} neurons, {stats.get('total_synapses', 0)} synapses"
                    )

                    #  CRITICAL: Apply genome's simulation_timestep to system
                    #  configuration
                    self._apply_genome_physiology_parameters(
                        self._current_genome, self._core_api_service
                    )

                    #  CRITICAL: Set genome state to LOADED only after complete
                    #  brain development
                    #  This ensures genome is marked as loaded ONLY when
                    #  everything is truly complete
                    from feagi.core.state_manager import GenomeState

                    self.state_manager.set_genome_state(GenomeState.LOADED)
                    self.logger.info(
                        "Genome state set to LOADED - COMPLETE brain development finished (including synaptogenesis)"
                    )

                    #  STEP 3: After complete brain development, set final
                    #  states
                    self.logger.info(
                        "Setting final genome and brain states..."
                    )

                    # Set brain readiness to true - genome loading is complete
                    self.state_manager.set_brain_readiness(True)
                    self.logger.info(
                        "✅ Brain readiness set to True - complete genome loaded"
                    )

                    #  CRITICAL FIX: Set connectome state to READY after
                    #  successful brain development
                    #  This ensures API endpoints work correctly without
                    #  requiring ProcessManager
                    from feagi.core.state_manager import ConnectomeState

                    self.state_manager.set_connectome_state(
                        ConnectomeState.READY
                    )
                    self.logger.info(
                        "✅ Connectome state set to READY - API endpoints now functional"
                    )

                    # Log current burst engine state for monitoring

                    current_burst_state = (
                        self.state_manager.get_burst_engine_state()
                    )
                    self.logger.info(
                        f"📊 Current burst engine state: {current_burst_state}"
                    )

                    #  The process manager will detect the state changes and
                    #  handle service startup
                    self.logger.info(
                        "🎯 Genome loading complete - process manager will handle service coordination"
                    )

                    #  CRITICAL: Update burst engine with new genome directly
                    #  since event system is not available
                    try:
                        from feagi.npu.burst_engine import BurstEngine

                        burst_engine = BurstEngine.get_instance()
                        if burst_engine:
                            burst_engine.update_with_genome()
                            self.logger.info(
                                "✅ Burst engine updated with new genome successfully"
                            )
                        else:
                            self.logger.warning(
                                "⚠️ Burst engine instance not available for genome update"
                            )
                    except Exception as burst_error:
                        self.logger.warning(
                            f"Failed to update burst engine with genome: {burst_error}"
                        )
                        #  Don't fail genome loading for burst engine update
                        #  issues

                    #  CRITICAL: Emit GENOME_LOADED event to trigger burst
                    #  engine startup (fallback)
                    try:
                        from feagi.utils.event_system import (
                            EventPriority,
                            EventType,
                            emit_event,
                        )

                        # Get cortical area count for event data
                        current_cortical_area_count = len(
                            getattr(
                                self._connectome_manager, "cortical_areas", {}
                            )
                        )
                        success = emit_event(
                            EventType.GENOME_LOADED,
                            data={
                                "filename": filename,
                                "cortical_areas": current_cortical_area_count,
                            },
                            priority=EventPriority.HIGH,
                        )
                        if success:
                            self.logger.info(
                                f"📡 GENOME_LOADED event emitted for '{filename}'"
                            )
                        else:
                            self.logger.warning(
                                "Failed to emit GENOME_LOADED event - using direct burst engine update instead"
                            )
                    except Exception as event_error:
                        self.logger.warning(
                            f"Failed to emit GENOME_LOADED event: {event_error}"
                        )
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
                            self.logger.warning(
                                f"Failed to set genome validity: {result.unwrap_err()}"
                            )
                    return {
                        "success": False,
                        "error": f"Exception during brain development: {str(dev_error)}",
                    }

                #  CRITICAL: Update state manager with comprehensive brain
                #  statistics for health checks
                if self.state_manager:
                    #  Update state manager with comprehensive brain statistics
                    #  for health checks
                    try:
                        # Get statistics from connectome manager
                        cortical_area_count = len(
                            getattr(
                                self._connectome_manager, "cortical_areas", {}
                            )
                        )

                        # Calculate neuron and synapse counts if methods exist
                        total_neurons = 0
                        total_synapses = 0

                        if hasattr(
                            self._connectome_manager, "get_total_neuron_count"
                        ):
                            total_neurons = (
                                self._connectome_manager.get_total_neuron_count()
                            )
                        elif hasattr(
                            self._connectome_manager, "cortical_areas"
                        ):
                            # Fallback: count neurons in all cortical areas
                            for (
                                area_idx
                            ) in self._connectome_manager.cortical_areas:
                                try:
                                    if hasattr(
                                        self._connectome_manager,
                                        "get_neurons_by_area",
                                    ):
                                        area_neurons = self._connectome_manager.get_neurons_by_area(
                                            area_idx
                                        )
                                        total_neurons += (
                                            len(area_neurons)
                                            if area_neurons
                                            else 0
                                        )
                                except Exception:
                                    pass

                        if hasattr(
                            self._connectome_manager, "get_total_synapse_count"
                        ):
                            total_synapses = (
                                self._connectome_manager.get_total_synapse_count()
                            )

                        #  Update state manager with brain statistics (CRITICAL
                        #  for health check)
                        brain_stats_to_set = {
                            "neuron_count": total_neurons,
                            "synapse_count": total_synapses,
                            "cortical_area_count": cortical_area_count,
                        }
                        self.logger.debug(f"Setting brain stats after genome loading: {brain_stats_to_set}")
                        stats_result = self.state_manager.set_brain_stats(brain_stats_to_set)
                        if stats_result.is_err:
                            self.logger.warning(
                                f"Failed to set brain stats: {stats_result.unwrap_err()}"
                            )
                        else:
                            self.logger.info(f"Brain stats set successfully: {brain_stats_to_set}")

                        #  Create cortical list for health check compatibility
                        #  (CRITICAL)
                        cortical_ids = []
                        if hasattr(self._connectome_manager, "cortical_areas"):
                            for (
                                area_idx,
                                area,
                            ) in (
                                self._connectome_manager.cortical_areas.items()
                            ):
                                #  Try to get cortical_id from area object,
                                #  fallback to string representation
                                if (
                                    hasattr(area, "cortical_id")
                                    and area.cortical_id
                                ):
                                    cortical_ids.append(area.cortical_id)
                                else:
                                    cortical_ids.append(f"CID{area_idx:03d}")
                        cortical_result = self.state_manager.set_cortical_list(
                            cortical_ids
                        )
                        if cortical_result.is_err:
                            self.logger.warning(
                                f"Failed to set cortical list: {cortical_result.unwrap_err()}"
                            )

                        #  Set genome validity based on earlier validation
                        #  results
                        if (
                            not hasattr(self.state_manager, "genome_validity")
                            or self.state_manager.genome_validity is None
                        ):
                            validity_result = (
                                self.state_manager.set_genome_validity(True)
                            )
                            if validity_result.is_err:
                                self.logger.warning(
                                    f"Failed to set genome validity: {validity_result.unwrap_err()}"
                                )

                        # Ensure other state manager attributes are initialized
                        if (
                            not hasattr(self.state_manager, "connected_agents")
                            or self.state_manager.connected_agents is None
                        ):
                            self.state_manager.set_agent_count(0)

                        if not hasattr(
                            self.state_manager, "changes_saved_externally"
                        ):
                            self.state_manager.changes_saved_externally = False

                        if not hasattr(self.state_manager, "exit_condition"):
                            #  NOTE: exit_condition doesn't have a proper
                            #  setter method yet
                            #  This should be handled through proper state
                            #  management
                            result = self.state_manager.set_exit_condition(
                                False
                            )
                            if result.is_err:
                                self.logger.warning(
                                    "Failed to set exit condition"
                                )
                                # Continue anyway - this is not critical

                        self.logger.info(
                            f"State manager fully synchronized: {total_neurons} neurons, {total_synapses} synapses, {cortical_area_count} cortical areas"
                        )

                    except Exception as stats_error:
                        #  CRITICAL FIX: Do NOT reset brain_readiness to False
                        #  here!
                        #  Statistics updating is NOT critical for brain
                        #  functionality
                        self.logger.error(
                            f"WARNING: Error updating state manager statistics: {str(stats_error)}"
                        )
                        self.logger.warning(
                            "Statistics update failed but genome loading succeeded - brain is still functional"
                        )
                        # Don't fail genome loading for statistics issues
                        #  Don't touch brain_readiness or genome_state -
                        #  they're already correctly set

                #  Get cortical area count from connectome manager for return
                #  value
                cortical_area_count = len(
                    getattr(self._connectome_manager, "cortical_areas", {})
                )

                #  CRITICAL: Only increment genome counter for ACTUALLY NEW
                #  genomes
                if self.state_manager:
                    old_genome_counter = (
                        self.state_manager.get_genome_counter()
                    )

                    #  Check if this is genuinely a NEW genome (different from
                    #  what we had before)
                    is_new_genome = False
                    if (
                        not old_genome_data
                        or old_genome_data != self._current_genome
                        or old_genome_filename != filename
                    ):
                        #  Update timestamp to signal change to downstream
                        #  clients
                        new_genome_timestamp = int(
                            time.time() * 1000
                        )  # milliseconds
                        self.state_manager.set_genome_timestamp(
                            new_genome_timestamp
                        )
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
                            json.dumps(
                                old_genome_data, sort_keys=True
                            ).encode()
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

                    #  Only increment counter and update timestamp for
                    #  genuinely new genomes
                    if is_new_genome:
                        self.state_manager.increment_genome_counter()
                        current_genome_number = (
                            self.state_manager.get_genome_counter()
                        )
                        self.logger.info(
                            f"[OK] Genome counter incremented to {current_genome_number}"
                        )

                        new_genome_timestamp = int(
                            time.time() * 1000
                        )  # milliseconds
                        self.state_manager.set_genome_timestamp(
                            new_genome_timestamp
                        )
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

                #  DYNAMIC SIZING: Resize connectome based on genome
                #  requirements
                if self._connectome_manager and hasattr(
                    self._connectome_manager, "resize_for_genome"
                ):
                    try:
                        resize_success = (
                            self._connectome_manager.resize_for_genome(
                                genome_data
                            )
                        )
                        if resize_success:
                            self.logger.info(
                                "✅ [DYNAMIC SIZING] Connectome resized successfully based on genome requirements"
                            )
                        else:
                            self.logger.info(
                                "ℹ️  [DYNAMIC SIZING] Connectome resize not needed - current size is optimal"
                            )
                    except Exception as resize_error:
                        self.logger.warning(
                            f"⚠️  [DYNAMIC SIZING] Error during connectome resize: {resize_error}"
                        )
                        #  Don't fail genome loading for resize issues - it's
                        #  an optimization, not critical

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

                #  Return success with detailed information including
                #  validation status
                result = {
                    "success": True,
                    "cortical_area_count": cortical_area_count,
                    "message": "Genome loaded and state manager fully synchronized",
                }

                #  Include validation errors in response if validation failed
                #  but loading succeeded
                if not validation_result["valid"]:
                    result["validation_errors"] = validation_result.get(
                        "errors", []
                    )
                    result["genome_validity"] = False

                    # Check if auto-recovery was performed
                    if auto_recovery_details.get("recovery_performed", False):
                        result["message"] = (
                            f"Genome loaded with auto-recovery: {auto_recovery_details.get('recovery_summary', 'Auto-recovery performed')}"
                        )
                        result["auto_recovery_performed"] = True
                        result["removed_morphologies"] = (
                            auto_recovery_details.get(
                                "removed_morphologies", []
                            )
                        )
                        result["fixed_references"] = auto_recovery_details.get(
                            "fixed_references", []
                        )
                        # Include validation warnings from auto-recovery
                        result["validation_warnings"] = (
                            auto_recovery_details.get(
                                "validation_warnings", []
                            )
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
                    #  Even if validation passed, include warnings from
                    #  auto-recovery if any corrections were made
                    if auto_recovery_details.get("recovery_performed", False):
                        result["validation_warnings"] = (
                            auto_recovery_details.get(
                                "validation_warnings", []
                            )
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
                        self.logger.warning(
                            f"Failed to set genome validity: {result.unwrap_err()}"
                        )

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
                    self.logger.warning(
                        f"Failed to set genome validity: {result.unwrap_err()}"
                    )

            return {"success": False, "error": str(e)}

    def _handle_embryogenesis_progress(self, stage, percentage, message):
        """Handle progress updates from the neuroembryogenesis process."""
        self.logger.info(
            f"{stage} {percentage:.1f}% - {message}", status="[PROC]"
        )

        # Update state manager with development stage
        if self.state_manager:
            from feagi.bdu.embryogenesis.neuroembryogenesis import (
                DevelopmentStage,
            )

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
            if hasattr(self.state_manager._state, "neuroembryogenesis_stage"):
                self.state_manager._state.neuroembryogenesis_stage = (
                    stage_value
                )
                self.state_manager._state.neuroembryogenesis_progress = int(
                    percentage
                )

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
            for (
                cortical_id,
                area,
            ) in self._connectome_manager.cortical_areas.items():
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
                            "title": genome_data.get(
                                "genome_title", "Untitled Genome"
                            ),
                            "description": genome_data.get(
                                "genome_description", ""
                            ),
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
            self.logger.error(
                f"Error retrieving evolution change register: {str(e)}"
            )
            return {}

    def deploy_genome(self, genome_filepath: str) -> bool:
        """Deploy a genome from a file path."""
        try:
            self.logger.info(
                f"Deploying genome from {genome_filepath}", status="[DNA]"
            )

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

            #  Update state to LOADED - this is already done in load_genome but
            #  we do it again for safety
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
                f"Invalid JSON in genome file: {genome_filepath}",
                status="[ERR]",
            )

            # Update state to ERROR
            if self.state_manager:
                from feagi.core.state_manager import GenomeState

                self.state_manager.set_genome_state(GenomeState.ERROR)
                self.state_manager.set_brain_readiness(False)

            return False
        except Exception as e:
            self.logger.error(
                f"Error deploying genome: {str(e)}", status="[ERR]"
            )

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

        #  If state manager says no genome, but connectome has areas, consider
        #  it loaded
        #  This handles cases where genome was loaded but state wasn't properly
        #  synced
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
        """Load a genome from the default templates directory.

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

            self.logger.debug(
                f"GENOME SERVICE: Found genome file at: {genome_path}"
            )
            self.logger.info(
                f"Loading {genome_name} genome from {genome_path}"
            )

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
            current_dir
            / "../../../../evo/defaults/genome",  # Relative to this file
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
    #  These methods handle cortical area modifications through proper data
    #  flow:
    #  API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis
    #  → ConnectomeManager

    def create_cortical_area(
        self,
        name: str,
        coordinates: Dict[str, int],
        dimensions: Dict[str, int],
        area_type: str,
        parameters: Dict[str, Any] = None,
    ) -> Optional[Dict[str, Any]]:
        """Create a new cortical area through proper genome modification
        pipeline.

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
                self.logger.error(
                    "Cannot create cortical area: No genome loaded"
                )
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

                #  Use provided cortical_id or generate a unique one using
                #  FEAGI's standard format
                if parameters and "cortical_id" in parameters:
                    cortical_id = parameters["cortical_id"]
                    # Remove cortical_id from parameters to avoid duplication
                    parameters = {
                        k: v
                        for k, v in parameters.items()
                        if k != "cortical_id"
                    }
                else:
                    #  Fallback: generate a unique cortical area ID using
                    #  FEAGI's standard format
                    import random
                    import string

                    try:
                        from feagi.api.v1.utils import generate_cortical_id
                    except ImportError:
                        # Fallback if utils module not available
                        def generate_cortical_id(prefix="C", seed="ABC"):
                            return f"{prefix}{seed}"

                    existing_ids = set(current_genome["blueprint"].keys())
                    attempts = 0
                    while attempts < 100:  # Prevent infinite loop
                        # Generate a random 3-character seed for the ID
                        seed = "".join(
                            random.choices(
                                string.ascii_uppercase + string.digits, k=3
                            )
                        )
                        cortical_id = generate_cortical_id(
                            prefix="C", seed=seed
                        )
                        if cortical_id not in existing_ids:
                            break
                        attempts += 1

                    if attempts >= 100:
                        raise ValueError(
                            "Failed to generate unique cortical area ID after 100 attempts"
                        )

                # Import cortical template for proper defaults
                from feagi.evo.templates import (
                    cortical_template,
                    cortical_template_memory,
                )

                # Check if this is a memory cortical area
                is_memory_area = (
                    parameters and parameters.get("sub_group_id") == "MEMORY"
                )

                # Choose appropriate template
                if is_memory_area:
                    template = cortical_template_memory
                    self.logger.info(
                        f"Creating memory cortical area {cortical_id} with memory template"
                    )
                else:
                    template = cortical_template

                #  Create new cortical area definition in hierarchical format
                #  with template defaults
                new_area = {
                    "cortical_id": cortical_id,
                    "cortical_name": name,
                    "coordinates_3d": coordinates,
                    "cortical_dimensions": dimensions,
                    "cortical_type": area_type,
                    "parameters": parameters or {},
                    # CRITICAL FIX: Add properties in the format expected by _extract_cortical_properties
                    "relative_coordinate": [coordinates["x"], coordinates["y"], coordinates["z"]],
                    "block_boundaries": [dimensions["width"], dimensions["height"], dimensions["depth"]],
                    "2d_coordinate": parameters.get("coordinates_2d", [0, 0]) if parameters else [0, 0],
                }

                # Apply template defaults to the new area
                for key, default_value in template.items():
                    if key not in new_area:
                        new_area[key] = default_value

                # Override with any provided parameters
                if parameters:
                    new_area.update(parameters)
                # Initialize embryogenesis for subsequent operations
                from feagi.bdu.embryogenesis.neuroembryogenesis import (
                    NeuroEmbryogenesis,
                )
                embryogenesis = NeuroEmbryogenesis(
                    self._connectome_manager, self.state_manager
                )
                # CRITICAL FIX: Ensure group_id is set for proper classification
                if "group_id" not in new_area:
                    if is_memory_area:
                        new_area["group_id"] = "MEMORY"
                    else:
                        new_area["group_id"] = parameters.get("cortical_group", "CUSTOM") if parameters else "CUSTOM"
                # CRITICAL FIX: Only memory areas need memory properties
                # Non-memory areas should not have memory properties
                if is_memory_area:
                    memory_defaults = {
                        "is_mem_type": True,
                        "longterm_mem_threshold": 100,
                        "lifespan_growth_rate": 1,
                        "init_lifespan": 9,
                    }
                    
                    # Add memory properties to memory areas only
                    for key, value in memory_defaults.items():
                        if key not in new_area:
                            new_area[key] = value

                # Additional memory area configuration (already handled above)
                if is_memory_area:
                    # Ensure temporal_depth is set for memory areas
                    if "temporal_depth" not in new_area:
                        new_area["temporal_depth"] = 1
                    # Ensure sub_group_id is set for memory areas
                    if "sub_group_id" not in new_area:
                        new_area["sub_group_id"] = "MEMORY"
                    # Update parameters dict as well for consistency
                    new_area["parameters"].update({
                        "sub_group_id": "MEMORY",
                        "temporal_depth": new_area.get("temporal_depth", 1),
                        "init_lifespan": new_area.get("init_lifespan", 9),
                        "lifespan_growth_rate": new_area.get("lifespan_growth_rate", 1),
                        "longterm_mem_threshold": new_area.get("longterm_mem_threshold", 100),
                    })

                # Add to hierarchical blueprint structure
                current_genome["blueprint"][cortical_id] = new_area

                # Update the genome through proper pipeline
                self._current_genome = current_genome

                # Update genome in state manager (single source of truth)
                if self.state_manager:
                    self.state_manager.genome = current_genome

                # Create the cortical area directly in ConnectomeManager
                #  ARCHITECTURE: Don't rebuild entire brain - just add the new
                #  area to preserve existing neuron associations
                try:
                    #  Use the enhanced parameters that include memory template
                    #  properties
                    enhanced_properties = new_area.get("parameters", {})

                    # Enforce memory-area semantics
                    if is_memory_area:
                        create_dims = (1, 1, 1)
                    else:
                        create_dims = (
                            dimensions["width"],
                            dimensions["height"],
                            dimensions["depth"],
                        )

                    self._connectome_manager.add_cortical_area(
                            name=name,
                            dimensions=tuple(create_dims),
                            position=tuple(
                                [
                                    coordinates["x"],
                                    coordinates["y"],
                                    coordinates["z"],
                                ]
                            ),
                            area_type=area_type,
                            properties=enhanced_properties,
                            cortical_id=cortical_id,
                        )

                    # MEMORY AREA: Do not create regular neurons; register with memory systems only
                    if is_memory_area:
                        temporal_depth = new_area.get("temporal_depth", 1)
                        memory_registered = (
                            self._connectome_manager.register_memory_area(
                                cortical_id=cortical_id,
                                temporal_depth=temporal_depth,
                            )
                        )
                        if memory_registered:
                            self.logger.info(
                                f"✅ Registered memory area {cortical_id} with temporal_depth={temporal_depth}"
                            )

                            # Register with BurstEngine MemoryProcessor as well
                            try:
                                from feagi.npu.burst_engine import BurstEngine

                                burst_engine = BurstEngine.get_instance()
                                if burst_engine:
                                    memory_properties = {
                                        "temporal_depth": temporal_depth,
                                        "init_lifespan": new_area.get(
                                            "init_lifespan", 9
                                        ),
                                        "lifespan_growth_rate": new_area.get(
                                            "lifespan_growth_rate", 1.0
                                        ),
                                        "longterm_mem_threshold": new_area.get(
                                            "longterm_mem_threshold", 100
                                        ),
                                    }
                                    processor_registered = burst_engine.register_memory_area_with_processor(
                                        cortical_id, memory_properties
                                    )
                                    if processor_registered:
                                        self.logger.info(
                                            f"✅ Registered memory area {cortical_id} with MemoryProcessor"
                                        )
                                    else:
                                        self.logger.warning(
                                            f"⚠️  Failed to register memory area {cortical_id} with MemoryProcessor"
                                        )
                                else:
                                    self.logger.warning(
                                        "⚠️  BurstEngine instance not available for memory area registration"
                                    )
                            except Exception as burst_error:
                                self.logger.warning(
                                    f"Failed to register memory area with BurstEngine: {burst_error}"
                                )

                        # Prepare return payload for memory area (no regular neurons)
                        success = True
                        if success and transaction:
                            transaction.commit()
                        elif transaction:
                            transaction.rollback()
                            return None

                        return {
                            "cortical_id": cortical_id,
                            "name": name,
                            "coordinates": coordinates,
                            "dimensions": {"width": 1, "height": 1, "depth": 1},
                            "type": area_type,
                            "parameters": parameters or {},
                            "neuron_count": 0,
                            "excitability": None,
                        }

                    # NON-MEMORY AREA: proceed with regular neuron creation
                    #  Extract proper neuron properties from the cortical area template
                    area = self._connectome_manager.cortical_areas[cortical_id]
                    width, height, depth = area.dimensions
                    neurons_per_voxel = new_area.get("per_voxel_neuron_cnt", 1)
                    area_neuron_count = width * height * depth * neurons_per_voxel

                    # Generate positions
                    positions = []
                    for z in range(depth):
                        for y in range(height):
                            for x in range(width):
                                for _ in range(neurons_per_voxel):
                                    positions.append((x, y, z))

                    base_threshold = new_area.get("firing_threshold", 1.0)
                    base_decay_rate = 1.0 - (new_area.get("leak_coefficient", 0) / 100.0)
                    base_refractory = new_area.get("refractory_period", 1)
                    excitability = new_area.get("neuron_excitability", 1.0)

                    self.logger.info(
                        f"Creating neurons with properties: threshold={base_threshold}, decay_rate={base_decay_rate}, refractory={base_refractory}, excitability={excitability}"
                    )

                    neuron_ids = self._connectome_manager.batch_create_neurons(
                        cortical_id=cortical_id,
                        positions=positions,
                        threshold=base_threshold,
                        membrane_potential=0.0,
                        resting_potential=0.0,
                        decay_rate=base_decay_rate,
                        refractory_period=base_refractory,
                    )

                    # Update per-area excitability cache in NPU (per-area, not per-neuron)
                    area_ex = float(new_area.get("neuron_excitability", 1.0))
                    try:
                        npu = getattr(self._connectome_manager, "_npu_interface", None)
                        if npu and hasattr(npu, "set_area_excitability"):
                            cidx = self._connectome_manager.cortical_mapping.get_idx(cortical_id)
                            if cidx is not None:
                                npu.set_area_excitability(cidx, area_ex)
                    except Exception:
                        pass

                    self.logger.info(
                        f"✅ Created cortical area {cortical_id} with {area_neuron_count} neurons, proper position mapping, and excitability={area_ex}"
                    )

                    # Register as memory area if needed
                    if is_memory_area:
                        temporal_depth = new_area.get("temporal_depth", 1)
                        memory_registered = (
                            self._connectome_manager.register_memory_area(
                                cortical_id=cortical_id,
                                temporal_depth=temporal_depth,
                            )
                        )
                        if memory_registered:
                            self.logger.info(
                                f"✅ Registered memory area {cortical_id} with temporal_depth={temporal_depth}"
                            )

                            #  CRITICAL: Also register with BurstEngine
                            #  MemoryProcessor
                            try:
                                from feagi.npu.burst_engine import BurstEngine

                                burst_engine = BurstEngine.get_instance()
                                if burst_engine:
                                    #  Prepare memory area properties for
                                    #  BurstEngine registration
                                    memory_properties = {
                                        "temporal_depth": temporal_depth,
                                        "init_lifespan": new_area.get(
                                            "init_lifespan", 9
                                        ),
                                        "lifespan_growth_rate": new_area.get(
                                            "lifespan_growth_rate", 1.0
                                        ),
                                        "longterm_mem_threshold": new_area.get(
                                            "longterm_mem_threshold", 100
                                        ),
                                    }

                                    processor_registered = burst_engine.register_memory_area_with_processor(
                                        cortical_id, memory_properties
                                    )
                                    if processor_registered:
                                        self.logger.info(
                                            f"✅ Registered memory area {cortical_id} with MemoryProcessor"
                                        )
                                    else:
                                        self.logger.warning(
                                            f"⚠️  Failed to register memory area {cortical_id} with MemoryProcessor"
                                        )
                                else:
                                    self.logger.warning(
                                        "⚠️  BurstEngine instance not available for memory area registration"
                                    )
                            except Exception as burst_error:
                                self.logger.warning(
                                    f"Failed to register memory area with BurstEngine: {burst_error}"
                                )
                        else:
                            self.logger.warning(
                                f"⚠️  Failed to register memory area {cortical_id}"
                            )

                    success = True

                except Exception as create_error:
                    self.logger.error(
                        f"Error creating cortical area {cortical_id}: {str(create_error)}"
                    )
                    # Clean up on failure
                    if cortical_id in self._connectome_manager.cortical_areas:
                        del self._connectome_manager.cortical_areas[
                            cortical_id
                        ]
                    success = False

                if success and transaction:
                    transaction.commit()
                elif transaction:
                    transaction.rollback()
                    return None

                if success:
                    self.logger.info(
                        f"Created cortical area: {cortical_id} ({name}) with template properties"
                    )
                    return {
                        "cortical_id": cortical_id,
                        "name": name,
                        "coordinates": coordinates,
                        "dimensions": dimensions,
                        "type": area_type,
                        "parameters": parameters or {},
                        "neuron_count": area_neuron_count,
                        "excitability": excitability,
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
        """Update an existing cortical area with intelligent routing for
        optimal performance.

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
                self.logger.error(
                    "Cannot update cortical area: No genome loaded"
                )
                return None

            if cortical_id not in self._current_genome["blueprint"]:
                self.logger.error(
                    f"Cannot update cortical area: {cortical_id} not found in genome"
                )
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
                    self.logger.warning(
                        f"No changes provided for cortical area {cortical_id}"
                    )
                    if transaction:
                        transaction.rollback()
                    return self._current_genome["blueprint"][cortical_id]

                # INTELLIGENT ROUTING: Classify changes for optimal performance
                from feagi.api.core.services.genome.change_classifier import (
                    ChangeType,
                    CorticalChangeClassifier,
                )

                change_type = CorticalChangeClassifier.classify_changes(
                    changes
                )
                CorticalChangeClassifier.log_classification_result(
                    changes, change_type
                )

                # ROUTE BASED ON CHANGE TYPE for optimal performance
                if change_type == ChangeType.PARAMETER:
                    # FAST PATH: Direct parameter updates only (~2-5ms)
                    result = self._update_parameters_only(
                        cortical_id, changes, transaction
                    )

                elif change_type == ChangeType.METADATA:
                    # FASTEST PATH: Metadata updates only (~1ms)
                    result = self._update_metadata_only(
                        cortical_id, changes, transaction
                    )

                elif change_type == ChangeType.STRUCTURAL:
                    # LOCALIZED REBUILD PATH: Structural changes (~100-200ms)
                    result = self._update_with_localized_rebuild(
                        cortical_id, changes, transaction
                    )

                elif change_type == ChangeType.HYBRID:
                    #  HYBRID PATH: Optimized combination (uses localized
                    #  rebuild for structural parts)
                    result = self._update_hybrid(
                        cortical_id, changes, transaction
                    )

                else:
                    # Fallback to safe localized rebuild (avoid global rebuild)
                    self.logger.warning(
                        f"Unknown change type {change_type}, using localized rebuild"
                    )
                    result = self._update_with_localized_rebuild(
                        cortical_id, changes, transaction
                    )

                # Log performance metrics (API debug only)
                duration = time.time() - start_time
                try:
                    if self.state_manager.is_debug_api_enabled():
                        self.logger.info(
                            f"[API-DEBUG] {cortical_id} updated via {change_type.value} "
                            f"path in {duration * 1000:.1f}ms"
                        )
                except Exception:
                    pass

                return result

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error updating cortical area: {str(e)}")
            return None

    def delete_cortical_area(self, cortical_id: str) -> bool:
        """Delete a cortical area through proper genome modification pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures cortical area modifications
        go through the proper data flow to maintain genome consistency.

        Args:
            cortical_id: ID of cortical area to delete

        Returns:
            bool: True if area was deleted successfully
        """
        try:
            if not self.is_genome_loaded():
                self.logger.error(
                    "Cannot delete cortical area: No genome loaded"
                )
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

                #  Apply the cortical area deletion by triggering brain
                #  development
                #  ARCHITECTURE: Pass hierarchical genome directly (single
                #  source of truth)
                # NeuroEmbryogenesis now supports hierarchical format natively
                success = embryogenesis.develop_brain_from_genome_data(
                    current_genome
                )

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
    #  API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis
    #  → ConnectomeManager

    def create_morphology(self, morphology_data: Dict[str, Any]) -> bool:
        """Create a new morphology through proper genome modification pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures morphology modifications
        go through the proper data flow to maintain genome consistency.

        Args:
            morphology_data: Dictionary containing morphology definition
                           Must include: name, type, parameters
                           Optional: dimension_sensitive (bool)

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
                    "dimension_sensitive": morphology_data.get(
                        "dimension_sensitive", False
                    ),  # Include dimension_sensitive field
                }

                # Update the genome through proper pipeline
                self._current_genome = current_genome

                # Update genome in state manager (single source of truth)
                if self.state_manager:
                    self.state_manager.genome = current_genome

                #  Morphology is now available in genome - NeuroEmbryogenesis
                #  will automatically
                #  pick it up through get_morphology_registry() when needed for
                #  synaptogenesis

                if transaction:
                    transaction.commit()

                self.logger.info(
                    f"Successfully created morphology '{name}' of type '{morphology_data['type']}'"
                )
                return True

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error creating morphology: {str(e)}")
            return False

    def update_morphology(
        self, morphology_id: str, updates: Dict[str, Any]
    ) -> bool:
        """Update an existing morphology through proper genome modification
        pipeline.

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
                    or morphology_id
                    not in current_genome["neuron_morphologies"]
                ):
                    self.logger.warning(
                        f"Morphology '{morphology_id}' not found in genome"
                    )
                    return False

                morphology = current_genome["neuron_morphologies"][
                    morphology_id
                ]

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

                # Apply the morphology update by triggering brain development
                #  ARCHITECTURE: Pass hierarchical genome directly (single
                #  source of truth)
                success = embryogenesis.develop_brain_from_genome_data(
                    current_genome
                )

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

    def _check_morphology_usage(
        self, genome: Dict[str, Any], morphology_id: str
    ) -> Dict[str, List[str]]:
        """Check if a morphology is being used anywhere in the genome.

        COMPREHENSIVE SAFETY CHECK: Scans all genome sections for morphology references
        to prevent deletion of morphologies that are still in use.

        Args:
            genome: The genome dictionary to scan
            morphology_id: The morphology ID to search for

        Returns:
            Dictionary with usage locations:
            {
                "cortical_areas": ["area1", "area2", ...],
                "cortical_mappings": ["mapping1", "mapping2", ...],
                "blueprints": ["blueprint1", "blueprint2", ...]
            }
        """
        usage_locations = {
            "cortical_areas": [],
            "cortical_mappings": [],
            "blueprints": [],
        }

        try:
            # Check cortical areas for morphology usage
            if "blueprint" in genome:
                for area_id, area_data in genome["blueprint"].items():
                    if isinstance(area_data, dict):
                        # Check direct neuron_morphology reference
                        if area_data.get("neuron_morphology") == morphology_id:
                            usage_locations["cortical_areas"].append(area_id)

                        # Check in cortical parameters if they exist
                        if "cortical_parameters" in area_data:
                            params = area_data["cortical_parameters"]
                            if (
                                isinstance(params, dict)
                                and params.get("neuron_morphology")
                                == morphology_id
                            ):
                                usage_locations["cortical_areas"].append(
                                    area_id
                                )

            # Check cortical mappings for morphology usage
            if "cortical_mappings" in genome:
                for mapping_id, mapping_data in genome[
                    "cortical_mappings"
                ].items():
                    if isinstance(mapping_data, dict):
                        # Check mapping parameters for morphology references
                        if "parameters" in mapping_data:
                            params = mapping_data["parameters"]
                            if isinstance(params, dict):
                                #  Check various parameter fields that might
                                #  reference morphologies
                                for param_key, param_value in params.items():
                                    if param_value == morphology_id:
                                        usage_locations[
                                            "cortical_mappings"
                                        ].append(f"{mapping_id}:{param_key}")

            # Check any other blueprint sections
            if "blueprint" in genome:
                blueprint = genome["blueprint"]
                if isinstance(blueprint, dict):
                    for blueprint_key, blueprint_data in blueprint.items():
                        if isinstance(blueprint_data, dict):
                            #  Deep scan for morphology references in blueprint
                            #  data
                            self._scan_dict_for_morphology(
                                blueprint_data,
                                morphology_id,
                                blueprint_key,
                                usage_locations["blueprints"],
                            )

            self.logger.debug(
                f"Morphology usage check for '{morphology_id}': {usage_locations}"
            )

        except Exception as e:
            self.logger.error(f"Error checking morphology usage: {e}")
            #  Return empty dict on error to be safe (prevents deletion if we
            #  can't verify safety)
            return {"error": [f"Could not verify morphology safety: {str(e)}"]}

        # Remove duplicates and return only non-empty categories
        filtered_usage = {}
        for category, items in usage_locations.items():
            unique_items = list(set(items))
            if unique_items:
                filtered_usage[category] = unique_items

        return filtered_usage

    def _scan_dict_for_morphology(
        self,
        data: Dict[str, Any],
        morphology_id: str,
        context_key: str,
        usage_list: List[str],
    ) -> None:
        """Recursively scan a dictionary for morphology references.

        Args:
            data: Dictionary to scan
            morphology_id: Morphology ID to search for
            context_key: Context for where this reference was found
            usage_list: List to append findings to
        """
        if not isinstance(data, dict):
            return

        for key, value in data.items():
            if value == morphology_id:
                usage_list.append(f"{context_key}:{key}")
            elif isinstance(value, dict):
                self._scan_dict_for_morphology(
                    value, morphology_id, f"{context_key}:{key}", usage_list
                )
            elif isinstance(value, list):
                for i, item in enumerate(value):
                    if item == morphology_id:
                        usage_list.append(f"{context_key}:{key}[{i}]")
                    elif isinstance(item, dict):
                        self._scan_dict_for_morphology(
                            item,
                            morphology_id,
                            f"{context_key}:{key}[{i}]",
                            usage_list,
                        )

    def delete_morphology(self, morphology_id: str) -> bool:
        """Delete a morphology through proper genome modification pipeline.

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
                    or morphology_id
                    not in current_genome["neuron_morphologies"]
                ):
                    self.logger.warning(
                        f"Morphology '{morphology_id}' not found in genome"
                    )
                    return False

                morphology = current_genome["neuron_morphologies"][
                    morphology_id
                ]

                # Don't allow deleting core morphologies
                if morphology.get("source") == "core":
                    raise ValueError("Cannot delete core morphologies")

                #  Don't allow deleting function morphologies (critical system
                #  components)
                if morphology.get("type") == "function":
                    raise ValueError(
                        f"Cannot delete function morphology '{morphology_id}' - "
                        f"function morphologies are critical system components and cannot be removed"
                    )

                #  CRITICAL SAFETY CHECK: Verify morphology is not in use
                #  before deletion
                usage_locations = self._check_morphology_usage(
                    current_genome, morphology_id
                )
                if usage_locations:
                    # Build detailed error message listing all usage locations
                    usage_details = []
                    for location_type, items in usage_locations.items():
                        if items:
                            usage_details.append(
                                f"{location_type}: {', '.join(items)}"
                            )

                    usage_summary = "; ".join(usage_details)
                    raise ValueError(
                        f"Cannot delete morphology '{morphology_id}' - it is currently in use. "
                        f"Found {sum(len(items) for items in usage_locations.values())} usage(s): {usage_summary}. "
                        f"Remove all references to this morphology before deleting it."
                    )

                # Safe to remove from genome structure
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

                # The morphology has been removed from the genome structure
                # No additional embryogenesis processing needed for deletion
                success = True

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
    #  These methods handle cortical mapping modifications through proper data
    #  flow:
    #  API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis
    #  → ConnectomeManager

    def update_cortical_mapping(self, mapping: Dict[str, Any]) -> bool:
        """Update cortical mapping through proper genome modification pipeline.

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
                self.logger.error(
                    "Cannot update cortical mapping: No genome loaded"
                )
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

                #  Convert the formatted mapping data back to the genome array
                #  format
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
                                #  Expected array: [morphology_id, scalar,
                                #  multiplier, plasticity_flag, constant, ltp,
                                #  ltd]
                                connection_array = [
                                    connection.get("morphology_id", ""),
                                    connection.get(
                                        "morphology_scalar", [1, 1, 1]
                                    ),
                                    connection.get(
                                        "postSynapticCurrent_multiplier", 1
                                    ),
                                    connection.get("plasticity_flag", False),
                                    connection.get("plasticity_constant", 1),
                                    connection.get("ltp_multiplier", 1),
                                    connection.get("ltd_multiplier", 1),
                                ]
                                connection_arrays.append(connection_array)

                        if connection_arrays:
                            genome_mapping[target_area_id] = connection_arrays

                    #  Update the cortical area's parameters with the new
                    #  mapping
                    # Ensure blueprint section exists in hierarchical genome
                    if "blueprint" not in current_genome:
                        current_genome["blueprint"] = {}

                    # Find or create the area in the hierarchical blueprint
                    if area_id in current_genome["blueprint"]:
                        area_def = current_genome["blueprint"][area_id]
                    else:
                        #  Create area definition from templates for system
                        #  areas (like _power)
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
                            if (
                                area_id
                                in cortical_types["CORE"]["supported_devices"]
                            ):
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
                                    "x": area_template.get(
                                        "coordinate_3d", [0, 0, 0]
                                    )[0],
                                    "y": area_template.get(
                                        "coordinate_3d", [0, 0, 0]
                                    )[1],
                                    "z": area_template.get(
                                        "coordinate_3d", [0, 0, 0]
                                    )[2],
                                },
                                "dimensions": {
                                    "x": area_template.get(
                                        "resolution", [1, 1, 1]
                                    )[0],
                                    "y": area_template.get(
                                        "resolution", [1, 1, 1]
                                    )[1],
                                    "z": area_template.get(
                                        "resolution", [1, 1, 1]
                                    )[2],
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

                    #  CRITICAL FIX: Update mapping in correct hierarchical
                    #  genome location
                    #  ConnectionAnalyzer expects mappings in
                    #  'cortical_mapping_dst', not 'parameters.mapping'
                    area_def["cortical_mapping_dst"] = genome_mapping

                # Update the genome through proper pipeline
                self._current_genome = current_genome

                # CRITICAL FIX: Persist genome changes to StateManager
                # This ensures other parts of the system see the updated genome
                self.logger.info(
                    "🧠 [MAPPING-DEBUG] Saving updated genome to StateManager..."
                )
                self.state_manager.genome = current_genome
                self.logger.info(
                    "🧠 [MAPPING-DEBUG] Genome saved to StateManager successfully"
                )

                # Trigger NeuroEmbryogenesis to update ConnectomeManager
                from feagi.bdu.embryogenesis.neuroembryogenesis import (
                    NeuroEmbryogenesis,
                )

                self.logger.info(
                    "🧠 [MAPPING-DEBUG] Creating NeuroEmbryogenesis instance..."
                )
                
                embryogenesis = NeuroEmbryogenesis(
                    self._connectome_manager, self.state_manager
                )
                self.logger.info(
                    "🧠 [MAPPING-DEBUG] NeuroEmbryogenesis created successfully"
                )

                #  CRITICAL FIX: Load the genome data into the
                #  NeuroEmbryogenesis instance
                #  This ensures the morphology definitions are available for
                #  cortical mapping
                self.logger.info(
                    "🧠 [MAPPING-DEBUG] Loading genome data into NeuroEmbryogenesis..."
                )
                if not embryogenesis._load_genome_data(current_genome):
                    self.logger.error(
                        "🧠 [MAPPING-DEBUG] ERROR: Failed to load genome data into NeuroEmbryogenesis"
                    )
                    if transaction:
                        transaction.rollback()
                    return False
                self.logger.info(
                    "🧠 [MAPPING-DEBUG] Genome data loaded successfully"
                )

                # Apply the cortical mapping update
                self.logger.info(
                    f"🧠 [MAPPING-DEBUG] Calling embryogenesis.update_cortical_mapping with: {mapping}"
                )
                success = embryogenesis.update_cortical_mapping(mapping)
                self.logger.info(
                    f"🧠 [MAPPING-DEBUG] embryogenesis.update_cortical_mapping result: {success}"
                )

                if success and transaction:
                    transaction.commit()
                elif transaction:
                    transaction.rollback()
                    return False

                if success:
                    self.logger.info("Updated cortical mapping")

                    #  ARCHITECTURE COMPLIANCE: Invalidate StateManager cache
                    #  after mapping updates
                    #  This ensures /v1/cortical_mapping/mapping shows fresh
                    #  data immediately
                    self.logger.info(
                        "🧠 [MAPPING-DEBUG] Invalidating cortical areas cache via StateManager..."
                    )
                    try:
                        self.state_manager.invalidate_cortical_areas_cache()
                        self.logger.info(
                            "🧠 [MAPPING-DEBUG] Cache invalidated successfully - fresh data will be served"
                        )

                    except Exception as cache_clear_error:
                        self.logger.warning(
                            f"🧠 [MAPPING-DEBUG] Cache invalidation failed: {cache_clear_error}"
                        )

                return success

            except Exception as e:
                if transaction:
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(f"Error updating cortical mapping: {str(e)}")
            return False

    def update_cortical_mapping_properties(
        self, update_data: Dict[str, Any]
    ) -> bool:
        """Update cortical mapping properties between two specific cortical
        areas.

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

            self.logger.info(
                "🧠 [MAPPING-DEBUG] update_cortical_mapping_properties called"
            )
            self.logger.info(f"🧠 [MAPPING-DEBUG] src_area: {src_area}")
            self.logger.info(f"🧠 [MAPPING-DEBUG] dst_area: {dst_area}")
            self.logger.info(
                f"🧠 [MAPPING-DEBUG] mapping_data: {mapping_data}"
            )

            if not src_area or not dst_area:
                self.logger.error(
                    "🧠 [MAPPING-DEBUG] ERROR: Source and destination cortical areas must be specified"
                )
                return False

            if not self.is_genome_loaded():
                self.logger.error(
                    "🧠 [MAPPING-DEBUG] ERROR: Cannot update cortical mapping properties: No genome loaded"
                )
                return False

            self.logger.info(
                f"🧠 [MAPPING-DEBUG] Updating cortical mapping properties from {src_area} to {dst_area}"
            )

            # Begin genome transaction for atomic modification
            if self.state_manager:
                transaction = self.state_manager.begin_genome_transaction()
                self.logger.info(
                    "🧠 [MAPPING-DEBUG] Started genome transaction"
                )
            else:
                transaction = None
                self.logger.warning(
                    "🧠 [MAPPING-DEBUG] No StateManager - no transaction protection"
                )

            try:
                # Get current genome for modification
                current_genome = self.get_genome()
                if not current_genome:
                    self.logger.error(
                        "🧠 [MAPPING-DEBUG] ERROR: Cannot update cortical mapping properties: Genome data not available"
                    )
                    return False

                self.logger.info(
                    "🧠 [MAPPING-DEBUG] Current genome loaded successfully"
                )

                # CRITICAL FIX: Handle empty mapping_data as deletion request
                if not mapping_data or len(mapping_data) == 0:
                    self.logger.info(
                        f"🧠 [MAPPING-DEBUG] Empty mapping_data detected - treating as deletion request for {src_area} -> {dst_area}"
                    )
                    #  Use the delete_cortical_mapping method for proper
                    #  synapse removal
                    success = self.delete_cortical_mapping(src_area, dst_area)
                    self.logger.info(
                        f"🧠 [MAPPING-DEBUG] Deletion result: {success}"
                    )
                else:
                    #  Convert mapping_data to the expected format for normal
                    #  updates
                    formatted_mapping = {src_area: {dst_area: mapping_data}}
                    self.logger.info(
                        f"🧠 [MAPPING-DEBUG] Formatted mapping: {formatted_mapping}"
                    )

                    # Use the existing update_cortical_mapping method
                    self.logger.info(
                        "🧠 [MAPPING-DEBUG] Calling update_cortical_mapping..."
                    )
                    success = self.update_cortical_mapping(formatted_mapping)
                    self.logger.info(
                        f"🧠 [MAPPING-DEBUG] update_cortical_mapping result: {success}"
                    )

                if success and transaction:
                    self.logger.info(
                        "🧠 [MAPPING-DEBUG] Committing transaction..."
                    )
                    transaction.commit()
                    self.logger.info(
                        "🧠 [MAPPING-DEBUG] Transaction committed successfully"
                    )
                elif transaction:
                    self.logger.error(
                        "🧠 [MAPPING-DEBUG] Rolling back transaction due to failure"
                    )
                    transaction.rollback()
                    return False

                if success:
                    self.logger.info(
                        "🧠 [MAPPING-DEBUG] SUCCESS: Mapping properties updated successfully"
                    )
                else:
                    self.logger.error(
                        "🧠 [MAPPING-DEBUG] FAILURE: Mapping properties update failed"
                    )

                return success

            except Exception as e:
                self.logger.error(
                    f"🧠 [MAPPING-DEBUG] EXCEPTION in inner try block: {e}"
                )
                self.logger.exception(
                    "🧠 [MAPPING-DEBUG] Exception traceback:"
                )
                if transaction:
                    self.logger.error(
                        "🧠 [MAPPING-DEBUG] Rolling back transaction due to exception"
                    )
                    transaction.rollback()
                raise e

        except Exception as e:
            self.logger.error(
                f"🧠 [MAPPING-DEBUG] EXCEPTION in outer try block: {e}"
            )
            self.logger.exception(
                "🧠 [MAPPING-DEBUG] Outer exception traceback:"
            )
            return False

    def delete_cortical_mapping(
        self, src_cortical_area: str, dst_cortical_area: str
    ) -> bool:
        """Delete cortical mapping and all associated synapses between two
        cortical areas.

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
                self.logger.error(
                    "Cannot delete cortical mapping: No genome loaded"
                )
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

                #  Step 1: SAFETY CHECK - Ensure we're only removing MAPPING,
                #  not the cortical area itself
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

                #  SAFETY: We're only modifying mapping properties, NOT
                #  deleting the cortical area
                src_area_def = blueprint[src_cortical_area]

                # Remove from cortical_mapping_dst if it exists
                mapping_dst = src_area_def.get("cortical_mapping_dst", {})
                if dst_cortical_area in mapping_dst:
                    del mapping_dst[dst_cortical_area]
                    self.logger.info(
                        f"Removed {dst_cortical_area} from {src_cortical_area} mapping destinations"
                    )

                # Remove from parameters.mapping if it exists
                parameters = src_area_def.get("parameters", {})
                if "mapping" in parameters:
                    mapping_params = parameters["mapping"]
                    if (
                        isinstance(mapping_params, dict)
                        and dst_cortical_area in mapping_params
                    ):
                        del mapping_params[dst_cortical_area]
                        self.logger.info(
                            f"Removed {dst_cortical_area} from {src_cortical_area} mapping parameters"
                        )

                # Step 2: Update genome in state manager
                self._current_genome = current_genome
                if self.state_manager:
                    self.state_manager.genome = current_genome

                # Step 3: Remove existing synapses between cortical areas
                #  Use direct synapse deletion approach (following
                #  delete_cortical_connection pattern)
                connectome_manager = self._connectome_manager
                if connectome_manager:
                    try:
                        # Get all neurons in source and target areas
                        source_neurons = (
                            connectome_manager.get_neurons_by_cortical_area(
                                src_cortical_area
                            )
                        )
                        target_neurons = (
                            connectome_manager.get_neurons_by_cortical_area(
                                dst_cortical_area
                            )
                        )

                        deleted_count = 0
                        self.logger.info(
                            f"Deleting synapses from {len(source_neurons)} source neurons to {len(target_neurons)} target neurons"
                        )

                        # Delete all synapses between source and target areas
                        for source_id in source_neurons:
                            for target_id in target_neurons:
                                if connectome_manager.has_synapse(
                                    source_id, target_id
                                ):
                                    success_remove = (
                                        connectome_manager.remove_synapse(
                                            source_id, target_id
                                        )
                                    )
                                    if success_remove:
                                        deleted_count += 1

                        self.logger.info(
                            f"Successfully deleted {deleted_count} synapses between {src_cortical_area} and {dst_cortical_area}"
                        )
                        success = True

                    except Exception as e:
                        self.logger.error(
                            f"Failed to delete synapses between areas: {e}"
                        )
                        success = False
                        if transaction:
                            transaction.rollback()
                        return False
                else:
                    self.logger.error(
                        "ConnectomeManager not available for synapse deletion"
                    )
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
    #  These methods handle genome loading and management through proper data
    #  flow:
    #  API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis
    #  → ConnectomeManager

    def reset_genome(self) -> bool:
        """Reset genome through proper genome modification pipeline.

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
                # Create an empty genome structure
                empty_genome = {
                    "blueprint": {},
                    "neuron_morphologies": {},
                    "cortical_mappings": {},
                    "version": "2.1",
                }

                # Clear the current genome and update state manager
                self._current_genome = empty_genome
                if self.state_manager:
                    self.state_manager.genome = empty_genome
                    
                    # Comprehensive state clearing for genome reset
                    self._clear_state_for_genome_loading()

                # Trigger NeuroEmbryogenesis to reset the connectome
                from feagi.bdu.embryogenesis.neuroembryogenesis import (
                    NeuroEmbryogenesis,
                )

                embryogenesis = NeuroEmbryogenesis(
                    self._connectome_manager, self.state_manager
                )

                # Apply the genome reset by developing brain from empty genome
                #  ARCHITECTURE: Pass hierarchical genome directly (single
                #  source of truth)
                success = embryogenesis.develop_brain_from_genome_data(
                    empty_genome
                )

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

    def amalgamate_genome(
        self, amalgamation_data: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Perform genome amalgamation through proper genome modification
        pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures genome amalgamation
        goes through the proper data flow to maintain genome consistency.

        Args:
            amalgamation_data: Dictionary containing amalgamation parameters

        Returns:
            Dict containing amalgamation result information
        """
        try:
            # Check if there's already a pending amalgamation
            if self.state_manager and hasattr(self.state_manager, 'pending_amalgamation'):
                pending = getattr(self.state_manager, 'pending_amalgamation', {})
                if pending:
                    # Check timeout (500 seconds as in legacy)
                    import time
                    amalgamation_timeout = 500
                    elapsed_time = time.time() - pending.get("initiation_time", 0)
                    if elapsed_time <= amalgamation_timeout:
                        raise ValueError("An existing amalgamation attempt is pending")
                    else:
                        # Clear expired amalgamation
                        self.state_manager.pending_amalgamation = {}

            # Generate amalgamation ID in legacy format (timestamp + '_A')
            from datetime import datetime
            import time
            now = datetime.now()
            amalgamation_id = str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_A'

            # Extract genome payload from amalgamation data
            genome_payload = amalgamation_data.get("genome_payload")
            if not genome_payload:
                raise ValueError("No genome payload provided for amalgamation")

            # Convert flat genome to hierarchical format for circuit size calculation
            from feagi.evo.genome_processor import genome_2_1_convertor
            
            # Handle both flat and hierarchical genome formats
            if "blueprint" in genome_payload:
                blueprint = genome_payload["blueprint"]
                self.logger.info(f"Processing genome blueprint with {len(blueprint)} entries")
                
                # Check if it's flat format (has flattened keys)
                if blueprint and isinstance(blueprint, dict):
                    blueprint_keys = list(blueprint.keys())
                    sample_keys = blueprint_keys[:5]
                    self.logger.info(f"Sample blueprint keys: {sample_keys}")
                    
                    if blueprint_keys and any("10c-" in key and "-cx-" in key for key in blueprint_keys[:5]):
                        # It's flat format, convert to hierarchical for size calculation
                        self.logger.info("Detected flat genome format, converting to hierarchical")
                        converted_genome = genome_2_1_convertor(flat_genome=blueprint)
                        blueprint_for_size = converted_genome["blueprint"]
                        self.logger.info(f"Converted to hierarchical format with {len(blueprint_for_size)} cortical areas")
                    else:
                        # Already hierarchical
                        self.logger.info("Detected hierarchical genome format")
                        blueprint_for_size = blueprint
                else:
                    blueprint_for_size = {}
            else:
                raise ValueError("Invalid genome payload: missing blueprint")

            # Calculate circuit size using the core API service method
            if hasattr(self, 'core_api_service') and self.core_api_service:
                circuit_size = self.core_api_service.calculate_circuit_size(blueprint_for_size)
            else:
                # Fallback calculation
                circuit_size = [1, 1, 1]

            # Store pending amalgamation data in state manager
            if self.state_manager:
                pending_amalgamation = {
                    "genome_id": amalgamation_data.get("genome_id", "unknown"),
                    "genome_title": amalgamation_data.get("genome_title", "Unknown Genome"),
                    "genome_payload": genome_payload,
                    "initiation_time": time.time(),
                    "amalgamation_id": amalgamation_id,
                    "circuit_size": circuit_size
                }
                
                self.state_manager.pending_amalgamation = pending_amalgamation
                
                # Update amalgamation history
                if not hasattr(self.state_manager, 'amalgamation_history'):
                    self.state_manager.amalgamation_history = {}
                self.state_manager.amalgamation_history[amalgamation_id] = "pending"

            return {
                "success": True,
                "amalgamation_id": amalgamation_id,
                "circuit_size": circuit_size
            }

        except Exception as e:
            self.logger.error(f"Error during genome amalgamation: {str(e)}")
            return {"success": False, "error": str(e)}

    def cancel_amalgamation(self, amalgamation_id: str) -> bool:
        """Cancel genome amalgamation through proper genome modification
        pipeline.

        ARCHITECTURE COMPLIANCE: This method ensures amalgamation cancellation
        goes through the proper data flow to maintain genome consistency.

        Args:
            amalgamation_id: ID of amalgamation to cancel

        Returns:
            bool: True if amalgamation was cancelled successfully
        """
        try:
            # Clear pending amalgamation (legacy behavior)
            if self.state_manager:
                self.state_manager.pending_amalgamation = {}
                
                # Update amalgamation history
                if hasattr(self.state_manager, 'amalgamation_history') and amalgamation_id in self.state_manager.amalgamation_history:
                    self.state_manager.amalgamation_history[amalgamation_id] = "cancelled"
                
                self.logger.info(f"Amalgamation {amalgamation_id} cancelled successfully")
                return True
            
            return False

        except Exception as e:
            self.logger.error(f"Error cancelling amalgamation: {str(e)}")
            return False

    def append_file_to_genome(self, file_data: Dict[str, Any]) -> bool:
        """Append file content to genome through proper genome modification
        pipeline.

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
                #  TODO: Implement file appending to genome - merge file_data
                #  into current_genome
                #  then call
                #  embryogenesis.develop_brain_from_genome_data(updated_genome)
                self.logger.error(
                    "File appending to genome not yet implemented"
                )
                success = False

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
    #  API → Service → GenomeService → StateManager.genome → NeuroEmbryogenesis
    #  → ConnectomeManager

    def create_brain_region(
        self,
        region_id: str,
        region_name: str,
        parent_region_id: str = "root",
        coordinates: Dict[str, int] = None,
        dimensions: Dict[str, int] = None,
        parameters: Dict[str, Any] = None,
    ) -> bool:
        """Create a brain region through proper genome modification pipeline.

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
                self.logger.error(
                    "Cannot create brain region: No genome loaded"
                )
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
                    raise ValueError(
                        f"Brain region '{region_id}' already exists"
                    )

                # Extract areas and regions from parameters for proper storage
                areas = []
                regions = []
                clean_parameters = {}
                
                if parameters:
                    areas = parameters.get("areas", [])
                    regions = parameters.get("regions", [])
                    # Store other parameters excluding areas/regions (to avoid duplication)
                    clean_parameters = {k: v for k, v in parameters.items() 
                                      if k not in ["areas", "regions"]}

                # Enforce one-region-per-area: remove any of these areas from existing regions
                if areas:
                    areas_set = set(areas)
                    for existing_region_id, existing_region in current_genome["brain_regions"].items():
                        # Skip the new region (not added yet) by design; iterate current existing only
                        # Remove from modern key
                        if "cortical_areas" in existing_region and existing_region["cortical_areas"]:
                            existing_region["cortical_areas"] = [
                                a for a in existing_region["cortical_areas"] if a not in areas_set
                            ]
                        # Remove from legacy key
                        if "areas" in existing_region and existing_region["areas"]:
                            existing_region["areas"] = [
                                a for a in existing_region["areas"] if a not in areas_set
                            ]
                        # Also clean stale I/O listings in the old region
                        if "inputs" in existing_region and existing_region["inputs"]:
                            existing_region["inputs"] = [
                                a for a in existing_region["inputs"] if a not in areas_set
                            ]
                        if "outputs" in existing_region and existing_region["outputs"]:
                            existing_region["outputs"] = [
                                a for a in existing_region["outputs"] if a not in areas_set
                            ]
                
                # Strict coordinates required for region creation
                if coordinates is None:
                    raise ValueError("coordinates are required for brain region creation")
                for key in ("x", "y", "z"):
                    if key not in coordinates:
                        raise ValueError("coordinates must include x, y, z")

                # Create new brain region definition
                new_region = {
                    "region_id": region_id,
                    "region_name": region_name,
                    "parent_region_id": parent_region_id,
                    "coordinates": {"x": int(coordinates["x"]), "y": int(coordinates["y"]), "z": int(coordinates["z"])},
                    # Normalized coordinates for readers expecting list format
                    "coordinate_3d": [
                        int(coordinates["x"]),
                        int(coordinates["y"]),
                        int(coordinates["z"]),
                    ],
                    "dimensions": dimensions
                    or {"width": 1, "height": 1, "depth": 1},
                    "parameters": clean_parameters,
                    "child_regions": regions,
                    "cortical_areas": areas,
                }

                # If 2D coordinates provided via parameters, mirror on top-level for readers
                if parameters and isinstance(parameters.get("coordinates_2d"), list):
                    new_region["coordinate_2d"] = parameters["coordinates_2d"]

                # Add to genome structure
                current_genome["brain_regions"][region_id] = new_region

                # Update parent region's children list (use consistent 'regions' field)
                if parent_region_id in current_genome["brain_regions"]:
                    parent_region = current_genome["brain_regions"][parent_region_id]
                    
                    # Use consistent 'regions' field name (not 'child_regions')
                    if "regions" not in parent_region:
                        parent_region["regions"] = []
                    if region_id not in parent_region["regions"]:
                        parent_region["regions"].append(region_id)
                    
                    # Also update legacy 'child_regions' for backward compatibility
                    if "child_regions" not in parent_region:
                        parent_region["child_regions"] = []
                    if region_id not in parent_region["child_regions"]:
                        parent_region["child_regions"].append(region_id)

                # Update cortical areas' own properties to reflect new region association
                if areas:
                    blueprint = current_genome.get("blueprint", {})
                    for area_id in areas:
                        if area_id in blueprint:
                            area_def = blueprint[area_id]
                            if "parameters" not in area_def or not isinstance(area_def["parameters"], dict):
                                area_def["parameters"] = {}
                            area_def["parameters"]["parent_region_id"] = region_id
                            area_def["parameters"]["parent_region_title"] = region_name

                # Update the genome through proper pipeline
                self._current_genome = current_genome

                # CRITICAL FIX: Update genome in state manager (single source of truth)
                # This ensures brain regions follow the same architectural pattern as cortical areas
                if self.state_manager:
                    self.state_manager.genome = current_genome

                # Trigger NeuroEmbryogenesis to update ConnectomeManager
                from feagi.bdu.embryogenesis.neuroembryogenesis import (
                    NeuroEmbryogenesis,
                )

                embryogenesis = NeuroEmbryogenesis(
                    self._connectome_manager, self.state_manager
                )

                #  Apply the brain region creation by triggering brain
                #  development
                #  ARCHITECTURE: Pass hierarchical genome directly (single
                #  source of truth)
                success = embryogenesis.develop_brain_from_genome_data(
                    current_genome
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
        """Update a brain region through proper genome modification pipeline.

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
                self.logger.error(
                    "Cannot update brain region: No genome loaded"
                )
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
                    self.logger.warning(
                        f"Brain region {region_id} not found in genome"
                    )
                    return False

                # Get existing region definition
                region_def = current_genome["brain_regions"][region_id]

                # Update fields if provided
                if region_name is not None:
                    region_def["region_name"] = region_name
                if coordinates is not None:
                    # Strict update; validate keys
                    for key in ("x", "y", "z"):
                        if key not in coordinates:
                            raise ValueError("coordinates must include x, y, z")
                    region_def["coordinates"] = {
                        "x": int(coordinates["x"]),
                        "y": int(coordinates["y"]),
                        "z": int(coordinates["z"]),
                    }
                    region_def["coordinate_3d"] = [
                        int(coordinates["x"]),
                        int(coordinates["y"]),
                        int(coordinates["z"]),
                    ]
                if dimensions is not None:
                    region_def["dimensions"] = dimensions
                if parameters is not None:
                    region_def["parameters"].update(parameters)
                    # Mirror normalized 2D coordinates to top-level for readers
                    if isinstance(parameters.get("coordinates_2d"), list):
                        region_def["coordinate_2d"] = parameters["coordinates_2d"]

                # Handle parent region change
                if (
                    parent_region_id is not None
                    and parent_region_id != region_def.get("parent_region_id")
                ):
                    old_parent_id = region_def.get("parent_region_id")

                    # Remove from old parent's children list
                    if (
                        old_parent_id
                        and old_parent_id in current_genome["brain_regions"]
                    ):
                        old_parent = current_genome["brain_regions"][
                            old_parent_id
                        ]
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
                        new_parent = current_genome["brain_regions"][
                            parent_region_id
                        ]
                        if "child_regions" not in new_parent:
                            new_parent["child_regions"] = []
                        if region_id not in new_parent["child_regions"]:
                            new_parent["child_regions"].append(region_id)

                    region_def["parent_region_id"] = parent_region_id

                # Update the genome through proper pipeline
                self._current_genome = current_genome

                # Ensure StateManager sees the updated genome (single source of truth)
                if self.state_manager:
                    self.state_manager.genome = current_genome

                # Lightweight refresh: DO NOT rebuild neurons when updating region metadata
                success = True
                try:
                    # Reload hierarchy to reflect updated region metadata
                    if hasattr(self._connectome_manager, "brain_region_hierarchy"):
                        self._connectome_manager.brain_region_hierarchy.load_from_genome(current_genome)

                    # Sync cached brain_regions structure
                    if hasattr(self._connectome_manager, "brain_regions") and "brain_regions" in current_genome:
                        self._connectome_manager.brain_regions.update(current_genome["brain_regions"])

                    # Run mapping validation only (no neurogenesis)
                    if hasattr(self._connectome_manager, "_trigger_brain_region_validation"):
                        self._connectome_manager._trigger_brain_region_validation()
                except Exception as _e:
                    self.logger.warning(f"Region update sync encountered a non-fatal error: {_e}")

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

    def delete_brain_region(
        self, region_id: str, delete_members: bool = False
    ) -> bool:
        """Delete a brain region through proper genome modification pipeline.

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
                self.logger.error(
                    "Cannot delete brain region: No genome loaded"
                )
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
                    self.logger.warning(
                        f"Brain region {region_id} not found in genome"
                    )
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
                        self.delete_brain_region(
                            child_region_id, delete_members=True
                        )

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
                            if (
                                child_region_id
                                not in parent_region["child_regions"]
                            ):
                                parent_region["child_regions"].append(
                                    child_region_id
                                )
                            # Update child's parent reference
                            if (
                                child_region_id
                                in current_genome["brain_regions"]
                            ):
                                current_genome["brain_regions"][
                                    child_region_id
                                ]["parent_region_id"] = parent_region_id

                    #  Move cortical areas to parent region (using hierarchical
                    #  blueprint)
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
                    parent_region = current_genome["brain_regions"][
                        parent_region_id
                    ]
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
                success = embryogenesis.develop_brain_from_genome_data(
                    current_genome
                )

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

    def _analyze_genome_requirements(
        self, genome_data: Dict[str, Any]
    ) -> Dict[str, int]:
        """Analyze genome to calculate required neuron and synapse capacity.

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
                        #  CRITICAL: Account for neuron density (neurons per
                        #  voxel)
                        neurons_per_voxel = area_props.get(
                            "neurons_per_voxel", 1
                        )
                        area_neurons = (
                            dims[0] * dims[1] * dims[2] * neurons_per_voxel
                        )
                        total_neurons += area_neurons

                        self.logger.debug(
                            f"[GENOME ANALYSIS] {cortical_id}: {area_neurons:,} neurons ({dims[0]}x{dims[1]}x{dims[2]} * {neurons_per_voxel} neurons/voxel)"
                        )

            # Estimate synapses from cortical mappings
            for src_id, dst_mappings in cortical_mappings.items():
                for dst_id, connections in dst_mappings.items():
                    #  Rough estimate: each connection creates synapses
                    #  proportional to area sizes
                    src_area = cortical_areas.get(src_id)
                    dst_area = cortical_areas.get(dst_id)

                    if (
                        src_area
                        and dst_area
                        and "dimensions" in src_area
                        and "dimensions" in dst_area
                    ):
                        src_neurons = (
                            src_area["dimensions"][0]
                            * src_area["dimensions"][1]
                            * src_area["dimensions"][2]
                        )
                        dst_neurons = (
                            dst_area["dimensions"][0]
                            * dst_area["dimensions"][1]
                            * dst_area["dimensions"][2]
                        )

                        #  Estimate synapses per connection (conservative
                        #  estimate)
                        synapses_per_connection = (
                            min(src_neurons, dst_neurons) // 10
                        )
                        total_synapses += synapses_per_connection * len(
                            connections
                        )

            # Apply buffer multiplier for safety (50% extra capacity)
            buffer_multiplier = 1.5
            min_neuron_capacity = 100_000
            min_synapse_capacity = 1_000_000

            recommended_neurons = max(
                int(total_neurons * buffer_multiplier), min_neuron_capacity
            )
            recommended_synapses = max(
                int(total_synapses * buffer_multiplier), min_synapse_capacity
            )

            self.logger.info(
                f"[GENOME ANALYSIS] Required neurons: {total_neurons:,}"
            )
            self.logger.info(
                f"[GENOME ANALYSIS] Required synapses: {total_synapses:,}"
            )
            self.logger.info(
                f"[GENOME ANALYSIS] Recommended capacity: {recommended_neurons:,} neurons, {recommended_synapses:,} synapses"
            )

            return {
                "neurons_required": total_neurons,
                "synapses_required": total_synapses,
                "recommended_neuron_capacity": recommended_neurons,
                "recommended_synapse_capacity": recommended_synapses,
            }

        except Exception as e:
            self.logger.error(f"Error analyzing genome requirements: {e}")
            # Return safe defaults if analysis fails
            return {
                "neurons_required": 0,
                "synapses_required": 0,
                "recommended_neuron_capacity": 100_000,
                "recommended_synapse_capacity": 1_000_000,
            }

    def _update_parameters_only(
        self, cortical_id: str, changes: Dict[str, Any], transaction
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
            from feagi.api.core.services.genome.change_classifier import (
                CorticalChangeClassifier,
            )

            parameter_changes = {
                k: v
                for k, v in changes.items()
                if k in CorticalChangeClassifier.PARAMETER_TO_NEURON_PROPERTY
            }

            if not parameter_changes:
                self.logger.warning(
                    f"No parameter changes found in {list(changes.keys())}"
                )
                return area_def

            # Update genome parameters section
            if "parameters" not in area_def:
                area_def["parameters"] = {}
            area_def["parameters"].update(parameter_changes)

            # Commit genome changes to both internal cache and state manager
            self._current_genome = current_genome

            #  CRITICAL: Synchronize StateManager's genome to maintain
            #  consistency
            if self.state_manager:
                self.state_manager.genome = current_genome
                self.logger.debug(
                    "[PARAMETER-UPDATE] Synchronized genome with StateManager"
                )

            # Direct neuron updates (NO REBUILD!)
            from feagi.api.core.services.genome.parameter_updater import (
                CorticalParameterUpdater,
            )

            updater = CorticalParameterUpdater(self._connectome_manager)

            # Update neurons in arrays
            neuron_update_success = updater.update_neuron_parameters(
                cortical_id, parameter_changes
            )

            # Update ConnectomeManager cortical area properties for consistency
            connectome_update_success = (
                self._connectome_manager.update_cortical_area_properties(
                    cortical_id, parameter_changes
                )
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
                    f"in {duration * 1000:.1f}ms"
                )
                return current_genome["blueprint"][cortical_id]
            else:
                self.logger.error(f"Parameter update failed for {cortical_id}")
                return None

        except Exception as e:
            duration = time.time() - start_time
            self.logger.error(
                f"Fast parameter update failed after {duration * 1000:.1f}ms: {e}"
            )
            if transaction:
                transaction.rollback()
            return None

    def _update_metadata_only(
        self, cortical_id: str, changes: Dict[str, Any], transaction
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
            from feagi.api.core.services.genome.change_classifier import (
                CorticalChangeClassifier,
            )

            metadata_changes = {
                k: v
                for k, v in changes.items()
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
                    f"in {duration * 1000:.1f}ms"
                )
                return current_genome["blueprint"][cortical_id]
            else:
                self.logger.error(f"Metadata update failed for {cortical_id}")
                return None

        except Exception as e:
            duration = time.time() - start_time
            self.logger.error(
                f"Metadata update failed after {duration * 1000:.1f}ms: {e}"
            )
            if transaction:
                transaction.rollback()
            return None

    def _update_with_full_rebuild(
        self, cortical_id: str, changes: Dict[str, Any], transaction
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

            # Preserve existing coordinates if not explicitly changed
            preserved_coords_3d = None
            preserved_coords_2d = None
            # Try multiple known fields to preserve existing coordinates
            try:
                preserved_coords_3d = area_def.get("coordinates_3d")
                if preserved_coords_3d is None:
                    preserved_coords_3d = area_def.get("coordinates")
                if preserved_coords_3d is None:
                    preserved_coords_3d = area_def.get("relative_coordinate")
            except Exception:
                preserved_coords_3d = None
            try:
                # Stored as top-level '2d_coordinate' or under parameters
                preserved_coords_2d = area_def.get("2d_coordinate")
                if preserved_coords_2d is None:
                    preserved_coords_2d = (
                        area_def.get("parameters", {}).get("coordinates_2d")
                    )
            except Exception:
                preserved_coords_2d = None

            # Apply all changes to area definition
            for key, value in changes.items():
                if key in [
                    "cortical_name",
                    "coordinates_3d",
                    "cortical_dimensions",
                    "cortical_type",
                ]:
                    area_def[key] = value
                else:
                    # Parameter changes
                    if "parameters" not in area_def:
                        area_def["parameters"] = {}
                    area_def["parameters"][key] = value

            # Restore coordinates if not provided in this update
            if "coordinates_3d" not in changes and preserved_coords_3d is not None:
                area_def["coordinates_3d"] = preserved_coords_3d
            if (
                preserved_coords_2d is not None
                and "2d_coordinate" not in area_def
            ):
                # Keep existing 2D coordinate when present
                area_def["2d_coordinate"] = preserved_coords_2d

            # Commit genome changes to both internal cache and state manager
            self._current_genome = current_genome

            #  CRITICAL: Synchronize StateManager's genome to maintain
            #  consistency
            if self.state_manager:
                self.state_manager.genome = current_genome
                self.logger.debug(
                    "[FULL-REBUILD] Synchronized genome with StateManager"
                )

            # Full brain rebuild (existing logic)
            from feagi.bdu.embryogenesis.neuroembryogenesis import (
                NeuroEmbryogenesis,
            )

            embryogenesis = NeuroEmbryogenesis(
                self._connectome_manager, self.state_manager
            )

            #  ARCHITECTURE: Pass hierarchical genome directly (single source
            #  of truth)
            # NeuroEmbryogenesis now supports hierarchical format natively
            success = embryogenesis.develop_brain_from_genome_data(
                current_genome
            )

            if success and transaction:
                transaction.commit()
            elif transaction:
                transaction.rollback()
                return None

            duration = time.time() - start_time
            if success:
                self.logger.info(
                    f"[FULL-REBUILD] Structural update completed for {cortical_id} "
                    f"in {duration * 1000:.1f}ms"
                )
                return current_genome["blueprint"][cortical_id]
            else:
                self.logger.error(f"Full rebuild failed for {cortical_id}")
                return None

        except Exception as e:
            duration = time.time() - start_time
            self.logger.error(
                f"Full rebuild failed after {duration * 1000:.1f}ms: {e}"
            )
            if transaction:
                transaction.rollback()
            return None

    def _update_hybrid(
        self, cortical_id: str, changes: Dict[str, Any], transaction
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
                ChangeType,
                CorticalChangeClassifier,
            )

            separated = CorticalChangeClassifier.separate_changes_by_type(
                changes
            )

            self.logger.info(
                f"[HYBRID-UPDATE] Processing {len(separated[ChangeType.STRUCTURAL])} structural, "
                f"{len(separated[ChangeType.PARAMETER])} parameter, "
                f"{len(separated[ChangeType.METADATA])} metadata changes"
            )

            # Apply metadata changes first (fastest)
            if separated[ChangeType.METADATA]:
                result = self._update_metadata_only(
                    cortical_id, separated[ChangeType.METADATA], None
                )
                if not result:
                    return None

            #  If we have structural changes, do localized rebuild (includes
            #  parameters)
            if separated[ChangeType.STRUCTURAL]:
                #  Combine structural + parameter changes for single localized
                #  rebuild
                combined_changes = {
                    **separated[ChangeType.STRUCTURAL],
                    **separated[ChangeType.PARAMETER],
                }
                result = self._update_with_localized_rebuild(
                    cortical_id, combined_changes, transaction
                )
            else:
                # Only parameter changes remain - use fast path
                if separated[ChangeType.PARAMETER]:
                    result = self._update_parameters_only(
                        cortical_id,
                        separated[ChangeType.PARAMETER],
                        transaction,
                    )
                else:
                    # Only metadata was updated
                    if transaction:
                        transaction.commit()
                    result = self._current_genome["blueprint"][cortical_id]

            duration = time.time() - start_time
            self.logger.info(
                f"[HYBRID-UPDATE] Completed hybrid update for {cortical_id} "
                f"in {duration * 1000:.1f}ms"
            )

            return result

        except Exception as e:
            duration = time.time() - start_time
            self.logger.error(
                f"Hybrid update failed after {duration * 1000:.1f}ms: {e}"
            )
            if transaction:
                transaction.rollback()
            return None

    def _update_with_localized_rebuild(
        self, cortical_id: str, changes: Dict[str, Any], transaction
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

        # 🔍 CHECKPOINT 0: Initial state before expansion
        initial_synapse_count = self._connectome_manager.get_synapse_count()
        initial_neuron_count = self._connectome_manager.get_neuron_count()

        self.logger.info("🔍 [EXPANSION-DEBUG] CHECKPOINT 0 - Initial State:")
        self.logger.info(
            f"🔍 [EXPANSION-DEBUG]   - Cortical area: {cortical_id}"
        )
        self.logger.info(
            f"🔍 [EXPANSION-DEBUG]   - Initial synapses: {initial_synapse_count}"
        )
        self.logger.info(
            f"🔍 [EXPANSION-DEBUG]   - Initial neurons: {initial_neuron_count}"
        )
        self.logger.info(
            f"🔍 [EXPANSION-DEBUG]   - Changes requested: {changes}"
        )

        try:
            # 1. Update genome (hierarchical format) first
            current_genome = self._current_genome.copy()
            area_def = current_genome["blueprint"][cortical_id]

            # Apply all changes to area definition
            for key, value in changes.items():
                if key in [
                    "cortical_name",
                    "coordinates_3d",
                    "cortical_dimensions",
                    "cortical_type",
                ]:
                    area_def[key] = value
                else:
                    # Parameter changes
                    if "parameters" not in area_def:
                        area_def["parameters"] = {}
                    area_def["parameters"][key] = value

            # Commit genome changes to both internal cache and state manager
            self._current_genome = current_genome

            #  CRITICAL: Synchronize StateManager's genome to maintain
            #  consistency
            if self.state_manager:
                self.state_manager.genome = current_genome
                self.logger.info(
                    "[LOCALIZED-REBUILD] Synchronized genome with StateManager"
                )

            # 2. Extract properties for area recreation
            properties = self._extract_area_properties_from_genome(
                cortical_id, current_genome
            )

            #  3. PROPER REAL-TIME APPROACH: Resize cortical area without
            #  deleting neurons
            #  This preserves memory locations for GPU compatibility and avoids
            #  lag
            self.logger.info(
                f"[LOCALIZED-REBUILD] Resizing cortical area {cortical_id} dimensions: {changes.get('cortical_dimensions')}"
            )
            old_area_existed = (
                cortical_id in self._connectome_manager.cortical_areas
            )
            new_dimensions = tuple(
                changes.get("cortical_dimensions", [1, 1, 1])
            )

            if old_area_existed:
                area = self._connectome_manager.cortical_areas[cortical_id]
                old_dimensions = area.dimensions
                new_dimensions = tuple(
                    changes.get("cortical_dimensions", old_dimensions)
                )

                if new_dimensions != old_dimensions:
                    self.logger.info(
                        f"[LOCALIZED-REBUILD] Changing dimensions from {old_dimensions} to {new_dimensions}"
                    )

                    #  Use the proper cortical area resize method - NO NEURON
                    #  DELETION
                    removed_neuron_indices = area.resize(new_dimensions)

                    if removed_neuron_indices:
                        # Map indices to neuron IDs and perform batch logical removal (no compaction)
                        neuron_ids_to_remove = []
                        for neuron_idx in removed_neuron_indices:
                            nid = self._connectome_manager.index_to_neuron_id.get(neuron_idx)
                            if nid is not None:
                                neuron_ids_to_remove.append(nid)
                        if neuron_ids_to_remove:
                            try:
                                removed = self._connectome_manager.neuron_array.remove_neurons_batch(neuron_ids_to_remove)
                                self.logger.info(
                                    f"[LOCALIZED-REBUILD] Marked {removed} neurons as deleted (logical removal)"
                                )
                            except Exception as rm_err:
                                self.logger.warning(
                                    f"[LOCALIZED-REBUILD] Failed logical removal for {len(neuron_ids_to_remove)} neurons: {rm_err}"
                                )

                    # Calculate if we need more neurons
                    old_volume = (
                        old_dimensions[0]
                        * old_dimensions[1]
                        * old_dimensions[2]
                    )
                    new_volume = (
                        new_dimensions[0]
                        * new_dimensions[1]
                        * new_dimensions[2]
                    )
                    neurons_per_voxel = properties.get("neurons_per_voxel", 1)

                    if new_volume > old_volume:
                        additional_neurons_needed = (
                            new_volume - old_volume
                        ) * neurons_per_voxel
                        self.logger.info(
                            f"[LOCALIZED-REBUILD] Need {additional_neurons_needed} additional neurons - reusing from free pool"
                        )

                        # Reuse neurons from free pool - NO MEMORY ALLOCATION
                        new_neurons = self._reuse_neurons_for_area_expansion(
                            cortical_id, additional_neurons_needed, properties
                        )

                        #  🔍 CHECKPOINT 3: INTELLIGENT PATTERN EXTENSION -
                        #  Extend existing synaptic patterns to new neurons
                        self.logger.info(
                            f"🔍 [EXPANSION-DEBUG] CHECKPOINT 3 - Starting pattern extension for {cortical_id}"
                        )
                        try:
                            from feagi.api.core.services.expansion import (
                                ConnectionAnalyzer,
                                PatternExtender,
                            )

                            #  🔍 CHECKPOINT 4: Analyze existing connectivity
                            #  patterns
                            self.logger.info(
                                f"🔍 [EXPANSION-DEBUG] CHECKPOINT 4 - Analyzing connectivity for {cortical_id}"
                            )
                            analyzer = ConnectionAnalyzer(
                                self._connectome_manager, self.state_manager
                            )
                            analysis = analyzer.analyze_area_connectivity(
                                cortical_id
                            )

                            self.logger.info(
                                "🔍 [EXPANSION-DEBUG] Connectivity analysis results:"
                            )
                            self.logger.info(
                                f"🔍 [EXPANSION-DEBUG]   - Internal mappings: {analysis.get('internal_count', 0)}"
                            )
                            self.logger.info(
                                f"🔍 [EXPANSION-DEBUG]   - Incoming mappings: {analysis.get('incoming_count', 0)}"
                            )
                            self.logger.info(
                                f"🔍 [EXPANSION-DEBUG]   - Outgoing mappings: {analysis.get('outgoing_count', 0)}"
                            )
                            self.logger.info(
                                f"🔍 [EXPANSION-DEBUG]   - Total mappings: {analysis.get('total_mappings', 0)}"
                            )

                            if analysis.get("total_mappings", 0) > 0:
                                #  🔍 CHECKPOINT 5: Use the exact newly created
                                #  neurons from expansion
                                self.logger.info(
                                    "🔍 [EXPANSION-DEBUG] CHECKPOINT 5 - Using newly created neurons"
                                )

                                self.logger.info(
                                    f"🔍 [EXPANSION-DEBUG]   - New neurons created: {len(new_neurons)}"
                                )
                                self.logger.info(
                                    f"🔍 [EXPANSION-DEBUG]   - New neuron IDs: {sorted(list(new_neurons))[:10]}{'...' if len(new_neurons) > 10 else ''}"
                                )

                                if new_neurons:
                                    #  🔍 CHECKPOINT 6: Extend existing patterns
                                    #  to new neurons
                                    self.logger.info(
                                        "🔍 [EXPANSION-DEBUG] CHECKPOINT 6 - Starting pattern extension"
                                    )
                                    extender = PatternExtender(
                                        self._connectome_manager,
                                        self.state_manager,
                                    )
                                    synapses_created = (
                                        extender.extend_patterns_for_expansion(
                                            cortical_id=cortical_id,
                                            old_dimensions=old_dimensions,
                                            new_dimensions=new_dimensions,
                                            new_neurons=set(new_neurons),
                                        )
                                    )

                                    self.logger.info(
                                        "🔍 [EXPANSION-DEBUG] CHECKPOINT 7 - Pattern extension completed"
                                    )
                                    self.logger.info(
                                        f"🔍 [EXPANSION-DEBUG]   - New synapses created: {synapses_created}"
                                    )
                                    self.logger.info(
                                        f"[LOCALIZED-REBUILD] Extended connectivity patterns: {synapses_created} new synapses created for {cortical_id}"
                                    )
                                else:
                                    self.logger.warning(
                                        "🔍 [EXPANSION-DEBUG] CHECKPOINT 6 - No new neurons identified for pattern extension"
                                    )
                                    self.logger.warning(
                                        "[LOCALIZED-REBUILD] Could not identify new neurons for pattern extension"
                                    )
                            else:
                                self.logger.info(
                                    "🔍 [EXPANSION-DEBUG] CHECKPOINT 5 - No cortical mappings found"
                                )
                                self.logger.info(
                                    f"[LOCALIZED-REBUILD] No cortical mappings found for {cortical_id} - skipping pattern extension"
                                )

                        except ImportError as e:
                            self.logger.warning(
                                f"[LOCALIZED-REBUILD] Pattern extension unavailable: {e}"
                            )
                        except Exception as e:
                            self.logger.error(
                                f"[LOCALIZED-REBUILD] Error during pattern extension: {e}"
                            )
                            import traceback

                            self.logger.error(
                                f"[LOCALIZED-REBUILD] Traceback: {traceback.format_exc()}"
                            )

                    self.logger.info(
                        f"[LOCALIZED-REBUILD] Cortical area {cortical_id} resized - {len(self._connectome_manager.neuron_array.free_indices)} neurons in free pool"
                    )
                else:
                    self.logger.info(
                        f"[LOCALIZED-REBUILD] No dimension change needed for {cortical_id}"
                    )
            else:
                #  Area doesn't exist - create it properly using existing
                #  methods
                self.logger.info(
                    f"[LOCALIZED-REBUILD] Creating new cortical area {cortical_id}"
                )
                self._connectome_manager.add_cortical_area(
                    name=properties["name"],
                    dimensions=tuple(properties["dimensions"]),
                    position=tuple(properties["position"]),
                    area_type=properties.get("area_type", "custom"),
                    properties=properties,
                    cortical_id=cortical_id,
                )

            # 4. Update area properties without disrupting neuron structure
            if old_area_existed:
                area = self._connectome_manager.cortical_areas[cortical_id]

                # Update properties in-place - preserves neuron assignments
                if "name" in properties:
                    area.name = properties["name"]
                # Only update runtime position if caller explicitly changed coordinates
                if "coordinates_3d" in changes and "position" in properties:
                    area.position = tuple(properties["position"])
                if "area_type" in properties:
                    area.area_type = properties["area_type"]

                # Update additional properties
                for key, value in properties.items():
                    if key not in [
                        "name",
                        "position",
                        "area_type",
                        "dimensions",
                    ]:
                        area.properties[key] = value

                self.logger.info(
                    f"[LOCALIZED-REBUILD] Updated properties for existing area {cortical_id}"
                )

            # 5. Update connections if needed (preserve existing structure)
            #  Only rebuild connections that specifically involve dimension
            #  changes AND have existing mappings
            if old_area_existed and new_dimensions != old_dimensions:
                self.logger.info(
                    f"[LOCALIZED-REBUILD] Checking for existing connections to rebuild for area {cortical_id}"
                )

                #  Check if this area has any existing cortical mappings to
                #  rebuild
                has_outgoing_mappings = False
                has_incoming_mappings = False

                # Check for outgoing mappings (this area -> others)
                for gene_key in current_genome["blueprint"].keys():
                    if isinstance(gene_key, str) and gene_key.startswith(
                        f"_____10c-{cortical_id}-cx-dstmap-d"
                    ):
                        has_outgoing_mappings = True
                        break

                # Check for incoming mappings (others -> this area)
                if (
                    not has_outgoing_mappings
                ):  # Only check if we haven't found outgoing yet
                    for gene_key, gene_value in current_genome[
                        "blueprint"
                    ].items():
                        if (
                            isinstance(gene_key, str)
                            and gene_key.startswith("_____10c-")
                            and gene_key.endswith("-cx-dstmap-d")
                            and not gene_key.startswith(
                                f"_____10c-{cortical_id}-"
                            )
                            and isinstance(gene_value, dict)
                            and cortical_id in gene_value
                        ):
                            has_incoming_mappings = True
                            break

                if has_outgoing_mappings or has_incoming_mappings:
                    self.logger.info(
                        f"[LOCALIZED-REBUILD] Found existing mappings for {cortical_id} - rebuilding synaptic connections"
                    )
                    #  CRITICAL: Rebuild synaptic connections to include new
                    #  neurons in expanded area
                    #  This ensures cortical mappings extend to all neurons in
                    #  the resized area
                    self._rebuild_connections_for_area(
                        cortical_id, current_genome
                    )
                    self.logger.info(
                        f"[LOCALIZED-REBUILD] Rebuilt synaptic connections for expanded area {cortical_id}"
                    )
                else:
                    self.logger.info(
                        f"[LOCALIZED-REBUILD] No existing cortical mappings found for {cortical_id} - skipping connection rebuild"
                    )
                    self.logger.info(
                        f"[LOCALIZED-REBUILD] Area {cortical_id} was likely created via API and has no predefined mappings"
                    )

            # NO VISUALIZATION INTERRUPTION NEEDED
            #  Since we're not deleting/recreating neurons, neuron IDs remain
            #  stable
            # and the visualization stream can continue uninterrupted

            if transaction:
                transaction.commit()

            # 🔍 CHECKPOINT 8: Final results
            final_synapse_count = self._connectome_manager.get_synapse_count()
            final_neuron_count = self._connectome_manager.get_neuron_count()
            synapses_added = final_synapse_count - initial_synapse_count
            neurons_added = final_neuron_count - initial_neuron_count

            duration = time.time() - start_time

            self.logger.info(
                "🔍 [EXPANSION-DEBUG] CHECKPOINT 8 - Final Results:"
            )
            self.logger.info(
                f"🔍 [EXPANSION-DEBUG]   - Final synapses: {final_synapse_count} (+{synapses_added})"
            )
            self.logger.info(
                f"🔍 [EXPANSION-DEBUG]   - Final neurons: {final_neuron_count} (+{neurons_added})"
            )
            self.logger.info(
                f"🔍 [EXPANSION-DEBUG]   - Duration: {duration * 1000:.1f}ms"
            )

            self.logger.info(
                f"[LOCALIZED-REBUILD] Real-time cortical area resize completed for {cortical_id} "
                f"in {duration * 1000:.1f}ms (no neuron deletion, GPU-friendly)"
            )
            return current_genome["blueprint"][cortical_id]

        except Exception as e:
            duration = time.time() - start_time
            self.logger.error(
                f"Real-time cortical area resize failed after {duration * 1000:.1f}ms: {e}"
            )

            if transaction:
                transaction.rollback()
            raise e

    def _reuse_neurons_for_area_expansion(
        self,
        cortical_id: str,
        additional_neurons_needed: int,
        properties: Dict[str, Any],
    ) -> List[int]:
        """Create additional neurons for cortical area expansion using FEAGI's
        proper allocation.

        ARCHITECTURE COMPLIANCE: Uses NeuronArray.batch_create_neurons() with free pool reuse
        instead of direct SoA manipulation. This ensures Rust-friendly memory patterns.

        Args:
            cortical_id: The cortical area being expanded
            additional_neurons_needed: Number of additional neurons required
            properties: Area properties for neuron configuration
        """
        if additional_neurons_needed <= 0:
            return

        self.logger.info(
            f"[EXPANSION] Creating {additional_neurons_needed} additional neurons for {cortical_id} (FEAGI-compliant)"
        )

        try:
            # Access area via connectome manager (used below for dimensions)
            area = self._connectome_manager.cortical_areas[cortical_id]

            #  Generate positions for additional neurons distributed across
            #  expanded area
            positions = self._generate_positions_for_expansion(
                cortical_id, additional_neurons_needed, properties
            )

            #  Use ConnectomeManager's batch creation method (handles position
            #  mapping automatically)
            neuron_ids = self._connectome_manager.batch_create_neurons(
                cortical_id=cortical_id,
                positions=positions,
                threshold=properties.get("fire_t", 1.0),
                membrane_potential=0.0,
                resting_potential=0.0,
                decay_rate=1.0 - (properties.get("leak_c", 0) / 100.0),
                refractory_period=properties.get("refrac", 1),
            )

            # Update per-area excitability cache in NPU
            excitability = properties.get("neuron_excitability", 1.0)
            try:
                npu = getattr(self._connectome_manager, "_npu_interface", None)
                if npu and hasattr(npu, "set_area_excitability"):
                    cidx = self._connectome_manager.cortical_mapping.get_idx(cortical_id)
                    if cidx is not None:
                        npu.set_area_excitability(cidx, float(excitability))
            except Exception:
                pass

            self.logger.info(
                f"[EXPANSION] Created {len(neuron_ids)} expansion neurons with automatic position mapping and excitability={excitability}"
            )

            free_pool_size = len(
                self._connectome_manager.neuron_array.free_indices
            )
            self.logger.info(
                f"[EXPANSION] Successfully allocated and registered {len(neuron_ids)} expansion neurons for {cortical_id} "
                f"(free pool size: {free_pool_size})"
            )

            return neuron_ids

        except Exception as e:
            self.logger.error(
                f"[EXPANSION] Error creating additional neurons for {cortical_id}: {e}"
            )
            raise

    def _generate_positions_for_expansion(
        self, cortical_id: str, neuron_count: int, properties: Dict[str, Any]
    ) -> List[Tuple[int, int, int]]:
        """Generate neuron positions for area expansion.

        CRITICAL FIX: Only place neurons in voxels that don't already have neurons.
        This ensures expansion neurons go to the NEW expanded regions, not existing ones.

        Args:
            cortical_id: The cortical area being expanded
            neuron_count: Number of neurons to place
            properties: Area properties containing density information

        Returns:
            List of (x, y, z) position tuples for NEW/EMPTY voxels only
        """
        area = self._connectome_manager.cortical_areas[cortical_id]
        width, height, depth = area.dimensions
        positions = []

        # Find all voxels that DON'T have neurons yet (NPU SoA authoritative)
        empty_voxels = []
        try:
            npu = getattr(self._connectome_manager, "_npu_interface", None)
            if npu is None:
                raise RuntimeError("NPU Interface required for expansion position generation")
            cortical_idx = npu.get_cortical_idx_by_id(cortical_id)
            if cortical_idx is None:
                raise RuntimeError(f"Unknown cortical_id: {cortical_id}")

            na = npu.neuron_array
            import numpy as np
            total = int(na.neuron_count)
            if total > 0:
                mask = (na.cortical_idxs[:total] == cortical_idx)
                ix = np.nonzero(mask)[0]
                occ = set(zip(na.coordinates_x[ix].tolist(), na.coordinates_y[ix].tolist(), na.coordinates_z[ix].tolist()))
            else:
                occ = set()

            for x in range(width):
                for y in range(height):
                    for z in range(depth):
                        position = (x, y, z)
                        if position not in occ:
                            empty_voxels.append(position)
        except Exception as e:
            self.logger.error(f"[EXPANSION] NPU position scan failed for {cortical_id}: {e}")
            # If NPU scan fails, proceed with no empty voxels

        self.logger.info(
            f"[EXPANSION] Found {len(empty_voxels)} empty voxels in expanded area {cortical_id}"
        )
        self.logger.info(
            f"[EXPANSION] Empty voxels: {empty_voxels[:10]}..."
        )  # Log first 10 for debugging

        #  Distribute neurons across the empty voxels (the newly expanded
        #  regions)
        if len(empty_voxels) == 0:
            # No empty voxels available in the expanded region; return no positions
            self.logger.warning(
                f"[EXPANSION] No empty voxels available for {cortical_id}; generated 0 positions"
            )
            return positions

        for i in range(neuron_count):
            if i < len(empty_voxels):
                positions.append(empty_voxels[i])
            else:
                # If we have more neurons than empty voxels, cycle through them
                voxel_idx = i % len(empty_voxels)
                positions.append(empty_voxels[voxel_idx])

        self.logger.info(
            f"[EXPANSION] Generated {len(positions)} positions for expansion neurons: {positions}"
        )
        return positions

    def _extract_area_properties_from_genome(
        self, cortical_id: str, genome: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Extract cortical area properties from hierarchical genome format."""
        try:
            area_def = genome["blueprint"][cortical_id]

            # Build base properties
            props = {
                "name": area_def.get("cortical_name", cortical_id),
                "dimensions": area_def.get("cortical_dimensions", [1, 1, 1]),
                "position": (
                    area_def.get("coordinates_3d")
                    or area_def.get("coordinates")
                    or area_def.get("relative_coordinate")
                    or [0, 0, 0]
                ),
                "area_type": area_def.get("cortical_type", "custom"),
                "neurons_per_voxel": area_def.get("parameters", {}).get(
                    "per_voxel_neuron_cnt", 1
                ),
                "fire_t": area_def.get("parameters", {}).get(
                    "firing_threshold", 1.0
                ),
                "leak_c": area_def.get("parameters", {}).get(
                    "leak_coefficient", 0.0
                ),
                "refrac": area_def.get("parameters", {}).get(
                    "refractory_period", 1
                ),
                "fire_increment": area_def.get("parameters", {}).get(
                    "fire_increment", 0.0
                ),
                "leak_variability": area_def.get("parameters", {}).get(
                    "leak_variability", 0.0
                ),
            }
            # Include 2D coordinates if available to preserve UI placement
            coords_2d = area_def.get("2d_coordinate")
            if coords_2d is None:
                coords_2d = area_def.get("parameters", {}).get("coordinates_2d")
            if coords_2d is not None:
                props["coordinates_2d"] = coords_2d
            return props
        except KeyError as e:
            raise ValueError(
                f"Missing required property in genome for area {cortical_id}: {e}"
            ) from e

    def _rebuild_neurons_for_area(
        self, cortical_id: str, properties: Dict[str, Any]
    ) -> None:
        """Rebuild neurons for a specific cortical area using FEAGI-compliant
        allocation.

        ARCHITECTURE COMPLIANCE: Uses NeuronArray.batch_create_neurons() with free pool reuse
        instead of direct SoA manipulation. This ensures Rust-friendly memory patterns.
        """
        try:
            area = self._connectome_manager.cortical_areas[cortical_id]

            # Calculate neuron count for this area
            width, height, depth = area.dimensions
            neurons_per_voxel = properties.get("neurons_per_voxel", 1)
            area_neuron_count = width * height * depth * neurons_per_voxel

            self.logger.info(
                f"[LOCALIZED-REBUILD] Creating {area_neuron_count} neurons for {cortical_id} (FEAGI-compliant)"
            )

            # Generate all positions for the cortical area
            positions = []
            for x in range(width):
                for y in range(height):
                    for z in range(depth):
                        for _ in range(neurons_per_voxel):
                            positions.append((x, y, z))

            # Calculate base properties
            base_threshold = properties.get("fire_t", 1.0)
            base_decay_rate = 1.0 - (properties.get("leak_c", 0) / 100.0)
            base_refractory = properties.get("refrac", 1)

            # Handle position-based variations for thresholds
            thresholds = [base_threshold] * area_neuron_count
            fire_increment = properties.get("fire_increment", 0.0)
            if fire_increment != 0.0:
                for i, (x, y, z) in enumerate(positions):
                    thresholds[i] = base_threshold + (z * fire_increment)

            # Handle leak variability for decay rates
            decay_rates = [base_decay_rate] * area_neuron_count
            leak_variability = properties.get("leak_variability", 0.0)
            base_leak = properties.get("leak_c", 0.0)
            if leak_variability != 0.0 and base_leak != 0.0:
                import numpy as np

                np.random.seed(42)  # Deterministic for reproducibility
                variations = (
                    np.random.uniform(
                        -leak_variability, leak_variability, area_neuron_count
                    )
                    / 100.0
                )
                for i in range(area_neuron_count):
                    varied_leak = np.clip(
                        base_leak / 100.0 + variations[i], 0.0, 1.0
                    )
                    decay_rates[i] = 1.0 - varied_leak

            #  Use ConnectomeManager's batch creation method (handles position
            #  mapping automatically)
            neuron_ids = self._connectome_manager.batch_create_neurons(
                cortical_id=cortical_id,
                positions=positions,
                threshold=thresholds,
                membrane_potential=0.0,
                resting_potential=0.0,
                decay_rate=decay_rates,
                refractory_period=base_refractory,
            )

            # CRITICAL FIX: Set excitability for all created neurons
            excitability = properties.get("neuron_excitability", 1.0)
            neuron_array = self._connectome_manager.neuron_array
            # Per-neuron excitability removed; handled via per-area cache

            self.logger.info(
                f"[LOCALIZED-REBUILD] Created {len(neuron_ids)} neurons with automatic position mapping and excitability={excitability}"
            )

            free_pool_size = len(
                self._connectome_manager.neuron_array.free_indices
            )
            self.logger.info(
                f"[LOCALIZED-REBUILD] Successfully allocated and registered {len(neuron_ids)} neurons for {cortical_id} "
                f"(free pool size after allocation: {free_pool_size})"
            )

        except Exception as e:
            self.logger.error(
                f"Failed to rebuild neurons for area {cortical_id}: {e}"
            )
            raise

    def _rebuild_connections_for_area(
        self, cortical_id: str, genome: Dict[str, Any]
    ) -> None:
        """Rebuild synaptic connections involving the specified cortical area.

        ARCHITECTURE COMPLIANCE: Uses ConnectomeManager directly to avoid circular dependencies.
        GenomeService should not access CoreAPIService (which creates GenomeService).
        """
        try:
            # Import NeuroEmbryogenesis for connection building
            from feagi.bdu.embryogenesis.neuroembryogenesis import (
                NeuroEmbryogenesis,
            )

            # Create temporary embryogenesis instance for connection building
            embryogenesis = NeuroEmbryogenesis(
                self._connectome_manager, self.state_manager
            )
            embryogenesis.genome = genome

            # 1. Remove all existing connections involving this area
            self._remove_area_connections(cortical_id)

            # 2. Rebuild outgoing connections (this area -> others) from genome
            #  Look for cortical mappings in the FLAT genome format
            #  (_____10c-{area}-cx-dstmap-d)
            self.logger.info(
                f"[LOCALIZED-REBUILD] Scanning genome for outgoing connections from {cortical_id}"
            )
            outgoing_mappings = {}
            for gene_key, gene_value in genome["blueprint"].items():
                if isinstance(gene_key, str) and gene_key.startswith(
                    f"_____10c-{cortical_id}-cx-dstmap-d"
                ):
                    self.logger.info(
                        f"[LOCALIZED-REBUILD] Found outgoing mapping: {gene_key}"
                    )
                    if isinstance(gene_value, dict) and gene_value:
                        outgoing_mappings.update(gene_value)

            if outgoing_mappings:
                self.logger.info(
                    f"[LOCALIZED-REBUILD] Rebuilding {len(outgoing_mappings)} outgoing connections from {cortical_id}"
                )
                api_mapping = {cortical_id: outgoing_mappings}
                embryogenesis.update_cortical_mapping(api_mapping)
            else:
                self.logger.info(
                    f"[LOCALIZED-REBUILD] No outgoing mappings found for {cortical_id}"
                )

            # 3. Rebuild incoming connections (others -> this area)
            # Look for any mappings that target this cortical area
            self.logger.info(
                f"[LOCALIZED-REBUILD] Scanning genome for incoming connections to {cortical_id}"
            )
            incoming_mappings = {}
            for gene_key, gene_value in genome["blueprint"].items():
                if (
                    isinstance(gene_key, str)
                    and gene_key.startswith("_____10c-")
                    and gene_key.endswith("-cx-dstmap-d")
                    and not gene_key.startswith(f"_____10c-{cortical_id}-")
                ):
                    # Extract source area from gene key
                    parts = gene_key.split("-")
                    if len(parts) >= 3:
                        source_area_id = parts[1]

                        # Check if this mapping targets our cortical area
                        if (
                            isinstance(gene_value, dict)
                            and cortical_id in gene_value
                        ):
                            if source_area_id not in incoming_mappings:
                                incoming_mappings[source_area_id] = {}
                            incoming_mappings[source_area_id][cortical_id] = (
                                gene_value[cortical_id]
                            )
                            self.logger.info(
                                f"[LOCALIZED-REBUILD] Found incoming mapping: {source_area_id} -> {cortical_id}"
                            )

            # Rebuild each incoming mapping
            for source_area_id, mapping_data in incoming_mappings.items():
                self.logger.info(
                    f"[LOCALIZED-REBUILD] Rebuilding incoming connection: {source_area_id} -> {cortical_id}"
                )
                api_mapping = {source_area_id: mapping_data}
                embryogenesis.update_cortical_mapping(api_mapping)

            total_rebuilt = len(outgoing_mappings) + len(incoming_mappings)
            self.logger.info(
                f"[LOCALIZED-REBUILD] Completed rebuilding {total_rebuilt} connection types for {cortical_id}"
            )

        except Exception as e:
            self.logger.error(
                f"Failed to rebuild connections for area {cortical_id}: {e}"
            )
            import traceback

            self.logger.error(
                f"[LOCALIZED-REBUILD] Traceback: {traceback.format_exc()}"
            )
            raise

    def _remove_area_connections(self, cortical_id: str) -> None:
        """Remove all synaptic connections involving the specified cortical
        area."""
        try:
            #  Get all neurons in the area (before deletion, these would be the
            #  old neurons)
            #  After area deletion and recreation, we need to remove
            #  connections to the old neurons

            #  For now, we rely on ConnectomeManager.delete_cortical_area() to
            #  handle
            #  connection cleanup. This method is called as a safety net for
            #  any remaining connections.

            # Get all existing connections and remove those involving this area
            connection_ids_to_remove = []
            for (
                connection_id,
                connection,
            ) in self._connectome_manager.cortical_connections.items():
                if (
                    connection["source_area_id"] == cortical_id
                    or connection["target_area_id"] == cortical_id
                ):
                    connection_ids_to_remove.append(connection_id)

            for connection_id in connection_ids_to_remove:
                self._connectome_manager.delete_cortical_connection(
                    connection_id, delete_synapses=True
                )

            self.logger.info(
                f"[LOCALIZED-REBUILD] Removed {len(connection_ids_to_remove)} connection definitions for {cortical_id}"
            )

        except Exception as e:
            self.logger.warning(
                f"Error removing connections for area {cortical_id}: {e}"
            )

    def _initialize_state_manager(self):
        """Initialize state manager reference and ensure consistency."""
        try:
            from feagi.core.state_manager import get_state_manager

            self.state_manager = get_state_manager()
        except Exception as e:
            self.logger.error(f"Failed to get state manager: {e}")
            self.state_manager = None

    def _apply_genome_physiology_parameters(
        self, genome_data: Dict[str, Any], core_api_service=None
    ) -> None:
        """Apply genome physiology parameters (like simulation_timestep) to
        system configuration.

        This ensures the genome's simulation_timestep overwrites the current stimulation period.
        Also maintains backward compatibility with old genomes using burst_delay.

        Args:
            genome_data: The loaded genome data containing physiology parameters
        """
        try:
            # Extract simulation_timestep from genome physiology section
            physiology = genome_data.get("physiology", {})
            timestep = physiology.get("simulation_timestep")

            if timestep is None:
                # Backward compatibility: Check for burst_delay in physiology
                timestep = physiology.get("burst_delay")

            if timestep is None:
                # Backward compatibility: Check for old top-level burst_delay
                timestep = genome_data.get("burst_delay")

            if timestep is not None:
                self.logger.info(
                    f"📊 [GENOME] Found simulation timestep in genome: {timestep}s"
                )

                # Validate simulation timestep
                if not isinstance(timestep, (int, float)) or timestep <= 0:
                    self.logger.warning(
                        f"Invalid simulation_timestep in genome: {timestep} (must be positive number)"
                    )
                    return

                # Convert timestep (stimulation period) to frequency
                burst_frequency_hz = 1.0 / float(timestep)

                # Apply same validation as API endpoint
                if burst_frequency_hz <= 0.0 or burst_frequency_hz > 10000.0:
                    self.logger.warning(
                        f"Invalid frequency {burst_frequency_hz}Hz from timestep {timestep}s (must be 0 < freq <= 10000)"
                    )
                    return

                # Get state manager
                if not self.state_manager:
                    self.logger.warning(
                        "No state manager available - cannot apply genome simulation_timestep"
                    )
                    return

                # Use CoreAPIService's proven burst engine update mechanism
                if core_api_service:
                    try:
                        # Use the same method that manual API endpoints use
                        config_update = {
                            "burst_frequency_hz": burst_frequency_hz
                        }
                        success = core_api_service.update_burst_engine_config(
                            config_update
                        )

                        if success:
                            self.logger.info(
                                f"✅ [GENOME] Applied genome simulation_timestep: {timestep}s → {burst_frequency_hz}Hz"
                            )
                        else:
                            self.logger.warning(
                                f"Failed to apply genome frequency {burst_frequency_hz}Hz to burst engine"
                            )

                    except Exception as e:
                        self.logger.warning(
                            f"Error updating burst engine with genome frequency: {e}"
                        )
                        # Fallback to state manager only
                        self.state_manager.set_burst_frequency(
                            burst_frequency_hz
                        )
                        self.logger.info(
                            f"✅ [GENOME] Applied genome simulation_timestep to state manager: {timestep}s → {burst_frequency_hz}Hz"
                        )
                else:
                    # Fallback: Update state manager only
                    self.state_manager.set_burst_frequency(burst_frequency_hz)
                    self.logger.info(
                        f"✅ [GENOME] Applied genome simulation_timestep to state manager: {timestep}s → {burst_frequency_hz}Hz"
                    )
                    self.logger.warning(
                        "CoreAPIService not available - frequency will apply on next restart"
                    )

            else:
                self.logger.debug(
                    "No simulation_timestep or burst_delay found in genome physiology - keeping current stimulation period"
                )

        except Exception as e:
            self.logger.error(
                f"Error applying genome physiology parameters: {e}"
            )
            import traceback

            self.logger.debug(
                f"Genome physiology error traceback: {traceback.format_exc()}"
            )

    def get_current_genome(self) -> Dict[str, Any]:
        """Get the current genome copy (internal helper)."""
        return self._current_genome or {}

    def update_physiology(self, updates: Dict[str, Any]) -> bool:
        """Update physiology section in the current genome and validate.

        Args:
            updates: dict of physiology fields to update

        Returns:
            True if updated and valid; False otherwise
        """
        try:
            if not isinstance(self._current_genome, dict):
                self.logger.error("No genome loaded; cannot update physiology")
                return False
            genome = self._current_genome
            if "physiology" not in genome or not isinstance(
                genome["physiology"], dict
            ):
                genome["physiology"] = {}
            physiology = genome["physiology"]

            # Merge updates (typed correction will be handled by validator)
            for k, v in (updates or {}).items():
                physiology[k] = v

            # Validate physiology and sanitize missing fields/types
            try:
                from feagi.evo.genome_validator import (
                    sanitize_missing_physiology,
                    validate_physiology_section,
                )

                sanitize_missing_physiology(genome)
                result = validate_physiology_section(genome)
                if not result.get("valid", False):
                    self.logger.warning(
                        f"Physiology validation warnings: {result.get('errors', [])}"
                    )
            except Exception as e:
                # Do not fail the update due to validation logging only
                self.logger.debug(
                    f"Physiology validation skipped due to error: {e}"
                )

            # Update connectome manager reference if it keeps a genome copy
            try:
                if hasattr(self._connectome_manager, "genome"):
                    self._connectome_manager.genome = genome
            except Exception:
                pass

            # Persist state manager brain stats if needed—no change here
            return True
        except Exception as e:
            self.logger.error(f"Failed to update physiology: {e}")
            return False
