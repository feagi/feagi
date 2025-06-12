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

"""Base service class for all FEAGI domain services."""

from typing import Any, Optional

from feagi.utils.logger import setup_logger

logger = setup_logger()


class BaseService:
    """
    Base class for all FEAGI domain services.

    Provides common functionality and patterns used across all services.
    """

    def __init__(self, connectome_manager, state_manager=None):
        """
        Initialize base service.

        Args:
            connectome_manager: ConnectomeManager instance
            state_manager: FeagiStateManager instance (optional)
        """
        self._connectome_manager = connectome_manager
        self.state_manager = state_manager
        self.logger = logger

    def _validate_connectome_ready(self) -> bool:
        """Check if connectome manager is ready for operations."""
        if not self._connectome_manager:
            self.logger.warning("Connectome manager not available")
            return False

        if (
            not hasattr(self._connectome_manager, "fcl_manager")
            or not self._connectome_manager.fcl_manager
        ):
            self.logger.warning("FCL manager not initialized")
            return False

        return True

    def _validate_genome_loaded(self) -> bool:
        """
        Check if a genome is currently loaded with robust fallback validation.

        This method uses multiple validation approaches to handle timing issues
        between genome loading and state manager synchronization.
        """
        # Method 1: Check state manager (preferred method)
        if self.state_manager:
            try:
                state_manager_result = self.state_manager.is_genome_loaded()
                if state_manager_result:
                    return True
                else:
                    # State manager says no genome, but let's double-check with connectome
                    self.logger.debug(
                        "State manager reports no genome loaded, checking connectome directly"
                    )
            except Exception as e:
                self.logger.warning(f"Error checking state manager genome status: {e}")

        # REMOVED: Unapproved connectome pre-loading mechanism
        # Previously this method would check if cortical areas exist in connectome
        # and treat that as "genome loaded", bypassing proper neuroembryogenesis.
        # This caused corruption by loading connectomes without BiDirectionalCorticalMap
        # synchronization. Connectome serialization/deserialization should be
        # user-controlled features, not automatic defaults.

        self.logger.debug(
            "Genome validation: Only checking state manager - no automatic connectome loading"
        )
        return False

    def _safe_execute(self, operation, error_message: str, default_return=None):
        """
        Safely execute an operation with error handling.

        Args:
            operation: Function to execute
            error_message: Error message prefix
            default_return: Default value to return on error

        Returns:
            Operation result or default_return on error
        """
        try:
            return operation()
        except Exception as e:
            self.logger.error(f"{error_message}: {str(e)}")
            return default_return

    def _validate_state_consistency(self) -> bool:
        """
        Validate that state manager and connectome manager are in sync.

        This method ensures that both managers have consistent data and
        that all required attributes are properly set for health checks.

        Returns:
            bool: True if state is consistent, False otherwise
        """
        try:
            if not self.state_manager:
                self.logger.warning(
                    "Cannot validate state consistency - no state manager"
                )
                return False

            if not self._connectome_manager:
                self.logger.warning(
                    "Cannot validate state consistency - no connectome manager"
                )
                return False

            # Check if genome is loaded
            if not self.state_manager.is_genome_loaded():
                self.logger.debug("Genome not loaded - state validation skipped")
                return True  # This is a valid state

            # Validate critical attributes exist for health checks
            required_attributes = [
                "brain_stats",
                "cortical_list",
                "genome_validity",
                "connected_agents",
                "changes_saved_externally",
                "exit_condition",
            ]

            missing_attributes = []
            for attr in required_attributes:
                if not hasattr(self.state_manager, attr):
                    missing_attributes.append(attr)

            if missing_attributes:
                self.logger.warning(
                    f"State manager missing critical attributes: {missing_attributes}"
                )
                return False

            # Validate brain_stats has the expected structure
            brain_stats = getattr(self.state_manager, "brain_stats", {})
            if not isinstance(brain_stats, dict):
                self.logger.warning("brain_stats is not a dictionary")
                return False

            expected_stats = ["neuron_count", "synapse_count", "cortical_area_count"]
            for stat in expected_stats:
                if stat not in brain_stats:
                    self.logger.warning(f"brain_stats missing key: {stat}")
                    return False

            # Validate cortical_list exists and is non-empty for loaded genomes
            cortical_list = getattr(self.state_manager, "cortical_list", [])
            if not isinstance(cortical_list, list):
                self.logger.warning("cortical_list is not a list")
                return False

            # Check connectome manager consistency
            if hasattr(self._connectome_manager, "cortical_areas"):
                connectome_area_count = len(self._connectome_manager.cortical_areas)
                state_area_count = brain_stats.get("cortical_area_count", 0)

                if connectome_area_count != state_area_count:
                    self.logger.warning(
                        f"Cortical area count mismatch: connectome={connectome_area_count}, state={state_area_count}"
                    )
                    return False

            self.logger.debug("State consistency validation passed")
            return True

        except Exception as e:
            self.logger.error(f"Error during state consistency validation: {str(e)}")
            return False

    def _sync_state_if_needed(self) -> bool:
        """
        Synchronize state between managers if they're out of sync.

        This method attempts to fix any inconsistencies found during validation.

        Returns:
            bool: True if sync was successful or not needed, False if sync failed
        """
        try:
            if not self.state_manager or not self._connectome_manager:
                return False

            # Only sync if genome is loaded
            if not self.state_manager.is_genome_loaded():
                return True

            # Re-sync brain statistics from connectome manager
            self.logger.info("Re-synchronizing state manager with connectome manager")

            # Get fresh statistics from connectome manager
            cortical_area_count = len(
                getattr(self._connectome_manager, "cortical_areas", {})
            )

            # Calculate neuron and synapse counts
            total_neurons = 0
            total_synapses = 0

            if hasattr(self._connectome_manager, "get_total_neuron_count"):
                total_neurons = self._connectome_manager.get_total_neuron_count()
            elif hasattr(self._connectome_manager, "cortical_areas"):
                # Fallback: count neurons in all cortical areas
                for area_idx in self._connectome_manager.cortical_areas:
                    try:
                        if hasattr(self._connectome_manager, "get_neurons_by_area"):
                            area_neurons = self._connectome_manager.get_neurons_by_area(
                                area_idx
                            )
                            total_neurons += len(area_neurons) if area_neurons else 0
                    except Exception:
                        pass

            if hasattr(self._connectome_manager, "get_total_synapse_count"):
                total_synapses = self._connectome_manager.get_total_synapse_count()

            # Update brain statistics
            self.state_manager.brain_stats = {
                "neuron_count": total_neurons,
                "synapse_count": total_synapses,
                "cortical_area_count": cortical_area_count,
            }

            # Update cortical list
            cortical_ids = []
            if hasattr(self._connectome_manager, "cortical_areas"):
                for area_idx, area in self._connectome_manager.cortical_areas.items():
                    if hasattr(area, "cortical_id") and area.cortical_id:
                        cortical_ids.append(area.cortical_id)
                    else:
                        cortical_ids.append(f"CID{area_idx:03d}")
            self.state_manager.cortical_list = cortical_ids

            # Ensure other required attributes exist
            if (
                not hasattr(self.state_manager, "connected_agents")
                or self.state_manager.connected_agents is None
            ):
                self.state_manager.connected_agents = (
                    {}
                )  # Dictionary of connected agents, not a count

            if not hasattr(self.state_manager, "changes_saved_externally"):
                self.state_manager.changes_saved_externally = False

            if not hasattr(self.state_manager, "exit_condition"):
                self.state_manager.exit_condition = False

            if not hasattr(self.state_manager, "genome_validity"):
                self.state_manager.genome_validity = (
                    True  # Assume valid if genome is loaded
                )

            self.logger.info("State synchronization completed successfully")
            return True

        except Exception as e:
            self.logger.error(f"Error during state synchronization: {str(e)}")
            return False
