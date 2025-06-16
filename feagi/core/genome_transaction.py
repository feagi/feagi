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

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)
import time
import uuid
from contextlib import contextmanager

from feagi.core.state_manager import GenomeState


class GenomeTransaction:
    """Handles atomic genome modifications with connectome synchronization"""

    def __init__(self, description: str, core_api_service=None):
        """Initialize a genome transaction.

        Args:
            description: Human-readable description of what this transaction does
            core_api_service: CoreAPIService instance (will be obtained via dependency injection if not provided)
        """
        self.description = description
        self._core_api_service = core_api_service
        self._state_manager = None
        self._changes = []
        self._has_committed = False
        self._transaction_id = str(uuid.uuid4())

    def record_change(self, operation, path, old_value, new_value):
        """Record a change to be applied in this transaction.

        Args:
            operation: The type of change (add, modify, delete)
            path: Path to the genome element being changed
            old_value: Previous value (for rollback)
            new_value: New value to be set
        """
        self._changes.append(
            {
                "operation": operation,
                "path": path,
                "old_value": old_value,
                "new_value": new_value,
                "timestamp": time.time(),
            }
        )

    def commit(self):
        """Commit all changes and synchronize with connectome"""
        if self._has_committed:
            try:
                logger.warning("Transaction already committed", status="[WARN]")
            except TypeError:
                logger.warning("[WARN] Transaction already committed")
            return False

        try:
            # Apply changes to genome
            for change in self._changes:
                self._apply_to_genome(
                    change["operation"],
                    change["path"],
                    change["old_value"],
                    change["new_value"],
                )

            # Synchronize with connectome
            self._synchronize_connectome()

            # Update state - using LOADED instead of MODIFIED
            self._get_state_manager().set_genome_state(GenomeState.LOADED)

            self._has_committed = True
            return True
        except Exception as e:
            # Roll back changes
            try:
                logger.error(f"Transaction failed: {e}", status="[ERR]")
            except TypeError:
                logger.error(f"[ERR] Transaction failed: {e}")
            self.rollback()
            self._get_state_manager().set_genome_state(GenomeState.ERROR)
            return False

    def rollback(self):
        """Revert all changes made in this transaction."""
        if not self._has_committed:
            return

        # Get latest genome
        genome = self._get_core_api_service().get_genome()
        if not genome:
            try:
                logger.error("Cannot rollback - no genome loaded", status="[WARN]")
            except TypeError:
                logger.error("[WARN] Cannot rollback - no genome loaded")
            return

        # Apply changes in reverse order
        for change in reversed(self._changes):
            try:
                # Apply the reverse operation
                if change["operation"] == "add":
                    self._delete_at_path(genome, change["path"])
                elif change["operation"] == "delete":
                    self._set_at_path(genome, change["path"], change["old_value"])
                elif change["operation"] == "modify":
                    self._set_at_path(genome, change["path"], change["old_value"])
                # Add support for update_cortical_area used in tests
                elif change["operation"] == "update_cortical_area":
                    self._set_at_path(
                        genome, f"cortical_areas.{change['path']}", change["old_value"]
                    )

            except Exception as e:
                try:
                    logger.error(f"Error during rollback: {e}", status="[WARN]")
                except TypeError:
                    logger.error(f"[WARN] Error during rollback: {e}")

        # Re-synchronize connectome after rollback
        self._synchronize_connectome(genome)
        # Update state
        self._get_state_manager().set_genome_state(GenomeState.LOADED)
        try:
            logger.info(
                f"Transaction {self._transaction_id} rolled back: {self.description}",
                status="[BACK]",
            )
        except TypeError:
            logger.info(
                f"[BACK] Transaction {self._transaction_id} rolled back: {self.description}"
            )

    def _set_at_path(self, genome, path, value):
        """Set a value at the specified path in the genome."""
        parts = path.split(".")
        current = genome
        for _i, part in enumerate(parts[:-1]):
            if part not in current:
                current[part] = {}
            current = current[part]
        current[parts[-1]] = value

    def _delete_at_path(self, genome, path):
        """Delete a value at the specified path in the genome."""
        parts = path.split(".")
        current = genome
        for part in parts[:-1]:
            if part not in current:
                return
            current = current[part]
        if parts[-1] in current:
            del current[parts[-1]]

    def _apply_to_genome(self, operation, path, old_value, new_value):
        """Apply a change to the genome"""
        logger.info(f"Applying genome operation: {operation}", status="[DNA]")

        # Handle different operation types
        if operation == "update_cortical_area":
            cortical_id = path
            properties = new_value
            self._core_api_service.update_cortical_area_properties(
                cortical_id, properties
            )

        elif operation == "add_cortical_area":
            self._core_api_service.add_cortical_area(new_value)

        elif operation == "delete_cortical_area":
            self._core_api_service.delete_cortical_area(path)

        # Add more operation types as needed
        else:
            raise ValueError(f"Unknown operation type: {operation}")

    def _synchronize_connectome(self, genome=None):
        """Synchronize changes with the connectome

        Args:
            genome: Optional genome data. If not provided, will get from core_api_service
        """
        try:
            logger.info("Synchronizing changes with connectome", status="[PROC]")
        except TypeError:
            logger.info("[PROC] Synchronizing changes with connectome")

        if genome is None and self._core_api_service:
            genome = self._core_api_service.get_genome()

        # Handle synchronization based on changes
        # This would depend on the specific requirements of each change type

    @contextmanager
    def transaction_scope(self):
        """Context manager for automatic commit/rollback"""
        try:
            yield self
            self.commit()
        except Exception:
            self.rollback()
            raise

    def _get_core_api_service(self):
        """Get the CoreAPIService instance."""
        if self._core_api_service is None:
            raise ValueError("No CoreAPIService instance provided")
        return self._core_api_service

    def _get_state_manager(self):
        """Get the FeagiStateManager instance."""
        if self._state_manager is None:
            if self._core_api_service and hasattr(
                self._core_api_service, "state_manager"
            ):
                self._state_manager = self._core_api_service.state_manager
            else:
                from feagi.core.state_manager import FeagiStateManager

                self._state_manager = FeagiStateManager.instance()
        return self._state_manager


def begin_genome_transaction_context(self):
    """Context manager for genome transactions"""
    from contextlib import contextmanager

    from feagi.core.genome_transaction import GenomeTransaction

    @contextmanager
    def transaction_context():
        transaction = GenomeTransaction(self)
        try:
            yield transaction
            transaction.commit()
        except Exception:
            transaction.rollback()
            raise

    return transaction_context()
