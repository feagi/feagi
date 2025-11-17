"""FEAGI BiDirectional Cortical Mapping.

RTOS/GPU/Rust/SIMD-Compatible O(1) Cortical ID ↔ Index Translation

This module provides lock-free, atomic bidirectional mapping between cortical
area IDs (strings) and cortical indices (integers) with zero fallback logic.

Architecture Principles:
- Lock-free atomic operations (RTOS-compatible)
- GPU memory layout friendly (Dict → Array conversion ready)
- Rust migration ready (HashMap<String, i32> and HashMap<i32, String>)
- SIMD optimization ready (vectorizable batch operations)
- Single solid execution path (no exceptions, no fallbacks)
    - Core areas pre-allocated (_death=0, _power=1)
"""

# Standard imports
import logging
import threading
from typing import Dict, Optional, Set, Tuple

# MEMORY OPTIMIZATION: Import invalid cortical index constant
from feagi.bdu.models.neuron import INVALID_CORTICAL_IDX

logger = logging.getLogger(__name__)


class BiDirectionalCorticalMap:
    """Lock-free bidirectional cortical_id ↔ cortical_idx mapping for O(1)
    lookups.

    RTOS/GPU/Rust/SIMD Compliance:
    - No threading locks (atomic dict operations only)
    - Simple data structures (Dict[str, int] and Dict[int, str])
    - Direct Rust HashMap translation path
    - GPU array/texture convertible
    - Vectorizable batch operations
    - Single solid execution path

    Core Areas:
    - _death → 0 (always present, cannot be removed)
            - _power → 1 (always present, cannot be removed)
    """

    def __init__(self):
        """Initialize bidirectional mapping with core areas pre-allocated."""
        # Thread safety lock for concurrent access
        self.__lock = threading.RLock()
        
        # Private internal dictionaries - no external access allowed
        self.__id_to_idx: Dict[str, int] = {}
        self.__idx_to_id: Dict[int, str] = {}

        # Pre-allocate core areas - these MUST always be present
        # No fallbacks - if these aren't here, it's a system failure
        self.__id_to_idx["_death"] = 0
        self.__id_to_idx["_power"] = 1
        self.__idx_to_id[0] = "_death"
        self.__idx_to_id[1] = "_power"

    def add_mapping(self, cortical_id: str, cortical_idx: int) -> bool:
        """Add bidirectional mapping with conflict resolution.

        Single solid execution path - no exceptions, just error codes.

        Args:
            cortical_id: String identifier for cortical area
            cortical_idx: Integer index for cortical area

        Returns:
            True if mapping added successfully, False if invalid input
        """
        with self.__lock:
            # Input validation - single path, no exceptions
            if not cortical_id or cortical_idx == INVALID_CORTICAL_IDX:
                return False

            # Protect core areas (0,1) from modification
            if cortical_idx in (0, 1) and cortical_id not in ("_death", "_power"):
                return False

            # Atomic update - remove any existing conflicting mappings
            existing_idx = self.__id_to_idx.get(cortical_id)
            existing_id = self.__idx_to_id.get(cortical_idx)

            if existing_idx is not None and existing_idx != cortical_idx:
                del self.__idx_to_id[existing_idx]

            if existing_id is not None and existing_id != cortical_id:
                del self.__id_to_idx[existing_id]

            # Add new mapping atomically
            self.__id_to_idx[cortical_id] = cortical_idx
            self.__idx_to_id[cortical_idx] = cortical_id

            return True

    def get_idx(self, cortical_id: str) -> Optional[int]:
        """Get cortical_idx from cortical_id (O(1) lookup).

        Thread-safe atomic read operation.

        Args:
            cortical_id: String identifier to look up

        Returns:
            Integer cortical_idx if found, None otherwise
        """
        with self.__lock:
            return self.__id_to_idx.get(cortical_id)

    def get_id(self, cortical_idx: int) -> Optional[str]:
        """Get cortical_id from cortical_idx (O(1) lookup).

        Thread-safe atomic read operation.

        Args:
            cortical_idx: Integer index to look up

        Returns:
            String cortical_id if found, None otherwise
        """
        with self.__lock:
            return self.__idx_to_id.get(cortical_idx)

    def remove_by_id(self, cortical_id: str) -> bool:
        """Remove mapping by cortical_id.

        Single solid execution path with core area protection.

        Args:
            cortical_id: String identifier to remove

        Returns:
            True if mapping was removed, False if not found or protected
        """
        # Protect core areas from removal
        if cortical_id in ("_death", "_power"):
            return False

        cortical_idx = self.__id_to_idx.get(cortical_id)
        if cortical_idx is not None:
            del self.__id_to_idx[cortical_id]
            if cortical_idx in self.__idx_to_id:
                del self.__idx_to_id[cortical_idx]
            return True
        return False

    def remove_by_idx(self, cortical_idx: int) -> bool:
        """Remove mapping by cortical_idx.

        Single solid execution path with core area protection.

        Args:
            cortical_idx: Integer index to remove

        Returns:
            True if mapping was removed, False if not found or protected
        """
        # Protect core areas from removal
        if cortical_idx in (0, 1):
            return False

        cortical_id = self.__idx_to_id.get(cortical_idx)
        if cortical_id is not None:
            del self.__idx_to_id[cortical_idx]
            if cortical_id in self.__id_to_idx:
                del self.__id_to_idx[cortical_id]
            return True
        return False

    def validate_consistency(self) -> Tuple[bool, Set[str]]:
        """Validate bidirectional mapping consistency.

        Lock-free validation for RTOS compatibility.

        Returns:
            Tuple of (is_consistent, set_of_error_messages)
        """
        errors = set()

        # Check id_to_idx → idx_to_id consistency
        for cortical_id, cortical_idx in self.__id_to_idx.items():
            if cortical_idx not in self.__idx_to_id:
                errors.add(
                    f"ID→IDX mapping {cortical_id}→{cortical_idx} missing reverse mapping"
                )
            elif self.__idx_to_id[cortical_idx] != cortical_id:
                errors.add(
                    f"ID→IDX mapping {cortical_id}→{cortical_idx} conflicts with reverse"
                )

        # Check idx_to_id → id_to_idx consistency
        for cortical_idx, cortical_id in self.__idx_to_id.items():
            if cortical_id not in self.__id_to_idx:
                errors.add(
                    f"IDX→ID mapping {cortical_idx}→{cortical_id} missing reverse mapping"
                )
            elif self.__id_to_idx[cortical_id] != cortical_idx:
                errors.add(
                    f"IDX→ID mapping {cortical_idx}→{cortical_id} conflicts with reverse"
                )

        # Verify core areas are intact (critical system requirement)
        if (
            self.__id_to_idx.get("_death") != 0
            or self.__idx_to_id.get(0) != "_death"
        ):
            errors.add("Core area _death not properly mapped to index 0")
        if (
            self.__id_to_idx.get("_power") != 1
            or self.__idx_to_id.get(1) != "_power"
        ):
            errors.add("Core area _power not properly mapped to index 1")

        return len(errors) == 0, errors

    def get_all_mappings(self) -> Dict[str, int]:
        """Get snapshot of current ID→IDX mappings.

        Thread-safe atomic snapshot.

        Returns:
            Copy of the current ID→IDX mapping dictionary
        """
        with self.__lock:
            return self.__id_to_idx.copy()

    def get_stats(self) -> Dict[str, int]:
        """Get mapping statistics.

        Lock-free statistics gathering for RTOS compatibility.

        Returns:
            Dictionary with mapping statistics
        """
        is_consistent, _ = self.validate_consistency()
        return {
            "total_mappings": len(self.__id_to_idx),
            "core_areas": 2,  # Always _death=0, _power=1
            "custom_areas": len(self.__id_to_idx) - 2,
            "consistency_valid": is_consistent,
        }

    def clear(self, preserve_core_areas: bool = True) -> None:
        """Clear all mappings with core area protection.

        Single solid execution path - core areas always preserved.

        Args:
            preserve_core_areas: Always True (core areas cannot be removed)
        """
        # Clear everything
        self.__id_to_idx.clear()
        self.__idx_to_id.clear()

        # Always restore core areas (critical system requirement)
        self.__id_to_idx["_death"] = 0
        self.__id_to_idx["_power"] = 1
        self.__idx_to_id[0] = "_death"
        self.__idx_to_id[1] = "_power"

    def __len__(self) -> int:
        """Return number of mappings."""
        return len(self.__id_to_idx)

    def __contains__(self, key) -> bool:
        """Check if cortical_id or cortical_idx exists in mapping."""
        if isinstance(key, str):
            return key in self.__id_to_idx
        elif isinstance(key, int):
            return key in self.__idx_to_id
        return False

    def __repr__(self) -> str:
        """String representation for debugging."""
        stats = self.get_stats()
        return f"BiDirectionalCorticalMap(mappings={stats['total_mappings']}, core={stats['core_areas']}, custom={stats['custom_areas']})"
