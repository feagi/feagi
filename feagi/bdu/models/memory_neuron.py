# Copyright 2016-2025 Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""
Memory Neuron Array - Specialized SoA for memory neurons with temporal pattern tracking.

Memory neurons are created dynamically based on upstream firing patterns and have
special lifecycle properties including lifespan management and long-term memory conversion.
"""

import hashlib
import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# Constants for memory neuron management
INVALID_MEMORY_NEURON_IDX = 4294967295  # Max uint32 value for invalid indices
DEFAULT_INITIAL_LIFESPAN = 9
DEFAULT_LIFESPAN_GROWTH_RATE = 1.0
DEFAULT_LONGTERM_THRESHOLD = 100


@dataclass
class MemoryPatternKey:
    """Represents a temporal pattern key for memory neuron identification.

    Uses bitmap sequence approach for optimal performance.
    """

    pattern_data: Tuple[bytes, ...]  # Serialized bitmap sequence
    temporal_depth: int
    source_cortical_areas: Tuple[str, ...]

    def __hash__(self) -> int:
        return hash(
            (
                self.pattern_data,
                self.temporal_depth,
                self.source_cortical_areas,
            )
        )

    def __eq__(self, other) -> bool:
        if not isinstance(other, MemoryPatternKey):
            return False
        return (
            self.pattern_data == other.pattern_data
            and self.temporal_depth == other.temporal_depth
            and self.source_cortical_areas == other.source_cortical_areas
        )


class MemoryNeuronArray:
    """
    Structure of Arrays (SoA) for memory neurons - CPU optimized, separate from regular neurons.

    Memory neurons are created based on temporal firing patterns and have special lifecycle
    properties including aging, lifespan growth, and long-term memory conversion.

    Architecture:
    - No spatial coordinates (purely abstract pattern-based)
    - Pattern-based identification using RoaringBitmap sequences
    - Lifecycle management with aging and reactivation
    - CPU-optimized operations (separate from GPU neural processing)
    """

    def __init__(self, capacity: int):
        """Initialize memory neuron array with specified capacity.

        Args:
            capacity: Maximum number of memory neurons to support
        """
        self.capacity = capacity
        self.next_available_index = 0
        self.deleted_indices: Set[int] = set()  # Reuse deleted neuron slots

        # Pattern identification: pattern_key -> neuron_index
        self.pattern_to_index: Dict[MemoryPatternKey, int] = {}
        self.index_to_pattern: Dict[int, MemoryPatternKey] = {}

        # Rust-friendly digest mapping (primary for future migration): digest(bytes[32]) -> neuron_index
        # Digest is computed deterministically from (pattern_data bytes, temporal_depth, source_cortical_areas)
        self.pattern_digest_to_index: Dict[bytes, int] = {}
        self.index_to_pattern_digest: Dict[int, bytes] = {}

        # Memory-specific lifecycle properties
        self.lifespan_current = np.zeros(
            capacity, dtype=np.uint32
        )  # Current remaining lifespan
        self.lifespan_initial = np.zeros(
            capacity, dtype=np.uint32
        )  # Initial lifespan value
        self.lifespan_growth_rate = np.zeros(
            capacity, dtype=np.float32
        )  # Growth rate on reactivation
        self.is_longterm_memory = np.zeros(
            capacity, dtype=np.bool_
        )  # Long-term memory flag

        # Lifecycle tracking
        self.creation_burst = np.zeros(
            capacity, dtype=np.uint64
        )  # Burst when neuron was created
        self.last_activation_burst = np.zeros(
            capacity, dtype=np.uint64
        )  # Last time pattern was seen
        self.activation_count = np.zeros(
            capacity, dtype=np.uint32
        )  # Total activations since creation

        # Memory area association
        self.cortical_area_id = [
            ""
        ] * capacity  # Which memory area this neuron belongs to

        # Validity tracking
        self.is_active = np.zeros(
            capacity, dtype=np.bool_
        )  # Whether neuron is active

        logger.info(f"MemoryNeuronArray initialized with capacity {capacity}")

    @staticmethod
    def _compute_pattern_digest(pattern_key: MemoryPatternKey) -> bytes:
        """Compute a stable 32-byte digest for a pattern key.

        The digest is computed over:
        - temporal_depth (u32 LE)
        - number of source areas (u32 LE) and each area id as UTF-8 bytes with length prefix
        - number of timesteps (u32 LE) and each timestep's serialized bitmap with length prefix

        Returns:
            32-byte digest (blake2b-256)
        """
        h = hashlib.blake2b(digest_size=32)
        # temporal_depth
        h.update(
            int(pattern_key.temporal_depth).to_bytes(4, "little", signed=False)
        )
        # source areas (stable order already stored)
        h.update(
            len(pattern_key.source_cortical_areas).to_bytes(
                4, "little", signed=False
            )
        )
        for area_id in pattern_key.source_cortical_areas:
            b = area_id.encode("utf-8")
            h.update(len(b).to_bytes(4, "little", signed=False))
            h.update(b)
        # pattern_data bitmaps
        h.update(
            len(pattern_key.pattern_data).to_bytes(4, "little", signed=False)
        )
        for blob in pattern_key.pattern_data:
            h.update(len(blob).to_bytes(4, "little", signed=False))
            h.update(blob)
        return h.digest()

    def create_memory_neuron(
        self,
        pattern_key: MemoryPatternKey,
        cortical_area_id: str,
        current_burst: int,
        initial_lifespan: int = DEFAULT_INITIAL_LIFESPAN,
        lifespan_growth_rate: float = DEFAULT_LIFESPAN_GROWTH_RATE,
    ) -> Optional[int]:
        """Create a new memory neuron for the given pattern.

        Args:
            pattern_key: Temporal pattern this neuron represents
            cortical_area_id: Which memory cortical area this neuron belongs to
            current_burst: Current burst number
            initial_lifespan: Initial lifespan in burst cycles
            lifespan_growth_rate: Growth rate for lifespan on reactivation

        Returns:
            Neuron index if successful, None if capacity exceeded or pattern exists
        """
        # Prefer digest-based lookup (Rust-friendly); keep legacy mapping for compatibility
        digest = self._compute_pattern_digest(pattern_key)
        existing_idx = self.pattern_digest_to_index.get(digest)
        if existing_idx is None and pattern_key in self.pattern_to_index:
            existing_idx = self.pattern_to_index[pattern_key]

        if existing_idx is not None:
            if self.is_active[existing_idx]:
                # Reactivate existing neuron instead of creating new one
                self.reactivate_memory_neuron(existing_idx, current_burst)
                return existing_idx
            else:
                # Reuse inactive neuron slot
                neuron_idx = existing_idx
        else:
            # Get new neuron index
            neuron_idx = self._get_next_neuron_index()
            if neuron_idx is None:
                logger.warning(
                    f"Memory neuron capacity {self.capacity} exceeded for area {cortical_area_id}"
                )
                return None

        # Initialize neuron properties
        self.pattern_to_index[pattern_key] = neuron_idx
        self.index_to_pattern[neuron_idx] = pattern_key
        # Store digest mapping as primary path
        self.pattern_digest_to_index[digest] = neuron_idx
        self.index_to_pattern_digest[neuron_idx] = digest

        self.lifespan_current[neuron_idx] = initial_lifespan
        self.lifespan_initial[neuron_idx] = initial_lifespan
        self.lifespan_growth_rate[neuron_idx] = lifespan_growth_rate
        self.is_longterm_memory[neuron_idx] = False

        self.creation_burst[neuron_idx] = current_burst
        self.last_activation_burst[neuron_idx] = current_burst
        self.activation_count[neuron_idx] = 1

        self.cortical_area_id[neuron_idx] = cortical_area_id
        self.is_active[neuron_idx] = True

        logger.debug(
            f"Created memory neuron {neuron_idx} for pattern in area {cortical_area_id}"
        )
        return neuron_idx

    def reactivate_memory_neuron(
        self, neuron_idx: int, current_burst: int
    ) -> bool:
        """Reactivate an existing memory neuron and update its lifespan.

        Args:
            neuron_idx: Index of neuron to reactivate
            current_burst: Current burst number

        Returns:
            True if successful, False otherwise
        """
        if (
            not self._is_valid_index(neuron_idx)
            or not self.is_active[neuron_idx]
        ):
            return False

        # Update activation tracking
        self.last_activation_burst[neuron_idx] = current_burst
        self.activation_count[neuron_idx] += 1

        # Grow lifespan additively if not long-term memory
        if not self.is_longterm_memory[neuron_idx]:
            current_lifespan = int(self.lifespan_current[neuron_idx])
            # Treat growth_rate as additive increment per activation
            increment = int(self.lifespan_growth_rate[neuron_idx])
            # Saturating addition within uint32 range
            max_u32 = np.iinfo(np.uint32).max
            new_lifespan = current_lifespan + increment
            if new_lifespan > max_u32:
                new_lifespan = max_u32
            self.lifespan_current[neuron_idx] = np.uint32(new_lifespan)

            logger.debug(
                f"Memory neuron {neuron_idx} reactivated: lifespan {current_lifespan} -> {new_lifespan}"
            )

        return True

    def find_memory_neuron_by_pattern(
        self, pattern_key: MemoryPatternKey
    ) -> Optional[int]:
        """Find memory neuron index for given pattern.

        Args:
            pattern_key: Pattern to search for

        Returns:
            Neuron index if found and active, None otherwise
        """
        # Try digest-based lookup first
        digest = self._compute_pattern_digest(pattern_key)
        neuron_idx = self.pattern_digest_to_index.get(digest)
        if neuron_idx is None:
            neuron_idx = self.pattern_to_index.get(pattern_key)
        if neuron_idx is not None and self.is_active[neuron_idx]:
            return neuron_idx
        return None

    def age_memory_neurons(self, current_burst: int) -> List[int]:
        """Age all memory neurons and return list of neurons that died.

        Args:
            current_burst: Current burst number

        Returns:
            List of neuron indices that died from aging
        """
        n = self.next_available_index
        if n == 0:
            return []
        active = self.is_active[:n]
        not_longterm = ~self.is_longterm_memory[:n]
        eligible = active & not_longterm
        if not np.any(eligible):
            return []
        lifespans = self.lifespan_current[:n]
        # Decrement lifespans for eligible neurons where lifespan > 0
        positive = eligible & (lifespans > 0)
        lifespans[positive] -= 1
        # Determine deaths (eligible that reached 0)
        died_mask = eligible & (lifespans == 0)
        died_indices = np.flatnonzero(died_mask).astype(int).tolist()
        if died_indices:
            self.is_active[died_indices] = False
            # Update deleted_indices set in bulk
            for idx in died_indices:
                self.deleted_indices.add(int(idx))
            logger.debug(
                f"Memory neurons died from aging at burst {current_burst}: {died_indices}"
            )
        return died_indices

    def age_by_bursts(self, delta_bursts: int) -> List[int]:
        """Age all eligible memory neurons by an arbitrary number of bursts.

        This method is vectorized for efficiency. It safely subtracts the given
        delta from current lifespans of active, non-long-term neurons, clamps at
        zero to avoid underflow, and deactivates neurons whose lifespan reaches
        zero. Returns the list of neuron indices that died.

        Args:
            delta_bursts: Number of bursts to age by (must be >= 0)

        Returns:
            List of indices for neurons that died due to aging
        """
        if delta_bursts is None:
            return []
        try:
            delta = int(delta_bursts)
        except Exception:
            return []
        if delta <= 0:
            return []

        n = self.next_available_index
        if n == 0:
            return []

        # Determine eligibility mask once
        active_mask = self.is_active[:n]
        not_ltm_mask = ~self.is_longterm_memory[:n]
        eligible_mask = active_mask & not_ltm_mask
        if not np.any(eligible_mask):
            return []

        # Work on eligible indices to avoid dtype underflow
        eligible_indices = np.flatnonzero(eligible_mask)
        current_life = self.lifespan_current[eligible_indices].astype(
            np.int64, copy=False
        )
        # Subtract and clamp to [0, max_u32]
        current_life -= delta
        died_local_mask = current_life <= 0
        current_life = np.clip(current_life, 0, np.iinfo(np.uint32).max)
        # Write back
        self.lifespan_current[eligible_indices] = current_life.astype(
            np.uint32, copy=False
        )

        # Determine global indices that died
        if not np.any(died_local_mask):
            return []
        died_indices = eligible_indices[died_local_mask].astype(int).tolist()
        # Deactivate and mark for reuse
        self.is_active[died_indices] = False
        for idx in died_indices:
            self.deleted_indices.add(int(idx))
        logger.debug(
            f"Memory neurons died from aging by Δ={delta}: {died_indices[:10]}{'...' if len(died_indices) > 10 else ''}"
        )
        return died_indices

    def check_longterm_conversion(
        self, longterm_threshold: int = DEFAULT_LONGTERM_THRESHOLD
    ) -> List[int]:
        """Check for memory neurons that should convert to long-term memory.

        Args:
            longterm_threshold: Lifespan threshold for long-term conversion

        Returns:
            List of neuron indices that converted to long-term memory
        """
        n = self.next_available_index
        if n == 0:
            return []
        active = self.is_active[:n]
        not_longterm = ~self.is_longterm_memory[:n]
        eligible = active & not_longterm
        if not np.any(eligible):
            return []
        meets_threshold = eligible & (
            self.lifespan_current[:n] >= longterm_threshold
        )
        converted_indices = (
            np.flatnonzero(meets_threshold).astype(int).tolist()
        )
        if converted_indices:
            self.is_longterm_memory[converted_indices] = True
            # Optional detailed logging remains minimal to avoid overhead
            logger.debug(
                f"Converted to long-term (threshold={longterm_threshold}): {converted_indices}"
            )
        return converted_indices

    def get_active_neurons_for_area(self, cortical_area_id: str) -> List[int]:
        """Get all active memory neuron indices for a specific cortical area.

        Args:
            cortical_area_id: Cortical area ID to search for

        Returns:
            List of active neuron indices in the area
        """
        active_neurons = []
        for neuron_idx in range(self.next_available_index):
            if (
                self.is_active[neuron_idx]
                and self.cortical_area_id[neuron_idx] == cortical_area_id
            ):
                active_neurons.append(neuron_idx)
        return active_neurons

    def get_memory_neuron_info(
        self, neuron_idx: int
    ) -> Optional[Dict[str, Any]]:
        """Get detailed information about a memory neuron.

        Args:
            neuron_idx: Neuron index to query

        Returns:
            Dictionary with neuron information or None if invalid
        """
        if (
            not self._is_valid_index(neuron_idx)
            or not self.is_active[neuron_idx]
        ):
            return None

        pattern_key = self.index_to_pattern.get(neuron_idx)
        return {
            "neuron_idx": neuron_idx,
            "pattern_key": pattern_key,
            "cortical_area_id": self.cortical_area_id[neuron_idx],
            "lifespan_current": int(self.lifespan_current[neuron_idx]),
            "lifespan_initial": int(self.lifespan_initial[neuron_idx]),
            "lifespan_growth_rate": float(
                self.lifespan_growth_rate[neuron_idx]
            ),
            "is_longterm_memory": bool(self.is_longterm_memory[neuron_idx]),
            "creation_burst": int(self.creation_burst[neuron_idx]),
            "last_activation_burst": int(
                self.last_activation_burst[neuron_idx]
            ),
            "activation_count": int(self.activation_count[neuron_idx]),
            "is_active": bool(self.is_active[neuron_idx]),
        }

    def get_statistics(self) -> Dict[str, Any]:
        """Get statistics about the memory neuron array.

        Returns:
            Dictionary with array statistics
        """
        active_mask = self.is_active[: self.next_available_index]
        longterm_mask = (
            self.is_longterm_memory[: self.next_available_index] & active_mask
        )

        return {
            "capacity": self.capacity,
            "total_neurons_created": self.next_available_index,
            "active_neurons": int(np.sum(active_mask)),
            "total_active_neurons": int(np.sum(active_mask)),
            "longterm_memory_neurons": int(np.sum(longterm_mask)),
            "deleted_indices_available": len(self.deleted_indices),
            "memory_usage_bytes": self._calculate_memory_usage(),
            "average_lifespan_active": (
                float(
                    np.mean(
                        self.lifespan_current[: self.next_available_index][
                            active_mask
                        ]
                    )
                )
                if np.any(active_mask)
                else 0.0
            ),
            "total_activations": int(
                np.sum(
                    self.activation_count[: self.next_available_index][
                        active_mask
                    ]
                )
            ),
        }

    def _get_next_neuron_index(self) -> Optional[int]:
        """Get next available neuron index, reusing deleted indices if
        available."""
        # Reuse deleted index if available
        if self.deleted_indices:
            return self.deleted_indices.pop()

        # Check capacity
        if self.next_available_index >= self.capacity:
            return None

        # Use next sequential index
        idx = self.next_available_index
        self.next_available_index += 1
        return idx

    def _deactivate_memory_neuron(self, neuron_idx: int) -> None:
        """Deactivate a memory neuron but keep it for potential
        reactivation."""
        if self._is_valid_index(neuron_idx):
            self.is_active[neuron_idx] = False
            self.deleted_indices.add(neuron_idx)
            # Keep pattern mapping for potential reactivation
            # Don't remove from pattern_to_index or index_to_pattern

    def _is_valid_index(self, neuron_idx: int) -> bool:
        """Check if neuron index is valid."""
        return 0 <= neuron_idx < self.next_available_index

    def collect_garbage(
        self, current_burst: int, prune_inactive_after_bursts: Optional[int]
    ) -> int:
        """Remove pattern mappings for neurons that are inactive and stale.

        Args:
            current_burst: Current burst number
            prune_inactive_after_bursts: If provided, only prune entries whose
                last_activation_burst is older than this threshold; if None, prune all inactive.

        Returns:
            Number of mapping entries pruned.
        """
        n = self.next_available_index
        if n == 0:
            return 0
        inactive = ~self.is_active[:n]
        if prune_inactive_after_bursts is not None:
            age = current_burst - self.last_activation_burst[:n]
            stale = inactive & (age >= prune_inactive_after_bursts)
        else:
            stale = inactive
        if not np.any(stale):
            return 0
        indices = np.flatnonzero(stale).astype(int).tolist()
        pruned = 0
        for idx in indices:
            # Remove legacy pattern mapping
            pattern = self.index_to_pattern.pop(idx, None)
            if pattern is not None:
                self.pattern_to_index.pop(pattern, None)
            # Remove digest mapping
            digest = self.index_to_pattern_digest.pop(idx, None)
            if digest is not None:
                self.pattern_digest_to_index.pop(digest, None)
            pruned += 1
        logger.info(
            f"[MEMORY-GC] Pruned {pruned} stale pattern mappings (inactive) at burst {current_burst}"
        )
        return pruned

    def _calculate_memory_usage(self) -> int:
        """Calculate total memory usage in bytes."""
        # NumPy arrays
        array_memory = (
            self.lifespan_current.nbytes
            + self.lifespan_initial.nbytes
            + self.lifespan_growth_rate.nbytes
            + self.is_longterm_memory.nbytes
            + self.creation_burst.nbytes
            + self.last_activation_burst.nbytes
            + self.activation_count.nbytes
            + self.is_active.nbytes
        )

        # Python objects (rough estimate)
        pattern_memory = (
            len(self.pattern_to_index) * 200
        )  # Rough estimate for pattern keys
        string_memory = (
            len(self.cortical_area_id) * 50
        )  # Rough estimate for strings

        return array_memory + pattern_memory + string_memory
