"""
Memory formation utilities backed by Fire Ledger.

Responsibilities:
- Register memory areas with custom windows
- Extract temporal patterns as compact keys
- Manage MemoryNeuronArray creation/reactivation signals (pure computation)

This module emits decisions; the caller applies them to arrays.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple
import numpy as np


@dataclass
class MemoryConfig:
    lookback_steps: int = 50
    pattern_duration: int = 10
    min_activation_count: int = 3


class MemoryFormationManager:
    def __init__(self, fire_ledger):
        self.fire_ledger = fire_ledger

    def configure_memory_area(self, memory_area_idx: int, window_size: int, upstream_idxs: List[int]) -> None:
        self.fire_ledger.configure_memory_area(memory_area_idx, window_size, upstream_idxs)

    def get_temporal_pattern_key(
        self,
        area_idx: int,
        upstream_idxs: List[int],
        config: MemoryConfig,
    ) -> Optional[Tuple[bytes, ...]]:
        """Build a compact temporal pattern key from recent history.

        Returns a tuple of serialized sets per timestep for determinism.
        """
        if config.pattern_duration <= 0:
            return None

        timesteps = min(config.pattern_duration, config.lookback_steps)
        parts: List[bytes] = []
        for offset in range(-timesteps, 0):
            bm = self.fire_ledger.get_timestep_pattern(area_idx, offset)
            # Serialize deterministically as sorted bytes
            arr = np.fromiter((int(x) for x in bm), dtype=np.int64)
            if arr.size:
                arr.sort()
                parts.append(arr.tobytes())
            else:
                parts.append(b"")
        return tuple(parts)

    def should_create_or_reactivate(
        self,
        area_idx: int,
        config: MemoryConfig,
    ) -> bool:
        """Heuristic: if union over lookback exceeds threshold, consider active."""
        union_bm = self.fire_ledger.get_firing_history(area_idx, config.lookback_steps)
        count = 0
        for _ in union_bm:
            count += 1
            if count >= config.min_activation_count:
                return True
        return False


