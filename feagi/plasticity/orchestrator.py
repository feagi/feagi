"""
Plasticity orchestrator - bridges Fire Ledger with plasticity computations.

No stateful loops; pure orchestration for STDP and memory formation.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import numpy as np

from .stdp import STDPConfig, STDPComputer
from .memory import MemoryConfig, MemoryFormationManager


@dataclass
class PlasticityInputs:
    syn_source_ids: np.ndarray
    syn_target_ids: np.ndarray
    source_cortical_idx: int
    target_cortical_idx: int


class PlasticityOrchestrator:
    def __init__(self, fire_ledger):
        self.fire_ledger = fire_ledger
        self._stdp = STDPComputer(fire_ledger)
        self._memory = MemoryFormationManager(fire_ledger)

    def stdp_activity(
        self,
        inputs: PlasticityInputs,
        stdp_cfg: STDPConfig,
    ) -> Tuple[np.ndarray, np.ndarray]:
        return self._stdp.compute_activity_factors(
            syn_source_ids=inputs.syn_source_ids,
            syn_target_ids=inputs.syn_target_ids,
            source_cortical_idx=inputs.source_cortical_idx,
            target_cortical_idx=inputs.target_cortical_idx,
            config=stdp_cfg,
        )

    def configure_memory_area(self, memory_area_idx: int, window_size: int, upstream_idxs: List[int]) -> None:
        self._memory.configure_memory_area(memory_area_idx, window_size, upstream_idxs)

    def memory_pattern_key(self, area_idx: int, upstream_idxs: List[int], mem_cfg: MemoryConfig) -> Tuple[bytes, ...]:
        key = self._memory.get_temporal_pattern_key(area_idx, upstream_idxs, mem_cfg)
        return key if key is not None else tuple()

    def memory_activation(self, area_idx: int, mem_cfg: MemoryConfig) -> bool:
        return self._memory.should_create_or_reactivate(area_idx, mem_cfg)

    @staticmethod
    def group_synapses_by_area_pairs(
        syn_source_ids: np.ndarray,
        syn_target_ids: np.ndarray,
        neuron_to_area: dict,
    ) -> Dict[Tuple[int, int], np.ndarray]:
        """Group synapse indices by (source_area, target_area) deterministically.

        Returns mapping to index arrays (views) without copying large arrays.
        """
        count = int(len(syn_source_ids))
        groups: Dict[Tuple[int, int], List[int]] = {}
        # Deterministic iteration order by index
        for i in range(count):
            src = int(syn_source_ids[i])
            tgt = int(syn_target_ids[i])
            s_area = int(neuron_to_area.get(src, -1))
            t_area = int(neuron_to_area.get(tgt, -1))
            if s_area < 0 or t_area < 0:
                continue
            key = (s_area, t_area)
            if key not in groups:
                groups[key] = []
            groups[key].append(i)
        # Convert to numpy indices
        return {k: np.asarray(v, dtype=np.int32) for k, v in groups.items()}


