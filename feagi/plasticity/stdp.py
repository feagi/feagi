"""
STDP computations against Fire Ledger history.

Design goals:
- Deterministic, side-effect free computation
- Pure functions taking numpy arrays and FireLedgerInterface accessors
- RTOS-friendly: no dynamic sleeps, no unbounded loops
- Rust-ready: keep data in POD-like numpy arrays; easy to port
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import numpy as np


@dataclass
class STDPConfig:
    """Configuration for STDP computation.

    All values are simple scalars to keep FFI boundaries clean later.
    """
    lookback_steps: int = 20
    tau_pre: float = 20.0
    tau_post: float = 20.0
    a_plus: float = 0.01
    a_minus: float = 0.012
    max_pairs_per_synapse: int = 8  # deterministic cap


class STDPComputer:
    """Compute per-synapse pre/post correlation factors using Fire Ledger.

    This class does not own synapse arrays; it accepts neuron and synapse
    mappings and produces arrays that match synapse order for downstream
    PlasticityManager.update_plasticity.
    """

    def __init__(self, fire_ledger):
        # Depend on interface only; no imports to avoid cycles
        self.fire_ledger = fire_ledger

    def _bitmap_to_indices(self, bitmap) -> np.ndarray:
        # FireLedger RoaringBitmap placeholder supports iteration
        if bitmap is None:
            return np.array([], dtype=np.int32)
        return np.fromiter((int(x) for x in bitmap), dtype=np.int32)

    def compute_activity_factors(
        self,
        syn_source_ids: np.ndarray,
        syn_target_ids: np.ndarray,
        source_cortical_idx: int,
        target_cortical_idx: int,
        config: STDPConfig,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute activity factors for STDP given synapse endpoints.

        Returns: (pre_activity, post_activity), both float32 arrays aligned with synapses.
        """

        count = int(len(syn_source_ids))
        if count == 0:
            empty = np.zeros(0, dtype=np.float32)
            return empty, empty

        # Union over lookback for quick presence masks per area
        pre_union = self._bitmap_to_indices(
            self.fire_ledger.get_firing_history(source_cortical_idx, config.lookback_steps)
        )
        post_union = self._bitmap_to_indices(
            self.fire_ledger.get_firing_history(target_cortical_idx, config.lookback_steps)
        )

        # Fast set-membership using array hashing via numpy structured masks
        # Convert to boolean membership maps by hashing into fixed table
        # RTOS-friendly deterministic: fixed-size hash table
        table_size = max(1024, int(2 ** np.ceil(np.log2(max(1, pre_union.size + post_union.size)))))
        pre_table = np.zeros(table_size, dtype=np.int32)
        post_table = np.zeros(table_size, dtype=np.int32)

        if pre_union.size > 0:
            pre_table[np.mod(pre_union, table_size)] = 1
        if post_union.size > 0:
            post_table[np.mod(post_union, table_size)] = 1

        # Membership lookups
        pre_members = pre_table[np.mod(syn_source_ids, table_size)]
        post_members = post_table[np.mod(syn_target_ids, table_size)]

        # Simple Hebbian proxy: counts scaled by coefficients
        # For more accurate STDP timing, extend ledger with per-step bitmaps; for now
        # use union presence as proxy (deterministic, bounded work)
        pre_activity = (pre_members.astype(np.float32)) * 1.0
        post_activity = (post_members.astype(np.float32)) * 1.0

        return pre_activity, post_activity

    def compute_timing_factors(
        self,
        syn_source_ids: np.ndarray,
        syn_target_ids: np.ndarray,
        source_cortical_idx: int,
        target_cortical_idx: int,
        config: STDPConfig,
    ) -> np.ndarray:
        """Timing-based STDP factor per synapse using last-spike offsets.

        Positive values indicate potentiation; negative indicate depression.
        Deterministic: bounded by lookback and capped pairs per synapse.
        """
        count = int(len(syn_source_ids))
        if count == 0:
            return np.zeros(0, dtype=np.float32)

        # Build last-spike offset maps (closest to 0 == most recent)
        def last_offsets(cortical_idx: int, lookback: int) -> Dict[int, int]:
            result: Dict[int, int] = {}
            # Iterate from most recent (-1) backwards
            for step in range(1, lookback + 1):
                bm = self.fire_ledger.get_timestep_pattern(cortical_idx, -step)
                if bm is None:
                    continue
                for nid in bm:
                    # Store first time we encounter -> most recent
                    if nid not in result:
                        result[int(nid)] = step
            return result

        pre_last = last_offsets(source_cortical_idx, config.lookback_steps)
        post_last = last_offsets(target_cortical_idx, config.lookback_steps)

        # Vectorized over synapses using numpy where possible
        factors = np.zeros(count, dtype=np.float32)

        # Python loop but bounded and simple; ready to port to Rust
        for i in range(count):
            src = int(syn_source_ids[i])
            tgt = int(syn_target_ids[i])
            pre_o = pre_last.get(src)
            post_o = post_last.get(tgt)
            if pre_o is None or post_o is None:
                continue
            # delta>0 means pre earlier (smaller step means more recent)
            # Using offsets expressed as steps before current, so
            # if pre_o > post_o -> pre fired earlier in time than post
            # Convert to time difference in steps (post - pre)
            d = float(pre_o - post_o)
            if d > 0.0:
                # pre earlier than post -> potentiation
                factors[i] = config.a_plus * np.exp(-d / max(1e-6, config.tau_pre))
            elif d < 0.0:
                # post earlier than pre -> depression
                dd = -d
                factors[i] = -config.a_minus * np.exp(-dd / max(1e-6, config.tau_post))
            else:
                # same step -> strong potentiation
                factors[i] = config.a_plus

        return factors


