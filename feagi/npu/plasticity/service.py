"""
PlasticityService - independent thread that computes plasticity every burst.

RTOS-friendly design:
- No sleeps/timeouts; uses a condition variable signaled by BurstEngine
- Read-only access to FireLedger; compute-only here
- Mutations are enqueued to NPUInterface as bounded commands (drop-on-full)
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass
import threading
import numpy as np

from .orchestrator import PlasticityOrchestrator
from .stdp import STDPConfig
from .memory import MemoryConfig


@dataclass
class PlasticityConfig:
    queue_capacity: int
    max_ops_per_burst: int
    stdp: Dict[str, Any]
    memory: Dict[str, Any]


class PlasticityService:
    def __init__(
        self,
        fire_ledger,
        npu_interface,
        plasticity_config: PlasticityConfig,
        state_manager=None,
    ) -> None:
        self._ledger = fire_ledger
        self._npu = npu_interface
        self._orchestrator = PlasticityOrchestrator(fire_ledger)
        self._cfg = plasticity_config
        self._state_manager = state_manager

        self._cv = threading.Condition()
        self._latest_timestep = -1
        self._running = False

        # Configure NPU bounded queue
        self._npu.configure_plasticity_queue(self._cfg.queue_capacity)

        # Prebuild configs (strict; if missing, computations will be skipped)
        self._stdp_cfg = STDPConfig(
            lookback_steps=int(self._cfg.stdp["lookback_steps"]),
            tau_pre=float(self._cfg.stdp["tau_pre"]),
            tau_post=float(self._cfg.stdp["tau_post"]),
            a_plus=float(self._cfg.stdp["a_plus"]),
            a_minus=float(self._cfg.stdp["a_minus"]),
            max_pairs_per_synapse=8,
        ) if self._cfg.stdp else None

        self._mem_cfg = MemoryConfig(
            lookback_steps=int(self._cfg.memory["lookback_steps"]),
            pattern_duration=int(self._cfg.memory["pattern_duration"]),
            min_activation_count=int(self._cfg.memory["min_activation_count"]),
        ) if self._cfg.memory else None

    # ==== BurstEngine hook ====
    def notify_burst(self, timestep: int) -> None:
        with self._cv:
            self._latest_timestep = int(timestep)
            self._cv.notify()

    def start(self) -> threading.Thread:
        self._running = True
        t = threading.Thread(target=self._run, daemon=True)
        t.start()
        return t

    def stop(self) -> None:
        self._running = False
        with self._cv:
            self._cv.notify_all()

    def _run(self) -> None:
        while self._running:
            with self._cv:
                # Wait for a burst notification
                self._cv.wait()
                ts = self._latest_timestep

            if not self._running:
                break

            # Compute and enqueue STDP commands every burst
            try:
                self._compute_enqueue_stdp()
            except Exception:
                pass

            # Compute and enqueue memory commands every burst
            try:
                self._compute_enqueue_memory()
            except Exception:
                pass

    def _compute_enqueue_stdp(self) -> None:
        if self._stdp_cfg is None:
            return
        syn = getattr(self._npu, 'synapse_array', None)
        if syn is None or getattr(syn, 'count', 0) <= 0:
            return

        count = int(syn.count)
        # Only plastic synapses
        mask = syn.is_plastic_flags[:count]
        if not np.any(mask):
            return
        idxs = np.nonzero(mask)[0].astype(np.int32)
        src = syn.source_neuron_ids[idxs].astype(np.int32, copy=False)
        tgt = syn.target_neuron_ids[idxs].astype(np.int32, copy=False)

        # Group by (area_src, area_tgt)
        groups = PlasticityOrchestrator.group_synapses_by_area_pairs(src, tgt, getattr(self._npu, 'neuron_to_area', {}))
        if not groups:
            return

        # Accumulate deltas deterministically by group key order
        all_indices: List[int] = []
        all_deltas: List[float] = []

        for key in sorted(groups.keys()):
            s_area, t_area = key
            gidx = groups[key]
            gsrc = src[gidx]
            gtgt = tgt[gidx]
            timing = self._orchestrator.stdp_activity(
                inputs=type('PI', (), {
                    'syn_source_ids': gsrc,
                    'syn_target_ids': gtgt,
                    'source_cortical_idx': s_area,
                    'target_cortical_idx': t_area,
                })(),
                stdp_cfg=self._stdp_cfg,
            )[0]  # use activity factors (or switch to timing_factors if desired)

            coeffs = syn.plasticity_coeffs[idxs[gidx]].astype(np.float32, copy=False)
            deltas = (timing.astype(np.float32, copy=False)) * coeffs

            all_indices.extend(idxs[gidx].tolist())
            all_deltas.extend(deltas.tolist())

        if not all_indices:
            return

        # Single command (counts as 1 op in queue) with coalesced deltas
        cmd = {
            'type': 'update_weights_delta',
            'indices': np.asarray(all_indices, dtype=np.int32),
            'deltas': np.asarray(all_deltas, dtype=np.float32),
        }
        self._npu.enqueue_plasticity_commands([cmd])

    def _compute_enqueue_memory(self) -> None:
        if self._mem_cfg is None:
            return
        # Iterate memory areas, if any are configured
        areas = getattr(self._ledger, 'get_active_areas', None)
        if areas is None:
            return
        for area_idx in areas():
            # Simple heuristic: if active enough, emit a create command with a pattern key
            if not self._orchestrator.memory_activation(area_idx, self._mem_cfg):
                continue
            key = self._orchestrator.memory_pattern_key(area_idx, [], self._mem_cfg)
            if not key:
                continue
            cmd = {
                'type': 'create_memory_neurons',
                'area_idx': int(area_idx),
                'pattern_key': key,
                'count': 1,
            }
            self._npu.enqueue_plasticity_commands([cmd])


