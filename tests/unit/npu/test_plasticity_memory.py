import numpy as np

from feagi.npu.fire_ledger import FireLedgerInterface
from feagi.npu.fire_queue import FiringNeuron
from feagi.npu.plasticity.memory import MemoryConfig, MemoryFormationManager


def test_memory_pattern_key_determinism():
    ledger = FireLedgerInterface(default_window_size=5)

    # area 3 fires single neuron over three steps
    for t, nid in enumerate([30, 30, 30], start=1):
        ledger.archive_timestep(t, {
            3: [FiringNeuron(neuron_id=nid, cortical_idx=3, membrane_potential=0.0, coordinates=(0,0,0), threshold=1.0)]
        })

    mem = MemoryFormationManager(ledger)
    cfg = MemoryConfig(lookback_steps=4, pattern_duration=3)
    key1 = mem.get_temporal_pattern_key(3, [], cfg)
    key2 = mem.get_temporal_pattern_key(3, [], cfg)

    assert key1 == key2
    assert isinstance(key1, tuple)
    assert len(key1) == 3


