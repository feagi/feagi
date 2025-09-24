import numpy as np

from feagi.npu.fire_ledger import FireLedgerInterface
from feagi.npu.fire_queue import FiringNeuron
from feagi.npu.plasticity.stdp import STDPConfig, STDPComputer


def test_stdp_activity_and_timing_basic():
    ledger = FireLedgerInterface(default_window_size=5)

    # Two areas: 1 (pre), 2 (post)
    # t-1: pre fires {10}
    # t-2: post fires {20}
    neurons_by_area = {
        1: [FiringNeuron(neuron_id=10, cortical_idx=1, membrane_potential=0.0, coordinates=(0,0,0), threshold=1.0)],
        2: []
    }
    ledger.archive_timestep(1, neurons_by_area)

    neurons_by_area = {
        1: [],
        2: [FiringNeuron(neuron_id=20, cortical_idx=2, membrane_potential=0.0, coordinates=(0,0,0), threshold=1.0)]
    }
    ledger.archive_timestep(2, neurons_by_area)

    stdp = STDPComputer(ledger)
    cfg = STDPConfig(lookback_steps=3)

    src = np.array([10], dtype=np.int32)
    tgt = np.array([20], dtype=np.int32)

    pre_act, post_act = stdp.compute_activity_factors(src, tgt, 1, 2, cfg)
    assert pre_act.shape == (1,)
    assert post_act.shape == (1,)
    assert pre_act[0] >= 1.0
    assert post_act[0] >= 1.0

    timing = stdp.compute_timing_factors(src, tgt, 1, 2, cfg)
    assert timing.shape == (1,)
    # With pre at t-1 and post at t-2, d = (1)-(2) < 0 => depression
    assert timing[0] <= 0.0


