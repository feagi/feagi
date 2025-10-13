import numpy as np

from feagi.npu.fire_ledger import FireLedgerInterface
from feagi.npu.fire_queue import FiringNeuron
from feagi.npu.interface import NPUInterface
from feagi.npu.plasticity.service import PlasticityService, PlasticityConfig


def test_end_to_end_plasticity_flow():
    ledger = FireLedgerInterface(default_window_size=3)
    npu = NPUInterface(max_neurons=100, max_synapses=8, max_memory_neurons=8)

    # Plastic synapses 1->2
    npu.synapse_array.add_synapses_batch(
        source_neuron_ids=[1],
        target_neuron_ids=[2],
        weights=[0.1],
        delays=[1],
        conductances=[1.0],
        synapse_types=[3],
        plasticity_types=[1],
        plasticity_coefficients=[0.01],
    )
    npu.neuron_to_area = {1: 10, 2: 11}

    cfg = PlasticityConfig(
        queue_capacity=16,
        max_ops_per_burst=8,
        stdp={'lookback_steps': 2, 'tau_pre': 20.0, 'tau_post': 20.0, 'a_plus': 0.01, 'a_minus': 0.012},
        memory={'lookback_steps': 2, 'pattern_duration': 1, 'min_activation_count': 1},
    )

    svc = PlasticityService(ledger, npu, cfg)

    # Simulate a burst with spikes
    ledger.archive_timestep(1, {
        10: [FiringNeuron(neuron_id=1, cortical_idx=10, membrane_potential=0.0, coordinates=(0,0,0), threshold=1.0)],
        11: [FiringNeuron(neuron_id=2, cortical_idx=11, membrane_potential=0.0, coordinates=(0,0,0), threshold=1.0)],
    })

    svc._compute_enqueue_stdp()

    before = float(npu.synapse_array.weights[0])
    applied = npu.apply_plasticity_ops(max_ops=8)
    after = float(npu.synapse_array.weights[0])
    assert applied >= 1
    assert after != before


