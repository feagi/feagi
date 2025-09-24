import numpy as np

from feagi.npu.fire_ledger import FireLedgerInterface
from feagi.npu.fire_queue import FiringNeuron
from feagi.npu.interface import NPUInterface
from feagi.npu.plasticity.service import PlasticityService, PlasticityConfig


def test_plasticity_queue_drop_and_apply_budget():
    ledger = FireLedgerInterface(default_window_size=3)
    npu = NPUInterface(max_neurons=100, max_synapses=10, max_memory_neurons=10)

    # Prepare a few plastic synapses
    npu.synapse_array.add_synapses_batch(
        source_neuron_ids=[1, 2, 3],
        target_neuron_ids=[4, 5, 6],
        weights=[0.1, 0.2, 0.3],
        delays=[1, 1, 1],
        conductances=[1.0, 1.0, 1.0],
        synapse_types=[3, 3, 3],  # plastic
        plasticity_types=[1, 1, 1],
        plasticity_coefficients=[0.01, 0.01, 0.01],
    )

    # Map neurons to areas
    npu.neuron_to_area = {1: 1, 2: 1, 3: 1, 4: 2, 5: 2, 6: 2}

    # Ledger spikes
    ledger.archive_timestep(1, {
        1: [FiringNeuron(neuron_id=1, cortical_idx=1, membrane_potential=0.0, coordinates=(0,0,0), threshold=1.0)],
        2: [FiringNeuron(neuron_id=4, cortical_idx=2, membrane_potential=0.0, coordinates=(0,0,0), threshold=1.0)],
    })

    cfg = PlasticityConfig(
        queue_capacity=1,  # allow only 1 enqueued command
        max_ops_per_burst=1,
        stdp={
            'lookback_steps': 2,
            'tau_pre': 20.0,
            'tau_post': 20.0,
            'a_plus': 0.01,
            'a_minus': 0.012,
        },
        memory={
            'lookback_steps': 2,
            'pattern_duration': 1,
            'min_activation_count': 1,
        }
    )

    svc = PlasticityService(ledger, npu, cfg)
    # Directly call internal producer to avoid threading in unit test
    svc._compute_enqueue_stdp()
    # The queue has 1 command; enqueuing memory will drop
    svc._compute_enqueue_memory()

    # Apply only 1 op per burst
    applied = npu.apply_plasticity_ops(max_ops=1)
    assert applied == 1


