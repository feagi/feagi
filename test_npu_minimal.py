#!/usr/bin/env python3
"""
Minimal NPU test to verify basic functionality after architecture migration.
"""

import pytest
import numpy as np
from typing import List, Dict, Any, Tuple

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.npu.interface import NPUInterface
from feagi.npu.data_structures import NeuronArray, SynapseArray, BackendType


class TestNPUBasic:
    """Test basic NPU functionality with new architecture."""
    
    def test_npu_neuron_array_initialization(self):
        """Test NPU neuron array initialization."""
        npu_neurons = NeuronArray(max_neurons=1000, backend=BackendType.CPU)
        
        assert npu_neurons.max_neurons == 1000
        assert npu_neurons.backend == BackendType.CPU
        assert npu_neurons.neuron_count == 0
        assert len(npu_neurons.membrane_potentials) >= 1000
        assert len(npu_neurons.thresholds) >= 1000
        assert len(npu_neurons.valid_mask) >= 1000
        assert isinstance(npu_neurons.neuron_id_to_index, dict)
        assert isinstance(npu_neurons.index_to_neuron_id, dict)
    
    def test_synapse_array_initialization(self):
        """Test NPU synapse array initialization."""
        npu_synapses = SynapseArray(max_synapses=1000)
        
        assert npu_synapses.max_synapses == 1000
        assert npu_synapses.count == 0
        assert len(npu_synapses.source_neuron_ids) >= 1000
        assert len(npu_synapses.target_neuron_ids) >= 1000
        assert len(npu_synapses.weights) >= 1000
        assert isinstance(npu_synapses.source_neuron_index, dict)
        assert isinstance(npu_synapses.target_neuron_index, dict)
    
    def test_neuron_creation_with_genome_params(self):
        """Test neuron creation with required genome parameters."""
        npu = NPUInterface(backend=BackendType.CPU)
        
        # Create neurons with all required genome parameters
        created_indices = npu.neuron_array.add_neurons_batch(
            neuron_ids=[1, 2],
            positions=[(0, 0, 0), (1, 0, 0)],
            neuron_types=[0, 0],
            initial_potentials=[0.5, 0.8],
            thresholds=[1.0, 1.0],
            leak_coefficients=[0.95, 0.95],
            cortical_idx=0,
            # Required genome parameters
            decay_rates=[0.95, 0.95],
            refractory_periods=[1, 1],
            excitabilities=[1.0, 1.0],
            resting_potentials=[0.0, 0.0],
            consecutive_fire_limits=[10, 10]
        )
        
        assert len(created_indices) == 2
        assert npu.neuron_array.neuron_count == 2
        assert npu.neuron_array.membrane_potentials[0] == 0.5
        assert npu.neuron_array.membrane_potentials[1] == 0.8
        
    def test_synapse_creation(self):
        """Test synapse creation."""
        npu = NPUInterface(backend=BackendType.CPU)
        
        # Create neurons first
        npu.neuron_array.add_neurons_batch(
            neuron_ids=[1, 2],
            positions=[(0, 0, 0), (1, 0, 0)],
            neuron_types=[0, 0],
            initial_potentials=[0.0, 0.0],
            thresholds=[1.0, 1.0],
            leak_coefficients=[0.95, 0.95],
            cortical_idx=0,
            decay_rates=[0.95, 0.95],
            refractory_periods=[1, 1],
            excitabilities=[1.0, 1.0],
            resting_potentials=[0.0, 0.0],
            consecutive_fire_limits=[10, 10]
        )
        
        # Create synapse
        success = npu.synapse_array.create_synapse(
            source_neuron_id=1,
            target_neuron_id=2,
            weight=0.5
        )
        
        assert success == True
        assert npu.synapse_array.count == 1
        
        # Check synapse properties
        connections = npu.synapse_array.get_outgoing_connections(1)
        assert len(connections) == 1
        assert connections[0] == (2, 0.5)
        
        # Check incoming connections (our fix!)
        incoming = npu.synapse_array.get_incoming_connections(2)
        assert len(incoming) == 1
        assert incoming[0] == (1, 0.5)
    
    def test_connectome_manager_npu_integration(self):
        """Test that ConnectomeManager integrates with NPU correctly."""
        cm = ConnectomeManager.instance()
        
        # Should have NPU interface
        assert hasattr(cm, '_npu_interface')
        assert cm._npu_interface is not None
        
        # Should have neuron and synapse arrays
        npu_interface = cm._npu_interface
        assert hasattr(npu_interface, 'neuron_array')
        assert hasattr(npu_interface, 'synapse_array')
        assert npu_interface.neuron_array is not None
        assert npu_interface.synapse_array is not None


if __name__ == "__main__":
    # Run the tests
    test_class = TestNPUBasic()
    
    try:
        print("🧪 Testing NPU neuron array initialization...")
        test_class.test_npu_neuron_array_initialization()
        print("✅ PASSED")
        
        print("🧪 Testing NPU synapse array initialization...")
        test_class.test_synapse_array_initialization() 
        print("✅ PASSED")
        
        print("🧪 Testing neuron creation with genome parameters...")
        test_class.test_neuron_creation_with_genome_params()
        print("✅ PASSED")
        
        print("🧪 Testing synapse creation...")
        test_class.test_synapse_creation()
        print("✅ PASSED")
        
        print("🧪 Testing ConnectomeManager NPU integration...")
        test_class.test_connectome_manager_npu_integration()
        print("✅ PASSED")
        
        print("\n🎉 All minimal NPU tests PASSED!")
        print("✅ NPU architecture migration is working correctly")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
