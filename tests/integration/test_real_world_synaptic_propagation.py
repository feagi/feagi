#!/usr/bin/env python3
"""
REAL-WORLD Integration Test for Synaptic Propagation

This test simulates real FEAGI conditions that would catch the critical bugs:
1. Large neuron IDs (like real systems)
2. No automatic NPU setup (like real systems)
3. Manual NPU configuration required
"""

import pytest
import numpy as np
from feagi.bdu.connectome_manager import ConnectomeManager


class TestRealWorldSynapticPropagation:
    """Test synaptic propagation under real-world conditions."""

    @pytest.fixture
    def real_world_setup(self):
        """Create a connectome that simulates real FEAGI conditions."""
        # Reset singleton
        ConnectomeManager.reset_singleton()
        
        config = {'max_neurons': 100000}  # Large capacity like real systems
        connectome = ConnectomeManager(config, max_synapses=1000000)
        
        # Create cortical areas
        area1_id = connectome.add_cortical_area(
            name="source_area", dimensions=(100, 100, 1), position=(0, 0, 0)
        )
        area2_id = connectome.add_cortical_area(
            name="target_area", dimensions=(100, 100, 1), position=(200, 0, 0)
        )
        
        # Create neurons with LARGE IDs (like real systems)
        # This would expose the neuron ID to array index bug
        source_neurons = []
        target_neurons = []
        
        # Start with large neuron IDs to simulate real conditions
        base_id = 50000
        for i in range(5):
            # Source neurons with large IDs
            source_id = connectome.create_neuron(
                cortical_id=area1_id,
                position=(i * 10, 0, 0),
                threshold=1.0
            )
            source_neurons.append(source_id)
            
            # Target neurons with large IDs  
            target_id = connectome.create_neuron(
                cortical_id=area2_id,
                position=(i * 10, 0, 0),
                threshold=0.5
            )
            target_neurons.append(target_id)
        
        # Create synapses
        synapses = []
        for i in range(5):
            success = connectome.create_synapse(
                pre_neuron_id=source_neurons[i],
                post_neuron_id=target_neurons[i],
                weight=2.0
            )
            synapses.append(success)
        
        return {
            'connectome': connectome,
            'source_neurons': source_neurons,
            'target_neurons': target_neurons,
            'synapses': synapses
        }

    def test_without_npu_should_fail(self, real_world_setup):
        """Test that should FAIL without NPU setup (like real systems)."""
        connectome = real_world_setup['connectome']
        source_neurons = real_world_setup['source_neurons']
        target_neurons = real_world_setup['target_neurons']
        
        print(f"Source neuron IDs: {source_neurons}")
        print(f"Target neuron IDs: {target_neurons}")
        
        # Set source neurons to fire
        for neuron_id in source_neurons:
            connectome.set_neuron_property(neuron_id, 'membrane_potential', 1.5)
        
        # This should FAIL with RuntimeError (no NPU configured)
        with pytest.raises(RuntimeError, match="NPU processor required"):
            connectome.update_membrane_potentials(current_timestep=1)

    def test_with_npu_large_ids(self, real_world_setup):
        """Test with NPU and large neuron IDs (would expose the bug)."""
        connectome = real_world_setup['connectome']
        source_neurons = real_world_setup['source_neurons']
        target_neurons = real_world_setup['target_neurons']
        
        print(f"Source neuron IDs: {source_neurons}")
        print(f"Target neuron IDs: {target_neurons}")
        
        # CRITICAL: Configure NPU (this is what real systems need)
        from feagi.npu.burst_engine_npu_integration import patch_burst_engine_for_npu
        from feagi.npu.burst_engine import BurstEngine
        
        # Patch BurstEngine for NPU
        patch_burst_engine_for_npu()
        
        # Create and configure BurstEngine with NPU
        burst_engine = BurstEngine(connectome_manager=connectome)
        burst_engine.initialize_npu_processor(
            max_neurons=100000,
            max_synapses=1000000,
            backend="cpu"
        )
        burst_engine.enable_npu_processing()
        
        # Verify NPU is configured
        assert hasattr(connectome, '_npu_processor')
        assert connectome._npu_processor is not None
        
        # Set source neurons to fire
        for neuron_id in source_neurons:
            connectome.set_neuron_property(neuron_id, 'membrane_potential', 1.5)
        
        # Get initial target potentials
        initial_potentials = {}
        for neuron_id in target_neurons:
            initial_potentials[neuron_id] = connectome.get_neuron_property(
                neuron_id, 'membrane_potential'
            )
        
        # Process neural burst
        fired_neurons = connectome.update_membrane_potentials(current_timestep=1)
        
        # CRITICAL ASSERTIONS that would catch the bug
        
        # 1. Source neurons should fire
        source_fired = [n for n in fired_neurons if n in source_neurons]
        assert len(source_fired) > 0, f"Source neurons {source_neurons} did not fire"
        
        # 2. Target neurons should receive synaptic input
        target_potentials_after = {}
        for neuron_id in target_neurons:
            target_potentials_after[neuron_id] = connectome.get_neuron_property(
                neuron_id, 'membrane_potential'
            )
        
        # Check that target neurons received input
        targets_with_input = []
        for neuron_id in target_neurons:
            if target_potentials_after[neuron_id] > initial_potentials[neuron_id]:
                targets_with_input.append(neuron_id)
        
        print(f"Targets with input: {targets_with_input}")
        print(f"Target potentials before: {list(initial_potentials.values())}")
        print(f"Target potentials after: {list(target_potentials_after.values())}")
        
        # CRITICAL: This assertion would FAIL with the neuron ID bug
        assert len(targets_with_input) > 0, (
            f"CRITICAL BUG: No target neurons received synaptic input! "
            f"This indicates neuron ID to array index mapping is broken. "
            f"Source neurons: {source_neurons}, Target neurons: {target_neurons}, "
            f"Fired neurons: {fired_neurons}"
        )
        
        # 3. Target neurons should fire in next cycle
        fired_neurons_2 = connectome.update_membrane_potentials(current_timestep=2)
        target_fired = [n for n in fired_neurons_2 if n in target_neurons]
        
        assert len(target_fired) > 0, (
            f"Target neurons did not fire despite receiving input. "
            f"Target potentials: {target_potentials_after}"
        )

    def test_neuron_id_to_index_mapping(self, real_world_setup):
        """Specific test for neuron ID to array index mapping bug."""
        connectome = real_world_setup['connectome']
        
        # Configure NPU
        from feagi.npu.burst_engine_npu_integration import patch_burst_engine_for_npu
        from feagi.npu.burst_engine import BurstEngine
        
        patch_burst_engine_for_npu()
        burst_engine = BurstEngine(connectome_manager=connectome)
        burst_engine.initialize_npu_processor(max_neurons=100000, max_synapses=1000000)
        burst_engine.enable_npu_processing()
        
        # Get a neuron with large ID
        large_neuron_id = real_world_setup['source_neurons'][0]
        print(f"Testing neuron ID: {large_neuron_id}")
        
        # Check NPU mapping
        npu = connectome._npu_processor
        if large_neuron_id in npu.neurons.neuron_id_to_index:
            array_index = npu.neurons.neuron_id_to_index[large_neuron_id]
            print(f"Neuron ID {large_neuron_id} maps to array index {array_index}")
            
            # CRITICAL: Array index should be small (0-4), not equal to neuron ID
            assert array_index < 100, (
                f"CRITICAL BUG: Array index {array_index} is too large! "
                f"Should be small index, not equal to neuron ID {large_neuron_id}"
            )
            
            # Verify we can access the array at this index
            assert array_index < len(npu.neurons.membrane_potentials), (
                f"Array index {array_index} is out of bounds for membrane_potentials array"
            )
        else:
            pytest.fail(f"Neuron ID {large_neuron_id} not found in NPU mapping")
