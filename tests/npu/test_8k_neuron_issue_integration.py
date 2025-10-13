"""
Test to reproduce and diagnose 8k neuron firing issue with ConnectomeManager integration.
"""

import pytest
import numpy as np
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.npu.burst_engine import BurstEngine


class Test8kNeuronIssue:
    """Test to identify where 8k neuron firing breaks in full integration."""
    
    def setup_method(self):
        """Setup test environment."""
        BurstEngine.reset_instance()
        
    def teardown_method(self):
        """Cleanup."""
        BurstEngine.reset_instance()
    
    def test_connectome_manager_with_8k_neurons(self):
        """Test ConnectomeManager can handle 8000+ neurons."""
        cm = ConnectomeManager(config_or_max_neurons=10000)
        
        # Create power area
        power_area = cm.create_cortical_area(
            cortical_id='_power',
            coordinates_3d=[(0, 0, 0)] * 100,
            cortical_name='Power'
        )
        print(f"Power area created with {len(power_area.coordinates_3d)} neurons")
        
        # Create target area with 8000 neurons (80x100 grid)
        target_coords = [(x, y, 0) for x in range(80) for y in range(100)]
        target_area = cm.create_cortical_area(
            cortical_id='test_8k',
            coordinates_3d=target_coords,
            cortical_name='Test 8k Area'
        )
        print(f"Target area created with {len(target_area.coordinates_3d)} neurons")
        
        # Get neuron IDs
        power_neuron_ids = cm.get_neurons_by_cortical_area('_power')
        target_neuron_ids = cm.get_neurons_by_cortical_area('test_8k')
        
        print(f"Power neurons: {len(power_neuron_ids)}")
        print(f"Target neurons: {len(target_neuron_ids)}")
        
        assert len(target_neuron_ids) == 8000, f"Expected 8000 neurons, got {len(target_neuron_ids)}"
        
        # Create projector pattern synapses
        synapse_count = 0
        for power_neuron in power_neuron_ids[:10]:  # Use 10 power neurons
            for target_neuron in target_neuron_ids[synapse_count:synapse_count+800]:
                cm.create_synapse(
                    source_neuron_id=power_neuron,
                    target_neuron_id=target_neuron,
                    weight=1.2  # Above threshold
                )
                synapse_count += 1
                if synapse_count >= 8000:
                    break
            if synapse_count >= 8000:
                break
        
        print(f"Created {synapse_count} synapses")
        assert synapse_count == 8000, f"Expected 8000 synapses, got {synapse_count}"
        
        # Check NPU interface has neurons
        npu = cm._npu_interface
        print(f"NPU neuron count: {npu.neuron_array.neuron_count}")
        assert npu.neuron_array.neuron_count >= 8100, "NPU should have 8100+ neurons"
        
    def test_burst_engine_with_8k_neurons(self):
        """Test BurstEngine processing with 8000+ neurons."""
        cm = ConnectomeManager(config_or_max_neurons=10000)
        
        # Create simple network
        power_coords = [(i, 0, 0) for i in range(10)]
        target_coords = [(x, y, 0) for x in range(90) for y in range(90)]  # 8100 neurons
        
        power_area = cm.create_cortical_area(
            cortical_id='_power',
            coordinates_3d=power_coords,
            cortical_name='Power'
        )
        
        target_area = cm.create_cortical_area(
            cortical_id='test_area',
            coordinates_3d=target_coords,
            cortical_name='Test Area'
        )
        
        print(f"Created areas: power={len(power_coords)}, target={len(target_coords)}")
        
        # Get neuron IDs
        power_neurons = cm.get_neurons_by_cortical_area('_power')
        target_neurons = cm.get_neurons_by_cortical_area('test_area')
        
        # Create synapses - each power neuron -> 810 target neurons
        for i, power_id in enumerate(power_neurons):
            start_idx = i * 810
            end_idx = start_idx + 810
            for target_id in target_neurons[start_idx:end_idx]:
                cm.create_synapse(power_id, target_id, weight=1.5)
        
        print(f"Created synapses connecting all {len(target_neurons)} target neurons")
        
        # Initialize BurstEngine
        engine = BurstEngine(connectome_manager=cm)
        
        # Set power neurons to fire
        npu = cm._npu_interface
        for power_id in power_neurons:
            idx = npu.neuron_array.neuron_id_to_index[power_id]
            npu.neuron_array.membrane_potentials[idx] = 1.5
        
        # Process burst
        print("\nProcessing burst...")
        fired_neurons = engine._process_burst()
        
        print(f"Neurons fired: {len(fired_neurons)}")
        
        # Check how many target neurons received input
        target_with_input = 0
        target_above_threshold = 0
        for target_id in target_neurons:
            idx = npu.neuron_array.neuron_id_to_index[target_id]
            potential = npu.neuron_array.membrane_potentials[idx]
            if potential > 0.0:
                target_with_input += 1
            if potential >= 1.0:
                target_above_threshold += 1
        
        print(f"Target neurons with input: {target_with_input}/{len(target_neurons)}")
        print(f"Target neurons above threshold: {target_above_threshold}/{len(target_neurons)}")
        
        # Assertions
        assert target_with_input >= len(target_neurons) * 0.8, \
            f"Only {target_with_input}/{len(target_neurons)} neurons received input"
        
        assert target_above_threshold >= len(target_neurons) * 0.5, \
            f"Only {target_above_threshold}/{len(target_neurons)} neurons above threshold"
    
    def test_synaptic_propagation_at_8k_scale(self):
        """Test if synaptic propagation works correctly at 8k+ scale."""
        cm = ConnectomeManager(config_or_max_neurons=12000)
        
        # Create areas
        source_coords = [(i, 0, 0) for i in range(100)]
        target_coords = [(x, y, 0) for x in range(100) for y in range(100)]  # 10k neurons
        
        source_area = cm.create_cortical_area(
            cortical_id='source',
            coordinates_3d=source_coords,
            cortical_name='Source'
        )
        
        target_area = cm.create_cortical_area(
            cortical_id='target',
            coordinates_3d=target_coords,
            cortical_name='Target'
        )
        
        source_neurons = cm.get_neurons_by_cortical_area('source')
        target_neurons = cm.get_neurons_by_cortical_area('target')
        
        print(f"Testing propagation: {len(source_neurons)} -> {len(target_neurons)} neurons")
        
        # Create full connectivity
        synapse_count = 0
        for i, source_id in enumerate(source_neurons):
            start = i * 100
            end = start + 100
            for target_id in target_neurons[start:end]:
                cm.create_synapse(source_id, target_id, weight=0.5)
                synapse_count += 1
        
        print(f"Created {synapse_count} synapses")
        
        # Test propagation
        npu = cm._npu_interface
        
        # Set all source neurons to fire
        for source_id in source_neurons:
            idx = npu.neuron_array.neuron_id_to_index[source_id]
            npu.neuron_array.membrane_potentials[idx] = 1.5
        
        # Propagate
        npu.synapse_array.propagate_activations(
            firing_neuron_ids=source_neurons,
            neuron_array=npu.neuron_array
        )
        
        # Check results
        potentials = []
        for target_id in target_neurons:
            idx = npu.neuron_array.neuron_id_to_index[target_id]
            potential = npu.neuron_array.membrane_potentials[idx]
            potentials.append(potential)
        
        potentials = np.array(potentials)
        print(f"\nPropagation results:")
        print(f"  Min potential: {potentials.min():.4f}")
        print(f"  Max potential: {potentials.max():.4f}")
        print(f"  Mean potential: {potentials.mean():.4f}")
        print(f"  Neurons with input: {np.sum(potentials > 0)}/{len(target_neurons)}")
        
        # All neurons should have received input
        assert np.sum(potentials > 0) == len(target_neurons), \
            f"Not all neurons received input: {np.sum(potentials > 0)}/{len(target_neurons)}"


if __name__ == "__main__":
    test = Test8kNeuronIssue()
    test.setup_method()
    
    try:
        print("="*80)
        print("Test 1: ConnectomeManager with 8k neurons")
        print("="*80)
        test.test_connectome_manager_with_8k_neurons()
        
        print("\n" + "="*80)
        print("Test 2: BurstEngine with 8k neurons")
        print("="*80)
        test.test_burst_engine_with_8k_neurons()
        
        print("\n" + "="*80)
        print("Test 3: Synaptic propagation at 8k+ scale")
        print("="*80)
        test.test_synaptic_propagation_at_8k_scale()
        
        print("\n" + "="*80)
        print("ALL TESTS PASSED - No 8k limit found")
        print("="*80)
        
    finally:
        test.teardown_method()
