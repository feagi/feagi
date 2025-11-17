"""
CRITICAL TEST: Cross-Cortical Area Synaptic Propagation Validation

This test specifically validates the major synaptic propagation problem where:
- Power area neurons are connected to another cortical area
- Power area neurons fire successfully 
- But neurons in the destination area are NOT firing

This test should FAIL if the synaptic propagation issue exists, catching the critical bug.
"""

import pytest
import numpy as np

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.npu.burst_engine_npu_integration import configure_npu_burst_engine
from feagi.api.core.services.core_api_service import CoreAPIService


class TestCrossCorticalSynapticPropagation:
    """Test synaptic propagation between cortical areas."""

    @pytest.fixture
    def connectome_setup(self):
        """Create a minimal connectome with power area and target area."""
        # Reset singleton
        ConnectomeManager.reset_singleton()
        
        config = {'max_neurons': 1000}
        connectome = ConnectomeManager(config, max_synapses=10000)
        
        # Create power area (cortical_idx=1, like _power)
        power_area_id = connectome.add_cortical_area(
            name="test_power",
            dimensions=(10, 10, 1),
            position=(0, 0, 0)
        )
        
        # Create target area (regular cortical area)
        target_area_id = connectome.add_cortical_area(
            name="test_target", 
            dimensions=(10, 10, 1),
            position=(20, 0, 0)  # Different position
        )
        
        # Add neurons to power area
        power_neurons = []
        for i in range(5):  # 5 power neurons
            neuron_id = connectome.create_neuron(
                cortical_id=power_area_id,
                position=(i, 0, 0),
                threshold=1.0
            )
            power_neurons.append(neuron_id)
        
        # Add neurons to target area  
        target_neurons = []
        for i in range(5):  # 5 target neurons
            neuron_id = connectome.create_neuron(
                cortical_id=target_area_id,
                position=(i, 0, 0), 
                threshold=0.5  # Lower threshold to ensure firing
            )
            target_neurons.append(neuron_id)
        
        # Create synapses from power area to target area
        # Each power neuron connects to corresponding target neuron
        synapses_created = []
        for i in range(5):
            success = connectome.create_synapse(
                pre_neuron_id=power_neurons[i],
                post_neuron_id=target_neurons[i],
                weight=2.0  # Strong weight to ensure downstream firing
            )
            synapses_created.append(success)
        
        return {
            'connectome': connectome,
            'power_neurons': power_neurons,
            'target_neurons': target_neurons,
            'power_area_id': power_area_id,
            'target_area_id': target_area_id,
            'synapses': synapses_created
        }

    def test_power_area_to_target_area_propagation(self, connectome_setup):
        """
        CRITICAL TEST: Validate that power area neurons can fire target area neurons.
        
        This test should FAIL if the synaptic propagation bug exists.
        """
        connectome = connectome_setup['connectome']
        power_neurons = connectome_setup['power_neurons']
        target_neurons = connectome_setup['target_neurons']
        
        # Configure NPU for synaptic propagation
        core_config = {
            'npu': {
                'backend': 'cpu',
                'max_neurons': 1000,
                'max_synapses': 10000
            }
        }
        core_api = CoreAPIService(connectome, config=core_config)
        burst_engine = core_api.get_burst_engine()
        
        # Verify NPU is configured
        assert burst_engine.npu_processor is not None, "NPU processor not configured"
        
        # Set power neurons to fire (membrane potential above threshold)
        for neuron_id in power_neurons:
            print(f"Setting neuron {neuron_id} membrane potential to 1.5")
            connectome.set_neuron_property(
                neuron_id, 
                'membrane_potential', 
                1.5  # Above threshold of 1.0
            )
            
            # Verify the update worked in NPU
            if burst_engine.npu_processor and neuron_id in burst_engine.npu_processor.neurons.neuron_id_to_index:
                idx = burst_engine.npu_processor.neurons.neuron_id_to_index[neuron_id]
                npu_potential = burst_engine.npu_processor.neurons.membrane_potentials[idx]
                print(f"NPU neuron {neuron_id} (idx {idx}) potential: {npu_potential}")
            else:
                print(f"Neuron {neuron_id} not found in NPU")
        
        # Get initial state of target neurons
        initial_target_potentials = {}
        for neuron_id in target_neurons:
            potential = connectome.get_neuron_property(neuron_id, 'membrane_potential')
            initial_target_potentials[neuron_id] = potential
            print(f"Target neuron {neuron_id} initial potential: {potential}")
        
        # Process one burst cycle - this should fire power neurons and propagate to targets
        print("\n=== PROCESSING BURST CYCLE ===")
        
        # Debug NPU state before firing
        npu = burst_engine.npu_processor.neurons
        print(f"NPU Debug - Valid mask: {npu.valid_mask[:10]}")
        print(f"NPU Debug - Refractory counters: {npu.refractory_counters[:10]}")
        print(f"NPU Debug - Thresholds: {npu.thresholds[:10]}")
        print(f"NPU Debug - Membrane potentials: {npu.membrane_potentials[:10]}")
        
        fired_neurons = connectome.update_membrane_potentials(current_timestep=1)
        print(f"Neurons that fired: {fired_neurons}")
        
        # Debug NPU state after firing
        print(f"NPU Debug AFTER - Membrane potentials: {npu.membrane_potentials[:10]}")
        print(f"NPU Debug AFTER - Refractory counters: {npu.refractory_counters[:10]}")
        
        # CRITICAL ASSERTION 1: Power neurons should fire
        power_neurons_fired = [n for n in fired_neurons if n in power_neurons]
        assert len(power_neurons_fired) > 0, (
            f"CRITICAL FAILURE: Power neurons did not fire. "
            f"Expected some of {power_neurons} to fire, but fired neurons were: {fired_neurons}"
        )
        print(f"✅ Power neurons fired: {power_neurons_fired}")
        
        # Check target neuron potentials after propagation
        final_target_potentials = {}
        for neuron_id in target_neurons:
            potential = connectome.get_neuron_property(neuron_id, 'membrane_potential')
            final_target_potentials[neuron_id] = potential
            print(f"Target neuron {neuron_id} final potential: {potential}")
        
        # CRITICAL ASSERTION 2: Target neurons should have received synaptic input
        target_neurons_activated = []
        for neuron_id in target_neurons:
            initial = initial_target_potentials[neuron_id]
            final = final_target_potentials[neuron_id]
            if final > initial:
                target_neurons_activated.append(neuron_id)
        
        assert len(target_neurons_activated) > 0, (
            f"CRITICAL SYNAPTIC PROPAGATION FAILURE: No target neurons received synaptic input. "
            f"Power neurons {power_neurons_fired} fired, but target neurons {target_neurons} "
            f"did not receive any synaptic input. "
            f"Initial potentials: {initial_target_potentials}, "
            f"Final potentials: {final_target_potentials}. "
            f"This indicates the synaptic propagation from power area to target area is broken!"
        )
        print(f"✅ Target neurons received synaptic input: {target_neurons_activated}")
        
        # Process second burst cycle - target neurons should now fire
        print("\n=== PROCESSING SECOND BURST CYCLE ===")
        fired_neurons_2 = connectome.update_membrane_potentials(current_timestep=2)
        print(f"Neurons that fired in second cycle: {fired_neurons_2}")
        
        # CRITICAL ASSERTION 3: Target neurons should fire in second cycle
        target_neurons_fired = [n for n in fired_neurons_2 if n in target_neurons]
        assert len(target_neurons_fired) > 0, (
            f"CRITICAL DOWNSTREAM FIRING FAILURE: Target neurons received synaptic input "
            f"but did not fire in the next cycle. "
            f"Target neurons with input: {target_neurons_activated}, "
            f"Target neurons that fired: {target_neurons_fired}, "
            f"Final potentials: {final_target_potentials}. "
            f"This indicates target neurons are not properly firing despite receiving input!"
        )
        print(f"✅ Target neurons fired in second cycle: {target_neurons_fired}")
        
        print("\n✅ CROSS-CORTICAL SYNAPTIC PROPAGATION WORKING CORRECTLY!")
        # Test completed successfully - all assertions passed

    def test_power_area_injection_and_propagation(self, connectome_setup):
        """
        Test power area injection (like FCL injection) followed by propagation.
        
        This simulates the real-world scenario where power areas are injected
        into the FCL and should propagate to connected areas.
        """
        connectome = connectome_setup['connectome']
        power_neurons = connectome_setup['power_neurons']
        target_neurons = connectome_setup['target_neurons']
        
        # Configure NPU
        core_config = {
            'npu': {
                'backend': 'cpu', 
                'max_neurons': 1000,
                'max_synapses': 10000
            }
        }
        core_api = CoreAPIService(connectome, config=core_config)
        burst_engine = core_api.get_burst_engine()
        
        # Simulate power injection by adding power neurons to FCL
        fcl_manager = connectome.fcl_manager
        initial_fcl_count = len(fcl_manager.get_firing_neurons(offset=0))
        
        # Inject power neurons into FCL (simulate power area injection)
        for neuron_id in power_neurons:
            fcl_manager.add_neuron_to_fcl(neuron_id)
        
        post_injection_fcl_count = len(fcl_manager.get_firing_neurons(offset=0))
        assert post_injection_fcl_count > initial_fcl_count, (
            "Power neuron injection into FCL failed"
        )
        print(f"✅ Power neurons injected into FCL: {post_injection_fcl_count - initial_fcl_count}")
        
        # Process burst - should propagate from FCL to connected areas
        fired_neurons = connectome.update_membrane_potentials(current_timestep=1)
        print(f"Neurons fired after FCL processing: {fired_neurons}")
        
        # Check if target neurons received input
        target_potentials_increased = []
        for neuron_id in target_neurons:
            potential = connectome.get_neuron_property(neuron_id, 'membrane_potential')
            if potential > 0.0:  # Should have received synaptic input
                target_potentials_increased.append(neuron_id)
        
        assert len(target_potentials_increased) > 0, (
            f"CRITICAL FCL PROPAGATION FAILURE: Power neurons were injected into FCL "
            f"and fired ({fired_neurons}), but target neurons did not receive synaptic input. "
            f"This indicates FCL-based synaptic propagation is broken!"
        )
        print(f"✅ FCL propagation successful: {len(target_potentials_increased)} target neurons activated")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
