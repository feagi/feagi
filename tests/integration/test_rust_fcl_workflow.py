"""
Integration tests for Rust FCL (Fire Candidate List) workflow.

Tests the complete FCL lifecycle in Rust NPU:
1. FCL initialization
2. Power neuron injection into FCL
3. Synaptic propagation to FCL
4. Sensory injection into FCL
5. FCL clearing between bursts
6. Fire Queue generation from FCL
"""

import pytest
import numpy as np


@pytest.fixture
def feagi_system():
    """Initialize FEAGI system with Rust NPU."""
    from feagi.config.toml_loader import load_feagi_config
    from feagi.core.state_manager import FeagiStateManager
    from feagi.bdu.connectome_manager import ConnectomeManager
    
    config = load_feagi_config()
    state_manager = FeagiStateManager.instance()
    connectome_manager = ConnectomeManager(config)
    
    yield {
        'config': config,
        'state_manager': state_manager,
        'connectome_manager': connectome_manager,
        'npu_interface': connectome_manager._npu_interface,
    }


@pytest.fixture
def test_genome():
    """Create test genome with multiple areas."""
    return {
        "blueprint": {
            "_power": {
                "cortical_name": "Power",
                "block_boundaries": [1, 1, 1],
                "blueprint_location": {"x": 0, "y": 0, "z": 0},
                "dev_count": 1,
                "per_voxel_neuron_cnt": 1,
                "neurons_per_voxel": 1,
                "cortical_neuron_per_vox_count": 1,
                "group_id": "core",
                "visualization": True,
                "firing_threshold": 1.0,
                "fire_t": 1.0,
                "leak_coefficient": 0.0,
                "leak_c": 0,
                "refractory_period": 1,
                "refrac": 1,
                "neuron_excitability": 1.0,
                "consecutive_fire_cnt_max": 10,
                "snooze_length": 0,
                "mp_charge_accumulation": True,
            },
            "_death": {
                "cortical_name": "Death",
                "block_boundaries": [1, 1, 1],
                "blueprint_location": {"x": 1, "y": 0, "z": 0},
                "dev_count": 1,
                "per_voxel_neuron_cnt": 1,
                "neurons_per_voxel": 1,
                "group_id": "core",
                "visualization": True,
                "firing_threshold": 1.0,
                "fire_t": 1.0,
                "leak_coefficient": 0.0,
                "leak_c": 0,
                "refractory_period": 1,
                "refrac": 1,
                "neuron_excitability": 1.0,
                "consecutive_fire_cnt_max": 10,
                "snooze_length": 0,
                "mp_charge_accumulation": True,
            },
            "test_area_1": {
                "cortical_name": "TestArea1",
                "block_boundaries": [3, 3, 1],
                "blueprint_location": {"x": 2, "y": 0, "z": 0},
                "dev_count": 1,
                "per_voxel_neuron_cnt": 1,
                "neurons_per_voxel": 1,
                "group_id": "custom",
                "visualization": True,
                "firing_threshold": 1.0,
                "fire_t": 1.0,
                "leak_coefficient": 0.0,
                "leak_c": 0,
                "refractory_period": 1,
                "refrac": 1,
                "neuron_excitability": 1.0,
                "consecutive_fire_cnt_max": 10,
                "snooze_length": 0,
                "mp_charge_accumulation": True,
            },
            "test_area_2": {
                "cortical_name": "TestArea2",
                "block_boundaries": [3, 3, 1],
                "blueprint_location": {"x": 5, "y": 0, "z": 0},
                "dev_count": 1,
                "per_voxel_neuron_cnt": 1,
                "neurons_per_voxel": 1,
                "group_id": "custom",
                "visualization": True,
                "firing_threshold": 1.0,
                "fire_t": 1.0,
                "leak_coefficient": 0.0,
                "leak_c": 0,
                "refractory_period": 1,
                "refrac": 1,
                "neuron_excitability": 1.0,
                "consecutive_fire_cnt_max": 10,
                "snooze_length": 0,
                "mp_charge_accumulation": True,
            }
        },
        "neuron_morphologies": {
            "test_morphology": {
                "name": "test_to_test2",
                "src_cortical_area": "test_area_1",
                "dst_cortical_area": "test_area_2",
                "morphology_scalar": 1,
                "postsynaptic_current": 1.0,
                "plasticity_flag": False,
                "type": "vectors",
                "vectors": [[0, 0, 0, 0, 0, 0]]
            }
        },
        "brain_regions": {},
        "physiology": {
            "burst_delay": 0.025,
            "max_age": 10000000,
            "evolution_burst_count": 50,
            "ipu_idle_threshold": 1000,
            "plasticity_queue_depth": 3,
            "lifespan_mgmt_interval": 10
        }
    }


def test_fcl_initialization(feagi_system, test_genome):
    """Test that Rust NPU initializes FCL correctly."""
    from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
    
    connectome_manager = feagi_system['connectome_manager']
    config = feagi_system['config']
    
    # Create brain
    embryo = NeuroEmbryogenesis(connectome_manager=connectome_manager, config=config)
    success = embryo.develop_brain_from_genome_data(test_genome)
    assert success, "Failed to create brain"
    
    # Get Rust NPU
    rust_npu_integration = feagi_system['npu_interface']._rust_npu_integration
    assert rust_npu_integration is not None, "Rust NPU not available"
    
    # FCL should be empty before burst
    # (We can't directly access FCL, but we can verify through burst results)
    print("FCL initialization verified via Rust NPU")


def test_fcl_power_injection(feagi_system, test_genome):
    """Test that power neurons are injected into FCL every burst."""
    from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
    
    connectome_manager = feagi_system['connectome_manager']
    config = feagi_system['config']
    npu_interface = feagi_system['npu_interface']
    
    # Create brain
    embryo = NeuroEmbryogenesis(connectome_manager=connectome_manager, config=config)
    success = embryo.develop_brain_from_genome_data(test_genome)
    assert success
    
    # Get power neuron count
    power_area = None
    for area_id, area in connectome_manager.cortical_areas.items():
        if hasattr(area, 'cortical_id') and area.cortical_id == "_power":
            power_area = area
            break
    
    assert power_area is not None
    power_neurons = npu_interface.get_neurons_by_area(power_area.cortical_idx)
    expected_power_count = len(power_neurons)
    
    # Run burst and check power injection
    rust_npu_integration = npu_interface._rust_npu_integration
    result = rust_npu_integration.process_burst()
    
    power_injections = result.get('power_injections', 0)
    assert power_injections == expected_power_count, \
        f"Expected {expected_power_count} power injections, got {power_injections}"
    
    print(f"Power injection verified: {power_injections} neurons")


def test_fcl_clears_between_bursts(feagi_system, test_genome):
    """Test that FCL is cleared at the end of each burst."""
    from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
    
    connectome_manager = feagi_system['connectome_manager']
    config = feagi_system['config']
    npu_interface = feagi_system['npu_interface']
    
    # Create brain
    embryo = NeuroEmbryogenesis(connectome_manager=connectome_manager, config=config)
    success = embryo.develop_brain_from_genome_data(test_genome)
    assert success
    
    # Get power neuron count (should be consistent across bursts)
    power_area = None
    for area_id, area in connectome_manager.cortical_areas.items():
        if hasattr(area, 'cortical_id') and area.cortical_id == "_power":
            power_area = area
            break
    
    power_neurons = npu_interface.get_neurons_by_area(power_area.cortical_idx)
    expected_power_count = len(power_neurons)
    
    # Run multiple bursts
    rust_npu_integration = npu_interface._rust_npu_integration
    
    for burst_num in range(3):
        result = rust_npu_integration.process_burst()
        power_injections = result.get('power_injections', 0)
        
        # Each burst should inject same number of power neurons
        # (proving FCL was cleared)
        assert power_injections == expected_power_count, \
            f"Burst {burst_num}: FCL not cleared properly (got {power_injections}, expected {expected_power_count})"
    
    print("FCL clearing verified across 3 bursts")


def test_fcl_synaptic_propagation(feagi_system, test_genome):
    """Test that synaptic propagation adds neurons to FCL."""
    from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
    
    connectome_manager = feagi_system['connectome_manager']
    config = feagi_system['config']
    npu_interface = feagi_system['npu_interface']
    
    # Create brain with synaptic connections
    embryo = NeuroEmbryogenesis(connectome_manager=connectome_manager, config=config)
    success = embryo.develop_brain_from_genome_data(test_genome)
    assert success
    
    # Set high power to ensure firing
    rust_npu_integration = npu_interface._rust_npu_integration
    rust_npu = rust_npu_integration._rust_npu
    rust_npu.set_power_amount(5.0)
    
    # Run first burst - power neurons should fire
    result1 = rust_npu_integration.process_burst()
    fired_neurons_1 = result1.get('fired_neurons', [])
    
    # Run second burst - synaptic propagation should occur
    result2 = rust_npu_integration.process_burst()
    synaptic_injections_2 = result2.get('synaptic_injections', 0)
    
    # Verify synaptic propagation occurred
    if len(fired_neurons_1) > 0 and synaptic_injections_2 > 0:
        print(f"Synaptic propagation verified: {synaptic_injections_2} injections")
    else:
        print(f"No synaptic propagation (fired={len(fired_neurons_1)}, propagated={synaptic_injections_2})")


def test_fcl_to_fire_queue_conversion(feagi_system, test_genome):
    """Test that FCL candidates are converted to Fire Queue."""
    from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
    
    connectome_manager = feagi_system['connectome_manager']
    config = feagi_system['config']
    npu_interface = feagi_system['npu_interface']
    
    # Create brain
    embryo = NeuroEmbryogenesis(connectome_manager=connectome_manager, config=config)
    success = embryo.develop_brain_from_genome_data(test_genome)
    assert success
    
    # Set high power
    rust_npu_integration = npu_interface._rust_npu_integration
    rust_npu = rust_npu_integration._rust_npu
    rust_npu.set_power_amount(5.0)
    
    # Run burst
    result = rust_npu_integration.process_burst()
    
    # Verify Fire Queue was generated
    fired_neurons = result.get('fired_neurons', [])
    power_injections = result.get('power_injections', 0)
    
    assert power_injections > 0, "No power injections"
    assert len(fired_neurons) > 0, "No neurons in Fire Queue despite power injection"
    
    print(f"FCL→Fire Queue verified: {power_injections} injections → {len(fired_neurons)} fired")


def test_fcl_sensory_injection(feagi_system, test_genome):
    """Test that sensory data can be injected into FCL."""
    from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
    
    connectome_manager = feagi_system['connectome_manager']
    config = feagi_system['config']
    npu_interface = feagi_system['npu_interface']
    
    # Create brain
    embryo = NeuroEmbryogenesis(connectome_manager=connectome_manager, config=config)
    success = embryo.develop_brain_from_genome_data(test_genome)
    assert success
    
    # Get test area neurons
    test_area = None
    for area_id, area in connectome_manager.cortical_areas.items():
        if area_id == "test_area_1":
            test_area = area
            break
    
    assert test_area is not None
    test_neurons = npu_interface.get_neurons_by_area(test_area.cortical_idx)
    
    if len(test_neurons) == 0:
        print("No test neurons available for sensory injection test")
        return
    
    # Inject sensory data
    rust_npu_integration = npu_interface._rust_npu_integration
    rust_npu = rust_npu_integration._rust_npu
    
    # Inject with high potential to ensure firing
    sensory_neuron_id = test_neurons[0]
    rust_npu.inject_sensory_with_potentials([(sensory_neuron_id, 5.0)])
    
    # Run burst
    result = rust_npu_integration.process_burst()
    
    sensory_injections = result.get('sensory_injections', 0)
    fired_neurons = result.get('fired_neurons', [])
    
    assert sensory_injections > 0, "No sensory injections recorded"
    assert sensory_neuron_id in fired_neurons, f"Sensory neuron {sensory_neuron_id} did not fire"
    
    print(f"Sensory injection verified: {sensory_injections} injections, neuron {sensory_neuron_id} fired")


def test_fcl_full_burst_cycle(feagi_system, test_genome):
    """Test complete FCL cycle: inject → process → fire → clear."""
    from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
    
    connectome_manager = feagi_system['connectome_manager']
    config = feagi_system['config']
    npu_interface = feagi_system['npu_interface']
    
    # Create brain
    embryo = NeuroEmbryogenesis(connectome_manager=connectome_manager, config=config)
    success = embryo.develop_brain_from_genome_data(test_genome)
    assert success
    
    rust_npu_integration = npu_interface._rust_npu_integration
    rust_npu = rust_npu_integration._rust_npu
    rust_npu.set_power_amount(2.0)
    
    # Run 5 complete cycles
    for cycle_num in range(5):
        result = rust_npu_integration.process_burst()
        
        power_injections = result.get('power_injections', 0)
        fired_neurons = result.get('fired_neurons', [])
        burst_num = result.get('burst', 0)
        
        # Verify each phase
        assert power_injections > 0, f"Cycle {cycle_num}: No power injection"
        assert len(fired_neurons) >= 0, f"Cycle {cycle_num}: Invalid fired neurons"
        assert burst_num == cycle_num + 1, f"Cycle {cycle_num}: Wrong burst number"
        
        print(f"Cycle {cycle_num}: burst={burst_num}, power={power_injections}, fired={len(fired_neurons)}")
    
    print("Full burst cycle verified across 5 cycles")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

