"""
Test power neuron injection workflow after genome load.

This test ensures that:
1. Power neurons are created during genome load
2. Power neurons have cortical_area=1 in Rust NPU
3. Power neurons are injected into FCL on every burst
"""

import pytest
import json
import os


@pytest.fixture
def feagi_system():
    """Initialize FEAGI system with clean state."""
    from feagi.config.toml_loader import load_feagi_config
    from feagi.core.state_manager import FeagiStateManager
    from feagi.bdu.connectome_manager import ConnectomeManager
    
    # Load config
    config = load_feagi_config()
    
    # Initialize state manager
    state_manager = FeagiStateManager.instance()
    
    # Create connectome manager
    connectome_manager = ConnectomeManager(config)
    
    yield {
        'config': config,
        'state_manager': state_manager,
        'connectome_manager': connectome_manager,
        'npu_interface': connectome_manager._npu_interface,
    }
    
    # Cleanup
    # Reset state manager if needed


@pytest.fixture
def minimal_genome():
    """Create a minimal genome with _power area."""
    return {
        "blueprint": {
            "_power": {
                "cortical_name": "Power",
                "block_boundaries": [1, 1, 1],
                "blueprint_location": {
                    "x": 0,
                    "y": 0,
                    "z": 0
                },
                "dev_count": 1,
                "per_voxel_neuron_cnt": 1,
                "neurons_per_voxel": 1,
                "cortical_neuron_per_vox_count": 1,
                "group_id": "core",
                "sub_group_id": None,
                "visualization": True,
                "firing_threshold": 1.0,
                "fire_t": 1.0,
                "leak_coefficient": 0.0,
                "leak_c": 0,
                "leak_variability": 0,
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
                "blueprint_location": {
                    "x": 1,
                    "y": 0,
                    "z": 0
                },
                "dev_count": 1,
                "per_voxel_neuron_cnt": 1,
                "neurons_per_voxel": 1,
                "cortical_neuron_per_vox_count": 1,
                "group_id": "core",
                "sub_group_id": None,
                "visualization": True,
                "firing_threshold": 1.0,
                "fire_t": 1.0,
                "leak_coefficient": 0.0,
                "leak_c": 0,
                "leak_variability": 0,
                "refractory_period": 1,
                "refrac": 1,
                "neuron_excitability": 1.0,
                "consecutive_fire_cnt_max": 10,
                "snooze_length": 0,
                "mp_charge_accumulation": True,
            }
        },
        "brain_regions": {},
        "neuron_morphologies": {},
        "physiology": {
            "burst_delay": 0.025,
            "max_age": 10000000,
            "evolution_burst_count": 50,
            "ipu_idle_threshold": 1000,
            "plasticity_queue_depth": 3,
            "lifespan_mgmt_interval": 10
        }
    }


def test_power_area_creation(feagi_system, minimal_genome):
    """Test that _power area is created with correct cortical_idx."""
    from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
    
    connectome_manager = feagi_system['connectome_manager']
    config = feagi_system['config']
    
    # Run embryogenesis
    embryo = NeuroEmbryogenesis(
        connectome_manager=connectome_manager,
        config=config
    )
    
    success = embryo.develop_brain_from_genome_data(minimal_genome)
    assert success, "Embryogenesis failed"
    
    # Check _power area exists
    power_area = None
    for area_id, area in connectome_manager.cortical_areas.items():
        if hasattr(area, 'cortical_id') and area.cortical_id == "_power":
            power_area = area
            break
    
    assert power_area is not None, "_power area not found"
    assert power_area.cortical_idx == 1, f"_power area should have cortical_idx=1, got {power_area.cortical_idx}"


def test_power_neurons_created(feagi_system, minimal_genome):
    """Test that power neurons are created during embryogenesis."""
    from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
    
    connectome_manager = feagi_system['connectome_manager']
    npu_interface = feagi_system['npu_interface']
    config = feagi_system['config']
    
    # Run embryogenesis
    embryo = NeuroEmbryogenesis(
        connectome_manager=connectome_manager,
        config=config
    )
    
    success = embryo.develop_brain_from_genome_data(minimal_genome)
    assert success, "Embryogenesis failed"
    
    # Find power area cortical_idx
    power_cortical_idx = None
    for area_id, area in connectome_manager.cortical_areas.items():
        if hasattr(area, 'cortical_id') and area.cortical_id == "_power":
            power_cortical_idx = area.cortical_idx
            break
    
    assert power_cortical_idx is not None, "_power area not found"
    
    # Check neurons were created
    power_neurons = npu_interface.get_neurons_by_area(power_cortical_idx)
    assert len(power_neurons) > 0, f"No neurons found in _power area (cortical_idx={power_cortical_idx})"
    
    print(f"✓ Found {len(power_neurons)} power neurons")


def test_power_neurons_have_correct_cortical_area(feagi_system, minimal_genome):
    """Test that power neurons have cortical_area=1 in Rust NPU."""
    from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
    
    connectome_manager = feagi_system['connectome_manager']
    npu_interface = feagi_system['npu_interface']
    config = feagi_system['config']
    
    # Run embryogenesis
    embryo = NeuroEmbryogenesis(
        connectome_manager=connectome_manager,
        config=config
    )
    
    success = embryo.develop_brain_from_genome_data(minimal_genome)
    assert success, "Embryogenesis failed"
    
    # Find power area cortical_idx
    power_cortical_idx = None
    for area_id, area in connectome_manager.cortical_areas.items():
        if hasattr(area, 'cortical_id') and area.cortical_id == "_power":
            power_cortical_idx = area.cortical_idx
            break
    
    # Get power neurons
    power_neurons = npu_interface.get_neurons_by_area(power_cortical_idx)
    assert len(power_neurons) > 0, "No power neurons found"
    
    # Verify mapping
    for neuron_id in power_neurons:
        mapped_cortical_idx = npu_interface.get_neuron_cortical_idx(neuron_id)
        assert mapped_cortical_idx == power_cortical_idx, \
            f"Neuron {neuron_id} has cortical_idx={mapped_cortical_idx}, expected {power_cortical_idx}"
    
    print(f"✓ All {len(power_neurons)} power neurons have correct cortical_area mapping")


def test_power_injection_in_burst(feagi_system, minimal_genome):
    """Test that power neurons are injected into FCL during burst."""
    from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
    
    connectome_manager = feagi_system['connectome_manager']
    npu_interface = feagi_system['npu_interface']
    config = feagi_system['config']
    
    # Run embryogenesis
    embryo = NeuroEmbryogenesis(
        connectome_manager=connectome_manager,
        config=config
    )
    
    success = embryo.develop_brain_from_genome_data(minimal_genome)
    assert success, "Embryogenesis failed"
    
    # Get power neurons count
    power_cortical_idx = None
    for area_id, area in connectome_manager.cortical_areas.items():
        if hasattr(area, 'cortical_id') and area.cortical_id == "_power":
            power_cortical_idx = area.cortical_idx
            break
    
    power_neurons = npu_interface.get_neurons_by_area(power_cortical_idx)
    expected_power_count = len(power_neurons)
    
    assert expected_power_count > 0, "No power neurons found"
    
    # Run a burst through Rust NPU
    rust_npu_integration = npu_interface._rust_npu_integration
    assert rust_npu_integration is not None, "Rust NPU integration not available"
    
    result = rust_npu_integration.process_burst()
    
    # Check power injections
    power_injections = result.get('power_injections', 0)
    
    assert power_injections == expected_power_count, \
        f"Expected {expected_power_count} power injections, got {power_injections}"
    
    print(f"✓ Power injection working: {power_injections} neurons injected")


def test_power_neurons_fire(feagi_system, minimal_genome):
    """Test that power neurons actually fire with sufficient power amount."""
    from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
    
    connectome_manager = feagi_system['connectome_manager']
    npu_interface = feagi_system['npu_interface']
    config = feagi_system['config']
    
    # Run embryogenesis
    embryo = NeuroEmbryogenesis(
        connectome_manager=connectome_manager,
        config=config
    )
    
    success = embryo.develop_brain_from_genome_data(minimal_genome)
    assert success, "Embryogenesis failed"
    
    # Get Rust NPU
    rust_npu_integration = npu_interface._rust_npu_integration
    rust_npu = rust_npu_integration._rust_npu
    
    # Set power amount high enough to cause firing (threshold is 1.0)
    rust_npu.set_power_amount(2.0)
    
    # Run a burst
    result = rust_npu_integration.process_burst()
    
    # Check that neurons fired
    fired_neurons = result.get('fired_neurons', [])
    power_injections = result.get('power_injections', 0)
    
    assert power_injections > 0, "No power injections occurred"
    assert len(fired_neurons) > 0, f"Power injected but no neurons fired (injections={power_injections})"
    
    print(f"✓ Power neurons firing: {len(fired_neurons)} neurons fired from {power_injections} injections")


def test_power_injection_persistence_across_bursts(feagi_system, minimal_genome):
    """Test that power injection works consistently across multiple bursts."""
    from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
    
    connectome_manager = feagi_system['connectome_manager']
    npu_interface = feagi_system['npu_interface']
    config = feagi_system['config']
    
    # Run embryogenesis
    embryo = NeuroEmbryogenesis(
        connectome_manager=connectome_manager,
        config=config
    )
    
    success = embryo.develop_brain_from_genome_data(minimal_genome)
    assert success, "Embryogenesis failed"
    
    # Get expected power neuron count
    power_cortical_idx = None
    for area_id, area in connectome_manager.cortical_areas.items():
        if hasattr(area, 'cortical_id') and area.cortical_id == "_power":
            power_cortical_idx = area.cortical_idx
            break
    
    power_neurons = npu_interface.get_neurons_by_area(power_cortical_idx)
    expected_power_count = len(power_neurons)
    
    # Run multiple bursts
    rust_npu_integration = npu_interface._rust_npu_integration
    
    for burst_num in range(5):
        result = rust_npu_integration.process_burst()
        power_injections = result.get('power_injections', 0)
        
        assert power_injections == expected_power_count, \
            f"Burst {burst_num}: Expected {expected_power_count} power injections, got {power_injections}"
    
    print(f"✓ Power injection consistent across 5 bursts")


if __name__ == "__main__":
    # Run tests with verbose output
    pytest.main([__file__, "-v", "-s"])

