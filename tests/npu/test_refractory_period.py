"""
Test refractory period functionality in Rust NPU.

This test validates that refractory_period behaves as an absolute burst-tick counter:
- refractory_period=1 means skip 1 burst between fires (pattern: 1_1_1_)
- refractory_period=5 means skip 5 bursts between fires (pattern: 1_____1_____1)
- refractory_period=10 means skip 10 bursts between fires
"""

import pytest
from feagi.npu.rust_npu_integration import RustNPUIntegration
from feagi.bdu.connectome_manager import ConnectomeManager


@pytest.fixture
def setup_npu_with_neuron():
    """Create NPU with a single neuron that receives constant input."""
    # Create connectome and NPU
    connectome = ConnectomeManager.instance(config_or_max_neurons=1000, max_synapses=1000)
    npu = RustNPUIntegration(connectome)
    
    # Create a test neuron
    neuron_id = connectome.add_neuron(
        cortical_id="test_area",
        coordinates=(0, 0, 0)
    )
    
    # Set neuron properties to ensure it will fire when above threshold
    connectome.update_neuron_property(neuron_id, "threshold", 10.0)
    connectome.update_neuron_property(neuron_id, "membrane_potential", 15.0)  # Above threshold
    connectome.update_neuron_property(neuron_id, "excitability", 1.0)  # Always fire
    connectome.update_neuron_property(neuron_id, "consecutive_fire_limit", 0)  # Unlimited
    
    yield npu, connectome, neuron_id
    
    # Cleanup
    del npu
    del connectome


def run_burst_cycles(npu, neuron_id, num_bursts, keep_potential_high=True):
    """
    Run N burst cycles and track when the neuron fires.
    
    Args:
        npu: RustNPUInterface instance
        neuron_id: ID of the neuron to monitor
        num_bursts: Number of bursts to simulate
        keep_potential_high: If True, reset potential to 15.0 after each burst
        
    Returns:
        List of burst numbers where the neuron fired
    """
    firing_pattern = []
    
    for burst_num in range(num_bursts):
        # Ensure neuron has high potential (simulates constant input)
        if keep_potential_high:
            npu._rust_npu.update_neuron_membrane_potential(neuron_id, 15.0)
        
        # Process burst
        result = npu.process_burst(power_neurons=[])
        
        # Check if our neuron fired
        if neuron_id in result['fired_neurons']:
            firing_pattern.append(burst_num)
    
    return firing_pattern


def validate_firing_pattern(firing_pattern, refractory_period, total_bursts):
    """
    Validate that the firing pattern matches expected refractory behavior.
    
    With refractory_period=N, neuron should:
    - Fire on burst X
    - Be blocked for N bursts
    - Fire again on burst X+N+1
    """
    if not firing_pattern:
        return False, "Neuron never fired"
    
    # Check intervals between fires
    for i in range(1, len(firing_pattern)):
        interval = firing_pattern[i] - firing_pattern[i-1]
        expected_interval = refractory_period + 1  # Fire, skip N, fire
        
        if interval != expected_interval:
            return False, f"Interval {interval} != expected {expected_interval} (between bursts {firing_pattern[i-1]} and {firing_pattern[i]})"
    
    return True, "Pattern correct"


@pytest.mark.parametrize("refractory_period", [1, 5, 10])
def test_refractory_period_absolute_timing(setup_npu_with_neuron, refractory_period):
    """
    Test that refractory_period is an absolute burst-tick counter.
    
    Test cases:
    - refractory_period=1: Fire, skip 1 burst, fire → pattern: 1_1_1_
    - refractory_period=5: Fire, skip 5 bursts, fire → pattern: 1_____1_____1
    - refractory_period=10: Fire, skip 10 bursts, fire → pattern: 1__________1
    """
    npu, connectome, neuron_id = setup_npu_with_neuron
    
    # Set refractory period
    connectome.update_neuron_property(neuron_id, "refractory_period", refractory_period)
    
    # Run enough bursts to see multiple firing cycles
    # Need at least 3 fire events to validate pattern consistency
    num_cycles = 3
    total_bursts = (refractory_period + 1) * num_cycles + 10  # Extra buffer
    
    # Run simulation
    firing_pattern = run_burst_cycles(npu, neuron_id, total_bursts)
    
    # Debug output
    print(f"\n{'='*60}")
    print(f"Refractory Period: {refractory_period}")
    print(f"Total Bursts: {total_bursts}")
    print(f"Firing Pattern: {firing_pattern}")
    print(f"Fire Count: {len(firing_pattern)}")
    
    # Validate pattern
    is_valid, message = validate_firing_pattern(firing_pattern, refractory_period, total_bursts)
    
    # Visual representation
    visual = ['_'] * total_bursts
    for burst in firing_pattern:
        visual[burst] = '1'
    print(f"Visual: {''.join(visual)}")
    print(f"Validation: {message}")
    print(f"{'='*60}\n")
    
    # Assertions
    assert len(firing_pattern) >= num_cycles, f"Expected at least {num_cycles} fires, got {len(firing_pattern)}"
    assert is_valid, f"Firing pattern validation failed: {message}"
    
    # Verify first interval (may be different if neuron starts in refractory)
    if len(firing_pattern) >= 2:
        interval = firing_pattern[1] - firing_pattern[0]
        expected_interval = refractory_period + 1
        assert interval == expected_interval, \
            f"First interval {interval} != expected {expected_interval}"


def test_refractory_period_zero_means_no_refractory(setup_npu_with_neuron):
    """
    Test that refractory_period=0 allows continuous firing every burst.
    """
    npu, connectome, neuron_id = setup_npu_with_neuron
    
    # Set refractory period to 0
    connectome.update_neuron_property(neuron_id, "refractory_period", 0)
    
    # Run 10 bursts
    total_bursts = 10
    firing_pattern = run_burst_cycles(npu, neuron_id, total_bursts)
    
    print(f"\nRefractory Period: 0")
    print(f"Firing Pattern: {firing_pattern}")
    print(f"Expected: {list(range(total_bursts))}")
    
    # Should fire every single burst
    assert firing_pattern == list(range(total_bursts)), \
        f"With refractory=0, neuron should fire every burst"


def test_refractory_period_update_takes_effect_immediately(setup_npu_with_neuron):
    """
    Test that updating refractory_period takes effect immediately.
    """
    npu, connectome, neuron_id = setup_npu_with_neuron
    
    # Start with refractory=1
    connectome.update_neuron_property(neuron_id, "refractory_period", 1)
    
    # Run 10 bursts with refractory=1
    pattern_phase1 = run_burst_cycles(npu, neuron_id, 10)
    
    # Change to refractory=5
    connectome.update_neuron_property(neuron_id, "refractory_period", 5)
    
    # Run 20 more bursts with refractory=5
    # Note: burst counter continues from previous phase
    pattern_phase2 = []
    for burst_num in range(10, 30):
        npu._rust_npu.update_neuron_membrane_potential(neuron_id, 15.0)
        result = npu.process_burst(power_neurons=[])
        if neuron_id in result['fired_neurons']:
            pattern_phase2.append(burst_num)
    
    print(f"\nPhase 1 (refractory=1): {pattern_phase1}")
    print(f"Phase 2 (refractory=5): {pattern_phase2}")
    
    # Validate phase 1: fires every 2 bursts
    if len(pattern_phase1) >= 2:
        interval1 = pattern_phase1[1] - pattern_phase1[0]
        assert interval1 == 2, f"Phase 1 interval {interval1} != 2"
    
    # Validate phase 2: fires every 6 bursts
    if len(pattern_phase2) >= 2:
        interval2 = pattern_phase2[1] - pattern_phase2[0]
        assert interval2 == 6, f"Phase 2 interval {interval2} != 6"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

