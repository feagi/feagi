"""
Integration test for refractory period using Fire Ledger.

This test validates refractory period behavior by:
1. Starting FEAGI with a test genome
2. Stimulating a neuron continuously
3. Reading Fire Ledger history via API
4. Validating firing pattern matches expected refractory behavior
"""

import pytest
import time
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.npu.burst_engine import BurstEngine
from feagi.npu.fire_ledger import FireLedgerInterface
from feagi.core.state_manager import FeagiStateManager


@pytest.fixture
def setup_burst_engine_with_test_neuron():
    """Set up a minimal burst engine with a test neuron."""
    # Get state manager and connectome
    state_manager = FeagiStateManager.instance()
    connectome = ConnectomeManager.instance()
    
    # Create a test cortical area
    test_area_id = "test_refrac"
    test_cortical_idx = connectome.create_cortical_area(
        cortical_id=test_area_id,
        coordinates=(0, 0, 0),
        dimensions=(1, 1, 1),  # Single neuron
        area_type="TEST"
    )
    
    # Get the neuron ID (first neuron in the area)
    neuron_ids = connectome.get_neurons_by_area(test_area_id)
    assert len(neuron_ids) > 0, "No neurons created in test area"
    test_neuron_id = neuron_ids[0]
    
    # Configure neuron to fire when stimulated
    connectome.update_neuron_property(test_neuron_id, "threshold", 5.0)
    connectome.update_neuron_property(test_neuron_id, "excitability", 1.0)
    connectome.update_neuron_property(test_neuron_id, "consecutive_fire_limit", 0)  # Unlimited
    connectome.update_neuron_property(test_neuron_id, "refractory_period", 0)  # Will be set in test
    
    # Create burst engine
    burst_engine = BurstEngine.instance()
    
    # Create Fire Ledger for the test area with sufficient history
    fire_ledger = FireLedgerInterface()
    fire_ledger.register_cortical_area(test_cortical_idx, window_size=50)
    burst_engine.fire_ledger = fire_ledger
    
    yield {
        'burst_engine': burst_engine,
        'connectome': connectome,
        'fire_ledger': fire_ledger,
        'test_area_id': test_area_id,
        'test_cortical_idx': test_cortical_idx,
        'test_neuron_id': test_neuron_id,
    }
    
    # Cleanup
    connectome.delete_cortical_area(test_area_id)


def stimulate_and_process_bursts(setup, num_bursts, keep_stimulated=True):
    """
    Stimulate neuron and process bursts, returning Fire Ledger history.
    
    Args:
        setup: Test setup dict
        num_bursts: Number of bursts to process
        keep_stimulated: If True, continuously stimulate the neuron
        
    Returns:
        List of burst numbers where the test neuron fired
    """
    burst_engine = setup['burst_engine']
    connectome = setup['connectome']
    test_neuron_id = setup['test_neuron_id']
    
    for _ in range(num_bursts):
        # Stimulate neuron to above threshold
        if keep_stimulated:
            connectome.update_neuron_property(test_neuron_id, "membrane_potential", 10.0)
        
        # Process burst
        burst_engine.process_burst()
    
    # Get firing history from Fire Ledger
    fire_ledger = setup['fire_ledger']
    test_cortical_idx = setup['test_cortical_idx']
    
    if test_cortical_idx not in fire_ledger.cortical_histories:
        return []
    
    cortical_history = fire_ledger.cortical_histories[test_cortical_idx]
    firing_pattern = []
    
    # Scan history backwards (most recent first)
    current_timestep = cortical_history.current_timestep
    for i in range(len(cortical_history.firing_history)):
        timestep = current_timestep - i
        bitmap = cortical_history.firing_history[-(i + 1)]
        
        # Check if our test neuron fired at this timestep
        if test_neuron_id in bitmap:
            firing_pattern.append(timestep)
    
    # Reverse to get chronological order
    firing_pattern.reverse()
    
    return firing_pattern


def validate_refractory_pattern(firing_pattern, refractory_period):
    """
    Validate that firing pattern matches expected refractory behavior.
    
    Returns:
        (is_valid, message, intervals)
    """
    if len(firing_pattern) < 2:
        return False, f"Not enough fires to validate (got {len(firing_pattern)}, need at least 2)", []
    
    intervals = []
    for i in range(1, len(firing_pattern)):
        interval = firing_pattern[i] - firing_pattern[i-1]
        intervals.append(interval)
    
    expected_interval = refractory_period + 1
    
    # Check if all intervals match expected
    incorrect_intervals = [(i, intervals[i]) for i in range(len(intervals)) 
                          if intervals[i] != expected_interval]
    
    if incorrect_intervals:
        return False, f"Incorrect intervals: {incorrect_intervals}, expected {expected_interval}", intervals
    
    return True, "All intervals correct", intervals


@pytest.mark.parametrize("refractory_period", [1, 5, 10])
def test_refractory_period_fire_ledger_integration(setup_burst_engine_with_test_neuron, refractory_period):
    """
    Integration test: Validate refractory period using Fire Ledger history.
    
    Test strategy:
    1. Set refractory_period to test value (1, 5, or 10)
    2. Continuously stimulate neuron for 40 bursts
    3. Read Fire Ledger history
    4. Validate firing pattern matches expected intervals
    """
    setup = setup_burst_engine_with_test_neuron
    connectome = setup['connectome']
    test_neuron_id = setup['test_neuron_id']
    
    # Set refractory period
    connectome.update_neuron_property(test_neuron_id, "refractory_period", refractory_period)
    
    # Process bursts with continuous stimulation
    num_bursts = 40
    firing_pattern = stimulate_and_process_bursts(setup, num_bursts, keep_stimulated=True)
    
    # Validate pattern
    is_valid, message, intervals = validate_refractory_pattern(firing_pattern, refractory_period)
    
    # Create visual representation
    visual = ['_'] * num_bursts
    for burst in firing_pattern:
        if 0 <= burst < num_bursts:
            visual[burst] = '1'
    visual_str = ''.join(visual)
    
    # Print detailed results
    print(f"\n{'='*70}")
    print(f"Refractory Period: {refractory_period}")
    print(f"Total Bursts: {num_bursts}")
    print(f"Firing Pattern: {firing_pattern}")
    print(f"Fire Count: {len(firing_pattern)}")
    print(f"Intervals: {intervals}")
    print(f"Expected Interval: {refractory_period + 1}")
    print(f"Visual: {visual_str}")
    print(f"Validation: {message}")
    print(f"{'='*70}\n")
    
    # Assertions
    assert len(firing_pattern) >= 3, \
        f"Expected at least 3 fires in {num_bursts} bursts, got {len(firing_pattern)}"
    
    assert is_valid, f"Firing pattern validation failed: {message}"
    
    # Verify consistency of all intervals
    if intervals:
        expected_interval = refractory_period + 1
        for i, interval in enumerate(intervals):
            assert interval == expected_interval, \
                f"Interval {i}: {interval} != expected {expected_interval}"


def test_refractory_period_zero_continuous_firing(setup_burst_engine_with_test_neuron):
    """
    Test that refractory_period=0 allows firing every single burst.
    """
    setup = setup_burst_engine_with_test_neuron
    connectome = setup['connectome']
    test_neuron_id = setup['test_neuron_id']
    
    # Set refractory period to 0
    connectome.update_neuron_property(test_neuron_id, "refractory_period", 0)
    
    # Process bursts
    num_bursts = 20
    firing_pattern = stimulate_and_process_bursts(setup, num_bursts, keep_stimulated=True)
    
    print(f"\nRefractory Period: 0")
    print(f"Firing Pattern: {firing_pattern}")
    print(f"Expected: Fire every burst")
    print(f"Fire Count: {len(firing_pattern)} / {num_bursts}")
    
    # Should fire every burst (or close to it, accounting for initialization)
    assert len(firing_pattern) >= num_bursts - 2, \
        f"With refractory=0, expected ~{num_bursts} fires, got {len(firing_pattern)}"


def test_refractory_period_update_takes_effect(setup_burst_engine_with_test_neuron):
    """
    Test that updating refractory_period mid-simulation takes effect immediately.
    """
    setup = setup_burst_engine_with_test_neuron
    connectome = setup['connectome']
    test_neuron_id = setup['test_neuron_id']
    fire_ledger = setup['fire_ledger']
    test_cortical_idx = setup['test_cortical_idx']
    
    # Phase 1: refractory=1 for 10 bursts
    connectome.update_neuron_property(test_neuron_id, "refractory_period", 1)
    phase1_pattern = stimulate_and_process_bursts(setup, 10, keep_stimulated=True)
    
    # Phase 2: Change to refractory=5 for 30 more bursts
    connectome.update_neuron_property(test_neuron_id, "refractory_period", 5)
    phase2_start = fire_ledger.cortical_histories[test_cortical_idx].current_timestep
    
    # Process more bursts
    for _ in range(30):
        connectome.update_neuron_property(test_neuron_id, "membrane_potential", 10.0)
        setup['burst_engine'].process_burst()
    
    # Get full firing pattern
    cortical_history = fire_ledger.cortical_histories[test_cortical_idx]
    full_pattern = []
    current_timestep = cortical_history.current_timestep
    
    for i in range(len(cortical_history.firing_history)):
        timestep = current_timestep - i
        bitmap = cortical_history.firing_history[-(i + 1)]
        if test_neuron_id in bitmap:
            full_pattern.append(timestep)
    
    full_pattern.reverse()
    
    # Split pattern into phases
    phase2_pattern = [t for t in full_pattern if t >= phase2_start]
    
    print(f"\nPhase 1 (refractory=1): {phase1_pattern}")
    print(f"Phase 2 (refractory=5): {phase2_pattern}")
    
    # Validate phase 1: interval should be 2
    if len(phase1_pattern) >= 2:
        phase1_intervals = [phase1_pattern[i+1] - phase1_pattern[i] 
                           for i in range(len(phase1_pattern)-1)]
        print(f"Phase 1 intervals: {phase1_intervals}")
        # Most intervals should be 2 (allowing for transition)
        assert sum(1 for i in phase1_intervals if i == 2) >= len(phase1_intervals) - 1, \
            f"Phase 1 intervals not consistent with refractory=1"
    
    # Validate phase 2: interval should be 6
    if len(phase2_pattern) >= 2:
        phase2_intervals = [phase2_pattern[i+1] - phase2_pattern[i] 
                           for i in range(len(phase2_pattern)-1)]
        print(f"Phase 2 intervals: {phase2_intervals}")
        # Most intervals should be 6 (allowing for transition)
        assert sum(1 for i in phase2_intervals if i == 6) >= len(phase2_intervals) - 1, \
            f"Phase 2 intervals not consistent with refractory=5"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

