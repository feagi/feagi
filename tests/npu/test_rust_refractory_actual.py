"""
REAL Rust NPU tests for refractory period logic.

These tests actually call the Rust NPU implementation via Python bindings
to validate the refractory period, consecutive fire limit, and snooze period behavior.
"""

import pytest

try:
    import feagi_rust
except ImportError:
    pytest.skip("feagi_rust module not available", allow_module_level=True)


def create_test_npu_with_neuron(refractory_period, consecutive_fire_limit, snooze_period):
    """
    Create a Rust NPU with a single test neuron configured with specified parameters.
    
    Returns:
        (npu, neuron_id)
    """
    npu = feagi_rust.RustNPU(1000, 1000, 100)
    
    # Add a neuron with specific refractory parameters
    neuron_id = npu.add_neuron(
        threshold=0.5,
        leak_coefficient=0.0,
        resting_potential=0.0,
        neuron_type=0,
        refractory_period=refractory_period,
        excitability=1.0,  # Always fire when above threshold
        consecutive_fire_limit=consecutive_fire_limit,
        snooze_period=snooze_period,
        cortical_area=0,
        x=0, y=0, z=0,
    )
    
    return npu, neuron_id


def run_burst_with_power_injection(npu, neuron_id, num_bursts):
    """
    Run multiple bursts with constant power injection to keep neuron above threshold.
    
    Returns:
        List of burst numbers where the neuron fired.
    """
    firing_pattern = []
    
    for burst in range(num_bursts):
        # Inject power to this neuron (always above threshold)
        result = npu.process_burst(power_neurons=[neuron_id])
        
        # Check if neuron fired (BurstResult has fired_neurons attribute)
        fired_neurons = result.fired_neurons if hasattr(result, 'fired_neurons') else []
        if neuron_id in fired_neurons:
            firing_pattern.append(burst)
    
    return firing_pattern


def visualize_pattern(firing_pattern, num_bursts):
    """Create visual representation of firing pattern."""
    visual = ['_'] * num_bursts
    for burst in firing_pattern:
        if 0 <= burst < num_bursts:
            visual[burst] = '1'
    return ''.join(visual)


@pytest.mark.parametrize("refrac,cfc_limit,snooze,expected_pattern", [
    # Basic cases
    (1, 3, 2, "1_1_1___1_1_1___"),  # Normal: fire 3x, extended refrac
    (1, 3, 0, "1_1_1_1_1_1_"),      # No snooze: continuous cycling
    (5, 3, 2, "1_____1_____1_______1_____1_____1_______"),  # Longer intervals
    
    # Corner case: refractory_period=0
    (0, 3, 2, "111__111__"),         # Fire every burst, but snooze after 3
    (0, 3, 0, "111111"),             # Fire every burst, no snooze
    (0, 0, 0, "111111111"),          # Fire every single burst (unlimited)
    
    # Corner case: consecutive_fire_limit=0 (unlimited)
    (1, 0, 2, "1_1_1_1_1_"),        # Unlimited fires, snooze ignored
    (5, 0, 10, "1_____1_____"),     # Unlimited fires, snooze ignored
    
    # Corner case: snooze_period=0 (no extended refractory)
    (1, 3, 0, "1_1_1_1_1_1_"),      # Hit limit but no extra refractory
    (5, 2, 0, "1_____1_____1_____"),  # Hit limit after 2, but no snooze
])
def test_rust_refractory_patterns(refrac, cfc_limit, snooze, expected_pattern):
    """
    Test Rust NPU refractory logic with various parameter combinations.
    
    This is a REAL test that calls actual Rust code via feagi_rust Python bindings.
    """
    num_bursts = len(expected_pattern)
    
    print(f"\n{'='*70}")
    print(f"Testing Rust NPU:")
    print(f"  refractory_period={refrac}, consecutive_fire_limit={cfc_limit}, snooze_period={snooze}")
    print(f"  Expected: {expected_pattern}")
    
    # Create NPU with test neuron
    npu, neuron_id = create_test_npu_with_neuron(refrac, cfc_limit, snooze)
    
    # Run bursts with power injection
    firing_pattern = run_burst_with_power_injection(npu, neuron_id, num_bursts)
    
    # Visualize
    actual_pattern = visualize_pattern(firing_pattern, num_bursts)
    
    print(f"  Actual:   {actual_pattern}")
    print(f"  Firing at bursts: {firing_pattern}")
    print(f"{'='*70}\n")
    
    # Assertion
    assert actual_pattern == expected_pattern, \
        f"Pattern mismatch!\nExpected: {expected_pattern}\nActual:   {actual_pattern}\n" \
        f"Rust NPU failed to implement correct refractory behavior!"


def test_rust_refractory_period_zero_fires_every_burst():
    """
    REAL Rust test: refractory_period=0, consecutive_fire_limit=0
    
    Should fire every single burst with no restrictions.
    """
    npu, neuron_id = create_test_npu_with_neuron(
        refractory_period=0,
        consecutive_fire_limit=0,
        snooze_period=0
    )
    
    firing_pattern = run_burst_with_power_injection(npu, neuron_id, 20)
    
    expected = list(range(20))
    print(f"\nRust NPU Test: refractory=0, limit=0, snooze=0")
    print(f"Expected: Fire every burst {expected}")
    print(f"Actual:   {firing_pattern}")
    
    assert firing_pattern == expected, "Rust NPU should fire every burst"


def test_rust_consecutive_limit_zero_means_unlimited():
    """
    REAL Rust test: consecutive_fire_limit=0
    
    Should fire indefinitely without ever applying extended refractory.
    """
    npu, neuron_id = create_test_npu_with_neuron(
        refractory_period=2,
        consecutive_fire_limit=0,  # Unlimited
        snooze_period=10  # Should be ignored
    )
    
    firing_pattern = run_burst_with_power_injection(npu, neuron_id, 30)
    
    # With refractory=2, should fire every 3 bursts (fire at 0, 3, 6, 9, ...)
    expected_fires = list(range(0, 30, 3))
    
    print(f"\nRust NPU Test: refractory=2, limit=0 (unlimited), snooze=10")
    print(f"Expected: {expected_fires}")
    print(f"Actual:   {firing_pattern}")
    
    assert firing_pattern == expected_fires, "Rust NPU should fire every 3 bursts indefinitely"
    
    # Check intervals are all 3
    intervals = [firing_pattern[i+1] - firing_pattern[i] for i in range(len(firing_pattern)-1)]
    assert all(i == 3 for i in intervals), f"All intervals should be 3, got {intervals}"


def test_rust_snooze_zero_no_extended_refractory():
    """
    REAL Rust test: snooze_period=0
    
    Hitting consecutive_fire_limit should still reset counter but not add extra refractory.
    """
    npu, neuron_id = create_test_npu_with_neuron(
        refractory_period=1,
        consecutive_fire_limit=3,
        snooze_period=0  # No extended refractory
    )
    
    firing_pattern = run_burst_with_power_injection(npu, neuron_id, 20)
    
    visual = visualize_pattern(firing_pattern, 20)
    expected_pattern = "1_1_1_1_1_1_1_1_1_1_"
    
    print(f"\nRust NPU Test: refractory=1, limit=3, snooze=0")
    print(f"Expected: {expected_pattern}")
    print(f"Actual:   {visual}")
    
    assert visual == expected_pattern, \
        f"Rust NPU failed: Expected {expected_pattern}, got {visual}"


def test_rust_realistic_scenario():
    """
    REAL Rust test: Realistic scenario matching user's genome.
    
    - refractory_period=1: Skip 1 burst between fires
    - consecutive_fire_limit=3: Can fire 3 times in a row
    - snooze_period=2: After 3rd fire, skip 3 total bursts (1 normal + 2 snooze)
    
    Expected pattern: 1_1_1___1_1_1___
    - Fire at 0, skip 1, fire at 2, skip 1, fire at 4 (hit limit=3)
    - Skip 3 bursts (1+2), fire at 8, repeat
    """
    npu, neuron_id = create_test_npu_with_neuron(
        refractory_period=1,
        consecutive_fire_limit=3,
        snooze_period=2
    )
    
    firing_pattern = run_burst_with_power_injection(npu, neuron_id, 24)
    
    visual = visualize_pattern(firing_pattern, 24)
    expected_pattern = "1_1_1___1_1_1___1_1_1___"
    
    print(f"\nRust NPU Test: Realistic scenario (refractory=1, limit=3, snooze=2)")
    print(f"Expected: {expected_pattern}")
    print(f"Actual:   {visual}")
    print(f"Firing: {firing_pattern}")
    
    assert visual == expected_pattern, \
        f"Rust NPU failed realistic scenario!\nExpected: {expected_pattern}\nActual: {visual}"
    
    # Detailed verification
    expected_fires = [0, 2, 4, 8, 10, 12, 16, 18, 20]
    assert firing_pattern == expected_fires, \
        f"Rust NPU firing pattern incorrect!\nExpected: {expected_fires}\nActual: {firing_pattern}"


def test_rust_user_exact_scenario():
    """
    REAL Rust test: Exact scenario from user's Fire Ledger.
    
    User reported neuron 16438 firing EVERY SINGLE BURST with:
    - refractory_period=1
    - consecutive_fire_limit=3
    - snooze_period=0
    
    This should produce pattern: 1_1_1_1_1_1_ (NOT 111111111111)
    """
    npu, neuron_id = create_test_npu_with_neuron(
        refractory_period=1,
        consecutive_fire_limit=3,
        snooze_period=0
    )
    
    # Run 20 bursts (same as Fire Ledger window)
    firing_pattern = run_burst_with_power_injection(npu, neuron_id, 20)
    
    visual = visualize_pattern(firing_pattern, 20)
    expected_pattern = "1_1_1_1_1_1_1_1_1_1_"  # Should NOT be "11111111111111111111"
    
    print(f"\n{'='*70}")
    print(f"USER'S EXACT SCENARIO TEST:")
    print(f"  refractory_period=1, consecutive_fire_limit=3, snooze_period=0")
    print(f"  Expected: {expected_pattern}")
    print(f"  Actual:   {visual}")
    print(f"  Firing: {firing_pattern}")
    
    if visual == "11111111111111111111":
        print(f"\n❌ CRITICAL BUG: Neuron fired EVERY burst (no refractory)!")
        print(f"   This matches the user's Fire Ledger bug!")
    
    print(f"{'='*70}\n")
    
    assert visual == expected_pattern, \
        f"CRITICAL BUG REPRODUCED!\n" \
        f"Rust NPU allows continuous firing despite refractory_period=1!\n" \
        f"Expected: {expected_pattern}\n" \
        f"Actual:   {visual}\n" \
        f"This is the EXACT bug the user reported!"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

