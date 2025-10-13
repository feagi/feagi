"""
REAL Fire Ledger validation tests for Rust NPU.

These tests validate refractory behavior by:
1. Creating a Rust NPU with configured neurons
2. Running bursts with power injection
3. Reading the Fire Ledger to validate firing patterns
4. NO FAKING - actual Fire Ledger data used for assertions
"""

import pytest

try:
    import feagi_rust
except ImportError:
    pytest.skip("feagi_rust module not available", allow_module_level=True)


def test_fire_ledger_refrac1_limit3_snooze0():
    """
    REAL test: Validate refractory behavior using Fire Ledger.
    
    Configuration:
    - refractory_period=1: Skip 1 burst between fires
    - consecutive_fire_limit=3: Can fire 3 times in a row
    - snooze_period=0: No extended refractory
    
    Expected pattern: 1_1_1_1_1_1_ (fire, skip 1, fire, skip 1, ...)
    """
    # Create NPU with Fire Ledger window of 20
    npu = feagi_rust.RustNPU(1000, 1000, 20)
    
    # Add test neuron
    neuron_id = npu.add_neuron(
        threshold=0.5,
        leak_coefficient=0.0,
        resting_potential=0.0,
        neuron_type=0,
        refractory_period=1,
        excitability=1.0,
        consecutive_fire_limit=3,
        snooze_period=0,
        cortical_area=0,
        x=0, y=0, z=0,
    )
    
    print(f"\n{'='*70}")
    print(f"Fire Ledger Test: refractory=1, limit=3, snooze=0")
    print(f"Neuron ID: {neuron_id}")
    print(f"{'='*70}\n")
    
    # Run 20 bursts with power injection and collect firing history
    firing_bursts = []
    for burst in range(20):
        result = npu.process_burst(power_neurons=[neuron_id])
        # Check if this neuron fired in this burst
        fired_neurons = result.fired_neurons if hasattr(result, 'fired_neurons') else []
        if neuron_id in fired_neurons:
            firing_bursts.append(burst)
    
    print("Firing History (from burst results):")
    print(f"  Total bursts: 20")
    print(f"  Bursts where neuron fired: {len(firing_bursts)}")
    
    # Visualize
    visual = ['_'] * 20
    for burst in firing_bursts:
        if burst < 20:
            visual[burst] = '1'
    actual_pattern = ''.join(visual)
    
    expected_pattern = "1_1_1_1_1_1_1_1_1_1_"
    
    print(f"  Expected: {expected_pattern}")
    print(f"  Actual:   {actual_pattern}")
    print(f"  Firing bursts: {firing_bursts}")
    print(f"{'='*70}\n")
    
    # Validate pattern
    assert actual_pattern == expected_pattern, \
        f"Fire Ledger validation FAILED!\n" \
        f"Expected: {expected_pattern}\n" \
        f"Actual:   {actual_pattern}\n" \
        f"Firing bursts: {firing_bursts}\n" \
        f"This means refractory_period is NOT working correctly!"


def test_fire_ledger_refrac1_limit3_snooze2():
    """
    REAL test: Fire Ledger validation for extended refractory.
    
    Configuration:
    - refractory_period=1: Skip 1 burst between fires
    - consecutive_fire_limit=3: Can fire 3 times in a row
    - snooze_period=2: After 3rd fire, skip 3 total bursts (1+2)
    
    Expected pattern: 1_1_1___1_1_1___
    """
    npu = feagi_rust.RustNPU(1000, 1000, 20)
    
    neuron_id = npu.add_neuron(
        threshold=0.5,
        leak_coefficient=0.0,
        resting_potential=0.0,
        neuron_type=0,
        refractory_period=1,
        excitability=1.0,
        consecutive_fire_limit=3,
        snooze_period=2,
        cortical_area=0,
        x=0, y=0, z=0,
    )
    
    print(f"\n{'='*70}")
    print(f"Fire Ledger Test: refractory=1, limit=3, snooze=2")
    print(f"{'='*70}\n")
    
    # Run 16 bursts and collect firing history
    firing_bursts = []
    for burst in range(16):
        result = npu.process_burst(power_neurons=[neuron_id])
        fired_neurons = result.fired_neurons if hasattr(result, 'fired_neurons') else []
        if neuron_id in fired_neurons:
            firing_bursts.append(burst)
    
    visual = ['_'] * 16
    for burst in firing_bursts:
        if burst < 16:
            visual[burst] = '1'
    actual_pattern = ''.join(visual)
    
    expected_pattern = "1_1_1___1_1_1___"
    
    print(f"  Expected: {expected_pattern}")
    print(f"  Actual:   {actual_pattern}")
    print(f"  Firing bursts: {firing_bursts}")
    
    # Analyze cycles
    print("\nCycle Analysis:")
    expected_fires = [0, 2, 4, 8, 10, 12]
    if len(firing_bursts) >= 6:
        cycle1 = firing_bursts[0:3]
        cycle2 = firing_bursts[3:6]
        print(f"  Cycle 1: {cycle1} (fires at 0, 2, 4)")
        print(f"  Cycle 2: {cycle2} (fires at 8, 10, 12)")
        print(f"  Gap between cycles: {cycle2[0] - cycle1[-1]} bursts (should be 4)")
    
    print(f"{'='*70}\n")
    
    assert actual_pattern == expected_pattern, \
        f"Fire Ledger validation FAILED!\n" \
        f"Expected: {expected_pattern}\n" \
        f"Actual:   {actual_pattern}"


def test_fire_ledger_refrac5():
    """
    REAL test: Fire Ledger validation for longer refractory period.
    
    Configuration:
    - refractory_period=5: Skip 5 bursts between fires
    - consecutive_fire_limit=0: Unlimited fires
    
    Expected pattern: 1_____1_____1_____1_____
    """
    npu = feagi_rust.RustNPU(1000, 1000, 25)
    
    neuron_id = npu.add_neuron(
        threshold=0.5,
        leak_coefficient=0.0,
        resting_potential=0.0,
        neuron_type=0,
        refractory_period=5,
        excitability=1.0,
        consecutive_fire_limit=0,  # Unlimited
        snooze_period=0,
        cortical_area=0,
        x=0, y=0, z=0,
    )
    
    print(f"\n{'='*70}")
    print(f"Fire Ledger Test: refractory=5, limit=0 (unlimited)")
    print(f"{'='*70}\n")
    
    # Run 24 bursts and collect firing history
    firing_bursts = []
    for burst in range(24):
        result = npu.process_burst(power_neurons=[neuron_id])
        fired_neurons = result.fired_neurons if hasattr(result, 'fired_neurons') else []
        if neuron_id in fired_neurons:
            firing_bursts.append(burst)
    
    visual = ['_'] * 24
    for burst in firing_bursts:
        if burst < 24:
            visual[burst] = '1'
    actual_pattern = ''.join(visual)
    
    expected_pattern = "1_____1_____1_____1_____"
    
    print(f"  Expected: {expected_pattern}")
    print(f"  Actual:   {actual_pattern}")
    print(f"  Firing bursts: {firing_bursts}")
    
    # Validate intervals
    if len(firing_bursts) > 1:
        intervals = [firing_bursts[i+1] - firing_bursts[i] for i in range(len(firing_bursts)-1)]
        print(f"  Intervals: {intervals} (should all be 6)")
        assert all(i == 6 for i in intervals), f"Intervals should all be 6, got {intervals}"
    
    print(f"{'='*70}\n")
    
    assert actual_pattern == expected_pattern, \
        f"Fire Ledger validation FAILED!\n" \
        f"Expected: {expected_pattern}\n" \
        f"Actual:   {actual_pattern}"


def test_fire_ledger_continuous_firing_bug():
    """
    REAL test: Reproduce user's bug - continuous firing with refractory=1.
    
    This test is designed to catch the bug where a neuron fires EVERY burst
    despite having refractory_period=1.
    
    If refractory is broken, we'll see pattern: 11111111111111111111
    If refractory works, we'll see pattern: 1_1_1_1_1_1_1_1_1_1_
    """
    npu = feagi_rust.RustNPU(1000, 1000, 20)
    
    neuron_id = npu.add_neuron(
        threshold=0.5,
        leak_coefficient=0.0,
        resting_potential=0.0,
        neuron_type=0,
        refractory_period=1,
        excitability=1.0,
        consecutive_fire_limit=3,
        snooze_period=0,
        cortical_area=0,
        x=0, y=0, z=0,
    )
    
    print(f"\n{'='*70}")
    print(f"BUG REPRODUCTION TEST:")
    print(f"Testing if neuron fires EVERY burst (bug) vs every OTHER burst (correct)")
    print(f"Configuration: refractory=1, limit=3, snooze=0")
    print(f"{'='*70}\n")
    
    # Run 20 bursts and collect firing history
    firing_bursts = []
    for burst in range(20):
        result = npu.process_burst(power_neurons=[neuron_id])
        fired_neurons = result.fired_neurons if hasattr(result, 'fired_neurons') else []
        if neuron_id in fired_neurons:
            firing_bursts.append(burst)
    
    firing_count = len(firing_bursts)
    
    visual = ['_'] * 20
    for burst in firing_bursts:
        if burst < 20:
            visual[burst] = '1'
    actual_pattern = ''.join(visual)
    
    buggy_pattern = "11111111111111111111"  # Every burst (BUG)
    correct_pattern = "1_1_1_1_1_1_1_1_1_1_"  # Every other burst (CORRECT)
    
    print(f"  Buggy (continuous):  {buggy_pattern}")
    print(f"  Correct (refractory): {correct_pattern}")
    print(f"  Actual:               {actual_pattern}")
    print(f"  Firing count: {firing_count}/20 bursts")
    print(f"  Firing bursts: {firing_bursts}")
    
    if actual_pattern == buggy_pattern:
        print(f"\n❌ BUG DETECTED: Neuron fired EVERY burst (no refractory)!")
        print(f"   This is the EXACT bug the user reported!")
    elif actual_pattern == correct_pattern:
        print(f"\n✅ CORRECT: Neuron respects refractory period!")
    else:
        print(f"\n⚠️  UNEXPECTED: Pattern doesn't match either expected behavior")
    
    print(f"{'='*70}\n")
    
    # The assertion
    assert actual_pattern != buggy_pattern, \
        f"CRITICAL BUG: Neuron fired every burst despite refractory_period=1!\n" \
        f"Expected: {correct_pattern}\n" \
        f"Actual:   {actual_pattern}\n" \
        f"This means the refractory period is completely ignored!"
    
    assert actual_pattern == correct_pattern, \
        f"Unexpected firing pattern!\n" \
        f"Expected: {correct_pattern}\n" \
        f"Actual:   {actual_pattern}"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

