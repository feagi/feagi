"""
Comprehensive test for refractory trio: refractory_period, consecutive_fire_limit, snooze_period.

This test validates all combinations and corner cases:
- refractory_period: Normal refractory (skip N bursts between fires)
- consecutive_fire_limit: Max consecutive fires before extended refractory
- snooze_period: Additional refractory bursts after hitting consecutive limit

Expected behavior:
1. Neuron fires, sets refractory_countdown = refractory_period
2. Countdown decrements each burst, neuron blocked until countdown=0
3. If consecutive_fire_count reaches consecutive_fire_limit, apply extended refractory:
   - countdown = refractory_period + snooze_period
4. When countdown expires after extended refractory, reset consecutive_fire_count to 0

Corner cases:
- refractory_period=0: No normal refractory (can fire every burst)
- consecutive_fire_limit=0: Unlimited fires (no extended refractory)
- snooze_period=0: No additional refractory when hitting limit
"""

import pytest


def simulate_firing_pattern(refractory_period, consecutive_fire_limit, snooze_period, num_bursts, always_above_threshold=True):
    """
    Simulate neuron firing pattern given the refractory trio parameters.
    
    Args:
        refractory_period: Normal refractory period (bursts to skip between fires)
        consecutive_fire_limit: Max consecutive fires before extended refractory (0=unlimited)
        snooze_period: Additional refractory bursts after hitting limit
        num_bursts: Total bursts to simulate
        always_above_threshold: If True, neuron always wants to fire (when not in refractory)
        
    Returns:
        (firing_pattern, details)
        - firing_pattern: List of burst numbers where neuron fired
        - details: Dict with diagnostic info for each burst
    """
    firing_pattern = []
    details = {}
    
    # Neuron state
    refractory_countdown = 0
    consecutive_fire_count = 0
    
    for burst in range(num_bursts):
        # Decrement refractory countdown
        if refractory_countdown > 0:
            old_countdown = refractory_countdown
            refractory_countdown -= 1
            
            # Check if extended refractory just expired
            if refractory_countdown == 0 and consecutive_fire_limit > 0 and consecutive_fire_count >= consecutive_fire_limit:
                consecutive_fire_count = 0  # Reset on extended refractory expiry
                details[burst] = {
                    'action': 'blocked_refrac_expired',
                    'old_countdown': old_countdown,
                    'new_countdown': refractory_countdown,
                    'cfc': consecutive_fire_count,
                }
            else:
                details[burst] = {
                    'action': 'blocked_refrac',
                    'old_countdown': old_countdown,
                    'new_countdown': refractory_countdown,
                    'cfc': consecutive_fire_count,
                }
            continue
        
        # Not in refractory - can potentially fire
        if always_above_threshold:
            # Fire!
            consecutive_fire_count += 1
            firing_pattern.append(burst)
            
            # Apply refractory
            if consecutive_fire_limit > 0 and consecutive_fire_count >= consecutive_fire_limit:
                # Hit limit - apply extended refractory
                refractory_countdown = refractory_period + snooze_period
                details[burst] = {
                    'action': 'fire_extended',
                    'cfc': consecutive_fire_count,
                    'countdown': refractory_countdown,
                    'refrac': refractory_period,
                    'snooze': snooze_period,
                }
            else:
                # Normal fire - apply normal refractory
                refractory_countdown = refractory_period
                details[burst] = {
                    'action': 'fire_normal',
                    'cfc': consecutive_fire_count,
                    'countdown': refractory_countdown,
                }
        else:
            details[burst] = {
                'action': 'below_threshold',
                'cfc': consecutive_fire_count,
            }
    
    return firing_pattern, details


def visualize_pattern(firing_pattern, num_bursts):
    """Create visual representation of firing pattern."""
    visual = ['_'] * num_bursts
    for burst in firing_pattern:
        if 0 <= burst < num_bursts:
            visual[burst] = '1'
    return ''.join(visual)


def analyze_pattern(firing_pattern, consecutive_fire_limit):
    """Analyze firing pattern to extract cycles and intervals."""
    if not firing_pattern:
        return {
            'cycles': [],
            'normal_intervals': [],
            'extended_intervals': [],
        }
    
    cycles = []
    current_cycle = []
    normal_intervals = []
    extended_intervals = []
    
    for i, burst in enumerate(firing_pattern):
        current_cycle.append(burst)
        
        # Check if we've completed a cycle (hit consecutive limit)
        if consecutive_fire_limit > 0 and len(current_cycle) == consecutive_fire_limit:
            cycles.append(current_cycle)
            current_cycle = []
            
            # Next interval is extended
            if i + 1 < len(firing_pattern):
                extended_intervals.append(firing_pattern[i + 1] - burst)
        elif i + 1 < len(firing_pattern):
            # Normal interval
            interval = firing_pattern[i + 1] - burst
            # Only count if within same cycle
            if consecutive_fire_limit == 0 or len(current_cycle) < consecutive_fire_limit:
                normal_intervals.append(interval)
    
    # Add incomplete cycle
    if current_cycle:
        cycles.append(current_cycle)
    
    return {
        'cycles': cycles,
        'normal_intervals': normal_intervals,
        'extended_intervals': extended_intervals,
    }


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
    (0, 0, 2, "111111111"),         # Fire every burst, unlimited
    
    # Corner case: snooze_period=0 (no extended refractory)
    (1, 3, 0, "1_1_1_1_1_1_"),      # Hit limit but no extra refractory
    (5, 2, 0, "1_____1_____1_____"),  # Hit limit after 2, but no snooze
    
    # Edge cases
    (10, 1, 0, "1__________1__________"),  # Single fire cycles
    (1, 10, 5, "1_1_1_1_1_1_1_1_1_1______"),  # Many fires before snooze
    (2, 2, 3, "1__1_____1__1_____"),  # refrac=2, limit=2, snooze=3
])
def test_refractory_trio_patterns(refrac, cfc_limit, snooze, expected_pattern):
    """
    Test various combinations of refractory trio parameters.
    
    Validates that the firing pattern matches expected behavior.
    """
    num_bursts = len(expected_pattern)
    firing_pattern, details = simulate_firing_pattern(refrac, cfc_limit, snooze, num_bursts)
    
    actual_pattern = visualize_pattern(firing_pattern, num_bursts)
    
    print(f"\n{'='*70}")
    print(f"Parameters:")
    print(f"  refractory_period={refrac}, consecutive_fire_limit={cfc_limit}, snooze_period={snooze}")
    print(f"Expected: {expected_pattern}")
    print(f"Actual:   {actual_pattern}")
    print(f"Firing Pattern: {firing_pattern}")
    
    # Analyze pattern
    analysis = analyze_pattern(firing_pattern, cfc_limit)
    print(f"Analysis:")
    print(f"  Cycles: {analysis['cycles']}")
    print(f"  Normal intervals: {analysis['normal_intervals']}")
    print(f"  Extended intervals: {analysis['extended_intervals']}")
    print(f"{'='*70}\n")
    
    # Assertion
    assert actual_pattern == expected_pattern, \
        f"Pattern mismatch!\nExpected: {expected_pattern}\nActual:   {actual_pattern}"


def test_refractory_period_zero_fires_every_burst():
    """
    Corner case: refractory_period=0, consecutive_fire_limit=0
    
    Should fire every single burst with no restrictions.
    """
    firing_pattern, _ = simulate_firing_pattern(
        refractory_period=0,
        consecutive_fire_limit=0,
        snooze_period=0,
        num_bursts=20
    )
    
    expected = list(range(20))
    print(f"\nrefractory=0, limit=0, snooze=0")
    print(f"Expected: Fire every burst {expected}")
    print(f"Actual:   {firing_pattern}")
    
    assert firing_pattern == expected, "Should fire every burst"


def test_consecutive_limit_zero_means_unlimited():
    """
    Corner case: consecutive_fire_limit=0
    
    Should fire indefinitely without ever applying extended refractory.
    """
    firing_pattern, details = simulate_firing_pattern(
        refractory_period=2,
        consecutive_fire_limit=0,  # Unlimited
        snooze_period=10,  # Should be ignored
        num_bursts=30
    )
    
    # With refractory=2, should fire every 3 bursts (fire at 0, 3, 6, 9, ...)
    expected_fires = list(range(0, 30, 3))
    
    print(f"\nrefractory=2, limit=0 (unlimited), snooze=10")
    print(f"Expected: {expected_fires}")
    print(f"Actual:   {firing_pattern}")
    
    # Check intervals are all 3
    intervals = [firing_pattern[i+1] - firing_pattern[i] for i in range(len(firing_pattern)-1)]
    print(f"Intervals: {intervals}")
    
    assert firing_pattern == expected_fires, "Should fire every 3 bursts indefinitely"
    assert all(i == 3 for i in intervals), "All intervals should be 3"
    
    # Verify no extended refractory was applied
    for burst, detail in details.items():
        if detail['action'].startswith('fire'):
            assert detail['countdown'] == 2, f"Burst {burst}: Should only have normal refractory"


def test_snooze_zero_no_extended_refractory():
    """
    Corner case: snooze_period=0
    
    Hitting consecutive_fire_limit should still reset counter but not add extra refractory.
    """
    firing_pattern, details = simulate_firing_pattern(
        refractory_period=1,
        consecutive_fire_limit=3,
        snooze_period=0,  # No extended refractory
        num_bursts=20
    )
    
    visual = visualize_pattern(firing_pattern, 20)
    print(f"\nrefractory=1, limit=3, snooze=0")
    print(f"Pattern: {visual}")
    print(f"Firing: {firing_pattern}")
    
    # Should fire, skip 1, fire, skip 1, fire, skip 1 (hits limit), repeat
    # Pattern: 1_1_1_1_1_1_ (continuous with normal refractory only)
    expected_pattern = "1_1_1_1_1_1_1_1_1_1_"
    assert visual == expected_pattern, f"Expected {expected_pattern}, got {visual}"
    
    # Verify that hitting limit doesn't add extra refractory
    fire_count = 0
    for burst, detail in details.items():
        if detail['action'] == 'fire_extended':
            fire_count += 1
            # Even when hitting limit, countdown should only be refractory_period (no snooze)
            assert detail['countdown'] == 1, \
                f"Burst {burst}: Extended refractory should be 1 (refrac) + 0 (snooze) = 1, got {detail['countdown']}"
        elif detail['action'] == 'fire_normal':
            assert detail['countdown'] == 1, f"Normal refractory should be 1"


def test_realistic_scenario_refrac1_limit3_snooze2():
    """
    Realistic scenario matching your genome:
    - refractory_period=1: Skip 1 burst between fires
    - consecutive_fire_limit=3: Can fire 3 times in a row
    - snooze_period=2: After 3rd fire, skip 3 total bursts (1 normal + 2 snooze)
    
    Expected pattern: 1_1_1___1_1_1___
    - Fire at 0, skip 1, fire at 2, skip 1, fire at 4 (hit limit=3)
    - Skip 3 bursts (1+2), fire at 8, repeat
    """
    firing_pattern, details = simulate_firing_pattern(
        refractory_period=1,
        consecutive_fire_limit=3,
        snooze_period=2,
        num_bursts=24
    )
    
    visual = visualize_pattern(firing_pattern, 24)
    expected_pattern = "1_1_1___1_1_1___1_1_1___"
    
    print(f"\nRealistic scenario: refractory=1, limit=3, snooze=2")
    print(f"Expected: {expected_pattern}")
    print(f"Actual:   {visual}")
    print(f"Firing: {firing_pattern}")
    
    # Analyze cycles
    analysis = analyze_pattern(firing_pattern, 3)
    print(f"\nAnalysis:")
    print(f"  Cycles: {analysis['cycles']}")
    print(f"  Normal intervals (within cycle): {analysis['normal_intervals']}")
    print(f"  Extended intervals (between cycles): {analysis['extended_intervals']}")
    
    assert visual == expected_pattern, f"Pattern mismatch"
    
    # Verify cycle structure
    assert len(analysis['cycles']) == 3, "Should have 3 complete cycles"
    assert all(len(cycle) == 3 for cycle in analysis['cycles']), "Each cycle should have 3 fires"
    
    # Verify normal intervals (within cycle): should all be 2
    assert all(i == 2 for i in analysis['normal_intervals']), \
        f"Normal intervals should all be 2, got {analysis['normal_intervals']}"
    
    # Verify extended intervals (between cycles): should all be 4
    # (last fire of cycle at T, next cycle starts at T+4: skip 1 for last normal refrac, skip 2 for snooze, fire at T+3+1=T+4)
    assert all(i == 4 for i in analysis['extended_intervals']), \
        f"Extended intervals should all be 4, got {analysis['extended_intervals']}"
    
    # Detailed verification
    expected_fires = [0, 2, 4, 8, 10, 12, 16, 18, 20]
    assert firing_pattern == expected_fires, f"Expected {expected_fires}, got {firing_pattern}"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

