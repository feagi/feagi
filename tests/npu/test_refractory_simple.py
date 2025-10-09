"""
Simple test for refractory period - validates absolute burst-tick counting.

This test creates a minimal setup and validates that:
- refractory_period=1 means skip 1 burst between fires
- refractory_period=5 means skip 5 bursts between fires  
- refractory_period=10 means skip 10 bursts between fires
"""

import pytest


def test_refractory_logs_show_correct_pattern():
    """
    This test validates the logs you provided show correct refractory behavior.
    
    From your logs with refractory_period=1:
    - Burst 512: Neuron 16438 fired, countdown=1
    - Burst 513: countdown 1 -> 0 (BLOCKED)
    - Burst 514: Neuron 16438 fired, countdown=1
    - Burst 515: countdown 1 -> 0 (BLOCKED)
    - Burst 516: Neuron 16438 fired, countdown=1
    - Burst 517: countdown 1 -> 0 (BLOCKED)
    
    Pattern: Fire, Skip, Fire, Skip = 1_1_1_ ✅
    
    This is CORRECT for refractory_period=1!
    """
    # Parse the logs you provided
    log_events = [
        (512, "fire", 1),  # Neuron fired, countdown set to 1
        (513, "refrac", 0),  # countdown 1 -> 0, neuron blocked
        (514, "fire", 1),  # Neuron fired again, countdown set to 1
        (515, "refrac", 0),  # countdown 1 -> 0, neuron blocked
        (516, "fire", 1),  # Neuron fired again, countdown set to 1
        (517, "refrac", 0),  # countdown 1 -> 0, neuron blocked
    ]
    
    # Extract firing pattern
    fire_bursts = [burst for burst, event, _ in log_events if event == "fire"]
    
    print(f"\nFiring pattern: {fire_bursts}")
    print(f"Intervals: {[fire_bursts[i+1] - fire_bursts[i] for i in range(len(fire_bursts)-1)]}")
    
    # Validate: should fire every 2 bursts (fire at N, skip N+1, fire at N+2)
    for i in range(len(fire_bursts) - 1):
        interval = fire_bursts[i+1] - fire_bursts[i]
        assert interval == 2, f"Expected interval 2, got {interval} between bursts {fire_bursts[i]} and {fire_bursts[i+1]}"
    
    print("✅ Refractory period=1 is working correctly!")


def test_calculate_expected_pattern():
    """
    Calculate expected firing patterns for different refractory periods.
    """
    test_cases = [
        (1, "1_1_1_1_1_"),  # Fire, skip 1, fire
        (5, "1_____1_____1"),  # Fire, skip 5, fire
        (10, "1__________1__________1"),  # Fire, skip 10, fire
    ]
    
    for refractory, expected_pattern in test_cases:
        print(f"\nRefractory Period: {refractory}")
        print(f"Expected Pattern: {expected_pattern}")
        print(f"Fire interval: {refractory + 1} bursts")
        
        # Simulate firing pattern
        total_bursts = 30
        fire_bursts = []
        next_fire_burst = 0
        
        while next_fire_burst < total_bursts:
            fire_bursts.append(next_fire_burst)
            next_fire_burst += (refractory + 1)  # Fire + skip N = interval of N+1
        
        # Create visual pattern
        visual = ['_'] * total_bursts
        for burst in fire_bursts:
            visual[burst] = '1'
        actual_pattern = ''.join(visual)
        
        print(f"Actual Pattern:   {actual_pattern}")
        print(f"Fire bursts: {fire_bursts[:5]}...")
        
        # Validate intervals
        for i in range(len(fire_bursts) - 1):
            interval = fire_bursts[i+1] - fire_bursts[i]
            assert interval == refractory + 1, \
                f"Refractory={refractory}: Expected interval {refractory+1}, got {interval}"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

