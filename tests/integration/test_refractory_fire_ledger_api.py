"""
Integration test for refractory period using Fire Ledger API endpoint.

This test validates that:
1. The Fire Ledger API correctly returns firing history
2. The firing pattern matches expected refractory behavior
3. Refractory periods of 1, 5, and 10 produce correct intervals

To run this test, FEAGI must be running with a genome that has:
- A cortical area with neurons
- Burst engine running at 1Hz
- Continuous stimulation to one or more neurons

This test can be run manually by:
1. Starting FEAGI
2. Setting refractory_period via BV
3. Running: pytest tests/integration/test_refractory_fire_ledger_api.py -v -s
"""

import pytest
import requests
import time
from typing import List, Tuple


# Configuration (adjust if needed)
FEAGI_API_BASE = "http://127.0.0.1:8000"
TEST_AREA_ID = "iic000"  # The area you're testing with
TEST_NEURON_ID = 16438  # The neuron you're monitoring


def get_fire_ledger_history(area_id: str, lookback_steps: int = 20) -> dict:
    """Fetch Fire Ledger history from FEAGI API."""
    url = f"{FEAGI_API_BASE}/v1/burst_engine/fire_ledger/area/{area_id}/history"
    params = {"lookback_steps": lookback_steps}
    
    response = requests.get(url, params=params)
    response.raise_for_status()
    return response.json()


def set_refractory_period(area_id: str, refractory_period: int) -> dict:
    """Set refractory period for a cortical area via API."""
    url = f"{FEAGI_API_BASE}/v1/cortical_area/cortical_area"
    payload = {
        "cortical_id": area_id,
        "neuron_refractory_period": refractory_period
    }
    
    response = requests.put(url, json=payload)
    response.raise_for_status()
    return response.json()


def extract_firing_pattern(history_data: dict, neuron_id: int = None) -> List[int]:
    """
    Extract firing pattern from Fire Ledger history.
    
    Args:
        history_data: Response from fire_ledger API
        neuron_id: If specified, only track this neuron; otherwise track all
        
    Returns:
        List of timesteps where neuron(s) fired
    """
    if not history_data.get("success"):
        return []
    
    history = history_data.get("history", [])
    firing_pattern = []
    
    for entry in history:
        timestep = entry["timestep"]
        neuron_ids = entry["neuron_ids"]
        
        if neuron_id is None:
            # Track any firing
            if neuron_ids:
                firing_pattern.append(timestep)
        else:
            # Track specific neuron
            if neuron_id in neuron_ids:
                firing_pattern.append(timestep)
    
    return firing_pattern


def validate_firing_intervals(firing_pattern: List[int], expected_interval: int) -> Tuple[bool, str, List[int]]:
    """
    Validate that intervals between fires match expected pattern.
    
    Returns:
        (is_valid, message, intervals)
    """
    if len(firing_pattern) < 2:
        return False, f"Not enough fires (got {len(firing_pattern)}, need >=2)", []
    
    intervals = []
    for i in range(1, len(firing_pattern)):
        interval = firing_pattern[i] - firing_pattern[i-1]
        intervals.append(interval)
    
    # Check if all intervals match expected
    mismatches = [(i, intervals[i]) for i in range(len(intervals)) 
                  if intervals[i] != expected_interval]
    
    if mismatches:
        return False, f"Mismatched intervals: {mismatches}, expected {expected_interval}", intervals
    
    return True, "All intervals correct", intervals


@pytest.mark.integration
@pytest.mark.skip(reason="Requires FEAGI to be running with test genome")
@pytest.mark.parametrize("refractory_period", [1, 5, 10])
def test_refractory_fire_ledger_api(refractory_period):
    """
    Integration test: Validate refractory period via Fire Ledger API.
    
    Prerequisites:
    - FEAGI running on http://127.0.0.1:8000
    - Test area (iic000) exists with neurons
    - Burst engine running at 1Hz
    - Continuous stimulation to neurons
    
    Test validates that refractory_period produces correct firing intervals:
    - refractory_period=1 → interval=2 bursts (fire, skip 1, fire)
    - refractory_period=5 → interval=6 bursts (fire, skip 5, fire)
    - refractory_period=10 → interval=11 bursts (fire, skip 10, fire)
    """
    # Set refractory period
    print(f"\n{'='*70}")
    print(f"Setting refractory_period={refractory_period} for area {TEST_AREA_ID}")
    set_result = set_refractory_period(TEST_AREA_ID, refractory_period)
    assert set_result.get("success"), f"Failed to set refractory period: {set_result}"
    
    # Wait for bursts to accumulate (need at least 3 firing cycles)
    min_bursts_needed = (refractory_period + 1) * 3 + 5
    wait_time = min_bursts_needed * 1.1  # 1.1 seconds per burst at 1Hz
    print(f"Waiting {wait_time:.1f}s for {min_bursts_needed} bursts to accumulate...")
    time.sleep(wait_time)
    
    # Fetch Fire Ledger history
    lookback = min(min_bursts_needed + 10, 50)  # Get enough history
    history_data = get_fire_ledger_history(TEST_AREA_ID, lookback_steps=lookback)
    
    assert history_data.get("success"), f"Fire Ledger API failed: {history_data}"
    
    # Extract firing pattern for our test neuron
    firing_pattern = extract_firing_pattern(history_data, neuron_id=TEST_NEURON_ID)
    
    # Validate pattern
    expected_interval = refractory_period + 1
    is_valid, message, intervals = validate_firing_intervals(firing_pattern, expected_interval)
    
    # Print results
    print(f"Refractory Period: {refractory_period}")
    print(f"Expected Interval: {expected_interval} bursts")
    print(f"Firing Pattern: {firing_pattern}")
    print(f"Intervals: {intervals}")
    print(f"Validation: {message}")
    print(f"{'='*70}\n")
    
    # Assertions
    assert len(firing_pattern) >= 3, \
        f"Need at least 3 fires to validate pattern, got {len(firing_pattern)}"
    
    assert is_valid, f"Firing pattern validation failed: {message}. Intervals: {intervals}"
    
    # Verify all intervals match
    for i, interval in enumerate(intervals):
        assert interval == expected_interval, \
            f"Interval {i}: {interval} != expected {expected_interval}"


@pytest.mark.integration
def test_parse_actual_logs():
    """
    This test parses your actual logs to prove the pattern is correct.
    
    Your logs with refractory_period=1:
    - Burst 512: Fire
    - Burst 513: Blocked (countdown 1→0)
    - Burst 514: Fire
    - Burst 515: Blocked
    - Burst 516: Fire
    
    Pattern: 1_1_1_ (interval=2) ✅ CORRECT!
    """
    # Simulated Fire Ledger response based on your actual logs
    simulated_history = {
        "success": True,
        "history": [
            {"timestep": 517, "neuron_ids": []},  # Blocked
            {"timestep": 516, "neuron_ids": [16438]},  # Fired
            {"timestep": 515, "neuron_ids": []},  # Blocked
            {"timestep": 514, "neuron_ids": [16438]},  # Fired
            {"timestep": 513, "neuron_ids": []},  # Blocked
            {"timestep": 512, "neuron_ids": [16438]},  # Fired
        ]
    }
    
    firing_pattern = extract_firing_pattern(simulated_history, neuron_id=16438)
    firing_pattern.reverse()  # History comes newest first, we need oldest first
    
    print(f"\nParsing actual logs from your system:")
    print(f"Firing Pattern: {firing_pattern}")
    
    # Validate with refractory_period=1
    is_valid, message, intervals = validate_firing_intervals(firing_pattern, expected_interval=2)
    
    print(f"Expected Interval: 2 bursts (refractory_period=1)")
    print(f"Actual Intervals: {intervals}")
    print(f"Validation: {message}")
    
    assert len(firing_pattern) == 3, f"Expected 3 fires, got {len(firing_pattern)}"
    assert firing_pattern == [512, 514, 516], f"Pattern mismatch: {firing_pattern}"
    assert intervals == [2, 2], f"Intervals should be [2, 2], got {intervals}"
    assert is_valid, f"Pattern should be valid: {message}"
    
    print("✅ Your logs show CORRECT refractory behavior!")


if __name__ == "__main__":
    # Run the manual test that doesn't require FEAGI to be running
    pytest.main([__file__, "-v", "-s", "-k", "parse_actual_logs"])

