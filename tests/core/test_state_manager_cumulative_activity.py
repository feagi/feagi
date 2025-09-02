import pytest

from feagi.core.state_manager import FeagiStateManager


def test_cumulative_activity_counters_increment_and_reset():
    sm = FeagiStateManager.instance()
    # Ensure clean start
    sm.reset_cumulative_activity()

    # Increment with several bursts
    sm.increment_cumulative_activity(5)
    sm.increment_cumulative_activity(3)
    sm.increment_cumulative_activity(0)

    counters = sm.get_cumulative_activity()
    assert counters["bursts"] == 3
    assert counters["neurons"] == 8

    # Reset and verify
    sm.reset_cumulative_activity()
    counters2 = sm.get_cumulative_activity()
    assert counters2["bursts"] == 0
    assert counters2["neurons"] == 0 