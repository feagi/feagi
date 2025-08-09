import time
import types

import pytest

from feagi.core.state_manager import FeagiStateManager


class DummyFCL:
    def __init__(self):
        self.current_timestep = 0

    def get_global_fcl(self, ts):
        return []


class DummyConnectome:
    def __init__(self):
        self.memory_neuron_array = types.SimpleNamespace(
            age_by_bursts=lambda delta: [],
            check_longterm_conversion=lambda longterm_threshold=100: [],
            collect_garbage=lambda current_burst, prune_inactive_after_bursts: 0,
        )
        self.genome = {
            "physiology": {
                "sleep_trigger_inactivity_window": 3,
                "sleep_trigger_neural_activity_max": 5,
            }
        }


class DummyMemoryProcessor:
    def __init__(self):
        self._updates = []

    def _update_state_manager_neuron_count(self, increment: int) -> None:
        self._updates.append(increment)


def test_sleep_manager_uses_cumulative_counters(monkeypatch):
    from feagi.process_manager import SleepManager

    fcl = DummyFCL()
    cm = DummyConnectome()
    mp = DummyMemoryProcessor()

    sm = FeagiStateManager.instance()
    sm.reset_cumulative_activity()

    # Configure a short monitoring interval
    sleep_mgr = SleepManager(
        fcl_manager=fcl,
        connectome_manager=cm,
        memory_processor=mp,
        window_bursts=50,  # overridden by genome physiology
        activity_threshold=9999,  # overridden by genome physiology
        monitor_interval=0.05,
        gc_prune_after_bursts=100,
    )

    try:
        sleep_mgr.start()
        # Simulate 3 bursts with small cumulative activity under threshold
        for i in range(3):
            fcl.current_timestep = i
            FeagiStateManager.instance().increment_cumulative_activity(1)
            time.sleep(0.06)
        # Give a moment for background thread to process
        time.sleep(0.2)
        counters = sm.get_cumulative_activity()
        # Expect counters reset to zero after maintenance ran
        assert counters["bursts"] == 0 or counters["bursts"] < 3
    finally:
        sleep_mgr.stop() 