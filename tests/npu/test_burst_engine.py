import time
import threading
import types
from feagi.npu.burst_engine import BurstEngine

class MockFCLManager:
    def __init__(self):
        self.area_fcl_history = {1: [set() for _ in range(3)]}
        self.current_window_index = 0

class MockConnectomeManager:
    def __init__(self):
        self.cortical_areas = {1: types.SimpleNamespace(id=1, properties={'__shed': True})}
        self.fcl_manager = MockFCLManager()
        self.calls = 0
    def update_membrane_potentials(self):
        self.calls += 1
        # Simulate some work
        time.sleep(0.01)
        return [1]

def test_burst_engine_runs_and_stops():
    connectome = MockConnectomeManager()
    engine = BurstEngine(connectome, desired_frequency_hz=20)
    # Run the burst engine in a separate thread
    t = threading.Thread(target=engine.run)
    t.start()
    # Let it run for a few bursts
    time.sleep(0.1)
    engine.stop()
    t.join(timeout=2)
    assert not engine.running
    assert connectome.calls > 0
    print(f"BurstEngine ran for {connectome.calls} bursts and stopped cleanly.")

if __name__ == "__main__":
    test_burst_engine_runs_and_stops() 