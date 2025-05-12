import time
import threading
import types
from queue import Queue, Empty
from feagi.npu.burst_engine import BurstEngine, FCLSampler
import pytest
from unittest.mock import Mock

class MockFCLManager:
    def __init__(self):
        self.area_fcl_history = {1: [set() for _ in range(3)]}
        self.current_window_index = 0
        self.counter = 0
    def get_global_fcl(self):
        # Return a unique value each call for testing
        self.counter += 1
        return f"fcl_snapshot_{self.counter}"

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
    # Fix: Explicitly provide both connectome_manager and fcl_manager
    engine = BurstEngine(
        connectome_manager=connectome, 
        fcl_manager=connectome.fcl_manager,
        config={"target_frequency": 20}
    )
    # Run the burst engine in a separate thread
    t = threading.Thread(target=engine.run)
    t.start()
    # Let it run for a few bursts
    time.sleep(0.1)
    engine.stop()
    t.join(timeout=2)
    assert not engine._running
    assert connectome.calls > 0
    print(f"BurstEngine ran for {connectome.calls} bursts and stopped cleanly.")

def test_fcl_sampler_samples_and_stops():
    fcl_manager = MockFCLManager()
    output_queue = Queue(maxsize=10)
    sampler = FCLSampler(fcl_manager, sample_frequency_hz=10, output_queue=output_queue)
    t = threading.Thread(target=sampler.run)
    t.start()
    # Let it sample a few times
    time.sleep(0.15)
    sampler.stop()
    t.join(timeout=2)
    # Collect samples from the queue
    samples = []
    try:
        while True:
            samples.append(output_queue.get_nowait())
    except Empty:
        pass
    assert len(samples) > 0, "FCLSampler did not sample any FCLs."
    print(f"FCLSampler sampled {len(samples)} FCLs: {samples}")

@pytest.fixture
def engine():
    """Create a burst engine for testing"""
    cm = Mock()
    fcl = Mock()
    config = {"target_frequency": 60.0}
    engine = BurstEngine(connectome_manager=cm, fcl_manager=fcl, config=config)
    return engine

if __name__ == "__main__":
    test_burst_engine_runs_and_stops()
    test_fcl_sampler_samples_and_stops() 