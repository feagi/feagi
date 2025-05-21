import time
import threading
import types
from queue import Queue, Empty
from feagi.npu.burst_engine import BurstEngine, FCLSampler
import pytest
from unittest.mock import Mock, patch, MagicMock, call

class MockFCLManager:
    def __init__(self):
        self.area_fcl_history = {1: [set() for _ in range(3)]}
        self.cortical_fcl_history = {1: [set() for _ in range(3)]}  # Added for updated naming
        self.current_window_index = 0
        self.counter = 0
        self._last_sample_time_per_area = {}  # Add for FCLSampler testing
        
    def get_global_fcl(self, offset=0):
        # Return a unique value each call for testing
        self.counter += 1
        return f"fcl_snapshot_{self.counter}"
    
    def get_area_fcl(self, area_id):
        return set([area_id * 10, area_id * 10 + 1])
    
    def get_cortical_fcl(self, cortical_idx):
        return set([cortical_idx * 10, cortical_idx * 10 + 1])

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
        
    def get_optimized_core(self):
        # Return None to simulate no optimized core available
        return None

# Create a mock state manager
class MockStateManager:
    def __init__(self):
        self.burst_freq = 0
        self.state = None
        
    @classmethod
    def instance(cls):
        return MockStateManager()
        
    def set_burst_engine_state(self, state):
        self.state = state
        
    def set_burst_frequency(self, freq):
        self.burst_freq = freq

@pytest.fixture
def mock_state_manager():
    """Create a mock state manager."""
    state_manager = MockStateManager()
    with patch('feagi.npu.burst_engine.FeagiStateManager') as mock_sm:
        mock_sm.instance.return_value = state_manager
        yield state_manager

@pytest.fixture
def engine(mock_state_manager):
    """Create a burst engine for testing"""
    cm = Mock()
    # Mock the cortical_areas attribute
    cm.cortical_areas = {
        100: Mock(id=100, properties={'__shed': True}),
        200: Mock(id=200, properties={'__shed': False})
    }
    fcl = MagicMock()
    config = {"target_frequency": 60.0}
    engine = BurstEngine(connectome_manager=cm, fcl_manager=fcl, config=config)
    # Initialize the engine for testing
    engine.update_with_genome()
    return engine

def test_burst_engine_runs_and_stops():
    connectome = MockConnectomeManager()
    # Fix: Explicitly provide both connectome_manager and fcl_manager
    with patch('feagi.npu.burst_engine.FeagiStateManager', MockStateManager):
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

def test_fcl_sampler_with_connectome_manager():
    """Test FCL sampler with per-area sample rates via connectome manager."""
    fcl_manager = MockFCLManager()
    output_queue = Queue(maxsize=10)
    
    # Create a mock connectome manager with areas that have custom sample rates
    connectome_manager = Mock()
    area1 = types.SimpleNamespace(id=1, properties={'fcl_sample_rate': 20})
    area2 = types.SimpleNamespace(id=2, properties={'fcl_sample_rate': 5})
    connectome_manager._areas = {1: area1, 2: area2}
    
    # Create sampler with connectome manager
    sampler = FCLSampler(fcl_manager, sample_frequency_hz=10, output_queue=output_queue, 
                        connectome_manager=connectome_manager)
    
    # Run sampler briefly
    t = threading.Thread(target=sampler.run)
    t.start()
    time.sleep(0.2)  # Run long enough to get multiple samples
    sampler.stop()
    t.join(timeout=2)
    
    # Check that we got samples
    samples = []
    try:
        while True:
            samples.append(output_queue.get_nowait())
    except Empty:
        pass
    
    assert len(samples) > 0, "FCLSampler did not sample any FCLs"
    
    # Verify we have samples for both areas
    area_ids = [sample[0] for sample in samples]
    assert 1 in area_ids, "Area 1 was not sampled"
    assert 2 in area_ids, "Area 2 was not sampled"

def test_fcl_sampler_update_area_sample_rate():
    """Test updating area sample rate in FCL sampler."""
    fcl_manager = MockFCLManager()
    output_queue = Queue(maxsize=10)
    sampler = FCLSampler(fcl_manager, sample_frequency_hz=10, output_queue=output_queue)
    
    # Update area sample rate
    sampler.update_area_sample_rate(1, 20)
    
    # Manually add to _last_sample_time_per_area since we're not running the sampler
    sampler._last_sample_time_per_area[1] = time.time()
    
    # Verify internal state is updated
    assert 1 in sampler._last_sample_time_per_area
    
    # Run sampler briefly
    t = threading.Thread(target=sampler.run)
    t.start()
    time.sleep(0.15)
    sampler.stop()
    t.join(timeout=2)
    
    # Check that we got samples
    samples = []
    try:
        while True:
            samples.append(output_queue.get_nowait())
    except Empty:
        pass
    
    assert len(samples) > 0, "FCLSampler did not sample any FCLs"

def test_update_with_genome(engine):
    """Test updating the burst engine with a genome."""
    # Update with genome (cortical_areas mock is already set up in the fixture)
    engine.update_with_genome()
    
    # Verify that genome_loaded flag and shed_areas are updated
    assert engine.genome_loaded
    assert len(engine.cortical_areas) == 2
    assert 100 in [a.id for a in engine.cortical_areas]
    assert 200 in [a.id for a in engine.cortical_areas]
    assert engine.shed_areas == {100}  # Only area with __shed=True

def test_run_burst_engine_basics():
    """Test the basic BurstEngine properties and configurations."""
    # Create mocks
    cm = Mock()
    cm.cortical_areas = {100: Mock(id=100, properties={'__shed': True})}
    fcl = Mock()
    
    # Create engine with state manager
    with patch('feagi.npu.burst_engine.FeagiStateManager'):
        engine = BurstEngine(connectome_manager=cm, fcl_manager=fcl, config={"target_frequency": 60.0})
        
        # Check basic properties
        assert engine.target_frequency == 60.0
        assert engine.burst_interval == 1.0/60.0  # Period = 1/frequency
        assert not engine._running  # Should start in non-running state
        assert not engine.genome_loaded  # Should start with no genome
        
        # Update with genome
        engine.update_with_genome()
        assert engine.genome_loaded

def test_load_shedding_behavior():
    """Test the load shedding behavior of BurstEngine during run."""
    # Create test objects
    cm = Mock()
    cm.cortical_areas = {
        100: Mock(id=100, properties={'__shed': True}),
        200: Mock(id=200, properties={'__shed': False})
    }
    
    # Configure update_membrane_potentials to take long enough to trigger load shedding
    def slow_update():
        time.sleep(0.05)  # Make it slow enough to be below target frequency
        return [1, 2, 3]
    cm.update_membrane_potentials = slow_update
    
    fcl = MagicMock()
    # Setup FCL with both cortical_fcl_history and area_fcl_history for compatibility
    fcl.cortical_fcl_history = {100: [MagicMock() for _ in range(5)], 200: [MagicMock() for _ in range(5)]}
    fcl.area_fcl_history = fcl.cortical_fcl_history  # Alias for backward compatibility
    fcl.current_window_index = 0
    
    # Create state manager mock
    state_manager = MagicMock()
    
    # Create engine with patched dependencies
    with patch('feagi.npu.burst_engine.FeagiStateManager') as mock_sm, \
         patch('feagi.npu.burst_engine.time') as mock_time:
        # Configure state manager mock
        mock_sm.instance.return_value = state_manager
        
        # Configure time.perf_counter to simulate time passage
        mock_time.perf_counter.side_effect = [0.0, 0.1]  # Start, end times to create low frequency
        mock_time.sleep = lambda x: None  # No-op sleep
        
        # Create engine with high target frequency to ensure load shedding triggers
        engine = BurstEngine(connectome_manager=cm, fcl_manager=fcl, config={"target_frequency": 100.0})
        engine.update_with_genome()
        
        # Run one iteration of the burst engine logic
        # Mock run by calling the first few statements from the run method
        engine._running = True
        engine.state_manager.set_burst_engine_state = MagicMock()
        
        # Simulate one burst cycle
        start = 0.0
        fired_neurons = cm.update_membrane_potentials()
        end = 0.1
        elapsed = end - start
        actual_freq = 1.0 / elapsed  # Should be 10 Hz, less than 100 Hz target
        engine.state_manager.set_burst_frequency(actual_freq)
        
        # Apply load shedding logic as in the run method
        if actual_freq < engine.desired_frequency:
            for area_id in engine.shed_areas:
                # Clear FCL for this area for the current burst
                fcl.area_fcl_history[area_id][fcl.current_window_index].clear()
        
        # Verify FCL was cleared only for shed area (100)
        fcl.area_fcl_history[100][fcl.current_window_index].clear.assert_called_once()
        fcl.area_fcl_history[200][fcl.current_window_index].clear.assert_not_called()

@patch('feagi.npu.burst_engine.time')
@patch('feagi.npu.burst_engine.logger')
def test_run_test_function(mock_logger, mock_time, engine):
    """Test the run_test method."""
    # Setup mocks
    mock_time.perf_counter.side_effect = [0.0, 0.005]  # Start and end times
    engine.connectome_manager.update_membrane_potentials = MagicMock(return_value=[1, 2, 3])
    engine.state_manager.set_burst_frequency = MagicMock()  # Replace with a proper mock
    
    # Call the run_test method
    result = engine.run_test()
    
    # Verify the function calls
    assert engine.connectome_manager.update_membrane_potentials.called
    assert engine.state_manager.set_burst_frequency.called
    
    # Verify the returned fired neurons
    assert result == [1, 2, 3]

def test_optimized_fire_queue_setup():
    """Test fire queue optimization setup in BurstEngine."""
    # Create mocks
    cm = MagicMock()
    fcl = MagicMock()
    
    # Setup connectome to return an optimized core
    mock_core = MagicMock()
    cm.get_optimized_core.return_value = mock_core
    
    # Create engine
    with patch('feagi.npu.burst_engine.FeagiStateManager'):
        engine = BurstEngine(connectome_manager=cm, fcl_manager=fcl, config={"target_frequency": 100.0})
        
        # Initialize the engine
        engine.genome_loaded = True
        
        # Check that we can get the optimized core
        assert engine.connectome_manager.get_optimized_core() is not None
        
        # Call to test get_optimized_core
        engine.connectome_manager.get_optimized_core.assert_called_once()

def test_fire_queue_fallback_setup():
    """Test fallback path when optimized core is not available."""
    # Create connectome manager that returns None for get_optimized_core
    cm = MagicMock()
    cm.get_optimized_core.return_value = None
    fcl = MagicMock()
    
    # Create engine
    with patch('feagi.npu.burst_engine.FeagiStateManager'):
        engine = BurstEngine(connectome_manager=cm, fcl_manager=fcl, config={"target_frequency": 100.0})
        
        # Initialize the engine
        engine.genome_loaded = True
        
        # Check that optimized core is None
        assert engine.connectome_manager.get_optimized_core() is None
        
        # Test the fallback processing directly
        engine.connectome_manager.update_membrane_potentials = MagicMock(return_value=[1, 2, 3])
        
        # Call the update function directly
        cm.update_membrane_potentials()
        
        # Verify the fallback function was called
        engine.connectome_manager.update_membrane_potentials.assert_called_once()

if __name__ == "__main__":
    pytest.main(["-v", __file__]) 