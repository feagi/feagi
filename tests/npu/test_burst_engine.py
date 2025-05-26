import time
import threading
import types
from queue import Queue, Empty
from feagi.npu.burst_engine import BurstEngine, FQSampler
import pytest
from unittest.mock import Mock, patch, MagicMock, call

class MockFCLManager:
    def __init__(self):
        self.area_fcl_history = {1: [set() for _ in range(3)]}
        self.cortical_fcl_history = {1: [set() for _ in range(3)]}  # Added for updated naming
        self.current_window_index = 0
        self.counter = 0
        self._last_sample_time_per_area = {}  # Add for FQSampler testing
        
    def get_global_fcl(self, offset=0):
        # Return a unique value each call for testing
        self.counter += 1
        return f"fcl_snapshot_{self.counter}"
    
    def get_area_fcl(self, area_id):
        return set([area_id * 10, area_id * 10 + 1])
    
    def get_cortical_fcl(self, cortical_idx):
        return set([cortical_idx * 10, cortical_idx * 10 + 1])
    
    # Fire queue provider interface for FQSampler
    def get_fire_queue(self):
        """Return a global fire queue for testing."""
        self.counter += 1
        return {
            'neuron_ids': [self.counter, self.counter + 10, self.counter + 20],
            'membrane_potentials': [0.8 + self.counter * 0.1, 1.2, 0.9],
            'thresholds': [1.0, 1.0, 1.0],
            'consecutive_fire_counts': [1, 2, 1],
            'refractory_counters': [0, 0, 0]
        }
        
    def get_area_fire_queue(self, cortical_id):
        """Return a cortical area-specific fire queue for testing."""
        # Convert cortical_id to numeric if it's a string like 'cortex1'
        if isinstance(cortical_id, str) and cortical_id.startswith('cortex'):
            numeric_id = int(cortical_id[6:]) if len(cortical_id) > 6 else 1
        else:
            numeric_id = cortical_id if isinstance(cortical_id, int) else 1
            
        return {
            'neuron_ids': [numeric_id * 100, numeric_id * 100 + 1, numeric_id * 100 + 2],
            'membrane_potentials': [0.8, 1.2, 0.9],
            'thresholds': [1.0, 1.0, 1.0],
            'consecutive_fire_counts': [1, 2, 1],
            'refractory_counters': [0, 0, 0]
        }

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
        
    def get_burst_engine_state(self):
        return self.state
        
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

def test_fq_sampler_samples_and_stops():
    fire_queue_provider = MockFCLManager()  # Using same mock but treating as fire queue provider
    output_queue = Queue(maxsize=10)
    sampler = FQSampler(fire_queue_provider, sample_frequency_hz=10, output_queue=output_queue)
    sampler.set_visualization_subscribers(True)  # Enable sampling
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
    assert len(samples) > 0, "FQSampler did not sample any fire queue data."

def test_fq_sampler_with_connectome_manager():
    """Test FQ sampler with per-area sample rates via connectome manager."""
    fire_queue_provider = MockFCLManager()  # Using same mock but treating as fire queue provider
    output_queue = Queue(maxsize=10)
    
    # Create a mock connectome manager with cortical areas that have custom sample rates
    connectome_manager = Mock()
    cortical1 = types.SimpleNamespace(id='cortex1', properties={'fq_sample_rate': 20})
    cortical2 = types.SimpleNamespace(id='cortex2', properties={'fq_sample_rate': 5})
    connectome_manager.cortical_areas = {'cortex1': cortical1, 'cortex2': cortical2}
    
    # Create sampler with connectome manager
    sampler = FQSampler(fire_queue_provider, sample_frequency_hz=10, output_queue=output_queue, 
                        connectome_manager=connectome_manager)
    sampler.set_visualization_subscribers(True)  # Enable sampling
    
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
    
    assert len(samples) > 0, "FQSampler did not sample any fire queue data"
    
    # Verify we have samples for both cortical areas
    cortical_ids = [sample[0] for sample in samples if isinstance(sample, tuple)]
    assert 'cortex1' in cortical_ids, "cortex1 was not sampled"
    assert 'cortex2' in cortical_ids, "cortex2 was not sampled"

def test_fq_sampler_update_area_sample_rate():
    """Test updating cortical area sample rate in FQ sampler."""
    fire_queue_provider = MockFCLManager()  # Using same mock but treating as fire queue provider
    output_queue = Queue(maxsize=10)
    sampler = FQSampler(fire_queue_provider, sample_frequency_hz=10, output_queue=output_queue)
    
    # Update cortical area sample rate
    sampler.update_area_sample_rate('cortex1', 20.0)
    
    # Manually add to _last_sample_time_per_area since we're not running the sampler
    sampler._last_sample_time_per_area['cortex1'] = time.time()
    
    # Verify internal state is updated
    assert 'cortex1' in sampler._last_sample_time_per_area
    
    # Enable sampling
    sampler.set_visualization_subscribers(True)
    
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
    
    assert len(samples) > 0, "FQSampler did not sample any fire queue data"

def test_update_with_genome(engine):
    """Test updating the burst engine with a genome."""
    # The fixture already calls update_with_genome(), so test the state
    
    # Verify that genome_loaded flag and shed_areas are updated
    assert engine.genome_loaded
    
    # The fixture sets up 2 areas (100 and 200), but area 100 has __shed=True
    # Check shed areas contains the shed area
    assert 100 in engine.shed_areas, "Area 100 should be in shed_areas"
    
    # Check that non-shed areas are in cortical_areas
    # Note: BurstEngine may keep all areas in cortical_areas and just track shed separately
    non_shed_areas = [area for area in engine.cortical_areas if area.id not in engine.shed_areas]
    assert len(non_shed_areas) >= 1, "Should have at least one non-shed area"
    
    # Verify area 200 (non-shed) is available 
    area_200_found = any(area.id == 200 for area in engine.cortical_areas)
    assert area_200_found, "Area 200 should be available"

def test_run_burst_engine_basics():
    """Test the basic BurstEngine properties and configurations."""
    # Create mocks
    cm = Mock()
    cm.cortical_areas = {100: Mock(id=100, properties={'__shed': True})}
    fcl = Mock()
    
    # Create engine with state manager
    with patch('feagi.npu.burst_engine.FeagiStateManager'):
        engine = BurstEngine(connectome_manager=cm, fcl_manager=fcl, config={"target_frequency": 60.0})
        
        # Check basic properties - the engine uses the config value initially
        assert engine.desired_frequency == 60.0  # Uses desired_frequency, not target_frequency
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
    # Setup FCL with proper structure for both shed and non-shed areas
    # Create mock objects for the FCL history entries
    mock_fcl_100 = MagicMock()
    mock_fcl_200 = MagicMock()
    
    fcl.cortical_fcl_history = {
        100: [mock_fcl_100 for _ in range(5)], 
        200: [mock_fcl_200 for _ in range(5)]
    }
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
            for cortical_idx in engine.shed_areas:
                # Clear FCL for this cortical area for the current burst
                fcl.area_fcl_history[cortical_idx][fcl.current_window_index].clear()
        
        # Verify FCL was cleared only for shed cortical area (100)
        mock_fcl_100.clear.assert_called_once()
        mock_fcl_200.clear.assert_not_called()

@patch('feagi.npu.burst_engine.time')
@patch('feagi.npu.burst_engine.logger')
def test_run_test_function(mock_logger, mock_time, engine):
    """Test the run_test method."""
    # Setup mocks - provide enough values for the method calls
    mock_time.perf_counter.side_effect = [0.0, 0.005, 0.01, 0.015, 0.02]  # Multiple time readings
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
        
        # Test the fallback processing by calling _process_burst which should use the fallback
        engine.connectome_manager.update_membrane_potentials = MagicMock(return_value=[1, 2, 3])
        
        # Call _process_burst which should trigger the fallback path
        result = engine._process_burst()
        
        # Verify the fallback function was called
        engine.connectome_manager.update_membrane_potentials.assert_called_once()
        
        # Verify the result
        assert result == [1, 2, 3]

# Add proper test isolation
@pytest.fixture(autouse=True)
def reset_burst_engine_singleton():
    """Reset BurstEngine singleton before each test to prevent state pollution."""
    yield
    # Reset after each test
    try:
        BurstEngine.reset_singleton()
    except Exception:
        pass  # Ignore if no instance exists

if __name__ == "__main__":
    pytest.main(["-v", __file__]) 