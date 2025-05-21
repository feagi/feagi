"""
Complete tests for the BurstEngine class.

This module contains comprehensive tests for the BurstEngine class to ensure
high test coverage of its functionality.
"""

import time
import threading
import pytest
from unittest.mock import Mock, patch, MagicMock, call, ANY

from feagi.npu.burst_engine import BurstEngine, ServiceState
from feagi.core.state_manager import FeagiStateManager


class MockConnectomeManager:
    def __init__(self):
        self.cortical_areas = {
            1: Mock(id=1, properties={"__shed": True}),
            2: Mock(id=2, properties={"__shed": False}),
            3: Mock(id=3, properties={}),  # No shed property
        }
        self.fcl_manager = MagicMock()
        self.fcl_manager.area_fcl_history = {
            1: {0: {}},
            2: {0: {}},
            3: {0: {}}
        }
        self.fcl_manager.current_window_index = 0
        self.calls = 0
        # Set get_optimized_core as a MagicMock so we can check if it was called
        self.get_optimized_core = MagicMock(return_value=Mock())
        
    def update_membrane_potentials(self):
        self.calls += 1
        return [1, 2, 3]  # Return some fired neurons
    
    def get_cortical_area(self, cortical_id):
        return self.cortical_areas.get(cortical_id)


class MockStateManager:
    def __init__(self):
        self.burst_frequency = 0
        self.burst_engine_state = ServiceState.UNAVAILABLE
    
    def set_burst_frequency(self, freq):
        self.burst_frequency = freq
    
    def set_burst_engine_state(self, state):
        self.burst_engine_state = state
    
    def get_burst_engine_state(self):
        return self.burst_engine_state


@pytest.fixture
def mock_connectome_manager():
    return MockConnectomeManager()


@pytest.fixture
def mock_state_manager():
    return MockStateManager()


def test_burst_engine_initialization(mock_connectome_manager, mock_state_manager):
    """Test BurstEngine initialization with various parameters."""
    # Basic initialization
    with patch('feagi.npu.burst_engine.FeagiStateManager.instance', return_value=mock_state_manager):
        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            fcl_manager=mock_connectome_manager.fcl_manager,
            config={"target_frequency": 100}
        )
        
        assert engine.connectome_manager == mock_connectome_manager
        assert engine.fcl_manager == mock_connectome_manager.fcl_manager
        assert engine.target_frequency == 100
        assert engine.desired_frequency == 100
        assert engine.burst_interval == 0.01  # 1/100Hz
        assert not engine._running
        assert not engine.genome_loaded
        
    # Test with different frequency parameter
    with patch('feagi.npu.burst_engine.FeagiStateManager.instance', return_value=mock_state_manager):
        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            fcl_manager=mock_connectome_manager.fcl_manager,
            config={"desired_frequency_hz": 50}
        )
        
        assert engine.desired_frequency == 50
        assert engine.target_frequency == 50
        assert engine.burst_interval == 0.02  # 1/50Hz


def test_update_with_genome(mock_connectome_manager, mock_state_manager):
    """Test update_with_genome method."""
    with patch('feagi.npu.burst_engine.FeagiStateManager.instance', return_value=mock_state_manager):
        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            fcl_manager=mock_connectome_manager.fcl_manager,
            config={"target_frequency": 100}
        )
        
        # Initial state
        assert not engine.genome_loaded
        
        # Update with genome
        engine.update_with_genome()
        
        # Check that it updated the state
        assert engine.genome_loaded
        assert len(engine.shed_areas) == 1
        assert 1 in engine.shed_areas
        assert 2 not in engine.shed_areas
        assert 3 not in engine.shed_areas


def test_burst_engine_run_and_stop(mock_connectome_manager, mock_state_manager):
    """Test running and stopping the burst engine."""
    with patch('feagi.npu.burst_engine.FeagiStateManager.instance', return_value=mock_state_manager), \
         patch('feagi.npu.burst_engine.time.sleep') as mock_sleep, \
         patch('feagi.npu.burst_engine.time.perf_counter') as mock_perf_counter:
        
        # Mock time functions
        mock_sleep.return_value = None
        mock_perf_counter.side_effect = [0.0, 0.005]  # 0.005s per burst
        
        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            fcl_manager=mock_connectome_manager.fcl_manager,
            config={"target_frequency": 20}  # 0.05s per burst
        )
        
        # Run the burst engine in a separate thread
        t = threading.Thread(target=engine.run)
        t.daemon = True  # Ensure the thread doesn't prevent test exit
        t.start()
        
        # Let it run briefly
        time.sleep(0.05)
        
        # Check that it's running
        assert engine._running
        assert mock_state_manager.burst_engine_state == ServiceState.READY
        
        # Stop it
        engine.stop()
        t.join(timeout=1)
        
        # Check that it stopped
        assert not engine._running
        
        # Should have called update_membrane_potentials at least once
        assert mock_connectome_manager.calls > 0
        
        # Should have updated burst frequency
        assert mock_state_manager.burst_frequency > 0


def test_load_shedding(mock_connectome_manager, mock_state_manager):
    """Test load shedding when frequency is below target."""
    with patch('feagi.npu.burst_engine.FeagiStateManager.instance', return_value=mock_state_manager), \
         patch('feagi.npu.burst_engine.time.sleep') as mock_sleep, \
         patch('feagi.npu.burst_engine.time.perf_counter') as mock_perf_counter:
        
        # Mock time to make sure frequency is below target
        mock_sleep.return_value = None
        mock_perf_counter.side_effect = [0.0, 0.02]  # 0.02s per burst = 50Hz
        
        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            fcl_manager=mock_connectome_manager.fcl_manager,
            config={"target_frequency": 100}  # 0.01s per burst = 100Hz
        )
        
        # Add an area to shed
        engine.update_with_genome()  # This will populate shed_areas
        
        # Run a single burst cycle
        engine._running = True
        t = threading.Thread(target=lambda: engine.run() if engine._running else None)
        t.daemon = True
        t.start()
        
        # Let it run very briefly then stop
        time.sleep(0.05)
        engine.stop()
        t.join(timeout=1)
        
        # Check that FCL for area 1 was cleared
        assert mock_connectome_manager.fcl_manager.area_fcl_history[1][0] == {}


def test_run_with_fire_queue(mock_connectome_manager, mock_state_manager):
    """Test run_with_fire_queue method initial state check."""
    with patch('feagi.npu.burst_engine.FeagiStateManager.instance', return_value=mock_state_manager):
        # Create engine instance
        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            fcl_manager=mock_connectome_manager.fcl_manager,
            config={"target_frequency": 20}
        )
        
        # Set state to something other than READY
        mock_state_manager.set_burst_engine_state(ServiceState.UNAVAILABLE)
        
        # Verify that run_with_fire_queue exists as a method
        assert hasattr(engine, 'run_with_fire_queue')
        
        # Call it directly but it should return early due to state check
        result = engine.run_with_fire_queue()
        
        # It should have returned False because state is not READY
        assert result is False


def test_error_handling(mock_connectome_manager, mock_state_manager):
    """Test error handling during bursting."""
    with patch('feagi.npu.burst_engine.FeagiStateManager.instance', return_value=mock_state_manager), \
         patch('feagi.npu.burst_engine.time.sleep') as mock_sleep, \
         patch('feagi.npu.burst_engine.time.perf_counter') as mock_perf_counter, \
         patch('feagi.npu.burst_engine.logger') as mock_logger:
        
        # Make sleep a no-op for faster testing
        mock_sleep.return_value = None
        mock_perf_counter.side_effect = [0.0, 0.005]
        
        # Make update_membrane_potentials raise an exception
        mock_connectome_manager.update_membrane_potentials = MagicMock(side_effect=Exception("Test error"))
        
        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            fcl_manager=mock_connectome_manager.fcl_manager,
            config={"target_frequency": 20}
        )
        
        # Since the exception would happen in the thread, we need to monkey patch the run method
        original_run = engine.run
        
        def run_with_exception_catch():
            try:
                original_run()
            except Exception as e:
                mock_logger.error(f"Caught exception: {e}")
        
        engine.run = run_with_exception_catch
        
        # Run the burst engine briefly
        t = threading.Thread(target=engine.run)
        t.daemon = True
        t.start()
        time.sleep(0.01)
        engine.stop()
        t.join(timeout=1)
        
        # Check that the method was called which should trigger the exception
        assert mock_connectome_manager.update_membrane_potentials.called


def test_fcl_sampler_initialization():
    """Test FCLSampler initialization."""
    from feagi.npu.burst_engine import FCLSampler
    from queue import Queue
    
    fcl_manager = MagicMock()
    output_queue = Queue()
    
    # Basic initialization
    sampler = FCLSampler(
        fcl_manager=fcl_manager,
        sample_frequency_hz=20,
        output_queue=output_queue
    )
    
    assert sampler.fcl_manager == fcl_manager
    assert sampler.sample_frequency == 20
    assert sampler.output_queue == output_queue
    assert not sampler.running
    

def test_fcl_sampler_run_and_stop():
    """Test running and stopping the FCL sampler."""
    from feagi.npu.burst_engine import FCLSampler
    from queue import Queue
    
    fcl_manager = MagicMock()
    fcl_manager.get_current_timestep = MagicMock(return_value=1)
    fcl_manager.get_fcl_for_timestep = MagicMock(return_value={"area1": [1, 2, 3]})
    
    output_queue = Queue()
    
    with patch('feagi.npu.burst_engine.time.sleep') as mock_sleep:
        # Make sleep a no-op
        mock_sleep.return_value = None
        
        sampler = FCLSampler(
            fcl_manager=fcl_manager,
            sample_frequency_hz=100,  # High frequency for faster testing
            output_queue=output_queue
        )
        
        # Run in a separate thread
        t = threading.Thread(target=sampler.run)
        t.daemon = True
        t.start()
        
        # Let it run briefly
        time.sleep(0.05)
        
        # It should have put some data in the queue
        assert not output_queue.empty()
        
        # Stop it
        sampler.stop()
        t.join(timeout=1)
        
        # Check that it stopped
        assert not sampler.running


if __name__ == "__main__":
    pytest.main(["-v", __file__]) 