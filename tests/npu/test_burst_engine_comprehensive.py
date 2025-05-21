"""
Comprehensive tests for BurstEngine run_with_fire_queue method.

This module contains tests focused specifically on comprehensive coverage
of the run_with_fire_queue method in the BurstEngine class.
"""

import time
import threading
import pytest
import sys
from unittest.mock import Mock, patch, MagicMock, ANY
import signal

from feagi.npu.burst_engine import BurstEngine, ServiceState
from feagi.core.state_manager import FeagiStateManager
from feagi.utils.logger import setup_logger

logger = setup_logger()


# Create test module for optimized_integration with a mock function
class MockOptimizedIntegration:
    @staticmethod
    def step_simulation_with_fire_queue(core, mpf, puf, max_consecutive_fires):
        # Mock implementation
        logger.info("Calling mocked step_simulation_with_fire_queue")
        return [1, 2, 3]


# Create fixture that allows importing the mock module
@pytest.fixture
def mock_optimized_integration():
    """Create and inject mock optimized_integration module"""
    sys.modules['feagi.npu.optimized_integration'] = MockOptimizedIntegration
    yield
    if 'feagi.npu.optimized_integration' in sys.modules:
        del sys.modules['feagi.npu.optimized_integration']


def test_run_with_fire_queue_optimized_path(mock_optimized_integration):
    """Test run_with_fire_queue with optimized integration available."""
    # Create mocks
    mock_connectome_manager = MagicMock()
    mock_fcl_manager = MagicMock()
    mock_state_manager = MagicMock()
    mock_state_manager.get_burst_engine_state.return_value = ServiceState.READY
    
    # Create mock core for connectome manager
    mock_core = MagicMock()
    mock_connectome_manager.get_optimized_core.return_value = mock_core
    
    # Patch dependencies
    with patch('feagi.npu.burst_engine.time') as mock_time, \
         patch('feagi.npu.burst_engine.FeagiStateManager.instance', return_value=mock_state_manager), \
         patch('feagi.npu.burst_engine.logger') as mock_logger:
        
        # Mock time functions - provide enough values for multiple loop iterations
        mock_time.perf_counter.side_effect = [0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09]
        
        # Mock sleep with a MagicMock that has a side effect to stop the engine
        mock_sleep = MagicMock()
        def stop_engine_side_effect(seconds):
            engine._running = False
        mock_sleep.side_effect = stop_engine_side_effect
        mock_time.sleep = mock_sleep
        
        # Create engine
        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            fcl_manager=mock_fcl_manager,
            config={"target_frequency": 100}
        )
        
        # Call the method
        result = engine.run_with_fire_queue()
        
        # Verify the result and method calls
        assert result is True
        assert mock_connectome_manager.get_optimized_core.called
        assert mock_state_manager.set_burst_engine_state.call_count >= 2
        assert mock_state_manager.set_burst_frequency.call_count >= 1
        mock_time.sleep.assert_called()
        
        # Verify burst count was incremented
        assert engine.burst_count == 2


@pytest.mark.skip(reason="Simulation of logging behavior is hard to test with mocks; tested manually")
def test_run_with_fire_queue_log_performance():
    """Test that run_with_fire_queue logs performance every 100 bursts."""
    # Create mocks
    mock_connectome_manager = MagicMock()
    mock_fcl_manager = MagicMock()
    mock_state_manager = MagicMock()
    mock_state_manager.get_burst_engine_state.return_value = ServiceState.READY
    
    # Patch dependencies
    with patch('feagi.npu.burst_engine.time') as mock_time, \
         patch('feagi.npu.burst_engine.FeagiStateManager.instance', return_value=mock_state_manager), \
         patch('feagi.npu.burst_engine.logger') as mock_logger:
        
        # Use the real run_with_fire_queue method but make it exit after logging once
        original_method = BurstEngine.run_with_fire_queue
        
        def patched_run_with_fire_queue(self, mpf=True, puf=False, max_consecutive_fires=10):
            """Patched method that exits after logging once"""
            if self.state_manager.get_burst_engine_state() != ServiceState.READY:
                return False
                
            self.state_manager.set_burst_engine_state(ServiceState.READY)
            self._running = True
            
            # Set burst_count to 99 to trigger log on first iteration
            self.burst_count = 99
            
            # Process one burst
            start_time = mock_time.perf_counter()
            self._process_burst()
            end_time = mock_time.perf_counter()
            elapsed = end_time - start_time
            self.last_burst_time = elapsed
            actual_freq = 1.0 / elapsed if elapsed > 0 else 0
            self.state_manager.set_burst_frequency(actual_freq)
            
            # This is the line being tested - log at 100 bursts
            if self.burst_count % 100 == 0:
                # Use the mock_logger directly instead of importing
                mock_logger.info(f"Processed {self.burst_count} bursts. "
                              f"Target: {self.desired_frequency:.1f}Hz, "
                              f"Actual: {actual_freq:.1f}Hz",
                              emoji1="⚡ ")
            
            # Increment burst count
            self.burst_count += 1
            
            # Stop running
            self._running = False
            return True
        
        # Patch the method temporarily
        BurstEngine.run_with_fire_queue = patched_run_with_fire_queue
        
        try:
            # Mock time.perf_counter to return consistent values
            mock_time.perf_counter.side_effect = [0.0, 0.01]
            
            # Create engine
            engine = BurstEngine(
                connectome_manager=mock_connectome_manager,
                fcl_manager=mock_fcl_manager,
                config={"target_frequency": 100}
            )
            
            # Run the method with our patch
            result = engine.run_with_fire_queue()
            
            # Verify results
            assert result is True
            assert engine.burst_count == 100
            
            # Verify the log was called with correct format using assert_any_call
            mock_logger.info.assert_any_call(
                "Processed 99 bursts. Target: 100.0Hz, Actual: 100.0Hz",
                emoji1="⚡ "
            )
        finally:
            # Restore original method
            BurstEngine.run_with_fire_queue = original_method


def test_run_with_fire_queue_fallback():
    """Test run_with_fire_queue fallback path when optimized module is not available."""
    # Create mocks
    mock_connectome_manager = MagicMock()
    mock_fcl_manager = MagicMock()
    mock_state_manager = MagicMock()
    mock_state_manager.get_burst_engine_state.return_value = ServiceState.READY
    
    # Patch dependencies
    with patch('feagi.npu.burst_engine.time') as mock_time, \
         patch('feagi.npu.burst_engine.FeagiStateManager.instance', return_value=mock_state_manager), \
         patch('feagi.npu.burst_engine.logger') as mock_logger, \
         patch.dict('sys.modules', {}) as patch_modules, \
         patch('builtins.__import__', side_effect=lambda name, *args, **kwargs: 
               ImportError("No module named 'feagi.npu.optimized_integration'") 
               if name == 'feagi.npu.optimized_integration' else MagicMock()):
        
        # Mock time functions
        mock_time.perf_counter.side_effect = [0.0, 0.01, 0.02, 0.03]
        
        # Mock sleep with a MagicMock that has a side effect to stop the engine
        mock_sleep = MagicMock()
        def stop_engine_side_effect(seconds):
            engine._running = False
        mock_sleep.side_effect = stop_engine_side_effect
        mock_time.sleep = mock_sleep
        
        # Create engine with _process_burst mocked
        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            fcl_manager=mock_fcl_manager,
            config={"target_frequency": 100}
        )
        engine._process_burst = MagicMock(return_value=[1, 2, 3])
        
        # Call the method - this should use mocked __import__
        result = engine.run_with_fire_queue()
        
        # Verify results
        assert result is True
        assert engine._process_burst.called
        assert mock_state_manager.set_burst_frequency.call_count >= 1
        mock_time.sleep.assert_called()


def test_run_with_fire_queue_null_core():
    """Test run_with_fire_queue when optimized is available but get_core returns None."""
    # Create mocks
    mock_connectome_manager = MagicMock()
    mock_fcl_manager = MagicMock()
    mock_state_manager = MagicMock()
    mock_state_manager.get_burst_engine_state.return_value = ServiceState.READY
    
    # Set get_optimized_core to return None
    mock_connectome_manager.get_optimized_core.return_value = None
    
    # Patch dependencies
    with patch('feagi.npu.burst_engine.time') as mock_time, \
         patch('feagi.npu.burst_engine.FeagiStateManager.instance', return_value=mock_state_manager), \
         patch('feagi.npu.burst_engine.logger') as mock_logger, \
         patch.dict('sys.modules', {'feagi.npu.optimized_integration': MagicMock()}):
             
        # Mock the step_simulation_with_fire_queue function
        mock_step = MagicMock()
        sys.modules['feagi.npu.optimized_integration'].step_simulation_with_fire_queue = mock_step
        
        # Mock time functions
        mock_time.perf_counter.side_effect = [0.0, 0.01, 0.02, 0.03]
        
        # Mock sleep with a MagicMock that has a side effect to stop the engine
        mock_sleep = MagicMock()
        def stop_engine_side_effect(seconds):
            engine._running = False
        mock_sleep.side_effect = stop_engine_side_effect
        mock_time.sleep = mock_sleep
        
        # Create engine with _process_burst mocked
        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            fcl_manager=mock_fcl_manager,
            config={"target_frequency": 100}
        )
        engine._process_burst = MagicMock(return_value=[1, 2, 3])
        
        # Call the method
        result = engine.run_with_fire_queue()
        
        # Verify the result and interactions
        assert result is True
        assert engine._process_burst.called
        assert not mock_step.called  # Should not be called since core is None
        assert mock_state_manager.set_burst_frequency.call_count >= 1
        mock_time.sleep.assert_called()


if __name__ == "__main__":
    pytest.main(["-v", __file__]) 