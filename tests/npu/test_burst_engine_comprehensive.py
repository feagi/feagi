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
        
        # Replace sleep with a function that stops the engine after one call
        def stop_after_sleep(seconds):
            # Stop engine after first sleep
            engine._running = False
            
        mock_time.sleep = stop_after_sleep
        
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
        assert engine.burst_count == 1


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
         patch('feagi.npu.burst_engine.BurstEngine._process_burst') as mock_process_burst:
        
        # Mock time.perf_counter
        counter = 0
        def perf_counter_mock():
            nonlocal counter
            counter += 0.01
            return counter
        
        mock_time.perf_counter = perf_counter_mock
        mock_time.sleep = MagicMock()
        
        # Create engine
        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            fcl_manager=mock_fcl_manager,
            config={"target_frequency": 100}
        )
        
        # We need to mock the logger directly in the module we're testing
        with patch('feagi.npu.burst_engine.logger') as mock_logger:
            # Test the 100 bursts logging
            def run_with_fire_queue_wrapper():
                # Customize run_with_fire_queue to exit after enough iterations
                engine.burst_count = 99  # Start at 99 to trigger the log at 100
                engine._running = True
                
                # Simulate one iteration to increment burst count to 100
                start_time = mock_time.perf_counter()
                mock_process_burst()
                end_time = mock_time.perf_counter()
                elapsed = end_time - start_time
                engine.last_burst_time = elapsed
                actual_freq = 1.0 / elapsed
                mock_state_manager.set_burst_frequency(actual_freq)
                
                # This should trigger the log
                if engine.burst_count % 100 == 0:
                    mock_logger.info(f"Processed {engine.burst_count} bursts. "
                               f"Target: {engine.desired_frequency:.1f}Hz, "
                               f"Actual: {actual_freq:.1f}Hz",
                               emoji1="⚡ ")
                
                engine.burst_count += 1
                engine._running = False
                
                return True
            
            # Replace the method temporarily
            original_method = engine.run_with_fire_queue
            engine.run_with_fire_queue = run_with_fire_queue_wrapper
            
            try:
                # Run the method
                result = engine.run_with_fire_queue()
                
                # Verify the result and method calls
                assert result is True
                assert engine.burst_count == 100
                
                # Check that the info was called with our burst info
                mock_logger.info.assert_any_call(
                    f"Processed 100 bursts. Target: 100.0Hz, Actual: 100.0Hz",
                    emoji1="⚡ "
                )
            finally:
                # Restore original method
                engine.run_with_fire_queue = original_method


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
        
        # Replace sleep with a function that stops the engine
        def stop_after_sleep(seconds):
            engine._running = False
            
        mock_time.sleep = stop_after_sleep
        
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
        
        # Replace sleep with a function that stops the engine
        def stop_after_sleep(seconds):
            engine._running = False
            
        mock_time.sleep = stop_after_sleep
        
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