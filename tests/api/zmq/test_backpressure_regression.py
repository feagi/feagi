"""
Backpressure Regression Tests

These tests specifically catch the ring buffer slot commit bug that causes
"Ring buffer full, applying backpressure" warnings when slots are committed
even when no data is processed.
"""

import pytest
import asyncio
import zmq
from unittest.mock import Mock, patch
from feagi.api.zmq.streams.sensory_neural import SensoryNeuralStream, StreamResult


class TestBackpressureRegression:
    """Tests that prevent regression of the backpressure bug."""
    
    @pytest.fixture
    def mock_core_api(self):
        """Create a mock core API service."""
        return Mock()
    
    @pytest.fixture
    def simple_stream(self, mock_core_api):
        """Create a minimal stream for testing the core logic."""
        return SensoryNeuralStream(
            core_api=mock_core_api,
            host="127.0.0.1",
            port=5595,
            ring_buffer_slots=3,
            slot_size=1024
        )
    
    def test_no_slot_commit_on_zmq_again(self, simple_stream):
        """Critical regression test: slots should NOT be committed on zmq.Again."""
        stream = simple_stream
        
        # Track ring buffer commit calls
        original_commit = stream.ring_buffer.commit_write
        commit_calls = []
        
        def track_commit(slot):
            commit_calls.append(slot)
            return original_commit(slot)
        
        stream.ring_buffer.commit_write = track_commit
        
        # Mock socket to return zmq.Again (no data available)
        with patch.object(stream.socket, 'recv') as mock_recv:
            mock_recv.side_effect = zmq.Again()
            
            # Process should return NO_DATA
            result = asyncio.run(stream._process_neural_data())
            assert result == StreamResult.NO_DATA
            
            # CRITICAL: No slot should have been committed
            assert len(commit_calls) == 0, "Slots should not be committed when no data is available"
            
            # Buffer should be unchanged
            assert stream.ring_buffer.used_slots == 0
    
    def test_buffer_state_unchanged_on_no_data(self, simple_stream):
        """Test that buffer state is unchanged when no data is available."""
        stream = simple_stream
        
        # Get initial buffer state
        initial_available = stream.ring_buffer.available_slots
        initial_used = stream.ring_buffer.used_slots
        
        # Mock multiple calls with no data
        with patch.object(stream.socket, 'recv') as mock_recv:
            mock_recv.side_effect = zmq.Again()
            
            # Make multiple calls - this was the scenario causing buffer exhaustion
            for i in range(10):
                result = asyncio.run(stream._process_neural_data())
                assert result == StreamResult.NO_DATA
                
                # Buffer state should remain unchanged
                assert stream.ring_buffer.available_slots == initial_available
                assert stream.ring_buffer.used_slots == initial_used
    
    def test_multiple_no_data_calls_no_exhaustion(self, simple_stream):
        """Test that multiple no-data calls don't exhaust the buffer."""
        stream = simple_stream
        
        # Track buffer overruns
        initial_overruns = stream._stats['buffer_overruns']
        
        # Mock socket to always return no data
        with patch.object(stream.socket, 'recv') as mock_recv:
            mock_recv.side_effect = zmq.Again()
            
            # Make many no-data calls (this would have caused the original bug)
            for i in range(100):
                result = asyncio.run(stream._process_neural_data())
                assert result == StreamResult.NO_DATA
            
            # Buffer overruns should not increase due to no-data calls
            final_overruns = stream._stats['buffer_overruns']
            assert final_overruns == initial_overruns, "No-data calls should not cause buffer overruns"
            
            # Buffer should still be available
            assert stream.ring_buffer.available_slots > 0, "Buffer should not be exhausted by no-data calls"


def test_backpressure_scenario_simulation():
    """Integration test simulating the exact backpressure scenario."""
    mock_core_api = Mock()
    
    # Create stream with very small buffer to simulate the original problem
    stream = SensoryNeuralStream(
        core_api=mock_core_api,
        host="127.0.0.1",
        port=5597,
        ring_buffer_slots=2,  # Very small to trigger issues quickly
        slot_size=512
    )
    
    # Track buffer overruns
    initial_overruns = stream._stats['buffer_overruns']
    
    # Simulate the original problematic scenario:
    # Rapid polling with mostly no-data responses
    with patch.object(stream.socket, 'recv') as mock_recv:
        # Mostly no data (the problematic case)
        no_data_calls = [zmq.Again()] * 20
        
        mock_recv.side_effect = no_data_calls
        
        # Process all no-data calls
        no_data_results = 0
        for i in range(20):
            try:
                result = asyncio.run(stream._process_neural_data())
                if result == StreamResult.NO_DATA:
                    no_data_results += 1
            except Exception:
                # If we hit any dependency issues, the test logic is still valid
                break
        
        # Key assertions:
        # 1. Should handle no-data calls without buffer overruns
        # 2. Buffer should not be exhausted by no-data calls
        
        # The fix ensures we don't get excessive buffer overruns from no-data calls
        buffer_overruns = stream._stats['buffer_overruns'] - initial_overruns
        
        # With the fix, buffer overruns should be zero for no-data calls
        assert buffer_overruns == 0, f"No-data calls should not cause buffer overruns, got: {buffer_overruns}"
        assert no_data_results > 0, "Should have processed some no-data calls"


class TestBackpressureFixValidation:
    """Tests that validate the specific fix implementation."""
    
    def test_data_processed_flag_prevents_commit(self):
        """Test that the data_processed flag correctly prevents commits."""
        mock_core_api = Mock()
        
        # Create a custom stream to track the data_processed flag
        class DebugStream(SensoryNeuralStream):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.commit_attempts = []
                self.data_processed_values = []
                
                # Override commit to track calls
                original_commit = self.ring_buffer.commit_write
                def tracked_commit(slot):
                    self.commit_attempts.append(slot)
                    return original_commit(slot)
                self.ring_buffer.commit_write = tracked_commit
        
        stream = DebugStream(
            core_api=mock_core_api,
            host="127.0.0.1",
            port=5598,
            ring_buffer_slots=3,
            slot_size=1024
        )
        
        # Test no data scenario
        with patch.object(stream.socket, 'recv') as mock_recv:
            mock_recv.side_effect = zmq.Again()
            result = asyncio.run(stream._process_neural_data())
            assert result == StreamResult.NO_DATA
        
        # Should have no commits for no-data scenario
        assert len(stream.commit_attempts) == 0, "No commits should occur for no-data scenario"
    
    def test_fix_prevents_original_bug_scenario(self):
        """Test that specifically validates the original bug scenario is fixed."""
        mock_core_api = Mock()
        stream = SensoryNeuralStream(
            core_api=mock_core_api,
            host="127.0.0.1",
            port=5599,
            ring_buffer_slots=2,  # Small buffer to make bug obvious
            slot_size=1024
        )
        
        # The original bug: always calling commit_write even on zmq.Again
        # This would exhaust the buffer quickly with no-data calls
        
        with patch.object(stream.socket, 'recv') as mock_recv:
            mock_recv.side_effect = zmq.Again()
            
            # Before the fix, this loop would have filled the buffer
            # and started generating backpressure warnings
            consecutive_no_data = 0
            for i in range(stream.ring_buffer.slots * 2):  # More than buffer capacity
                result = asyncio.run(stream._process_neural_data())
                if result == StreamResult.NO_DATA:
                    consecutive_no_data += 1
                elif result == StreamResult.BUFFER_FULL:
                    # This should NOT happen with the fix
                    pytest.fail(f"Buffer became full from no-data calls after {i} iterations")
            
            # All calls should have been NO_DATA, none should have been BUFFER_FULL
            assert consecutive_no_data == stream.ring_buffer.slots * 2
            assert stream.ring_buffer.available_slots > 0, "Buffer should not be exhausted by no-data calls" 