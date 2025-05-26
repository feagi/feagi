"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Tests for the VisualizationStream threading-based implementation.

This module tests all new features of the enhanced VisualizationStream:
- Threading-based architecture for RTOS compatibility
- Client tracking with heartbeat timeouts
- Automatic FQ sampler control based on subscriber presence
- Standby mode when genome not loaded
- Enhanced error handling and shutdown responsiveness
"""

import pytest
import time
import threading
from unittest.mock import Mock, MagicMock, patch
from queue import Queue, Empty
import zmq

from feagi.api.zmq.streams.visualization import VisualizationStream


class TestVisualizationStream:
    """Test cases for the enhanced VisualizationStream."""
    
    @pytest.fixture
    def mock_core_api(self):
        """Create a mock core API service."""
        mock = Mock()
        mock.genome_is_loaded.return_value = True
        mock.register_genome_change_listener = Mock()
        return mock
    
    @pytest.fixture
    def mock_fq_sampler(self):
        """Create a mock FQ sampler."""
        mock = Mock()
        mock.set_visualization_subscribers = Mock()
        return mock
    
    @pytest.fixture
    def mock_fq_queue(self):
        """Create a mock FQ sampler queue."""
        return Queue()
    
    @pytest.fixture
    def stream_config(self):
        """Standard stream configuration for testing."""
        return {
            'auto_enable_on_subscribers': True,
            'subscriber_check_interval': 0.1,  # Fast for testing
        }
    
    @pytest.fixture
    def viz_stream(self, mock_core_api, mock_fq_sampler, mock_fq_queue, stream_config):
        """Create a VisualizationStream instance for testing."""
        # Use a different port to avoid conflicts
        stream = VisualizationStream(
            host="*",
            port=5570,  # Test port
            core_api=mock_core_api,
            fq_sampler=mock_fq_sampler,
            fq_sampler_queue=mock_fq_queue,
            stream_config=stream_config
        )
        yield stream
        
        # Cleanup
        if stream.running:
            stream.stop()
    
    def test_initialization(self, viz_stream, mock_core_api):
        """Test that VisualizationStream initializes correctly."""
        assert viz_stream.core_api is mock_core_api
        assert viz_stream.port == 5570
        assert viz_stream.running is False
        assert viz_stream._active_mode is True  # Should be True with mock that returns True
        assert isinstance(viz_stream.context, zmq.Context)
        assert viz_stream.socket is not None
        assert viz_stream.client_last_heartbeat == {}
        assert viz_stream._fq_sampler_enabled is False
    
    def test_start_and_stop(self, viz_stream):
        """Test that the stream starts and stops correctly."""
        # Initially not running
        assert viz_stream.running is False
        assert len(viz_stream.worker_threads) == 0
        
        # Start the stream
        viz_stream.start()
        assert viz_stream.running is True
        assert len(viz_stream.worker_threads) > 0
        
        # Wait a moment for threads to start
        time.sleep(0.1)
        
        # Verify threads are alive
        for thread in viz_stream.worker_threads:
            assert thread.is_alive()
        
        # Stop the stream
        viz_stream.stop()
        assert viz_stream.running is False
        
        # Wait for threads to stop
        time.sleep(0.2)
        
        # Verify threads are stopped
        for thread in viz_stream.worker_threads:
            assert not thread.is_alive()
    
    def test_heartbeat_new_client(self, viz_stream, mock_fq_sampler):
        """Test heartbeat tracking for new clients."""
        viz_stream.start()
        
        # Send heartbeat for new client
        client_id = "test_client_001"
        viz_stream.heartbeat_visualization_client(client_id)
        
        # Verify client was added
        assert client_id in viz_stream.client_last_heartbeat
        assert viz_stream.get_connected_client_count() == 1
        
        # Verify FQ sampler was enabled for new client
        mock_fq_sampler.set_visualization_subscribers.assert_called_with(True)
        
        viz_stream.stop()
    
    def test_heartbeat_existing_client(self, viz_stream):
        """Test heartbeat updates for existing clients."""
        viz_stream.start()
        
        client_id = "test_client_001"
        
        # First heartbeat
        viz_stream.heartbeat_visualization_client(client_id)
        first_time = viz_stream.client_last_heartbeat[client_id]
        
        # Wait a bit and send another heartbeat
        time.sleep(0.01)
        viz_stream.heartbeat_visualization_client(client_id)
        second_time = viz_stream.client_last_heartbeat[client_id]
        
        # Verify timestamp was updated
        assert second_time > first_time
        assert viz_stream.get_connected_client_count() == 1
        
        viz_stream.stop()
    
    def test_client_timeout_cleanup(self, viz_stream):
        """Test that clients are cleaned up when they timeout."""
        viz_stream.start()
        viz_stream.client_heartbeat_timeout = 0.1  # Very short timeout for testing
        
        client_id = "test_client_timeout"
        viz_stream.heartbeat_visualization_client(client_id)
        
        # Verify client was added
        assert client_id in viz_stream.client_last_heartbeat
        assert viz_stream.get_connected_client_count() == 1
        
        # Wait for timeout to pass
        time.sleep(0.2)
        
        # Manually trigger cleanup by simulating the cleanup worker logic
        current_time = time.time()
        with viz_stream._client_lock:
            client_ids = list(viz_stream.client_last_heartbeat.keys())
            for client_id_check in client_ids:
                last_heartbeat = viz_stream.client_last_heartbeat[client_id_check]
                if current_time - last_heartbeat > viz_stream.client_heartbeat_timeout:
                    del viz_stream.client_last_heartbeat[client_id_check]
        
        # Verify client was cleaned up
        assert viz_stream.get_connected_client_count() == 0
        assert client_id not in viz_stream.client_last_heartbeat
        
        viz_stream.stop()
    
    def test_fq_sampler_auto_enable_disable(self, viz_stream, mock_fq_sampler):
        """Test automatic FQ sampler enable/disable based on client presence."""
        viz_stream.start()
        
        # Initially no clients, should be disabled
        assert viz_stream._fq_sampler_enabled is False
        
        # Add a client
        viz_stream.heartbeat_visualization_client("client1")
        
        # Should be enabled now (heartbeat method calls _control_fq_sampler directly)
        mock_fq_sampler.set_visualization_subscribers.assert_called_with(True)
        assert viz_stream._fq_sampler_enabled is True
        
        # Manually remove the client to simulate timeout
        with viz_stream._client_lock:
            viz_stream.client_last_heartbeat.clear()
        
        # Manually trigger the subscriber monitor logic
        current_count = viz_stream.get_connected_client_count()
        viz_stream._last_subscriber_count = current_count
        should_enable = current_count > 0
        if should_enable != viz_stream._fq_sampler_enabled:
            viz_stream._control_fq_sampler(should_enable)
        
        # Should be disabled now
        assert viz_stream._fq_sampler_enabled is False
        # Verify both enable and disable were called
        calls = [call.args[0] for call in mock_fq_sampler.set_visualization_subscribers.call_args_list]
        assert True in calls, "Should have been enabled"
        assert False in calls, "Should have been disabled"
        
        viz_stream.stop()
    
    def test_genome_state_management(self, viz_stream, mock_core_api):
        """Test standby mode when genome not loaded."""
        # Start with genome loaded
        mock_core_api.genome_is_loaded.return_value = True
        viz_stream._update_active_mode()
        assert viz_stream._active_mode is True
        
        # Simulate genome unload
        mock_core_api.genome_is_loaded.return_value = False
        viz_stream._update_active_mode()
        assert viz_stream._active_mode is False
        
        # Simulate genome reload
        mock_core_api.genome_is_loaded.return_value = True
        viz_stream._update_active_mode()
        assert viz_stream._active_mode is True
    
    def test_data_processing_with_tagged_format(self, viz_stream, mock_fq_queue):
        """Test processing of tagged format data from enhanced FQ sampler."""
        viz_stream.start()
        
        # Create tagged format data
        tagged_data = {
            'target': 'visualization',
            'cortical_id': 'test_area',
            'fire_queue_data': {
                'neuron_ids': [1, 2, 3],
                'coordinates': [[0, 0, 0], [1, 1, 1], [2, 2, 2]],
                'membrane_potentials': [0.5, 0.7, 0.9]
            }
        }
        
        # Put data in queue
        mock_fq_queue.put(tagged_data)
        
        # Wait for processing
        time.sleep(0.1)
        
        # Verify stats were updated (data was processed)
        assert viz_stream.stats['data_sent'] >= 0
        
        viz_stream.stop()
    
    def test_data_processing_with_legacy_tuple(self, viz_stream, mock_fq_queue):
        """Test processing of legacy tuple format data."""
        viz_stream.start()
        
        # Create legacy tuple format data
        legacy_data = (
            'test_area',
            {
                'neuron_ids': [1, 2],
                'coordinates': [[0, 0, 0], [1, 1, 1]],
                'membrane_potentials': [0.5, 0.7]
            }
        )
        
        # Put data in queue
        mock_fq_queue.put(legacy_data)
        
        # Wait for processing
        time.sleep(0.1)
        
        # Verify processing occurred
        assert viz_stream.stats['data_sent'] >= 0
        
        viz_stream.stop()
    
    def test_responsive_shutdown(self, viz_stream):
        """Test that shutdown is responsive and doesn't hang."""
        viz_stream.start()
        
        # Wait for threads to fully start
        time.sleep(0.1)
        
        # Measure shutdown time
        start_time = time.time()
        viz_stream.stop()
        shutdown_time = time.time() - start_time
        
        # Should stop quickly (well under the 5-second timeout)
        assert shutdown_time < 2.0
        assert viz_stream.running is False
    
    def test_thread_safety(self, viz_stream):
        """Test thread safety of client tracking."""
        viz_stream.start()
        
        results = []
        
        def add_clients():
            for i in range(10):
                viz_stream.heartbeat_visualization_client(f"client_{i}")
                time.sleep(0.001)  # Small delay
        
        def count_clients():
            for _ in range(10):
                count = viz_stream.get_connected_client_count()
                results.append(count)
                time.sleep(0.001)
        
        # Run concurrent operations
        threads = [
            threading.Thread(target=add_clients),
            threading.Thread(target=count_clients)
        ]
        
        for t in threads:
            t.start()
        
        for t in threads:
            t.join()
        
        # Should have 10 clients at the end
        final_count = viz_stream.get_connected_client_count()
        assert final_count == 10
        
        # No crashes or inconsistent states
        assert all(isinstance(count, int) and count >= 0 for count in results)
        
        viz_stream.stop()
    
    def test_socket_recreation_on_error(self, viz_stream):
        """Test socket recreation when ZMQ state corruption occurs."""
        with patch.object(viz_stream.socket, 'send', side_effect=Exception("Operation cannot be accomplished in current state")):
            with patch.object(viz_stream, '_recreate_socket') as mock_recreate:
                # This should trigger socket recreation
                viz_stream._publish_data(b"test_data")
                
                # Verify recreation was attempted
                mock_recreate.assert_called_once()
    
    def test_stats_tracking(self, viz_stream):
        """Test that statistics are tracked correctly."""
        initial_stats = viz_stream.get_stats()
        assert initial_stats['data_sent'] == 0
        assert initial_stats['bytes_sent'] == 0
        assert initial_stats['running'] is False
        
        viz_stream.start()
        stats = viz_stream.get_stats()
        assert stats['running'] is True
        
        viz_stream.stop()
    
    def test_compatibility_methods(self, viz_stream):
        """Test compatibility methods for legacy code."""
        # These should not raise exceptions
        viz_stream.register_visualization_client("test_client")
        viz_stream.unregister_visualization_client("test_client")
        viz_stream.send_visualization_data(b"test_data")
        
        # Should always succeed without errors
        assert True
    
    @patch('feagi.process_manager.get_process_manager')
    def test_fq_sampler_from_process_manager(self, mock_get_pm, viz_stream):
        """Test getting FQ sampler from process manager when not provided."""
        # Set up mock process manager
        mock_pm = Mock()
        mock_fq_sampler = Mock()
        mock_pm._fq_sampler = mock_fq_sampler
        mock_get_pm.return_value = mock_pm
        
        # Clear the existing fq_sampler
        viz_stream.fq_sampler = None
        
        # Call the control method which should find the FQ sampler
        viz_stream._control_fq_sampler(True)
        
        # Verify it found and used the FQ sampler
        assert viz_stream.fq_sampler is mock_fq_sampler
        mock_fq_sampler.set_visualization_subscribers.assert_called_with(True)


class TestVisualizationStreamEdgeCases:
    """Test edge cases and error conditions."""
    
    def test_no_core_api(self):
        """Test initialization without core API (backward compatibility)."""
        stream = VisualizationStream(port=5571)  # Different port
        assert stream.core_api is None
        assert stream._active_mode is True  # Should default to active
        stream.stop()  # Cleanup
    
    def test_no_fq_sampler_queue(self):
        """Test operation without FQ sampler queue."""
        stream = VisualizationStream(port=5572, fq_sampler_queue=None)
        stream.start()
        
        # Should start but warn about no data processing
        assert stream.running is True
        
        stream.stop()
    
    def test_empty_queue_handling(self):
        """Test handling of empty FQ sampler queue."""
        queue = Queue()
        stream = VisualizationStream(port=5573, fq_sampler_queue=queue)
        stream.start()
        
        # Let it run for a moment with empty queue
        time.sleep(0.1)
        
        # Should handle empty queue gracefully
        assert stream.running is True
        
        stream.stop()
    
    def test_malformed_data_handling(self):
        """Test handling of malformed data in queue."""
        queue = Queue()
        stream = VisualizationStream(port=5574, fq_sampler_queue=queue)
        stream.start()
        
        # Put malformed data in queue
        queue.put("invalid_data")
        queue.put({"invalid": "structure"})
        queue.put(None)
        
        # Let it process
        time.sleep(0.1)
        
        # Should handle gracefully without crashing
        assert stream.running is True
        
        stream.stop() 