import unittest
from unittest.mock import MagicMock, patch
import queue
import time

from feagi.npu.burst_engine import FCLSampler


class TestFCLSampler(unittest.TestCase):
    
    def setUp(self):
        # Create mock objects
        self.mock_fcl_manager = MagicMock()
        self.mock_connectome_manager = MagicMock()
        self.output_queue = queue.Queue(maxsize=10)
        
        # Configure mock connectome manager
        self.mock_connectome_manager._areas = {
            1: MagicMock(id=1, properties={}),
            2: MagicMock(id=2, properties={'fcl_sample_rate': 20.0})
        }
        
        # Create FCL sampler
        self.fcl_sampler = FCLSampler(
            fcl_manager=self.mock_fcl_manager,
            sample_frequency_hz=10.0,
            output_queue=self.output_queue,
            connectome_manager=self.mock_connectome_manager
        )
    
    def test_initialization(self):
        """Test if FCLSampler initializes properly."""
        # Check if attributes are set correctly
        self.assertEqual(self.fcl_sampler.sample_frequency, 10.0)
        self.assertEqual(self.fcl_sampler.sample_interval, 0.1)
        self.assertFalse(self.fcl_sampler.running)
        self.assertEqual(self.fcl_sampler._last_sample_time_per_area, {})
        self.assertEqual(self.fcl_sampler._max_retries, 3)
        self.assertEqual(self.fcl_sampler._retry_delay, 0.01)
    
    def test_stop(self):
        """Test stopping the FCL sampler."""
        # Set running state to True
        self.fcl_sampler.running = True
        
        # Call stop method
        self.fcl_sampler.stop()
        
        # Check if running flag is set to False
        self.assertFalse(self.fcl_sampler.running)
    
    def test_update_area_sample_rate(self):
        """Test updating sample rate for a specific area."""
        # Call the method
        self.fcl_sampler.update_area_sample_rate(1, 30.0)
        
        # Check if property was updated
        self.assertEqual(self.mock_connectome_manager._areas[1].properties['fcl_sample_rate'], 30.0)
        
        # Check if last sample time was reset
        self.assertEqual(self.fcl_sampler._last_sample_time_per_area[1], 0)
    
    @patch('feagi.npu.burst_engine.time.sleep', MagicMock())
    @patch('feagi.npu.burst_engine.time.perf_counter')
    def test_run_with_per_area_sampling(self, mock_perf_counter):
        """Test running FCL sampler with per-area sampling."""
        # Configure mock perf_counter to simulate time progression
        mock_perf_counter.side_effect = [1.0, 1.0]
        
        # Configure mock fcl_manager.get_area_fcl to return test data
        self.mock_fcl_manager.get_area_fcl.side_effect = lambda area_id: {
            1: [101, 102],
            2: [201, 202]
        }.get(area_id, [])
        
        # Create a flag to stop the loop after one iteration
        def side_effect(*args, **kwargs):
            self.fcl_sampler.running = False
            return 1.2
        
        # Patch time.perf_counter to set running to False after first call
        with patch('feagi.npu.burst_engine.time.perf_counter', side_effect=side_effect):
            # Run the sampler
            self.fcl_sampler.run()
        
        # Check if get_area_fcl was called for each area
        self.assertEqual(self.mock_fcl_manager.get_area_fcl.call_count, 2)
        
        # Check if items were put in the queue
        self.assertEqual(self.output_queue.qsize(), 2)
        
        # Check queue contents
        item1 = self.output_queue.get()
        self.assertEqual(item1[0], 1)  # area_id
        self.assertEqual(item1[1], [101, 102])  # fcl
        
        item2 = self.output_queue.get()
        self.assertEqual(item2[0], 2)  # area_id
        self.assertEqual(item2[1], [201, 202])  # fcl
    
    @patch('feagi.npu.burst_engine.time.sleep', MagicMock())
    @patch('feagi.npu.burst_engine.time.perf_counter')
    def test_run_with_global_sampling(self, mock_perf_counter):
        """Test running FCL sampler with global sampling (no connectome_manager)."""
        # Create sampler without connectome manager
        fcl_sampler = FCLSampler(
            fcl_manager=self.mock_fcl_manager,
            sample_frequency_hz=10.0,
            output_queue=self.output_queue,
            connectome_manager=None
        )
        
        # Configure mock perf_counter to simulate time progression
        mock_perf_counter.side_effect = [1.0, 1.0]
        
        # Configure mock fcl_manager.get_global_fcl to return test data
        self.mock_fcl_manager.get_global_fcl.return_value = [1001, 1002]
        
        # Create a flag to stop the loop after one iteration
        def side_effect(*args, **kwargs):
            fcl_sampler.running = False
            return 1.2
        
        # Patch time.perf_counter to set running to False after first call
        with patch('feagi.npu.burst_engine.time.perf_counter', side_effect=side_effect):
            # Run the sampler
            fcl_sampler.run()
        
        # Check if get_global_fcl was called
        self.mock_fcl_manager.get_global_fcl.assert_called_once()
        
        # Check if item was put in the queue
        self.assertEqual(self.output_queue.qsize(), 1)
        
        # Check queue content
        item = self.output_queue.get()
        self.assertEqual(item, [1001, 1002])
    
    @patch('feagi.npu.burst_engine.time.sleep', MagicMock())
    @patch('feagi.npu.burst_engine.logger')
    def test_run_with_error_handling(self, mock_logger):
        """Test error handling in FCL sampler."""
        # Configure mock fcl_manager.get_area_fcl to raise an exception
        self.mock_fcl_manager.get_area_fcl.side_effect = Exception("Test error")
        
        # Configure mock output_queue.put_nowait to raise an exception
        self.output_queue = MagicMock()
        self.output_queue.put_nowait.side_effect = Exception("Queue error")
        self.fcl_sampler.output_queue = self.output_queue
        
        # Create a flag to stop the loop after testing error conditions
        def side_effect(*args, **kwargs):
            self.fcl_sampler.running = False
            return None
        
        # Patch time.perf_counter to set running to False after first call
        with patch('feagi.npu.burst_engine.time.sleep', side_effect=side_effect):
            # Run the sampler
            self.fcl_sampler.run()
        
        # Check if appropriate error was logged
        mock_logger.error.assert_called()


if __name__ == '__main__':
    unittest.main() 