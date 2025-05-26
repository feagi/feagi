import unittest
from unittest.mock import MagicMock, Mock, patch
import time
import threading
from queue import Queue, Empty

from feagi.npu.burst_engine import FQSampler


class TestFQSampler(unittest.TestCase):
    
    def setUp(self):
        # Create mock fire queue provider
        self.mock_provider = MagicMock()
        self.mock_provider.get_fire_queue.return_value = {
            'neuron_ids': [1, 2, 3],
            'membrane_potentials': [0.8, 1.2, 0.9],
            'thresholds': [1.0, 1.0, 1.0],
            'consecutive_fire_counts': [1, 2, 1],
            'refractory_counters': [0, 0, 0]
        }
        self.mock_provider.get_area_fire_queue.return_value = {
            'neuron_ids': [10, 20, 30],
            'membrane_potentials': [0.7, 1.1, 0.95],
            'thresholds': [1.0, 1.0, 1.0],
            'consecutive_fire_counts': [1, 1, 2],
            'refractory_counters': [0, 0, 0]
        }
        
        # Create output queue
        self.output_queue = Queue(maxsize=10)
        
        # Create FQ sampler
        self.fq_sampler = FQSampler(
            self.mock_provider, 
            10.0, 
            self.output_queue
        )

    def test_initialization(self):
        """Test if FQSampler initializes properly."""
        self.assertEqual(self.fq_sampler.sample_frequency, 10.0)
        self.assertEqual(self.fq_sampler.sample_interval, 0.1)
        self.assertFalse(self.fq_sampler.running)
        self.assertEqual(self.fq_sampler._last_sample_time_per_area, {})
        self.assertEqual(self.fq_sampler._max_retries, 3)
        self.assertEqual(self.fq_sampler._retry_delay, 0.001)

    def test_stop(self):
        """Test stopping the FQ sampler."""
        # Set running to True manually
        self.fq_sampler.running = True
        
        # Call stop
        self.fq_sampler.stop()
        
        # Check that running is now False
        self.assertFalse(self.fq_sampler.running)

    def test_update_area_sample_rate(self):
        """Test updating cortical area sample rate."""
        # Initially, _last_sample_time_per_area should be empty
        self.assertEqual(len(self.fq_sampler._last_sample_time_per_area), 0)
        
        # Update cortical area sample rate 
        self.fq_sampler.update_area_sample_rate('cortex1', 30.0)
        
        # Check that _last_sample_time_per_area was updated
        self.assertEqual(len(self.fq_sampler._last_sample_time_per_area), 1)
        self.assertIn('cortex1', self.fq_sampler._last_sample_time_per_area)

    def test_subscriber_flags(self):
        """Test visualization and motor subscriber flags."""
        # Test initial state
        self.assertFalse(self.fq_sampler._has_visualization_subscribers)
        self.assertFalse(self.fq_sampler._has_motor_subscribers)
        
        # Test setting visualization subscribers
        self.fq_sampler.set_visualization_subscribers(True)
        self.assertTrue(self.fq_sampler._has_visualization_subscribers)
        
        self.fq_sampler.set_visualization_subscribers(False)
        self.assertFalse(self.fq_sampler._has_visualization_subscribers)
        
        # Test setting motor subscribers
        self.fq_sampler.set_motor_subscribers(True)
        self.assertTrue(self.fq_sampler._has_motor_subscribers)
        
        self.fq_sampler.set_motor_subscribers(False)
        self.assertFalse(self.fq_sampler._has_motor_subscribers)

    def test_run_without_subscribers(self):
        """Test that run() skips sampling when no subscribers are set."""
        # Don't set any subscribers - both should remain False
        self.assertFalse(self.fq_sampler._has_visualization_subscribers)
        self.assertFalse(self.fq_sampler._has_motor_subscribers)
        
        # Set the sampler to stop after a short time
        def stop_sampler():
            time.sleep(0.05)
            self.fq_sampler.stop()
        
        stop_thread = threading.Thread(target=stop_sampler)
        stop_thread.start()
        
        # Run the sampler
        self.fq_sampler.run()
        
        stop_thread.join()
        
        # Verify no fire queue methods were called
        self.mock_provider.get_fire_queue.assert_not_called()
        self.mock_provider.get_area_fire_queue.assert_not_called()
        
        # Verify output queue is empty
        self.assertTrue(self.output_queue.empty())

    @patch('time.perf_counter')
    def test_run_with_subscribers(self, mock_perf_counter):
        """Test that run() samples when subscribers are set."""
        # Mock time to control sampling timing - provide more values for multiple loop iterations
        mock_perf_counter.side_effect = [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5] * 10
        
        # Set visualization subscribers
        self.fq_sampler.set_visualization_subscribers(True)
        
        # Set the sampler to stop after a short time
        def stop_sampler():
            time.sleep(0.15)
            self.fq_sampler.stop()
        
        stop_thread = threading.Thread(target=stop_sampler)
        stop_thread.start()
        
        # Run the sampler 
        self.fq_sampler.run()
        
        stop_thread.join()
        
        # Verify fire queue method was called
        self.mock_provider.get_fire_queue.assert_called()
        
        # Verify we got samples in output queue
        samples = []
        try:
            while True:
                samples.append(self.output_queue.get_nowait())
        except Empty:
            pass
        
        self.assertGreater(len(samples), 0)

    def test_run_with_connectome_manager(self):
        """Test run() with connectome manager for per-cortical-area sampling."""
        # Create mock connectome manager
        mock_cm = Mock()
        cortical1 = Mock()
        cortical1.id = 'cortex1'
        cortical1.properties = {'fq_sample_rate': 20}
        cortical2 = Mock() 
        cortical2.id = 'cortex2'
        cortical2.properties = {'fq_sample_rate': 30}
        mock_cm.cortical_areas = {'cortex1': cortical1, 'cortex2': cortical2}
        
        # Create sampler with connectome manager
        sampler = FQSampler(self.mock_provider, 10.0, self.output_queue, mock_cm)
        sampler.set_visualization_subscribers(True)
        
        # Set the sampler to stop after a short time
        def stop_sampler():
            time.sleep(0.1)
            sampler.stop()
        
        stop_thread = threading.Thread(target=stop_sampler)
        stop_thread.start()
        
        # Run the sampler
        sampler.run()
        
        stop_thread.join()
        
        # Verify area fire queue method was called
        self.mock_provider.get_area_fire_queue.assert_called()
        
        # Verify we got samples
        samples = []
        try:
            while True:
                samples.append(self.output_queue.get_nowait())
        except Empty:
            pass
        
        self.assertGreater(len(samples), 0)

    def test_fire_queue_data_methods(self):
        """Test the private methods for getting fire queue data."""
        # Test global fire queue data
        global_data = self.fq_sampler._get_global_fire_queue_data()
        self.assertIsNotNone(global_data)
        self.assertIn('neuron_ids', global_data)
        self.assertIn('coordinates', global_data)  # Should add coordinates
        
        # Test cortical area fire queue data
        area_data = self.fq_sampler._get_area_fire_queue_data('cortex1')
        self.assertIsNotNone(area_data)
        self.assertIn('neuron_ids', area_data)
        self.assertIn('coordinates', area_data)  # Should add coordinates

    def test_coordinate_generation(self):
        """Test coordinate generation for neurons."""
        neuron_ids = [1, 2, 3]
        
        # Test global coordinate generation
        coords = self.fq_sampler._get_global_neuron_coordinates(neuron_ids)
        self.assertEqual(len(coords), len(neuron_ids))
        for coord in coords:
            self.assertEqual(len(coord), 3)  # Should be (x, y, z)
            
        # Test cortical area coordinate generation
        coords = self.fq_sampler._get_neuron_coordinates('cortex1', neuron_ids)
        self.assertEqual(len(coords), len(neuron_ids))
        for coord in coords:
            self.assertEqual(len(coord), 3)  # Should be (x, y, z)


if __name__ == '__main__':
    unittest.main() 