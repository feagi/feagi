"""
Unit tests for UnifiedFQSampler in feagi.npu.fq_sampler module.

Tests the new modular architecture without backward compatibility methods.
"""

import unittest
import time
import threading
from queue import Queue, Empty
from unittest.mock import Mock, patch

from feagi.npu.fq_sampler import UnifiedFQSampler


class TestUnifiedFQSampler(unittest.TestCase):
    """Unit test class for UnifiedFQSampler."""
    
    def setUp(self):
        # Create mock fire queue provider
        self.mock_provider = Mock()
        self.mock_provider.get_fire_queue.return_value = {
            'neuron_ids': [1, 2, 3],
            'membrane_potentials': [0.8, 1.2, 0.9],
            'coordinates': [(0, 0, 0), (1, 1, 1), (2, 2, 2)]
        }
        self.mock_provider.get_area_fire_queue.return_value = {
            'neuron_ids': [4, 5, 6],
            'membrane_potentials': [0.7, 1.1, 0.8],
            'coordinates': [(0, 0, 0), (1, 1, 1), (2, 2, 2)]
        }
        
        # Create output queue
        self.output_queue = Queue(maxsize=10)
        
        # Create UnifiedFQSampler
        self.fq_sampler = UnifiedFQSampler(
            self.mock_provider, 
            10.0, 
            self.output_queue,
            sampling_mode='global'
        )

    def test_initialization(self):
        """Test if UnifiedFQSampler initializes properly."""
        self.assertEqual(self.fq_sampler.sample_frequency, 10.0)
        self.assertEqual(self.fq_sampler.sample_interval, 0.1)
        self.assertFalse(self.fq_sampler.running)
        self.assertEqual(self.fq_sampler.sampling_mode, 'global')
        self.assertEqual(self.fq_sampler.max_retries, 3)
        self.assertEqual(self.fq_sampler.target_areas, [])

    def test_stop(self):
        """Test stopping the FQ sampler."""
        # Set running to True manually
        self.fq_sampler.running = True
        
        # Call stop
        self.fq_sampler.stop()
        
        # Check that running is now False
        self.assertFalse(self.fq_sampler.running)

    def test_set_target_areas(self):
        """Test setting target areas for sampling."""
        # Initially, target_areas should be empty
        self.assertEqual(len(self.fq_sampler.target_areas), 0)
        
        # Set target areas
        target_areas = ['cortex1', 'cortex2']
        self.fq_sampler.set_target_areas(target_areas)
        
        # Check that target_areas was updated
        self.assertEqual(self.fq_sampler.target_areas, target_areas)

    def test_sampling_modes(self):
        """Test different sampling modes."""
        # Test global mode
        sampler_global = UnifiedFQSampler(
            self.mock_provider, 10.0, self.output_queue, sampling_mode='global'
        )
        self.assertEqual(sampler_global.sampling_mode, 'global')
        
        # Test motor_only mode
        sampler_motor = UnifiedFQSampler(
            self.mock_provider, 10.0, self.output_queue, sampling_mode='motor_only'
        )
        self.assertEqual(sampler_motor.sampling_mode, 'motor_only')
        
        # Test areas_only mode
        sampler_areas = UnifiedFQSampler(
            self.mock_provider, 10.0, self.output_queue, 
            sampling_mode='areas_only', target_areas=['cortex1']
        )
        self.assertEqual(sampler_areas.sampling_mode, 'areas_only')
        self.assertEqual(sampler_areas.target_areas, ['cortex1'])

    def test_performance_stats(self):
        """Test performance statistics functionality."""
        stats = self.fq_sampler.get_performance_stats()
        
        # Verify stats structure
        self.assertIsInstance(stats, dict)
        self.assertIn('sample_frequency', stats)
        self.assertIn('sampling_mode', stats)
        self.assertIn('samples_generated', stats)
        self.assertIn('simd_enabled', stats)
        self.assertIn('zero_copy_enabled', stats)
        
        # Verify values
        self.assertEqual(stats['sample_frequency'], 10.0)
        self.assertEqual(stats['sampling_mode'], 'global')
        self.assertEqual(stats['samples_generated'], 0)  # No samples yet

    def test_frequency_setting(self):
        """Test setting sample frequency."""
        # Test setting new frequency
        self.fq_sampler.set_sample_frequency(20.0)
        self.assertEqual(self.fq_sampler.sample_frequency, 20.0)
        self.assertEqual(self.fq_sampler.sample_interval, 0.05)
        
        # Test setting zero frequency (should be ignored - keeps previous value)
        self.fq_sampler.set_sample_frequency(0)
        self.assertEqual(self.fq_sampler.sample_frequency, 20.0)  # Should remain unchanged

    def test_run_global_mode(self):
        """Test run() in global sampling mode."""
        # Set the sampler to stop after a short time
        def stop_sampler():
            time.sleep(0.15)
            self.fq_sampler.stop()
        
        stop_thread = threading.Thread(target=stop_sampler)
        stop_thread.start()
        
        # Run the sampler
        self.fq_sampler.run()
        
        stop_thread.join()
        
        # Verify fire queue method was called (new architecture always attempts sampling)
        self.mock_provider.get_fire_queue.assert_called()

    def test_run_with_connectome_manager(self):
        """Test run() with connectome manager."""
        # Create mock connectome manager
        mock_cm = Mock()
        mock_cm.cortical_areas = {
            'cortex1': Mock(id='cortex1', properties={'fq_sample_rate': 20}),
            'cortex2': Mock(id='cortex2', properties={'fq_sample_rate': 30})
        }
        
        # Create sampler with connectome manager in areas_only mode
        sampler = UnifiedFQSampler(
            self.mock_provider, 10.0, self.output_queue, mock_cm,
            sampling_mode='areas_only', target_areas=['cortex1', 'cortex2']
        )
        
        # Set the sampler to stop after a short time
        def stop_sampler():
            time.sleep(0.1)
            sampler.stop()
        
        stop_thread = threading.Thread(target=stop_sampler)
        stop_thread.start()
        
        # Run the sampler
        sampler.run()
        
        stop_thread.join()
        
        # Should have attempted sampling
        self.assertFalse(sampler.running)

    def test_direct_sampling(self):
        """Test direct sampling functionality."""
        # Test direct sampling (may return None if no brain data)
        result = self.fq_sampler.sample_direct()
        
        # Result could be None or bytes depending on implementation
        self.assertTrue(result is None or isinstance(result, bytes))

    def test_error_handling(self):
        """Test error handling in the sampler."""
        # Create provider that raises exceptions
        error_provider = Mock()
        error_provider.get_fire_queue.side_effect = Exception("Test error")
        
        sampler = UnifiedFQSampler(error_provider, 50, self.output_queue)
        
        # Set the sampler to stop after a short time
        def stop_sampler():
            time.sleep(0.05)
            sampler.stop()
        
        stop_thread = threading.Thread(target=stop_sampler)
        stop_thread.start()
        
        # Run the sampler - should handle exceptions gracefully
        sampler.run()
        
        stop_thread.join()
        
        # Should not have crashed
        self.assertFalse(sampler.running)


if __name__ == '__main__':
    unittest.main() 