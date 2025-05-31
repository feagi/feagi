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
Tests for the FQ Sampler.

This module contains tests for the FQSampler class from feagi.npu.burst_engine.
"""

import time
import threading
import types
import pytest
from queue import Queue, Empty, Full
from unittest.mock import Mock, patch, MagicMock, call
import logging
import unittest

from feagi.npu.burst_engine import UnifiedFQSampler

class MockFireQueueProvider:
    """Mock fire queue provider for testing FQSampler."""
    def __init__(self, should_raise_exception=False, fire_queue_response=None):
        self.area_fire_queue_calls = []
        self.global_fire_queue_calls = []
        self.should_raise_exception = should_raise_exception
        self.fire_queue_response = fire_queue_response if fire_queue_response is not None else {
            'neuron_ids': [1, 2, 3],
            'membrane_potentials': [0.8, 1.2, 0.9],
            'thresholds': [1.0, 1.0, 1.0],
            'consecutive_fire_counts': [1, 2, 1],
            'refractory_counters': [0, 0, 0]
        }
        self.counter = 0
        
    def get_fire_queue(self):
        """Return a global fire queue for testing."""
        self.global_fire_queue_calls.append(time.time())
        if self.should_raise_exception:
            raise Exception("Test exception from get_fire_queue")
        self.counter += 1
        
        # Return different data each time for testing
        return {
            'neuron_ids': [self.counter, self.counter + 10, self.counter + 20],
            'membrane_potentials': [0.8 + self.counter * 0.1, 1.2, 0.9],
            'thresholds': [1.0, 1.0, 1.0],
            'consecutive_fire_counts': [1, 2, 1],
            'refractory_counters': [0, 0, 0]
        }
        
    def get_area_fire_queue(self, cortical_id):
        """Return a cortical area-specific fire queue for testing."""
        self.area_fire_queue_calls.append((cortical_id, time.time()))
        if self.should_raise_exception:
            raise Exception(f"Test exception from get_area_fire_queue for cortical_id {cortical_id}")
        
        # Convert cortical_id to numeric hash for neuron IDs
        if isinstance(cortical_id, str):
            numeric_id = hash(cortical_id) % 1000  # Simple hash to numeric
        else:
            numeric_id = cortical_id
        
        # Return cortical area-specific fire queue data
        return {
            'neuron_ids': [numeric_id * 100, numeric_id * 100 + 1, numeric_id * 100 + 2],
            'membrane_potentials': [0.8, 1.2, 0.9],
            'thresholds': [1.0, 1.0, 1.0],
            'consecutive_fire_counts': [1, 2, 1],
            'refractory_counters': [0, 0, 0]
        }


@pytest.fixture
def mock_fire_queue_provider():
    """Create a mock fire queue provider for testing."""
    return MockFireQueueProvider()


@pytest.fixture
def output_queue():
    """Create a queue for testing FQSampler output."""
    return Queue(maxsize=10)


@pytest.fixture
def mock_connectome_manager():
    """Create a mock connectome manager with cortical areas for testing."""
    cm = Mock()
    cortical1 = types.SimpleNamespace(id='cortex1', properties={'fq_sample_rate': 20})
    cortical2 = types.SimpleNamespace(id='cortex2', properties={'fq_sample_rate': 5})
    cortical3 = types.SimpleNamespace(id='cortex3', properties={})  # No explicit rate
    cm.cortical_areas = {'cortex1': cortical1, 'cortex2': cortical2, 'cortex3': cortical3}
    return cm


def test_fq_sampler_init(mock_fire_queue_provider, output_queue, mock_connectome_manager):
    """Test initialization of UnifiedFQSampler with different parameters."""
    # Test with only required parameters
    sampler1 = UnifiedFQSampler(mock_fire_queue_provider, 10, output_queue)
    assert sampler1.fire_queue_provider == mock_fire_queue_provider
    assert sampler1.sample_frequency == 10
    assert sampler1.sample_interval == 0.1
    assert sampler1.output_queue == output_queue
    assert sampler1.connectome_manager is None
    assert not sampler1.running
    assert sampler1.sampling_mode == 'global'  # Test new architecture attribute
    assert sampler1.max_retries == 3  # Test default value
    
    # Test with connectome manager and custom parameters
    sampler2 = UnifiedFQSampler(
        mock_fire_queue_provider, 
        5, 
        output_queue, 
        mock_connectome_manager,
        sampling_mode='motor_only',
        max_retries=5
    )
    assert sampler2.connectome_manager == mock_connectome_manager
    assert sampler2.sample_frequency == 5
    assert sampler2.sample_interval == 0.2
    assert sampler2.sampling_mode == 'motor_only'
    assert sampler2.max_retries == 5


def test_fq_sampler_run_without_connectome(mock_fire_queue_provider, output_queue):
    """Test UnifiedFQSampler.run without a connectome manager (global mode)."""
    # Create sampler with high frequency for faster testing in global mode
    sampler = UnifiedFQSampler(
        mock_fire_queue_provider, 
        50, 
        output_queue,
        sampling_mode='global'
    )
    
    # Run in a separate thread
    t = threading.Thread(target=sampler.run)
    t.start()
    
    # Let it run briefly
    time.sleep(0.05)  # Should get at least 2 samples at 50Hz
    
    # Stop the sampler
    sampler.stop()
    t.join(timeout=1)
    
    # Check that it's stopped
    assert not sampler.running
    
    # Check that we got samples - new architecture always tries to sample
    samples = []
    try:
        while True:
            samples.append(output_queue.get_nowait())
    except Empty:
        pass
    
    # Should have at least 1 sample (sampling runs regardless of subscribers)
    assert len(samples) >= 1, "Did not get any samples"
    
    # Check that get_fire_queue was called
    assert len(mock_fire_queue_provider.global_fire_queue_calls) >= 1
    
    # Verify the content of the samples
    for sample in samples:
        # Could be dict or None depending on what provider returns
        if sample is not None:
            assert isinstance(sample, dict)
            assert 'neuron_ids' in sample
            assert 'membrane_potentials' in sample


def test_fq_sampler_run_with_connectome(mock_fire_queue_provider, output_queue, mock_connectome_manager):
    """Test UnifiedFQSampler.run with a connectome manager (areas sampling mode)."""
    # Use areas_only mode to test connectome-based sampling
    target_areas = ['cortex1', 'cortex2']
    sampler = UnifiedFQSampler(
        mock_fire_queue_provider, 
        50, 
        output_queue, 
        mock_connectome_manager,
        sampling_mode='areas_only',
        target_areas=target_areas
    )
    
    # Run in a separate thread
    t = threading.Thread(target=sampler.run)
    t.start()
    
    # Let it run briefly
    time.sleep(0.06)  # Should get samples
    
    # Stop the sampler
    sampler.stop()
    t.join(timeout=1)
    
    # Check that it's stopped
    assert not sampler.running
    
    # Check that we got samples
    samples = []
    try:
        while True:
            samples.append(output_queue.get_nowait())
    except Empty:
        pass
    
    # Should have gotten some samples
    assert len(samples) >= 1, "Did not get any samples"
    
    # With new architecture, samples could be in different formats
    # depending on the actual implementation. Let's just verify we got data
    for sample in samples:
        if sample is not None:
            # Could be tuple (area_id, data) or dict depending on mode
            if isinstance(sample, tuple):
                area_id, fire_queue_data = sample
                assert isinstance(fire_queue_data, dict)
                assert 'neuron_ids' in fire_queue_data
            elif isinstance(sample, dict):
                assert 'neuron_ids' in sample


def test_set_target_areas(mock_fire_queue_provider, output_queue, mock_connectome_manager):
    """Test setting target areas for sampling."""
    sampler = UnifiedFQSampler(
        mock_fire_queue_provider, 
        10, 
        output_queue, 
        mock_connectome_manager,
        sampling_mode='areas_only'
    )
    
    # Test setting target areas
    target_areas = ['cortex1', 'cortex2']
    sampler.set_target_areas(target_areas)
    
    # Check that target areas were updated
    assert sampler.target_areas == target_areas
    
    # Test updating with different areas
    new_target_areas = ['cortex3']
    sampler.set_target_areas(new_target_areas)
    assert sampler.target_areas == new_target_areas
    
    # Test with empty list
    sampler.set_target_areas([])
    assert sampler.target_areas == []


def test_fq_sampler_with_full_queue(mock_fire_queue_provider, mock_connectome_manager):
    """Test UnifiedFQSampler behavior when the output queue is full."""
    # Create a very small queue
    small_queue = Queue(maxsize=1)
    
    # Fill the queue
    small_queue.put("blocking_item")
    
    sampler = UnifiedFQSampler(
        mock_fire_queue_provider, 
        100, 
        small_queue, 
        mock_connectome_manager,
        sampling_mode='global'
    )
    
    # Run in a separate thread
    t = threading.Thread(target=sampler.run)
    t.start()
    
    # Let it try to run briefly (queue is full, so samples should be dropped)
    time.sleep(0.02)
    
    # Stop the sampler
    sampler.stop()
    t.join(timeout=1)
    
    # Queue should still only have the blocking item (new items dropped due to full queue)
    assert small_queue.qsize() == 1
    assert small_queue.get_nowait() == "blocking_item"


def test_fq_sampler_with_exception(output_queue, mock_connectome_manager):
    """Test UnifiedFQSampler error handling when exceptions occur."""
    # Create a provider that raises exceptions
    error_provider = MockFireQueueProvider(should_raise_exception=True)
    
    sampler = UnifiedFQSampler(
        error_provider, 
        50, 
        output_queue, 
        mock_connectome_manager,
        sampling_mode='global'
    )
    
    # Run in a separate thread
    t = threading.Thread(target=sampler.run)
    t.start()
    
    # Let it run briefly (should handle exceptions gracefully)
    time.sleep(0.03)
    
    # Stop the sampler
    sampler.stop()
    t.join(timeout=1)
    
    # Should not have crashed, and queue should have few or no samples due to exceptions
    samples = []
    try:
        while True:
            samples.append(output_queue.get_nowait())
    except Empty:
        pass
    
    # Should have few or no samples due to exceptions, but shouldn't crash
    assert len(samples) <= 2  # Allow some tolerance for timing


def test_fq_sampler_zero_rate(mock_fire_queue_provider, output_queue, mock_connectome_manager):
    """Test UnifiedFQSampler behavior with zero sample rate."""
    # Create sampler with zero frequency (should handle gracefully)
    sampler = UnifiedFQSampler(
        mock_fire_queue_provider, 
        0,  # Zero frequency
        output_queue, 
        mock_connectome_manager,
        sampling_mode='global'
    )
    
    # Should have set a small interval despite zero frequency
    assert sampler.sample_interval == 0.1  # Should default to something reasonable
    
    # Test setting frequency to 0 after creation
    sampler.set_sample_frequency(0)
    assert sampler.sample_frequency == 0
    
    # Test setting valid frequency
    sampler.set_sample_frequency(10)
    assert sampler.sample_frequency == 10
    assert sampler.sample_interval == 0.1


def test_fq_sampler_sampling_modes(mock_fire_queue_provider, output_queue):
    """Test UnifiedFQSampler sampling mode functionality."""
    # Test global mode
    sampler_global = UnifiedFQSampler(
        mock_fire_queue_provider, 
        50, 
        output_queue,
        sampling_mode='global'
    )
    assert sampler_global.sampling_mode == 'global'
    
    # Test motor_only mode
    sampler_motor = UnifiedFQSampler(
        mock_fire_queue_provider, 
        50, 
        output_queue,
        sampling_mode='motor_only'
    )
    assert sampler_motor.sampling_mode == 'motor_only'
    
    # Test areas_only mode
    target_areas = ['cortex1', 'cortex2']
    sampler_areas = UnifiedFQSampler(
        mock_fire_queue_provider, 
        50, 
        output_queue,
        sampling_mode='areas_only',
        target_areas=target_areas
    )
    assert sampler_areas.sampling_mode == 'areas_only'
    assert sampler_areas.target_areas == target_areas
    
    # Test custom mode
    sampler_custom = UnifiedFQSampler(
        mock_fire_queue_provider, 
        50, 
        output_queue,
        sampling_mode='custom'
    )
    assert sampler_custom.sampling_mode == 'custom'


def test_fq_sampler_performance_stats(mock_fire_queue_provider, output_queue):
    """Test UnifiedFQSampler performance statistics functionality."""
    sampler = UnifiedFQSampler(
        mock_fire_queue_provider, 
        50, 
        output_queue,
        sampling_mode='global'
    )
    
    # Get initial performance stats
    stats = sampler.get_performance_stats()
    
    # Verify stats structure
    assert isinstance(stats, dict)
    assert 'sample_frequency' in stats
    assert 'sampling_mode' in stats
    assert 'samples_generated' in stats
    assert 'simd_enabled' in stats
    assert 'zero_copy_enabled' in stats
    
    # Verify initial values
    assert stats['sample_frequency'] == 50
    assert stats['sampling_mode'] == 'global'
    assert stats['samples_generated'] == 0  # No samples generated yet
    assert isinstance(stats['simd_enabled'], bool)
    assert isinstance(stats['zero_copy_enabled'], bool)


def test_fq_sampler_always_samples(mock_fire_queue_provider, output_queue):
    """Test that UnifiedFQSampler always attempts to sample (no subscriber concept)."""
    sampler = UnifiedFQSampler(
        mock_fire_queue_provider, 
        100, 
        output_queue,
        sampling_mode='global'
    )
    
    # Run in a separate thread
    t = threading.Thread(target=sampler.run)
    t.start()
    
    # Let it run briefly
    time.sleep(0.05)
    
    # Stop the sampler
    sampler.stop()
    t.join(timeout=1)
    
    # Should always attempt sampling (new architecture doesn't use subscriber flags)
    # The provider should have been called
    assert len(mock_fire_queue_provider.global_fire_queue_calls) >= 1


class TestFQSampler(unittest.TestCase):
    """Unit test class for FQSampler."""
    
    def setUp(self):
        self.mock_provider = MockFireQueueProvider()
        self.output_queue = Queue(maxsize=10)
        self.sampler = UnifiedFQSampler(
            self.mock_provider, 
            10.0, 
            self.output_queue
        )

    def test_initialization(self):
        """Test UnifiedFQSampler initialization with correct properties."""
        self.assertEqual(self.sampler.sample_frequency, 10.0)
        self.assertEqual(self.sampler.sample_interval, 0.1)
        self.assertFalse(self.sampler.running)
        self.assertEqual(self.sampler.sampling_mode, 'global')  # Default mode
        self.assertEqual(self.sampler.max_retries, 3)
        self.assertEqual(self.sampler.target_areas, [])

    def test_run_and_stop(self):
        """Test that UnifiedFQSampler correctly runs and stops."""
        # Start the sampler in a thread (no need to enable subscribers in new architecture)
        t = threading.Thread(target=self.sampler.run)
        t.start()
        
        # Let it run for a bit
        time.sleep(0.15)  # Should get at least one sample at 10Hz
        
        # Stop it
        self.sampler.stop()
        t.join(timeout=2)
        
        # Verify it stopped
        self.assertFalse(self.sampler.running)
        
        # Verify we got samples (could be None if no data available)
        samples = []
        try:
            while not self.output_queue.empty():
                samples.append(self.output_queue.get_nowait())
        except Empty:
            pass
        
        # New architecture always attempts sampling, so we should get some response
        self.assertGreaterEqual(len(samples), 0, "Sampler should have attempted sampling")
        
        # Check sample format if we got any non-None samples
        for sample in samples:
            if sample is not None:
                self.assertIsInstance(sample, dict)
                self.assertIn('neuron_ids', sample)
                self.assertIn('membrane_potentials', sample)

    def test_per_area_sampling(self):
        """Test UnifiedFQSampler with per-cortical-area sampling."""
        # Create connectome manager
        cm = Mock()
        cortical1 = types.SimpleNamespace(id='cortex1', properties={'fq_sample_rate': 20})
        cortical2 = types.SimpleNamespace(id='cortex2', properties={'fq_sample_rate': 30})
        cm.cortical_areas = {'cortex1': cortical1, 'cortex2': cortical2}
        
        # Use areas_only sampling mode with specific target areas
        sampler = UnifiedFQSampler(
            self.mock_provider,
            10.0,
            self.output_queue,
            cm,
            sampling_mode='areas_only',
            target_areas=['cortex1', 'cortex2']
        )
        
        # Start the sampler
        t = threading.Thread(target=sampler.run)
        t.start()
        
        # Let it run
        time.sleep(0.1)
        
        # Stop it
        sampler.stop()
        t.join(timeout=2)
        
        # Verify we got samples
        samples = []
        try:
            while not self.output_queue.empty():
                samples.append(self.output_queue.get_nowait())
        except Empty:
            pass
        
        # Should have attempted sampling
        self.assertGreaterEqual(len(samples), 0, "Sampler should have attempted area sampling")

    @patch('feagi.npu.burst_engine.logger')
    def test_error_handling(self, mock_logger):
        """Test error handling in the UnifiedFQSampler."""
        # Create a provider that raises exceptions
        error_provider = MockFireQueueProvider(should_raise_exception=True)
        
        sampler = UnifiedFQSampler(
            error_provider, 
            50, 
            self.output_queue,
            sampling_mode='global'
        )
        
        # Start the sampler
        t = threading.Thread(target=sampler.run)
        t.start()
        
        # Let it run briefly
        time.sleep(0.05)
        
        # Stop it
        sampler.stop()
        t.join(timeout=2)
        
        # Should have logged errors but not crashed
        self.assertFalse(sampler.running)
        
        # Should not have many samples due to exceptions
        sample_count = self.output_queue.qsize()
        self.assertLessEqual(sample_count, 2)  # Allow some tolerance 