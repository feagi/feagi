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

from feagi.npu.burst_engine import FQSampler

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
    """Test initialization of FQSampler with different parameters."""
    # Test with only required parameters
    sampler1 = FQSampler(mock_fire_queue_provider, 10, output_queue)
    assert sampler1.fire_queue_provider == mock_fire_queue_provider
    assert sampler1.sample_frequency == 10
    assert sampler1.sample_interval == 0.1
    assert sampler1.output_queue == output_queue
    assert sampler1.connectome_manager is None
    assert not sampler1.running
    assert len(sampler1._last_sample_time_per_area) == 0
    
    # Test with connectome manager
    sampler2 = FQSampler(mock_fire_queue_provider, 5, output_queue, mock_connectome_manager)
    assert sampler2.connectome_manager == mock_connectome_manager
    assert sampler2.sample_frequency == 5
    assert sampler2.sample_interval == 0.2


def test_fq_sampler_run_without_connectome(mock_fire_queue_provider, output_queue):
    """Test FQSampler.run without a connectome manager (global mode)."""
    # Create sampler with high frequency for faster testing
    sampler = FQSampler(mock_fire_queue_provider, 50, output_queue)
    
    # Set retry parameters for faster testing
    sampler._max_retries = 1
    sampler._retry_delay = 0.001
    
    # Enable subscribers for testing
    sampler.set_visualization_subscribers(True)
    
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
    
    # Check that we got samples
    samples = []
    try:
        while True:
            samples.append(output_queue.get_nowait())
    except Empty:
        pass
    
    # Should have at least 1 sample
    assert len(samples) >= 1, "Did not get any samples"
    
    # Check that get_fire_queue was called
    assert len(mock_fire_queue_provider.global_fire_queue_calls) >= 1
    
    # Verify the content of the samples - each sample should be a fire queue dict
    for sample in samples:
        assert isinstance(sample, dict)
        assert 'neuron_ids' in sample
        assert 'membrane_potentials' in sample
        assert len(sample['neuron_ids']) == 3


def test_fq_sampler_run_with_connectome(mock_fire_queue_provider, output_queue, mock_connectome_manager):
    """Test FQSampler.run with a connectome manager (per-area mode)."""
    sampler = FQSampler(mock_fire_queue_provider, 50, output_queue, mock_connectome_manager)
    
    # Enable subscribers for testing
    sampler.set_visualization_subscribers(True)
    
    # Run in a separate thread
    t = threading.Thread(target=sampler.run)
    t.start()
    
    # Let it run briefly
    time.sleep(0.06)  # Should get at least 1 sample for each area
    
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
    
    # Should have at least 3 samples (one for each area)
    assert len(samples) >= 3, "Did not get enough samples"
    
    # Check that all cortical areas were sampled
    cortical_ids = [sample[0] for sample in samples if isinstance(sample, tuple)]
    assert 'cortex1' in cortical_ids, "cortex1 wasn't sampled"
    assert 'cortex2' in cortical_ids, "cortex2 wasn't sampled"
    assert 'cortex3' in cortical_ids, "cortex3 wasn't sampled"
    
    # Check format of samples (should be (cortical_id, fire_queue_data) tuples)
    for sample in samples:
        if isinstance(sample, tuple):
            cortical_id, fire_queue_data = sample
            assert cortical_id in ['cortex1', 'cortex2', 'cortex3']
            assert isinstance(fire_queue_data, dict)
            assert 'neuron_ids' in fire_queue_data
            assert 'membrane_potentials' in fire_queue_data
            assert len(fire_queue_data['neuron_ids']) == 3


def test_update_area_sample_rate(mock_fire_queue_provider, output_queue, mock_connectome_manager):
    """Test updating the sample rate for a specific area."""
    sampler = FQSampler(mock_fire_queue_provider, 10, output_queue, mock_connectome_manager)
    
    # Update rate for cortex1
    sampler.update_area_sample_rate('cortex1', 50.0)
    
    # Check that _last_sample_time_per_area was updated
    assert 'cortex1' in sampler._last_sample_time_per_area
    assert isinstance(sampler._last_sample_time_per_area['cortex1'], float)
    
    # Test updating a non-existent cortical area (should not throw exception)
    sampler.update_area_sample_rate('cortex999', 30.0)
    
    # Test updating without connectome manager (should not throw exception)
    sampler2 = FQSampler(mock_fire_queue_provider, 10, output_queue)
    sampler2.update_area_sample_rate('cortex1', 25.0)


def test_fq_sampler_with_full_queue(mock_fire_queue_provider, mock_connectome_manager):
    """Test FQSampler behavior when the output queue is full."""
    # Create a very small queue
    small_queue = Queue(maxsize=1)
    
    # Fill the queue
    small_queue.put("blocking_item")
    
    sampler = FQSampler(mock_fire_queue_provider, 100, small_queue, mock_connectome_manager)
    sampler.set_visualization_subscribers(True)
    
    # Run in a separate thread
    t = threading.Thread(target=sampler.run)
    t.start()
    
    # Let it try to run briefly (queue is full, so samples should be dropped)
    time.sleep(0.02)
    
    # Stop the sampler
    sampler.stop()
    t.join(timeout=1)
    
    # Queue should still only have the blocking item
    assert small_queue.qsize() == 1
    assert small_queue.get_nowait() == "blocking_item"


def test_fq_sampler_with_exception(output_queue, mock_connectome_manager):
    """Test FQSampler error handling when exceptions occur."""
    # Create a provider that raises exceptions
    error_provider = MockFireQueueProvider(should_raise_exception=True)
    
    sampler = FQSampler(error_provider, 50, output_queue, mock_connectome_manager)
    sampler.set_visualization_subscribers(True)
    
    # Run in a separate thread
    t = threading.Thread(target=sampler.run)
    t.start()
    
    # Let it run briefly (should handle exceptions gracefully)
    time.sleep(0.03)
    
    # Stop the sampler
    sampler.stop()
    t.join(timeout=1)
    
    # Should not have crashed, and queue should be empty or minimal
    samples = []
    try:
        while True:
            samples.append(output_queue.get_nowait())
    except Empty:
        pass
    
    # Should have few or no samples due to exceptions
    assert len(samples) <= 1


def test_fq_sampler_zero_rate(mock_fire_queue_provider, output_queue, mock_connectome_manager):
    """Test FQSampler behavior with zero sample rates."""
    # Set cortical area properties to have zero rate
    mock_connectome_manager.cortical_areas['cortex1'].properties['fq_sample_rate'] = 0
    mock_connectome_manager.cortical_areas['cortex2'].properties['fq_sample_rate'] = 0
    mock_connectome_manager.cortical_areas['cortex3'].properties['fq_sample_rate'] = 0
    
    sampler = FQSampler(mock_fire_queue_provider, 50, output_queue, mock_connectome_manager)
    sampler.set_visualization_subscribers(True)
    
    # Run in a separate thread
    t = threading.Thread(target=sampler.run)
    t.start()
    
    # Let it run briefly
    time.sleep(0.03)
    
    # Stop the sampler
    sampler.stop()
    t.join(timeout=1)
    
    # Should get minimal samples since rates are 0
    samples = []
    try:
        while True:
            samples.append(output_queue.get_nowait())
    except Empty:
        pass
    
    # With zero rates for areas, should get few samples
    assert len(samples) <= 2


def test_fq_sampler_subscriber_flags(mock_fire_queue_provider, output_queue):
    """Test FQSampler subscriber flag functionality."""
    sampler = FQSampler(mock_fire_queue_provider, 50, output_queue)
    
    # Test initial state
    assert not sampler._has_visualization_subscribers
    assert not sampler._has_motor_subscribers
    
    # Test setting visualization subscribers
    sampler.set_visualization_subscribers(True)
    assert sampler._has_visualization_subscribers
    
    sampler.set_visualization_subscribers(False)
    assert not sampler._has_visualization_subscribers
    
    # Test setting motor subscribers
    sampler.set_motor_subscribers(True)
    assert sampler._has_motor_subscribers
    
    sampler.set_motor_subscribers(False)
    assert not sampler._has_motor_subscribers


def test_fq_sampler_no_subscribers_skip(mock_fire_queue_provider, output_queue):
    """Test that FQSampler skips sampling when no subscribers."""
    sampler = FQSampler(mock_fire_queue_provider, 100, output_queue)
    
    # Don't set any subscribers (both should be False)
    assert not sampler._has_visualization_subscribers
    assert not sampler._has_motor_subscribers
    
    # Run in a separate thread
    t = threading.Thread(target=sampler.run)
    t.start()
    
    # Let it run briefly
    time.sleep(0.05)
    
    # Stop the sampler
    sampler.stop()
    t.join(timeout=1)
    
    # Should not have sampled anything since no subscribers
    assert len(mock_fire_queue_provider.global_fire_queue_calls) == 0
    assert len(mock_fire_queue_provider.area_fire_queue_calls) == 0
    
    # Queue should be empty
    assert output_queue.empty()


class TestFQSampler(unittest.TestCase):
    """Unit test class for FQSampler."""
    
    def setUp(self):
        self.mock_provider = MockFireQueueProvider()
        self.output_queue = Queue(maxsize=10)
        self.sampler = FQSampler(
            self.mock_provider, 
            10.0, 
            self.output_queue
        )

    def test_initialization(self):
        """Test FQSampler initialization with correct properties."""
        self.assertEqual(self.sampler.sample_frequency, 10.0)
        self.assertEqual(self.sampler.sample_interval, 0.1)
        self.assertFalse(self.sampler.running)
        self.assertEqual(self.sampler._last_sample_time_per_area, {})
        self.assertEqual(self.sampler._max_retries, 3)
        self.assertEqual(self.sampler._retry_delay, 0.001)

    def test_run_and_stop(self):
        """Test that FQSampler correctly runs and stops."""
        # Enable subscribers
        self.sampler.set_visualization_subscribers(True)
        
        # Start the sampler in a thread
        t = threading.Thread(target=self.sampler.run)
        t.start()
        
        # Let it run for a bit
        time.sleep(0.15)  # Should get at least one sample at 10Hz
        
        # Stop it
        self.sampler.stop()
        t.join(timeout=2)
        
        # Verify it stopped
        self.assertFalse(self.sampler.running)
        
        # Verify we got samples
        samples = []
        try:
            while not self.output_queue.empty():
                samples.append(self.output_queue.get_nowait())
        except Empty:
            pass
        
        self.assertGreater(len(samples), 0, "FQSampler did not produce any samples")
        
        # Check sample format
        for sample in samples:
            self.assertIsInstance(sample, dict)
            self.assertIn('neuron_ids', sample)
            self.assertIn('membrane_potentials', sample)

    def test_per_area_sampling(self):
        """Test FQSampler with per-cortical-area sampling."""
        # Create connectome manager
        cm = Mock()
        cortical1 = types.SimpleNamespace(id='cortex1', properties={'fq_sample_rate': 20})
        cortical2 = types.SimpleNamespace(id='cortex2', properties={'fq_sample_rate': 30})
        cm.cortical_areas = {'cortex1': cortical1, 'cortex2': cortical2}
        
        sampler = FQSampler(
            self.mock_provider,
            10.0,
            self.output_queue,
            cm
        )
        
        sampler.set_visualization_subscribers(True)
        
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
        
        self.assertGreater(len(samples), 0, "FQSampler did not produce any samples")

    @patch('feagi.npu.burst_engine.logger')
    def test_error_handling(self, mock_logger):
        """Test error handling in the FQSampler."""
        # Create a provider that raises exceptions
        error_provider = MockFireQueueProvider(should_raise_exception=True)
        
        sampler = FQSampler(error_provider, 50, self.output_queue)
        sampler.set_visualization_subscribers(True)
        
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
        self.assertLessEqual(sample_count, 1) 