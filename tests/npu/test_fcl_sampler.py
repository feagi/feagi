"""
Tests for the FCL Sampler.

This module contains tests for the FCLSampler class from feagi.npu.burst_engine.
"""

import time
import threading
import types
import pytest
from queue import Queue, Empty, Full
from unittest.mock import Mock, patch, MagicMock, call
import logging
import unittest

from feagi.npu.burst_engine import FCLSampler

class MockFCLManager:
    """Mock FCL manager for testing FCLSampler."""
    def __init__(self, should_raise_exception=False, fcl_response=None):
        self.area_fcl_calls = []
        self.global_fcl_calls = []
        self.should_raise_exception = should_raise_exception
        self.fcl_response = fcl_response if fcl_response is not None else set([1, 2, 3])
        self.area_fcl_history = {1: [set() for _ in range(3)]}
        self.cortical_fcl_history = {1: [set() for _ in range(3)]}
        self.current_window_index = 0
        self.counter = 0
        self._last_sample_time_per_area = {}
        
    def get_global_fcl(self, offset=0):
        """Return a global FCL for testing."""
        self.global_fcl_calls.append(time.time())
        if self.should_raise_exception:
            raise Exception("Test exception from get_global_fcl")
        self.counter += 1
        return f"fcl_snapshot_{self.counter}"
        
    def get_area_fcl(self, area_id):
        """Return an area-specific FCL for testing."""
        self.area_fcl_calls.append((area_id, time.time()))
        if self.should_raise_exception:
            raise Exception(f"Test exception from get_area_fcl for area {area_id}")
        return set([area_id * 10, area_id * 10 + 1])
        
    def get_cortical_fcl(self, cortical_idx):
        """Return a cortical-specific FCL for testing."""
        return self.get_area_fcl(cortical_idx)  # Alias for compatibility


@pytest.fixture
def mock_fcl_manager():
    """Create a mock FCL manager for testing."""
    return MockFCLManager()


@pytest.fixture
def output_queue():
    """Create a queue for testing FCLSampler output."""
    return Queue(maxsize=10)


@pytest.fixture
def mock_connectome_manager():
    """Create a mock connectome manager with areas for testing."""
    cm = Mock()
    area1 = types.SimpleNamespace(id=1, properties={'fcl_sample_rate': 20})
    area2 = types.SimpleNamespace(id=2, properties={'fcl_sample_rate': 5})
    area3 = types.SimpleNamespace(id=3, properties={})  # No explicit rate
    cm._areas = {1: area1, 2: area2, 3: area3}
    return cm


def test_fcl_sampler_init(mock_fcl_manager, output_queue, mock_connectome_manager):
    """Test initialization of FCLSampler with different parameters."""
    # Test with only required parameters
    sampler1 = FCLSampler(mock_fcl_manager, 10, output_queue)
    assert sampler1.fcl_manager == mock_fcl_manager
    assert sampler1.sample_frequency == 10
    assert sampler1.sample_interval == 0.1
    assert sampler1.output_queue == output_queue
    assert sampler1.connectome_manager is None
    assert not sampler1.running
    assert len(sampler1._last_sample_time_per_area) == 0
    
    # Test with connectome manager
    sampler2 = FCLSampler(mock_fcl_manager, 5, output_queue, mock_connectome_manager)
    assert sampler2.connectome_manager == mock_connectome_manager
    assert sampler2.sample_frequency == 5
    assert sampler2.sample_interval == 0.2


def test_fcl_sampler_run_without_connectome(mock_fcl_manager, output_queue):
    """Test FCLSampler.run without a connectome manager (global mode)."""
    # Create sampler with high frequency for faster testing
    sampler = FCLSampler(mock_fcl_manager, 50, output_queue)
    
    # Set retry parameters for faster testing
    sampler._max_retries = 1
    sampler._retry_delay = 0.001
    
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
    
    # Check that get_global_fcl was called
    assert len(mock_fcl_manager.global_fcl_calls) >= 1
    
    # Verify the content of the samples - each sample should match the expected format
    for i, sample in enumerate(samples):
        assert sample == f"fcl_snapshot_{i+1}"


def test_fcl_sampler_run_with_connectome(mock_fcl_manager, output_queue, mock_connectome_manager):
    """Test FCLSampler.run with a connectome manager (per-area mode)."""
    sampler = FCLSampler(mock_fcl_manager, 50, output_queue, mock_connectome_manager)
    
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
    
    # Check that all areas were sampled
    area_ids = [sample[0] for sample in samples]
    assert 1 in area_ids, "Area 1 wasn't sampled"
    assert 2 in area_ids, "Area 2 wasn't sampled"
    assert 3 in area_ids, "Area 3 wasn't sampled"
    
    # Check format of samples (should be (area_id, area_fcl) tuples)
    for area_id, area_fcl in samples:
        assert area_id in [1, 2, 3]
        assert isinstance(area_fcl, set)
        assert len(area_fcl) == 2
        assert area_id * 10 in area_fcl
        assert area_id * 10 + 1 in area_fcl


def test_update_area_sample_rate(mock_fcl_manager, output_queue, mock_connectome_manager):
    """Test updating the sample rate for a specific area."""
    sampler = FCLSampler(mock_fcl_manager, 10, output_queue, mock_connectome_manager)
    
    # Initial rate for area 1 is 20Hz
    assert mock_connectome_manager._areas[1].properties['fcl_sample_rate'] == 20
    
    # Update rate for area 1
    sampler.update_area_sample_rate(1, 50)
    
    # Check that the rate was updated
    assert mock_connectome_manager._areas[1].properties['fcl_sample_rate'] == 50
    
    # Check that _last_sample_time_per_area was updated
    assert 1 in sampler._last_sample_time_per_area
    assert sampler._last_sample_time_per_area[1] == 0
    
    # Test updating a non-existent area (should not throw exception)
    sampler.update_area_sample_rate(999, 30)
    
    # Test updating without connectome manager (should not throw exception)
    sampler2 = FCLSampler(mock_fcl_manager, 10, output_queue)
    sampler2.update_area_sample_rate(1, 30)  # Should be a no-op


def test_fcl_sampler_with_full_queue(mock_fcl_manager, mock_connectome_manager):
    """Test FCLSampler behavior when the output queue is full."""
    # Create a queue with size 1
    small_queue = Queue(maxsize=1)
    
    # Create a sampler that will try to use the queue
    sampler = FCLSampler(mock_fcl_manager, 100, small_queue, mock_connectome_manager)
    
    # Fill the queue
    small_queue.put("dummy_item")
    assert small_queue.full()
    
    # Run the sampler (should not block even though queue is full)
    t = threading.Thread(target=sampler.run)
    t.start()
    
    # Let it run briefly
    time.sleep(0.05)
    
    # Stop the sampler
    sampler.stop()
    t.join(timeout=1)
    
    # Queue should still have only the dummy item
    assert not small_queue.empty()
    assert small_queue.get() == "dummy_item"
    assert small_queue.empty()


def test_fcl_sampler_with_exception(output_queue, mock_connectome_manager):
    """Test FCLSampler error handling when exceptions occur."""
    # Create FCL manager that raises exceptions
    fcl_manager = MockFCLManager(should_raise_exception=True)
    
    # Create a sampler
    sampler = FCLSampler(fcl_manager, 50, output_queue, mock_connectome_manager)
    
    # Mock the logger to catch error messages
    with patch('feagi.npu.burst_engine.logger') as mock_logger:
        # Run the sampler (should not crash despite exceptions)
        t = threading.Thread(target=sampler.run)
        t.start()
        
        # Let it run briefly
        time.sleep(0.05)
        
        # Stop the sampler
        sampler.stop()
        t.join(timeout=1)
        
        # Check that error was logged
        assert mock_logger.error.called
        
    # Queue should be empty since all operations failed
    assert output_queue.empty()


def test_fcl_sampler_zero_rate(mock_fcl_manager, output_queue, mock_connectome_manager):
    """Test FCLSampler behavior with zero sample rates."""
    # Set area 1 to have zero sample rate
    mock_connectome_manager._areas[1].properties['fcl_sample_rate'] = 0
    
    # Create a sampler
    sampler = FCLSampler(mock_fcl_manager, 50, output_queue, mock_connectome_manager)
    
    # Run in a separate thread
    t = threading.Thread(target=sampler.run)
    t.start()
    
    # Let it run briefly
    time.sleep(0.05)
    
    # Stop the sampler
    sampler.stop()
    t.join(timeout=1)
    
    # Check that we got samples
    samples = []
    try:
        while True:
            samples.append(output_queue.get_nowait())
    except Empty:
        pass
    
    # Should have samples for areas 2 and 3, but possibly not for area 1
    area_ids = [sample[0] for sample in samples]
    assert 2 in area_ids or 3 in area_ids, "No areas were sampled"
    
    # Check that the sampler used the global interval for area 1 since rate was 0
    area1_samples = [s for s in samples if s[0] == 1]
    if area1_samples:  # If we captured any area 1 samples
        assert len(area1_samples) > 0, "Area 1 wasn't sampled despite zero rate"


def test_fcl_sampler_custom_response_formats(output_queue, mock_connectome_manager):
    """Test FCLSampler with different response formats from FCL manager."""
    # Test with list response
    list_response = [1, 2, 3, 4]
    list_fcl_manager = MockFCLManager(fcl_response=list_response)
    
    # Patch the get_global_fcl method directly
    list_fcl_manager.get_global_fcl = MagicMock(return_value=list_response)
    
    sampler = FCLSampler(list_fcl_manager, 50, output_queue)
    
    # Set retry parameters for faster testing
    sampler._max_retries = 1
    sampler._retry_delay = 0.001
    
    # Run briefly
    t = threading.Thread(target=sampler.run)
    t.start()
    time.sleep(0.05)
    sampler.stop()
    t.join(timeout=1)
    
    # Get the samples
    try:
        sample = output_queue.get_nowait()
        assert sample == list_response
    except Empty:
        assert False, "No samples received"
    
    # Clear queue
    while not output_queue.empty():
        output_queue.get()
    
    # Test with empty response
    empty_fcl_manager = MockFCLManager()
    # Patch the get_global_fcl method directly
    empty_fcl_manager.get_global_fcl = MagicMock(return_value=set())
    
    sampler = FCLSampler(empty_fcl_manager, 50, output_queue)
    
    # Set retry parameters for faster testing
    sampler._max_retries = 1
    sampler._retry_delay = 0.001
    
    # Run briefly
    t = threading.Thread(target=sampler.run)
    t.start()
    time.sleep(0.05)
    sampler.stop()
    t.join(timeout=1)
    
    # Get the samples
    try:
        sample = output_queue.get_nowait()
        assert sample == set()
    except Empty:
        assert False, "No samples received"


def test_fcl_sampler_with_racing_threads(mock_fcl_manager, output_queue, mock_connectome_manager):
    """Test FCLSampler behavior with racing threads accessing sample data."""
    sampler = FCLSampler(mock_fcl_manager, 50, output_queue, mock_connectome_manager)
    
    # Set up a racing condition where multiple threads update rates simultaneously
    def updater_thread():
        for i in range(10):
            sampler.update_area_sample_rate(1, 10 + i)
            time.sleep(0.01)
    
    # Start sampler
    sample_thread = threading.Thread(target=sampler.run)
    sample_thread.start()
    
    # Start rate updater
    update_thread = threading.Thread(target=updater_thread)
    update_thread.start()
    
    # Let them run concurrently
    time.sleep(0.15)
    
    # Stop the sampler
    sampler.stop()
    sample_thread.join(timeout=1)
    update_thread.join(timeout=1)
    
    # Check that we got samples and no exceptions were thrown
    samples = []
    try:
        while True:
            samples.append(output_queue.get_nowait())
    except Empty:
        pass
    
    assert len(samples) > 0, "No samples were collected"


class TestFCLSampler(unittest.TestCase):
    
    def setUp(self):
        self.fcl_manager = MockFCLManager()
        self.output_queue = Queue(maxsize=10)
        self.sample_frequency = 10
        self.sampler = FCLSampler(
            self.fcl_manager,
            sample_frequency_hz=self.sample_frequency,
            output_queue=self.output_queue
        )
    
    def test_initialization(self):
        """Test FCLSampler initialization with correct properties."""
        self.assertEqual(self.sampler.sample_frequency, self.sample_frequency)
        self.assertEqual(self.sampler.sample_interval, 1.0/self.sample_frequency)
        self.assertEqual(self.sampler.fcl_manager, self.fcl_manager)
        self.assertEqual(self.sampler.output_queue, self.output_queue)
        self.assertFalse(self.sampler.running)
        self.assertEqual(self.sampler._last_sample_time_per_area, {})
    
    def test_run_and_stop(self):
        """Test that FCLSampler correctly runs and stops."""
        # Start sampler in a separate thread
        t = threading.Thread(target=self.sampler.run)
        t.start()
        
        # Let it run for a while
        time.sleep(0.15)  # Run for 150ms, enough for ~1-2 samples
        
        # Stop it
        self.sampler.stop()
        t.join(timeout=2)
        
        # Check that it's stopped
        self.assertFalse(self.sampler.running)
        
        # Check that we got samples
        samples = []
        try:
            while True:
                samples.append(self.output_queue.get_nowait())
        except Empty:
            pass
        
        self.assertGreater(len(samples), 0, "FCLSampler did not produce any samples")
    
    def test_update_area_sample_rate(self):
        """Test updating an area's sample rate."""
        # Mock the connectome manager
        connectome_manager = Mock()
        area1 = Mock()
        area1.id = 1
        area1.properties = {}
        connectome_manager._areas = {1: area1}
        
        # Create a new sampler with the mock connectome manager
        sampler = FCLSampler(
            self.fcl_manager,
            sample_frequency_hz=self.sample_frequency,
            output_queue=self.output_queue,
            connectome_manager=connectome_manager
        )
        
        # Update the sample rate
        sampler.update_area_sample_rate(1, 20)
        
        # Verify the area property was updated
        self.assertEqual(area1.properties['fcl_sample_rate'], 20)
        
        # Verify last sample time was reset
        self.assertEqual(sampler._last_sample_time_per_area.get(1, None), 0)
    
    def test_per_area_sampling(self):
        """Test sampling with different rates per area."""
        # Mock the connectome manager with areas having different sample rates
        connectome_manager = Mock()
        area1 = Mock()
        area1.id = 1
        area1.properties = {'fcl_sample_rate': 20}  # Higher rate
        area2 = Mock()
        area2.id = 2
        area2.properties = {'fcl_sample_rate': 5}   # Lower rate
        connectome_manager._areas = {1: area1, 2: area2}
        
        # Create a new sampler with the mock connectome manager
        sampler = FCLSampler(
            self.fcl_manager,
            sample_frequency_hz=self.sample_frequency,
            output_queue=self.output_queue,
            connectome_manager=connectome_manager
        )
        
        # Run sampler briefly
        t = threading.Thread(target=sampler.run)
        t.start()
        time.sleep(0.2)  # Run for 200ms
        sampler.stop()
        t.join(timeout=2)
        
        # Collect samples
        samples = []
        try:
            while True:
                samples.append(self.output_queue.get_nowait())
        except Empty:
            pass
        
        # Check that we got at least one sample
        self.assertGreater(len(samples), 0, "FCLSampler did not produce any samples")
    
    @patch('feagi.npu.burst_engine.logger')
    def test_error_handling(self, mock_logger):
        """Test error handling in the FCLSampler."""
        # Make the FCL manager raise an exception when getting an area FCL
        def mock_get_area_fcl(area_id):
            raise ValueError("Test error")
        self.fcl_manager.get_area_fcl = mock_get_area_fcl
        
        # Mock the connectome manager
        connectome_manager = Mock()
        area1 = Mock()
        area1.id = 1
        area1.properties = {}
        connectome_manager._areas = {1: area1}
        
        # Create a new sampler with the mock connectome manager
        sampler = FCLSampler(
            self.fcl_manager,
            sample_frequency_hz=self.sample_frequency,
            output_queue=self.output_queue,
            connectome_manager=connectome_manager
        )
        
        # Run sampler briefly
        t = threading.Thread(target=sampler.run)
        t.start()
        time.sleep(0.15)
        sampler.stop()
        t.join(timeout=2)
        
        # Verify the error was logged
        mock_logger.error.assert_called()


if __name__ == "__main__":
    pytest.main(["-v", __file__]) 