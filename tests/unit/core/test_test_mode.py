"""
Unit tests for FEAGI Test Mode Module.

This module tests the functionality of the test_mode.py module, including
the FeagiTestRunner class and run_test_mode function.
"""
import pytest
from unittest.mock import MagicMock, patch, call
import threading
import time

from feagi.test_mode import FeagiTestRunner, run_test_mode
from feagi.core.state_manager import FeagiStateManager


@pytest.fixture
def mock_core_api():
    """Create a mocked CoreAPIService for testing."""
    mock_api = MagicMock()
    
    # Setup the connectome manager mock
    mock_connectome = MagicMock()
    mock_connectome.cortical_areas = {
        "visual_cortex": MagicMock(properties={"group": "IPU"}),
        "motor_cortex": MagicMock(properties={"group": "OPU"}),
        "association_cortex": MagicMock(properties={"group": "CPL"})
    }
    mock_api.get_connectome_manager.return_value = mock_connectome
    
    # Setup FCL manager mock
    mock_fcl = MagicMock()
    mock_fcl.current_timestep = 1
    mock_fcl.get_cortical_fcl.return_value = set()
    mock_api.get_fcl_manager.return_value = mock_fcl
    
    # Setup burst engine mock
    mock_burst = MagicMock()
    mock_api.get_burst_engine.return_value = mock_burst
    
    # Setup genome mock
    mock_api.get_genome.return_value = {
        "cortical_areas": {
            "visual_cortex": {"group": "IPU", "dimensions": {"x": 10, "y": 10, "z": 1}}
        }
    }
    
    # Setup load_essential_genome mock
    mock_api.load_essential_genome.return_value = True
    
    # Setup register_agent mock
    mock_api.register_agent.return_value = True
    
    # Setup FCL sampler mock
    mock_fcl_sampler = MagicMock()
    mock_fcl_sampler.sample_fcl = MagicMock(return_value={})
    mock_api.fcl_sampler = mock_fcl_sampler
    
    return mock_api


@pytest.fixture
def mock_activity_generator():
    """Create a mocked timed_cortical_activity_generator."""
    with patch('feagi.test_mode.timed_cortical_activity_generator') as mock_gen:
        # Configure the generator to return sensory data
        mock_gen.return_value.__next__.return_value = {
            "visual_cortex": ([1, 2], [1, 2], [0, 0], [0.5, 0.8])
        }
        yield mock_gen


@pytest.fixture
def test_runner(mock_core_api):
    """Create a FeagiTestRunner instance for testing."""
    # Patch the FeagiStateManager.instance() method
    with patch('feagi.test_mode.FeagiStateManager') as mock_state_manager_cls:
        mock_state_manager = MagicMock()
        mock_state_manager_cls.instance.return_value = mock_state_manager
        
        # Create the test runner
        runner = FeagiTestRunner(
            core_api_service=mock_core_api,
            test_duration=2,
            frequency_hz=5
        )
        
        yield runner


class TestFeagiTestRunner:
    """Tests for the FeagiTestRunner class."""
    
    def test_initialization(self, test_runner, mock_core_api):
        """Test that the FeagiTestRunner initializes correctly."""
        assert test_runner.core_api == mock_core_api
        assert test_runner.test_duration == 2
        assert test_runner.frequency_hz == 5
        assert test_runner.is_running is False
        assert test_runner.test_result is None
    
    def test_load_genome(self, test_runner, mock_core_api):
        """Test that load_genome calls the correct methods and handles success."""
        result = test_runner.load_genome()
        
        assert result is True
        mock_core_api.load_essential_genome.assert_called_once()
    
    def test_load_genome_failure(self, test_runner, mock_core_api):
        """Test that load_genome handles failures correctly."""
        # Configure the mock to simulate a failure
        with patch.object(test_runner, 'load_genome', return_value=False):
            result = test_runner.load_genome()
            assert result is False
    
    def test_init_sensory_data_generator(self, test_runner, mock_core_api, mock_activity_generator):
        """Test that init_sensory_data_generator initializes correctly."""
        result = test_runner.init_sensory_data_generator()
        
        assert result is True
        mock_core_api.get_genome.assert_called_once()
        mock_activity_generator.assert_called_once()
    
    def test_init_sensory_data_generator_no_genome(self, test_runner, mock_core_api):
        """Test that init_sensory_data_generator handles missing genome data."""
        # Configure the mock to return no genome data
        mock_core_api.get_genome.return_value = None
        
        result = test_runner.init_sensory_data_generator()
        
        assert result is False
        mock_core_api.get_genome.assert_called_once()
    
    def test_capture_initial_state(self, test_runner, mock_core_api):
        """Test that capture_initial_state correctly captures FCL states."""
        # Configure the mock to return specific FCL data
        fcl_manager = mock_core_api.get_fcl_manager.return_value
        fcl_manager.get_cortical_fcl.return_value = {1, 2, 3}
        
        test_runner.capture_initial_state()
        
        # Check that the initial FCLs were captured
        assert "visual_cortex" in test_runner.initial_fcls
        assert "motor_cortex" in test_runner.initial_fcls
        assert "association_cortex" in test_runner.initial_fcls
        assert test_runner.initial_fcls["visual_cortex"] == {1, 2, 3}
        
        # Verify that get_cortical_fcl was called for each cortical area
        expected_calls = [
            call("visual_cortex"),
            call("motor_cortex"),
            call("association_cortex")
        ]
        fcl_manager.get_cortical_fcl.assert_has_calls(expected_calls, any_order=True)
    
    @patch('feagi.npu.fcl_manager.BitMap')
    def test_inject_sensory_data(self, mock_bitmap_class, test_runner, mock_activity_generator):
        """Test that inject_sensory_data correctly processes sensory data."""
        # Create a mock bitmap instance
        mock_bitmap = MagicMock()
        mock_bitmap.__len__.return_value = 2
        mock_bitmap_class.return_value = mock_bitmap
        
        # Setup the mock to return neurons at positions
        connectome = test_runner.connectome
        area = connectome.cortical_areas["visual_cortex"]
        area.get_neurons_at_position.return_value = {1, 2}
        
        # Initialize the sensory data generator
        test_runner.init_sensory_data_generator()
        
        # Call the method
        result = test_runner.inject_sensory_data()
        
        assert result is True
        
        # Verify that neurons were retrieved at the correct positions
        area.get_neurons_at_position.assert_has_calls([
            call((1, 1, 0)),
            call((2, 2, 0))
        ], any_order=True)
        
        # Verify that FCL was updated
        fcl_manager = test_runner.fcl_manager
        fcl_manager.update_fcl.assert_called_once()
    
    def test_check_neural_activity_no_change(self, test_runner):
        """Test that check_neural_activity correctly detects no changes."""
        # Setup initial state
        test_runner.initial_fcls = {
            "visual_cortex": {1, 2, 3},
            "motor_cortex": {4, 5, 6},
            "association_cortex": {7, 8, 9}
        }
        
        # Configure FCL manager to return the same FCLs
        fcl_manager = test_runner.fcl_manager
        fcl_manager.get_cortical_fcl.side_effect = lambda area_id: {
            "visual_cortex": {1, 2, 3},
            "motor_cortex": {4, 5, 6},
            "association_cortex": {7, 8, 9}
        }.get(area_id, set())
        
        # Call the method
        changed, active_areas = test_runner.check_neural_activity()
        
        assert changed is False
        assert active_areas == []
    
    def test_check_neural_activity_with_change(self, test_runner):
        """Test that check_neural_activity correctly detects changes."""
        # Setup initial state
        test_runner.initial_fcls = {
            "visual_cortex": {1, 2, 3},
            "motor_cortex": {4, 5, 6},
            "association_cortex": {7, 8, 9}
        }
        
        # Configure FCL manager to return different FCLs
        fcl_manager = test_runner.fcl_manager
        fcl_manager.get_cortical_fcl.side_effect = lambda area_id: {
            "visual_cortex": {1, 2, 3},  # No change
            "motor_cortex": {4, 5, 6, 10},  # Changed
            "association_cortex": {7, 8}  # Changed
        }.get(area_id, set())
        
        # Call the method
        changed, active_areas = test_runner.check_neural_activity()
        
        assert changed is True
        assert set(active_areas) == {"motor_cortex", "association_cortex"}
        assert test_runner.areas_with_activity == {"motor_cortex", "association_cortex"}
    
    def test_run_test(self, test_runner):
        """Test that run_test starts the test thread."""
        # Mock the _run_test_thread method
        test_runner._run_test_thread = MagicMock()
        
        # Call the method
        result = test_runner.run_test()
        
        assert result is True
        assert test_runner.test_thread is not None
        assert test_runner.test_thread.daemon is True
        
        # Wait a short time for the thread to start
        time.sleep(0.1)
        
        # Verify that _run_test_thread was called
        test_runner._run_test_thread.assert_called_once()
    
    def test_run_test_already_running(self, test_runner):
        """Test that run_test handles already running tests."""
        # Set the is_running flag
        test_runner.is_running = True
        
        # Call the method
        result = test_runner.run_test()
        
        assert result is False
    
    def test_wait_for_completion(self, test_runner):
        """Test that wait_for_completion waits for the test thread."""
        # Create a mock thread
        mock_thread = MagicMock()
        test_runner.test_thread = mock_thread
        test_runner.test_result = True
        
        # Call the method
        result = test_runner.wait_for_completion(timeout=1)
        
        assert result is True
        mock_thread.join.assert_called_once_with(1)
    
    def test_get_test_result(self, test_runner):
        """Test that get_test_result returns the correct result."""
        # Set the test result
        test_runner.test_result = True
        
        # Call the method
        result = test_runner.get_test_result()
        
        assert result is True
        
    def test_register_visualization_agent(self, test_runner, mock_core_api):
        """Test that register_visualization_agent registers a visualization agent."""
        # Call the method
        result = test_runner.register_visualization_agent()
        
        assert result is True
        assert test_runner.is_visualization_agent_registered is True
        
        # Verify that the agent was registered with the right parameters
        mock_core_api.register_agent.assert_called_once_with(
            agent_id=test_runner.visualization_agent_id,
            agent_type="visualization",
            agent_ip="127.0.0.1",
            agent_data_port=5555,
            agent_version="1.0.0",
            controller_version="1.0.0",
            capabilities={"visualization": True}
        )
    
    def test_hook_fcl_sampler(self, test_runner, mock_core_api):
        """Test that hook_fcl_sampler correctly hooks into the FCL sampler."""
        # Configure mock FCL sampler with set_visualization_subscribers method
        mock_fcl_sampler = mock_core_api.fcl_sampler
        mock_fcl_sampler.set_visualization_subscribers = MagicMock()
        
        # Call the method
        result = test_runner.hook_fcl_sampler()
        
        # Verify the result is True
        assert result is True
        
        # Verify that set_visualization_subscribers was called with True
        mock_fcl_sampler.set_visualization_subscribers.assert_called_once_with(True)


class TestRunTestMode:
    """Tests for the run_test_mode function."""
    
    @patch('feagi.test_mode.FeagiTestRunner')
    def test_run_test_mode_success(self, mock_test_runner_cls, mock_core_api):
        """Test that run_test_mode runs a test and returns the result."""
        # Configure the mock test runner
        mock_test_runner = mock_test_runner_cls.return_value
        mock_test_runner.get_test_result.return_value = True
        
        # Call the function
        result = run_test_mode(mock_core_api, test_duration=5, frequency_hz=10)
        
        # Verify that the test runner was created with the correct parameters
        mock_test_runner_cls.assert_called_once_with(
            core_api_service=mock_core_api,
            sample_genome_path=None,
            test_duration=5,
            frequency_hz=10,
            test_visualization=False
        )
        
        # Verify that the test was run
        mock_test_runner._run_test_thread.assert_called_once()
        
        # Verify that the result was returned
        assert result is True
    
    @patch('feagi.test_mode.FeagiTestRunner')
    def test_run_test_mode_failure(self, mock_test_runner_cls, mock_core_api):
        """Test that run_test_mode handles test failures."""
        # Configure the mock test runner
        mock_test_runner = mock_test_runner_cls.return_value
        mock_test_runner.get_test_result.return_value = False
        
        # Call the function
        result = run_test_mode(mock_core_api)
        
        # Verify that the test runner was created with the correct parameters
        mock_test_runner_cls.assert_called_once_with(
            core_api_service=mock_core_api,
            sample_genome_path=None,
            test_duration=10,
            frequency_hz=10,
            test_visualization=False
        )
        
        # Verify that the test was run
        mock_test_runner._run_test_thread.assert_called_once()
        
        # Verify that the result was returned
        assert result is False
        
    @patch('feagi.test_mode.FeagiTestRunner')
    def test_run_test_mode_with_visualization(self, mock_test_runner_cls, mock_core_api):
        """Test that run_test_mode runs a test with visualization testing."""
        # Configure the mock test runner
        mock_test_runner = mock_test_runner_cls.return_value
        mock_test_runner.get_test_result.return_value = True
        
        # Call the function with visualization testing enabled
        result = run_test_mode(mock_core_api, test_visualization=True)
        
        # Verify that the test runner was created with the correct parameters
        mock_test_runner_cls.assert_called_once_with(
            core_api_service=mock_core_api,
            sample_genome_path=None,
            test_duration=10,
            frequency_hz=10,
            test_visualization=True
        )
        
        # Verify that the test was run
        mock_test_runner._run_test_thread.assert_called_once()
        
        # Verify that the result was returned
        assert result is True 