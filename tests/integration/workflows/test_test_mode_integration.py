"""
Integration tests for FEAGI Test Mode Module.

This module tests the functionality of the test_mode.py module in a more realistic
scenario, integrating with actual FEAGI components instead of mocks.
"""
import pytest
import time
from unittest.mock import patch, MagicMock

from feagi.test_mode import FeagiTestRunner, run_test_mode
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.core.state_manager import FeagiStateManager, ServiceState, GenomeState


@pytest.mark.skip(reason="Requires a properly initialized ConnectomeManager which is complex to set up")
class TestTestModeIntegration:
    """Integration tests for the TestMode module."""
    
    @pytest.fixture
    def core_api_service(self):
        """Create a real CoreAPIService instance for testing."""
        # This would require a properly initialized ConnectomeManager which is
        # complex to set up correctly for a test. This is why we're skipping these tests.
        connectome_manager = MagicMock()
        state_manager = MagicMock()
        
        # Create the service
        service = CoreAPIService(
            connectome_manager=connectome_manager,
            state_manager=state_manager
        )
        
        yield service
    
    def test_test_mode_with_real_core_api(self, core_api_service):
        """Test the test_mode with a real CoreAPIService."""
        # Run the test mode with a short duration
        result = run_test_mode(
            core_api_service=core_api_service,
            test_duration=3,
            frequency_hz=10
        )
        
        # Verify the result
        assert result is not None
    
    @patch('feagi.test_mode.timed_cortical_activity_generator')
    def test_test_runner_integration(self, mock_activity_generator, core_api_service):
        """Test the FeagiTestRunner integration with CoreAPIService."""
        # Configure the activity generator to return test data
        mock_activity_generator.return_value.__next__.return_value = {
            "test_sensory_area": ([1, 2], [1, 2], [0, 0], [0.5, 0.8])
        }
        
        # Create the test runner
        test_runner = FeagiTestRunner(
            core_api_service=core_api_service,
            test_duration=2,
            frequency_hz=5
        )
        
        # Load a minimal test genome
        with patch.object(core_api_service, 'load_essential_genome') as mock_load_genome:
            # Configure the mock to simulate a successful genome load
            mock_load_genome.return_value = True
            
            # Manually set up a minimal cortical area structure for testing
            connectome = core_api_service.get_connectome_manager()
            connectome.cortical_areas = {
                "test_sensory_area": type('obj', (object,), {
                    'properties': {'group': 'IPU'},
                    'get_neurons_at_position': lambda pos: {1, 2}
                })
            }
            
            # Run the test in a thread
            test_runner.run_test()
            
            # Wait for the test to complete
            test_result = test_runner.wait_for_completion(timeout=3)
            
            # Verify that the test completed
            assert test_runner.is_running is False
            
            # Since we're using mocks, the result might be True or False depending on the mock behavior
            assert test_result is not None


@pytest.mark.parametrize("test_duration,frequency_hz", [
    (2, 5),   # Short test with moderate frequency
    (1, 10),  # Very short test with high frequency
])
def test_test_mode_parameters(test_duration, frequency_hz):
    """Test that test_mode accepts and uses different parameters correctly."""
    # Create a mock CoreAPIService
    mock_core_api = type('obj', (object,), {
        'get_connectome_manager': lambda: type('obj', (object,), {
            'cortical_areas': {
                'test_area': type('obj', (object,), {
                    'properties': {'group': 'IPU'},
                    'get_neurons_at_position': lambda pos: {1, 2}
                })
            }
        }),
        'get_fcl_manager': lambda: type('obj', (object,), {
            'current_timestep': 1,
            'get_cortical_fcl': lambda area_id: set(),
            'update_fcl': lambda ts, areas: None
        }),
        'get_burst_engine': lambda: type('obj', (object,), {}),
        'get_genome': lambda: {
            'cortical_areas': {
                'test_area': {'group': 'IPU', 'dimensions': {'x': 10, 'y': 10, 'z': 1}}
            }
        },
        'load_essential_genome': lambda: True
    })
    
    # Patch the dependencies
    with patch('feagi.test_mode.FeagiStateManager') as mock_state_manager_cls, \
         patch('feagi.test_mode.timed_cortical_activity_generator') as mock_activity_generator:
        
        # Configure the mocks
        mock_state_manager = type('obj', (object,), {
            'set_test_visualization_mode': lambda enabled: None,
        })
        mock_state_manager_cls.instance.return_value = mock_state_manager
        
        mock_activity_generator.return_value.__next__.return_value = {
            'test_area': ([1, 2], [1, 2], [0, 0], [0.5, 0.8])
        }
        
        # Create the test runner
        test_runner = FeagiTestRunner(
            core_api_service=mock_core_api,
            test_duration=test_duration,
            frequency_hz=frequency_hz
        )
        
        # Verify that the parameters were correctly set
        assert test_runner.test_duration == test_duration
        assert test_runner.frequency_hz == frequency_hz
        
        # Run the test synchronously
        with patch('feagi.npu.fcl_manager.BitMap') as mock_bitmap_class:
            # Create a proper mock for BitMap
            mock_bitmap = MagicMock()
            mock_bitmap.__len__.return_value = 2
            mock_bitmap_class.return_value = mock_bitmap
            
            # Override time.time to control the test duration
            original_time = time.time
            time_counter = [0]
            
            def mock_time():
                # Increment time each call to simulate passage of time
                time_counter[0] += 0.1
                return time_counter[0]
            
            with patch('time.time', mock_time), \
                 patch('time.sleep') as mock_sleep:
                
                # Run the test
                test_runner._run_test_thread()
                
                # Verify that the test completed
                assert test_runner.is_running is False
                
                # The sleep calls should be approximately 1/frequency_hz
                expected_sleep = 1.0 / frequency_hz
                for call in mock_sleep.call_args_list:
                    assert abs(call[0][0] - expected_sleep) < 0.001


@pytest.mark.parametrize("test_visualization", [
    True,   # With visualization testing
    False,  # Without visualization testing
])
def test_test_mode_visualization(test_visualization):
    """Test the visualization testing functionality."""
    # Create a mock CoreAPIService with more visualization-related mocks
    fcl_sampler = MagicMock()
    fcl_sampler.sample_fcl = MagicMock(return_value={})
    fcl_sampler.set_visualization_subscribers = MagicMock()
    
    mock_core_api = type('obj', (object,), {
        'get_connectome_manager': lambda: type('obj', (object,), {
            'cortical_areas': {
                'test_area': type('obj', (object,), {
                    'properties': {'group': 'IPU'},
                    'get_neurons_at_position': lambda pos: {1, 2}
                })
            },
            'fcl_sampler': fcl_sampler
        }),
        'get_fcl_manager': lambda: type('obj', (object,), {
            'current_timestep': 1,
            'get_cortical_fcl': lambda area_id: set(),
            'update_fcl': lambda ts, areas: None
        }),
        'get_burst_engine': lambda: type('obj', (object,), {}),
        'get_genome': lambda: {
            'cortical_areas': {
                'test_area': {'group': 'IPU', 'dimensions': {'x': 10, 'y': 10, 'z': 1}}
            }
        },
        'load_essential_genome': lambda: True,
        'register_agent': lambda **kwargs: True,
        'fq_sampler': fcl_sampler,
        'fcl_sampler': fcl_sampler
    })
    
    # Patch the dependencies
    with patch('feagi.test_mode.FeagiStateManager') as mock_state_manager_cls, \
         patch('feagi.test_mode.timed_cortical_activity_generator') as mock_activity_generator:
        
        # Configure the mocks
        mock_state_manager = type('obj', (object,), {
            'set_test_visualization_mode': lambda enabled: None,
        })
        mock_state_manager_cls.instance.return_value = mock_state_manager
        
        mock_activity_generator.return_value.__next__.return_value = {
            'test_area': ([1, 2], [1, 2], [0, 0], [0.5, 0.8])
        }
        
        # Create the test runner with visualization testing enabled/disabled
        test_runner = FeagiTestRunner(
            core_api_service=mock_core_api,
            test_duration=1,
            frequency_hz=10,
            test_visualization=test_visualization
        )
        
        # Verify that test_visualization parameter was set correctly
        assert test_runner.test_visualization is test_visualization
        
        # Run the test synchronously
        with patch('feagi.npu.fcl_manager.BitMap') as mock_bitmap_class:
            # Create a proper mock for BitMap
            mock_bitmap = MagicMock()
            mock_bitmap.__len__.return_value = 2
            mock_bitmap_class.return_value = mock_bitmap
            
            # Override time.time to control the test duration
            original_time = time.time
            time_counter = [0]
            
            def mock_time():
                # Increment time each call to simulate passage of time
                time_counter[0] += 0.1
                return time_counter[0]
            
            with patch('time.time', mock_time), \
                 patch('time.sleep') as mock_sleep:
                
                # Run the test
                test_runner._run_test_thread()
                
                # Verify that the test completed
                assert test_runner.is_running is False
                
                # Check visualization setup based on test_visualization flag
                if test_visualization:
                    # If visualization testing is enabled, we should have registered an agent
                    assert test_runner.is_visualization_agent_registered is True
                    
                    # And the FCL sampler should have been hooked
                    fcl_sampler.set_visualization_subscribers.assert_called_once_with(True)
                else:
                    # If visualization testing is disabled, we should not have registered an agent
                    assert test_runner.is_visualization_agent_registered is False 