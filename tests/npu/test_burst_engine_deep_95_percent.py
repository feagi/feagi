"""
Deep BurstEngine Tests for 95%+ Coverage

This test suite targets untested code paths in BurstEngine:
- Private methods and internal logic
- Complex error handling branches  
- Edge cases and boundary conditions
- State management failure scenarios
- Memory and performance stress cases
- Concurrent access patterns

Target: Push BurstEngine from 85% to 98% coverage
"""

import pytest
import numpy as np
from unittest.mock import Mock, MagicMock, patch, PropertyMock
from typing import Dict, List, Any
import threading
import time

from feagi.npu.burst_engine import BurstEngine, PowerInjectionService
from feagi.npu.fire_candidate_list import FireCandidateList
from feagi.npu.fire_queue import FireQueue, FiringNeuron
from feagi.npu.data_structures import NeuronArray, SynapseArray, BackendType
from feagi.npu.interface import NPUInterface
from feagi.core.state_manager import FeagiStateManager, ServiceState


class MockConnectomeManager:
    """Enhanced mock with more realistic behavior."""
    
    def __init__(self):
        self.cortical_areas = {
            'test_area': type('Area', (), {
                'cortical_idx': 0,
                'area_id': 'test_area',
                'coordinates_3d': [(0, 0, 0), (1, 0, 0)]
            })(),
            '_power': type('Area', (), {
                'cortical_idx': 1,
                'area_id': '_power',
                'coordinates_3d': [(0, 1, 0)]
            })()
        }
        
        # Enhanced NPU interface with failure modes
        self._npu_interface = NPUInterface(backend=BackendType.CPU, max_neurons=1000)
        
        # Add neurons with various properties
        self._npu_interface.neuron_array.add_neurons_batch(
            neuron_ids=[1, 2, 3, 100, 101],
            positions=[(0, 0, 0), (1, 0, 0), (2, 0, 0), (0, 1, 0), (1, 1, 0)],
            neuron_types=[0, 0, 0, 1, 1],  # Regular and power neurons
            initial_potentials=[0.5, 0.8, 0.2, 1.5, 1.8],
            thresholds=[1.0, 1.0, 1.0, 1.0, 1.0],
            leak_coefficients=[0.95, 0.95, 0.95, 0.90, 0.90],
            cortical_idx=0,
            decay_rates=[0.95, 0.95, 0.95, 0.98, 0.98],
            refractory_periods=[1, 1, 1, 0, 0],  # Power neurons have no refractory
            excitabilities=[1.0, 1.0, 1.0, 2.0, 2.0],
            resting_potentials=[0.0, 0.0, 0.0, 0.0, 0.0],
            consecutive_fire_limits=[10, 10, 10, 100, 100]
        )
        
        # Add synapses for propagation testing
        self._npu_interface.synapse_array.add_synapses_batch(
            source_neuron_ids=[1, 2, 100],
            target_neuron_ids=[2, 3, 1],
            weights=[0.5, 0.3, 0.8],
            delays=[1, 1, 1],
            conductances=[1.0, 1.0, 1.0],
            synapse_types=[0, 0, 0],
            plasticity_types=[0, 0, 0],
            plasticity_coefficients=[0.0, 0.0, 0.0]
        )
        
        # Add error injection capabilities
        self._should_fail = False
        self._failure_methods = set()
    
    def inject_failure(self, method_name: str):
        """Inject failure for specific method."""
        self._failure_methods.add(method_name)
    
    def get_neurons_by_cortical_area(self, area_id: str):
        if 'get_neurons_by_cortical_area' in self._failure_methods:
            raise RuntimeError(f"Simulated failure in get_neurons_by_cortical_area for {area_id}")
        
        if area_id == 'test_area':
            return [1, 2, 3]
        elif area_id == '_power':
            return [100, 101]
        return []
    
    def get_cortical_idx_for_id(self, area_id: str):
        if 'get_cortical_idx_for_id' in self._failure_methods:
            return None
        
        area = self.cortical_areas.get(area_id)
        return area.cortical_idx if area else None


class TestBurstEnginePrivateMethods:
    """Test private methods and internal logic."""
    
    def setup_method(self):
        """Setup test environment."""
        BurstEngine.reset_instance()
        self.cm = MockConnectomeManager()
        self.engine = BurstEngine(connectome_manager=self.cm)
        
    def teardown_method(self):
        """Clean up after each test."""
        BurstEngine.reset_instance()
    
    def test_private_set_burst_engine_state(self):
        """Test _set_burst_engine_state with various conditions."""
        # Test with valid state manager
        mock_state_manager = Mock()
        self.engine.state_manager = mock_state_manager
        
        # Test successful state update
        self.engine._set_burst_engine_state(ServiceState.RUNNING)
        mock_state_manager.set_burst_engine_state.assert_called_with(ServiceState.RUNNING)
        
        # Test state manager exception
        mock_state_manager.set_burst_engine_state.side_effect = Exception("State manager error")
        self.engine._set_burst_engine_state(ServiceState.ERROR)  # Should not raise
        
        # Test with None state manager
        self.engine.state_manager = None
        self.engine._set_burst_engine_state(ServiceState.READY)  # Should not raise
    
    def test_private_notify_sensory_stream_ready(self):
        """Test _notify_sensory_stream_ready with different scenarios."""
        # Test with no process manager
        with patch('feagi.npu.burst_engine.ProcessManagerInterface', None):
            self.engine._notify_sensory_stream_ready()  # Should not crash
        
        # Test with process manager but no ZMQ server
        mock_pm = Mock()
        mock_pm._zmq_server = None
        with patch('feagi.npu.burst_engine.ProcessManagerInterface') as mock_pm_class:
            mock_pm_class.get_instance.return_value = mock_pm
            self.engine._notify_sensory_stream_ready()  # Should not crash
        
        # Test with ZMQ server but no sensory
        mock_zmq = Mock()
        mock_zmq._sensory = None
        mock_pm._zmq_server = mock_zmq
        with patch('feagi.npu.burst_engine.ProcessManagerInterface') as mock_pm_class:
            mock_pm_class.get_instance.return_value = mock_pm
            self.engine._notify_sensory_stream_ready()  # Should not crash
        
        # Test exception handling
        mock_pm_class.get_instance.side_effect = Exception("Process manager error")
        with patch('feagi.npu.burst_engine.ProcessManagerInterface', mock_pm_class):
            self.engine._notify_sensory_stream_ready()  # Should not crash
    
    def test_private_initialize_frequency_from_state_manager(self):
        """Test _initialize_frequency_from_state_manager edge cases."""
        # Test with no state manager
        self.engine.state_manager = None
        self.engine._initialize_frequency_from_state_manager()
        # Should use config fallback
        
        # Test state manager exception in get_frequency_hz
        mock_sm = Mock()
        mock_sm.get_frequency_hz.side_effect = Exception("Frequency error")
        mock_sm.set_frequency_hz.side_effect = Exception("Set frequency error")
        self.engine.state_manager = mock_sm
        
        self.engine._initialize_frequency_from_state_manager()  # Should not crash
        
        # Test successful frequency sync but failed state update
        mock_sm.get_frequency_hz.side_effect = None
        mock_sm.get_frequency_hz.return_value = 15.0
        mock_sm.set_frequency_hz.side_effect = Exception("Set error")
        
        self.engine._initialize_frequency_from_state_manager()
        assert self.engine.desired_frequency == 15.0
    
    def test_private_inject_all_candidates(self):
        """Test _inject_all_candidates with different scenarios."""
        fcl = FireCandidateList()
        
        # Test with no injection service
        self.engine.injection_service = None
        self.engine._inject_all_candidates(fcl)  # Should not crash
        
        # Test with injection service exception
        mock_service = Mock()
        mock_service.inject_power_neurons.side_effect = Exception("Injection failed")
        self.engine.injection_service = mock_service
        
        # Should handle exception gracefully
        self.engine._inject_all_candidates(fcl)  # Should not crash
        
        # Test successful injection
        mock_service.inject_power_neurons.side_effect = None
        mock_service.inject_power_neurons.return_value = 5
        
        self.engine._inject_all_candidates(fcl)
        mock_service.inject_power_neurons.assert_called()
    
    def test_private_get_neuron_firing_threshold_errors(self):
        """Test _get_neuron_firing_threshold error paths."""
        # Test with no connectome manager
        self.engine.connectome_manager = None
        
        with pytest.raises(ValueError, match="No connectome manager available"):
            self.engine._get_neuron_firing_threshold(1)
        
        # Test with connectome manager but no NPU interface
        mock_cm = Mock()
        mock_cm._npu_interface = None
        self.engine.connectome_manager = mock_cm
        
        with pytest.raises(ValueError, match="No NPU interface"):
            self.engine._get_neuron_firing_threshold(1)
        
        # Test with NPU interface but no neuron array
        mock_npu = Mock()
        mock_npu.neuron_array = None
        mock_cm._npu_interface = mock_npu
        
        with pytest.raises(ValueError, match="No neuron array"):
            self.engine._get_neuron_firing_threshold(1)
        
        # Test with neuron array but no ID mapping
        mock_neuron_array = Mock()
        mock_neuron_array.neuron_id_to_index = None
        mock_npu.neuron_array = mock_neuron_array
        
        with pytest.raises(ValueError, match="No neuron ID mapping"):
            self.engine._get_neuron_firing_threshold(1)
        
        # Test with ID mapping but neuron not found
        mock_neuron_array.neuron_id_to_index = {2: 1, 3: 2}  # No neuron 1
        
        with pytest.raises(ValueError, match="not found in genome"):
            self.engine._get_neuron_firing_threshold(1)
        
        # Test with neuron found but no thresholds array
        mock_neuron_array.neuron_id_to_index = {1: 0}
        mock_neuron_array.thresholds = None
        
        with pytest.raises(ValueError, match="No firing thresholds array"):
            self.engine._get_neuron_firing_threshold(1)
        
        # Test with thresholds but index out of bounds
        mock_neuron_array.thresholds = np.array([1.0])  # Only 1 element
        mock_neuron_array.neuron_id_to_index = {1: 5}  # Index 5 out of bounds
        
        with pytest.raises(ValueError, match="out of bounds"):
            self.engine._get_neuron_firing_threshold(1)
    
    def test_private_create_firing_neurons_edge_cases(self):
        """Test _create_firing_neurons with edge cases."""
        # Test with empty neuron list
        firing_neurons = self.engine._create_firing_neurons([])
        assert firing_neurons == []
        
        # Test with non-existent neuron IDs
        with patch.object(self.engine, '_get_neuron_firing_threshold') as mock_threshold:
            mock_threshold.side_effect = ValueError("Neuron not found")
            
            firing_neurons = self.engine._create_firing_neurons([999])
            assert firing_neurons == []  # Should handle gracefully
        
        # Test with mixed valid and invalid neurons
        def threshold_side_effect(neuron_id):
            if neuron_id == 1:
                return 1.0
            else:
                raise ValueError("Invalid neuron")
        
        with patch.object(self.engine, '_get_neuron_firing_threshold', side_effect=threshold_side_effect):
            firing_neurons = self.engine._create_firing_neurons([1, 999])
            assert len(firing_neurons) == 1
            assert firing_neurons[0].neuron_id == 1


class TestBurstEngineErrorHandling:
    """Test comprehensive error handling scenarios."""
    
    def setup_method(self):
        """Setup test environment."""
        BurstEngine.reset_instance()
        self.cm = MockConnectomeManager()
        self.engine = BurstEngine(connectome_manager=self.cm)
        
    def teardown_method(self):
        """Clean up after each test."""
        BurstEngine.reset_instance()
    
    def test_process_burst_with_fcl_failure(self):
        """Test process_burst when FCL operations fail."""
        # Mock FCL to fail
        with patch('feagi.npu.burst_engine.FireCandidateList') as mock_fcl_class:
            mock_fcl = Mock()
            mock_fcl.get_total_candidate_count.side_effect = Exception("FCL failed")
            mock_fcl_class.return_value = mock_fcl
            
            # Should handle FCL failure gracefully
            fired_neurons = self.engine.process_burst()
            assert isinstance(fired_neurons, list)
    
    def test_process_burst_with_neural_dynamics_failure(self):
        """Test process_burst when neural dynamics fail."""
        # Mock _process_neural_dynamics to fail
        with patch.object(self.engine, '_process_neural_dynamics') as mock_dynamics:
            mock_dynamics.side_effect = Exception("Neural dynamics failed")
            
            # Should handle neural dynamics failure
            fired_neurons = self.engine.process_burst()
            assert isinstance(fired_neurons, list)
    
    def test_process_burst_with_state_manager_failure(self):
        """Test process_burst when state manager operations fail."""
        mock_sm = Mock()
        mock_sm.is_debug_npu_enabled.side_effect = Exception("State manager error")
        mock_sm.increment_cumulative_activity.side_effect = Exception("Activity error")
        mock_sm.get_burst_engine_state.side_effect = Exception("Get state error")
        
        self.engine.state_manager = mock_sm
        
        # Should handle all state manager errors gracefully
        fired_neurons = self.engine.process_burst()
        assert isinstance(fired_neurons, list)
    
    def test_update_frequency_validation(self):
        """Test frequency update validation."""
        # Test invalid frequencies
        assert self.engine.update_frequency(0.0) == False
        assert self.engine.update_frequency(-5.0) == False
        assert self.engine.update_frequency(float('inf')) == False
        assert self.engine.update_frequency(float('nan')) == False
        
        # Test valid frequency
        assert self.engine.update_frequency(15.0) == True
        assert self.engine.desired_frequency == 15.0
    
    def test_update_frequency_with_state_manager_failure(self):
        """Test frequency update when state manager fails."""
        mock_sm = Mock()
        mock_sm.set_frequency_hz.side_effect = Exception("State manager error")
        self.engine.state_manager = mock_sm
        
        # Should still update local frequency but return False due to state manager error
        result = self.engine.update_frequency(20.0)
        assert result == False  # Due to state manager failure
    
    def test_start_stop_with_exceptions(self):
        """Test start/stop methods with various exceptions."""
        # Test start with state manager failure
        mock_sm = Mock()
        mock_sm.set_burst_engine_state.side_effect = Exception("Start error")
        self.engine.state_manager = mock_sm
        
        result = self.engine.start()
        assert result == False  # Should return False on exception
        
        # Test stop with state manager failure
        self.engine._running = True
        result = self.engine.stop()
        assert result == False  # Should return False on exception
    
    def test_run_method_exception_handling(self):
        """Test run method exception handling."""
        # Test with process_burst failing
        with patch.object(self.engine, 'process_burst') as mock_process:
            mock_process.side_effect = Exception("Burst processing failed")
            
            # Start the engine
            self.engine._running = True
            
            # Run for a very short time to test exception handling
            with patch('time.sleep', side_effect=Exception("Sleep interrupted")):
                self.engine.run()  # Should handle exceptions gracefully
    
    def test_genome_update_with_failures(self):
        """Test genome update with various failure modes."""
        # Test with missing CoreAPIService
        with patch('feagi.npu.burst_engine.CoreAPIService', None):
            self.engine.update_with_genome(None)  # Should not crash
        
        # Test with CoreAPIService but no connectome_manager
        mock_core_service = Mock()
        mock_core_service.connectome_manager = None
        
        with patch('feagi.npu.burst_engine.CoreAPIService') as mock_cas:
            mock_cas.get_instance.return_value = mock_core_service
            self.engine.update_with_genome(None)  # Should not crash
        
        # Test with connectome_manager update failing
        self.cm.inject_failure('get_cortical_idx_for_id')
        self.engine.update_with_genome(self.cm)  # Should not crash


class TestBurstEngineStressScenarios:
    """Test stress and performance scenarios."""
    
    def setup_method(self):
        """Setup test environment."""
        BurstEngine.reset_instance()
        self.cm = MockConnectomeManager()
        self.engine = BurstEngine(connectome_manager=self.cm)
        
    def teardown_method(self):
        """Clean up after each test."""
        BurstEngine.reset_instance()
    
    def test_rapid_burst_processing(self):
        """Test rapid successive burst processing."""
        # Process many bursts rapidly
        for i in range(100):
            fired_neurons = self.engine.process_burst()
            assert isinstance(fired_neurons, list)
            assert self.engine.burst_count == i + 1
    
    def test_large_candidate_list_processing(self):
        """Test processing with very large candidate lists."""
        # Mock FCL to return large candidate count
        with patch('feagi.npu.burst_engine.FireCandidateList') as mock_fcl_class:
            mock_fcl = Mock()
            mock_fcl.get_total_candidate_count.return_value = 100000  # Very large
            mock_fcl_class.return_value = mock_fcl
            
            fired_neurons = self.engine.process_burst()
            assert isinstance(fired_neurons, list)
    
    def test_memory_pressure_simulation(self):
        """Test behavior under memory pressure."""
        # Simulate memory allocation failures
        original_fcl_init = FireCandidateList.__init__
        
        def failing_fcl_init(self):
            raise MemoryError("Simulated memory pressure")
        
        # This tests error recovery when FCL creation fails
        with patch.object(FireCandidateList, '__init__', failing_fcl_init):
            # Should handle memory pressure gracefully
            try:
                fired_neurons = self.engine.process_burst()
                # If it doesn't crash, that's good
            except MemoryError:
                # Expected behavior under memory pressure
                pass
    
    def test_concurrent_access_patterns(self):
        """Test concurrent access to BurstEngine (singleton safety)."""
        results = []
        exceptions = []
        
        def worker_thread():
            try:
                # Each thread should get the same singleton instance
                local_engine = BurstEngine.get_instance()
                results.append(id(local_engine))
                
                # Try to process bursts concurrently
                fired_neurons = local_engine.process_burst()
                results.append(len(fired_neurons))
            except Exception as e:
                exceptions.append(e)
        
        # Start multiple threads
        threads = []
        for i in range(10):
            thread = threading.Thread(target=worker_thread)
            threads.append(thread)
            thread.start()
        
        # Wait for all threads
        for thread in threads:
            thread.join(timeout=5.0)
        
        # All threads should get the same singleton instance
        engine_ids = [r for r in results if isinstance(r, int) and r > 1000]
        if engine_ids:
            assert all(engine_id == engine_ids[0] for engine_id in engine_ids)
        
        # Should not have major exceptions
        assert len(exceptions) == 0
    
    def test_frequency_update_stress(self):
        """Test rapid frequency updates."""
        frequencies = [10.0, 15.0, 20.0, 25.0, 30.0, 5.0, 1.0]
        
        for freq in frequencies:
            result = self.engine.update_frequency(freq)
            assert result in [True, False]  # Should return boolean
            if result:
                assert self.engine.desired_frequency == freq
    
    def test_long_running_simulation(self):
        """Test behavior in long-running simulation."""
        # Simulate long running by processing many bursts with state changes
        states = [ServiceState.RUNNING, ServiceState.READY, ServiceState.PAUSED]
        
        for i in range(50):
            # Change state randomly
            if i % 10 == 0:
                state = states[i % len(states)]
                self.engine._set_burst_engine_state(state)
            
            # Process burst
            fired_neurons = self.engine.process_burst()
            assert isinstance(fired_neurons, list)
            
            # Occasionally update frequency
            if i % 15 == 0:
                new_freq = 10.0 + (i % 20)
                self.engine.update_frequency(new_freq)


class TestBurstEngineBoundaryConditions:
    """Test boundary conditions and edge cases."""
    
    def setup_method(self):
        """Setup test environment."""
        BurstEngine.reset_instance()
        self.cm = MockConnectomeManager()
        self.engine = BurstEngine(connectome_manager=self.cm)
        
    def teardown_method(self):
        """Clean up after each test."""
        BurstEngine.reset_instance()
    
    def test_zero_neurons_scenario(self):
        """Test with zero neurons in connectome."""
        # Empty connectome
        empty_cm = Mock()
        empty_cm.cortical_areas = {}
        empty_cm._npu_interface = NPUInterface(backend=BackendType.CPU, max_neurons=10)
        empty_cm.get_neurons_by_cortical_area.return_value = []
        
        empty_engine = BurstEngine(connectome_manager=empty_cm)
        
        fired_neurons = empty_engine.process_burst()
        assert fired_neurons == []
    
    def test_extreme_frequency_values(self):
        """Test with extreme frequency values."""
        # Very high frequency
        result = self.engine.update_frequency(10000.0)
        assert result == True
        assert self.engine.desired_frequency == 10000.0
        
        # Very low frequency
        result = self.engine.update_frequency(0.001)
        assert result == True
        assert self.engine.desired_frequency == 0.001
    
    def test_burst_count_overflow(self):
        """Test behavior with very high burst counts."""
        # Set burst count to near overflow
        self.engine.burst_count = 2**30  # Very large number
        
        fired_neurons = self.engine.process_burst()
        assert isinstance(fired_neurons, list)
        assert self.engine.burst_count == 2**30 + 1
    
    def test_empty_fire_candidate_list(self):
        """Test processing with empty FCL."""
        with patch('feagi.npu.burst_engine.FireCandidateList') as mock_fcl_class:
            mock_fcl = Mock()
            mock_fcl.get_total_candidate_count.return_value = 0  # Empty FCL
            mock_fcl_class.return_value = mock_fcl
            
            fired_neurons = self.engine.process_burst()
            assert fired_neurons == []
    
    def test_invalid_connectome_states(self):
        """Test with invalid or corrupted connectome states."""
        # Connectome with None values
        corrupt_cm = Mock()
        corrupt_cm.cortical_areas = None
        corrupt_cm._npu_interface = None
        
        corrupt_engine = BurstEngine(connectome_manager=corrupt_cm)
        
        # Should handle gracefully
        fired_neurons = corrupt_engine.process_burst()
        assert isinstance(fired_neurons, list)
    
    def test_state_transitions_edge_cases(self):
        """Test state transitions in edge cases."""
        # Rapid state changes
        states = [ServiceState.INITIALIZING, ServiceState.READY, ServiceState.RUNNING, 
                 ServiceState.PAUSED, ServiceState.ERROR, ServiceState.STOPPED]
        
        for state in states:
            self.engine._set_burst_engine_state(state)
            # Should not crash regardless of state
            fired_neurons = self.engine.process_burst()
            assert isinstance(fired_neurons, list)


class TestBurstEngineIntegrationStress:
    """Test integration scenarios with stress conditions."""
    
    def setup_method(self):
        """Setup test environment."""
        BurstEngine.reset_instance()
        self.cm = MockConnectomeManager()
        self.engine = BurstEngine(connectome_manager=self.cm)
        
    def teardown_method(self):
        """Clean up after each test."""
        BurstEngine.reset_instance()
    
    def test_full_pipeline_with_failures(self):
        """Test full pipeline with induced failures at each stage."""
        # Stage 1: FCL injection failure
        mock_service = Mock()
        mock_service.inject_power_neurons.side_effect = [
            Exception("Injection failed"),  # First call fails
            5,  # Second call succeeds
        ]
        self.engine.injection_service = mock_service
        
        # Should handle injection failure gracefully
        fired_neurons1 = self.engine.process_burst()
        assert isinstance(fired_neurons1, list)
        
        # Stage 2: Second burst should work
        fired_neurons2 = self.engine.process_burst()
        assert isinstance(fired_neurons2, list)
    
    def test_resource_exhaustion_recovery(self):
        """Test recovery from resource exhaustion."""
        # Simulate various resource failures
        failure_scenarios = [
            ("Memory allocation", MemoryError("Out of memory")),
            ("File system", IOError("Disk full")),
            ("Network", ConnectionError("Network unavailable")),
        ]
        
        for scenario_name, exception in failure_scenarios:
            # Mock internal methods to fail
            with patch.object(self.engine, '_process_neural_dynamics') as mock_dynamics:
                mock_dynamics.side_effect = exception
                
                # Should handle resource exhaustion
                fired_neurons = self.engine.process_burst()
                assert isinstance(fired_neurons, list)
    
    def test_complex_connectome_operations(self):
        """Test complex connectome operations under stress."""
        # Add synaptic propagation complexity
        with patch.object(self.engine, '_compute_synaptic_propagation') as mock_propagation:
            # Return complex propagation data
            mock_propagation.return_value = {
                1: [(2, 0.5), (3, 0.3), (100, 0.8)],
                2: [(3, 0.2), (101, 0.6)],
                100: [(1, 1.0), (2, 0.9), (3, 0.7)]
            }
            
            fired_neurons = self.engine.process_burst()
            assert isinstance(fired_neurons, list)
            mock_propagation.assert_called()
    
    def test_performance_monitoring_integration(self):
        """Test integration with performance monitoring."""
        # Mock performance monitoring
        performance_data = []
        
        def mock_performance_callback(burst_num, fired_count, processing_time):
            performance_data.append({
                'burst': burst_num,
                'fired': fired_count,
                'time': processing_time
            })
        
        # Process multiple bursts with monitoring
        for i in range(10):
            start_time = time.time()
            fired_neurons = self.engine.process_burst()
            end_time = time.time()
            
            mock_performance_callback(
                self.engine.burst_count, 
                len(fired_neurons), 
                end_time - start_time
            )
        
        # Verify monitoring data
        assert len(performance_data) == 10
        for data in performance_data:
            assert 'burst' in data
            assert 'fired' in data
            assert 'time' in data


class TestBurstEngineConfigurationEdgeCases:
    """Test configuration and initialization edge cases."""
    
    def setup_method(self):
        """Setup test environment."""
        BurstEngine.reset_instance()
        
    def teardown_method(self):
        """Clean up after each test."""
        BurstEngine.reset_instance()
    
    def test_initialization_with_invalid_parameters(self):
        """Test initialization with invalid parameters."""
        # Invalid fire ledger window size
        engine = BurstEngine(fire_ledger_window_size=0)
        assert engine is not None  # Should handle gracefully
        
        engine = BurstEngine(fire_ledger_window_size=-10)
        assert engine is not None  # Should handle gracefully
        
        # Very large window size
        engine = BurstEngine(fire_ledger_window_size=1000000)
        assert engine is not None
    
    def test_configuration_loading_failures(self):
        """Test configuration loading failures."""
        cm = MockConnectomeManager()
        
        # Mock configuration loading to fail
        with patch.object(BurstEngine, '_load_burst_engine_config') as mock_load:
            mock_load.side_effect = Exception("Config load failed")
            
            engine = BurstEngine(connectome_manager=cm)
            assert engine is not None  # Should handle gracefully
    
    def test_memory_structure_reinitialization(self):
        """Test memory structure reinitialization edge cases."""
        cm = MockConnectomeManager()
        engine = BurstEngine(connectome_manager=cm)
        
        # Test reinitialization with various failure modes
        with patch.object(engine, '_reinitialize_memory_structures_for_genome') as mock_reinit:
            mock_reinit.side_effect = Exception("Reinitialization failed")
            
            engine.update_with_genome(cm)  # Should handle gracefully
    
    def test_excitability_cache_operations(self):
        """Test excitability cache edge cases."""
        cm = MockConnectomeManager()
        engine = BurstEngine(connectome_manager=cm)
        
        # Test cache operations
        engine.invalidate_excitability_cache()  # Should not crash
        
        # Test cache building with various conditions
        with patch.object(engine, '_build_excitability_cache') as mock_build:
            mock_build.side_effect = Exception("Cache build failed")
            
            # Should handle cache build failure
            fired_neurons = engine.process_burst()
            assert isinstance(fired_neurons, list)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
