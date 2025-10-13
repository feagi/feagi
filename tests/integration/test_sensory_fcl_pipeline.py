"""
Integration tests for the complete sensory→FCL pipeline.

Tests the end-to-end flow:
1. Sensory data arrives via SHM
2. SensoryNeuralStream decodes it
3. BrainService processes coordinates
4. ConnectomeManager maps xyz→neuron_id and id→idx
5. PowerInjectionService injects into FCL
6. FCL contains the expected neurons

This validates the strict, SIMD-optimized mapping path with no fallbacks.
"""
import pytest
import numpy as np
import tempfile
import os
import time
from unittest.mock import Mock, patch, MagicMock
from typing import Dict, List, Any, Tuple

from feagi.npu.fire_candidate_list import FireCandidateList
from feagi.api.core.services.brain.brain_service import BrainService
from feagi.npu.burst_engine import PowerInjectionService
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.api.zmq.streams.sensory_neural import SensoryNeuralStream
from feagi.core.state_manager import FeagiStateManager


class TestSensoryFCLPipeline:
    """Integration tests for sensory data → FCL injection pipeline."""

    @pytest.fixture
    def mock_connectome_manager(self):
        """Mock ConnectomeManager with deterministic mappings."""
        cm = Mock(spec=ConnectomeManager)
        
        # Mock batch_voxel_to_neuron_lookup: (x,y,z) → [(neuron_id, weight), ...]
        def mock_batch_lookup(cortical_id: str, candidate_positions: set, post_synaptic_current: float):
            results = []
            for pos in candidate_positions:
                x, y, z = pos
                # Deterministic mapping: neuron_id = x*1000 + y*100 + z
                neuron_id = x * 1000 + y * 100 + z
                results.append((neuron_id, post_synaptic_current))
            return results
        
        cm.batch_voxel_to_neuron_lookup.side_effect = mock_batch_lookup
        
        # Mock get_cortical_idx_for_id: cortical_id → cortical_idx
        cortical_mapping = {
            'iic000': 0, 'iic100': 1, 'iic200': 2, 'iic300': 3,
            'iic400': 4, 'iic500': 5, 'iic600': 6, 'iic700': 7, 'iic800': 8
        }
        cm.get_cortical_idx_for_id.side_effect = lambda cid: cortical_mapping.get(cid)
        
        # Mock get_neuron_position for position mapping
        def mock_get_position(neuron_id: int):
            # Reverse the deterministic mapping
            z = neuron_id % 100
            y = (neuron_id // 100) % 10
            x = neuron_id // 1000
            return [0, x, y, z]  # [cortical_idx, x, y, z]
        
        cm.get_neuron_position.side_effect = mock_get_position
        
        return cm

    @pytest.fixture
    def mock_fcl(self):
        """Mock FireCandidateList that tracks enqueued candidates."""
        fcl = Mock(spec=FireCandidateList)
        fcl.enqueued_candidates = []  # Track what gets enqueued
        
        def mock_add_candidates(cortical_idx: int, neuron_ids: np.ndarray, 
                              potential_deltas: np.ndarray, excitatory_mask: np.ndarray = None):
            # Store the enqueued data for verification
            for i, neuron_id in enumerate(neuron_ids):
                fcl.enqueued_candidates.append({
                    'cortical_idx': cortical_idx,
                    'neuron_id': int(neuron_id),
                    'potential_delta': float(potential_deltas[i])
                })
            return len(neuron_ids)
        
        fcl.add_candidates_soa.side_effect = mock_add_candidates
        return fcl

    @pytest.fixture
    def mock_state_manager(self):
        """Mock StateManager with debug flags enabled."""
        sm = Mock(spec=FeagiStateManager)
        sm.is_debug_npu_enabled.return_value = True
        sm.is_debug_shm_enabled.return_value = True
        return sm

    @pytest.fixture
    def power_injection_service(self, mock_connectome_manager, mock_fcl):
        """PowerInjectionService with mocked dependencies."""
        service = PowerInjectionService(mock_connectome_manager, mock_fcl)
        return service

    @pytest.fixture
    def brain_service(self, mock_connectome_manager):
        """BrainService with mocked ConnectomeManager."""
        service = BrainService(mock_connectome_manager)
        service._connectome_manager = mock_connectome_manager
        
        # Mock _get_burst_engine to return a mock with injection_service
        mock_burst_engine = Mock()
        mock_burst_engine.injection_service = None  # Will be set in tests
        mock_burst_engine.is_debug_npu_enabled.return_value = True
        service._get_burst_engine = Mock(return_value=mock_burst_engine)
        
        return service

    def test_xyz_to_neuron_id_mapping(self, mock_connectome_manager):
        """Test ConnectomeManager batch_voxel_to_neuron_lookup."""
        candidate_positions = {(1, 2, 3), (4, 5, 6)}
        
        results = mock_connectome_manager.batch_voxel_to_neuron_lookup(
            cortical_id='iic400',
            candidate_positions=candidate_positions,
            post_synaptic_current=1.0
        )
        
        expected = [(1203, 1.0), (4506, 1.0)]  # x*1000 + y*100 + z
        assert sorted(results) == sorted(expected)

    def test_cortical_id_to_idx_mapping(self, mock_connectome_manager):
        """Test ConnectomeManager get_cortical_idx_for_id."""
        assert mock_connectome_manager.get_cortical_idx_for_id('iic400') == 4
        assert mock_connectome_manager.get_cortical_idx_for_id('iic000') == 0
        assert mock_connectome_manager.get_cortical_idx_for_id('unknown') is None

    def test_fcl_enqueue_mechanics(self, mock_fcl):
        """Test FireCandidateList add_candidates_soa."""
        neuron_ids = np.array([1203, 4506], dtype=np.uint32)
        potentials = np.array([0.5, 0.7], dtype=np.float32)
        excitatory_mask = np.ones(2, dtype=bool)
        
        added = mock_fcl.add_candidates_soa(
            cortical_idx=4,
            neuron_ids=neuron_ids,
            potential_deltas=potentials,
            excitatory_mask=excitatory_mask
        )
        
        assert added == 2
        assert len(mock_fcl.enqueued_candidates) == 2
        assert mock_fcl.enqueued_candidates[0]['neuron_id'] == 1203
        assert mock_fcl.enqueued_candidates[0]['potential_delta'] == 0.5
        assert mock_fcl.enqueued_candidates[1]['neuron_id'] == 4506
        assert abs(mock_fcl.enqueued_candidates[1]['potential_delta'] - 0.7) < 1e-6

    def test_power_injection_service_xyz_format(self, power_injection_service, mock_fcl):
        """Test PowerInjectionService with xyz coordinate format."""
        activations = {
            'iic400': {
                'coordinates_x': np.array([1, 4]),
                'coordinates_y': np.array([2, 5]),
                'coordinates_z': np.array([3, 6]),
                'membrane_potentials': np.array([0.5, 0.7])
            }
        }
        
        injected = power_injection_service.inject_external_activations(
            activations=activations,
            current_timestep=1,
            source="test"
        )
        
        assert injected == 2
        assert len(mock_fcl.enqueued_candidates) == 2
        
        # Verify the mapping: (1,2,3) → 1203, (4,5,6) → 4506
        candidates = sorted(mock_fcl.enqueued_candidates, key=lambda x: x['neuron_id'])
        assert candidates[0]['neuron_id'] == 1203
        assert candidates[0]['cortical_idx'] == 4  # iic400 → idx 4
        assert candidates[0]['potential_delta'] == 0.5
        assert candidates[1]['neuron_id'] == 4506
        assert candidates[1]['potential_delta'] == 0.7

    def test_power_injection_service_neuron_id_list_format(self, power_injection_service, mock_fcl):
        """Test PowerInjectionService with direct neuron_id list format."""
        activations = {
            'iic200': [1203, 4506, 7890]
        }
        
        injected = power_injection_service.inject_external_activations(
            activations=activations,
            current_timestep=1,
            source="test"
        )
        
        assert injected == 3
        assert len(mock_fcl.enqueued_candidates) == 3
        
        # All should have cortical_idx=2 (iic200) and default potential=1.0
        for candidate in mock_fcl.enqueued_candidates:
            assert candidate['cortical_idx'] == 2
            assert candidate['potential_delta'] == 1.0
        
        neuron_ids = [c['neuron_id'] for c in mock_fcl.enqueued_candidates]
        assert sorted(neuron_ids) == [1203, 4506, 7890]

    def test_brain_service_stimulate_neurons_unified(self, brain_service, mock_fcl):
        """Test BrainService stimulate_neurons_unified end-to-end."""
        # Set up the injection service
        brain_service._get_burst_engine().injection_service = Mock()
        brain_service._get_burst_engine().injection_service.inject_external_activations = Mock(return_value=2)
        
        neural_data = {
            'iic400': {
                'coordinates_x': np.array([1, 4]),
                'coordinates_y': np.array([2, 5]),
                'coordinates_z': np.array([3, 6]),
                'membrane_potentials': np.array([0.5, 0.7])
            }
        }
        
        result = brain_service.stimulate_neurons_unified(neural_data)
        
        # Verify injection service was called
        brain_service._get_burst_engine().injection_service.inject_external_activations.assert_called_once()
        call_args = brain_service._get_burst_engine().injection_service.inject_external_activations.call_args
        
        # Check the activations passed to injection service
        activations = call_args[1]['activations']  # kwargs
        assert 'iic400' in activations
        assert len(activations['iic400']) == 2  # 2 unique neuron IDs
        assert 1203 in activations['iic400']  # (1,2,3) → 1203
        assert 4506 in activations['iic400']  # (4,5,6) → 4506

    def test_unresolved_cortical_id_handling(self, power_injection_service, mock_fcl):
        """Test handling of unresolved cortical IDs (no fallbacks)."""
        activations = {
            'unknown_area': {
                'coordinates_x': np.array([1]),
                'coordinates_y': np.array([2]),
                'coordinates_z': np.array([3]),
                'membrane_potentials': np.array([0.5])
            }
        }
        
        injected = power_injection_service.inject_external_activations(
            activations=activations,
            current_timestep=1,
            source="test"
        )
        
        # Should inject 0 neurons (area skipped)
        assert injected == 0
        assert len(mock_fcl.enqueued_candidates) == 0

    def test_empty_batch_lookup_handling(self, power_injection_service, mock_fcl, mock_connectome_manager):
        """Test handling when batch_voxel_to_neuron_lookup returns empty."""
        # Override mock to return empty results
        mock_connectome_manager.batch_voxel_to_neuron_lookup.return_value = []
        
        activations = {
            'iic400': {
                'coordinates_x': np.array([1]),
                'coordinates_y': np.array([2]),
                'coordinates_z': np.array([3]),
                'membrane_potentials': np.array([0.5])
            }
        }
        
        injected = power_injection_service.inject_external_activations(
            activations=activations,
            current_timestep=1,
            source="test"
        )
        
        # Should inject 0 neurons (no mapping found)
        assert injected == 0
        assert len(mock_fcl.enqueued_candidates) == 0

    @patch('feagi.core.state_manager.FeagiStateManager.instance')
    def test_debug_logging_integration(self, mock_state_instance, power_injection_service, mock_fcl):
        """Test that debug logging works correctly."""
        mock_state_instance.return_value.is_debug_npu_enabled.return_value = True
        
        activations = {
            'iic400': {
                'coordinates_x': np.array([1]),
                'coordinates_y': np.array([2]),
                'coordinates_z': np.array([3]),
                'membrane_potentials': np.array([0.5])
            }
        }
        
        with patch('feagi.npu.burst_engine.logger') as mock_logger:
            injected = power_injection_service.inject_external_activations(
                activations=activations,
                current_timestep=1,
                source="test"
            )
            
            # Verify debug logs were called
            assert mock_logger.info.called
            assert injected == 1

    def test_multiple_areas_processing(self, power_injection_service, mock_fcl):
        """Test processing multiple cortical areas in one call."""
        activations = {
            'iic000': {
                'coordinates_x': np.array([1]),
                'coordinates_y': np.array([2]),
                'coordinates_z': np.array([3]),
                'membrane_potentials': np.array([0.5])
            },
            'iic100': {
                'coordinates_x': np.array([4]),
                'coordinates_y': np.array([5]),
                'coordinates_z': np.array([6]),
                'membrane_potentials': np.array([0.7])
            }
        }
        
        injected = power_injection_service.inject_external_activations(
            activations=activations,
            current_timestep=1,
            source="test"
        )
        
        assert injected == 2
        assert len(mock_fcl.enqueued_candidates) == 2
        
        # Verify both areas were processed with correct cortical_idx
        cortical_indices = {c['cortical_idx'] for c in mock_fcl.enqueued_candidates}
        assert cortical_indices == {0, 1}  # iic000→0, iic100→1

    def test_simd_optimization_unique_coordinates(self, power_injection_service, mock_fcl):
        """Test SIMD optimization with duplicate coordinates."""
        activations = {
            'iic400': {
                'coordinates_x': np.array([1, 1, 4]),  # Duplicate (1,2,3)
                'coordinates_y': np.array([2, 2, 5]),
                'coordinates_z': np.array([3, 3, 6]),
                'membrane_potentials': np.array([0.5, 0.6, 0.7])
            }
        }
        
        injected = power_injection_service.inject_external_activations(
            activations=activations,
            current_timestep=1,
            source="test"
        )
        
        # Should process unique coordinates: (1,2,3) and (4,5,6)
        assert injected == 2
        assert len(mock_fcl.enqueued_candidates) == 2
        
        neuron_ids = {c['neuron_id'] for c in mock_fcl.enqueued_candidates}
        assert neuron_ids == {1203, 4506}


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
