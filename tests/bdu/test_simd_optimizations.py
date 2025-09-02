"""
Tests for SIMD-optimized methods in FEAGI.

This test module provides comprehensive coverage for the newly implemented
SIMD optimizations including:
- stimulate_neurons_unified() method in BrainService
- batch_voxel_to_neuron_lookup() method in ConnectomeManager
- Vectorized neurogenesis optimizations
- Coordinate-based neural stimulation format
"""

from unittest.mock import patch

import numpy as np
import pytest

from feagi.api.core.services.brain.brain_service import BrainService
from feagi.bdu.connectome_manager import ConnectomeManager


class TestSIMDOptimizations:
    """Test suite for SIMD-optimized methods."""

    @pytest.fixture
    def connectome_manager(self):
        """Create a ConnectomeManager instance for testing."""
        ConnectomeManager.reset_singleton()
        return ConnectomeManager(1000)

    @pytest.fixture
    def brain_service(self, connectome_manager):
        """Create a BrainService instance for testing."""
        return BrainService(connectome_manager, None)

    @pytest.fixture
    def sample_neural_data(self):
        """Create sample neural data in the unified format."""
        return {
            'test_area_1': {
                'coordinates_x': np.array([1, 2, 3, 4], dtype=np.uint16),
                'coordinates_y': np.array([1, 2, 3, 4], dtype=np.uint16),
                'coordinates_z': np.array([0, 0, 1, 1], dtype=np.uint16),
                'membrane_potentials': np.array([0.8, 1.2, 0.9, 1.1], dtype=np.float32),
            },
            'test_area_2': {
                'coordinates_x': np.array([5, 6], dtype=np.uint16),
                'coordinates_y': np.array([5, 6], dtype=np.uint16),
                'coordinates_z': np.array([0, 0], dtype=np.uint16),
                'membrane_potentials': np.array([1.0, 0.9], dtype=np.float32),
            }
        }

    def test_stimulate_neurons_unified_method_exists(self, brain_service):
        """Test that the stimulate_neurons_unified method exists and has correct signature."""
        assert hasattr(brain_service, 'stimulate_neurons_unified')
        
        # Check method signature
        import inspect
        sig = inspect.signature(brain_service.stimulate_neurons_unified)
        params = list(sig.parameters.keys())
        
        assert 'neural_data' in params
        assert len(params) >= 1  # At least neural_data parameter

    def test_stimulate_neurons_unified_data_validation(self, brain_service, sample_neural_data):
        """Test that the unified stimulation method validates input data correctly."""
        # Test with valid data structure
        result = brain_service.stimulate_neurons_unified(sample_neural_data)
        
        assert isinstance(result, dict)
        assert 'success' in result
        assert 'method' in result
        assert result['method'] == 'unified_coordinate_based_simd_optimized'

    def test_stimulate_neurons_unified_empty_data(self, brain_service):
        """Test unified stimulation with empty data."""
        empty_data = {}
        result = brain_service.stimulate_neurons_unified(empty_data)
        
        assert result['success'] is True
        assert result['total_stimulated'] == 0
        assert result['areas_processed'] == 0

    def test_stimulate_neurons_unified_invalid_data_format(self, brain_service):
        """Test unified stimulation with invalid data format."""
        invalid_data = {
            'test_area': {
                'coordinates_x': [1, 2, 3],  # Should be numpy array
                'coordinates_y': np.array([1, 2, 3], dtype=np.uint16),
                'coordinates_z': np.array([0, 0, 0], dtype=np.uint16),
                # Missing membrane_potentials
            }
        }
        
        result = brain_service.stimulate_neurons_unified(invalid_data)
        assert isinstance(result, dict)
        # Should handle gracefully even with invalid format

    def test_stimulate_neurons_unified_numpy_conversion(self, brain_service):
        """Test that the method correctly converts data to numpy arrays."""
        # Test with mixed data types that need conversion
        mixed_data = {
            'test_area': {
                'coordinates_x': [1, 2, 3],  # Python list
                'coordinates_y': np.array([1, 2, 3], dtype=np.int64),  # Wrong dtype
                'coordinates_z': np.array([0, 0, 0], dtype=np.uint16),  # Correct
                'membrane_potentials': [0.8, 1.2, 0.9],  # Python list
            }
        }
        
        result = brain_service.stimulate_neurons_unified(mixed_data)
        assert isinstance(result, dict)
        assert 'success' in result

    def test_stimulate_neurons_unified_simd_vectorization(self, brain_service, sample_neural_data):
        """Test that the method uses SIMD vectorization (numpy operations)."""
        with patch('numpy.column_stack') as mock_column_stack, \
             patch('numpy.unique') as mock_unique:
            
            # Set up mocks to track vectorized operations
            mock_column_stack.return_value = np.array([[1, 1, 0], [2, 2, 0]])
            mock_unique.return_value = (np.array([[1, 1, 0], [2, 2, 0]]), np.array([0, 1]))
            
            brain_service.stimulate_neurons_unified(sample_neural_data)
            
            # Verify that numpy vectorized operations were called
            assert mock_column_stack.called
            assert mock_unique.called

    def test_batch_voxel_to_neuron_lookup_method_exists(self, connectome_manager):
        """Test that the batch_voxel_to_neuron_lookup method exists."""
        assert hasattr(connectome_manager, 'batch_voxel_to_neuron_lookup')

    def test_batch_voxel_to_neuron_lookup_empty_positions(self, connectome_manager):
        """Test batch lookup with empty positions set."""
        result = connectome_manager.batch_voxel_to_neuron_lookup(
            cortical_id='test_area',
            candidate_positions=set(),
            post_synaptic_current=1.0
        )
        
        assert isinstance(result, list)
        assert len(result) == 0

    def test_batch_voxel_to_neuron_lookup_valid_positions(self, connectome_manager):
        """Test batch lookup with valid positions."""
        # First create a cortical area
        area_id = connectome_manager.add_cortical_area(
            name="test_area", 
            dimensions=(10, 10, 1), 
            position=(0, 0, 0), 
            area_type="sensory"
        )
        
        # Create some neurons in the area
        connectome_manager.create_neuron(cortical_id=area_id, position=(1, 1, 0))
        connectome_manager.create_neuron(cortical_id=area_id, position=(1, 1, 0))
        connectome_manager.create_neuron(cortical_id=area_id, position=(2, 2, 0))
        connectome_manager.create_neuron(cortical_id=area_id, position=(2, 2, 0))
        
        candidate_positions = {(1, 1, 0), (2, 2, 0), (3, 3, 0)}
        
        result = connectome_manager.batch_voxel_to_neuron_lookup(
            cortical_id=area_id,
            candidate_positions=candidate_positions,
            post_synaptic_current=1.5
        )
        
        assert isinstance(result, list)
        # Should find neurons at (1,1,0) and (2,2,0), but not (3,3,0)
        assert len(result) == 4  # 2 neurons at each of 2 positions
        
        # Check that all results have correct weight
        for _neuron_id, weight in result:
            assert weight == 1.5



    def test_numpy_simd_operations_basic(self):
        """Test basic numpy SIMD operations used in optimizations."""
        # Test coordinate matrix operations
        coords_x = np.array([1, 2, 3, 2, 1], dtype=np.uint16)
        coords_y = np.array([1, 2, 3, 2, 1], dtype=np.uint16)
        coords_z = np.array([0, 0, 1, 0, 0], dtype=np.uint16)
        
        # Test vectorized coordinate matrix creation
        coordinate_matrix = np.column_stack((coords_x, coords_y, coords_z))
        assert coordinate_matrix.shape == (5, 3)
        
        # Test vectorized unique coordinate finding
        unique_coords, inverse_indices = np.unique(coordinate_matrix, axis=0, return_inverse=True)
        
        # Should find 3 unique coordinates: (1,1,0), (2,2,0), (3,3,1)
        assert len(unique_coords) == 3
        assert len(inverse_indices) == 5

    def test_numpy_simd_operations_performance(self):
        """Test that numpy operations work efficiently with larger datasets."""
        # Create larger test dataset
        size = 10000
        coords_x = np.random.randint(0, 100, size, dtype=np.uint16)
        coords_y = np.random.randint(0, 100, size, dtype=np.uint16)
        coords_z = np.random.randint(0, 10, size, dtype=np.uint16)
        
        # Test vectorized operations
        coordinate_matrix = np.column_stack((coords_x, coords_y, coords_z))
        unique_coords, inverse_indices = np.unique(coordinate_matrix, axis=0, return_inverse=True)
        
        # Verify results are sensible
        assert coordinate_matrix.shape == (size, 3)
        assert len(unique_coords) <= size  # Should be fewer unique than total
        assert len(inverse_indices) == size

    def test_coordinate_data_types(self, brain_service):
        """Test that coordinate data types are handled correctly."""
        # Test with different numpy dtypes
        test_data = {
            'test_area': {
                'coordinates_x': np.array([1, 2, 3], dtype=np.int32),  # Different dtype
                'coordinates_y': np.array([1, 2, 3], dtype=np.uint16),  # Different dtype
                'coordinates_z': np.array([0, 0, 0], dtype=np.uint8),   # Different dtype
                'membrane_potentials': np.array([0.8, 1.2, 0.9], dtype=np.float64),  # Different dtype
            }
        }
        
        result = brain_service.stimulate_neurons_unified(test_data)
        assert isinstance(result, dict)
        assert 'success' in result

    def test_stimulate_neurons_unified_multiple_areas(self, brain_service, sample_neural_data):
        """Test unified stimulation with multiple cortical areas."""
        result = brain_service.stimulate_neurons_unified(sample_neural_data)
        
        assert result['areas_processed'] == 2
        assert 'area_results' in result
        assert 'test_area_1' in result['area_results']
        assert 'test_area_2' in result['area_results']

    def test_stimulate_neurons_unified_error_handling(self, brain_service):
        """Test error handling in unified stimulation method."""
        # Test with None input
        result = brain_service.stimulate_neurons_unified(None)
        assert result['success'] is False
        assert 'error' in result

    @pytest.mark.parametrize("intensity", [0.0, 0.5, 1.0, 1.5, 2.0])
    def test_stimulate_neurons_unified_intensity_values(self, brain_service, intensity):
        """Test unified stimulation with different intensity values."""
        test_data = {
            'test_area': {
                'coordinates_x': np.array([1, 2], dtype=np.uint16),
                'coordinates_y': np.array([1, 2], dtype=np.uint16),
                'coordinates_z': np.array([0, 0], dtype=np.uint16),
                'membrane_potentials': np.array([intensity, intensity], dtype=np.float32),
            }
        }
        
        result = brain_service.stimulate_neurons_unified(test_data)
        assert isinstance(result, dict)
        assert 'success' in result

    def test_coordinate_array_validation(self, brain_service):
        """Test validation of coordinate array lengths."""
        # Test mismatched array lengths
        mismatched_data = {
            'test_area': {
                'coordinates_x': np.array([1, 2, 3], dtype=np.uint16),
                'coordinates_y': np.array([1, 2], dtype=np.uint16),  # Different length
                'coordinates_z': np.array([0, 0, 0], dtype=np.uint16),
                'membrane_potentials': np.array([0.8, 1.2, 0.9], dtype=np.float32),
            }
        }
        
        result = brain_service.stimulate_neurons_unified(mismatched_data)
        # Should handle gracefully or report error
        assert isinstance(result, dict)

    def test_simd_optimization_flags(self, brain_service, sample_neural_data):
        """Test that SIMD optimization flags are set correctly in results."""
        result = brain_service.stimulate_neurons_unified(sample_neural_data)
        
        # Check for SIMD optimization indicators
        assert result.get('method') == 'unified_coordinate_based_simd_optimized'
        
        if 'area_results' in result:
            for _area_id, area_result in result['area_results'].items():
                if area_result.get('success'):
                    assert 'optimization_used' in area_result
                    assert area_result['optimization_used'] == 'simd_vectorized'

    def test_memory_efficiency(self, brain_service):
        """Test memory efficiency with large coordinate arrays."""
        # Create large dataset to test memory efficiency
        size = 50000
        large_data = {
            'large_area': {
                'coordinates_x': np.arange(size, dtype=np.uint16),
                'coordinates_y': np.arange(size, dtype=np.uint16),
                'coordinates_z': np.zeros(size, dtype=np.uint16),
                'membrane_potentials': np.ones(size, dtype=np.float32),
            }
        }
        
        # This should not cause memory issues with SIMD optimization
        result = brain_service.stimulate_neurons_unified(large_data)
        assert isinstance(result, dict)
        assert 'success' in result

    def test_backward_compatibility(self, brain_service):
        """Test that the new unified method maintains backward compatibility."""
        # Test that old stimulate_neurons method still exists
        assert hasattr(brain_service, 'stimulate_neurons')
        
        # Test that it works with the old interface
        result = brain_service.stimulate_neurons(['1', '2', '3'], 1.0)
        assert isinstance(result, dict)
        assert 'success' in result


class TestNeurogenesisSIMDOptimizations:
    """Test suite for neurogenesis SIMD optimizations."""

    def test_vectorized_position_generation(self):
        """Test vectorized position generation for neurogenesis."""
        width, height, depth = 10, 10, 5
        total_voxels = width * height * depth
        
        # Test that we can generate all positions efficiently
        positions = []
        for x in range(width):
            for y in range(height):
                for z in range(depth):
                    positions.append((x, y, z))
        
        assert len(positions) == total_voxels
        
        # Test numpy conversion
        positions_array = np.array(positions, dtype=np.uint16)
        assert positions_array.shape == (total_voxels, 3)

    def test_vectorized_unique_position_finding(self):
        """Test vectorized unique position finding for neurogenesis."""
        # Create test positions with some duplicates
        positions = [(1, 1, 0), (2, 2, 0), (1, 1, 0), (3, 3, 0), (2, 2, 0)]
        positions_array = np.array(positions, dtype=np.uint16)
        
        # Test numpy unique operation
        unique_positions, inverse_indices = np.unique(positions_array, axis=0, return_inverse=True)
        
        assert len(unique_positions) == 3  # Should find 3 unique positions
        assert len(inverse_indices) == 5   # Should have mapping for all 5 original positions
        
        # Test that inverse indices correctly map back
        reconstructed = unique_positions[inverse_indices]
        np.testing.assert_array_equal(reconstructed, positions_array)

    def test_neuron_grouping_by_position(self):
        """Test efficient neuron grouping by position."""
        # Simulate neuron IDs and their positions
        neuron_ids = [101, 102, 103, 104, 105]
        positions = [(1, 1, 0), (1, 1, 0), (2, 2, 0), (1, 1, 0), (2, 2, 0)]
        
        positions_array = np.array(positions, dtype=np.uint16)
        unique_positions, inverse_indices = np.unique(positions_array, axis=0, return_inverse=True)
        
        # Group neurons by position using vectorized operations
        position_to_neurons = {}
        for i, unique_pos in enumerate(unique_positions):
            mask = (inverse_indices == i)
            neurons_at_pos = np.array(neuron_ids)[mask]
            position_to_neurons[tuple(unique_pos)] = neurons_at_pos.tolist()
        
        # Verify grouping
        assert len(position_to_neurons) == 2
        assert (1, 1, 0) in position_to_neurons
        assert (2, 2, 0) in position_to_neurons
        assert len(position_to_neurons[(1, 1, 0)]) == 3  # neurons 101, 102, 104
        assert len(position_to_neurons[(2, 2, 0)]) == 2  # neurons 103, 105 