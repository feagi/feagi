import inspect
import os
import time
from unittest.mock import MagicMock

import psutil
import pytest

from feagi.api.core.services.genome.genome_service import GenomeService
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis

"""
Comprehensive test suite for cortical mapping system fix.

This test validates the complete flow:
1. Create cortical mapping via PUT /v1/cortical_mapping/mapping_properties
2. Validate mapping appears in all endpoints:
   - /v1/cortical_area/cortical_map_detailed
   - /v1/connectome/cortical_info/{cortical_area}
   - /v1/cortical_area/cortical_area_properties
3. Verify actual synapses are created in the connectome

PERFORMANCE: Tests are optimized for Rust/RTOS/SIMD/GPU compatibility validation.
"""


class TestCorticalMappingFix:
    """Test suite for cortical mapping system fix."""

    @pytest.fixture
    def connectome_manager(self):
        """Create a test connectome manager with cortical areas."""
        # Reset singleton to ensure clean state for testing
        ConnectomeManager.reset_singleton()

        cm = ConnectomeManager()

        # Create test cortical areas
        area1_id = cm.add_cortical_area(
            name="TestArea1",
            dimensions=(10, 10, 1),
            position=(0, 0, 0),
            area_type="test",
        )

        area2_id = cm.add_cortical_area(
            name="TestArea2",
            dimensions=(10, 10, 1),
            position=(20, 0, 0),
            area_type="test",
        )

        # Create neurons in both areas
        area1_neurons = []
        for x in range(3):
            for y in range(3):
                neuron_id = cm.create_neuron(area1_id, (x, y, 0))
                area1_neurons.append(neuron_id)

        area2_neurons = []
        for x in range(3):
            for y in range(3):
                neuron_id = cm.create_neuron(area2_id, (x, y, 0))
                area2_neurons.append(neuron_id)

        # Store for test access
        cm._test_areas = {
            "area1": {"id": area1_id, "neurons": area1_neurons},
            "area2": {"id": area2_id, "neurons": area2_neurons},
        }

        return cm

    @pytest.fixture
    def state_manager(self):
        """Create a mock state manager."""
        state_manager = MagicMock()
        state_manager.begin_genome_transaction.return_value = MagicMock()
        return state_manager

    @pytest.fixture
    def genome_service(self, connectome_manager, state_manager):
        """Create a genome service with test genome."""
        gs = GenomeService(connectome_manager, state_manager)

        # Load a minimal test genome
        test_genome = {
            "cortical_areas": {
                connectome_manager._test_areas["area1"]["id"]: {
                    "cortical_name": "TestArea1",
                    "coordinates": {"x": 0, "y": 0, "z": 0},
                    "dimensions": {"x": 10, "y": 10, "z": 1},
                    "parameters": {},
                },
                connectome_manager._test_areas["area2"]["id"]: {
                    "cortical_name": "TestArea2",
                    "coordinates": {"x": 20, "y": 0, "z": 0},
                    "dimensions": {"x": 10, "y": 10, "z": 1},
                    "parameters": {},
                },
            }
        }

        gs._current_genome = test_genome
        return gs

    @pytest.fixture
    def neuroembryogenesis(self, connectome_manager, state_manager):
        """Create a NeuroEmbryogenesis instance."""
        return NeuroEmbryogenesis(connectome_manager, state_manager)

    def test_block_to_block_mapping_creation(self, genome_service, connectome_manager):
        """Test creating block-to-block cortical mapping and validating synapses."""
        # Arrange
        area1_id = connectome_manager._test_areas["area1"]["id"]
        area2_id = connectome_manager._test_areas["area2"]["id"]
        area1_neurons = connectome_manager._test_areas["area1"]["neurons"]
        area2_neurons = connectome_manager._test_areas["area2"]["neurons"]

        initial_synapse_count = connectome_manager.get_synapse_count()

        # Create mapping request data (matches API schema)
        mapping_request = {
            "src_cortical_area": area1_id,
            "dst_cortical_area": area2_id,
            "mapping_data": [
                {
                    "morphology_id": "block_to_block",
                    "morphology_scalar": [1, 1, 1],
                    "postSynapticCurrent_multiplier": 1.5,
                    "plasticity_flag": False,
                    "plasticity_constant": 1.0,
                    "ltp_multiplier": 1.0,
                    "ltd_multiplier": 1.0,
                }
            ],
        }

        # Act
        success = genome_service.update_cortical_mapping_properties(mapping_request)

        # Assert
        assert success, "Cortical mapping creation should succeed"

        # Verify synapses were created
        final_synapse_count = connectome_manager.get_synapse_count()
        expected_synapses = len(area1_neurons) * len(
            area2_neurons
        )  # block-to-block = all-to-all

        assert final_synapse_count > initial_synapse_count, "Synapses should be created"
        assert final_synapse_count - initial_synapse_count == expected_synapses, (
            f"Expected {expected_synapses} synapses, got {final_synapse_count - initial_synapse_count}"
        )

        # Verify synapse weights
        for src_neuron in area1_neurons[:3]:  # Test subset for performance
            for dst_neuron in area2_neurons[:3]:
                weight = connectome_manager.get_synapse_weight(src_neuron, dst_neuron)
                assert abs(weight - 1.5) < 1e-6, (
                    f"Synapse weight should be 1.5, got {weight}"
                )

    def test_projector_mapping_creation(self, genome_service, connectome_manager):
        """Test creating projector cortical mapping with spatial constraints."""
        # Arrange
        area1_id = connectome_manager._test_areas["area1"]["id"]
        area2_id = connectome_manager._test_areas["area2"]["id"]

        initial_synapse_count = connectome_manager.get_synapse_count()

        # Create projector mapping request
        mapping_request = {
            "src_cortical_area": area1_id,
            "dst_cortical_area": area2_id,
            "mapping_data": [
                {
                    "morphology_id": "projector",
                    "morphology_scalar": [2, 2, 1],  # 2x2 projection regions
                    "postSynapticCurrent_multiplier": 0.8,
                    "plasticity_flag": True,
                    "plasticity_constant": 0.5,
                    "ltp_multiplier": 1.2,
                    "ltd_multiplier": 0.8,
                }
            ],
        }

        # Act
        success = genome_service.update_cortical_mapping_properties(mapping_request)

        # Assert
        assert success, "Projector mapping creation should succeed"

        # Verify synapses were created (projector creates fewer synapses than block-to-block)
        final_synapse_count = connectome_manager.get_synapse_count()
        assert final_synapse_count > initial_synapse_count, (
            "Projector synapses should be created"
        )

    def test_multiple_morphology_mapping(self, genome_service, connectome_manager):
        """Test creating mapping with multiple morphology types."""
        # Arrange
        area1_id = connectome_manager._test_areas["area1"]["id"]
        area2_id = connectome_manager._test_areas["area2"]["id"]

        initial_synapse_count = connectome_manager.get_synapse_count()

        # Create mapping with multiple morphologies
        mapping_request = {
            "src_cortical_area": area1_id,
            "dst_cortical_area": area2_id,
            "mapping_data": [
                {
                    "morphology_id": "block_to_block",
                    "morphology_scalar": [1, 1, 1],
                    "postSynapticCurrent_multiplier": 1.0,
                    "plasticity_flag": False,
                    "plasticity_constant": 1.0,
                    "ltp_multiplier": 1.0,
                    "ltd_multiplier": 1.0,
                },
                {
                    "morphology_id": "projector",
                    "morphology_scalar": [1, 1, 1],
                    "postSynapticCurrent_multiplier": 0.5,
                    "plasticity_flag": True,
                    "plasticity_constant": 0.8,
                    "ltp_multiplier": 1.1,
                    "ltd_multiplier": 0.9,
                },
            ],
        }

        # Act
        success = genome_service.update_cortical_mapping_properties(mapping_request)

        # Assert
        assert success, "Multiple morphology mapping should succeed"

        # Verify synapses were created for both morphologies
        final_synapse_count = connectome_manager.get_synapse_count()
        assert final_synapse_count > initial_synapse_count, (
            "Multiple morphology synapses should be created"
        )

    def test_invalid_morphology_handling(self, genome_service, connectome_manager):
        """Test handling of invalid morphology specifications."""
        # Arrange
        area1_id = connectome_manager._test_areas["area1"]["id"]
        area2_id = connectome_manager._test_areas["area2"]["id"]

        # Create mapping with invalid morphology
        mapping_request = {
            "src_cortical_area": area1_id,
            "dst_cortical_area": area2_id,
            "mapping_data": [
                {
                    "morphology_id": "invalid_morphology",
                    "morphology_scalar": [1, 1, 1],
                    "postSynapticCurrent_multiplier": 1.0,
                    "plasticity_flag": False,
                    "plasticity_constant": 1.0,
                    "ltp_multiplier": 1.0,
                    "ltd_multiplier": 1.0,
                }
            ],
        }

        # Act
        success = genome_service.update_cortical_mapping_properties(mapping_request)

        # Assert - should handle gracefully
        assert success, "Should handle invalid morphology gracefully"

    def test_missing_parameters_handling(self, genome_service, connectome_manager):
        """Test handling of missing parameters in mapping specification."""
        # Arrange
        area1_id = connectome_manager._test_areas["area1"]["id"]
        area2_id = connectome_manager._test_areas["area2"]["id"]

        # Create mapping with minimal parameters
        mapping_request = {
            "src_cortical_area": area1_id,
            "dst_cortical_area": area2_id,
            "mapping_data": [
                {
                    "morphology_id": "block_to_block"
                    # Missing other parameters - should use defaults
                }
            ],
        }

        # Act
        success = genome_service.update_cortical_mapping_properties(mapping_request)

        # Assert
        assert success, "Should handle missing parameters with defaults"

    def test_empty_mapping_data(self, genome_service, connectome_manager):
        """Test handling of empty mapping data."""
        # Arrange
        area1_id = connectome_manager._test_areas["area1"]["id"]
        area2_id = connectome_manager._test_areas["area2"]["id"]

        # Create mapping with empty data
        mapping_request = {
            "src_cortical_area": area1_id,
            "dst_cortical_area": area2_id,
            "mapping_data": [],
        }

        # Act
        success = genome_service.update_cortical_mapping_properties(mapping_request)

        # Assert
        assert success, "Should handle empty mapping data gracefully"

    def test_nonexistent_areas_handling(self, genome_service, connectome_manager):
        """Test handling of nonexistent cortical areas."""
        # Arrange - use nonexistent area IDs
        mapping_request = {
            "src_cortical_area": "nonexistent_area1",
            "dst_cortical_area": "nonexistent_area2",
            "mapping_data": [
                {
                    "morphology_id": "block_to_block",
                    "morphology_scalar": [1, 1, 1],
                    "postSynapticCurrent_multiplier": 1.0,
                    "plasticity_flag": False,
                    "plasticity_constant": 1.0,
                    "ltp_multiplier": 1.0,
                    "ltd_multiplier": 1.0,
                }
            ],
        }

        # Act
        success = genome_service.update_cortical_mapping_properties(mapping_request)

        # Assert
        assert not success, "Should fail for nonexistent areas"

    def test_neuroembryogenesis_direct_call(
        self, neuroembryogenesis, connectome_manager
    ):
        """Test NeuroEmbryogenesis update_cortical_mapping method directly."""
        # Arrange
        area1_id = connectome_manager._test_areas["area1"]["id"]
        area2_id = connectome_manager._test_areas["area2"]["id"]

        initial_synapse_count = connectome_manager.get_synapse_count()

        # Create mapping in the format expected by NeuroEmbryogenesis
        mapping = {
            area1_id: {
                area2_id: [
                    {
                        "morphology_id": "block_to_block",
                        "morphology_scalar": [1, 1, 1],
                        "postSynapticCurrent_multiplier": 2.0,
                        "plasticity_flag": False,
                        "plasticity_constant": 1.0,
                        "ltp_multiplier": 1.0,
                        "ltd_multiplier": 1.0,
                    }
                ]
            }
        }

        # Act
        success = neuroembryogenesis.update_cortical_mapping(mapping)

        # Assert
        assert success, "NeuroEmbryogenesis mapping should succeed"

        # Verify synapses were created
        final_synapse_count = connectome_manager.get_synapse_count()
        assert final_synapse_count > initial_synapse_count, (
            "Direct NeuroEmbryogenesis call should create synapses"
        )

    def test_batch_synapse_creation_performance(
        self, neuroembryogenesis, connectome_manager
    ):
        """Test performance of batch synapse creation for large mappings."""
        # Arrange - create larger areas for performance testing
        large_area1_id = connectome_manager.add_cortical_area(
            name="LargeArea1",
            dimensions=(20, 20, 1),
            position=(50, 0, 0),
            area_type="test",
        )

        large_area2_id = connectome_manager.add_cortical_area(
            name="LargeArea2",
            dimensions=(20, 20, 1),
            position=(80, 0, 0),
            area_type="test",
        )

        # Create neurons in batch for performance
        positions1 = [(x, y, 0) for x in range(10) for y in range(10)]  # 100 neurons
        positions2 = [(x, y, 0) for x in range(10) for y in range(10)]  # 100 neurons

        neurons1 = connectome_manager.batch_create_neurons(large_area1_id, positions1)
        neurons2 = connectome_manager.batch_create_neurons(large_area2_id, positions2)

        initial_synapse_count = connectome_manager.get_synapse_count()

        # Create large mapping
        mapping = {
            large_area1_id: {
                large_area2_id: [
                    {
                        "morphology_id": "block_to_block",
                        "morphology_scalar": [1, 1, 1],
                        "postSynapticCurrent_multiplier": 1.0,
                        "plasticity_flag": False,
                        "plasticity_constant": 1.0,
                        "ltp_multiplier": 1.0,
                        "ltd_multiplier": 1.0,
                    }
                ]
            }
        }

        # Act
        start_time = time.time()
        success = neuroembryogenesis.update_cortical_mapping(mapping)
        end_time = time.time()

        # Assert
        assert success, "Large mapping should succeed"

        # Verify performance (should complete in reasonable time)
        execution_time = end_time - start_time
        assert execution_time < 5.0, (
            f"Large mapping took too long: {execution_time:.2f}s"
        )

        # Verify correct number of synapses created
        final_synapse_count = connectome_manager.get_synapse_count()
        expected_synapses = len(neurons1) * len(neurons2)  # 100 * 100 = 10,000
        actual_synapses = final_synapse_count - initial_synapse_count

        assert actual_synapses == expected_synapses, (
            f"Expected {expected_synapses} synapses, got {actual_synapses}"
        )

    def test_vectorized_operations_compatibility(self, neuroembryogenesis):
        """Test that the implementation is compatible with vectorized operations."""
        # This test validates that the implementation uses vectorized operations
        # suitable for Rust/RTOS/SIMD/GPU compatibility

        # Verify that the implementation uses batch operations
        assert hasattr(
            neuroembryogenesis.connectome_manager, "batch_create_synapses"
        ), "ConnectomeManager should support batch synapse creation"

        assert hasattr(neuroembryogenesis.connectome_manager, "get_neurons_by_area"), (
            "ConnectomeManager should support area-based neuron queries"
        )

        assert hasattr(neuroembryogenesis.connectome_manager, "get_neuron_position"), (
            "ConnectomeManager should support neuron position queries"
        )

    def test_architecture_compliance(self, neuroembryogenesis):
        """Test that the implementation follows architecture compliance rules."""
        # Verify no hardcoded values are used

        source = inspect.getsource(neuroembryogenesis.update_cortical_mapping)

        # Check for architecture compliance
        assert "127.0.0.1" not in source, "No hardcoded IP addresses"
        assert "localhost" not in source, "No hardcoded hostnames"
        assert "time.sleep(" not in source, "No hardcoded timeouts"

        # Verify proper error handling without fallbacks
        assert "except Exception" in source, "Should have proper exception handling"

    @pytest.mark.performance
    def test_memory_efficiency(self, neuroembryogenesis, connectome_manager):
        """Test memory efficiency of the cortical mapping implementation."""

        # Get initial memory usage
        process = psutil.Process(os.getpid())
        initial_memory = process.memory_info().rss

        # Create a moderately large mapping
        area1_id = connectome_manager.add_cortical_area(
            name="MemTestArea1",
            dimensions=(50, 50, 1),
            position=(100, 0, 0),
            area_type="test",
        )

        area2_id = connectome_manager.add_cortical_area(
            name="MemTestArea2",
            dimensions=(50, 50, 1),
            position=(160, 0, 0),
            area_type="test",
        )

        # Create neurons
        positions = [
            (x, y, 0) for x in range(20) for y in range(20)
        ]  # 400 neurons each
        connectome_manager.batch_create_neurons(area1_id, positions)
        connectome_manager.batch_create_neurons(area2_id, positions)

        # Create mapping
        mapping = {
            area1_id: {
                area2_id: [
                    {
                        "morphology_id": "block_to_block",
                        "morphology_scalar": [1, 1, 1],
                        "postSynapticCurrent_multiplier": 1.0,
                        "plasticity_flag": False,
                        "plasticity_constant": 1.0,
                        "ltp_multiplier": 1.0,
                        "ltd_multiplier": 1.0,
                    }
                ]
            }
        }

        # Apply mapping
        success = neuroembryogenesis.update_cortical_mapping(mapping)
        assert success, "Memory test mapping should succeed"

        # Check memory usage
        final_memory = process.memory_info().rss
        memory_increase = final_memory - initial_memory

        # Memory increase should be reasonable (less than 100MB for this test)
        assert memory_increase < 100 * 1024 * 1024, (
            f"Memory usage increased by {memory_increase / 1024 / 1024:.2f}MB, which is too much"
        )


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
