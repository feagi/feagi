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

"""Tests for the multi-GPU functionality."""

import pytest
import numpy as np
import os
from unittest.mock import patch, MagicMock

from feagi.bdu.multi_gpu import (
    MultiGPUConfig, 
    PartitionMethod, 
    SyncMode, 
    MultiGPUManager,
    BrainPartition
)
from feagi.bdu.models.array_backend import BackendType
from feagi.bdu.connectome_manager_gpu import ConnectomeManagerGPU
from feagi.bdu.models.cortical_area import CorticalArea

# Define area types for testing
SENSORY_AREA = "sensory"
ASSOCIATION_AREA = "association"

class TestMultiGPUConfig:
    """Test the MultiGPUConfig class."""
    
    def test_initialization(self):
        """Test that the config can be initialized with different options."""
        # Test default initialization
        config = MultiGPUConfig()
        assert config.enabled is False
        assert config.partition_method == PartitionMethod.CORTICAL_AREAS
        assert config.sync_mode == SyncMode.TIMESTEP
        assert config.sync_frequency == 1
        
        # Test with explicit parameters
        config = MultiGPUConfig(
            enabled=True,
            num_devices=2,
            device_ids=[0, 1],
            partition_method=PartitionMethod.BALANCED,
            sync_mode=SyncMode.BATCH,
            sync_frequency=10,
            backend_type=BackendType.PYTORCH,
            communication_optimization=True
        )
        assert config.enabled is True
        assert config.num_devices == 2
        assert config.device_ids == [0, 1]
        assert config.partition_method == PartitionMethod.BALANCED
        assert config.sync_mode == SyncMode.BATCH
        assert config.sync_frequency == 10
        assert config.backend_type == BackendType.PYTORCH
        assert config.communication_optimization is True
        
        # Test with string parameters
        config = MultiGPUConfig(
            partition_method="balanced",
            sync_mode="batch",
            backend_type="numpy"
        )
        assert config.partition_method == PartitionMethod.BALANCED
        assert config.sync_mode == SyncMode.BATCH
        assert config.backend_type == BackendType.NUMPY
    
    def test_validation(self):
        """Test config validation."""
        # Test invalid sync frequency
        config = MultiGPUConfig(sync_frequency=0)
        assert config.sync_frequency == 1
        
        # Test device_ids truncation when too many provided
        config = MultiGPUConfig(num_devices=2, device_ids=[0, 1, 2, 3])
        assert len(config.device_ids) == 2
        
        # Test device count validation
        with patch('feagi.bdu.multi_gpu.TORCH_AVAILABLE', True):
            with patch('torch.cuda.device_count', return_value=1):
                config = MultiGPUConfig(enabled=True, backend_type=BackendType.PYTORCH)
                assert config.enabled is False  # Should be disabled due to device count


class TestBrainPartition:
    """Test the BrainPartition class."""
    
    def test_initialization(self):
        """Test partition initialization."""
        # Test with NumPy backend (always available)
        partition = BrainPartition(
            partition_id=0,
            device_id=0,
            total_partitions=2,
            backend_type=BackendType.NUMPY
        )
        assert partition.partition_id == 0
        assert partition.device_id == 0
        assert partition.total_partitions == 2
        assert partition.backend_type == BackendType.NUMPY
        assert isinstance(partition.area_ids, set)
        assert isinstance(partition.neuron_ids, set)
        assert isinstance(partition.boundary_outgoing, dict)
        assert isinstance(partition.boundary_incoming, dict)
        
        # Metrics should be initialized
        assert "computation_time" in partition.metrics
        assert "total_neurons" in partition.metrics
        assert "boundary_connections" in partition.metrics
    
    def test_add_area_and_neuron(self):
        """Test adding areas and neurons to a partition."""
        partition = BrainPartition(
            partition_id=0,
            device_id=0,
            total_partitions=2,
            backend_type=BackendType.NUMPY
        )
        
        # Add areas
        partition.add_area("area_1")
        partition.add_area("area_2")
        assert "area_1" in partition.area_ids
        assert "area_2" in partition.area_ids
        assert len(partition.area_ids) == 2
        
        # Add neurons
        partition.add_neuron(101)
        partition.add_neuron(102)
        partition.add_neuron(103)
        assert 101 in partition.neuron_ids
        assert 102 in partition.neuron_ids
        assert 103 in partition.neuron_ids
        assert len(partition.neuron_ids) == 3
        assert partition.metrics["total_neurons"] == 3
    
    def test_add_boundary_connection(self):
        """Test adding boundary connections between partitions."""
        partition = BrainPartition(
            partition_id=0,
            device_id=0,
            total_partitions=2,
            backend_type=BackendType.NUMPY
        )
        
        # Add neurons
        partition.add_neuron(101)
        partition.add_neuron(102)
        
        # Add outgoing connection from partition neuron to external neuron
        partition.add_boundary_connection(101, 201, 0.5)
        assert 101 in partition.boundary_outgoing
        assert (201, 0.5) in partition.boundary_outgoing[101]
        assert partition.metrics["boundary_connections"] == 1
        
        # Add incoming connection from external neuron to partition neuron
        partition.add_boundary_connection(301, 102, 0.8)
        assert 102 in partition.boundary_incoming
        assert (301, 0.8) in partition.boundary_incoming[102]
        # Only outgoing connections count toward the boundary_connections metric
        assert partition.metrics["boundary_connections"] == 1
        
        # Add another outgoing connection
        partition.add_boundary_connection(101, 202, 0.6)
        assert len(partition.boundary_outgoing[101]) == 2
        assert partition.metrics["boundary_connections"] == 2


@pytest.mark.skipif(not os.environ.get('FEAGI_TEST_MULTI_GPU'), 
                  reason="Full multi-GPU tests skipped by default. Set FEAGI_TEST_MULTI_GPU=1 to run.")
class TestMultiGPUManager:
    """Test the MultiGPUManager class with actual data."""
    
    @pytest.fixture
    def setup_connectome(self):
        """Set up a test connectome."""
        connectome = ConnectomeManagerGPU(1000)  # Small connectome for testing
        
        # Add areas
        for i in range(4):
            area = CorticalArea(
                name=f"area_{i}",
                dimensions=(10, 10, 5),
                position=(i * 20, 0, 0),
                area_type=SENSORY_AREA if i == 0 else ASSOCIATION_AREA,
                area_id=f"area_{i}"
            )
            connectome.add_cortical_area(area)
        
        # Add neurons to each area
        neuron_ids = []
        for area_id in connectome.cortical_areas:
            for i in range(10):  # 10 neurons per area
                position = (np.random.randint(0, 10), np.random.randint(0, 10), np.random.randint(0, 5))
                neuron_id = connectome.add_neuron(
                    area_id=area_id,
                    position=position,
                    threshold=0.5,
                    refractory_period=1,
                    decay_rate=0.1
                )
                neuron_ids.append(neuron_id)
        
        # Add some synapses
        for i in range(len(neuron_ids) - 1):
            connectome.add_synapse(neuron_ids[i], neuron_ids[i + 1], 0.8)
        
        return connectome, neuron_ids
    
    def test_initialization(self, setup_connectome):
        """Test initializing the manager with a connectome."""
        connectome, _ = setup_connectome
        
        # Create config
        config = MultiGPUConfig(
            enabled=True,
            num_devices=2,  # Use 2 virtual devices
            backend_type=BackendType.NUMPY  # Use NumPy for reliable testing
        )
        
        # Initialize manager
        manager = MultiGPUManager(config)
        manager.initialize(connectome)
        
        # Check initialization
        assert manager.initialized is True
        assert len(manager.partitions) == 2
        
        # Check partitioning
        assert len(manager.area_to_partition) == 4  # All areas should be assigned
        assert len(manager.neuron_to_partition) == 40  # All neurons should be assigned
        
        # Check boundary connections
        boundary_connections = 0
        for partition in manager.partitions:
            boundary_connections += partition.metrics["boundary_connections"]
        
        # There should be at least some boundary connections
        assert boundary_connections > 0
    
    def test_partition_methods(self, setup_connectome):
        """Test different partitioning methods."""
        connectome, _ = setup_connectome
        
        # Test cortical areas partitioning
        config_areas = MultiGPUConfig(
            enabled=True,
            num_devices=2,
            partition_method=PartitionMethod.CORTICAL_AREAS,
            backend_type=BackendType.NUMPY
        )
        manager_areas = MultiGPUManager(config_areas)
        manager_areas.initialize(connectome)
        
        # Test balanced partitioning
        config_balanced = MultiGPUConfig(
            enabled=True,
            num_devices=2,
            partition_method=PartitionMethod.BALANCED,
            backend_type=BackendType.NUMPY
        )
        manager_balanced = MultiGPUManager(config_balanced)
        manager_balanced.initialize(connectome)
        
        # Test grid partitioning
        config_grid = MultiGPUConfig(
            enabled=True,
            num_devices=2,
            partition_method=PartitionMethod.GRID,
            backend_type=BackendType.NUMPY
        )
        manager_grid = MultiGPUManager(config_grid)
        manager_grid.initialize(connectome)
        
        # All methods should assign all neurons
        assert len(manager_areas.neuron_to_partition) == 40
        assert len(manager_balanced.neuron_to_partition) == 40
        assert len(manager_grid.neuron_to_partition) == 40


# Mock tests that don't require actual GPUs
class TestMultiGPUManagerMock:
    """Mock tests for the MultiGPUManager that don't require actual GPUs."""
    
    def test_update_membrane_potentials_fallback(self):
        """Test that membrane update falls back to connectome's implementation when not initialized."""
        # Create mocks
        mock_connectome = MagicMock()
        mock_connectome.update_membrane_potentials.return_value = [1, 2, 3]
        
        # Create config with multi-GPU disabled
        config = MultiGPUConfig(enabled=False)
        
        # Create manager
        manager = MultiGPUManager(config)
        manager.connectome = mock_connectome
        
        # Call update
        result = manager.update_membrane_potentials(current_timestep=10)
        
        # Should fall back to connectome's implementation
        mock_connectome.update_membrane_potentials.assert_called_once_with(10)
        assert result == [1, 2, 3]
    
    def test_distributed_backend_initialization(self):
        """Test distributed backend initialization with patched dependencies."""
        # Patch PyTorch distributed
        with patch('feagi.bdu.multi_gpu.TORCH_AVAILABLE', True):
            with patch('feagi.bdu.multi_gpu.dist.is_initialized', return_value=False):
                with patch('feagi.bdu.multi_gpu.dist.init_process_group') as mock_init:
                    with patch('feagi.bdu.multi_gpu.torch.cuda.is_available', return_value=True):
                        with patch('feagi.bdu.multi_gpu.torch.cuda.device_count', return_value=2):
                            # Create config with PyTorch backend
                            config = MultiGPUConfig(
                                enabled=True,
                                backend_type=BackendType.PYTORCH
                            )
                            
                            # Create manager
                            manager = MultiGPUManager(config)
                            
                            # Should have initialized PyTorch distributed
                            mock_init.assert_called_once()
    
    def test_synchronization(self):
        """Test the synchronize method."""
        # Mock PyTorch
        with patch('feagi.bdu.multi_gpu.TORCH_AVAILABLE', True):
            with patch('feagi.bdu.multi_gpu.torch.cuda.is_available', return_value=True):
                with patch('feagi.bdu.multi_gpu.torch.cuda.device_count', return_value=2):
                    with patch('feagi.bdu.multi_gpu.torch.cuda.synchronize') as mock_sync:
                        # Create config with PyTorch backend
                        config = MultiGPUConfig(
                            enabled=True,
                            num_devices=2,  # Explicitly specify 2 devices
                            device_ids=[0, 1],
                            backend_type=BackendType.PYTORCH
                        )
                        
                        # Create manager with mocked config
                        manager = MultiGPUManager(config)
                        
                        # Manually set initialized to True and make sure enabled stays True
                        manager.initialized = True
                        manager.config.enabled = True
                        
                        # Call synchronize
                        manager.synchronize()
                        
                        # Should have called synchronize for each device
                        assert mock_sync.call_count == 2
        
        # Note: We're skipping testing the CuPy path since it's more complex to mock
        # properly and the PyTorch path already tests the main functionality 