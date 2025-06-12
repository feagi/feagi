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

"""Multi-GPU support for Brain Development Unit.

This module provides classes and utilities for distributing BDU computation
across multiple GPUs, including domain decomposition strategies and
communication protocols.
"""

import logging
import os
import time
from enum import Enum
from typing import Any, Dict, List, Optional, Set, Tuple, Union

import numpy as np

# Conditional imports for different backend support
try:
    import torch
    import torch.distributed as dist

    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

try:
    import cupy as cp

    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False

try:
    import wgpu

    WGPU_AVAILABLE = True
except ImportError:
    WGPU_AVAILABLE = False

from feagi.bdu.models.array_backend import BackendType

logger = logging.getLogger(__name__)


class PartitionMethod(Enum):
    """Methods for partitioning the brain across multiple GPUs."""

    CORTICAL_AREAS = "cortical_areas"  # Partition by cortical areas
    BALANCED = "balanced"  # Balanced partitioning by workload
    GRID = "grid"  # Grid-based spatial partitioning
    CUSTOM = "custom"  # Custom partitioning function


class SyncMode(Enum):
    """Synchronization modes for multi-GPU operation."""

    TIMESTEP = "timestep"  # Synchronize after each timestep
    BATCH = "batch"  # Synchronize after a batch of timesteps
    ADAPTIVE = "adaptive"  # Adaptive synchronization based on activity


class MultiGPUConfig:
    """Configuration for multi-GPU operation."""

    def __init__(
        self,
        enabled: bool = False,
        num_devices: int = 0,  # 0 means auto-detect
        device_ids: Optional[List[int]] = None,
        partition_method: Union[str, PartitionMethod] = PartitionMethod.CORTICAL_AREAS,
        sync_mode: Union[str, SyncMode] = SyncMode.TIMESTEP,
        sync_frequency: int = 1,
        backend_type: Union[str, BackendType] = BackendType.PYTORCH,
        communication_optimization: bool = True,
    ):
        """Initialize multi-GPU configuration.

        Args:
            enabled: Whether multi-GPU operation is enabled
            num_devices: Number of devices to use (0 means auto-detect)
            device_ids: Specific device IDs to use (None means auto-select)
            partition_method: Method for partitioning the brain
            sync_mode: Synchronization mode
            sync_frequency: Synchronization frequency (in timesteps)
            backend_type: Backend type to use
            communication_optimization: Whether to optimize communication
        """
        self.enabled = enabled

        # Convert string enum values to enum types if needed
        if isinstance(partition_method, str):
            try:
                self.partition_method = PartitionMethod(partition_method)
            except ValueError:
                logger.warning(
                    f"Unknown partition method: {partition_method}. Using CORTICAL_AREAS."
                )
                self.partition_method = PartitionMethod.CORTICAL_AREAS
        else:
            self.partition_method = partition_method

        if isinstance(sync_mode, str):
            try:
                self.sync_mode = SyncMode(sync_mode)
            except ValueError:
                logger.warning(f"Unknown sync mode: {sync_mode}. Using TIMESTEP.")
                self.sync_mode = SyncMode.TIMESTEP
        else:
            self.sync_mode = sync_mode

        if isinstance(backend_type, str):
            try:
                self.backend_type = BackendType(backend_type)
            except ValueError:
                logger.warning(f"Unknown backend type: {backend_type}. Using PYTORCH.")
                self.backend_type = BackendType.PYTORCH
        else:
            self.backend_type = backend_type

        # Set the number of devices
        if num_devices == 0:
            # Auto-detect available devices
            if self.backend_type == BackendType.PYTORCH and TORCH_AVAILABLE:
                if torch.cuda.is_available():
                    self.num_devices = torch.cuda.device_count()
                else:
                    self.num_devices = 1  # CPU only
            elif self.backend_type == BackendType.CUPY and CUPY_AVAILABLE:
                try:
                    self.num_devices = cp.cuda.runtime.getDeviceCount()
                except:
                    self.num_devices = 1  # Error or CPU only
            elif self.backend_type == BackendType.WEBGPU and WGPU_AVAILABLE:
                self.num_devices = 1  # WebGPU currently treats all GPUs as one
            else:
                self.num_devices = 1  # Default to single device
        else:
            self.num_devices = num_devices

        # Use provided device IDs or auto-select
        self.device_ids = device_ids or list(range(self.num_devices))
        if len(self.device_ids) > self.num_devices:
            logger.warning(
                f"More device IDs provided than available devices. Using only the first {self.num_devices}."
            )
            self.device_ids = self.device_ids[: self.num_devices]

        self.sync_frequency = sync_frequency
        self.communication_optimization = communication_optimization

        # Verify configuration is valid
        self._validate_config()

    def _validate_config(self):
        """Validate the configuration."""
        # Check backend availability
        if self.backend_type == BackendType.PYTORCH and not TORCH_AVAILABLE:
            logger.warning("PyTorch is not available. Disabling multi-GPU operation.")
            self.enabled = False
        elif self.backend_type == BackendType.CUPY and not CUPY_AVAILABLE:
            logger.warning("CuPy is not available. Disabling multi-GPU operation.")
            self.enabled = False
        elif self.backend_type == BackendType.WEBGPU and not WGPU_AVAILABLE:
            logger.warning("WebGPU is not available. Disabling multi-GPU operation.")
            self.enabled = False

        # Check number of devices
        if self.num_devices < 2:
            logger.warning(
                "Fewer than 2 devices available. Disabling multi-GPU operation."
            )
            self.enabled = False

        # Validate sync frequency
        if self.sync_frequency < 1:
            logger.warning("Sync frequency must be at least 1. Setting to 1.")
            self.sync_frequency = 1


class BrainPartition:
    """Represents a partition of the brain for multi-GPU processing."""

    def __init__(
        self,
        partition_id: int,
        device_id: int,
        total_partitions: int,
        backend_type: BackendType,
    ):
        """Initialize a brain partition.

        Args:
            partition_id: ID of this partition
            device_id: GPU device ID to use
            total_partitions: Total number of partitions
            backend_type: Backend type to use
        """
        self.partition_id = partition_id
        self.device_id = device_id
        self.total_partitions = total_partitions
        self.backend_type = backend_type

        # Areas and neurons assigned to this partition
        self.area_ids: Set[str] = set()
        self.neuron_ids: Set[int] = set()

        # Boundary connections (connections to neurons in other partitions)
        self.boundary_outgoing: Dict[int, List[Tuple[int, float]]] = (
            {}
        )  # source_id -> [(target_id, weight), ...]
        self.boundary_incoming: Dict[int, List[Tuple[int, float]]] = (
            {}
        )  # target_id -> [(source_id, weight), ...]

        # Performance metrics
        self.metrics = {
            "computation_time": 0.0,
            "communication_time": 0.0,
            "active_neurons": 0,
            "total_neurons": 0,
            "boundary_connections": 0,
        }

        # Initialize device
        self._initialize_device()

    def _initialize_device(self):
        """Initialize the device for this partition."""
        if self.backend_type == BackendType.PYTORCH and TORCH_AVAILABLE:
            # Set device for PyTorch
            if torch.cuda.is_available() and self.device_id < torch.cuda.device_count():
                self.device = torch.device(f"cuda:{self.device_id}")
                logger.info(
                    f"Partition {self.partition_id} using CUDA device {self.device_id}: {torch.cuda.get_device_name(self.device_id)}"
                )
            else:
                self.device = torch.device("cpu")
                logger.info(f"Partition {self.partition_id} using CPU")
        elif self.backend_type == BackendType.CUPY and CUPY_AVAILABLE:
            # Set device for CuPy
            cp.cuda.Device(self.device_id).use()
            self.device = cp.cuda.Device(self.device_id)
            logger.info(
                f"Partition {self.partition_id} using CUDA device {self.device_id}: {cp.cuda.runtime.getDeviceProperties(self.device_id)['name'].decode()}"
            )
        elif self.backend_type == BackendType.WEBGPU and WGPU_AVAILABLE:
            # No explicit device selection for WebGPU yet
            self.adapter = wgpu.request_adapter()
            self.device = self.adapter.request_device()
            logger.info(
                f"Partition {self.partition_id} using WebGPU device: {self.adapter.request_adapter_info().description}"
            )
        else:
            # CPU fallback
            self.device = "cpu"
            logger.info(f"Partition {self.partition_id} using CPU")

    def add_area(self, area_id: str):
        """Add a cortical area to this partition.

        Args:
            area_id: ID of the cortical area to add
        """
        self.area_ids.add(area_id)
        logger.debug(f"Added area {area_id} to partition {self.partition_id}")

    def add_neuron(self, neuron_id: int):
        """Add a neuron to this partition.

        Args:
            neuron_id: ID of the neuron to add
        """
        self.neuron_ids.add(neuron_id)
        self.metrics["total_neurons"] = len(self.neuron_ids)

    def add_boundary_connection(self, source_id: int, target_id: int, weight: float):
        """Add a boundary connection (connection to a neuron in another partition).

        Args:
            source_id: ID of the source neuron
            target_id: ID of the target neuron
            weight: Weight of the connection
        """
        if source_id in self.neuron_ids:
            # This is an outgoing connection
            if source_id not in self.boundary_outgoing:
                self.boundary_outgoing[source_id] = []
            self.boundary_outgoing[source_id].append((target_id, weight))
        else:
            # This is an incoming connection
            if target_id not in self.boundary_incoming:
                self.boundary_incoming[target_id] = []
            self.boundary_incoming[target_id].append((source_id, weight))

        self.metrics["boundary_connections"] = sum(
            len(targets) for targets in self.boundary_outgoing.values()
        )


class MultiGPUManager:
    """Manager for multi-GPU operation in the BDU."""

    def __init__(self, config: MultiGPUConfig):
        """Initialize the multi-GPU manager.

        Args:
            config: Multi-GPU configuration
        """
        self.config = config
        self.partitions: List[BrainPartition] = []
        self.initialized = False

        # Maps for looking up partition ID by area/neuron
        self.area_to_partition: Dict[str, int] = {}
        self.neuron_to_partition: Dict[int, int] = {}

        # Reference to the connectome manager (set during initialize)
        self.connectome = None

        # Initialize distributed backend if enabled
        if self.config.enabled:
            self._initialize_distributed()

    def _initialize_distributed(self):
        """Initialize distributed backend for communication."""
        if not self.config.enabled:
            return

        if self.config.backend_type == BackendType.PYTORCH and TORCH_AVAILABLE:
            # Initialize PyTorch distributed
            if not dist.is_initialized():
                try:
                    # Use NCCL backend for GPU, gloo for CPU
                    backend = "nccl" if torch.cuda.is_available() else "gloo"
                    if torch.cuda.is_available():
                        # Use configuration system for distributed training hosts
                        from feagi.config.toml_loader import (
                            get_host_config,
                            load_feagi_config,
                        )

                        config = load_feagi_config()
                        host_config = get_host_config(config)

                        os.environ["MASTER_ADDR"] = (
                            host_config.zmq_host
                        )  # Use configured host instead of hardcoded
                        os.environ["MASTER_PORT"] = "29500"
                        dist.init_process_group(backend=backend, rank=0, world_size=1)
                        logger.info(
                            f"Initialized PyTorch distributed with {backend} backend on {host_config.zmq_host}"
                        )
                    else:
                        logger.warning(
                            "CUDA not available for PyTorch distributed. Using single device."
                        )
                except Exception as e:
                    logger.error(f"Failed to initialize PyTorch distributed: {e}")
                    self.config.enabled = False

        elif self.config.backend_type == BackendType.CUPY and CUPY_AVAILABLE:
            # CuPy doesn't have a built-in distributed API
            # We'll implement custom communication functions
            pass

        elif self.config.backend_type == BackendType.WEBGPU and WGPU_AVAILABLE:
            # WebGPU doesn't have a built-in distributed API yet
            # We'll implement custom communication functions
            pass

    def initialize(self, connectome):
        """Initialize the multi-GPU manager with the connectome manager.

        Args:
            connectome: ConnectomeManagerGPU instance
        """
        if self.initialized:
            return

        self.connectome = connectome

        if not self.config.enabled:
            logger.info("Multi-GPU operation is disabled.")
            return

        # Create partitions
        for i in range(self.config.num_devices):
            partition = BrainPartition(
                partition_id=i,
                device_id=self.config.device_ids[i],
                total_partitions=self.config.num_devices,
                backend_type=self.config.backend_type,
            )
            self.partitions.append(partition)

        # Partition the brain
        self._partition_brain()

        # Find boundary connections
        self._find_boundary_connections()

        self.initialized = True
        logger.info(
            f"Initialized multi-GPU manager with {len(self.partitions)} partitions"
        )

    def _partition_brain(self):
        """Partition the brain across GPUs."""
        if self.config.partition_method == PartitionMethod.CORTICAL_AREAS:
            self._partition_by_cortical_areas()
        elif self.config.partition_method == PartitionMethod.BALANCED:
            self._partition_balanced()
        elif self.config.partition_method == PartitionMethod.GRID:
            self._partition_grid()
        elif self.config.partition_method == PartitionMethod.CUSTOM:
            # Use custom partitioning function
            logger.warning("Custom partitioning not implemented. Using cortical areas.")
            self._partition_by_cortical_areas()

    def _partition_by_cortical_areas(self):
        """Partition the brain by cortical areas."""
        # Get all cortical areas
        area_ids = list(self.connectome.cortical_areas.keys())

        # Simple round-robin assignment
        for i, area_id in enumerate(area_ids):
            partition_id = i % len(self.partitions)
            self.partitions[partition_id].add_area(area_id)
            self.area_to_partition[area_id] = partition_id

        # Assign neurons based on their areas using vectorized operations
        neuron_ids = list(self.connectome.neuron_id_to_index.keys())
        if neuron_ids:
            # Vectorized area lookup for all neurons
            area_ids_for_neurons = [
                self.connectome.get_area_for_neuron(nid) for nid in neuron_ids
            ]

            # Vectorized partition assignment
            for neuron_id, area_id in zip(neuron_ids, area_ids_for_neurons):
                if area_id in self.area_to_partition:
                    partition_id = self.area_to_partition[area_id]
                    self.partitions[partition_id].add_neuron(neuron_id)
                    self.neuron_to_partition[neuron_id] = partition_id

    def _partition_balanced(self):
        """Partition the brain with balanced workload."""
        # Get all neurons
        neuron_ids = list(self.connectome.neuron_id_to_index.keys())

        # Calculate workload for each neuron
        workloads = {}
        for neuron_id in neuron_ids:
            # Count outgoing connections as workload
            outgoing = self.connectome.get_outgoing_connections(neuron_id)
            workloads[neuron_id] = len(outgoing)

        # Sort neurons by workload (descending)
        sorted_neurons = sorted(neuron_ids, key=lambda n: workloads[n], reverse=True)

        # Initialize partition workloads
        partition_workloads = [0] * len(self.partitions)

        # Assign neurons to partitions using greedy algorithm
        for neuron_id in sorted_neurons:
            # Find partition with minimum workload
            min_workload_idx = partition_workloads.index(min(partition_workloads))

            # Assign neuron to this partition
            self.partitions[min_workload_idx].add_neuron(neuron_id)
            self.neuron_to_partition[neuron_id] = min_workload_idx

            # Update partition workload
            partition_workloads[min_workload_idx] += workloads[neuron_id]

            # Also assign the area if not already assigned
            area_id = self.connectome.get_area_for_neuron(neuron_id)
            if area_id not in self.area_to_partition:
                self.area_to_partition[area_id] = min_workload_idx
                self.partitions[min_workload_idx].add_area(area_id)

    def _partition_grid(self):
        """Partition the brain using a grid-based spatial approach."""
        # This is a simple implementation that divides the brain into spatial regions

        # Get all neurons and their positions
        neuron_positions = {}
        for neuron_id in self.connectome.neuron_id_to_index.keys():
            position = self.connectome.get_neuron_position(neuron_id)
            area_id = self.connectome.get_area_for_neuron(neuron_id)
            area = self.connectome.cortical_areas[area_id]

            # Convert to global coordinates
            global_position = tuple(p + a for p, a in zip(position, area.position))
            neuron_positions[neuron_id] = global_position

        # Find the bounding box of all neurons
        if not neuron_positions:
            # No neurons yet
            return

        min_coords = [
            min(pos[i] for pos in neuron_positions.values()) for i in range(3)
        ]
        max_coords = [
            max(pos[i] for pos in neuron_positions.values()) for i in range(3)
        ]

        # Divide the space into regions based on number of partitions
        # For simplicity, we'll just divide along the first dimension
        dimension = 0  # X dimension
        region_size = (max_coords[dimension] - min_coords[dimension]) / len(
            self.partitions
        )

        # Assign neurons to partitions using vectorized operations
        neuron_ids = list(neuron_positions.keys())
        positions = np.array(list(neuron_positions.values()))

        # Vectorized region calculation
        region_indices = (
            (positions[:, dimension] - min_coords[dimension]) / region_size
        ).astype(int)
        region_indices = np.clip(region_indices, 0, len(self.partitions) - 1)

        # Bulk assignment
        for neuron_id, region_idx in zip(neuron_ids, region_indices):
            region_idx = int(region_idx)  # Convert from numpy int

            # Assign neuron to partition
            self.partitions[region_idx].add_neuron(neuron_id)
            self.neuron_to_partition[neuron_id] = region_idx

            # Also assign the area if not already assigned
            area_id = self.connectome.get_area_for_neuron(neuron_id)
            if area_id not in self.area_to_partition:
                self.area_to_partition[area_id] = region_idx
                self.partitions[region_idx].add_area(area_id)

    def _find_boundary_connections(self):
        """Find boundary connections between partitions."""
        for neuron_id in self.neuron_to_partition:
            source_partition = self.neuron_to_partition[neuron_id]

            # Get outgoing connections
            outgoing = self.connectome.get_outgoing_connections(neuron_id)
            for target_id, weight in outgoing:
                if target_id in self.neuron_to_partition:
                    target_partition = self.neuron_to_partition[target_id]

                    # If connection crosses partition boundary
                    if source_partition != target_partition:
                        # Add to boundary connections
                        self.partitions[source_partition].add_boundary_connection(
                            neuron_id, target_id, weight
                        )

    def update_membrane_potentials(self, current_timestep=None):
        """Update membrane potentials across all partitions.

        Args:
            current_timestep: Current simulation timestep

        Returns:
            List of neurons that fired
        """
        if not self.config.enabled or not self.initialized:
            # Fall back to single-GPU implementation
            return self.connectome.update_membrane_potentials(current_timestep)

        # Determine if synchronization is needed
        sync_needed = False
        if self.config.sync_mode == SyncMode.TIMESTEP:
            sync_needed = True
        elif self.config.sync_mode == SyncMode.BATCH:
            if current_timestep % self.config.sync_frequency == 0:
                sync_needed = True
        # For adaptive sync, we'd check activity levels

        # Update each partition
        all_fired_neurons = []
        for partition in self.partitions:
            # TODO: Implement actual distributed update
            # For now, this is just a placeholder

            # In a real implementation, each partition would:
            # 1. Update its own neurons
            # 2. Collect fired neurons
            # 3. Exchange boundary information if sync_needed
            pass

        # Combine results and return
        return all_fired_neurons

    def exchange_fcl(self):
        """Exchange fire candidate lists between partitions."""
        if not self.config.enabled or not self.initialized:
            return

        # TODO: Implement FCL exchange using appropriate backend
        # For PyTorch, we'd use all_gather
        # For other backends, we'd implement custom communication
        pass

    def synchronize(self):
        """Synchronize all devices."""
        if not self.config.enabled or not self.initialized:
            return

        if self.config.backend_type == BackendType.PYTORCH and TORCH_AVAILABLE:
            for device_id in self.config.device_ids:
                if torch.cuda.is_available():
                    torch.cuda.synchronize(device_id)
        elif self.config.backend_type == BackendType.CUPY and CUPY_AVAILABLE:
            for partition in self.partitions:
                cp.cuda.Device(partition.device_id).synchronize()
        # WebGPU doesn't have an explicit synchronize method yet
