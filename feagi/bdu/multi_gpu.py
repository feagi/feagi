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

import threading
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

"""Multi-GPU management for the BDU.

This module provides utilities for managing multiple GPU devices
and distributing neural computation across them.
"""


class PartitionMethod(Enum):
    """Methods for partitioning brain regions across GPUs."""

    MEMORY_BASED = "memory_based"
    CORTICAL_AREA = "cortical_area"
    NEURON_COUNT = "neuron_count"
    BALANCED = "balanced"


class SyncMode(Enum):
    """Synchronization modes for multi-GPU processing."""

    ASYNC = "async"
    SYNC = "sync"
    BARRIER = "barrier"
    PIPELINE = "pipeline"


@dataclass
class GPUDevice:
    """Information about a GPU device."""

    device_id: int
    name: str
    memory_total: int
    memory_available: int
    compute_capability: Optional[Tuple[int, int]] = None
    backend_type: str = "unknown"


@dataclass
class BrainPartition:
    """Represents a partition of the brain assigned to a specific GPU."""

    device_id: int
    cortical_areas: List[str]
    neuron_range: Tuple[int, int]  # (start_idx, end_idx)
    synapse_count: int
    memory_usage: int


@dataclass
class MultiGPUConfig:
    """Configuration for multi-GPU processing."""

    enabled: bool = False
    device_count: Optional[int] = None
    backend_preference: Optional[str] = None
    memory_threshold: float = 0.8  # Use up to 80% of GPU memory
    sync_frequency: int = 100  # Synchronize every N timesteps


class MultiGPUManager:
    """Manager for coordinating neural processing across multiple GPUs.

    This class handles:
    - GPU device discovery and selection
    - Workload distribution across GPUs
    - Memory management across devices
    - Synchronization between GPU operations
    """

    def __init__(self, backend_preference: Optional[str] = None):
        """Initialize multi-GPU manager.

        Args:
            backend_preference: Preferred backend ('pytorch', 'cupy', 'wgpu', or None for auto)
        """
        self.backend_preference = backend_preference
        self.available_devices: List[GPUDevice] = []
        self.active_devices: List[GPUDevice] = []
        self.device_locks: Dict[int, threading.Lock] = {}

        self._discover_devices()
        logger.info(
            f"MultiGPUManager initialized with {len(self.available_devices)} available devices"
        )

    def _discover_devices(self) -> None:
        """Discover available GPU devices across different backends."""
        self.available_devices = []

        # Try PyTorch CUDA devices
        try:
            import torch

            if torch.cuda.is_available():
                for i in range(torch.cuda.device_count()):
                    props = torch.cuda.get_device_properties(i)
                    device = GPUDevice(
                        device_id=i,
                        name=props.name,
                        memory_total=props.total_memory,
                        memory_available=props.total_memory
                        - torch.cuda.memory_allocated(i),
                        compute_capability=(props.major, props.minor),
                        backend_type="pytorch",
                    )
                    self.available_devices.append(device)
                    self.device_locks[i] = threading.Lock()
                    logger.debug(f"Found PyTorch CUDA device {i}: {props.name}")
        except ImportError:
            logger.debug("PyTorch not available for GPU discovery")
        except Exception as e:
            logger.warning(f"Error discovering PyTorch CUDA devices: {e}")

        # Try CuPy devices
        try:
            import cupy as cp

            device_count = cp.cuda.runtime.getDeviceCount()
            for i in range(device_count):
                # Avoid duplicate devices if PyTorch already found them
                if not any(
                    d.device_id == i and d.backend_type == "pytorch"
                    for d in self.available_devices
                ):
                    props = cp.cuda.runtime.getDeviceProperties(i)
                    mem_info = cp.cuda.Device(i).mem_info
                    device = GPUDevice(
                        device_id=i,
                        name=props["name"].decode(),
                        memory_total=mem_info[0],  # Total memory
                        memory_available=mem_info[0] - mem_info[1],  # Total - Used
                        compute_capability=(props["major"], props["minor"]),
                        backend_type="cupy",
                    )
                    self.available_devices.append(device)
                    if i not in self.device_locks:
                        self.device_locks[i] = threading.Lock()
                    logger.debug(
                        f"Found CuPy CUDA device {i}: {props['name'].decode()}"
                    )
        except ImportError:
            logger.debug("CuPy not available for GPU discovery")
        except Exception as e:
            logger.warning(f"Error discovering CuPy CUDA devices: {e}")

        # Try WebGPU devices (placeholder - actual implementation would depend on wgpu-py)
        try:
            # import wgpu  # Unused import removed

            # WebGPU device discovery would go here
            # This is a placeholder since WebGPU device enumeration is more complex
            logger.debug(
                "WebGPU backend available but device discovery not implemented"
            )
        except ImportError:
            logger.debug("WebGPU not available for GPU discovery")

    def get_available_devices(self) -> List[GPUDevice]:
        """Get list of available GPU devices.

        Returns:
            List of available GPU devices
        """
        return self.available_devices.copy()

    def select_devices(self, device_count: Optional[int] = None) -> List[GPUDevice]:
        """Select GPU devices for processing.

        Args:
            device_count: Number of devices to select (None for all available)

        Returns:
            List of selected GPU devices
        """
        if device_count is None:
            device_count = len(self.available_devices)

        # Sort devices by available memory (descending)
        sorted_devices = sorted(
            self.available_devices, key=lambda d: d.memory_available, reverse=True
        )

        selected = sorted_devices[:device_count]
        self.active_devices = selected

        logger.info(f"Selected {len(selected)} GPU devices for processing")
        return selected

    def distribute_workload(
        self, total_neurons: int, total_synapses: int
    ) -> Dict[int, Dict[str, int]]:
        """Distribute neural processing workload across active GPUs.

        Args:
            total_neurons: Total number of neurons to process
            total_synapses: Total number of synapses to process

        Returns:
            Dictionary mapping device_id to workload allocation
        """
        if not self.active_devices:
            raise RuntimeError("No active GPU devices selected")

        # Simple memory-based distribution
        total_memory = sum(d.memory_available for d in self.active_devices)
        distribution = {}

        for device in self.active_devices:
            memory_ratio = device.memory_available / total_memory
            allocated_neurons = int(total_neurons * memory_ratio)
            allocated_synapses = int(total_synapses * memory_ratio)

            distribution[device.device_id] = {
                "neurons": allocated_neurons,
                "synapses": allocated_synapses,
                "memory_ratio": memory_ratio,
            }

        logger.info(f"Distributed workload across {len(self.active_devices)} devices")
        return distribution

    def synchronize_devices(self) -> None:
        """Synchronize all active GPU devices."""
        for device in self.active_devices:
            with self.device_locks[device.device_id]:
                try:
                    if device.backend_type == "pytorch":
                        import torch

                        torch.cuda.synchronize(device.device_id)
                    elif device.backend_type == "cupy":
                        import cupy as cp

                        with cp.cuda.Device(device.device_id):
                            cp.cuda.Stream.null.synchronize()
                    # WebGPU synchronization would go here
                except Exception as e:
                    logger.warning(
                        f"Failed to synchronize device {device.device_id}: {e}"
                    )

    def get_memory_stats(self) -> Dict[int, Dict[str, int]]:
        """Get memory statistics for all active devices.

        Returns:
            Dictionary mapping device_id to memory statistics
        """
        stats = {}

        for device in self.active_devices:
            try:
                if device.backend_type == "pytorch":
                    import torch

                    allocated = torch.cuda.memory_allocated(device.device_id)
                    reserved = torch.cuda.memory_reserved(device.device_id)
                    stats[device.device_id] = {
                        "allocated": allocated,
                        "reserved": reserved,
                        "available": device.memory_total - allocated,
                    }
                elif device.backend_type == "cupy":
                    import cupy as cp

                    with cp.cuda.Device(device.device_id):
                        mem_info = cp.cuda.Device().mem_info
                        stats[device.device_id] = {
                            "allocated": mem_info[1],  # Used memory
                            "available": mem_info[0],  # Free memory
                            "total": device.memory_total,
                        }
            except Exception as e:
                logger.warning(
                    f"Failed to get memory stats for device {device.device_id}: {e}"
                )
                stats[device.device_id] = {"error": str(e)}

        return stats

    def cleanup(self) -> None:
        """Clean up multi-GPU resources."""
        self.active_devices = []
        self.device_locks.clear()
        logger.info("MultiGPUManager cleaned up")


# Global instance for singleton-like access
_multi_gpu_manager: Optional[MultiGPUManager] = None


def get_multi_gpu_manager(backend_preference: Optional[str] = None) -> MultiGPUManager:
    """Get the global MultiGPUManager instance.

    Args:
        backend_preference: Preferred backend for GPU operations

    Returns:
        MultiGPUManager instance
    """
    global _multi_gpu_manager

    if _multi_gpu_manager is None:
        _multi_gpu_manager = MultiGPUManager(backend_preference)

    return _multi_gpu_manager


def reset_multi_gpu_manager() -> None:
    """Reset the global MultiGPUManager instance (for testing)."""
    global _multi_gpu_manager

    if _multi_gpu_manager:
        _multi_gpu_manager.cleanup()

    _multi_gpu_manager = None
    logger.debug("Reset MultiGPUManager for testing")
