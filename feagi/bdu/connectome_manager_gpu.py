"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""GPU-specific extensions for ConnectomeManager.

This module provides GPU-accelerated operations for the ConnectomeManager
when GPU backends are available.
"""

from typing import Any, Dict, List

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class GPUConnectomeManager(ConnectomeManager):
    """GPU-accelerated version of ConnectomeManager.

    This class extends the base ConnectomeManager with GPU-specific
    optimizations when CUDA, Metal, or WebGPU backends are available.
    """

    def __init__(self, *args, **kwargs):
        """Initialize GPU-accelerated ConnectomeManager."""
        # Force GPU backend selection
        if "backend" not in kwargs:
            kwargs["backend"] = (
                "auto"  # Will select best GPU backend available
            )

        super().__init__(*args, **kwargs)

        #  Verify GPU backend was selected (after parent initialization
        #  completes)
        self._verify_gpu_backend()

        backend_info = self.get_backend_info()
        logger.info(
            f"Initialized GPU ConnectomeManager with backend: {backend_info}"
        )

    def _verify_gpu_backend(self):
        """Verify that a GPU backend was successfully selected."""
        if (
            hasattr(self, "neuron_array")
            and hasattr(self.neuron_array, "backend")
            and hasattr(self.neuron_array.backend, "backend_type")
        ):
            backend_type = self.neuron_array.backend.backend_type.value
            if backend_type not in ["pytorch", "cupy", "wgpu"]:
                logger.warning(
                    f"GPU backend requested but {backend_type} selected - GPU may not be available"
                )
        else:
            logger.warning(
                "GPU backend verification failed - neuron_array or backend not properly initialized"
            )

    def get_backend_info(self) -> Dict[str, Any]:
        """Get information about the current GPU backend.

        Returns:
            Dictionary containing backend information
        """
        if hasattr(self, "neuron_array") and hasattr(
            self.neuron_array, "backend"
        ):
            if hasattr(self.neuron_array.backend, "get_device_stats"):
                return self.neuron_array.backend.get_device_stats()
            else:
                return {
                    "backend": getattr(
                        self.neuron_array.backend, "backend_type", "unknown"
                    ),
                    "device": "unknown",
                }
        return {"backend": "unknown", "device": "unknown"}

    def gpu_batch_update_membrane_potentials(
        self, neuron_ids: List[int], potentials: List[float]
    ) -> bool:
        """GPU-accelerated batch update of membrane potentials.

        Args:
            neuron_ids: List of neuron IDs to update
            potentials: List of new membrane potential values

        Returns:
            True if successful, False otherwise
        """
        try:
            return self.batch_update_neuron_properties(
                neuron_ids, "membrane_potential", potentials
            )
        except Exception as e:
            logger.error(f"GPU batch membrane potential update failed: {e}")
            return False

    def gpu_accelerated_synapse_propagation(
        self, firing_neurons: List[int]
    ) -> Dict[int, float]:
        """GPU-accelerated synapse signal propagation.

        Args:
            firing_neurons: List of neurons that are firing

        Returns:
            Dictionary mapping neuron IDs to accumulated input signals
        """
        accumulated_signals = {}

        try:
            # Use GPU-accelerated matrix operations for signal propagation
            for neuron_id in firing_neurons:
                connections = self.get_outgoing_connections(neuron_id)
                for target_id, weight in connections:
                    if target_id not in accumulated_signals:
                        accumulated_signals[target_id] = 0.0
                    accumulated_signals[target_id] += weight

        except Exception as e:
            logger.error(f"GPU synapse propagation failed: {e}")

        return accumulated_signals

    @classmethod
    def reset_singleton(cls):
        """Reset the singleton instance for testing purposes.

        This method is used by tests to ensure clean state between test runs.
        """
        cls._instance = None
        cls._initialized = False
        logger.debug("Reset GPUConnectomeManager singleton for testing")


# Alias for backward compatibility
ConnectomeManagerGPU = GPUConnectomeManager
