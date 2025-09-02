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

"""
CPU Backend Implementation for FEAGI.

This module provides a CPU-based implementation of the backend interface
using NumPy for tensor operations.
"""

from feagi.utils.logger import setup_logger

logger = setup_logger("feagi.core.backend.cpu")
import platform
from typing import Any, Optional, Tuple, Union

import numpy as np

from feagi.core.backend.interface import (
    BackendCapability,
    BackendInterface,
    BackendType,
    register_backend,
)


class CPUBackend(BackendInterface):
    """CPU Backend implementation using NumPy.

    This backend provides a reference implementation of the backend interface
    using NumPy for tensor operations. It serves as a fallback when GPU
    acceleration is not available.
    """

    def __init__(self):
        """Initialize the CPU backend."""
        super().__init__(name="numpy", device="cpu")

        # Set capabilities
        self._capabilities = {
            BackendCapability.MATRIX_MULTIPLICATION,
            BackendCapability.CONVOLUTION,
            BackendCapability.ELEMENT_WISE_OPERATIONS,
            BackendCapability.BITMAP_OPERATIONS,
            BackendCapability.RANDOM_NUMBER_GENERATION,
        }

        # Optional: If NumPy has sparse matrix support
        if hasattr(np, "sparse"):
            self._capabilities.add(BackendCapability.SPARSE_OPERATIONS)

    def initialize(self) -> bool:
        """Initialize the CPU backend.

        Returns:
            True if initialization succeeded, False otherwise.
        """
        try:
            # Log CPU information
            cpu_info = self._get_cpu_info()
            logger.info(f"Initializing CPU backend with: {cpu_info}")

            # Check NumPy version
            logger.info(f"NumPy version: {np.__version__}")

            # Check if NumPy has BLAS/LAPACK support
            has_blas = self._check_blas_support()
            if has_blas:
                logger.info("NumPy is using BLAS/LAPACK for linear algebra")
            else:
                logger.warning(
                    "NumPy may not be using optimized BLAS/LAPACK libraries"
                )

            self._initialized = True
            return True
        except Exception as e:
            logger.error(f"Failed to initialize CPU backend: {e}")
            return False

    def shutdown(self) -> None:
        """Shutdown the CPU backend (no-op for CPU)."""
        self._initialized = False
        logger.info("CPU backend shut down")

    def supports_capability(self, capability: BackendCapability) -> bool:
        """Check if the CPU backend supports a specific capability.

        Args:
            capability: The capability to check.

        Returns:
            True if the capability is supported, False otherwise.
        """
        return capability in self._capabilities

    def create_tensor(
        self,
        shape: Tuple[int, ...],
        dtype: Any = np.float32,
        data: Optional[Union[np.ndarray, list]] = None,
    ) -> np.ndarray:
        """Create a NumPy array with the given shape and type.

        Args:
            shape: Shape of the tensor.
            dtype: Data type of the tensor.
            data: Optional data to initialize the tensor with.

        Returns:
            A NumPy array.
        """
        if data is not None:
            # Convert to NumPy array if it's not already
            if not isinstance(data, np.ndarray):
                data = np.array(data, dtype=dtype)

            # Ensure the shape is correct
            if data.shape != shape:
                data = data.reshape(shape)

            return data
        else:
            return np.zeros(shape, dtype=dtype)

    def to_numpy(self, tensor: np.ndarray) -> np.ndarray:
        """Convert a NumPy array to a NumPy array (no-op).

        Args:
            tensor: NumPy array.

        Returns:
            The same NumPy array.
        """
        return tensor

    def from_numpy(self, array: np.ndarray) -> np.ndarray:
        """Convert a NumPy array to a NumPy array (no-op).

        Args:
            array: NumPy array.

        Returns:
            The same NumPy array.
        """
        return array

    def synchronize(self) -> None:
        """Ensure all pending operations are complete (no-op for CPU)."""
        pass  # CPU operations are synchronous, so no need to synchronize

    def _get_cpu_info(self) -> str:
        """Get information about the CPU.

        Returns:
            String describing the CPU.
        """
        if platform.system() == "Darwin":
            # On macOS
            try:
                import subprocess

                result = subprocess.run(
                    ["sysctl", "-n", "machdep.cpu.brand_string"],
                    capture_output=True,
                    text=True,
                    check=True,
                )
                return result.stdout.strip()
            except Exception:
                return f"Unknown CPU on macOS {platform.machine()}"
        elif platform.system() == "Linux":
            # On Linux
            try:
                with open("/proc/cpuinfo", "r") as f:
                    for line in f:
                        if line.startswith("model name"):
                            return line.split(":", 1)[1].strip()
                return f"Unknown CPU on Linux {platform.machine()}"
            except Exception:
                return f"Unknown CPU on Linux {platform.machine()}"
        else:
            # Other platforms
            return f"Unknown CPU on {platform.system()} {platform.machine()}"

    def _check_blas_support(self) -> bool:
        """Check if NumPy is using an optimized BLAS/LAPACK implementation.

        Returns:
            True if BLAS/LAPACK support is available, False otherwise.
        """
        try:
            # Create two matrices
            a = np.random.rand(100, 100)
            b = np.random.rand(100, 100)

            # Time matrix multiplication
            import time

            start = time.time()
            np.dot(a, b)
            end = time.time()

            # Very rough heuristic: if it's too fast, it's probably using BLAS
            # (this is not reliable, but it's a reasonable heuristic)
            duration = end - start
            return duration < 0.01
        except Exception:
            return False


# Register the CPU backend
register_backend(BackendType.CPU, CPUBackend)
