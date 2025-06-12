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

"""
Tests for the WebGPU backend.
"""

import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from feagi.core.backend import BackendType
from feagi.core.backend.interface import get_backend

# Skip tests if wgpu is not available
try:
    import wgpu

    WEBGPU_AVAILABLE = True
except ImportError:
    WEBGPU_AVAILABLE = False


@pytest.mark.skipif(not WEBGPU_AVAILABLE, reason="WebGPU not available")
class TestWebGPUBackend(unittest.TestCase):
    def setUp(self):
        self.backend = get_backend(BackendType.WEBGPU)
        if self.backend is None:
            self.skipTest("WebGPU backend not available")

    def tearDown(self):
        if self.backend is not None:
            self.backend.shutdown()

    def test_initialization(self):
        self.assertTrue(self.backend.initialize())

    def test_tensor_creation(self):
        self.backend.initialize()
        shape = (10, 10)
        tensor = self.backend.create_tensor(shape)
        self.assertEqual(tensor.shape, shape)

    def test_to_numpy(self):
        self.backend.initialize()
        shape = (5, 5)
        tensor = self.backend.create_tensor(shape, dtype=np.float32)
        # Skip if device or buffer is a MagicMock (mocked wgpu)
        import unittest.mock

        if isinstance(self.backend.device, unittest.mock.MagicMock) or isinstance(
            tensor.buffer, unittest.mock.MagicMock
        ):
            self.skipTest(
                "WebGPU device or buffer is a MagicMock (mocked, not real hardware)"
            )
        np_array = self.backend.to_numpy(tensor)
        self.assertIsInstance(np_array, np.ndarray)
        self.assertEqual(np_array.shape, shape)

    def test_from_numpy(self):
        self.backend.initialize()
        np_array = np.random.random((3, 3)).astype(np.float32)
        tensor = self.backend.from_numpy(np_array)
        import unittest.mock

        if isinstance(self.backend.device, unittest.mock.MagicMock) or isinstance(
            tensor.buffer, unittest.mock.MagicMock
        ):
            self.skipTest(
                "WebGPU device or buffer is a MagicMock (mocked, not real hardware)"
            )
        np_result = self.backend.to_numpy(tensor)
        np.testing.assert_allclose(np_array, np_result)

    def test_capability_support(self):
        # If we're here, we have a backend, but it might be a CPUBackend due to fallback
        from feagi.core.backend.webgpu import WebGPUBackend

        if not isinstance(self.backend, WebGPUBackend):
            self.skipTest("WebGPU backend not available, using fallback")

        if not self.backend.initialized:
            self.backend.initialize()

        if not self.backend.initialized:
            self.skipTest("WebGPU backend could not be initialized")

        for capability in [
            "matrix_multiplication",
            "element_wise_operations",
            "random_generation",
            "bitmap_operations",
        ]:
            self.assertTrue(self.backend.supports_capability(capability))


@pytest.mark.skipif(WEBGPU_AVAILABLE, reason="Testing when WebGPU is not available")
class TestWebGPUBackendUnavailable(unittest.TestCase):
    @patch("feagi.core.backend.webgpu.wgpu", None)
    def test_backend_unavailable(self):
        backend = get_backend(BackendType.WEBGPU)
        self.assertIsNone(backend)


if __name__ == "__main__":
    unittest.main()
