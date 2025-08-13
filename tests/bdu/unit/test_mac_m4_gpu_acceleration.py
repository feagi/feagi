#!/usr/bin/env python3
"""
Test Mac M4 GPU acceleration for FEAGI array backend.

This test verifies that FEAGI can leverage Apple's Metal Performance Shaders (MPS)
for GPU acceleration on Mac M4 systems.
"""

import os
import sys
import time

import pytest

# Add the project root to the path for direct imports
project_root = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)
sys.path.insert(0, project_root)


class TestMacM4GPUAcceleration:
    """Test GPU acceleration on Mac M4 systems."""

    def test_pytorch_mps_availability(self):
        """Test that PyTorch MPS is available and working on Mac M4."""
        torch = pytest.importorskip("torch")

        # Check MPS availability
        assert hasattr(torch.backends, "mps"), "MPS backends not available"
        assert torch.backends.mps.is_available(), "MPS not available on this system"
        assert torch.backends.mps.is_built(), "MPS not built into PyTorch"

        print("✅ PyTorch MPS is available and built")

    def test_pytorch_mps_operations(self):
        """Test that MPS operations work correctly."""
        torch = pytest.importorskip("torch")

        if not (hasattr(torch.backends, "mps") and torch.backends.mps.is_available()):
            pytest.skip("MPS not available")

        device = torch.device("mps")

        # Test tensor creation
        a = torch.zeros((100, 100), device=device, dtype=torch.float32)
        b = torch.ones((100, 100), device=device, dtype=torch.float32)

        # Test operations
        result = torch.matmul(a, b)
        torch.mps.synchronize()

        # Verify results
        assert result.device.type == "mps"
        assert result.shape == (100, 100)
        assert torch.allclose(result, torch.zeros_like(result))

        print("✅ PyTorch MPS operations working correctly")

    def test_feagi_array_backend_uses_mps(self):
        """Test that FEAGI's array backend correctly uses MPS on Mac M4."""
        torch = pytest.importorskip("torch")

        if not (hasattr(torch.backends, "mps") and torch.backends.mps.is_available()):
            pytest.skip("MPS not available")

        # Import array backend directly to avoid ConnectomeManager import issues
        sys.path.insert(0, os.path.join(project_root, "feagi", "bdu", "models"))

        try:
            from array_backend import ArrayBackend, BackendType

            # Test AUTO backend selection
            backend = ArrayBackend("auto")

            # Should select PyTorch if available
            assert backend.backend_type == BackendType.PYTORCH, (
                f"Expected PyTorch backend, got {backend.backend_type.value}"
            )

            # Should use MPS device
            assert hasattr(backend, "device"), "Backend should have device attribute"
            assert backend.device == "mps", f"Expected MPS device, got {backend.device}"

            # Test array creation
            test_array = backend.zeros((10, 10))
            assert isinstance(test_array, torch.Tensor), (
                f"Expected torch.Tensor, got {type(test_array)}"
            )
            assert "mps" in str(test_array.device), (
                f"Expected MPS device, tensor is on {test_array.device}"
            )

            print("✅ FEAGI ArrayBackend correctly uses MPS for GPU acceleration")

        except ImportError as e:
            pytest.skip(f"Could not import FEAGI array backend: {e}")

    def test_mps_performance_characteristics(self):
        """Test MPS performance characteristics (not necessarily faster for small operations)."""
        torch = pytest.importorskip("torch")

        if not (hasattr(torch.backends, "mps") and torch.backends.mps.is_available()):
            pytest.skip("MPS not available")

        size = 1000
        iterations = 3

        # CPU timing
        cpu_times = []
        for _ in range(iterations):
            start = time.time()
            a_cpu = torch.zeros((size, size), dtype=torch.float32)
            b_cpu = torch.ones((size, size), dtype=torch.float32)
            result_cpu = torch.matmul(a_cpu, b_cpu)
            cpu_times.append(time.time() - start)

        avg_cpu_time = sum(cpu_times) / len(cpu_times)

        # MPS timing
        device = torch.device("mps")
        mps_times = []
        for _ in range(iterations):
            start = time.time()
            a_mps = torch.zeros((size, size), device=device, dtype=torch.float32)
            b_mps = torch.ones((size, size), device=device, dtype=torch.float32)
            result_mps = torch.matmul(a_mps, b_mps)
            torch.mps.synchronize()
            mps_times.append(time.time() - start)

        avg_mps_time = sum(mps_times) / len(mps_times)

        # Verify operations completed successfully
        assert avg_mps_time > 0, "MPS operations took no time (suspicious)"
        assert avg_cpu_time > 0, "CPU operations took no time (suspicious)"

        speedup = avg_cpu_time / avg_mps_time
        print(f"🖥️  Average CPU time: {avg_cpu_time * 1000:.2f}ms")
        print(f"🚀 Average MPS time: {avg_mps_time * 1000:.2f}ms")
        print(f"🎯 Speedup ratio: {speedup:.2f}x")

        # Note: For small matrices, CPU might be faster due to GPU overhead
        # This test just ensures MPS is working, not necessarily faster
        if speedup > 1.0:
            print("🎉 MPS is faster than CPU for this test!")
        else:
            print("⚠️  CPU is faster for this test size (GPU overhead effects)")

    def test_webgpu_not_required_for_mac_m4(self):
        """Verify that WebGPU is not required for Mac M4 acceleration."""
        torch = pytest.importorskip("torch")

        # WebGPU availability check (should not be required)
        try:
            import wgpu

            print(f"🌐 WebGPU is available: {wgpu.__version__}")
        except ImportError:
            print("❌ WebGPU not installed (and that's fine for Mac M4)")

        # The point is that PyTorch MPS should be sufficient
        if hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
            print("✅ Mac M4 has PyTorch MPS - WebGPU not needed!")
        else:
            pytest.fail("Neither PyTorch MPS nor WebGPU available")

    def test_array_backend_device_selection_priority(self):
        """Test that array backend correctly prioritizes CUDA > MPS > CPU."""
        torch = pytest.importorskip("torch")

        # Import array backend directly
        sys.path.insert(0, os.path.join(project_root, "feagi", "bdu", "models"))

        try:
            from array_backend import ArrayBackend, BackendType

            backend = ArrayBackend("pytorch")

            # On Mac M4, should prioritize MPS over CPU (no CUDA available)
            if torch.cuda.is_available():
                assert backend.device == "cuda", "Should use CUDA if available"
            elif hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
                assert backend.device == "mps", (
                    "Should use MPS if CUDA not available but MPS is"
                )
            else:
                assert backend.device == "cpu", (
                    "Should fall back to CPU if neither CUDA nor MPS available"
                )

            print(f"✅ Device selection working correctly: {backend.device}")

        except ImportError as e:
            pytest.skip(f"Could not import FEAGI array backend: {e}")


if __name__ == "__main__":
    # Run tests manually if executed as script
    print("🍎 MAC M4 GPU ACCELERATION TESTS")
    print("=" * 40)

    test_instance = TestMacM4GPUAcceleration()

    try:
        test_instance.test_pytorch_mps_availability()
        test_instance.test_pytorch_mps_operations()
        test_instance.test_feagi_array_backend_uses_mps()
        test_instance.test_mps_performance_characteristics()
        test_instance.test_webgpu_not_required_for_mac_m4()
        test_instance.test_array_backend_device_selection_priority()

        print("\n🎉 ALL TESTS PASSED!")
        print("✅ Your Mac M4 is properly configured for GPU acceleration!")
        print("🚀 FEAGI will use Apple Metal GPU for neural operations!")

    except Exception as e:
        print(f"\n❌ TEST FAILED: {e}")
        import traceback

        traceback.print_exc()
