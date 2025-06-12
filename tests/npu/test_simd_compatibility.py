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
SIMD Compatibility Tests for FEAGI NPU Module.

This module tests SIMD optimization availability and compatibility across
different hardware platforms and implementations, ensuring optimal performance
for neural processing operations.

Similar to test_wgpu_compatibility.py and test_rust_rtos_compatibility.py,
this provides comprehensive validation of SIMD features.
"""

import platform
import subprocess
import time
from typing import Dict, List, Optional, Tuple
from unittest.mock import Mock, patch

import numpy as np
import psutil
import pytest

# Try to import SIMD-related modules
try:
    import cpufeature

    CPU_FEATURE_AVAILABLE = True
except ImportError:
    CPU_FEATURE_AVAILABLE = False

try:
    from feagi.npu.optimized_structures import GlobalNeuronArray

    OPTIMIZED_STRUCTURES_AVAILABLE = True
except ImportError:
    OPTIMIZED_STRUCTURES_AVAILABLE = False

try:
    from feagi.rust.data_structures.gna import GlobalNeuronArray as RustGNA

    RUST_SIMD_AVAILABLE = True
except ImportError:
    RUST_SIMD_AVAILABLE = False


class SIMDCompatibilityTester:
    """Test SIMD compatibility and performance across different implementations."""

    def __init__(self):
        self.results = {}
        self.cpu_info = self._detect_cpu_features()
        self.numpy_info = self._detect_numpy_simd()

    def _detect_cpu_features(self) -> Dict[str, bool]:
        """Detect available CPU SIMD features."""
        features = {}

        # Platform-specific detection
        system = platform.system()
        machine = platform.machine()

        if machine.lower() in ["x86_64", "amd64"]:
            # x86_64 features
            features.update(
                {
                    "sse": self._check_x86_feature("sse"),
                    "sse2": self._check_x86_feature("sse2"),
                    "sse3": self._check_x86_feature("sse3"),
                    "ssse3": self._check_x86_feature("ssse3"),
                    "sse4_1": self._check_x86_feature("sse4_1"),
                    "sse4_2": self._check_x86_feature("sse4_2"),
                    "avx": self._check_x86_feature("avx"),
                    "avx2": self._check_x86_feature("avx2"),
                    "avx512f": self._check_x86_feature("avx512f"),
                    "fma": self._check_x86_feature("fma"),
                }
            )
        elif machine.lower() in ["arm64", "aarch64"]:
            # ARM features
            features.update(
                {
                    "neon": True,  # ARM64 always has NEON
                    "asimd": self._check_arm_feature("asimd"),
                    "sve": self._check_arm_feature("sve"),
                    "sve2": self._check_arm_feature("sve2"),
                }
            )

        return features

    def _check_x86_feature(self, feature: str) -> bool:
        """Check for specific x86 SIMD feature."""
        try:
            if CPU_FEATURE_AVAILABLE:
                return getattr(cpufeature, feature, False)

            # Fallback: check /proc/cpuinfo on Linux
            if platform.system() == "Linux":
                with open("/proc/cpuinfo", "r") as f:
                    return feature in f.read().lower()

            # Fallback: check via sysctl on macOS
            elif platform.system() == "Darwin":
                result = subprocess.run(
                    ["sysctl", "-a"], capture_output=True, text=True, timeout=5
                )
                return feature in result.stdout.lower()

        except Exception:
            pass

        return False

    def _check_arm_feature(self, feature: str) -> bool:
        """Check for specific ARM SIMD feature."""
        try:
            # Check /proc/cpuinfo on Linux
            if platform.system() == "Linux":
                with open("/proc/cpuinfo", "r") as f:
                    content = f.read().lower()
                    return feature in content or "asimd" in content

            # macOS ARM always has NEON/ASIMD
            elif platform.system() == "Darwin":
                return feature in ["neon", "asimd"]

        except Exception:
            pass

        return False

    def _detect_numpy_simd(self) -> Dict[str, any]:
        """Detect NumPy SIMD configuration."""
        info = {}

        try:
            # Get NumPy configuration
            config = np.show_config()
            info["config_available"] = True

            # Try to get SIMD information
            try:
                from numpy._core._simd import targets

                info["simd_targets"] = list(targets.keys()) if targets else []
                info["simd_extensions"] = {}

                for name, module in targets.items():
                    if module:
                        info["simd_extensions"][name] = {
                            "available": hasattr(module, "simd"),
                            "simd_f32": getattr(module, "simd_f32", False),
                            "simd_f64": getattr(module, "simd_f64", False),
                        }
            except ImportError:
                info["simd_targets"] = []
                info["simd_extensions"] = {}

            # Check BLAS backend
            try:
                info["blas_info"] = np.__config__.get_info("blas_opt")
            except Exception:
                info["blas_info"] = {}

        except Exception as e:
            info["config_available"] = False
            info["error"] = str(e)

        return info

    def test_numpy_vectorization_performance(
        self, size: int = 100000
    ) -> Dict[str, float]:
        """Test NumPy vectorization performance vs scalar operations."""

        # Generate test data
        a = np.random.random(size).astype(np.float32)
        b = np.random.random(size).astype(np.float32)
        c = np.random.random(size).astype(np.float32)

        # Test vectorized operations
        start_time = time.perf_counter()
        for _ in range(10):
            result_vec = a * b + c  # Should use SIMD if available
        vectorized_time = time.perf_counter() - start_time

        # Test scalar operations
        start_time = time.perf_counter()
        for _ in range(10):
            result_scalar = np.array([a[i] * b[i] + c[i] for i in range(len(a))])
        scalar_time = time.perf_counter() - start_time

        # Verify results are equivalent
        np.testing.assert_allclose(result_vec, result_scalar, rtol=1e-6)

        speedup = scalar_time / vectorized_time if vectorized_time > 0 else 0

        return {
            "vectorized_time": vectorized_time,
            "scalar_time": scalar_time,
            "speedup": speedup,
            "size": size,
        }

    def test_membrane_potential_simd(self, neuron_count: int = 10000) -> Dict[str, any]:
        """Test SIMD optimization in membrane potential updates."""

        # Setup test data
        membrane_potentials = np.random.random(neuron_count).astype(np.float32)
        decay_rates = np.random.uniform(0.8, 0.99, neuron_count).astype(np.float32)
        thresholds = np.ones(neuron_count, dtype=np.float32)

        # Test vectorized membrane potential decay (SIMD-optimized)
        start_time = time.perf_counter()
        for _ in range(100):
            # Vectorized decay operation
            membrane_potentials *= decay_rates

            # Vectorized threshold check
            fired_mask = membrane_potentials >= thresholds

            # Vectorized reset
            membrane_potentials[fired_mask] = 0.0

        simd_time = time.perf_counter() - start_time

        # Reset for scalar test
        membrane_potentials = np.random.random(neuron_count).astype(np.float32)

        # Test scalar version
        start_time = time.perf_counter()
        for _ in range(100):
            # Scalar operations
            for i in range(neuron_count):
                membrane_potentials[i] *= decay_rates[i]
                if membrane_potentials[i] >= thresholds[i]:
                    membrane_potentials[i] = 0.0

        scalar_time = time.perf_counter() - start_time

        speedup = scalar_time / simd_time if simd_time > 0 else 0

        return {
            "simd_time": simd_time,
            "scalar_time": scalar_time,
            "speedup": speedup,
            "neurons": neuron_count,
            "effective_simd": speedup > 2.0,  # Expect at least 2x speedup with SIMD
        }

    def test_rust_simd_availability(self) -> Dict[str, any]:
        """Test Rust SIMD implementation availability and performance."""

        if not RUST_SIMD_AVAILABLE:
            return {"available": False, "reason": "Rust SIMD module not available"}

        try:
            # Test Rust GNA creation
            gna = RustGNA(10000)

            # Test SIMD membrane potential update
            start_time = time.perf_counter()
            for _ in range(100):
                gna.update_membrane_potentials_simd(0.99)
            rust_simd_time = time.perf_counter() - start_time

            return {"available": True, "simd_time": rust_simd_time, "gna_created": True}

        except Exception as e:
            return {"available": False, "error": str(e)}

    def test_roaring_bitmap_simd(self) -> Dict[str, any]:
        """Test Roaring Bitmap SIMD optimizations."""

        try:
            import pyroaring

            # Create large bitmaps for testing
            bitmap1 = pyroaring.BitMap(range(0, 100000, 3))
            bitmap2 = pyroaring.BitMap(range(0, 100000, 5))

            # Test SIMD-optimized operations
            start_time = time.perf_counter()
            for _ in range(100):
                union_result = bitmap1 | bitmap2
                intersection_result = bitmap1 & bitmap2
                difference_result = bitmap1 - bitmap2
            simd_time = time.perf_counter() - start_time

            return {
                "available": True,
                "simd_time": simd_time,
                "operations": ["union", "intersection", "difference"],
                "bitmap_sizes": [len(bitmap1), len(bitmap2)],
            }

        except ImportError:
            return {"available": False, "reason": "PyRoaring not available"}
        except Exception as e:
            return {"available": False, "error": str(e)}

    def run_comprehensive_test(self) -> Dict[str, any]:
        """Run comprehensive SIMD compatibility test."""

        results = {
            "system_info": {
                "platform": platform.platform(),
                "processor": platform.processor(),
                "machine": platform.machine(),
                "cpu_count": psutil.cpu_count(logical=False),
                "cpu_count_logical": psutil.cpu_count(logical=True),
            },
            "cpu_features": self.cpu_info,
            "numpy_simd": self.numpy_info,
            "performance_tests": {},
            "compatibility": {},
            "recommendations": [],
        }

        # Run performance tests
        print("🧪 Running NumPy vectorization test...")
        results["performance_tests"]["numpy_vectorization"] = (
            self.test_numpy_vectorization_performance()
        )

        print("🧪 Running membrane potential SIMD test...")
        results["performance_tests"]["membrane_potential"] = (
            self.test_membrane_potential_simd()
        )

        print("🧪 Testing Rust SIMD availability...")
        results["compatibility"]["rust_simd"] = self.test_rust_simd_availability()

        print("🧪 Testing Roaring Bitmap SIMD...")
        results["compatibility"]["roaring_bitmap"] = self.test_roaring_bitmap_simd()

        # Generate recommendations
        results["recommendations"] = self._generate_recommendations(results)

        return results

    def _generate_recommendations(self, results: Dict[str, any]) -> List[str]:
        """Generate SIMD optimization recommendations."""
        recommendations = []

        # Check CPU features
        cpu_features = results["cpu_features"]
        if not cpu_features.get("avx2", False) and not cpu_features.get("neon", False):
            recommendations.append(
                "⚠️  No advanced SIMD features detected. Performance may be limited."
            )

        # Check NumPy SIMD
        numpy_perf = results["performance_tests"]["numpy_vectorization"]
        if numpy_perf["speedup"] < 2.0:
            recommendations.append(
                "⚠️  NumPy vectorization speedup is low. Check BLAS configuration."
            )

        # Check membrane potential performance
        membrane_perf = results["performance_tests"]["membrane_potential"]
        if not membrane_perf["effective_simd"]:
            recommendations.append(
                "⚠️  Membrane potential updates not effectively using SIMD."
            )

        # Check Rust SIMD
        rust_simd = results["compatibility"]["rust_simd"]
        if not rust_simd["available"]:
            recommendations.append(
                "💡 Consider building Rust extensions with SIMD support for optimal performance."
            )

        # Positive recommendations
        if numpy_perf["speedup"] > 5.0:
            recommendations.append("✅ Excellent NumPy SIMD performance detected.")

        if membrane_perf["effective_simd"]:
            recommendations.append("✅ Membrane potential updates are SIMD-optimized.")

        if rust_simd["available"]:
            recommendations.append("✅ Rust SIMD extensions are available.")

        return recommendations


# Test cases using pytest
class TestSIMDCompatibility:
    """Pytest test cases for SIMD compatibility."""

    @pytest.fixture
    def simd_tester(self):
        """Create SIMD compatibility tester."""
        return SIMDCompatibilityTester()

    def test_cpu_feature_detection(self, simd_tester):
        """Test CPU feature detection."""
        cpu_features = simd_tester.cpu_info

        # Should detect at least some features
        assert isinstance(cpu_features, dict)

        # Platform-specific checks
        machine = platform.machine().lower()
        if machine in ["x86_64", "amd64"]:
            # x86_64 should have at least SSE2
            assert cpu_features.get("sse2", False), "x86_64 should support SSE2"
        elif machine in ["arm64", "aarch64"]:
            # ARM64 should have NEON
            assert cpu_features.get("neon", False), "ARM64 should support NEON"

    def test_numpy_simd_detection(self, simd_tester):
        """Test NumPy SIMD detection."""
        numpy_info = simd_tester.numpy_info

        assert isinstance(numpy_info, dict)
        assert numpy_info.get("config_available", False), (
            "NumPy config should be available"
        )

    def test_vectorization_performance(self, simd_tester):
        """Test that vectorized operations are faster than scalar."""
        perf_results = simd_tester.test_numpy_vectorization_performance(size=10000)

        assert perf_results["speedup"] > 1.0, (
            "Vectorized operations should be faster than scalar"
        )
        assert perf_results["vectorized_time"] > 0
        assert perf_results["scalar_time"] > 0

    def test_membrane_potential_simd_performance(self, simd_tester):
        """Test membrane potential SIMD performance."""
        membrane_results = simd_tester.test_membrane_potential_simd(neuron_count=1000)

        assert membrane_results["speedup"] > 1.0, (
            "SIMD membrane updates should be faster"
        )
        assert membrane_results["simd_time"] > 0
        assert membrane_results["scalar_time"] > 0

    @pytest.mark.skipif(not RUST_SIMD_AVAILABLE, reason="Rust SIMD not available")
    def test_rust_simd_availability(self, simd_tester):
        """Test Rust SIMD availability."""
        rust_results = simd_tester.test_rust_simd_availability()

        assert rust_results["available"], (
            "Rust SIMD should be available when module is imported"
        )
        assert rust_results.get("gna_created", False), (
            "Should be able to create Rust GNA"
        )

    def test_roaring_bitmap_operations(self, simd_tester):
        """Test Roaring Bitmap SIMD operations."""
        bitmap_results = simd_tester.test_roaring_bitmap_simd()

        # PyRoaring should be available in our environment
        if bitmap_results["available"]:
            assert bitmap_results["simd_time"] > 0
            assert len(bitmap_results["operations"]) > 0

    def test_comprehensive_simd_assessment(self, simd_tester):
        """Run comprehensive SIMD assessment."""
        results = simd_tester.run_comprehensive_test()

        # Verify structure
        assert "system_info" in results
        assert "cpu_features" in results
        assert "numpy_simd" in results
        assert "performance_tests" in results
        assert "compatibility" in results
        assert "recommendations" in results

        # Print results for manual inspection
        print("\n" + "=" * 60)
        print("🧪 SIMD COMPATIBILITY TEST RESULTS")
        print("=" * 60)

        print(f"\n🖥️  System: {results['system_info']['platform']}")
        print(f"🧮 CPU: {results['system_info']['processor']}")
        print(f"🔧 Architecture: {results['system_info']['machine']}")
        print(
            f"⚙️  Cores: {results['system_info']['cpu_count']} physical, {results['system_info']['cpu_count_logical']} logical"
        )

        print("\n🎯 CPU SIMD Features:")
        for feature, available in results["cpu_features"].items():
            status = "✅" if available else "❌"
            print(f"   {status} {feature.upper()}")

        print("\n⚡ Performance Results:")
        numpy_perf = results["performance_tests"]["numpy_vectorization"]
        print(f"   📊 NumPy Vectorization: {numpy_perf['speedup']:.1f}x speedup")

        membrane_perf = results["performance_tests"]["membrane_potential"]
        print(f"   🧠 Membrane Potential: {membrane_perf['speedup']:.1f}x speedup")

        print("\n🔧 Compatibility:")
        rust_compat = results["compatibility"]["rust_simd"]
        rust_status = "✅" if rust_compat["available"] else "❌"
        print(f"   {rust_status} Rust SIMD")

        bitmap_compat = results["compatibility"]["roaring_bitmap"]
        bitmap_status = "✅" if bitmap_compat["available"] else "❌"
        print(f"   {bitmap_status} Roaring Bitmap SIMD")

        print("\n💡 Recommendations:")
        for rec in results["recommendations"]:
            print(f"   {rec}")

        print("\n" + "=" * 60)


if __name__ == "__main__":
    """Run SIMD compatibility test standalone."""
    tester = SIMDCompatibilityTester()
    results = tester.run_comprehensive_test()

    # Return appropriate exit code
    critical_issues = [r for r in results["recommendations"] if r.startswith("⚠️")]
    exit_code = 1 if critical_issues else 0

    print(
        f"\n🏁 SIMD Compatibility Test {'⚠️  ISSUES FOUND' if critical_issues else '✅ PASSED'}"
    )
    exit(exit_code)
