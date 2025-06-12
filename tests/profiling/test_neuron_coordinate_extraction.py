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
Neuron Coordinate Extraction Performance Profiling Tests

Tests the high-performance SIMD-optimized neuron coordinate extraction
methods in CoreAPIService, measuring throughput and efficiency across
different batch sizes and backend configurations.

This test suite validates:
- SIMD-optimized coordinate extraction performance
- NumPy vs Dictionary API throughput comparison
- Batch size optimization recommendations
- Backend efficiency (PyTorch, NumPy, SIMD)
- Memory usage patterns
- Scalability characteristics
"""

import json
import os
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import pytest

# Import FEAGI components
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.core.state_manager import FeagiStateManager
from feagi.utils.logger import setup_logger

logger = setup_logger()


class NeuronCoordinateExtractionProfiler:
    """
    Profiler for neuron coordinate extraction performance testing.

    Measures performance across different:
    - Batch sizes (100 to 100,000 neurons)
    - Extraction methods (dictionary vs numpy)
    - Backend configurations (SIMD, PyTorch, NumPy)
    - Memory layouts and access patterns
    """

    def __init__(self, connectome_manager: ConnectomeManager, core_api: CoreAPIService):
        """Initialize profiler with FEAGI components."""
        self.connectome_manager = connectome_manager
        self.core_api = core_api
        self.logger = logger

        # Performance tracking
        self.benchmark_results = {}
        self.batch_size_recommendations = {}

        # Test configuration
        self.test_batch_sizes = [100, 500, 1000, 2500, 5000, 10000, 25000, 50000]
        self.performance_thresholds = {
            "minimum_neurons_per_second": 10000,  # 10k neurons/sec minimum
            "target_neurons_per_second": 50000,  # 50k neurons/sec target
            "optimal_neurons_per_second": 100000,  # 100k neurons/sec optimal
        }

    def setup_test_connectome(self, neuron_count: int = 100000) -> Dict[str, Any]:
        """
        Create a test connectome with structured neuron placement.

        Args:
            neuron_count: Number of neurons to create

        Returns:
            Dictionary with setup statistics
        """
        start_time = time.perf_counter()

        # Create cortical areas in a 3D grid pattern
        areas_per_dimension = max(
            1, int(np.cbrt(neuron_count / 1000))
        )  # ~1000 neurons per area
        area_size = 10  # 10x10x10 neurons per area

        created_areas = 0
        created_neurons = 0

        for x in range(areas_per_dimension):
            for y in range(areas_per_dimension):
                for z in range(areas_per_dimension):
                    if created_neurons >= neuron_count:
                        break

                    # Create cortical area
                    area_name = f"test_area_{x}_{y}_{z}"
                    area_position = (
                        x * (area_size + 1),
                        y * (area_size + 1),
                        z * (area_size + 1),
                    )
                    area_dimensions = (area_size, area_size, area_size)

                    cortical_id = self.connectome_manager.add_cortical_area(
                        name=area_name,
                        dimensions=area_dimensions,
                        position=area_position,
                        area_type="test",
                    )
                    created_areas += 1

                    # Create neurons in this area - FIXED: Use area-relative coordinates
                    neurons_in_area = min(area_size**3, neuron_count - created_neurons)
                    neuron_positions = []

                    for nx in range(area_size):
                        for ny in range(area_size):
                            for nz in range(area_size):
                                if len(neuron_positions) >= neurons_in_area:
                                    break
                                # FIXED: Position neurons within the area boundaries (relative to area position)
                                neuron_positions.append(
                                    (nx, ny, nz)
                                )  # Relative coordinates within area
                            if len(neuron_positions) >= neurons_in_area:
                                break
                        if len(neuron_positions) >= neurons_in_area:
                            break

                    # Batch create neurons
                    created_ids = self.connectome_manager.batch_create_neurons(
                        cortical_id=cortical_id,
                        positions=neuron_positions,
                        threshold=1.0,
                        membrane_potential=0.0,
                    )
                    created_neurons += len(created_ids)

                if created_neurons >= neuron_count:
                    break
            if created_neurons >= neuron_count:
                break

        setup_time = time.perf_counter() - start_time

        return {
            "neuron_count": created_neurons,
            "cortical_area_count": created_areas,
            "setup_time_ms": setup_time * 1000,
            "neurons_per_second_setup": (
                created_neurons / setup_time if setup_time > 0 else 0
            ),
        }

    def profile_coordinate_extraction_methods(
        self, neuron_ids: List[int]
    ) -> Dict[str, Any]:
        """
        Profile different coordinate extraction methods for comparison.

        Args:
            neuron_ids: List of neuron IDs to extract coordinates for

        Returns:
            Performance comparison results
        """
        batch_size = len(neuron_ids)
        results = {
            "batch_size": batch_size,
            "methods": {},
            "validation": {},
            "recommendations": [],
        }

        # Method 1: Dictionary API
        try:
            start_time = time.perf_counter()
            dict_result = self.core_api.get_neuron_coordinates(neuron_ids)
            dict_time = time.perf_counter() - start_time

            if dict_result:
                valid_count = sum(dict_result.get("valid_indices", []))
                results["methods"]["dictionary_api"] = {
                    "time_ms": dict_time * 1000,
                    "neurons_per_second": (
                        valid_count / dict_time if dict_time > 0 else 0
                    ),
                    "valid_neurons": valid_count,
                    "memory_overhead_mb": self._estimate_dict_memory_overhead(
                        dict_result
                    ),
                    "performance_stats": dict_result.get("performance_stats", {}),
                    "success": True,
                }

                # Store for validation
                results["validation"]["dict_coordinates"] = dict_result
            else:
                results["methods"]["dictionary_api"] = {
                    "success": False,
                    "error": "No result returned",
                }

        except Exception as e:
            results["methods"]["dictionary_api"] = {"success": False, "error": str(e)}

        # Method 2: NumPy API (high-performance)
        try:
            start_time = time.perf_counter()
            numpy_result = self.core_api.get_neuron_coordinates_numpy(neuron_ids)
            numpy_time = time.perf_counter() - start_time

            if numpy_result is not None:
                results["methods"]["numpy_api"] = {
                    "time_ms": numpy_time * 1000,
                    "neurons_per_second": (
                        len(numpy_result) / numpy_time if numpy_time > 0 else 0
                    ),
                    "valid_neurons": len(numpy_result),
                    "result_shape": numpy_result.shape,
                    "memory_mb": numpy_result.nbytes / (1024 * 1024),
                    "data_type": str(numpy_result.dtype),
                    "success": True,
                }

                # Store for validation
                results["validation"]["numpy_coordinates"] = numpy_result
            else:
                results["methods"]["numpy_api"] = {
                    "success": False,
                    "error": "No result returned",
                }

        except Exception as e:
            results["methods"]["numpy_api"] = {"success": False, "error": str(e)}

        # Method 3: Built-in benchmark
        try:
            benchmark_result = self.core_api.benchmark_neuron_coordinate_extraction(
                batch_size
            )
            if "error" not in benchmark_result:
                results["methods"]["benchmark_method"] = {
                    "simd_config": benchmark_result.get("simd_config", {}),
                    "benchmark_results": benchmark_result.get("benchmark_results", {}),
                    "performance_comparison": benchmark_result.get(
                        "performance_comparison", {}
                    ),
                    "simd_analysis": benchmark_result.get("simd_analysis", {}),
                    "success": True,
                }
            else:
                results["methods"]["benchmark_method"] = {
                    "success": False,
                    "error": benchmark_result.get("error", "Unknown error"),
                }
        except Exception as e:
            results["methods"]["benchmark_method"] = {"success": False, "error": str(e)}

        # Validate coordinate consistency
        self._validate_coordinate_consistency(results)

        # Generate performance recommendations
        self._generate_performance_recommendations(results)

        return results

    def _estimate_dict_memory_overhead(self, dict_result: Dict[str, Any]) -> float:
        """Estimate memory overhead of dictionary API result."""
        try:
            # Rough estimation of dictionary memory usage
            neuron_count = len(dict_result.get("neuron_ids", []))

            # Each list contains neuron_count elements
            # neuron_ids (int), coordinates_x/y/z (float), valid_indices (bool)
            estimated_bytes = (
                neuron_count * 4  # neuron_ids as int32
                + neuron_count * 4 * 3  # coordinates as float32
                + neuron_count * 1  # valid_indices as bool
                + 1000  # Dictionary overhead, strings, etc.
            )

            return estimated_bytes / (1024 * 1024)
        except Exception:
            return 0.0

    def _validate_coordinate_consistency(self, results: Dict[str, Any]) -> None:
        """Validate that different methods return consistent coordinate data."""
        validation = results["validation"]

        dict_coords = validation.get("dict_coordinates")
        numpy_coords = validation.get("numpy_coordinates")

        if dict_coords and numpy_coords is not None:
            try:
                # Compare valid coordinates
                dict_valid_indices = dict_coords.get("valid_indices", [])
                dict_x = dict_coords.get("coordinates_x", [])
                dict_y = dict_coords.get("coordinates_y", [])
                dict_z = dict_coords.get("coordinates_z", [])

                # Extract valid coordinates from dict result
                valid_dict_coords = []
                for i, valid in enumerate(dict_valid_indices):
                    if valid and i < len(dict_x):
                        valid_dict_coords.append((dict_x[i], dict_y[i], dict_z[i]))

                # NumPy result already contains only valid coordinates
                numpy_coord_tuples = [(row[1], row[2], row[3]) for row in numpy_coords]

                # Compare coordinate counts
                validation["coordinate_count_match"] = len(valid_dict_coords) == len(
                    numpy_coord_tuples
                )

                # Compare actual coordinate values (with tolerance for floating point)
                if (
                    len(valid_dict_coords) == len(numpy_coord_tuples)
                    and len(valid_dict_coords) > 0
                ):
                    coord_differences = []
                    for dict_coord, numpy_coord in zip(
                        valid_dict_coords, numpy_coord_tuples
                    ):
                        diff = np.linalg.norm(
                            np.array(dict_coord) - np.array(numpy_coord)
                        )
                        coord_differences.append(diff)

                    max_diff = max(coord_differences) if coord_differences else 0
                    validation["max_coordinate_difference"] = max_diff
                    validation["coordinates_consistent"] = (
                        max_diff < 0.001
                    )  # 1mm tolerance
                else:
                    validation["coordinates_consistent"] = False
                    validation["max_coordinate_difference"] = float("inf")

            except Exception as e:
                validation["validation_error"] = str(e)
                validation["coordinates_consistent"] = False
        else:
            validation["coordinates_consistent"] = None  # Cannot validate

    def _generate_performance_recommendations(self, results: Dict[str, Any]) -> None:
        """Generate performance optimization recommendations."""
        recommendations = []
        batch_size = results["batch_size"]
        methods = results["methods"]

        # Check if both methods succeeded
        dict_success = methods.get("dictionary_api", {}).get("success", False)
        numpy_success = methods.get("numpy_api", {}).get("success", False)

        if dict_success and numpy_success:
            dict_perf = methods["dictionary_api"]["neurons_per_second"]
            numpy_perf = methods["numpy_api"]["neurons_per_second"]

            if numpy_perf > dict_perf * 1.5:
                recommendations.append(
                    f"NumPy API is {numpy_perf / dict_perf:.1f}x faster - recommend for batch size {batch_size}"
                )
            elif dict_perf > numpy_perf * 1.2:
                recommendations.append(
                    f"Dictionary API unexpectedly faster - investigate NumPy overhead for batch size {batch_size}"
                )
            else:
                recommendations.append(
                    f"Performance similar for batch size {batch_size} - choose based on API preference"
                )

        # Performance threshold analysis
        for method_name, method_data in methods.items():
            if method_data.get("success") and "neurons_per_second" in method_data:
                perf = method_data["neurons_per_second"]

                if perf < self.performance_thresholds["minimum_neurons_per_second"]:
                    recommendations.append(
                        f"{method_name}: Performance below minimum threshold ({perf:.0f} < {self.performance_thresholds['minimum_neurons_per_second']} neurons/sec)"
                    )
                elif perf >= self.performance_thresholds["optimal_neurons_per_second"]:
                    recommendations.append(
                        f"{method_name}: Optimal performance achieved ({perf:.0f} neurons/sec)"
                    )
                elif perf >= self.performance_thresholds["target_neurons_per_second"]:
                    recommendations.append(
                        f"{method_name}: Good performance ({perf:.0f} neurons/sec)"
                    )

        # SIMD analysis recommendations
        benchmark_data = methods.get("benchmark_method", {})
        if benchmark_data.get("success"):
            simd_analysis = benchmark_data.get("simd_analysis", {})
            if simd_analysis:
                utilization = simd_analysis.get("simd_utilization_percent", 0)
                if utilization < 50:
                    recommendations.append(
                        f"Low SIMD utilization ({utilization:.1f}%) - consider larger batch sizes"
                    )
                elif utilization > 80:
                    recommendations.append(
                        f"Excellent SIMD utilization ({utilization:.1f}%)"
                    )

        results["recommendations"] = recommendations

    def run_batch_size_scaling_analysis(self) -> Dict[str, Any]:
        """
        Run scaling analysis across different batch sizes.

        Returns:
            Complete scaling analysis results
        """
        scaling_results = {
            "batch_sizes": self.test_batch_sizes,
            "results_by_batch": {},
            "scaling_analysis": {},
            "optimal_batch_size": None,
        }

        # Test each batch size
        for batch_size in self.test_batch_sizes:
            self.logger.info(
                f"Testing coordinate extraction with batch size: {batch_size}"
            )

            # Generate test neuron IDs
            if hasattr(self.connectome_manager, "neuron_array"):
                max_neurons = self.connectome_manager.get_neuron_count()
                if max_neurons < batch_size:
                    self.logger.warning(
                        f"Insufficient neurons ({max_neurons}) for batch size {batch_size}, skipping"
                    )
                    continue

                # Create representative sample of neuron IDs
                neuron_ids = list(
                    range(
                        0,
                        min(batch_size, max_neurons),
                        max(1, max_neurons // batch_size),
                    )
                )
                if len(neuron_ids) < batch_size:
                    # Fill remaining with random valid IDs
                    remaining = batch_size - len(neuron_ids)
                    random_ids = np.random.choice(
                        max_neurons, size=remaining, replace=False
                    ).tolist()
                    neuron_ids.extend(random_ids)
                    neuron_ids = neuron_ids[:batch_size]  # Ensure exact batch size
            else:
                # Fallback to sequential IDs
                neuron_ids = list(range(batch_size))

            # Profile this batch size
            batch_results = self.profile_coordinate_extraction_methods(neuron_ids)
            scaling_results["results_by_batch"][batch_size] = batch_results

        # Analyze scaling characteristics
        self._analyze_scaling_characteristics(scaling_results)

        return scaling_results

    def _analyze_scaling_characteristics(self, scaling_results: Dict[str, Any]) -> None:
        """Analyze performance scaling characteristics across batch sizes."""
        results_by_batch = scaling_results["results_by_batch"]

        # Extract performance data for each method
        methods_performance = {}

        for batch_size, batch_result in results_by_batch.items():
            for method_name, method_data in batch_result.get("methods", {}).items():
                if method_data.get("success") and "neurons_per_second" in method_data:
                    if method_name not in methods_performance:
                        methods_performance[method_name] = {
                            "batch_sizes": [],
                            "neurons_per_second": [],
                        }

                    methods_performance[method_name]["batch_sizes"].append(batch_size)
                    methods_performance[method_name]["neurons_per_second"].append(
                        method_data["neurons_per_second"]
                    )

        # Analyze each method's scaling
        scaling_analysis = {}

        for method_name, perf_data in methods_performance.items():
            batch_sizes = np.array(perf_data["batch_sizes"])
            performance = np.array(perf_data["neurons_per_second"])

            if len(batch_sizes) >= 2:
                # Find optimal batch size (highest performance)
                optimal_idx = np.argmax(performance)
                optimal_batch_size = batch_sizes[optimal_idx]
                optimal_performance = performance[optimal_idx]

                # Calculate scaling efficiency
                scaling_efficiency = self._calculate_scaling_efficiency(
                    batch_sizes, performance
                )

                # Identify performance dropoff points
                dropoff_points = self._identify_performance_dropoffs(
                    batch_sizes, performance
                )

                scaling_analysis[method_name] = {
                    "optimal_batch_size": int(optimal_batch_size),
                    "optimal_performance": float(optimal_performance),
                    "scaling_efficiency": scaling_efficiency,
                    "performance_dropoff_points": dropoff_points,
                    "scaling_trend": self._categorize_scaling_trend(
                        batch_sizes, performance
                    ),
                }

        scaling_results["scaling_analysis"] = scaling_analysis

        # Overall optimal batch size recommendation
        if scaling_analysis:
            # Prefer NumPy API if available, otherwise use best performing method
            if "numpy_api" in scaling_analysis:
                scaling_results["optimal_batch_size"] = scaling_analysis["numpy_api"][
                    "optimal_batch_size"
                ]
            else:
                # Find method with highest optimal performance
                best_method = max(
                    scaling_analysis.items(), key=lambda x: x[1]["optimal_performance"]
                )
                scaling_results["optimal_batch_size"] = best_method[1][
                    "optimal_batch_size"
                ]

    def _calculate_scaling_efficiency(
        self, batch_sizes: np.ndarray, performance: np.ndarray
    ) -> float:
        """Calculate scaling efficiency (how well performance scales with batch size)."""
        if len(batch_sizes) < 2:
            return 0.0

        # Calculate correlation between batch size and performance
        correlation = np.corrcoef(batch_sizes, performance)[0, 1]

        # Return correlation as efficiency measure (1.0 = perfect scaling, 0.0 = no scaling)
        return max(0.0, correlation) if not np.isnan(correlation) else 0.0

    def _identify_performance_dropoffs(
        self, batch_sizes: np.ndarray, performance: np.ndarray
    ) -> List[int]:
        """Identify batch sizes where performance drops significantly."""
        dropoff_points = []

        if len(performance) < 2:
            return dropoff_points

        # Look for significant performance drops (>20% decrease)
        for i in range(1, len(performance)):
            perf_drop = (performance[i - 1] - performance[i]) / performance[i - 1]
            if perf_drop > 0.2:  # 20% drop
                dropoff_points.append(int(batch_sizes[i]))

        return dropoff_points

    def _categorize_scaling_trend(
        self, batch_sizes: np.ndarray, performance: np.ndarray
    ) -> str:
        """Categorize the scaling trend pattern."""
        if len(performance) < 2:
            return "insufficient_data"

        # Calculate correlation
        correlation = np.corrcoef(batch_sizes, performance)[0, 1]

        if np.isnan(correlation):
            return "no_clear_trend"
        elif correlation > 0.7:
            return "strong_positive_scaling"
        elif correlation > 0.3:
            return "moderate_positive_scaling"
        elif correlation > -0.3:
            return "flat_scaling"
        elif correlation > -0.7:
            return "moderate_negative_scaling"
        else:
            return "strong_negative_scaling"

    def save_profiling_results(
        self, results: Dict[str, Any], output_path: Optional[Path] = None
    ) -> Path:
        """
        Save profiling results to JSON file.

        Args:
            results: Complete profiling results
            output_path: Optional specific output path

        Returns:
            Path to saved results file
        """
        if output_path is None:
            # Use profiling logs directory
            output_dir = Path(__file__).parent / "logs"
            output_dir.mkdir(exist_ok=True)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            output_path = (
                output_dir / f"neuron_coordinate_extraction_profile_{timestamp}.json"
            )

        # Make results JSON-serializable
        json_results = self._make_json_serializable(results)

        with open(output_path, "w") as f:
            json.dump(json_results, f, indent=2)

        self.logger.info(f"Profiling results saved to: {output_path}")
        return output_path

    def _make_json_serializable(self, obj):
        """Recursively convert numpy types to Python native types for JSON serialization."""
        if isinstance(obj, dict):
            return {
                key: self._make_json_serializable(value) for key, value in obj.items()
            }
        elif isinstance(obj, list):
            return [self._make_json_serializable(item) for item in obj]
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, (np.int32, np.int64)):
            return int(obj)
        elif isinstance(obj, (np.float32, np.float64)):
            return float(obj)
        elif isinstance(obj, np.bool_):
            return bool(obj)
        else:
            return obj


# =================================================================
# PYTEST TEST FUNCTIONS
# =================================================================


@pytest.fixture
def test_connectome():
    """Create a test connectome for profiling."""
    # Initialize with performance-optimized backend
    connectome_manager = ConnectomeManager(
        config_or_max_neurons=100000,
        max_synapses=500000,
        backend="auto",  # Auto-select best available backend
    )

    yield connectome_manager

    # Cleanup
    del connectome_manager


@pytest.fixture
def test_core_api(test_connectome):
    """Create a test CoreAPIService with state manager."""
    # Initialize state manager
    state_manager = FeagiStateManager.instance()

    # Create CoreAPIService
    core_api = CoreAPIService(test_connectome, state_manager)

    yield core_api

    # Cleanup
    del core_api


@pytest.fixture
def coordinate_profiler(test_connectome, test_core_api):
    """Create coordinate extraction profiler."""
    profiler = NeuronCoordinateExtractionProfiler(test_connectome, test_core_api)
    yield profiler


def test_coordinate_extraction_setup(coordinate_profiler):
    """Test setting up test connectome for coordinate extraction profiling."""
    # Setup test connectome with 10,000 neurons
    setup_results = coordinate_profiler.setup_test_connectome(neuron_count=10000)

    # Validate setup
    assert setup_results["neuron_count"] > 0, "Should create neurons"
    assert setup_results["cortical_area_count"] > 0, "Should create cortical areas"
    assert setup_results["setup_time_ms"] > 0, "Should record setup time"
    assert setup_results["neurons_per_second_setup"] > 0, (
        "Should have positive setup rate"
    )

    # Performance expectations for setup
    assert setup_results["neurons_per_second_setup"] > 1000, (
        "Setup should be reasonably fast (>1000 neurons/sec)"
    )

    print(
        f"✅ Connectome setup: {setup_results['neuron_count']} neurons in {setup_results['setup_time_ms']:.1f}ms"
    )
    print(f"✅ Setup rate: {setup_results['neurons_per_second_setup']:.0f} neurons/sec")


def test_small_batch_coordinate_extraction(coordinate_profiler):
    """Test coordinate extraction performance for small batches (100-1000 neurons)."""
    # Setup connectome
    setup_results = coordinate_profiler.setup_test_connectome(neuron_count=5000)
    assert setup_results["neuron_count"] >= 1000, "Need sufficient neurons for testing"

    # Test small batch sizes
    small_batch_sizes = [100, 250, 500, 1000]

    for batch_size in small_batch_sizes:
        # Generate test neuron IDs
        neuron_ids = list(range(batch_size))

        # Profile extraction methods
        results = coordinate_profiler.profile_coordinate_extraction_methods(neuron_ids)

        # Validate results
        assert results["batch_size"] == batch_size
        assert "methods" in results
        assert "validation" in results

        # Check that at least one method succeeded
        successful_methods = [
            name for name, data in results["methods"].items() if data.get("success")
        ]
        assert len(successful_methods) > 0, (
            f"At least one method should succeed for batch size {batch_size}"
        )

        # Performance validation for successful methods
        for method_name in successful_methods:
            method_data = results["methods"][method_name]
            if "neurons_per_second" in method_data:
                assert method_data["neurons_per_second"] > 0, (
                    f"{method_name} should have positive performance"
                )
                print(
                    f"✅ {method_name} (batch {batch_size}): {method_data['neurons_per_second']:.0f} neurons/sec"
                )


def test_large_batch_coordinate_extraction(coordinate_profiler):
    """Test coordinate extraction performance for large batches (5000-50000 neurons)."""
    # Setup larger connectome
    setup_results = coordinate_profiler.setup_test_connectome(neuron_count=75000)
    assert setup_results["neuron_count"] >= 50000, (
        "Need sufficient neurons for large batch testing"
    )

    # Test large batch sizes
    large_batch_sizes = [5000, 10000, 25000, 50000]

    for batch_size in large_batch_sizes:
        # Generate test neuron IDs
        max_neurons = setup_results["neuron_count"]
        neuron_ids = list(
            range(0, min(batch_size, max_neurons), max(1, max_neurons // batch_size))
        )

        if len(neuron_ids) < batch_size:
            # Fill with additional IDs as needed
            additional_ids = list(range(len(neuron_ids), min(batch_size, max_neurons)))
            neuron_ids.extend(additional_ids)
            neuron_ids = neuron_ids[:batch_size]

        # Profile extraction methods
        results = coordinate_profiler.profile_coordinate_extraction_methods(neuron_ids)

        # Validate results
        assert results["batch_size"] == len(neuron_ids)

        # Check NumPy method performance for large batches
        numpy_method = results["methods"].get("numpy_api", {})
        if numpy_method.get("success"):
            numpy_performance = numpy_method["neurons_per_second"]

            # Large batches should show good performance
            assert numpy_performance > 1000, (
                f"NumPy method should handle large batches efficiently (got {numpy_performance:.0f} neurons/sec)"
            )
            print(
                f"✅ NumPy API (batch {batch_size}): {numpy_performance:.0f} neurons/sec"
            )

            # Memory efficiency check
            if "memory_mb" in numpy_method:
                memory_per_neuron = (
                    numpy_method["memory_mb"] / len(neuron_ids) * 1024 * 1024
                )  # bytes per neuron
                assert memory_per_neuron < 100, (
                    f"Memory usage should be efficient (<100 bytes/neuron, got {memory_per_neuron:.1f})"
                )
                print(f"✅ Memory efficiency: {memory_per_neuron:.1f} bytes/neuron")


def test_coordinate_extraction_accuracy(coordinate_profiler):
    """Test accuracy and consistency of coordinate extraction methods."""
    # Setup test connectome
    setup_results = coordinate_profiler.setup_test_connectome(neuron_count=2000)

    # Test with medium batch size for accuracy validation
    neuron_ids = list(range(1000))

    # Profile extraction methods
    results = coordinate_profiler.profile_coordinate_extraction_methods(neuron_ids)

    # Check validation results
    validation = results["validation"]

    if validation.get("coordinates_consistent") is not None:
        assert validation["coordinates_consistent"], (
            "Dictionary and NumPy methods should return consistent coordinates"
        )
        print(f"✅ Coordinate consistency validated")

        if "max_coordinate_difference" in validation:
            max_diff = validation["max_coordinate_difference"]
            assert max_diff < 0.001, (
                f"Coordinate differences should be minimal (got {max_diff})"
            )
            print(f"✅ Max coordinate difference: {max_diff:.6f}")

    if validation.get("coordinate_count_match"):
        print(f"✅ Coordinate count consistency validated")

    # Check for validation errors
    if "validation_error" in validation:
        pytest.fail(f"Coordinate validation error: {validation['validation_error']}")


def test_simd_optimization_effectiveness(coordinate_profiler):
    """Test SIMD optimization effectiveness and configuration."""
    # Setup connectome
    setup_results = coordinate_profiler.setup_test_connectome(neuron_count=10000)

    # Test with a batch size that should benefit from SIMD
    neuron_ids = list(range(5000))

    # Profile extraction methods
    results = coordinate_profiler.profile_coordinate_extraction_methods(neuron_ids)

    # Check benchmark method results for SIMD analysis
    benchmark_method = results["methods"].get("benchmark_method", {})

    if benchmark_method.get("success"):
        simd_config = benchmark_method.get("simd_config", {})
        simd_analysis = benchmark_method.get("simd_analysis", {})

        # Log SIMD configuration
        print(f"✅ SIMD Backend: {simd_config.get('backend', 'unknown')}")
        print(f"✅ SIMD Available: {simd_config.get('available', False)}")
        print(f"✅ Vector Width: {simd_config.get('vector_width', 'unknown')}")

        # If SIMD is available, check utilization
        if simd_config.get("available") and simd_analysis:
            utilization = simd_analysis.get("simd_utilization_percent", 0)
            print(f"✅ SIMD Utilization: {utilization:.1f}%")

            # SIMD should provide some benefit
            if "actual_speedup" in simd_analysis:
                speedup = simd_analysis["actual_speedup"]
                assert speedup >= 1.0, (
                    f"SIMD should provide speedup >= 1.0x (got {speedup:.2f}x)"
                )
                print(f"✅ SIMD Speedup: {speedup:.2f}x")


def test_performance_scaling_analysis(coordinate_profiler):
    """Test performance scaling analysis across different batch sizes."""
    # Setup larger connectome for scaling tests
    setup_results = coordinate_profiler.setup_test_connectome(neuron_count=60000)

    # Run scaling analysis
    scaling_results = coordinate_profiler.run_batch_size_scaling_analysis()

    # Validate scaling analysis
    assert "batch_sizes" in scaling_results
    assert "results_by_batch" in scaling_results
    assert "scaling_analysis" in scaling_results

    # Should have tested multiple batch sizes
    tested_batches = len(scaling_results["results_by_batch"])
    assert tested_batches >= 3, (
        f"Should test multiple batch sizes (tested {tested_batches})"
    )

    # Check scaling analysis results
    scaling_analysis = scaling_results["scaling_analysis"]

    for method_name, analysis in scaling_analysis.items():
        print(f"✅ {method_name} scaling analysis:")
        print(f"   Optimal batch size: {analysis['optimal_batch_size']}")
        print(
            f"   Optimal performance: {analysis['optimal_performance']:.0f} neurons/sec"
        )
        print(f"   Scaling trend: {analysis['scaling_trend']}")
        print(f"   Scaling efficiency: {analysis['scaling_efficiency']:.2f}")

        # Validate analysis results
        assert analysis["optimal_batch_size"] > 0, (
            "Should have valid optimal batch size"
        )
        assert analysis["optimal_performance"] > 0, (
            "Should have positive optimal performance"
        )
        assert analysis["scaling_efficiency"] >= 0, (
            "Should have non-negative scaling efficiency"
        )


@pytest.mark.benchmark
def test_coordinate_extraction_benchmark_suite(coordinate_profiler):
    """
    Comprehensive benchmark suite for coordinate extraction performance.

    This test runs the complete profiling suite and saves results.
    """
    # Setup large test connectome
    setup_results = coordinate_profiler.setup_test_connectome(neuron_count=100000)

    print(f"✅ Benchmark setup: {setup_results['neuron_count']} neurons created")

    # Run comprehensive scaling analysis
    scaling_results = coordinate_profiler.run_batch_size_scaling_analysis()

    # Add setup results to scaling results
    scaling_results["setup_results"] = setup_results

    # Performance summary
    scaling_analysis = scaling_results["scaling_analysis"]

    print("\n📊 PERFORMANCE BENCHMARK SUMMARY")
    print("=" * 50)

    for method_name, analysis in scaling_analysis.items():
        print(f"\n{method_name.upper()}:")
        print(f"  • Optimal batch size: {analysis['optimal_batch_size']:,} neurons")
        print(
            f"  • Peak performance: {analysis['optimal_performance']:,.0f} neurons/sec"
        )
        print(f"  • Scaling trend: {analysis['scaling_trend']}")
        print(f"  • Scaling efficiency: {analysis['scaling_efficiency']:.1%}")

        if analysis.get("performance_dropoff_points"):
            print(f"  • Performance drops at: {analysis['performance_dropoff_points']}")

    # Overall recommendation
    if scaling_results.get("optimal_batch_size"):
        print(
            f"\n🎯 RECOMMENDED BATCH SIZE: {scaling_results['optimal_batch_size']:,} neurons"
        )

    # Save benchmark results
    output_path = coordinate_profiler.save_profiling_results(scaling_results)
    print(f"\n💾 Benchmark results saved to: {output_path}")

    # Performance assertions
    best_performance = 0
    for analysis in scaling_analysis.values():
        best_performance = max(best_performance, analysis["optimal_performance"])

    # Should achieve reasonable performance
    min_expected_performance = coordinate_profiler.performance_thresholds[
        "minimum_neurons_per_second"
    ]
    assert best_performance >= min_expected_performance, (
        f"Best performance ({best_performance:.0f} neurons/sec) should exceed minimum threshold "
        f"({min_expected_performance} neurons/sec)"
    )

    print(
        f"\n✅ Benchmark completed successfully - Peak performance: {best_performance:.0f} neurons/sec"
    )


if __name__ == "__main__":
    # Allow running the profiling tests directly
    pytest.main([__file__, "-v", "--tb=short"])
