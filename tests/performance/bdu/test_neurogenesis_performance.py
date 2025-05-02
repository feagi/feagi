#!/usr/bin/env python3
"""
Performance benchmark for neurogenesis across different backends.

This test evaluates the performance of the neurogenesis process in FEAGI
using different available backends (CPU, GPU, etc.) and logs the results.
"""

import os
import sys
import time
import json
import logging
import argparse
import datetime
import tempfile
import cProfile
import pstats
import io
import psutil
import platform
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any, Union, Callable

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("neurogenesis_benchmark")

# Add the project root to the path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.insert(0, project_root)

# Import FEAGI modules
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.bdu.neuroembryogenesis import Neuroembryogenesis, DevelopmentStage
from feagi.utils.config import FeagiConfig
from feagi.core.backend import BackendType, get_backend, get_available_backends


class NeurogenesisPerformanceBenchmark:
    """
    Benchmark for testing neurogenesis performance across backends.
    """
    
    def __init__(self, log_dir: str = None):
        """
        Initialize the benchmark.
        
        Args:
            log_dir: Directory to store logs. Defaults to tests/performance/logs.
        """
        self.log_dir = log_dir or os.path.join(os.path.dirname(os.path.dirname(__file__)), "logs")
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Generate a timestamp for this benchmark run
        self.timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Path to the barebones genome file
        self.genome_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), 
                                       "feagi/evo/defaults/genome/barebones_genome.json")
        
        # Performance data
        self.results = {}
        
        # Define test scenarios
        self.neuron_count_scenario = [10, 100, 1000, 10000]  # Skip the largest values
        self.dimension_scenario = [(1, 1, 1), (10, 10, 10), (100, 100, 100)]  # Skip 1000x1000x1000
        
        # Get system specifications
        self.system_specs = self._get_system_specs()
        
        # Store the process for resource monitoring
        self.process = psutil.Process(os.getpid())
        
    def _get_genome_file(self, modify_func: Optional[Callable[[Dict], Dict]] = None) -> str:
        """
        Create a temporary copy of the genome file for testing.
        
        Args:
            modify_func: Optional function to modify the genome before writing to temp file
        
        Returns:
            Path to the temporary genome file
        """
        # Copy to a temporary file so tests don't modify the original
        with tempfile.NamedTemporaryFile(suffix='.json', delete=False, mode='w') as temp:
            temp_path = temp.name
            
            # Read the essential genome
            with open(self.genome_path, 'r') as f:
                genome = json.load(f)
            
            # Apply modification function if provided
            if modify_func is not None:
                genome = modify_func(genome)
            
            # Write modified genome to temp file
            json.dump(genome, temp, indent=2)
        
        return temp_path
    
    def _create_test_area_genome(self, dimensions: Tuple[int, int, int], neurons_per_voxel: int) -> Dict:
        """
        Create a test genome by adding a test cortical area to the barebones genome.
        
        Args:
            dimensions: (x, y, z) dimensions of the cortical area
            neurons_per_voxel: Number of neurons per voxel
            
        Returns:
            Modified genome dictionary
        """
        def modifier(genome: Dict) -> Dict:
            # Instead of modifying the genome directly, we'll just track what we want to add
            # and will use the connectome_manager.add_cortical_area API later
            
            # Store the dimensions and neurons_per_voxel in the stats
            # so we can use them later when creating the cortical area
            if "benchmark_settings" not in genome:
                genome["benchmark_settings"] = {}
                
            genome["benchmark_settings"]["test_area_dimensions"] = dimensions
            genome["benchmark_settings"]["test_area_neurons_per_voxel"] = neurons_per_voxel
            
            # Update stats (these will be used for informational purposes only)
            total_neurons = dimensions[0] * dimensions[1] * dimensions[2] * neurons_per_voxel
            genome["stats"]["innate_cortical_area_count"] += 1
            genome["stats"]["innate_neuron_count"] += total_neurons
            
            return genome
        
        return modifier
    
    def _get_config(self, backend_type: BackendType) -> FeagiConfig:
        """Create a configuration for testing with the specified backend."""
        with tempfile.TemporaryDirectory() as temp_dir:
            config = FeagiConfig()
            
            # Basic configuration
            config.set("connectome_path", temp_dir)
            config.set("skip_memory_neurogenesis", True)
            config.set("connectome.max_neurons", 100000)  # Use a reasonable max neuron count
            config.set("connectome.max_synapses_per_neuron", 100)
            
            # Backend-specific configuration
            config.set("backend.type", backend_type.value)
            
            if backend_type == BackendType.CUDA:
                config.set("backend.cuda.device_id", 0)
            elif backend_type == BackendType.WEBGPU:
                config.set("backend.wgpu.device_id", 0)
            
            return config
    
    def _run_with_profiling(self, func, *args, **kwargs) -> Tuple[Any, pstats.Stats]:
        """Run a function with cProfile to collect performance metrics."""
        profiler = cProfile.Profile()
        profiler.enable()
        
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        
        profiler.disable()
        
        # Create Stats object for the profiler
        s = io.StringIO()
        ps = pstats.Stats(profiler, stream=s).sort_stats('cumtime')
        
        # Store execution time
        ps.execution_time = end_time - start_time
        
        return result, ps
    
    def _run_benchmark_for_backend(self, backend_type: BackendType, modifier_func: Callable[[Dict], Dict], description: str) -> Dict:
        """
        Run the neurogenesis benchmark for a specific backend with a modified genome.
        
        Args:
            backend_type: The backend type to use
            modifier_func: Function to modify the genome
            description: Description of the test scenario
            
        Returns:
            Dictionary with performance results
        """
        logger.info(f"Running benchmark for {backend_type.value} backend with {description}...")
        
        # Create config and get the backend
        config = self._get_config(backend_type)
        backend = get_backend(backend_type)
        
        if backend is None:
            logger.warning(f"Backend {backend_type.value} is not available, skipping")
            return {
                "status": "skipped",
                "reason": f"Backend {backend_type.value} is not available"
            }
        
        # Create ConnectomeManager with the specified backend
        connectome_manager = ConnectomeManager(config, max_test_neurons=100000)
        
        # Store progress logs
        progress_logs = []
        
        def progress_callback(stage, progress, message):
            progress_logs.append((stage.value, progress, message))
        
        # Create the neuroembryogenesis instance
        embryo = Neuroembryogenesis(
            connectome_manager=connectome_manager,
            config=config,
            progress_callback=progress_callback
        )
        
        # Get the modified genome file
        genome_file = self._get_genome_file(modifier_func)
        
        try:
            # Setup phase timing
            setup_start = time.time()
            embryo.load_genome(genome_file)
            embryo._setup_cortical_areas()
            
            # Add our test cortical area directly using the ConnectomeManager API
            test_area_id = None
            if "benchmark_settings" in embryo.genome:
                test_dims = embryo.genome["benchmark_settings"].get("test_area_dimensions")
                test_neurons_per_voxel = embryo.genome["benchmark_settings"].get("test_area_neurons_per_voxel")
                
                if test_dims and test_neurons_per_voxel:
                    # Generate a new area ID that doesn't conflict with existing ones
                    test_area_id = max(embryo.cortical_areas.keys(), default=-1) + 1
                    
                    # Add the test area to the connectome manager
                    area = connectome_manager.add_cortical_area(
                        area_id=test_area_id,
                        name="Performance Test Area",
                        area_type="interconnect",
                        dimensions=test_dims,
                        position=(0, 0, 0),
                        properties={
                            "neurons_per_voxel": test_neurons_per_voxel
                        }
                    )
                    
                    # Register the test area with the embryo instance
                    embryo.cortical_areas[test_area_id] = area
                    embryo.cortical_id_map[test_area_id] = "benchmark-test-area"
                    embryo.reverse_cortical_id_map["benchmark-test-area"] = test_area_id
                    
                    # We need to add the property to be extracted by the embryo
                    embryo._extract_cortical_properties_cache = getattr(embryo, "_extract_cortical_properties_cache", {})
                    embryo._extract_cortical_properties_cache["benchmark-test-area"] = {
                        "name": "Performance Test Area",
                        "dimensions": test_dims,
                        "position": (0, 0, 0),
                        "neurons_per_voxel": test_neurons_per_voxel,
                        "fire_t": 1.0,
                        "refrac": 0,
                        "leak_c": 10,
                    }
                    
                    # Monkey patch the _extract_cortical_properties method for our test
                    original_extract_method = embryo._extract_cortical_properties
                    
                    def patched_extract_cortical_properties(cortical_id):
                        if cortical_id == "benchmark-test-area" and hasattr(embryo, "_extract_cortical_properties_cache"):
                            return embryo._extract_cortical_properties_cache.get(cortical_id, {})
                        return original_extract_method(cortical_id)
                    
                    embryo._extract_cortical_properties = patched_extract_cortical_properties
                    
                    logger.info(f"Added benchmark test area with dimensions {test_dims} and {test_neurons_per_voxel} neurons per voxel")
            
            setup_end = time.time()
            setup_time = setup_end - setup_start
            
            # Run neurogenesis with profiling
            logger.info(f"Running neurogenesis with {backend_type.value} backend...")
            result, stats = self._run_with_profiling(embryo._perform_neurogenesis)
            neurogenesis_time = stats.execution_time
            
            # Get statistics for each cortical area
            area_stats = {}
            total_neurons = 0
            total_voxels = 0
            
            for area_id, area in embryo.cortical_areas.items():
                neurons = connectome_manager.get_neurons_by_area(area_id)
                neuron_count = len(neurons)
                total_neurons += neuron_count
                
                # Calculate voxel count using the area's dimensions
                dimensions = area.dimensions  # This is already a tuple
                voxel_count = dimensions[0] * dimensions[1] * dimensions[2]
                total_voxels += voxel_count
                
                area_stats[area.name] = {
                    "neuron_count": neuron_count,
                    "voxel_count": voxel_count,
                    "dimensions": {
                        "x": dimensions[0],
                        "y": dimensions[1],
                        "z": dimensions[2]
                    },
                    "neurons_per_voxel": neuron_count / voxel_count if voxel_count > 0 else 0
                }
                
                # Print detailed stats for our test area
                if area_id == test_area_id:
                    expected_neurons = dimensions[0] * dimensions[1] * dimensions[2] * test_neurons_per_voxel
                    if neuron_count != expected_neurons:
                        logger.warning(f"Expected {expected_neurons} neurons in test area, but got {neuron_count}")
            
            # Extract top 10 function calls by cumulative time
            stats.sort_stats('cumtime')
            top_functions = []
            for func, (calls, _, cum_time, _, _) in list(stats.stats.items())[:10]:
                func_name = f"{func[2]}:{func[1]}" if func[2] != "~" else func[2]
                top_functions.append({
                    "function": func_name,
                    "calls": calls,
                    "cumulative_time": cum_time,
                    "time_per_call": cum_time/calls if calls > 0 else 0
                })
            
            # Record memory usage if available
            memory_usage = None
            if hasattr(backend, "get_memory_usage"):
                memory_usage = backend.get_memory_usage()
            
            # Calculate neurons per second
            neurons_per_second = total_neurons / neurogenesis_time if neurogenesis_time > 0 else 0
            
            # Record results
            benchmark_results = {
                "status": "success",
                "backend": backend_type.value,
                "timestamp": self.timestamp,
                "description": description,
                "setup_time": setup_time,
                "neurogenesis_time": neurogenesis_time,
                "total_neurons": total_neurons,
                "total_voxels": total_voxels,
                "neurons_per_voxel_avg": total_neurons / total_voxels if total_voxels > 0 else 0,
                "neurons_per_second": neurons_per_second,
                "memory_usage": memory_usage,
                "top_functions": top_functions,
                "area_stats": area_stats,
                "progress_logs": progress_logs
            }
            
            logger.info(f"Benchmark for {backend_type.value} with {description} completed: "
                        f"{total_neurons} neurons in {neurogenesis_time:.3f}s "
                        f"({neurons_per_second:.1f} neurons/s)")
            
            return benchmark_results
            
        except Exception as e:
            logger.error(f"Error in benchmark for {backend_type.value} with {description}: {str(e)}")
            import traceback
            traceback.print_exc()
            
            return {
                "status": "error",
                "backend": backend_type.value,
                "timestamp": self.timestamp,
                "description": description,
                "error": str(e),
                "traceback": traceback.format_exc()
            }
        
        finally:
            # Clean up the temporary genome file
            try:
                os.unlink(genome_file)
            except Exception:
                pass
    
    def run_neuron_count_scaling(self) -> Dict:
        """
        Run tests with 1x1x1 cortical area with varying neuron counts per voxel.
        """
        logger.info("Running neuron count scaling benchmark...")
        results = {}
        
        # Get available backends
        available_backends = get_available_backends()
        
        # For each neuron count
        for count in self.neuron_count_scenario:
            scenario_results = {}
            description = f"1x1x1 area with {count} neurons/voxel"
            
            # Run for each backend
            for backend_type in available_backends:
                modifier = self._create_test_area_genome((1, 1, 1), count)
                result = self._run_benchmark_for_backend(backend_type, modifier, description)
                scenario_results[backend_type.value] = result
            
            results[str(count)] = scenario_results
        
        self.results["neuron_count_scaling"] = results
        return results
    
    def run_dimension_scaling(self) -> Dict:
        """
        Run tests with varying dimensions but only 1 neuron per voxel.
        """
        logger.info("Running dimension scaling benchmark...")
        results = {}
        
        # Get available backends
        available_backends = get_available_backends()
        
        # For each dimension
        for dimensions in self.dimension_scenario:
            scenario_results = {}
            dim_str = f"{dimensions[0]}x{dimensions[1]}x{dimensions[2]}"
            description = f"{dim_str} area with 1 neuron/voxel"
            
            # Run for each backend
            for backend_type in available_backends:
                modifier = self._create_test_area_genome(dimensions, 1)
                result = self._run_benchmark_for_backend(backend_type, modifier, description)
                scenario_results[backend_type.value] = result
            
            results[dim_str] = scenario_results
        
        self.results["dimension_scaling"] = results
        return results
    
    def run_all_backends(self) -> Dict:
        """
        Run the basic benchmark for all available backends.
        
        Returns:
            Dictionary with results for each backend
        """
        available_backends = get_available_backends()
        logger.info(f"Found {len(available_backends)} available backends: {[b.value for b in available_backends]}")
        
        # Run the original benchmark with the essential genome
        results = {}
        description = "essential genome"
        
        # Run for CPU first as a baseline
        if BackendType.CPU in available_backends:
            results[BackendType.CPU.value] = self._run_benchmark_for_backend(
                BackendType.CPU, 
                lambda x: x,  # No modification to genome
                description
            )
        
        # Then run for all other backends
        for backend_type in available_backends:
            if backend_type != BackendType.CPU:
                results[backend_type.value] = self._run_benchmark_for_backend(
                    backend_type,
                    lambda x: x,  # No modification to genome
                    description
                )
        
        self.results["essential_genome"] = results
        return results
    
    def run_all_tests(self) -> Dict:
        """
        Run all benchmark tests.
        """
        # Run original benchmark with essential genome
        self.run_all_backends()
        
        # Run neuron count scaling tests
        self.run_neuron_count_scaling()
        
        # Run dimension scaling tests
        self.run_dimension_scaling()
        
        return self.results
    
    def save_results(self) -> str:
        """
        Save the benchmark results to a JSON file.
        
        Returns:
            Path to the saved file
        """
        if not self.results:
            logger.warning("No results to save")
            return None
        
        # Create filename with timestamp
        filename = f"neurogenesis_benchmark_{self.timestamp}.json"
        filepath = os.path.join(self.log_dir, filename)
        
        with open(filepath, 'w') as f:
            json.dump(self.results, f, indent=2)
        
        logger.info(f"Benchmark results saved to {filepath}")
        return filepath
    
    def print_summary(self) -> None:
        """Print a summary of the benchmark results."""
        if not self.results:
            logger.warning("No results to summarize")
            return
        
        print("\n" + "=" * 80)
        print("NEUROGENESIS BENCHMARK SUMMARY")
        print("=" * 80)
        
        # Print essential genome results
        if "essential_genome" in self.results:
            print("\nEssential Genome Results:")
            print("-" * 40)
            for backend, result in self.results["essential_genome"].items():
                self._print_result_summary(backend, result)
        
        # Print neuron count scaling results
        if "neuron_count_scaling" in self.results:
            print("\nNeuron Count Scaling Results (1x1x1 area):")
            print("-" * 40)
            for count, backend_results in self.results["neuron_count_scaling"].items():
                print(f"\nNeurons per voxel: {count}")
                for backend, result in backend_results.items():
                    self._print_result_summary(backend, result, indent="  ")
        
        # Print dimension scaling results
        if "dimension_scaling" in self.results:
            print("\nDimension Scaling Results (1 neuron/voxel):")
            print("-" * 40)
            for dimensions, backend_results in self.results["dimension_scaling"].items():
                print(f"\nDimensions: {dimensions}")
                for backend, result in backend_results.items():
                    self._print_result_summary(backend, result, indent="  ")
    
    def _print_result_summary(self, backend: str, result: Dict, indent: str = "") -> None:
        """Helper method to print a summary of a benchmark result."""
        print(f"{indent}Backend: {backend}")
        
        if result["status"] == "success":
            print(f"{indent}Total neurons: {result['total_neurons']}")
            print(f"{indent}Neurogenesis time: {result['neurogenesis_time']:.3f} seconds")
            print(f"{indent}Performance: {result['neurons_per_second']:.1f} neurons/second")
            
            if result.get("memory_usage"):
                print(f"{indent}Memory usage: {result['memory_usage']}")
        
        elif result["status"] == "skipped":
            print(f"{indent}Skipped: {result['reason']}")
        
        else:  # error
            print(f"{indent}Error: {result.get('error', 'Unknown error')}")
    
    def _get_system_specs(self) -> Dict:
        """Get system specifications for benchmarking context."""
        specs = {
            "os": platform.system(),
            "os_version": platform.version(),
            "os_release": platform.release(),
            "python_version": platform.python_version(),
            "processor": platform.processor(),
            "cpu_count": psutil.cpu_count(logical=False),
            "cpu_count_logical": psutil.cpu_count(logical=True),
            "total_memory": psutil.virtual_memory().total,
        }
        
        # Add GPU info if available
        try:
            from feagi.core.resource_mgr import ResourceManager
            resource_mgr = ResourceManager.get_instance()
            resources = resource_mgr.resources
            
            if resources.get("gpu_available", False):
                specs["gpu_info"] = {
                    "gpu_count": resources.get("gpu_count", 0),
                    "gpu_names": resources.get("gpu_names", []),
                    "gpu_memory": resources.get("gpu_memory", [])
                }
                
            if resources.get("metal_available", False):
                specs["metal_available"] = True
                
            if resources.get("webgpu_available", False):
                specs["webgpu_available"] = True
        except Exception as e:
            logger.warning(f"Failed to get GPU information: {e}")
            
        return specs

    def _monitor_resources(self, start=False):
        """Monitor system resources during benchmark execution."""
        if start:
            # Reset peak values
            self._peak_memory = 0
            self._peak_cpu = 0
            return
            
        # Update resource usage peaks
        memory_info = self.process.memory_info()
        memory_usage = memory_info.rss
        cpu_percent = self.process.cpu_percent()
        
        self._peak_memory = max(self._peak_memory, memory_usage)
        self._peak_cpu = max(self._peak_cpu, cpu_percent)

    def print_system_specs(self):
        """Print system specifications."""
        print("\n" + "=" * 80)
        print("SYSTEM SPECIFICATIONS")
        print("=" * 80)
        
        specs = self.system_specs
        print(f"OS: {specs.get('os')} {specs.get('os_version')} {specs.get('os_release')}")
        print(f"Python: {specs.get('python_version')}")
        print(f"Processor: {specs.get('processor')}")
        print(f"CPU Cores: {specs.get('cpu_count')} (Physical), {specs.get('cpu_count_logical')} (Logical)")
        print(f"Total Memory: {specs.get('total_memory') / (1024**3):.2f} GB")
        
        if "gpu_info" in specs:
            gpu_info = specs["gpu_info"]
            print(f"GPU Count: {gpu_info.get('gpu_count', 0)}")
            for i, (name, memory) in enumerate(zip(
                    gpu_info.get('gpu_names', []), 
                    gpu_info.get('gpu_memory', [])
                )):
                print(f"  GPU {i}: {name}, Memory: {memory}")
                
        if specs.get("metal_available"):
            print("Apple Metal: Available")
            
        if specs.get("webgpu_available"):
            print("WebGPU: Available")


def main():
    """Run the benchmark from the command line."""
    parser = argparse.ArgumentParser(description="Neurogenesis Performance Benchmark")
    parser.add_argument("--log-dir", default=None, help="Directory to store logs (defaults to tests/performance/logs)")
    parser.add_argument("--neuron-count-only", action="store_true", help="Run only neuron count scaling tests")
    parser.add_argument("--dimension-only", action="store_true", help="Run only dimension scaling tests")
    parser.add_argument("--essential-only", action="store_true", help="Run only essential genome tests")
    parser.add_argument("--quick", action="store_true", help="Run a quick benchmark with minimal tests")
    args = parser.parse_args()
    
    benchmark = NeurogenesisPerformanceBenchmark(log_dir=args.log_dir)
    
    # Print system specifications
    benchmark.print_system_specs()
    
    # If quick mode is enabled, reduce the test scenarios
    if args.quick:
        benchmark.neuron_count_scenario = [10, 100, 1000]  # Only test smaller neuron counts
        benchmark.dimension_scenario = [(1, 1, 1), (10, 10, 10)]  # Only test smaller dimensions
    
    # Run specified tests based on arguments
    if args.neuron_count_only:
        benchmark.run_neuron_count_scaling()
    elif args.dimension_only:
        benchmark.run_dimension_scaling()
    elif args.essential_only:
        benchmark.run_all_backends()
    else:
        # Run all tests by default
        benchmark.run_all_tests()
    
    benchmark.save_results()
    benchmark.print_summary()


if __name__ == "__main__":
    main() 