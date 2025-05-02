#!/usr/bin/env python3
"""
Performance benchmark for synaptogenesis across different backends.

This test evaluates the performance of the synaptogenesis process in FEAGI
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
import numpy as np
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any, Union, Callable

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("synaptogenesis_benchmark")

# Add the project root to the path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.insert(0, project_root)

# Import FEAGI modules
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.bdu.neuroembryogenesis import Neuroembryogenesis, DevelopmentStage
from feagi.utils.config import FeagiConfig
from feagi.core.backend import BackendType, get_backend, get_available_backends

# Import test utilities from the central location
# Makes tests directory available in sys.path
tests_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if tests_path not in sys.path:
    sys.path.insert(0, tests_path)

from tests.utils.backend_utils import is_webgpu_available, is_cuda_available, get_system_info


# Custom JSON encoder to handle numpy types
class NumpyEncoder(json.JSONEncoder):
    """JSON encoder that handles numpy data types."""
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        return super().default(obj)


class SynaptogenesisBenchmark:
    """
    Performance benchmark for synaptogenesis across different backends.
    """
    
    def __init__(self, 
                 area_sizes=None,
                 neurons_per_voxel=1,
                 quick_test=False):
        """
        Initialize the benchmark parameters.
        
        Args:
            area_sizes: List of (x, y, z) tuples for area dimensions
            neurons_per_voxel: Number of neurons per voxel
            quick_test: If True, run a smaller subset of tests
        """
        self.timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.neurons_per_voxel = neurons_per_voxel
        
        # Set default area sizes if not provided
        if area_sizes is None:
            if quick_test:
                # Just run the smallest area size for quick testing
                self.area_sizes = [(100, 100, 1)]
            else:
                # Default sizes for full benchmark
                self.area_sizes = [
                    (100, 100, 1),
                    (1000, 1000, 1), 
                    # (10000, 10000, 1),  # Uncomment for very large benchmarks
                ]
        else:
            self.area_sizes = area_sizes
            
        # Create results directory if it doesn't exist
        self.results_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "logs")
        os.makedirs(self.results_dir, exist_ok=True)
        
        # Get system info from the shared utilities
        self.system_info = get_system_info()
        
        self.results = {
            "timestamp": self.timestamp,
            "system_info": self.system_info,
            "parameters": {
                "area_sizes": [(d[0], d[1], d[2]) for d in self.area_sizes],
                "neurons_per_voxel": self.neurons_per_voxel
            }
        }
        
        # Path to the barebones genome file
        self.genome_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), 
                                      "feagi/evo/defaults/genome/barebones_genome.json")
    
    def _get_genome_file(self) -> str:
        """
        Create a temporary copy of the genome file for testing.
        
        Returns:
            Path to the temporary genome file
        """
        # Copy to a temporary file so tests don't modify the original
        with tempfile.NamedTemporaryFile(suffix='.json', delete=False, mode='w') as temp:
            temp_path = temp.name
            
            # Create a minimal genome
            minimal_genome = {
                "blueprint": {},
                "cortical_mappings": [],
                "stats": {
                    "innate_cortical_area_count": 0,
                    "innate_neuron_count": 0,
                    "innate_synapse_count": 0
                }
            }
            
            # Write to temp file
            json.dump(minimal_genome, temp, indent=2)
        
        return temp_path
    
    def _get_config(self, backend_type: BackendType) -> FeagiConfig:
        """Create a configuration for testing with the specified backend."""
        config = FeagiConfig()
        
        with tempfile.TemporaryDirectory() as temp_dir:
            # Basic configuration
            config.set("connectome_path", temp_dir)
            config.set("skip_memory_neurogenesis", True)
            
            # Set higher neuron and synapse limits
            neurons_per_dim = max(self.area_sizes[0][0], self.area_sizes[0][1], self.area_sizes[0][2])
            max_neurons = neurons_per_dim * neurons_per_dim * 3  # Conservative estimate
            config.set("connectome.max_neurons", max_neurons)
            config.set("connectome.max_synapses_per_neuron", 10)  # For one-to-one mapping
            
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
    
    def _calculate_expected_neurons(self, dimensions: Tuple[int, int, int]) -> int:
        """
        Calculate the expected number of neurons for a given area size.
        
        Args:
            dimensions: The dimensions of the cortical area (x, y, z)
            
        Returns:
            Expected number of neurons
        """
        total_voxels = dimensions[0] * dimensions[1] * dimensions[2]
        return total_voxels * self.neurons_per_voxel
    
    def _calculate_expected_synapses(self, dimensions: Tuple[int, int, int]) -> int:
        """
        Calculate the expected number of synapses for a one-to-one mapping.
        
        Args:
            dimensions: The dimensions of the cortical area (x, y, z)
            
        Returns:
            Expected number of synapses
        """
        # For a projector mapping with one-to-one connectivity,
        # there should be one synapse for each neuron in the source area
        return self._calculate_expected_neurons(dimensions)
    
    def print_summary(self):
        """Print a summary of the benchmark results to the console."""
        print("=" * 80)
        print("SYNAPTOGENESIS BENCHMARK SUMMARY")
        print("=" * 80)
        
        # Area size scaling results
        if "size_scaling" in self.results:
            print("\nArea Size Scaling Results:")
            print("-" * 40)
            
            for dim_str, backends in self.results["size_scaling"].items():
                print(f"\nArea dimensions: {dim_str}")
                for backend, result in backends.items():
                    print(f"  Backend: {backend}")
                    print(f"  Total neurons: {result.get('neurons_actual', 'Unknown')}")
                    print(f"  Total synapses: {result.get('synapses_actual', 'Unknown')}")
                    print(f"  Expected synapses: {result.get('synapses_expected', 'Unknown')}")
                    print(f"  Synaptogenesis time: {result.get('time_taken', 0):.3f} seconds")
                    print(f"  Performance: {result.get('synapse_performance', 0):.1f} synapses/second")
                    
    def _run_benchmark_for_backend(self, backend_type: BackendType, dimensions: Tuple[int, int, int]) -> Dict:
        """
        Run the synaptogenesis benchmark for a specific backend with a modified genome.
        
        Args:
            backend_type: The backend type to use
            dimensions: The dimensions to use for the cortical areas
            
        Returns:
            Dictionary with performance results
        """
        dim_str = f"{dimensions[0]}x{dimensions[1]}x{dimensions[2]}"
        logger.info(f"Running benchmark for {backend_type.value} backend with {dim_str} areas with projector mapping...")
        
        # Create a genome file for the benchmark (we'll use a minimal genome)
        genome_file = self._get_genome_file()
        
        try:
            # Set up the configuration
            config = self._get_config(backend_type)
            
            # Create connectome manager with the specified backend
            connectome_manager = ConnectomeManager(config=config)
            
            # Create embryo object and load minimal genome
            setup_start = time.time()
            embryo = Neuroembryogenesis(connectome_manager)
            embryo.load_genome(genome_file)
            
            # Register morphology before creating areas
            connectome_manager._neuroembryogenesis_morphologies_registry = {
                "projector": {
                    "type": "functions",  # This needs to be "functions" not "function" 
                    "parameters": {
                        "source_coordinate_multiplier": [1, 1, 1],
                        "destination_coordinate_multiplier": [1, 1, 1]
                    }
                }
            }
            
            # Create the source and target cortical areas
            # Add source area
            src_area, src_cortical_id = connectome_manager.add_cortical_area(
                name="Source Test Area",
                area_type="interconnect", 
                dimensions=dimensions,
                position=(0, 0, 0),
                properties={
                    "neurons_per_voxel": self.neurons_per_voxel,
                    "fire_t": 1.0,
                    "leak_c": 10,
                    "refrac": 0
                }
            )
            src_area_id = src_area.id
            
            # Add target area
            dst_area, dst_cortical_id = connectome_manager.add_cortical_area(
                name="Target Test Area",
                area_type="interconnect",
                dimensions=dimensions,
                position=(dimensions[0] + 10, 0, 0),
                properties={
                    "neurons_per_voxel": self.neurons_per_voxel,
                    "fire_t": 1.0,
                    "leak_c": 10,
                    "refrac": 0
                }
            )
            dst_area_id = dst_area.id
            
            # Register the areas with the embryo instance
            embryo.cortical_areas[src_area_id] = src_area
            embryo.cortical_id_map[src_area_id] = src_cortical_id
            embryo.reverse_cortical_id_map[src_cortical_id] = src_area_id
            
            embryo.cortical_areas[dst_area_id] = dst_area
            embryo.cortical_id_map[dst_area_id] = dst_cortical_id
            embryo.reverse_cortical_id_map[dst_cortical_id] = dst_area_id
            
            # Add mapping data for synaptogenesis
            if "cortical_mappings" not in embryo.genome:
                embryo.genome["cortical_mappings"] = []
                
            embryo.genome["cortical_mappings"].append({
                "source": src_cortical_id,
                "destination": dst_cortical_id,
                "morphology": {
                    "morphology_id": "projector",
                    "type": "functions",
                    "parameters": {
                        "source_coordinate_multiplier": [1, 1, 1],
                        "destination_coordinate_multiplier": [1, 1, 1],
                        "src_subregion": [[0, 0, 0], dimensions],  # Full source area
                        "dst_subregion": [[0, 0, 0], dimensions],  # Full destination area
                        "connectivity_table": []
                    },
                    "plasticity": False,
                    "morphology_scalar": 1.0,
                    "postSynapticCurrent_multiplier": 1.0
                }
            })
            
            # Create neurons for our custom areas
            embryo._perform_neurogenesis()
            setup_end = time.time()
            setup_time = setup_end - setup_start
            
            # ------------------- Synaptogenesis Phase ------------------
            # Run the synaptogenesis benchmark
            synapse_start = time.time()
            success = embryo._perform_synaptogenesis()
            synapse_end = time.time()
            synapse_time = synapse_end - synapse_start
            
            # ------------------- Gathering Results ------------------
            # Calculate the expected numbers
            neurons_expected = self._calculate_expected_neurons(dimensions) * 2  # Source and target areas
            synapses_expected = self._calculate_expected_synapses(dimensions)
            
            # Get the actual numbers
            neurons_actual = connectome_manager.get_neuron_count()
            synapses_actual = connectome_manager.synapse_manager.get_synapse_count()
            
            # Calculate performance metrics
            neuron_performance = neurons_actual / setup_time if setup_time > 0 else 0
            synapse_performance = synapses_actual / synapse_time if synapse_time > 0 else 0
            
            # Get development statistics
            stats = embryo.get_development_statistics()
            
            # Collect results
            result = {
                "backend": backend_type.value,
                "dimensions": dim_str,
                "success": success,
                "time_taken": synapse_time,
                "neurons_expected": neurons_expected,
                "neurons_actual": neurons_actual,
                "neuron_performance": neuron_performance,
                "synapses_expected": synapses_expected,
                "synapses_actual": synapses_actual,
                "synapse_performance": synapse_performance,
                "statistics": stats,
                "setup_time": setup_time
            }
            
            logger.info(f"Benchmark complete: {synapses_actual} synapses created in {synapse_time:.3f}s")
            return result
            
        except Exception as e:
            logger.error(f"Error in benchmark for {backend_type.value}: {str(e)}")
            import traceback
            traceback.print_exc()
            
            return {
                "backend": backend_type.value,
                "dimensions": dim_str,
                "success": False,
                "error": str(e),
                "traceback": traceback.format_exc()
            }
        finally:
            # Clean up the temporary genome file
            try:
                os.unlink(genome_file)
            except Exception:
                pass
    
    def run_size_scaling(self) -> Dict:
        """
        Run benchmarks for different cortical area dimensions.
        
        Returns:
            Dictionary with benchmark results
        """
        results = {}
        
        # Get available backends
        available_backends = [BackendType.CPU]
        
        if is_webgpu_available():
            available_backends.append(BackendType.WEBGPU)
            
        if is_cuda_available():
            available_backends.append(BackendType.CUDA)
        
        for dimensions in self.area_sizes:
            dim_str = f"{dimensions[0]}x{dimensions[1]}x{dimensions[2]}"
            logger.info(f"Running benchmark for {dim_str} cortical areas...")
            
            scenario_results = {}
            
            # Run for each backend
            for backend_type in available_backends:
                result = self._run_benchmark_for_backend(backend_type, dimensions)
                scenario_results[backend_type.value] = result
            
            results[dim_str] = scenario_results
            
        self.results["size_scaling"] = results
        return results
    
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
        filename = f"synaptogenesis_benchmark_{self.timestamp}.json"
        filepath = os.path.join(self.results_dir, filename)
        
        # Create a serializable version of the results
        serializable_results = self.results.copy()
        
        # Write to JSON file
        with open(filepath, 'w') as f:
            json.dump(serializable_results, f, indent=2, cls=NumpyEncoder)
        
        logger.info(f"Benchmark results saved to {filepath}")
        return filepath
    
    def print_system_specs(self):
        """Print system specifications."""
        print("=" * 80)
        print("SYSTEM SPECIFICATIONS")
        print("=" * 80)
        
        specs = self.system_info
        print(f"OS: {specs.get('os')} {specs.get('os_version')} {specs.get('os_release')}")
        print(f"Python: {specs.get('python_version')} ({specs.get('python_implementation')})")
        print(f"Processor: {specs.get('cpu')}")
        print(f"CPU Cores: {specs.get('cpu_count')}")
        
        if "gpus" in specs and specs["gpus"] != "Unknown":
            for i, gpu in enumerate(specs["gpus"]):
                print(f"GPU {i+1}: {gpu.get('name', 'Unknown')}")
                print(f"  Memory: {gpu.get('memory_total', 'Unknown')} MB")
                print(f"  Driver: {gpu.get('driver', 'Unknown')}")
                
        print(f"Available Backends: {', '.join(specs.get('available_backends', []))}")
        print("=" * 80)


def main():
    """Run the benchmark from the command line."""
    parser = argparse.ArgumentParser(description="Synaptogenesis performance benchmark")
    parser.add_argument("--quick", action="store_true", help="Run a smaller subset of tests")
    parser.add_argument("--essential-only", action="store_true", help="Only run essential tests")
    parser.add_argument("--neuron-per-voxel", type=int, default=1, help="Number of neurons per voxel")
    args = parser.parse_args()
    
    # Create the benchmark instance
    benchmark = SynaptogenesisBenchmark(
        neurons_per_voxel=args.neuron_per_voxel,
        quick_test=args.quick
    )
    
    # Print system specifications
    benchmark.print_system_specs()
    
    # Run the size scaling benchmark
    size_results = benchmark.run_size_scaling()
    
    # Save results to file
    benchmark.save_results()
    
    # Print summary
    benchmark.print_summary()


if __name__ == "__main__":
    main() 