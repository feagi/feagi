#!/usr/bin/env python
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

"""Benchmark script for comparing standard and GPU-optimized ConnectomeManager implementations.

This script measures and compares the performance of ConnectomeManager and
ConnectomeManagerGPU for various operations, including:
1. Creating neurons
2. Creating synapses
3. Updating membrane potentials (signal propagation)
4. Area-based operations

The benchmark is configurable to test different scales of neural networks.
"""

import argparse
import json
import logging
import os
import time
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.bdu.connectome_manager_gpu import ConnectomeManagerGPU

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class ConnectomeBenchmark:
    """Benchmark class for comparing ConnectomeManager implementations."""

    def __init__(self, output_dir="./benchmark_results"):
        """Initialize the benchmark.

        Args:
            output_dir: Directory to save benchmark results
        """
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.results = {"standard": {}, "gpu_optimized": {}}

    def run_benchmark(
        self,
        max_neurons=1_000_000,
        n_areas=10,
        n_neurons_per_area=10000,
        connectivity_density=0.01,
        n_signals=10000,
    ):
        """Run the benchmark suite.

        Args:
            max_neurons: Maximum number of neurons in the connectome
            n_areas: Number of cortical areas to create
            n_neurons_per_area: Number of neurons in each area
            connectivity_density: Density of connections between neurons (0-1)
            n_signals: Number of signals to propagate in the update benchmark
        """
        total_neurons = n_areas * n_neurons_per_area
        if total_neurons > max_neurons:
            logger.warning(
                f"Total neurons ({total_neurons}) exceeds max_neurons ({max_neurons}). Adjusting parameters."
            )
            n_neurons_per_area = max_neurons // n_areas
            total_neurons = n_areas * n_neurons_per_area

        logger.info(
            f"Running benchmark with {n_areas} areas, {n_neurons_per_area} neurons per area "
            f"({total_neurons} total neurons), and {connectivity_density:.1%} connectivity"
        )

        # Run benchmarks for each implementation
        self._benchmark_impl(
            "standard",
            max_neurons,
            n_areas,
            n_neurons_per_area,
            connectivity_density,
            n_signals,
        )
        self._benchmark_impl(
            "gpu_optimized",
            max_neurons,
            n_areas,
            n_neurons_per_area,
            connectivity_density,
            n_signals,
        )

        # Generate comparison plots
        self._generate_plots()

        # Save the results
        self._save_results()

        return self.results

    def _benchmark_impl(
        self,
        impl_name,
        max_neurons,
        n_areas,
        n_neurons_per_area,
        connectivity_density,
        n_signals,
    ):
        """Run benchmarks for a specific implementation.

        Args:
            impl_name: Name of the implementation ("standard" or "gpu_optimized")
            max_neurons: Maximum number of neurons
            n_areas: Number of cortical areas
            n_neurons_per_area: Number of neurons per area
            connectivity_density: Synapse density
            n_signals: Number of signals to propagate
        """
        logger.info(f"Benchmarking {impl_name} implementation")

        # Create the ConnectomeManager instance
        if impl_name == "standard":
            manager = ConnectomeManager(max_neurons)
        else:
            manager = ConnectomeManagerGPU(max_neurons)

        # Initialize results for this implementation
        self.results[impl_name] = {
            "create_areas": 0,
            "create_neurons": 0,
            "create_neurons_batch": 0,
            "create_synapses": 0,
            "create_synapses_batch": 0,
            "update_membrane_potentials": 0,
            "get_neurons_by_area": 0,
            "cortical_areas": [],
            "neuron_ids": [],
            "synapse_specs": [],
        }

        # Benchmark 1: Creating cortical areas
        logger.info("Benchmarking area creation")
        start_time = time.time()

        for i in range(n_areas):
            area_size = int(np.cbrt(n_neurons_per_area)) + 1
            area_id = manager.add_cortical_area(
                name=f"Area_{i}",
                dimensions=(area_size, area_size, area_size),
                position=(i * 100, 0, 0),
                area_type="custom",
            )
            self.results[impl_name]["cortical_areas"].append(area_id)

        self.results[impl_name]["create_areas"] = time.time() - start_time

        # Benchmark 2: Creating neurons (one by one)
        logger.info("Benchmarking single neuron creation")
        if n_neurons_per_area > 1000:
            # For large neuron counts, only benchmark a subset
            sample_size = 1000
            logger.info(
                f"Using sample of {sample_size} neurons for individual creation benchmark"
            )
        else:
            sample_size = n_neurons_per_area

        area_id = self.results[impl_name]["cortical_areas"][0]
        area_size = int(np.cbrt(n_neurons_per_area)) + 1

        start_time = time.time()
        for i in range(sample_size):
            x = i % area_size
            y = (i // area_size) % area_size
            z = i // (area_size * area_size)
            neuron_id = manager.create_neuron(area_id, (x, y, z))
            self.results[impl_name]["neuron_ids"].append(neuron_id)

        # Scale the time to match the full neuron count
        self.results[impl_name]["create_neurons"] = (time.time() - start_time) * (
            n_neurons_per_area / sample_size
        )

        # Benchmark 3: Creating neurons (batch)
        logger.info("Benchmarking batch neuron creation")

        # Create neurons for the remaining areas in batch
        for area_idx in range(1, n_areas):
            area_id = self.results[impl_name]["cortical_areas"][area_idx]

            # Generate position list
            positions = []
            for i in range(n_neurons_per_area):
                x = i % area_size
                y = (i // area_size) % area_size
                z = i // (area_size * area_size)
                positions.append((x, y, z))

            start_time = time.time()

            if hasattr(manager, "batch_create_neurons"):
                # Use batch method if available
                neuron_ids = manager.batch_create_neurons(area_id, positions)
            else:
                # Fall back to creating one by one
                neuron_ids = []
                for pos in positions:
                    nid = manager.create_neuron(area_id, pos)
                    neuron_ids.append(nid)

            batch_time = time.time() - start_time
            self.results[impl_name]["create_neurons_batch"] += batch_time

            # Store some neuron IDs for later use
            self.results[impl_name]["neuron_ids"].extend(neuron_ids[:100])

        # Average batch time per area
        self.results[impl_name]["create_neurons_batch"] /= n_areas - 1

        # Benchmark 4: Creating synapses (one by one)
        logger.info("Benchmarking single synapse creation")

        # Get a sample of neurons for synapse creation
        all_neuron_ids = []
        for area_idx in range(n_areas):
            area_id = self.results[impl_name]["cortical_areas"][area_idx]
            if hasattr(manager, "get_neurons_by_area"):
                area_neurons = manager.get_neurons_by_area(area_id)
                all_neuron_ids.extend(area_neurons[:100])  # Limit to 100 per area

        n_sample_neurons = min(1000, len(all_neuron_ids))
        n_synapses = int(n_sample_neurons * connectivity_density)

        synapse_specs = []
        for i in range(n_synapses):
            pre = np.random.choice(all_neuron_ids)
            post = np.random.choice(all_neuron_ids)
            weight = np.random.random()
            synapse_specs.append((pre, post, weight))

        self.results[impl_name]["synapse_specs"] = synapse_specs

        # Benchmark single synapse creation (limited to 100)
        start_time = time.time()
        for i in range(min(100, n_synapses)):
            pre, post, weight = synapse_specs[i]
            manager.create_synapse(pre, post, weight)

        # Scale the time to match the full synapse count
        self.results[impl_name]["create_synapses"] = (time.time() - start_time) * (
            n_synapses / min(100, n_synapses)
        )

        # Benchmark 5: Creating synapses (batch)
        logger.info("Benchmarking batch synapse creation")
        start_time = time.time()

        if hasattr(manager, "batch_create_synapses"):
            # Use batch method if available
            manager.batch_create_synapses(synapse_specs)
        else:
            # Fall back to creating one by one
            for pre, post, weight in synapse_specs:
                manager.create_synapse(pre, post, weight)

        self.results[impl_name]["create_synapses_batch"] = time.time() - start_time

        # Benchmark 6: Updating membrane potentials
        logger.info("Benchmarking membrane potential updates")

        # Set some neurons as active to ensure propagation happens
        active_neurons = np.random.choice(all_neuron_ids, size=n_signals, replace=False)
        for neuron_id in active_neurons:
            manager.set_neuron_property(
                neuron_id, "membrane_potential", 2.0
            )  # Above threshold

        start_time = time.time()
        manager.update_membrane_potentials()
        self.results[impl_name]["update_membrane_potentials"] = time.time() - start_time

        # Benchmark 7: Getting neurons by area
        logger.info("Benchmarking get_neurons_by_area")
        area_id = self.results[impl_name]["cortical_areas"][0]

        start_time = time.time()
        neurons = manager.get_neurons_by_area(area_id)
        self.results[impl_name]["get_neurons_by_area"] = time.time() - start_time

    def _generate_plots(self):
        """Generate comparison plots between implementations."""
        logger.info("Generating comparison plots")

        # Set up the plot
        fig, axs = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle("ConnectomeManager Performance Comparison", fontsize=16)

        # Common operations to plot
        operations = [
            ("create_neurons", "Create Neurons\n(One by One)"),
            ("create_neurons_batch", "Create Neurons\n(Batch)"),
            ("create_synapses", "Create Synapses\n(One by One)"),
            ("create_synapses_batch", "Create Synapses\n(Batch)"),
        ]

        # Plot the first set of comparisons
        axs[0, 0].bar(
            [op[1] for op in operations],
            [self.results["standard"][op[0]] for op in operations],
            alpha=0.7,
            label="Standard",
        )
        axs[0, 0].bar(
            [op[1] for op in operations],
            [self.results["gpu_optimized"][op[0]] for op in operations],
            alpha=0.7,
            label="GPU Optimized",
        )
        axs[0, 0].set_ylabel("Time (seconds)")
        axs[0, 0].set_title("Neuron and Synapse Creation Performance")
        axs[0, 0].legend()

        # Calculate speedup
        speedups = []
        op_names = []
        for op_name, _ in operations:
            if self.results["standard"][op_name] > 0:
                speedup = (
                    self.results["standard"][op_name]
                    / self.results["gpu_optimized"][op_name]
                )
                speedups.append(speedup)
                op_names.append(op_name)

        # Plot speedup
        axs[0, 1].bar(op_names, speedups, color="green")
        axs[0, 1].axhline(y=1, color="r", linestyle="-", alpha=0.3)
        axs[0, 1].set_ylabel("Speedup Factor\n(Standard / GPU Optimized)")
        axs[0, 1].set_title("Performance Speedup")

        # Plot membrane potential update time
        update_ops = ["update_membrane_potentials", "get_neurons_by_area"]
        update_labels = ["Update Membrane\nPotentials", "Get Neurons\nBy Area"]

        axs[1, 0].bar(
            update_labels,
            [self.results["standard"][op] for op in update_ops],
            alpha=0.7,
            label="Standard",
        )
        axs[1, 0].bar(
            update_labels,
            [self.results["gpu_optimized"][op] for op in update_ops],
            alpha=0.7,
            label="GPU Optimized",
        )
        axs[1, 0].set_ylabel("Time (seconds)")
        axs[1, 0].set_title("Neural Network Operations Performance")
        axs[1, 0].legend()

        # Calculate speedup for update operations
        update_speedups = []
        for op in update_ops:
            if self.results["standard"][op] > 0:
                speedup = (
                    self.results["standard"][op] / self.results["gpu_optimized"][op]
                )
                update_speedups.append(speedup)

        axs[1, 1].bar(update_labels, update_speedups, color="green")
        axs[1, 1].axhline(y=1, color="r", linestyle="-", alpha=0.3)
        axs[1, 1].set_ylabel("Speedup Factor\n(Standard / GPU Optimized)")
        axs[1, 1].set_title("Update Operations Speedup")

        plt.tight_layout(rect=[0, 0, 1, 0.95])
        plt.savefig(os.path.join(self.output_dir, "connectome_benchmark.png"))
        plt.close()

    def _save_results(self):
        """Save the benchmark results to a JSON file."""
        # Convert results to serializable format
        serializable_results = {"standard": {}, "gpu_optimized": {}}

        for impl in ["standard", "gpu_optimized"]:
            for key, value in self.results[impl].items():
                if key not in ["cortical_areas", "neuron_ids", "synapse_specs"]:
                    serializable_results[impl][key] = value

        with open(
            os.path.join(self.output_dir, "connectome_benchmark_results.json"), "w"
        ) as f:
            json.dump(serializable_results, f, indent=2)

        logger.info(f"Results saved to {self.output_dir}")


def main():
    """Run the benchmark with command-line arguments."""
    parser = argparse.ArgumentParser(description="ConnectomeManager Benchmark")
    parser.add_argument(
        "--max-neurons",
        type=int,
        default=1_000_000,
        help="Maximum number of neurons in the connectome",
    )
    parser.add_argument(
        "--n-areas", type=int, default=5, help="Number of cortical areas to create"
    )
    parser.add_argument(
        "--n-neurons-per-area",
        type=int,
        default=10000,
        help="Number of neurons per area",
    )
    parser.add_argument(
        "--connectivity-density",
        type=float,
        default=0.01,
        help="Connectivity density (0-1)",
    )
    parser.add_argument(
        "--n-signals", type=int, default=1000, help="Number of signals to propagate"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="./benchmark_results",
        help="Directory to save benchmark results",
    )

    args = parser.parse_args()

    benchmark = ConnectomeBenchmark(output_dir=args.output_dir)
    results = benchmark.run_benchmark(
        max_neurons=args.max_neurons,
        n_areas=args.n_areas,
        n_neurons_per_area=args.n_neurons_per_area,
        connectivity_density=args.connectivity_density,
        n_signals=args.n_signals,
    )

    # Print summary results
    print("\n=== BENCHMARK RESULTS ===")
    print("Operation                   | Standard (s) | GPU Optimized (s) | Speedup")
    print("-" * 75)

    operations = [
        "create_areas",
        "create_neurons",
        "create_neurons_batch",
        "create_synapses",
        "create_synapses_batch",
        "update_membrane_potentials",
        "get_neurons_by_area",
    ]

    for op in operations:
        std_time = results["standard"][op]
        gpu_time = results["gpu_optimized"][op]
        if gpu_time > 0:
            speedup = std_time / gpu_time
        else:
            speedup = float("inf")

        print(f"{op:28s} | {std_time:12.6f} | {gpu_time:16.6f} | {speedup:7.2f}x")


if __name__ == "__main__":
    main()
