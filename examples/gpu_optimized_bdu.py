#!/usr/bin/env python3
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
GPU-Optimized BDU Example

This example demonstrates how to create and use the GPU-optimized ConnectomeManager
for improved performance with large-scale neural networks.
"""

import time

import matplotlib.pyplot as plt
import numpy as np

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.bdu.connectome_manager_gpu import ConnectomeManagerGPU


def benchmark_standard_vs_gpu(num_neurons=10000, num_synapses=100000):
    """
    Benchmark standard ConnectomeManager vs GPU-optimized ConnectomeManager.

    Args:
        num_neurons: Number of neurons to create
        num_synapses: Number of synapses to create
    """
    print(f"Benchmarking with {num_neurons} neurons and {num_synapses} synapses")

    # Set up standard ConnectomeManager
    print("\nSetting up standard ConnectomeManager...")
    start_time = time.time()
    std_cm = ConnectomeManager(max_neurons=num_neurons * 2)

    # Create a test area
    area_id = std_cm.add_cortical_area(
        name="Test Area",
        dimensions=(100, 100, 10),
        position=(0, 0, 0),
        area_type="test",
    )

    # Create neurons
    print("Creating neurons...")
    neuron_ids = []
    for i in range(num_neurons):
        x, y, z = i % 100, (i // 100) % 100, i // 10000
        neuron_id = std_cm.create_neuron(
            area_id=area_id, position=(x, y, z), threshold=1.0, decay_rate=0.5
        )
        neuron_ids.append(neuron_id)

    # Create random synapses
    print("Creating synapses...")
    for _i in range(num_synapses):
        pre_id = neuron_ids[np.random.randint(0, len(neuron_ids))]
        post_id = neuron_ids[np.random.randint(0, len(neuron_ids))]
        if pre_id != post_id:
            std_cm.create_synapse(pre_id, post_id, weight=np.random.random())

    std_setup_time = time.time() - start_time
    print(f"Standard setup time: {std_setup_time:.2f} seconds")

    # Convert to GPU-optimized ConnectomeManager
    print("\nConverting to GPU-optimized ConnectomeManager...")
    start_time = time.time()
    # gpu_cm = std_cm.to_gpu_optimized()  # Unused variable removed
    std_cm.to_gpu_optimized()
    gpu_conversion_time = time.time() - start_time
    print(f"GPU conversion time: {gpu_conversion_time:.2f} seconds")

    # Alternatively, create a GPU-optimized ConnectomeManager directly
    print("\nSetting up GPU-optimized ConnectomeManager directly...")
    start_time = time.time()
    direct_gpu_cm = ConnectomeManagerGPU(max_neurons=num_neurons * 2)

    # Create a test area
    gpu_area_id = direct_gpu_cm.add_cortical_area(
        name="Test Area",
        dimensions=(100, 100, 10),
        position=(0, 0, 0),
        area_type="test",
    )

    # Create neurons
    print("Creating neurons...")
    gpu_neuron_ids = []
    for i in range(num_neurons):
        x, y, z = i % 100, (i // 100) % 100, i // 10000
        neuron_id = direct_gpu_cm.create_neuron(
            area_id=gpu_area_id, position=(x, y, z), threshold=1.0, decay_rate=0.5
        )
        gpu_neuron_ids.append(neuron_id)

    # Create random synapses using batch operation for better performance
    print("Creating synapses in batch...")
    synapse_specs = []
    for i in range(num_synapses):
        pre_id = gpu_neuron_ids[np.random.randint(0, len(gpu_neuron_ids))]
        post_id = gpu_neuron_ids[np.random.randint(0, len(gpu_neuron_ids))]
        if pre_id != post_id:
            synapse_specs.append((pre_id, post_id, np.random.random()))

    direct_gpu_cm.batch_create_synapses(synapse_specs)

    gpu_setup_time = time.time() - start_time
    print(f"Direct GPU setup time: {gpu_setup_time:.2f} seconds")

    # Benchmark membrane potential updates
    print("\nBenchmarking membrane potential updates...")

    # Standard ConnectomeManager
    print("Standard ConnectomeManager:")
    num_iterations = 10
    std_times = []

    # Set some neurons to fire initially
    initial_firing = np.random.choice(neuron_ids, size=100, replace=False)
    for neuron_id in initial_firing:
        std_cm.set_neuron_property(
            neuron_id, "membrane_potential", 2.0
        )  # Above threshold

    for i in range(num_iterations):
        start_time = time.time()
        fired = std_cm.update_membrane_potentials()
        duration = time.time() - start_time
        print(
            f"  Iteration {i + 1}: {duration:.4f} seconds, {len(fired)} neurons fired"
        )
        std_times.append(duration)

    # GPU-optimized ConnectomeManager
    print("\nGPU-optimized ConnectomeManager:")
    gpu_times = []

    # Set some neurons to fire initially
    initial_firing = np.random.choice(gpu_neuron_ids, size=100, replace=False)
    for neuron_id in initial_firing:
        direct_gpu_cm.set_neuron_property(
            neuron_id, "membrane_potential", 2.0
        )  # Above threshold

    for i in range(num_iterations):
        start_time = time.time()
        fired = direct_gpu_cm.update_membrane_potentials()
        duration = time.time() - start_time
        print(
            f"  Iteration {i + 1}: {duration:.4f} seconds, {len(fired)} neurons fired"
        )
        gpu_times.append(duration)

    # Plot results
    plt.figure(figsize=(10, 6))
    plt.subplot(1, 2, 1)
    plt.bar(
        ["Standard", "GPU (conversion)", "GPU (direct)"],
        [std_setup_time, gpu_conversion_time, gpu_setup_time],
    )
    plt.title("Setup Time (seconds)")
    plt.ylabel("Time (s)")

    plt.subplot(1, 2, 2)
    plt.plot(range(1, num_iterations + 1), std_times, "b-", label="Standard")
    plt.plot(range(1, num_iterations + 1), gpu_times, "r-", label="GPU")
    plt.title("Membrane Potential Update Time")
    plt.xlabel("Iteration")
    plt.ylabel("Time (s)")
    plt.legend()

    plt.tight_layout()
    plt.savefig("bdu_benchmark.png")
    plt.show()

    # Print summary
    print("\nPerformance Summary:")
    print(f"Standard setup time: {std_setup_time:.2f} seconds")
    print(f"GPU conversion time: {gpu_conversion_time:.2f} seconds")
    print(f"Direct GPU setup time: {gpu_setup_time:.2f} seconds")
    print(f"Average standard update time: {np.mean(std_times):.4f} seconds")
    print(f"Average GPU update time: {np.mean(gpu_times):.4f} seconds")
    print(f"Speedup factor for updates: {np.mean(std_times) / np.mean(gpu_times):.2f}x")


if __name__ == "__main__":
    # Run with smaller numbers for quick demo
    benchmark_standard_vs_gpu(num_neurons=1000, num_synapses=10000)

    # Uncomment for a more comprehensive benchmark
    # benchmark_standard_vs_gpu(num_neurons=10000, num_synapses=100000)
