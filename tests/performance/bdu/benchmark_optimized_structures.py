"""
Benchmark script for optimized data structures.

This script benchmarks the performance of the optimized data structures
against the standard Python implementations.
"""

import time
import random
import numpy as np
import argparse
import logging
import os
import sys
from pathlib import Path

# Add the project root directory to the Python path
project_root = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(project_root))

# Try to import the optimized structures
try:
    from feagi.npu.optimized_structures import (
        GlobalNeuronArray as OptimizedGNA,
        FireCandidateList as OptimizedFCL,
        Connectome as OptimizedConnectome,
        OptimizedFeagiCore,
        RUST_AVAILABLE,
    )
except ImportError:
    RUST_AVAILABLE = False
    logging.warning("Optimized structures not available. Only benchmarking standard implementations.")

# Standard Python implementations (defined inline for comparison)
class StandardGNA:
    """Standard Python implementation of GNA for benchmarking."""
    
    def __init__(self, capacity):
        self.capacity = capacity
        self.membrane_potentials = [0.0] * capacity
        self.thresholds = [1.0] * capacity
        self.refractory_periods = [0] * capacity
        self.refractory_counters = [0] * capacity
        self.last_fired = [0] * capacity
    
    def update_membrane_potentials(self, decay_factor):
        for i in range(self.capacity):
            self.membrane_potentials[i] *= decay_factor
    
    def update_refractory_counters(self):
        for i in range(self.capacity):
            if self.refractory_counters[i] > 0:
                self.refractory_counters[i] -= 1
    
    def find_fire_candidates(self, timestep):
        candidates = []
        for i in range(self.capacity):
            if self.refractory_counters[i] > 0:
                continue
            if self.membrane_potentials[i] >= self.thresholds[i]:
                candidates.append(i)
        return candidates
    
    def process_fired_neurons(self, fired_list, timestep):
        for neuron_id in fired_list:
            self.membrane_potentials[neuron_id] = 0.0
            self.refractory_counters[neuron_id] = self.refractory_periods[neuron_id]
            self.last_fired[neuron_id] = timestep

class StandardFCL:
    """Standard Python implementation of FCL for benchmarking."""
    
    def __init__(self, neuron_ids=None):
        self.neurons = set(neuron_ids or [])
    
    def add(self, neuron_id):
        self.neurons.add(neuron_id)
    
    def add_multiple(self, neuron_ids):
        self.neurons.update(neuron_ids)
    
    def remove(self, neuron_id):
        self.neurons.discard(neuron_id)
    
    def clear(self):
        self.neurons.clear()
    
    def contains(self, neuron_id):
        return neuron_id in self.neurons
    
    def __len__(self):
        return len(self.neurons)
    
    def is_empty(self):
        return len(self.neurons) == 0
    
    def to_list(self):
        return list(self.neurons)
    
    def __iter__(self):
        return iter(self.neurons)

class StandardConnectome:
    """Standard Python implementation of Connectome for benchmarking."""
    
    def __init__(self, neuron_count):
        self.neuron_count = neuron_count
        self.connections = {}  # source_id -> [(target_id, weight)]
    
    def add_connection(self, source_id, target_id, weight, **kwargs):
        if source_id not in self.connections:
            self.connections[source_id] = []
        self.connections[source_id].append((target_id, weight))
    
    def connection_count(self):
        return sum(len(conns) for conns in self.connections.values())
    
    def propagate_activations(self, source_activations, target_buffer):
        for source_id, activation in enumerate(source_activations):
            if activation <= 0.0:
                continue
            
            if source_id not in self.connections:
                continue
            
            for target_id, weight in self.connections[source_id]:
                target_buffer[target_id] += activation * weight
        
        return target_buffer

class StandardFeagiCore:
    """Standard Python implementation of FEAGI Core for benchmarking."""
    
    def __init__(self, neuron_capacity):
        self.gna = StandardGNA(neuron_capacity)
        self.fcl = StandardFCL()
        self.connectome = StandardConnectome(neuron_capacity)
        self.current_timestep = 0
    
    def step(self):
        # 1. Decay membrane potentials
        self.gna.update_membrane_potentials(0.95)
        
        # 2. Update refractory counters
        self.gna.update_refractory_counters()
        
        # 3. Find neurons ready to fire
        fire_candidates = self.gna.find_fire_candidates(self.current_timestep)
        
        # 4. Update the FCL
        self.fcl.clear()
        self.fcl.add_multiple(fire_candidates)
        
        # 5. Process fired neurons
        self.gna.process_fired_neurons(fire_candidates, self.current_timestep)
        
        # Increment timestep
        self.current_timestep += 1
    
    def propagate_activations(self):
        # Create activations array (1.0 for fired neurons)
        activations = [0.0] * self.gna.capacity
        for neuron_id in self.fcl:
            activations[neuron_id] = 1.0
        
        # Create target buffer
        target_buffer = [0.0] * self.gna.capacity
        
        # Propagate activations
        return self.connectome.propagate_activations(activations, target_buffer)

def benchmark_gna(neuron_count, iterations=10):
    """Benchmark GNA performance."""
    print(f"\nBenchmarking GNA with {neuron_count} neurons, {iterations} iterations")
    
    # Create standard GNA
    standard_gna = StandardGNA(neuron_count)
    
    # Initialize with random membrane potentials
    for i in range(neuron_count):
        standard_gna.membrane_potentials[i] = random.random()
    
    # Benchmark standard GNA
    start_time = time.time()
    for i in range(iterations):
        standard_gna.update_membrane_potentials(0.95)
        standard_gna.update_refractory_counters()
        candidates = standard_gna.find_fire_candidates(i)
        standard_gna.process_fired_neurons(candidates, i)
    standard_time = time.time() - start_time
    print(f"Standard GNA: {standard_time:.6f} seconds")
    
    if RUST_AVAILABLE:
        # Create optimized GNA
        optimized_gna = OptimizedGNA(neuron_count)
        
        # Initialize with random membrane potentials
        for i in range(neuron_count):
            optimized_gna.set_membrane_potential(i, random.random())
        
        # Benchmark optimized GNA
        start_time = time.time()
        for i in range(iterations):
            optimized_gna.update_membrane_potentials(0.95)
            optimized_gna.update_refractory_counters()
            candidates = optimized_gna.find_fire_candidates(i)
            optimized_gna.process_fired_neurons(candidates, i)
        optimized_time = time.time() - start_time
        print(f"Optimized GNA: {optimized_time:.6f} seconds")
        
        # Calculate speedup
        speedup = standard_time / optimized_time
        print(f"Speedup: {speedup:.2f}x")
    else:
        print("Optimized GNA not available")

def benchmark_fcl(neuron_count, iterations=10):
    """Benchmark FCL performance."""
    print(f"\nBenchmarking FCL with {neuron_count} neurons, {iterations} iterations")
    
    # Create standard FCL
    standard_fcl = StandardFCL()
    
    # Benchmark standard FCL
    start_time = time.time()
    for i in range(iterations):
        # Add random neurons
        neurons_to_add = [random.randint(0, neuron_count-1) for _ in range(neuron_count // 10)]
        standard_fcl.add_multiple(neurons_to_add)
        
        # Remove some neurons
        for _ in range(neuron_count // 20):
            standard_fcl.remove(random.randint(0, neuron_count-1))
        
        # Check contains for many neurons
        for _ in range(neuron_count // 5):
            standard_fcl.contains(random.randint(0, neuron_count-1))
        
        # Convert to list
        standard_fcl.to_list()
        
        # Clear
        standard_fcl.clear()
    standard_time = time.time() - start_time
    print(f"Standard FCL: {standard_time:.6f} seconds")
    
    if RUST_AVAILABLE:
        # Create optimized FCL
        optimized_fcl = OptimizedFCL()
        
        # Benchmark optimized FCL
        start_time = time.time()
        for i in range(iterations):
            # Add random neurons
            neurons_to_add = [random.randint(0, neuron_count-1) for _ in range(neuron_count // 10)]
            optimized_fcl.add_multiple(neurons_to_add)
            
            # Remove some neurons
            for _ in range(neuron_count // 20):
                optimized_fcl.remove(random.randint(0, neuron_count-1))
            
            # Check contains for many neurons
            for _ in range(neuron_count // 5):
                optimized_fcl.contains(random.randint(0, neuron_count-1))
            
            # Convert to list
            optimized_fcl.to_list()
            
            # Clear
            optimized_fcl.clear()
        optimized_time = time.time() - start_time
        print(f"Optimized FCL: {optimized_time:.6f} seconds")
        
        # Calculate speedup
        speedup = standard_time / optimized_time
        print(f"Speedup: {speedup:.2f}x")
    else:
        print("Optimized FCL not available")

def benchmark_connectome(neuron_count, connection_density=0.01, iterations=10):
    """Benchmark Connectome performance."""
    print(f"\nBenchmarking Connectome with {neuron_count} neurons, density {connection_density}, {iterations} iterations")
    
    # Create standard Connectome
    standard_connectome = StandardConnectome(neuron_count)
    
    # Add random connections
    connection_count = int(neuron_count * neuron_count * connection_density)
    for _ in range(connection_count):
        source_id = random.randint(0, neuron_count-1)
        target_id = random.randint(0, neuron_count-1)
        weight = random.random()
        standard_connectome.add_connection(source_id, target_id, weight)
    
    print(f"Created connectome with {standard_connectome.connection_count()} connections")
    
    # Create random activations
    activations = [0.0] * neuron_count
    for i in range(neuron_count // 10):  # 10% neurons active
        activations[random.randint(0, neuron_count-1)] = 1.0
    
    # Benchmark standard Connectome
    start_time = time.time()
    for i in range(iterations):
        target_buffer = [0.0] * neuron_count
        standard_connectome.propagate_activations(activations, target_buffer)
    standard_time = time.time() - start_time
    print(f"Standard Connectome: {standard_time:.6f} seconds")
    
    if RUST_AVAILABLE:
        # Create optimized Connectome
        optimized_connectome = OptimizedConnectome(neuron_count)
        
        # Add the same connections
        for source_id, targets in standard_connectome.connections.items():
            for target_id, weight in targets:
                optimized_connectome.add_connection(source_id, target_id, weight)
        
        # Benchmark optimized Connectome
        start_time = time.time()
        for i in range(iterations):
            target_buffer = [0.0] * neuron_count
            optimized_connectome.propagate_activations(activations, target_buffer)
        optimized_time = time.time() - start_time
        print(f"Optimized Connectome: {optimized_time:.6f} seconds")
        
        # Calculate speedup
        speedup = standard_time / optimized_time
        print(f"Speedup: {speedup:.2f}x")
    else:
        print("Optimized Connectome not available")

def benchmark_core(neuron_count, connection_density=0.01, iterations=10):
    """Benchmark FEAGI Core performance."""
    print(f"\nBenchmarking FEAGI Core with {neuron_count} neurons, density {connection_density}, {iterations} iterations")
    
    # Create standard Core
    standard_core = StandardFeagiCore(neuron_count)
    
    # Add random connections
    connection_count = int(neuron_count * neuron_count * connection_density)
    for _ in range(connection_count):
        source_id = random.randint(0, neuron_count-1)
        target_id = random.randint(0, neuron_count-1)
        weight = random.random()
        standard_core.connectome.add_connection(source_id, target_id, weight)
    
    # Set random membrane potentials
    for i in range(neuron_count):
        standard_core.gna.membrane_potentials[i] = random.random()
    
    # Benchmark standard Core
    start_time = time.time()
    for i in range(iterations):
        standard_core.step()
        standard_core.propagate_activations()
    standard_time = time.time() - start_time
    print(f"Standard Core: {standard_time:.6f} seconds")
    
    if RUST_AVAILABLE:
        # Create optimized Core
        optimized_core = OptimizedFeagiCore(neuron_count)
        
        # We can't easily add connections or set membrane potentials on the optimized core
        # This benchmark is indicative but not directly comparable
        
        # Benchmark optimized Core
        start_time = time.time()
        for i in range(iterations):
            optimized_core.step()
            optimized_core.propagate_activations()
        optimized_time = time.time() - start_time
        print(f"Optimized Core: {optimized_time:.6f} seconds")
        
        # Calculate speedup
        speedup = standard_time / optimized_time
        print(f"Speedup: {speedup:.2f}x")
    else:
        print("Optimized Core not available")

def main():
    parser = argparse.ArgumentParser(description="Benchmark optimized data structures")
    parser.add_argument("--neuron-count", type=int, default=10000, help="Number of neurons")
    parser.add_argument("--connection-density", type=float, default=0.001, help="Connection density")
    parser.add_argument("--iterations", type=int, default=10, help="Number of iterations")
    args = parser.parse_args()
    
    # Run benchmarks
    benchmark_gna(args.neuron_count, args.iterations)
    benchmark_fcl(args.neuron_count, args.iterations)
    benchmark_connectome(args.neuron_count, args.connection_density, args.iterations)
    benchmark_core(args.neuron_count, args.connection_density, args.iterations)

if __name__ == "__main__":
    main() 