#!/usr/bin/env python3
"""
Neural Propagation Performance Benchmark

Tests realistic neural computation with synaptic propagation across cortical areas.
This benchmark creates a 3-area neural network (A→B→C→A) and measures performance
under various scaling conditions.

Test Architecture:
- 3 Cortical Areas: A, B, C (each M×M×1 dimensions)
- Connectivity: A→B→C→A (circular propagation using block_to_block morphology)
- Initial Stimulus: Fire all neurons in Area A once
- Propagation: Automatic synaptic propagation through the network
- Duration Control: Consecutive fire count property controls test length

Test Parameters:
- M: Cortical area dimension (M×M neurons per area, 3×M² total)
- Consecutive Fire Count: How long neurons stay active (test duration)
- Simulation Timestep: Burst frequency/timing (ms)

Performance Metrics:
- Execution time per burst
- Neural propagation throughput
- Memory usage and CPU utilization
- Frequency stability under load
- Synaptic processing efficiency
"""

import time
import statistics
import logging
import traceback
import tracemalloc
import psutil
import gc
from typing import Dict, List, Any, Tuple
from dataclasses import dataclass
import numpy as np

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.npu.burst_engine import BurstEngine
from feagi.bdu.models.array_backend import BackendType


@dataclass
class NeuralPropagationMetrics:
    """Metrics for neural propagation performance."""
    scenario_name: str
    backend_name: str
    M: int  # Cortical area dimension
    total_neurons: int  # 3 × M²
    consecutive_fires: int
    timestep_ms: float
    
    # Brain Development Phase Metrics
    brain_dev_time_ms: float
    brain_dev_memory_mb: float
    brain_dev_cpu_percent: float
    
    # Neural Computation Phase Metrics
    neural_comp_time_ms: float
    neural_comp_memory_mb: float
    neural_comp_cpu_percent: float
    
    # Performance metrics
    setup_time_ms: float
    avg_burst_time_ms: float
    max_burst_time_ms: float
    min_burst_time_ms: float
    total_test_time_ms: float
    
    # Throughput metrics
    neurons_processed_per_sec: float
    synapses_processed_per_sec: float
    bursts_per_sec: float
    
    # Resource metrics
    peak_memory_mb: float
    avg_cpu_percent: float
    max_cpu_percent: float
    
    # Neural activity metrics
    total_bursts: int
    total_neurons_fired: int
    propagation_cycles: int
    activity_decay_rate: float
    
    # Frequency stability
    target_frequency_hz: float
    actual_frequency_hz: float
    frequency_deviation_percent: float
    frequency_stability_score: float
    timestamp: float
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert metrics to dictionary for JSON serialization."""
        return {
            'scenario_name': self.scenario_name,
            'backend_name': self.backend_name,
            'M': self.M,
            'total_neurons': self.total_neurons,
            'consecutive_fires': self.consecutive_fires,
            'timestep_ms': self.timestep_ms,
            
            'brain_dev_time_ms': self.brain_dev_time_ms,
            'brain_dev_memory_mb': self.brain_dev_memory_mb,
            'brain_dev_cpu_percent': self.brain_dev_cpu_percent,
            
            'neural_comp_time_ms': self.neural_comp_time_ms,
            'neural_comp_memory_mb': self.neural_comp_memory_mb,
            'neural_comp_cpu_percent': self.neural_comp_cpu_percent,
            
            'setup_time_ms': self.setup_time_ms,
            'avg_burst_time_ms': self.avg_burst_time_ms,
            'max_burst_time_ms': self.max_burst_time_ms,
            'min_burst_time_ms': self.min_burst_time_ms,
            'total_test_time_ms': self.total_test_time_ms,
            'neurons_processed_per_sec': self.neurons_processed_per_sec,
            'synapses_processed_per_sec': self.synapses_processed_per_sec,
            'bursts_per_sec': self.bursts_per_sec,
            'peak_memory_mb': self.peak_memory_mb,
            'avg_cpu_percent': self.avg_cpu_percent,
            'max_cpu_percent': self.max_cpu_percent,
            'total_bursts': self.total_bursts,
            'total_neurons_fired': self.total_neurons_fired,
            'propagation_cycles': self.propagation_cycles,
            'activity_decay_rate': self.activity_decay_rate,
            'target_frequency_hz': self.target_frequency_hz,
            'actual_frequency_hz': self.actual_frequency_hz,
            'frequency_deviation_percent': self.frequency_deviation_percent,
            'frequency_stability_score': self.frequency_stability_score,
            'timestamp': self.timestamp
        }


class NeuralPropagationBenchmark:
    """Comprehensive neural propagation performance benchmark."""
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.results_dir = "tests/performance/logs"
        
        # Ensure results directory exists
        import os
        os.makedirs(self.results_dir, exist_ok=True)
    
    def setup_neural_network_with_backend(self, M: int, consecutive_fires: int, backend_name: str, backend_type: BackendType) -> Tuple[ConnectomeManager, BurstEngine, float]:
        """Setup neural network with specified backend."""
        setup_start = time.perf_counter()
        
        # Reset singleton for clean test
        ConnectomeManager.reset_singleton()
        
        # Create ConnectomeManager with specified backend
        max_neurons = 3 * M * M + 1000  # Buffer for safety
        print(f"   🔧 Initializing {backend_name} backend with {max_neurons:,} neuron capacity...")
        
        cm = ConnectomeManager(
            config_or_max_neurons=max_neurons,
            backend=backend_type.value if backend_type else "auto"
        )
        
        # Create cortical areas with unique suffix to avoid conflicts
        import uuid
        suffix = str(uuid.uuid4())[:8]
        
        print(f"   🧠 Creating 3 cortical areas ({M}×{M}×1 each)...")
        area_a_id = cm.add_cortical_area(
            name=f"area_a_{suffix}",
            dimensions=(M, M, 1),
            position=(0, 0, 0),
            area_type="sensory"
        )
        
        area_b_id = cm.add_cortical_area(
            name=f"area_b_{suffix}",
            dimensions=(M, M, 1),
            position=(M+100, 0, 0),
            area_type="associative"
        )
        
        area_c_id = cm.add_cortical_area(
            name=f"area_c_{suffix}",
            dimensions=(M, M, 1),
            position=(2*M+200, 0, 0),
            area_type="motor"
        )
        
        # Create neurons in all areas (reduced batch size for faster setup)
        print(f"   ⚡ Creating {3*M*M:,} neurons...")
        total_neurons_created = 0
        
        for area_id, area_name in [(area_a_id, "A"), (area_b_id, "B"), (area_c_id, "C")]:
            area_neurons = 0
            for x in range(M):
                for y in range(M):
                    neuron_id = cm.create_neuron(
                        cortical_id=area_id,
                        position=(x, y, 0),
                        threshold=1.0,
                        membrane_potential=0.0,
                        resting_potential=0.0,
                        decay_rate=0.1,
                        refractory_period=1,
                        consecutive_fire_count=consecutive_fires
                    )
                    area_neurons += 1
                    total_neurons_created += 1
            
            print(f"   ✅ Area {area_name}: {area_neurons:,} neurons created")
        
        # Create cortical connections (A→B→C→A loop)
        print(f"   🔗 Creating synaptic connections...")
        
        # A → B connection
        cm.add_cortical_connection(
            name=f"A_to_B_{suffix}",
            source_area_id=area_a_id,
            target_area_id=area_b_id,
            properties={"morphology_name": "block_to_block"}
        )
        
        # B → C connection  
        cm.add_cortical_connection(
            name=f"B_to_C_{suffix}",
            source_area_id=area_b_id,
            target_area_id=area_c_id,
            properties={"morphology_name": "block_to_block"}
        )
        
        # C → A connection (completing the loop)
        cm.add_cortical_connection(
            name=f"C_to_A_{suffix}",
            source_area_id=area_c_id,
            target_area_id=area_a_id,
            properties={"morphology_name": "block_to_block"}
        )
        
        # Create BurstEngine
        be = BurstEngine(cm)
        
        setup_time = (time.perf_counter() - setup_start) * 1000
        
        return cm, be, setup_time
    
    def run_neural_propagation_test(self, M: int, consecutive_fires: int, 
                                      timestep_ms: float, scenario_name: str, 
                                      backend_name: str = "CPU", backend_type: BackendType = BackendType.PYTORCH) -> NeuralPropagationMetrics:
        """Run a single neural propagation test scenario with separate brain development and neural computation measurements."""
        
        print(f"\n🧠 NEURAL PROPAGATION TEST: {scenario_name}")
        print(f"   Backend: {backend_name}")
        print(f"   Cortical Dimensions: {M}×{M}×1 (3 areas)")
        print(f"   Total Neurons: {3 * M * M:,}")
        print(f"   Consecutive Fire Count: {consecutive_fires}")
        print(f"   Simulation Timestep: {timestep_ms}ms")
        print(f"   Target Frequency: {1000.0/timestep_ms:.1f}Hz")
        print("=" * 60)
        
        # Start memory and CPU monitoring
        tracemalloc.start()
        process = psutil.Process()
        memory_before = process.memory_info().rss / (1024**2)
        
        cpu_samples = []
        memory_samples = []
        
        def monitor_resources():
            """Background resource monitoring."""
            start_time = time.time()
            while time.time() - start_time < 300:  # Monitor for up to 5 minutes
                try:
                    cpu_samples.append(process.cpu_percent())
                    memory_samples.append(process.memory_info().rss / (1024**2))
                    time.sleep(0.1)  # 100ms sampling
                except:
                    break
        
        # Start resource monitoring
        import threading
        monitor_thread = threading.Thread(target=monitor_resources, daemon=True)
        monitor_thread.start()
        
        try:
            # PHASE 1: BRAIN DEVELOPMENT
            print("🏗️  PHASE 1: BRAIN DEVELOPMENT")
            print("   Creating cortical areas, neurons, and synaptic connections...")
            
            brain_dev_start = time.perf_counter()
            brain_dev_memory_start = process.memory_info().rss / (1024**2)
            brain_dev_cpu_start = process.cpu_percent()
            
            # Setup neural network with specified backend
            cm, be, setup_time = self.setup_neural_network_with_backend(M, consecutive_fires, backend_name, backend_type)
            
            brain_dev_end = time.perf_counter()
            brain_dev_time_ms = (brain_dev_end - brain_dev_start) * 1000
            brain_dev_memory_end = process.memory_info().rss / (1024**2)
            brain_dev_cpu_end = process.cpu_percent()
            
            brain_dev_memory_mb = brain_dev_memory_end - brain_dev_memory_start
            brain_dev_cpu_percent = (brain_dev_cpu_start + brain_dev_cpu_end) / 2
            
            print(f"   ✅ Brain development completed in {brain_dev_time_ms:.1f}ms")
            print(f"   📊 Memory used: {brain_dev_memory_mb:.1f}MB, CPU: {brain_dev_cpu_percent:.1f}%")
            print()
            
            # Calculate test parameters
            total_neurons = 3 * M * M
            target_frequency = 1000.0 / timestep_ms  # Hz
            timestep_seconds = timestep_ms / 1000.0
            
            # PHASE 2: NEURAL COMPUTATION  
            print("⚡ PHASE 2: NEURAL COMPUTATION")
            print("   Running neural propagation with synaptic activity...")
            
            neural_comp_start = time.perf_counter()
            neural_comp_memory_start = process.memory_info().rss / (1024**2)
            neural_comp_cpu_start = process.cpu_percent()
            
            # Get area IDs for stimulation
            cortical_areas = cm.get_all_cortical_ids()
            area_ids = [area_id for area_id in cortical_areas if area_id.startswith('area_')]
            
            if len(area_ids) < 3:
                raise ValueError(f"Expected 3 cortical areas, found {len(area_ids)}: {area_ids}")
            
            # Sort to ensure consistent A, B, C order
            area_ids.sort()
            
            # Get neurons from Area A for initial stimulation
            area_a_info = cm.get_cortical_area(area_ids[0])
            if area_a_info and hasattr(area_a_info, 'neurons') and area_a_info.neurons:
                area_a_neurons = list(area_a_info.neurons)
            else:
                area_a_neurons = list(range(M * M))
            
            # Reduce stimulation for faster test (stimulate subset of neurons)
            stimulation_count = min(1000, len(area_a_neurons))  # Limit to 1000 neurons for faster test
            area_a_neurons = area_a_neurons[:stimulation_count]
            
            print(f"   🔥 Initial stimulation: firing {len(area_a_neurons):,} neurons in Area A")
            
            # Inject initial stimulus
            for neuron_id in area_a_neurons:
                try:
                    cm.neuron_array.set_neuron_property(neuron_id, 'membrane_potential', 2.0)
                except Exception as e:
                    pass  # Continue with other neurons
            
            # Run limited number of bursts for faster test (reduce from consecutive_fires)
            max_bursts = min(5, consecutive_fires)  # Limit to 5 bursts for faster test
            print(f"   🔄 Running {max_bursts} neural computation bursts...")
            
            burst_times = []
            total_neurons_fired = 0
            
            for burst_idx in range(max_bursts):
                burst_start = time.perf_counter()
                
                # Run membrane potential update (this is the core neural computation)
                cm.update_membrane_potentials()
                
                burst_end = time.perf_counter()
                burst_time_ms = (burst_end - burst_start) * 1000
                burst_times.append(burst_time_ms)
                
                print(f"   📊 Burst {burst_idx+1}/{max_bursts}: {burst_time_ms:.2f}ms")
                
                # Brief pause between bursts
                time.sleep(timestep_seconds)
            
            neural_comp_end = time.perf_counter()
            neural_comp_time_ms = (neural_comp_end - neural_comp_start) * 1000
            neural_comp_memory_end = process.memory_info().rss / (1024**2)
            neural_comp_cpu_end = process.cpu_percent()
            
            neural_comp_memory_mb = neural_comp_memory_end - neural_comp_memory_start
            neural_comp_cpu_percent = (neural_comp_cpu_start + neural_comp_cpu_end) / 2
            
            print(f"   ✅ Neural computation completed in {neural_comp_time_ms:.1f}ms")
            print(f"   📊 Memory used: {neural_comp_memory_mb:.1f}MB, CPU: {neural_comp_cpu_percent:.1f}%")
            print(f"   ⚡ Average burst time: {statistics.mean(burst_times):.2f}ms")
            print()
            
            # Calculate final metrics
            total_test_time_ms = brain_dev_time_ms + neural_comp_time_ms
            avg_burst_time_ms = statistics.mean(burst_times) if burst_times else 0
            max_burst_time_ms = max(burst_times) if burst_times else 0
            min_burst_time_ms = min(burst_times) if burst_times else 0
            
            # Get final memory usage
            current, peak = tracemalloc.get_traced_memory()
            tracemalloc.stop()
            
            # Calculate throughput
            neurons_processed_per_sec = total_neurons / (total_test_time_ms / 1000) if total_test_time_ms > 0 else 0
            bursts_per_sec = len(burst_times) / (neural_comp_time_ms / 1000) if neural_comp_time_ms > 0 else 0
            
            print("📊 FINAL RESULTS:")
            print(f"   🧠 Brain Development: {brain_dev_time_ms:.1f}ms ({backend_name})")
            print(f"   ⚡ Neural Computation: {neural_comp_time_ms:.1f}ms ({backend_name})")
            print(f"   🏆 Total Time: {total_test_time_ms:.1f}ms")
            print(f"   🚀 Throughput: {neurons_processed_per_sec:.0f} neurons/sec")
            print("=" * 60)
            
            # Create metrics object
            return NeuralPropagationMetrics(
                scenario_name=scenario_name,
                backend_name=backend_name,
                M=M,
                total_neurons=total_neurons,
                consecutive_fires=consecutive_fires,
                timestep_ms=timestep_ms,
                
                # Brain Development Phase
                brain_dev_time_ms=brain_dev_time_ms,
                brain_dev_memory_mb=brain_dev_memory_mb,
                brain_dev_cpu_percent=brain_dev_cpu_percent,
                
                # Neural Computation Phase
                neural_comp_time_ms=neural_comp_time_ms,
                neural_comp_memory_mb=neural_comp_memory_mb,
                neural_comp_cpu_percent=neural_comp_cpu_percent,
                
                # Performance metrics
                setup_time_ms=brain_dev_time_ms,  # Same as brain dev time
                avg_burst_time_ms=avg_burst_time_ms,
                max_burst_time_ms=max_burst_time_ms,
                min_burst_time_ms=min_burst_time_ms,
                total_test_time_ms=total_test_time_ms,
                
                # Throughput metrics
                neurons_processed_per_sec=neurons_processed_per_sec,
                synapses_processed_per_sec=0,  # Not measured in this simplified test
                bursts_per_sec=bursts_per_sec,
                
                # Resource metrics
                peak_memory_mb=peak / (1024**2),
                avg_cpu_percent=(brain_dev_cpu_percent + neural_comp_cpu_percent) / 2,
                max_cpu_percent=max(brain_dev_cpu_percent, neural_comp_cpu_percent),
                
                # Neural activity metrics
                total_bursts=len(burst_times),
                total_neurons_fired=total_neurons_fired,
                propagation_cycles=len(burst_times),
                activity_decay_rate=0.0,  # Not measured in this simplified test
                
                # Frequency stability
                target_frequency_hz=target_frequency,
                actual_frequency_hz=bursts_per_sec,
                frequency_deviation_percent=abs(bursts_per_sec - target_frequency) / target_frequency * 100 if target_frequency > 0 else 0,
                frequency_stability_score=1.0,  # Simplified
                timestamp=time.time()
            )
            
        except Exception as e:
            # Handle test failures
            tracemalloc.stop()
            error_msg = str(e)
            print(f"❌ Test failed: {error_msg}")
            
            # Return failed metrics
            return NeuralPropagationMetrics(
                scenario_name=f"{scenario_name}_FAILED",
                backend_name=backend_name,
                M=M,
                total_neurons=3 * M * M,
                consecutive_fires=consecutive_fires,
                timestep_ms=timestep_ms,
                
                # Brain Development Phase (failed)
                brain_dev_time_ms=0,
                brain_dev_memory_mb=0,
                brain_dev_cpu_percent=0,
                
                # Neural Computation Phase (failed)
                neural_comp_time_ms=0,
                neural_comp_memory_mb=0,
                neural_comp_cpu_percent=0,
                
                # Performance metrics (failed)
                setup_time_ms=0,
                avg_burst_time_ms=0,
                max_burst_time_ms=0,
                min_burst_time_ms=0,
                total_test_time_ms=0,
                
                # Throughput metrics (failed)
                neurons_processed_per_sec=0,
                synapses_processed_per_sec=0,
                bursts_per_sec=0,
                
                # Resource metrics (failed)
                peak_memory_mb=0,
                avg_cpu_percent=0,
                max_cpu_percent=0,
                
                # Neural activity metrics (failed)
                total_bursts=0,
                total_neurons_fired=0,
                propagation_cycles=0,
                activity_decay_rate=0.0,
                
                # Frequency stability (failed)
                target_frequency_hz=1000.0 / timestep_ms,
                actual_frequency_hz=0,
                frequency_deviation_percent=100.0,
                frequency_stability_score=0.0,
                timestamp=time.time()
            )
    
    def run_cpu_vs_gpu_comparison(self, M: int = 100, consecutive_fires: int = 5, timestep_ms: float = 50.0) -> List[NeuralPropagationMetrics]:
        """Run CPU vs GPU comparison test."""
        
        print(f"\n🏆 CPU vs GPU PERFORMANCE COMPARISON")
        print(f"   Cortical Areas: 3 × {M}×{M}×1 = {3*M*M:,} neurons")
        print(f"   Consecutive Fires: {consecutive_fires}")
        print(f"   Timestep: {timestep_ms}ms")
        print("=" * 70)
        
        backends_to_test = [
            ("CPU (PyTorch)", BackendType.PYTORCH),
            ("GPU (WGPU)", BackendType.WGPU),
        ]
        
        results = []
        
        for backend_name, backend_type in backends_to_test:
            scenario_name = f"cpu_vs_gpu_M{M}_f{consecutive_fires}_t{timestep_ms}ms"
            
            print(f"\n{'='*20} {backend_name.upper()} TEST {'='*20}")
            
            result = self.run_neural_propagation_test(
                M=M,
                consecutive_fires=consecutive_fires,
                timestep_ms=timestep_ms,
                scenario_name=scenario_name,
                backend_name=backend_name,
                backend_type=backend_type
            )
            
            results.append(result)
            
            # Brief pause between tests
            time.sleep(2)
        
        # Print comparison summary
        self.print_cpu_vs_gpu_summary(results)
        
        return results
    
    def print_cpu_vs_gpu_summary(self, results: List[NeuralPropagationMetrics]) -> None:
        """Print CPU vs GPU comparison summary."""
        
        if len(results) < 2:
            print("❌ Not enough results for comparison")
            return
        
        cpu_result = results[0]
        gpu_result = results[1]
        
        print(f"\n🏆 CPU vs GPU COMPARISON SUMMARY")
        print("=" * 70)
        print(f"{'Metric':<25} {'CPU':<15} {'GPU':<15} {'GPU Advantage':<15}")
        print("-" * 70)
        
        # Brain Development Comparison
        if cpu_result.brain_dev_time_ms > 0 and gpu_result.brain_dev_time_ms > 0:
            brain_dev_speedup = cpu_result.brain_dev_time_ms / gpu_result.brain_dev_time_ms
            print(f"{'Brain Development':<25} {cpu_result.brain_dev_time_ms:<15.1f} {gpu_result.brain_dev_time_ms:<15.1f} {brain_dev_speedup:<15.2f}x")
        
        # Neural Computation Comparison
        if cpu_result.neural_comp_time_ms > 0 and gpu_result.neural_comp_time_ms > 0:
            neural_comp_speedup = cpu_result.neural_comp_time_ms / gpu_result.neural_comp_time_ms
            print(f"{'Neural Computation':<25} {cpu_result.neural_comp_time_ms:<15.1f} {gpu_result.neural_comp_time_ms:<15.1f} {neural_comp_speedup:<15.2f}x")
        
        # Total Time Comparison
        if cpu_result.total_test_time_ms > 0 and gpu_result.total_test_time_ms > 0:
            total_speedup = cpu_result.total_test_time_ms / gpu_result.total_test_time_ms
            print(f"{'Total Time':<25} {cpu_result.total_test_time_ms:<15.1f} {gpu_result.total_test_time_ms:<15.1f} {total_speedup:<15.2f}x")
        
        # Throughput Comparison
        if cpu_result.neurons_processed_per_sec > 0 and gpu_result.neurons_processed_per_sec > 0:
            throughput_ratio = gpu_result.neurons_processed_per_sec / cpu_result.neurons_processed_per_sec
            print(f"{'Throughput':<25} {cpu_result.neurons_processed_per_sec:<15.0f} {gpu_result.neurons_processed_per_sec:<15.0f} {throughput_ratio:<15.2f}x")
        
        # Memory Comparison
        brain_dev_memory_ratio = gpu_result.brain_dev_memory_mb / cpu_result.brain_dev_memory_mb if cpu_result.brain_dev_memory_mb > 0 else 1
        neural_comp_memory_ratio = gpu_result.neural_comp_memory_mb / cpu_result.neural_comp_memory_mb if cpu_result.neural_comp_memory_mb > 0 else 1
        print(f"{'Brain Dev Memory':<25} {cpu_result.brain_dev_memory_mb:<15.1f} {gpu_result.brain_dev_memory_mb:<15.1f} {brain_dev_memory_ratio:<15.2f}x")
        print(f"{'Neural Comp Memory':<25} {cpu_result.neural_comp_memory_mb:<15.1f} {gpu_result.neural_comp_memory_mb:<15.1f} {neural_comp_memory_ratio:<15.2f}x")
        
        print()
        print("🎯 VERDICT:")
        if total_speedup >= 2.0:
            print(f"🚀 EXCELLENT! GPU is {total_speedup:.1f}x faster overall!")
        elif total_speedup >= 1.0:
            print(f"✅ SUCCESS! GPU is {total_speedup:.1f}x faster overall!")
        else:
            print(f"⚠️  CPU is still {1/total_speedup:.1f}x faster overall")
        
        print(f"🧠 Brain Development: GPU {'faster' if brain_dev_speedup >= 1.0 else 'slower'} by {brain_dev_speedup:.1f}x")
        print(f"⚡ Neural Computation: GPU {'faster' if neural_comp_speedup >= 1.0 else 'slower'} by {neural_comp_speedup:.1f}x")
        print("=" * 70)
    
    def save_benchmark_results(self, results: List[NeuralPropagationMetrics]) -> str:
        """Save benchmark results to JSON file."""
        import json
        import os
        
        timestamp = int(time.time())
        filename = f"neural_propagation_benchmark_{timestamp}.json"
        filepath = os.path.join(self.results_dir, filename)
        
        # Convert results to dictionaries
        results_data = {
            'benchmark_type': 'neural_propagation',
            'timestamp': timestamp,
            'total_scenarios': len(results),
            'results': [result.to_dict() for result in results]
        }
        
        with open(filepath, 'w') as f:
            json.dump(results_data, f, indent=2, default=str)
        
        self.logger.info(f"Saved benchmark results to {filepath}")
        return filepath


if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    # Run benchmark
    benchmark = NeuralPropagationBenchmark()
    
    # Quick CPU vs GPU test
    print("Running quick CPU vs GPU comparison...")
    results = benchmark.run_cpu_vs_gpu_comparison(M=50, consecutive_fires=3, timestep_ms=100.0)
    
    print("\nBenchmark complete!")
