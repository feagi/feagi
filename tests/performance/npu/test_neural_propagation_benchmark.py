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


@dataclass
class NeuralPropagationMetrics:
    """Metrics for neural propagation performance."""
    scenario_name: str
    M: int  # Cortical area dimension
    total_neurons: int  # 3 × M²
    consecutive_fires: int
    timestep_ms: float
    
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
        """Convert metrics to dictionary for serialization."""
        return {
            'scenario_name': self.scenario_name,
            'M': self.M,
            'total_neurons': self.total_neurons,
            'consecutive_fires': self.consecutive_fires,
            'timestep_ms': self.timestep_ms,
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
    
    def create_test_scenarios(self) -> List[Dict[str, Any]]:
        """Create comprehensive test scenarios with systematic parameter sweeps."""
        
        # Base scenario for reference
        base_scenario = {
            "M": 50,
            "consecutive_fires": 10,
            "timestep_ms": 50.0,
            "name": "baseline"
        }
        
        scenarios = []
        
        # 1. M Dimension Sweep (keeping other params constant)
        print("🔬 Creating M dimension sweep scenarios...")
        m_values = [10, 25, 50, 100, 200, 500, 1000, 1500, 2000]
        for m in m_values:
            scenarios.append({
                "M": m,
                "consecutive_fires": base_scenario["consecutive_fires"],
                "timestep_ms": base_scenario["timestep_ms"],
                "name": f"M_sweep_{m}"
            })
        
        # 2. Consecutive Fires Sweep (keeping other params constant)
        print("🔥 Creating consecutive fires sweep scenarios...")
        fire_counts = [1, 3, 5, 10, 20, 50, 100, 200]
        for fires in fire_counts:
            scenarios.append({
                "M": base_scenario["M"],
                "consecutive_fires": fires,
                "timestep_ms": base_scenario["timestep_ms"],
                "name": f"fires_sweep_{fires}"
            })
        
        # 3. Timestep Sweep (keeping other params constant)
        print("⏱️ Creating timestep sweep scenarios...")
        timesteps = [100.0, 50.0, 25.0, 10.0, 5.0, 2.0, 1.0]
        for ts in timesteps:
            scenarios.append({
                "M": base_scenario["M"],
                "consecutive_fires": base_scenario["consecutive_fires"],
                "timestep_ms": ts,
                "name": f"timestep_sweep_{ts}ms"
            })
        
        # 4. Extreme Stress Scenarios
        print("🚀 Creating extreme stress scenarios...")
        extreme_scenarios = [
            # Ultra-high neuron count
            {"M": 2000, "consecutive_fires": 10, "timestep_ms": 50.0, "name": "extreme_neurons_2000x2000"},
            {"M": 2000, "consecutive_fires": 5, "timestep_ms": 25.0, "name": "extreme_neurons_fast"},
            {"M": 2000, "consecutive_fires": 20, "timestep_ms": 5.0, "name": "extreme_neurons_ultra_fast"},
            
            # Ultra-high frequency
            {"M": 100, "consecutive_fires": 10, "timestep_ms": 5.0, "name": "ultra_high_freq_5ms"},
            {"M": 200, "consecutive_fires": 10, "timestep_ms": 5.0, "name": "ultra_high_freq_5ms_large"},
            {"M": 500, "consecutive_fires": 5, "timestep_ms": 5.0, "name": "ultra_high_freq_5ms_massive"},
            
            # Long duration tests
            {"M": 100, "consecutive_fires": 500, "timestep_ms": 10.0, "name": "long_duration_500_fires"},
            {"M": 200, "consecutive_fires": 1000, "timestep_ms": 20.0, "name": "very_long_duration_1000_fires"},
            
            # Combined extreme scenarios
            {"M": 1000, "consecutive_fires": 100, "timestep_ms": 10.0, "name": "combined_extreme_1000x1000"},
            {"M": 1500, "consecutive_fires": 50, "timestep_ms": 10.0, "name": "combined_extreme_1500x1500"},
            {"M": 2000, "consecutive_fires": 25, "timestep_ms": 10.0, "name": "ultimate_stress_2000x2000"},
        ]
        
        scenarios.extend(extreme_scenarios)
        
        # 5. Frequency Stability Test Scenarios
        print("📊 Creating frequency stability scenarios...")
        stability_scenarios = [
            {"M": 50, "consecutive_fires": 100, "timestep_ms": 5.0, "name": "stability_50x50_5ms"},
            {"M": 100, "consecutive_fires": 100, "timestep_ms": 5.0, "name": "stability_100x100_5ms"},
            {"M": 200, "consecutive_fires": 100, "timestep_ms": 5.0, "name": "stability_200x200_5ms"},
            {"M": 500, "consecutive_fires": 50, "timestep_ms": 5.0, "name": "stability_500x500_5ms"},
            {"M": 1000, "consecutive_fires": 25, "timestep_ms": 5.0, "name": "stability_1000x1000_5ms"},
        ]
        
        scenarios.extend(stability_scenarios)
        
        print(f"📋 Created {len(scenarios)} test scenarios:")
        print(f"   - M dimension sweep: {len(m_values)} scenarios")
        print(f"   - Consecutive fires sweep: {len(fire_counts)} scenarios")
        print(f"   - Timestep sweep: {len(timesteps)} scenarios")
        print(f"   - Extreme stress: {len(extreme_scenarios)} scenarios")
        print(f"   - Frequency stability: {len(stability_scenarios)} scenarios")
        
        return scenarios
    
    def setup_neural_network(self, M: int, consecutive_fires: int) -> Tuple[ConnectomeManager, BurstEngine]:
        """Set up the 3-area neural network with A→B→C→A connectivity."""
        
        setup_start = time.perf_counter()
        
        # Calculate total neurons needed (3 areas × M²)
        total_neurons = 3 * M * M
        
        # Create ConnectomeManager with sufficient capacity
        # Use unique timestamp to avoid singleton conflicts in testing
        import uuid
        unique_suffix = str(uuid.uuid4())[:8]
        cm = ConnectomeManager(config_or_max_neurons=total_neurons + 1000)  # Extra buffer
        
        # Create 3 cortical areas (A, B, C)
        area_ids = []
        for i, area_name in enumerate(['A', 'B', 'C']):
            area_id = f"area_{area_name}_{unique_suffix}"  # Make unique
            
            # Add cortical area with M×M×1 dimensions
            cm.add_cortical_area(
                name=f"Area_{area_name}_{unique_suffix}",  # Make unique
                dimensions=(M, M, 1),
                position=(i * M, 0, 0),  # Position areas side by side
                area_type="custom",
                properties={
                    "cortical_neuron_count": M * M,
                    "consecutive_fire_count": consecutive_fires,  # Control test duration
                    "firing_threshold": 1.0,
                    "refractory_period": 1,
                    "leak_coefficient": 0.1,
                    "leak_variability": 0.0,
                    "consecutive_fire_count_variability": 0.0,
                    "firing_threshold_increment": 0.0,
                    "firing_threshold_limit": 1.0,
                    "postsynaptic_current": 1.0,
                    "plasticity_constant": 0.0,
                    "postsynaptic_current_max": 1.0,
                    "neighbor_locator_rule_id": "rule_1",
                    "neighbor_locator_rule_param_id": "param_1"
                },
                cortical_id=area_id
            )
            area_ids.append(area_id)
            
            self.logger.info(f"Created cortical area {area_name}: {M}×{M}×1 structure")
            
            # CRITICAL FIX: Create neurons in the cortical area
            # The add_cortical_area() method only creates the area structure,
            # neurons must be explicitly created
            neuron_count = M * M
            
            self.logger.info(f"Creating {neuron_count} neurons in area {area_name}")
            
            # Create neurons at valid positions within the area bounds
            created_neurons = []
            for x in range(M):
                for y in range(M):
                    for z in range(1):  # Depth is 1
                        neuron_id = cm.create_neuron(
                            cortical_id=area_id,
                            position=(x, y, z),
                            threshold=1.0,
                            membrane_potential=0.0,
                            resting_potential=0.0,
                            decay_rate=0.1,
                            refractory_period=1
                        )
                        created_neurons.append(neuron_id)
            
            self.logger.info(f"✅ Created {len(created_neurons)} neurons in area {area_name}")
        
        # Create circular connectivity: A→B→C→A using probabilistic connectivity
        connections = [
            (area_ids[0], area_ids[1], "A_to_B"),  # A → B
            (area_ids[1], area_ids[2], "B_to_C"),  # B → C  
            (area_ids[2], area_ids[0], "C_to_A"),  # C → A
        ]
        
        for source_area, target_area, conn_name in connections:
            # Add connectivity rule
            rule_id = cm.add_connectivity_rule(
                name=conn_name,
                source_area_id=source_area,
                target_area_id=target_area,
                rule_type="probabilistic",  # Use probabilistic connectivity
                parameters={
                    "connection_probability": 0.1,  # 10% connectivity
                    "synapse_weight": 1.0,
                    "max_synapses_per_neuron": 100
                },
                enabled=True
            )
            
            # Apply the connectivity rule to create synapses
            synapses_created = cm.apply_connectivity_rule(
                rule_id=rule_id,
                max_synapses=M * M * 10  # Allow up to 10 synapses per neuron on average
            )
            
            self.logger.info(f"Created {conn_name}: {synapses_created:,} synapses")
        
        # Create BurstEngine
        be = BurstEngine(cm)
        
        setup_time = (time.perf_counter() - setup_start) * 1000
        
        self.logger.info(f"Neural network setup complete:")
        self.logger.info(f"  - 3 areas × {M}×{M} = {total_neurons:,} total neurons")
        self.logger.info(f"  - Consecutive fires: {consecutive_fires}")
        self.logger.info(f"  - Setup time: {setup_time:.1f}ms")
        
        return cm, be, setup_time
    
    def run_neural_propagation_test(self, M: int, consecutive_fires: int, 
                                  timestep_ms: float, scenario_name: str) -> NeuralPropagationMetrics:
        """Run a single neural propagation test scenario."""
        
        self.logger.info(f"🧠 Starting neural propagation test: {scenario_name}")
        self.logger.info(f"   M={M}, fires={consecutive_fires}, timestep={timestep_ms}ms")
        
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
            # Setup neural network
            cm, be, setup_time = self.setup_neural_network(M, consecutive_fires)
            
            # Calculate test parameters
            total_neurons = 3 * M * M
            target_frequency = 1000.0 / timestep_ms  # Hz
            timestep_seconds = timestep_ms / 1000.0
            
            # Get area IDs for stimulation (they were created with unique suffix in setup)
            # We need to get the actual area IDs from the ConnectomeManager
            cortical_areas = cm.get_all_cortical_ids()
            area_ids = [area_id for area_id in cortical_areas if area_id.startswith('area_')]
            
            if len(area_ids) < 3:
                raise ValueError(f"Expected 3 cortical areas, found {len(area_ids)}: {area_ids}")
            
            # Sort to ensure consistent A, B, C order
            area_ids.sort()
            
            # Initial stimulation: Fire all neurons in Area A
            # Get the actual neuron IDs from Area A
            area_a_info = cm.get_cortical_area(area_ids[0])
            if area_a_info and hasattr(area_a_info, 'neurons') and area_a_info.neurons:
                # area.neurons is a set, not a dict
                area_a_neurons = list(area_a_info.neurons)
            else:
                # Fallback: assume first M² neurons belong to Area A
                area_a_neurons = list(range(M * M))
            
            self.logger.info(f"🔥 Initial stimulation: firing {len(area_a_neurons):,} neurons in Area A")
            
            # Inject initial stimulus by setting membrane potentials above threshold
            for neuron_id in area_a_neurons:
                try:
                    # Set membrane potential above firing threshold to ensure firing
                    success = cm.neuron_array.set_neuron_property(neuron_id, 'membrane_potential', 2.0)  # Above threshold of 1.0
                    if not success:
                        self.logger.debug(f"Failed to set membrane potential for neuron {neuron_id}")
                except Exception as e:
                    # If direct setting fails, try alternative approach
                    self.logger.debug(f"Failed to set membrane potential for neuron {neuron_id}: {e}")
            
            # Also inject into FCL for good measure
            initial_stimulus = {area_ids[0]: area_a_neurons}
            
            # Get FCL manager and inject initial activity
            fcl_manager = cm._get_fcl_manager()
            if fcl_manager:
                fcl_manager.update_fcl(0, initial_stimulus)
            
            # Run propagation simulation
            burst_times = []
            fired_neuron_counts = []
            actual_frequencies = []
            
            test_start = time.perf_counter()
            last_burst_time = test_start
            
            # Estimate maximum bursts based on consecutive fires
            # Each neuron can fire for 'consecutive_fires' bursts, so test should run
            # long enough for activity to propagate and decay
            max_bursts = consecutive_fires * 5  # 5x safety factor for propagation
            max_test_time = 60.0  # Maximum 60 seconds per test
            
            burst_count = 0
            total_neurons_fired = 0
            
            self.logger.info(f"🚀 Starting propagation simulation (max {max_bursts} bursts)")
            
            for burst_num in range(max_bursts):
                # Calculate target burst time
                target_burst_time = test_start + (burst_num * timestep_seconds)
                current_time = time.perf_counter()
                
                # Wait if we're ahead of schedule
                if current_time < target_burst_time:
                    time.sleep(target_burst_time - current_time)
                
                # Execute burst
                burst_start = time.perf_counter()
                
                # Process neural propagation
                fired_neurons = cm.update_membrane_potentials()
                
                burst_end = time.perf_counter()
                burst_duration = (burst_end - burst_start) * 1000  # ms
                burst_times.append(burst_duration)
                
                # Track fired neurons
                fired_count = len(fired_neurons) if fired_neurons else 0
                fired_neuron_counts.append(fired_count)
                total_neurons_fired += fired_count
                
                # Calculate actual frequency
                if burst_num > 0:
                    actual_interval = burst_end - last_burst_time
                    actual_freq = 1.0 / actual_interval if actual_interval > 0 else 0
                    actual_frequencies.append(actual_freq)
                
                last_burst_time = burst_end
                burst_count += 1
                
                # Log progress every 10 bursts
                if burst_num % 10 == 0 and burst_num > 0:
                    avg_burst_time = statistics.mean(burst_times[-10:])
                    self.logger.info(f"   Burst {burst_num}: {fired_count} neurons fired, "
                                   f"{avg_burst_time:.2f}ms avg burst time")
                
                # Stop if no activity (network has settled)
                if fired_count == 0 and burst_num > consecutive_fires:
                    self.logger.info(f"   Network activity ceased at burst {burst_num}")
                    break
                
                # Stop if test is taking too long
                if (time.perf_counter() - test_start) > max_test_time:
                    self.logger.warning(f"   Test timeout at {max_test_time}s")
                    break
            
            test_end = time.perf_counter()
            total_test_time = (test_end - test_start) * 1000  # ms
            
            # Get memory tracking results
            current_memory, peak_memory = tracemalloc.get_traced_memory()
            tracemalloc.stop()
            
            # Calculate metrics
            avg_burst_time = statistics.mean(burst_times) if burst_times else 0
            max_burst_time = max(burst_times) if burst_times else 0
            min_burst_time = min(burst_times) if burst_times else 0
            
            avg_actual_freq = statistics.mean(actual_frequencies) if actual_frequencies else 0
            frequency_deviation = abs(avg_actual_freq - target_frequency) / target_frequency * 100 if target_frequency > 0 else 100
            
            # Calculate frequency stability (lower std dev = more stable)
            freq_std = statistics.stdev(actual_frequencies) if len(actual_frequencies) > 1 else 0
            frequency_stability = max(0, 100 - (freq_std / target_frequency * 100)) if target_frequency > 0 else 0
            
            # Resource usage
            avg_cpu = statistics.mean(cpu_samples) if cpu_samples else 0
            max_cpu = max(cpu_samples) if cpu_samples else 0
            peak_memory_mb = peak_memory / (1024**2)
            
            # Throughput calculations
            neurons_per_sec = total_neurons_fired / (total_test_time / 1000) if total_test_time > 0 else 0
            bursts_per_sec = burst_count / (total_test_time / 1000) if total_test_time > 0 else 0
            
            # Estimate synapses processed (rough calculation)
            avg_synapses_per_neuron = 10  # Based on 10% connectivity
            synapses_per_sec = neurons_per_sec * avg_synapses_per_neuron
            
            # Activity analysis
            propagation_cycles = burst_count // 3  # Rough estimate of A→B→C→A cycles
            activity_decay = (fired_neuron_counts[0] - fired_neuron_counts[-1]) / fired_neuron_counts[0] if fired_neuron_counts and fired_neuron_counts[0] > 0 else 0
            
            # Create metrics object
            metrics = NeuralPropagationMetrics(
                scenario_name=scenario_name,
                M=M,
                total_neurons=total_neurons,
                consecutive_fires=consecutive_fires,
                timestep_ms=timestep_ms,
                setup_time_ms=setup_time,
                avg_burst_time_ms=avg_burst_time,
                max_burst_time_ms=max_burst_time,
                min_burst_time_ms=min_burst_time,
                total_test_time_ms=total_test_time,
                neurons_processed_per_sec=neurons_per_sec,
                synapses_processed_per_sec=synapses_per_sec,
                bursts_per_sec=bursts_per_sec,
                peak_memory_mb=peak_memory_mb,
                avg_cpu_percent=avg_cpu,
                max_cpu_percent=max_cpu,
                total_bursts=burst_count,
                total_neurons_fired=total_neurons_fired,
                propagation_cycles=propagation_cycles,
                activity_decay_rate=activity_decay,
                target_frequency_hz=target_frequency,
                actual_frequency_hz=avg_actual_freq,
                frequency_deviation_percent=frequency_deviation,
                frequency_stability_score=frequency_stability,
                timestamp=time.time()
            )
            
            # Log results
            self.logger.info(f"✅ Test completed: {scenario_name}")
            self.logger.info(f"   Total time: {total_test_time:.1f}ms")
            self.logger.info(f"   Bursts: {burst_count}, Neurons fired: {total_neurons_fired:,}")
            self.logger.info(f"   Avg burst time: {avg_burst_time:.2f}ms")
            self.logger.info(f"   Throughput: {neurons_per_sec:,.0f} neurons/sec")
            self.logger.info(f"   Frequency: {avg_actual_freq:.1f}Hz (target: {target_frequency:.1f}Hz)")
            self.logger.info(f"   Memory: {peak_memory_mb:.1f}MB peak")
            
            return metrics
            
        except Exception as e:
            self.logger.error(f"❌ Test failed: {scenario_name}")
            self.logger.error(f"   Error: {str(e)}")
            self.logger.error(f"   Traceback: {traceback.format_exc()}")
            
            # Return minimal metrics for failed test
            return NeuralPropagationMetrics(
                scenario_name=f"{scenario_name}_FAILED",
                M=M,
                total_neurons=3 * M * M,
                consecutive_fires=consecutive_fires,
                timestep_ms=timestep_ms,
                setup_time_ms=0,
                avg_burst_time_ms=0,
                max_burst_time_ms=0,
                min_burst_time_ms=0,
                total_test_time_ms=0,
                neurons_processed_per_sec=0,
                synapses_processed_per_sec=0,
                bursts_per_sec=0,
                peak_memory_mb=0,
                avg_cpu_percent=0,
                max_cpu_percent=0,
                total_bursts=0,
                total_neurons_fired=0,
                propagation_cycles=0,
                activity_decay_rate=0,
                target_frequency_hz=1000.0 / timestep_ms,
                actual_frequency_hz=0,
                frequency_deviation_percent=100,
                frequency_stability_score=0,
                timestamp=time.time()
            )
        
        finally:
            # Cleanup
            try:
                del cm, be
                gc.collect()
            except:
                pass
    
    def run_comprehensive_benchmark(self) -> List[NeuralPropagationMetrics]:
        """Run the complete neural propagation benchmark suite."""
        
        print("🚀 FEAGI Neural Propagation Performance Benchmark")
        print("=" * 60)
        print("Testing realistic neural computation with synaptic propagation")
        print("Architecture: 3 cortical areas (A→B→C→A) with block_to_block connectivity")
        print()
        
        # Create test scenarios
        scenarios = self.create_test_scenarios()
        
        print(f"📊 Running {len(scenarios)} test scenarios...")
        print()
        
        all_metrics = []
        
        for i, scenario in enumerate(scenarios, 1):
            print(f"🧪 Test {i}/{len(scenarios)}: {scenario['name']}")
            print(f"   Parameters: M={scenario['M']}, fires={scenario['consecutive_fires']}, "
                  f"timestep={scenario['timestep_ms']}ms")
            
            try:
                metrics = self.run_neural_propagation_test(
                    M=scenario['M'],
                    consecutive_fires=scenario['consecutive_fires'],
                    timestep_ms=scenario['timestep_ms'],
                    scenario_name=scenario['name']
                )
                
                all_metrics.append(metrics)
                
                # Print quick summary
                if not metrics.scenario_name.endswith('_FAILED'):
                    print(f"   ✅ {metrics.avg_burst_time_ms:.2f}ms avg burst, "
                          f"{metrics.neurons_processed_per_sec:,.0f} neurons/sec, "
                          f"{metrics.frequency_deviation_percent:.1f}% freq dev")
                else:
                    print(f"   ❌ Test failed")
                
            except Exception as e:
                print(f"   ❌ Exception: {str(e)}")
                continue
            
            print()
            
            # Small delay between tests for system stability
            time.sleep(1.0)
        
        # Save results
        self.save_benchmark_results(all_metrics)
        
        # Print summary report
        self.print_benchmark_summary(all_metrics)
        
        return all_metrics
    
    def save_benchmark_results(self, metrics: List[NeuralPropagationMetrics]) -> str:
        """Save benchmark results to JSON file."""
        import json
        import os
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"neural_propagation_benchmark_{timestamp}.json"
        filepath = os.path.join(self.results_dir, filename)
        
        # Convert metrics to dictionaries
        results_data = {
            'benchmark_type': 'neural_propagation',
            'timestamp': time.time(),
            'total_scenarios': len(metrics),
            'successful_scenarios': len([m for m in metrics if not m.scenario_name.endswith('_FAILED')]),
            'failed_scenarios': len([m for m in metrics if m.scenario_name.endswith('_FAILED')]),
            'results': [m.to_dict() for m in metrics]
        }
        
        with open(filepath, 'w') as f:
            json.dump(results_data, f, indent=2)
        
        print(f"💾 Results saved to: {filepath}")
        return filepath
    
    def print_benchmark_summary(self, metrics: List[NeuralPropagationMetrics]) -> None:
        """Print comprehensive benchmark summary."""
        
        successful_metrics = [m for m in metrics if not m.scenario_name.endswith('_FAILED')]
        failed_count = len(metrics) - len(successful_metrics)
        
        if not successful_metrics:
            print("❌ No successful tests to analyze")
            return
        
        print("📊 NEURAL PROPAGATION BENCHMARK SUMMARY")
        print("=" * 60)
        print(f"Total scenarios: {len(metrics)}")
        print(f"Successful: {len(successful_metrics)}")
        print(f"Failed: {failed_count}")
        print()
        
        # Performance analysis
        print("🚀 PERFORMANCE ANALYSIS")
        print("-" * 30)
        
        # Best performers
        best_throughput = max(successful_metrics, key=lambda m: m.neurons_processed_per_sec)
        best_frequency = min(successful_metrics, key=lambda m: m.frequency_deviation_percent)
        fastest_burst = min(successful_metrics, key=lambda m: m.avg_burst_time_ms)
        
        print(f"🏆 Best throughput: {best_throughput.scenario_name}")
        print(f"   {best_throughput.neurons_processed_per_sec:,.0f} neurons/sec")
        print(f"   M={best_throughput.M}, timestep={best_throughput.timestep_ms}ms")
        print()
        
        print(f"🎯 Best frequency stability: {best_frequency.scenario_name}")
        print(f"   {best_frequency.frequency_deviation_percent:.2f}% deviation")
        print(f"   {best_frequency.actual_frequency_hz:.1f}Hz actual vs {best_frequency.target_frequency_hz:.1f}Hz target")
        print()
        
        print(f"⚡ Fastest burst processing: {fastest_burst.scenario_name}")
        print(f"   {fastest_burst.avg_burst_time_ms:.2f}ms average burst time")
        print(f"   M={fastest_burst.M}, {fastest_burst.total_neurons:,} total neurons")
        print()
        
        # Scaling analysis
        print("📈 SCALING ANALYSIS")
        print("-" * 20)
        
        # Group by parameter sweeps
        m_sweep_metrics = [m for m in successful_metrics if m.scenario_name.startswith('M_sweep_')]
        if m_sweep_metrics:
            m_sweep_metrics.sort(key=lambda m: m.M)
            print("M Dimension Scaling:")
            for m in m_sweep_metrics:
                print(f"   M={m.M:4d}: {m.avg_burst_time_ms:6.2f}ms, "
                      f"{m.neurons_processed_per_sec:8,.0f} neurons/sec")
            print()
        
        timestep_metrics = [m for m in successful_metrics if m.scenario_name.startswith('timestep_sweep_')]
        if timestep_metrics:
            timestep_metrics.sort(key=lambda m: m.timestep_ms, reverse=True)
            print("Timestep Scaling:")
            for m in timestep_metrics:
                print(f"   {m.timestep_ms:5.1f}ms: {m.frequency_deviation_percent:5.1f}% freq dev, "
                      f"{m.avg_burst_time_ms:6.2f}ms burst")
            print()
        
        # Extreme scenario results
        extreme_metrics = [m for m in successful_metrics if 'extreme' in m.scenario_name or 'ultra' in m.scenario_name]
        if extreme_metrics:
            print("🔥 EXTREME SCENARIO RESULTS")
            print("-" * 30)
            for m in extreme_metrics:
                status = "✅" if m.frequency_deviation_percent < 25 else "⚠️" if m.frequency_deviation_percent < 50 else "❌"
                print(f"{status} {m.scenario_name}")
                print(f"   {m.total_neurons:,} neurons, {m.avg_burst_time_ms:.2f}ms burst, "
                      f"{m.frequency_deviation_percent:.1f}% freq dev")
            print()
        
        # Resource usage summary
        print("💾 RESOURCE USAGE SUMMARY")
        print("-" * 25)
        max_memory = max(successful_metrics, key=lambda m: m.peak_memory_mb)
        max_cpu = max(successful_metrics, key=lambda m: m.max_cpu_percent)
        
        print(f"Peak memory usage: {max_memory.peak_memory_mb:.1f}MB ({max_memory.scenario_name})")
        print(f"Peak CPU usage: {max_cpu.max_cpu_percent:.1f}% ({max_cpu.scenario_name})")
        print()
        
        # Recommendations
        print("💡 RECOMMENDATIONS")
        print("-" * 18)
        
        # Find performance sweet spots
        good_performers = [m for m in successful_metrics 
                          if m.frequency_deviation_percent < 10 and m.avg_burst_time_ms < 50]
        
        if good_performers:
            best_overall = min(good_performers, key=lambda m: m.avg_burst_time_ms)
            print(f"🎯 Recommended configuration: {best_overall.scenario_name}")
            print(f"   M={best_overall.M}, timestep={best_overall.timestep_ms}ms")
            print(f"   Performance: {best_overall.avg_burst_time_ms:.2f}ms burst, "
                  f"{best_overall.neurons_processed_per_sec:,.0f} neurons/sec")
        
        # Identify bottlenecks
        slow_tests = [m for m in successful_metrics if m.avg_burst_time_ms > 100]
        if slow_tests:
            print(f"⚠️  Performance bottlenecks detected in {len(slow_tests)} scenarios")
            print("   Consider optimizing for large M values or high frequencies")
        
        failed_tests = [m for m in metrics if m.scenario_name.endswith('_FAILED')]
        if failed_tests:
            print(f"❌ {len(failed_tests)} scenarios failed - check system limits")
        
        print()
        print("🎉 Neural propagation benchmark complete!")


if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Run benchmark
    benchmark = NeuralPropagationBenchmark()
    results = benchmark.run_comprehensive_benchmark()
    
    print(f"\n✅ Benchmark completed with {len(results)} scenarios tested")
