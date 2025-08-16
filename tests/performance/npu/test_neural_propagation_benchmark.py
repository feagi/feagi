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
        """Convert metrics to dictionary for JSON serialization with detailed parameters."""
        return {
            # Test Identification
            'scenario_name': self.scenario_name,
            'backend_name': self.backend_name,
            'timestamp': self.timestamp,
            
            # Test Parameters (duration-based approach)
            'cortical_dimensions': f"{self.M}×{self.M}×1",
            'M': self.M,
            'total_neurons': self.total_neurons,
            'test_duration_sec': self.consecutive_fires * self.timestep_ms / 1000.0,
            'calculated_burst_count': self.consecutive_fires,
            'simulation_timestep_sec': self.timestep_ms / 1000.0,
            'simulation_timestep_ms': self.timestep_ms,  # Keep for compatibility
            'target_frequency_hz': self.target_frequency_hz,
            
            # Brain Development Phase Metrics
            'brain_development': {
                'time_ms': self.brain_dev_time_ms,
                'memory_mb': self.brain_dev_memory_mb,
                'cpu_percent': self.brain_dev_cpu_percent,
                'backend': self.backend_name,
                'operations': [
                    'cortical_area_creation',
                    'neuron_creation_sequential',
                    'synaptic_connection_setup'
                ]
            },
            
            # Neural Computation Phase Metrics
            'neural_computation': {
                'time_ms': self.neural_comp_time_ms,
                'memory_mb': self.neural_comp_memory_mb,
                'cpu_percent': self.neural_comp_cpu_percent,
                'backend': self.backend_name,
                'avg_burst_time_ms': self.avg_burst_time_ms,
                'max_burst_time_ms': self.max_burst_time_ms,
                'min_burst_time_ms': self.min_burst_time_ms,
                'total_bursts': self.total_bursts,
                'operations': [
                    'membrane_potential_updates',
                    'synaptic_propagation',
                    'neural_firing_detection'
                ]
            },
            
            # Overall Performance Metrics
            'performance': {
                'total_test_time_ms': self.total_test_time_ms,
                'setup_time_ms': self.setup_time_ms,
                'neurons_processed_per_sec': self.neurons_processed_per_sec,
                'synapses_processed_per_sec': self.synapses_processed_per_sec,
                'bursts_per_sec': self.bursts_per_sec,
                'actual_frequency_hz': self.actual_frequency_hz,
                'frequency_deviation_percent': self.frequency_deviation_percent,
                'frequency_stability_score': self.frequency_stability_score
            },
            
            # Resource Usage
            'resources': {
                'peak_memory_mb': self.peak_memory_mb,
                'avg_cpu_percent': self.avg_cpu_percent,
                'max_cpu_percent': self.max_cpu_percent
            },
            
            # Neural Activity Analysis
            'neural_activity': {
                'total_neurons_fired': self.total_neurons_fired,
                'propagation_cycles': self.propagation_cycles,
                'activity_decay_rate': self.activity_decay_rate
            },
            
            # Legacy flat structure for backward compatibility
            'brain_dev_time_ms': self.brain_dev_time_ms,
            'brain_dev_memory_mb': self.brain_dev_memory_mb,
            'brain_dev_cpu_percent': self.brain_dev_cpu_percent,
            'neural_comp_time_ms': self.neural_comp_time_ms,
            'neural_comp_memory_mb': self.neural_comp_memory_mb,
            'neural_comp_cpu_percent': self.neural_comp_cpu_percent,
            'consecutive_fires': self.consecutive_fires,
            'timestep_ms': self.timestep_ms
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
        
        # Create neurons using the SAME PIPELINE as real genome development
        print(f"   ⚡ Creating {3*M*M:,} neurons...")
        total_neurons_created = 0
        
        # Use the same neurogenesis approach as NeuroEmbryogenesis._perform_neurogenesis()
        neuron_array = cm.neuron_array
        
        for area_id, area_name in [(area_a_id, "A"), (area_b_id, "B"), (area_c_id, "C")]:
            area = cm.cortical_areas[area_id]
            area_neuron_count = M * M * 1  # M x M x 1 dimensions
            
            # FAST: Reserve array indices in bulk (same as genome development)
            start_idx = neuron_array.next_index
            end_idx = start_idx + area_neuron_count
            
            if end_idx > neuron_array.max_neurons:
                raise ValueError(f"Not enough capacity for {area_neuron_count} neurons")
            
            # FAST: Generate neuron IDs in bulk (same as genome development)
            neuron_ids = list(range(
                neuron_array._next_neuron_id,
                neuron_array._next_neuron_id + area_neuron_count
            ))
            neuron_array._next_neuron_id += area_neuron_count
            
            # FAST: Update mappings in bulk (same as genome development)
            for j, neuron_id in enumerate(neuron_ids):
                cm.set_neuron_mapping(neuron_id, start_idx + j)
            
            # FAST: Set properties with vectorized array slicing (same as genome development)
            neuron_array.valid_mask[start_idx:end_idx] = True
            neuron_array.membrane_potentials[start_idx:end_idx] = 0.0
            neuron_array.resting_potentials[start_idx:end_idx] = 0.0
            neuron_array.thresholds[start_idx:end_idx] = 1.0
            neuron_array.decay_rates[start_idx:end_idx] = 1.0  # NO DECAY - sustain activity
            neuron_array.refractory_periods[start_idx:end_idx] = 1
            neuron_array.refractory_counters[start_idx:end_idx] = 0
            neuron_array.cortical_idxs[start_idx:end_idx] = area.cortical_idx
            neuron_array.is_active[start_idx:end_idx] = True
            
            # FAST: Generate coordinates and set with vectorized operations (same as genome development)
            positions = []
            for x in range(M):
                for y in range(M):
                    positions.append((x, y, 0))
            
            coords_x = np.array([pos[0] for pos in positions], dtype=np.uint32)
            coords_y = np.array([pos[1] for pos in positions], dtype=np.uint32)
            coords_z = np.array([pos[2] for pos in positions], dtype=np.uint32)
            
            neuron_array.coordinates_x[start_idx:end_idx] = coords_x
            neuron_array.coordinates_y[start_idx:end_idx] = coords_y
            neuron_array.coordinates_z[start_idx:end_idx] = coords_z
            
            # Update neuron count and next_index
            neuron_array.neuron_count += area_neuron_count
            neuron_array.next_index = end_idx
            
            # Add neurons to cortical area (same as genome development)
            for i, neuron_id in enumerate(neuron_ids):
                area.add_neuron(neuron_id, positions[i])
            
            total_neurons_created += area_neuron_count
            print(f"   ✅ Area {area_name}: {area_neuron_count:,} neurons created")
        
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
        
        # CRITICAL: Actually create synapses between cortical areas
        # The add_cortical_connection() calls above only store metadata, not actual synapses!
        print(f"   🔗 Creating actual synapses between areas...")
        
        # Get neurons from each area
        area_a_neurons = list(cm.cortical_areas[area_a_id].neurons)
        area_b_neurons = list(cm.cortical_areas[area_b_id].neurons)
        area_c_neurons = list(cm.cortical_areas[area_c_id].neurons)
        
        # Create synapses: A → B (block_to_block: each neuron in A connects to corresponding neuron in B)
        synapse_specs = []
        weight = 5.0  # Strong enough to ensure downstream neurons fire
        
        print(f"   🔗 Creating block_to_block connectivity as specified...")
        
        # A → B connections (block_to_block: 1:1 mapping)
        for i, pre_neuron in enumerate(area_a_neurons):
            if i < len(area_b_neurons):  # Ensure we don't exceed target area size
                post_neuron = area_b_neurons[i]
                synapse_specs.append((pre_neuron, post_neuron, weight))
        
        # B → C connections (block_to_block: 1:1 mapping)
        for i, pre_neuron in enumerate(area_b_neurons):
            if i < len(area_c_neurons):
                post_neuron = area_c_neurons[i]
                synapse_specs.append((pre_neuron, post_neuron, weight))
        
        # C → A connections (block_to_block: 1:1 mapping, completing the loop)
        for i, pre_neuron in enumerate(area_c_neurons):
            if i < len(area_a_neurons):
                post_neuron = area_a_neurons[i]
                synapse_specs.append((pre_neuron, post_neuron, weight))
        
        # Batch create all synapses
        synapses_created = cm.batch_create_synapses(synapse_specs)
        print(f"   ✅ Created {synapses_created:,} synapses")
        
        # DEBUG: Verify synapses were actually created
        synapse_stats = cm.synapse_array.get_statistics()
        print(f"   📊 Total synapses in array: {synapse_stats['synapse_count']:,}")
        if synapse_stats['synapse_count'] == 0:
            print(f"   ⚠️ WARNING: No synapses created! Neural propagation will not work.")
        else:
            print(f"   🎯 Synaptic connectivity established: A→B→C→A loop ready")
        
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
            
            # Get area IDs for stimulation - use direct access to cortical_areas
            all_cortical_areas = cm.cortical_areas
            area_ids = [area_id for area_id in all_cortical_areas.keys() if area_id.startswith('area_')]
            
            if len(area_ids) < 3:
                # Try alternative approach - get all area IDs
                all_area_ids = list(all_cortical_areas.keys())
                print(f"   🔍 Debug: Found cortical areas: {all_area_ids}")
                # Use the last 3 areas created (should be our test areas)
                area_ids = all_area_ids[-3:] if len(all_area_ids) >= 3 else all_area_ids
                
            if len(area_ids) < 3:
                raise ValueError(f"Expected 3 cortical areas, found {len(area_ids)}: {area_ids}")
            
            # Sort to ensure consistent A, B, C order
            area_ids.sort()
            print(f"   🎯 Using cortical areas: {area_ids}")
            
            # Get neurons from Area A for initial stimulation
            area_a_info = cm.get_cortical_area(area_ids[0])
            if area_a_info and hasattr(area_a_info, 'neurons') and area_a_info.neurons:
                area_a_neurons = list(area_a_info.neurons)
            else:
                area_a_neurons = list(range(M * M))
            
            # Fire ALL neurons in Area A for maximum GPU workload
            stimulation_count = len(area_a_neurons)  # Fire ALL neurons in Area A
            # area_a_neurons already contains all neurons - no need to slice
            
            print(f"   🔥 Initial stimulation: firing {len(area_a_neurons):,} neurons in Area A")
            
            # Inject initial stimulus
            for neuron_id in area_a_neurons:
                try:
                    cm.neuron_array.set_neuron_property(neuron_id, 'membrane_potential', 2.0)
                except Exception as e:
                    pass  # Continue with other neurons
            
            # Run the specified number of bursts for neural computation
            max_bursts = consecutive_fires  # Use full consecutive_fires parameter
            print(f"   🔄 Running {max_bursts} neural computation bursts...")
            
            burst_times = []
            total_neurons_fired = 0
            
            # Progress reporting intervals
            progress_interval = max(1, max_bursts // 10)  # Report every 10%
            
            for burst_idx in range(max_bursts):
                burst_start = time.perf_counter()
                
                # Run membrane potential update (this is the core neural computation)
                cm.update_membrane_potentials()
                
                burst_end = time.perf_counter()
                burst_time_ms = (burst_end - burst_start) * 1000
                burst_times.append(burst_time_ms)
                
                # DEBUG: Check neural activity and count fired neurons
                active_neurons = np.sum(cm.active_neurons)
                
                # Count neurons that actually fired (have membrane potential >= threshold)
                fired_neurons_count = 0
                if hasattr(cm, 'neuron_array') and cm.neuron_array.neuron_count > 0:
                    potentials = cm.neuron_array.membrane_potentials[:cm.neuron_array.neuron_count]
                    thresholds = cm.neuron_array.thresholds[:cm.neuron_array.neuron_count]
                    fired_neurons_count = np.sum(potentials >= thresholds)
                
                if burst_idx % 50 == 0 or burst_idx < 5:
                    # Check membrane potentials
                    max_potential = np.max(cm.neuron_array.membrane_potentials[:cm.neuron_array.neuron_count])
                    avg_potential = np.mean(cm.neuron_array.membrane_potentials[:cm.neuron_array.neuron_count])
                    threshold_sample = cm.neuron_array.thresholds[0] if cm.neuron_array.neuron_count > 0 else 0
                    print(f"   🧠 Burst {burst_idx+1}: {active_neurons} active neurons, {fired_neurons_count:,} fired neurons, max_potential={max_potential:.3f}, avg_potential={avg_potential:.3f}, threshold={threshold_sample:.3f}")
                else:
                    print(f"   🧠 Burst {burst_idx+1}: {active_neurons} active neurons, {fired_neurons_count:,} fired neurons")
                
                # Progress reporting (only every 10% or at key milestones)
                if (burst_idx + 1) % progress_interval == 0 or burst_idx == 0 or burst_idx == max_bursts - 1:
                    avg_so_far = statistics.mean(burst_times) if burst_times else 0
                    progress_pct = ((burst_idx + 1) / max_bursts) * 100
                    print(f"   📊 Progress: {progress_pct:.0f}% ({burst_idx+1}/{max_bursts}) - Avg: {avg_so_far:.2f}ms/burst")
                
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
        
        # Initialize default values
        brain_dev_speedup = 1.0
        neural_comp_speedup = 1.0
        total_speedup = 1.0
        
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
