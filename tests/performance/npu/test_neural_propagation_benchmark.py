#!/usr/bin/env python3
"""
Comprehensive Neural Propagation Performance Benchmark Suite

This unified benchmark suite provides multiple testing modes for FEAGI performance analysis:

1. CPU vs GPU Performance Comparison:
   - Tests realistic neural computation with synaptic propagation across cortical areas
   - Creates a 3-area neural network (A→B→C→A) with block_to_block connectivity
   - Compares PyTorch (CPU) vs WGPU (GPU) backends
   - Large-scale testing with M=2000 (12M neurons)

2. Frequency Robustness Testing:
   - Tests burst frequency stability under increasing neuron loads
   - Progressive neuron scaling (100 to 1M neurons)
   - Measures frequency deviation, CPU usage, and memory consumption
   - Identifies breaking points where frequency degrades

3. Multi-Frequency Testing:
   - Tests robustness across multiple target frequencies (10-100Hz)
   - Evaluates system performance at different burst rates
   - Identifies optimal frequency ranges for different scales

Test Architecture:
- 3 Cortical Areas: A, B, C (each M×M×1 dimensions)
- Connectivity: A→B→C→A (circular propagation using block_to_block morphology)
- Initial Stimulus: Fire all neurons in Area A once
- Propagation: Automatic synaptic propagation through the network
- Duration Control: Consecutive fire count property controls test length

Usage:
- python test_neural_propagation_benchmark.py                    # CPU vs GPU (default)
- python test_neural_propagation_benchmark.py cpu-gpu           # CPU vs GPU only
- python test_neural_propagation_benchmark.py frequency         # Frequency robustness
- python test_neural_propagation_benchmark.py multi-frequency   # Multi-frequency test
- python test_neural_propagation_benchmark.py comprehensive     # Full test suite

Performance Metrics:
- Execution time per burst
- Neural propagation throughput
- Memory usage and CPU utilization
- Frequency stability under load
- Synaptic processing efficiency
- Frequency deviation percentages
- Resource usage analysis
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
from feagi.npu.interfaces import FiredNeuronEvent


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


@dataclass
class FrequencyRobustnessMetrics:
    """Metrics for burst frequency robustness testing."""
    neuron_count: int
    configured_frequency_hz: float
    actual_frequency_hz: float
    frequency_deviation_percent: float
    frequency_stability_coefficient: float  # Lower = more stable
    burst_time_avg_ms: float
    burst_time_p95_ms: float
    burst_time_p99_ms: float
    cpu_usage_avg_percent: float
    cpu_usage_peak_percent: float
    memory_usage_avg_mb: float
    memory_usage_peak_mb: float
    memory_growth_rate_mb_per_sec: float
    overrun_count: int
    total_bursts_measured: int
    measurement_duration_sec: float
    timestamp: float
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            'neuron_count': self.neuron_count,
            'configured_frequency_hz': self.configured_frequency_hz,
            'actual_frequency_hz': self.actual_frequency_hz,
            'frequency_deviation_percent': self.frequency_deviation_percent,
            'frequency_stability_coefficient': self.frequency_stability_coefficient,
            'burst_time_avg_ms': self.burst_time_avg_ms,
            'burst_time_p95_ms': self.burst_time_p95_ms,
            'burst_time_p99_ms': self.burst_time_p99_ms,
            'cpu_usage_avg_percent': self.cpu_usage_avg_percent,
            'cpu_usage_peak_percent': self.cpu_usage_peak_percent,
            'memory_usage_avg_mb': self.memory_usage_avg_mb,
            'memory_usage_peak_mb': self.memory_usage_peak_mb,
            'memory_growth_rate_mb_per_sec': self.memory_growth_rate_mb_per_sec,
            'overrun_count': self.overrun_count,
            'total_bursts_measured': self.total_bursts_measured,
            'measurement_duration_sec': self.measurement_duration_sec,
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
                activity_leak_coefficient=0.0,  # Not measured in this simplified test
                
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
                activity_leak_coefficient=0.0,
                
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
    
    def _create_firing_pattern(self, neuron_count: int, firing_rate: float = 0.1) -> Dict[int, List[int]]:
        """Create realistic firing pattern for frequency testing."""
        # Distribute neurons across cortical areas
        cortical_areas = min(max(1, neuron_count // 1000), 50)  # 1-50 areas
        neurons_per_area = neuron_count // cortical_areas
        
        neurons_by_cortical = {}
        
        for area_id in range(cortical_areas):
            start_neuron = area_id * neurons_per_area
            end_neuron = min((area_id + 1) * neurons_per_area, neuron_count)
            
            area_neurons = list(range(start_neuron, end_neuron))
            firing_count = max(1, int(len(area_neurons) * firing_rate))
            
            # Create reproducible but varied firing patterns
            np.random.seed(area_id + neuron_count)
            firing_neurons = np.random.choice(area_neurons, firing_count, replace=False)
            
            neurons_by_cortical[area_id] = firing_neurons.tolist()
        
        return neurons_by_cortical
    
    def _monitor_system_resources(self, duration: float) -> Dict[str, List[float]]:
        """Monitor CPU and memory usage during test execution."""
        cpu_samples = []
        memory_samples = []
        start_time = time.time()
        
        def monitor():
            process = psutil.Process()
            while time.time() - start_time < duration:
                try:
                    cpu_percent = process.cpu_percent()
                    memory_mb = process.memory_info().rss / (1024 * 1024)
                    
                    cpu_samples.append(cpu_percent)
                    memory_samples.append(memory_mb)
                    
                    time.sleep(0.1)  # 100ms sampling
                except Exception as e:
                    self.logger.warning(f"Resource monitoring error: {e}")
                    break
        
        import threading
        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
        
        return {'cpu': cpu_samples, 'memory': memory_samples}
    
    def _measure_frequency_robustness(self, neuron_count: int, 
                                    target_frequency: float) -> FrequencyRobustnessMetrics:
        """Measure frequency robustness for specific neuron count and frequency."""
        self.logger.info(f"Testing {neuron_count:,} neurons at {target_frequency}Hz")
        
        # Create test system
        ConnectomeManager.reset_singleton()
        cm = ConnectomeManager(config_or_max_neurons=neuron_count)
        be = BurstEngine(cm)
        
        # Start async processor if available
        async_processor = cm._get_async_fcl_processor()
        if async_processor:
            async_processor.start()
            time.sleep(0.1)  # Let it initialize
        
        try:
            # Create firing pattern
            neurons_by_cortical = self._create_firing_pattern(neuron_count)
            
            # Test parameters
            measurement_duration = 10.0  # seconds per test
            min_bursts_required = 50    # minimum bursts to measure
            
            # Start resource monitoring
            resource_data = self._monitor_system_resources(measurement_duration)
            
            # Start memory tracking
            tracemalloc.start()
            process = psutil.Process()
            memory_start = process.memory_info().rss / (1024 * 1024)
            
            # Measure burst timing
            burst_times = []
            actual_frequencies = []
            overrun_count = 0
            
            target_burst_interval = 1.0 / target_frequency
            measurement_start = time.perf_counter()
            last_burst_time = measurement_start
            
            # Run bursts for measurement duration
            while (time.perf_counter() - measurement_start < measurement_duration and 
                   len(burst_times) < min_bursts_required * 10):
                
                # Wait for next burst time
                next_burst_time = last_burst_time + target_burst_interval
                current_time = time.perf_counter()
                
                if current_time < next_burst_time:
                    time.sleep(next_burst_time - current_time)
                
                # Execute burst
                burst_start = time.perf_counter()
                
                # Process FCL update
                if async_processor:
                    event = FiredNeuronEvent(
                        timestep=len(burst_times),
                        neurons_by_cortical=neurons_by_cortical
                    )
                    async_processor.process_fired_neurons(event)
                else:
                    fcl_manager = cm._get_fcl_manager()
                    if fcl_manager:
                        fcl_manager.update_fcl(len(burst_times), neurons_by_cortical)
                
                burst_end = time.perf_counter()
                burst_duration = burst_end - burst_start
                burst_times.append(burst_duration * 1000)  # Convert to ms
                
                # Calculate actual frequency
                if len(burst_times) > 1:
                    actual_interval = burst_end - last_burst_time
                    actual_freq = 1.0 / actual_interval if actual_interval > 0 else 0
                    actual_frequencies.append(actual_freq)
                
                # Check for overruns
                if burst_duration > target_burst_interval:
                    overrun_count += 1
                
                last_burst_time = burst_end
            
            # Stop memory tracking
            memory_current, memory_peak = tracemalloc.get_traced_memory()
            tracemalloc.stop()
            memory_end = process.memory_info().rss / (1024 * 1024)
            
            # Wait for resource monitoring to complete
            time.sleep(0.2)
            
            # Calculate metrics
            measurement_duration_actual = time.perf_counter() - measurement_start
            avg_actual_frequency = statistics.mean(actual_frequencies) if actual_frequencies else 0
            frequency_deviation = abs(avg_actual_frequency - target_frequency) / target_frequency * 100
            
            # Calculate frequency stability (coefficient of variation)
            freq_std = statistics.stdev(actual_frequencies) if len(actual_frequencies) > 1 else 0
            freq_stability = freq_std / avg_actual_frequency if avg_actual_frequency > 0 else float('inf')
            
            # Resource usage statistics
            cpu_samples = resource_data['cpu']
            memory_samples = resource_data['memory']
            
            cpu_avg = statistics.mean(cpu_samples) if cpu_samples else 0
            cpu_peak = max(cpu_samples) if cpu_samples else 0
            memory_avg = statistics.mean(memory_samples) if memory_samples else 0
            memory_peak_mb = max(memory_samples) if memory_samples else 0
            
            # Memory growth rate
            memory_growth_rate = (memory_end - memory_start) / measurement_duration_actual if measurement_duration_actual > 0 else 0
            
            # Latency percentiles
            p95_latency = np.percentile(burst_times, 95) if burst_times else 0
            p99_latency = np.percentile(burst_times, 99) if burst_times else 0
            
            return FrequencyRobustnessMetrics(
                neuron_count=neuron_count,
                configured_frequency_hz=target_frequency,
                actual_frequency_hz=avg_actual_frequency,
                frequency_deviation_percent=frequency_deviation,
                frequency_stability_coefficient=freq_stability,
                burst_time_avg_ms=statistics.mean(burst_times) if burst_times else 0,
                burst_time_p95_ms=p95_latency,
                burst_time_p99_ms=p99_latency,
                cpu_usage_avg_percent=cpu_avg,
                cpu_usage_peak_percent=cpu_peak,
                memory_usage_avg_mb=memory_avg,
                memory_usage_peak_mb=memory_peak_mb,
                memory_growth_rate_mb_per_sec=memory_growth_rate,
                overrun_count=overrun_count,
                total_bursts_measured=len(burst_times),
                measurement_duration_sec=measurement_duration_actual,
                timestamp=time.time()
            )
            
        finally:
            if async_processor:
                async_processor.stop()
            
            # Clean up
            del cm, be
            gc.collect()
    
    def run_frequency_robustness_test(self, target_frequency: float = 15.0) -> List[FrequencyRobustnessMetrics]:
        """Test frequency robustness across different neuron counts."""
        self.logger.info(f"Testing frequency robustness at {target_frequency}Hz across neuron scales")
        
        # Progressive neuron count scaling
        neuron_count_progression = [
            100, 500, 1000, 2500, 5000, 7500, 10000,
            15000, 20000, 30000, 50000, 75000, 100000,
            150000, 200000, 300000, 500000, 750000, 1000000
        ]
        
        # Performance thresholds
        frequency_deviation_thresholds = {
            'acceptable': 5.0,    # 5% deviation acceptable
            'concerning': 15.0,   # 15% deviation concerning
            'critical': 30.0      # 30% deviation critical
        }
        
        metrics = []
        
        print(f"\n🎯 FREQUENCY ROBUSTNESS TEST - {target_frequency}Hz TARGET")
        print("=" * 70)
        
        for neuron_count in neuron_count_progression:
            try:
                metric = self._measure_frequency_robustness(neuron_count, target_frequency)
                metrics.append(metric)
                
                # Log progress
                deviation = metric.frequency_deviation_percent
                status = "✅" if deviation < 5 else "⚠️" if deviation < 15 else "❌"
                
                print(
                    f"{status} {neuron_count:,} neurons: "
                    f"{metric.actual_frequency_hz:.1f}Hz actual "
                    f"({deviation:.1f}% deviation), "
                    f"CPU: {metric.cpu_usage_avg_percent:.1f}%, "
                    f"Memory: {metric.memory_usage_avg_mb:.1f}MB"
                )
                
                # Early termination if frequency drops too much
                if deviation > 50:  # 50% deviation is critical failure
                    self.logger.warning(f"Critical frequency degradation at {neuron_count:,} neurons")
                    break
                    
            except Exception as e:
                self.logger.error(f"Failed to test {neuron_count:,} neurons: {e}")
                continue
        
        # Print summary
        self._print_frequency_robustness_summary(metrics, target_frequency)
        
        return metrics
    
    def run_multi_frequency_robustness_test(self) -> Dict[float, List[FrequencyRobustnessMetrics]]:
        """Test robustness across multiple target frequencies."""
        self.logger.info("Testing frequency robustness across multiple target frequencies")
        
        test_frequencies = [10.0, 15.0, 20.0, 30.0, 50.0, 100.0]  # Hz
        results = {}
        
        print(f"\n🎯 MULTI-FREQUENCY ROBUSTNESS TEST")
        print("=" * 70)
        
        for target_freq in test_frequencies:
            print(f"\n🔄 Testing target frequency: {target_freq}Hz")
            
            # Test subset of neuron counts for each frequency
            test_counts = [1000, 5000, 10000, 25000, 50000, 100000]
            metrics = []
            
            for neuron_count in test_counts:
                try:
                    metric = self._measure_frequency_robustness(neuron_count, target_freq)
                    metrics.append(metric)
                    
                    deviation = metric.frequency_deviation_percent
                    status = "✅" if deviation < 10 else "⚠️" if deviation < 20 else "❌"
                    
                    print(f"  {status} {neuron_count:,} neurons: {metric.actual_frequency_hz:.1f}Hz ({deviation:.1f}% dev)")
                    
                    # Stop if frequency degradation is too severe
                    if metric.frequency_deviation_percent > 40:
                        break
                        
                except Exception as e:
                    self.logger.error(f"Failed {target_freq}Hz test at {neuron_count:,} neurons: {e}")
                    continue
            
            results[target_freq] = metrics
        
        return results
    
    def _print_frequency_robustness_summary(self, metrics: List[FrequencyRobustnessMetrics], target_frequency: float):
        """Print frequency robustness summary."""
        if not metrics:
            print("❌ No frequency robustness metrics collected")
            return
        
        # Summary statistics
        successful_tests = [m for m in metrics if m.frequency_deviation_percent < 15]
        max_stable_neurons = max([m.neuron_count for m in successful_tests]) if successful_tests else 0
        
        print(f"\n📊 FREQUENCY ROBUSTNESS SUMMARY:")
        print(f"   Target Frequency: {target_frequency} Hz")
        print(f"   Tests Completed: {len(metrics)}")
        print(f"   Maximum Stable Neuron Count: {max_stable_neurons:,}")
        
        # Resource usage analysis
        if metrics:
            avg_cpu = statistics.mean([m.cpu_usage_avg_percent for m in metrics])
            max_cpu = max([m.cpu_usage_peak_percent for m in metrics])
            avg_memory = statistics.mean([m.memory_usage_avg_mb for m in metrics])
            max_memory = max([m.memory_usage_peak_mb for m in metrics])
            
            print(f"\n💻 RESOURCE USAGE ANALYSIS:")
            print(f"   Average CPU Usage: {avg_cpu:.1f}%")
            print(f"   Peak CPU Usage: {max_cpu:.1f}%")
            print(f"   Average Memory Usage: {avg_memory:.1f} MB")
            print(f"   Peak Memory Usage: {max_memory:.1f} MB")
        
        print("=" * 70)
    
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


def run_comprehensive_benchmark_suite():
    """Run the complete benchmark suite with multiple test modes."""
    # Configure logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    benchmark = NeuralPropagationBenchmark()
    
    print("🚀 FEAGI COMPREHENSIVE PERFORMANCE BENCHMARK SUITE")
    print("=" * 60)
    
    # 1. CPU vs GPU Performance Comparison
    print("\n1️⃣ CPU vs GPU Performance Comparison (M=2000)")
    cpu_gpu_results = benchmark.run_cpu_vs_gpu_comparison(M=2000, consecutive_fires=3, timestep_ms=100.0)
    
    # 2. Frequency Robustness Test
    print("\n2️⃣ Frequency Robustness Test (15Hz)")
    frequency_results = benchmark.run_frequency_robustness_test(target_frequency=15.0)
    
    # 3. Multi-Frequency Robustness Test
    print("\n3️⃣ Multi-Frequency Robustness Test")
    multi_freq_results = benchmark.run_multi_frequency_robustness_test()
    
    # Save all results
    import json
    import os
    timestamp = int(time.time())
    
    # Save CPU vs GPU results
    cpu_gpu_file = os.path.join(benchmark.results_dir, f"cpu_vs_gpu_benchmark_{timestamp}.json")
    with open(cpu_gpu_file, 'w') as f:
        json.dump({
            'benchmark_type': 'cpu_vs_gpu_comparison',
            'timestamp': timestamp,
            'results': [result.to_dict() for result in cpu_gpu_results]
        }, f, indent=2, default=str)
    
    # Save frequency robustness results
    freq_file = os.path.join(benchmark.results_dir, f"frequency_robustness_{timestamp}.json")
    with open(freq_file, 'w') as f:
        json.dump({
            'benchmark_type': 'frequency_robustness',
            'timestamp': timestamp,
            'target_frequency': 15.0,
            'results': [result.to_dict() for result in frequency_results]
        }, f, indent=2, default=str)
    
    # Save multi-frequency results
    multi_freq_file = os.path.join(benchmark.results_dir, f"multi_frequency_robustness_{timestamp}.json")
    with open(multi_freq_file, 'w') as f:
        json.dump({
            'benchmark_type': 'multi_frequency_robustness',
            'timestamp': timestamp,
            'results': {str(freq): [result.to_dict() for result in results] 
                       for freq, results in multi_freq_results.items()}
        }, f, indent=2, default=str)
    
    print(f"\n✅ Comprehensive benchmark suite completed!")
    print(f"📊 Results saved to:")
    print(f"   - CPU vs GPU: {cpu_gpu_file}")
    print(f"   - Frequency Robustness: {freq_file}")
    print(f"   - Multi-Frequency: {multi_freq_file}")
    
    return {
        'cpu_vs_gpu': cpu_gpu_results,
        'frequency_robustness': frequency_results,
        'multi_frequency': multi_freq_results
    }


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()
        
        # Configure logging
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        benchmark = NeuralPropagationBenchmark()
        
        if mode == "cpu-gpu":
            # CPU vs GPU comparison only
            print("Running CPU vs GPU comparison with M=2000...")
            results = benchmark.run_cpu_vs_gpu_comparison(M=2000, consecutive_fires=3, timestep_ms=100.0)
            
        elif mode == "frequency":
            # Frequency robustness test only
            print("Running frequency robustness test at 15Hz...")
            results = benchmark.run_frequency_robustness_test(target_frequency=15.0)
            
        elif mode == "multi-frequency":
            # Multi-frequency robustness test only
            print("Running multi-frequency robustness test...")
            results = benchmark.run_multi_frequency_robustness_test()
            
        elif mode == "comprehensive":
            # Full comprehensive suite
            results = run_comprehensive_benchmark_suite()
            
        else:
            print("Usage: python test_neural_propagation_benchmark.py [cpu-gpu|frequency|multi-frequency|comprehensive]")
            print("Default: cpu-gpu mode")
            results = benchmark.run_cpu_vs_gpu_comparison(M=2000, consecutive_fires=3, timestep_ms=100.0)
    else:
        # Default: CPU vs GPU comparison (maintains backward compatibility)
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        benchmark = NeuralPropagationBenchmark()
        
        print("Running CPU vs GPU comparison with M=2000...")
        print("💡 Tip: Use 'python test_neural_propagation_benchmark.py comprehensive' for full test suite")
        results = benchmark.run_cpu_vs_gpu_comparison(M=2000, consecutive_fires=3, timestep_ms=100.0)
    
    print("\nBenchmark complete!")
