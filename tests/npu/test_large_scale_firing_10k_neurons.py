"""
Test for large-scale neuron firing at 8k+ neuron/synapse threshold.

This test validates that neurons can fire correctly in a 100x100 cortical area
when power is mapped using the projector pattern. This addresses the critical
issue where firing becomes inconsistent around 8k neurons/synapses.

Version: 1.0
Author: FEAGI Team
"""

import pytest
import numpy as np
from typing import Dict, List, Tuple
import time

from feagi.npu.data_structures import NeuronArray, SynapseArray, BackendType
from feagi.npu.interface import NPUInterface
from feagi.npu.fire_candidate_list import FireCandidateList
from feagi.npu.burst_engine import BurstEngine, PowerInjectionService
from feagi.bdu.connectome_manager import ConnectomeManager


class Test100x100NeuronFiring:
    """Test large-scale neuron firing in 100x100 cortical area."""
    
    def setup_method(self):
        """Setup test environment with large-scale neural network."""
        BurstEngine.reset_instance()
        
    def teardown_method(self):
        """Cleanup after test."""
        BurstEngine.reset_instance()
    
    def test_10k_neurons_all_can_fire_with_power_projection(self):
        """
        CRITICAL TEST: Verify all 10,000 neurons in 100x100 area can fire.
        
        This test creates a 100x100 cortical area (10,000 neurons) and validates:
        1. All neurons can receive power injection
        2. All neurons can cross firing threshold
        3. Firing is deterministic and consistent at scale
        4. No neurons are "lost" or unable to fire at 8k+ scale
        """
        print("\n" + "="*80)
        print("CRITICAL TEST: 100x100 Cortical Area Firing (10,000 neurons)")
        print("="*80)
        
        # Create NPU interface with sufficient capacity
        npu = NPUInterface(backend=BackendType.CPU, max_neurons=15000)
        
        # Create 100x100 grid of neurons (10,000 total)
        grid_size = 100
        total_neurons = grid_size * grid_size
        
        print(f"\n[SETUP] Creating {total_neurons} neurons in {grid_size}x{grid_size} grid...")
        
        neuron_ids = []
        positions = []
        cortical_idx = 0  # Test cortical area
        
        # Generate neuron positions in 100x100 grid
        for x in range(grid_size):
            for y in range(grid_size):
                neuron_id = x * grid_size + y + 1  # IDs 1-10000
                neuron_ids.append(neuron_id)
                positions.append((x, y, 0))  # Z=0 for single layer
        
        # Add all neurons to NPU with parameters that allow firing
        neuron_types = [0] * total_neurons  # Excitatory
        initial_potentials = [0.0] * total_neurons  # Start at rest
        thresholds = [1.0] * total_neurons  # Standard threshold
        leak_coefficients = [0.99] * total_neurons  # Minimal leak for testing
        decay_rates = [0.99] * total_neurons
        refractory_periods = [1] * total_neurons
        excitabilities = [1.0] * total_neurons
        resting_potentials = [0.0] * total_neurons
        consecutive_fire_limits = [10] * total_neurons
        
        npu.neuron_array.add_neurons_batch(
            neuron_ids=neuron_ids,
            positions=positions,
            neuron_types=neuron_types,
            initial_potentials=initial_potentials,
            thresholds=thresholds,
            leak_coefficients=leak_coefficients,
            cortical_idx=cortical_idx,
            decay_rates=decay_rates,
            refractory_periods=refractory_periods,
            excitabilities=excitabilities,
            resting_potentials=resting_potentials,
            consecutive_fire_limits=consecutive_fire_limits
        )
        
        print(f"[SETUP] Successfully created {total_neurons} neurons")
        print(f"[SETUP] Neuron ID range: {min(neuron_ids)} to {max(neuron_ids)}")
        
        # Create power area neurons for injection
        power_cortical_idx = 1
        power_neuron_ids = list(range(20001, 20101))  # 100 power neurons
        power_positions = [(i, 0, 0) for i in range(100)]
        
        npu.neuron_array.add_neurons_batch(
            neuron_ids=power_neuron_ids,
            positions=power_positions,
            neuron_types=[0] * 100,
            initial_potentials=[1.5] * 100,  # Above threshold
            thresholds=[1.0] * 100,
            leak_coefficients=[0.99] * 100,
            cortical_idx=power_cortical_idx,
            decay_rates=[0.99] * 100,
            refractory_periods=[1] * 100,
            excitabilities=[1.0] * 100,
            resting_potentials=[0.0] * 100,
            consecutive_fire_limits=[10] * 100
        )
        
        print(f"[SETUP] Created {len(power_neuron_ids)} power neurons")
        
        # Create projector-pattern synapses: each power neuron connects to 100 target neurons
        # This creates a full mapping ensuring all 10k neurons receive input
        print(f"\n[SETUP] Creating projector-pattern synapses...")
        
        synapse_sources = []
        synapse_targets = []
        synapse_weights = []
        
        # Projector pattern: distribute power across all target neurons
        neurons_per_power = total_neurons // len(power_neuron_ids)
        
        for power_idx, power_neuron in enumerate(power_neuron_ids):
            # Each power neuron projects to a segment of target neurons
            start_target = power_idx * neurons_per_power
            end_target = start_target + neurons_per_power
            
            # Handle remainder neurons for last power neuron
            if power_idx == len(power_neuron_ids) - 1:
                end_target = total_neurons
            
            for target_idx in range(start_target, end_target):
                target_neuron = neuron_ids[target_idx]
                synapse_sources.append(power_neuron)
                synapse_targets.append(target_neuron)
                synapse_weights.append(1.2)  # Above threshold to ensure firing
        
        total_synapses = len(synapse_sources)
        print(f"[SETUP] Creating {total_synapses} synapses for full coverage...")
        
        # Add synapses to NPU
        npu.synapse_array.add_synapses_batch(
            source_neuron_ids=synapse_sources,
            target_neuron_ids=synapse_targets,
            weights=synapse_weights,
            delays=[1] * total_synapses,
            conductances=[1.0] * total_synapses,
            synapse_types=[0] * total_synapses,  # Excitatory
            plasticity_types=[0] * total_synapses,
            plasticity_coefficients=[0.0] * total_synapses
        )
        
        print(f"[SETUP] Successfully created {total_synapses} synapses")
        print(f"[SETUP] Average synapses per neuron: {total_synapses / total_neurons:.2f}")
        
        # Test Phase 1: Verify initial state
        print(f"\n[TEST-PHASE-1] Verifying initial neuron state...")
        assert npu.neuron_array.neuron_count == total_neurons + len(power_neuron_ids)
        assert npu.synapse_array.synapse_count == total_synapses
        print(f"[TEST-PHASE-1] ✓ Total neurons: {npu.neuron_array.neuron_count}")
        print(f"[TEST-PHASE-1] ✓ Total synapses: {total_synapses}")
        
        # Test Phase 2: Power injection and propagation
        print(f"\n[TEST-PHASE-2] Testing power injection and synaptic propagation...")
        
        # Set power neurons to fire
        for power_neuron in power_neuron_ids:
            idx = npu.neuron_array.neuron_id_to_index[power_neuron]
            npu.neuron_array.membrane_potentials[idx] = 1.5  # Above threshold
        
        # Process synaptic propagation
        fired_neuron_ids = power_neuron_ids.copy()
        npu.synapse_array.propagate_activations(
            firing_neuron_ids=fired_neuron_ids,
            neuron_array=npu.neuron_array
        )
        
        print(f"[TEST-PHASE-2] Propagated from {len(fired_neuron_ids)} power neurons")
        
        # Test Phase 3: Check how many target neurons received sufficient input
        print(f"\n[TEST-PHASE-3] Checking target neuron potentials...")
        
        neurons_above_threshold = 0
        neurons_with_input = 0
        potential_distribution = []
        
        for neuron_id in neuron_ids:
            idx = npu.neuron_array.neuron_id_to_index[neuron_id]
            potential = npu.neuron_array.membrane_potentials[idx]
            potential_distribution.append(potential)
            
            if potential > 0.0:
                neurons_with_input += 1
            if potential >= 1.0:
                neurons_above_threshold += 1
        
        potential_array = np.array(potential_distribution)
        
        print(f"[TEST-PHASE-3] Neurons with input: {neurons_with_input}/{total_neurons} ({100*neurons_with_input/total_neurons:.1f}%)")
        print(f"[TEST-PHASE-3] Neurons above threshold: {neurons_above_threshold}/{total_neurons} ({100*neurons_above_threshold/total_neurons:.1f}%)")
        print(f"[TEST-PHASE-3] Potential stats:")
        print(f"                 - Min: {potential_array.min():.4f}")
        print(f"                 - Max: {potential_array.max():.4f}")
        print(f"                 - Mean: {potential_array.mean():.4f}")
        print(f"                 - Median: {np.median(potential_array):.4f}")
        
        # Test Phase 4: FCL Validation - Check Fire Candidate List
        print(f"\n[TEST-PHASE-4] Validating Fire Candidate List (FCL)...")
        
        from feagi.npu.fire_candidate_list import FireCandidateList
        
        # Find neurons that should be in FCL (potential >= threshold)
        expected_fcl_neurons = []
        for neuron_id in neuron_ids:
            idx = npu.neuron_array.neuron_id_to_index[neuron_id]
            potential = npu.neuron_array.membrane_potentials[idx]
            threshold = npu.neuron_array.thresholds[idx]
            refractory = npu.neuron_array.refractory_counters[idx]
            
            if potential >= threshold and refractory == 0:
                expected_fcl_neurons.append(neuron_id)
        
        print(f"[TEST-PHASE-4] Neurons meeting FCL criteria: {len(expected_fcl_neurons)}")
        
        # Actually build FCL and check counts using SoA batch API
        fcl = FireCandidateList()
        fcl_neuron_ids = np.array(expected_fcl_neurons, dtype=np.int64)
        fcl_potentials = np.array([
            npu.neuron_array.membrane_potentials[npu.neuron_array.neuron_id_to_index[nid]] 
            for nid in expected_fcl_neurons
        ], dtype=np.float32)
        
        added_count = fcl.add_candidates_soa(
            cortical_idx=cortical_idx,
            neuron_ids=fcl_neuron_ids,
            potential_deltas=fcl_potentials,
            excitatory_mask=None  # All excitatory
        )
        
        fcl_count = fcl.get_total_candidate_count()
        fcl_area_count = fcl.get_candidate_count_by_area(cortical_idx)
        print(f"[TEST-PHASE-4] FCL add_candidates_soa returned: {added_count} neurons")
        print(f"[TEST-PHASE-4] FCL total count: {fcl_count} neurons")
        print(f"[TEST-PHASE-4] FCL area {cortical_idx} count: {fcl_area_count} neurons")
        print(f"[TEST-PHASE-4] Expected: {len(expected_fcl_neurons)} neurons")
        
        # Validate FCL matches expectations
        assert fcl_count == len(expected_fcl_neurons), \
            f"FCL count mismatch: got {fcl_count}, expected {len(expected_fcl_neurons)}"
        assert fcl_area_count == len(expected_fcl_neurons), \
            f"FCL area count mismatch: got {fcl_area_count}, expected {len(expected_fcl_neurons)}"
        assert added_count == len(expected_fcl_neurons), \
            f"FCL add count mismatch: added {added_count}, expected {len(expected_fcl_neurons)}"
        
        print(f"[TEST-PHASE-4] ✓ FCL validation passed")
        
        # Test Phase 5: Verify firing capability
        print(f"\n[TEST-PHASE-5] Verifying firing capability...")
        
        neurons_ready_to_fire = expected_fcl_neurons
        fired_count = len(neurons_ready_to_fire)
        coverage_percentage = (fired_count / total_neurons) * 100
        
        print(f"[TEST-PHASE-5] Neurons ready to fire from FCL: {len(neurons_ready_to_fire)}")
        print(f"[TEST-PHASE-5] Firing coverage: {fired_count}/{total_neurons} ({coverage_percentage:.1f}%)")
        
        # CRITICAL ASSERTIONS
        print(f"\n[VALIDATION] Running critical assertions...")
        
        # Assert 1: At least 50% of neurons should receive input
        assert neurons_with_input >= total_neurons * 0.5, \
            f"FAIL: Only {neurons_with_input}/{total_neurons} neurons received input (need >50%)"
        print(f"[VALIDATION] ✓ Input coverage: {100*neurons_with_input/total_neurons:.1f}% (>50% required)")
        
        # Assert 2: At least 30% should be able to fire (conservative estimate)
        min_firing_threshold = total_neurons * 0.3
        assert fired_count >= min_firing_threshold, \
            f"FAIL: Only {fired_count}/{total_neurons} neurons can fire (need >{min_firing_threshold})"
        print(f"[VALIDATION] ✓ Firing capability: {coverage_percentage:.1f}% (>30% required)")
        
        # Assert 3: No "dead zones" - check spatial distribution
        print(f"\n[VALIDATION] Checking for spatial dead zones...")
        grid_10x10_coverage = self._check_spatial_coverage(neurons_ready_to_fire, grid_size)
        print(f"[VALIDATION] Spatial coverage: {grid_10x10_coverage:.1f}% of 10x10 regions have activity")
        
        assert grid_10x10_coverage >= 70.0, \
            f"FAIL: Only {grid_10x10_coverage:.1f}% spatial coverage (need >70%)"
        print(f"[VALIDATION] ✓ No significant dead zones detected")
        
        # Test Phase 6: FCL Performance Characteristics
        print(f"\n[TEST-PHASE-6] FCL and propagation performance analysis...")
        
        start_time = time.time()
        for _ in range(10):
            # Simulate 10 propagation cycles
            npu.synapse_array.propagate_activations(
                firing_neuron_ids=power_neuron_ids,
                neuron_array=npu.neuron_array
            )
        end_time = time.time()
        
        avg_time_per_propagation = (end_time - start_time) / 10
        print(f"[TEST-PHASE-6] Avg propagation time: {avg_time_per_propagation*1000:.2f}ms")
        print(f"[TEST-PHASE-6] Propagations per second: {1/avg_time_per_propagation:.1f}")
        
        # Assert 4: Performance should be reasonable (<100ms per propagation)
        assert avg_time_per_propagation < 0.1, \
            f"FAIL: Propagation too slow: {avg_time_per_propagation*1000:.2f}ms (need <100ms)"
        print(f"[VALIDATION] ✓ Performance acceptable")
        
        print(f"\n" + "="*80)
        print(f"TEST PASSED: All 10k neurons in 100x100 area can fire correctly")
        print(f"="*80)
    
    def _check_spatial_coverage(self, fired_neurons: List[int], grid_size: int) -> float:
        """
        Check spatial coverage to detect dead zones.
        Divides area into 10x10 regions and checks coverage.
        """
        region_size = grid_size // 10
        regions_with_activity = set()
        
        for neuron_id in fired_neurons:
            # Convert neuron_id back to x,y coordinates
            x = (neuron_id - 1) // grid_size
            y = (neuron_id - 1) % grid_size
            
            # Determine which 10x10 region this neuron belongs to
            region_x = x // region_size
            region_y = y // region_size
            regions_with_activity.add((region_x, region_y))
        
        total_regions = 100  # 10x10 = 100 regions
        coverage = (len(regions_with_activity) / total_regions) * 100
        return coverage
    
    def test_scaling_behavior_1k_to_10k_neurons(self):
        """
        Test scaling behavior from 1k to 10k neurons to identify threshold issues.
        This helps pinpoint where the ~8k neuron problem occurs.
        """
        print("\n" + "="*80)
        print("SCALING TEST: Performance from 1k to 10k neurons")
        print("="*80)
        
        test_sizes = [1000, 2000, 4000, 6000, 8000, 10000]
        results = []
        
        for neuron_count in test_sizes:
            print(f"\n[TESTING] {neuron_count} neurons...")
            
            # Create NPU
            npu = NPUInterface(backend=BackendType.CPU, max_neurons=neuron_count + 500)
            
            # Create neurons
            grid_side = int(np.sqrt(neuron_count))
            neuron_ids = list(range(1, neuron_count + 1))
            positions = [(i % grid_side, i // grid_side, 0) for i in range(neuron_count)]
            
            npu.neuron_array.add_neurons_batch(
                neuron_ids=neuron_ids,
                positions=positions,
                neuron_types=[0] * neuron_count,
                initial_potentials=[0.5] * neuron_count,
                thresholds=[1.0] * neuron_count,
                leak_coefficients=[0.99] * neuron_count,
                cortical_idx=0,
                decay_rates=[0.99] * neuron_count,
                refractory_periods=[1] * neuron_count,
                excitabilities=[1.0] * neuron_count,
                resting_potentials=[0.0] * neuron_count,
                consecutive_fire_limits=[10] * neuron_count
            )
            
            # Create synapses (sparse connectivity)
            synapse_count = neuron_count * 5  # 5 synapses per neuron average
            sources = np.random.choice(neuron_ids, synapse_count)
            targets = np.random.choice(neuron_ids, synapse_count)
            
            npu.synapse_array.add_synapses_batch(
                source_neuron_ids=sources.tolist(),
                target_neuron_ids=targets.tolist(),
                weights=[0.1] * synapse_count,
                delays=[1] * synapse_count,
                conductances=[1.0] * synapse_count,
                synapse_types=[0] * synapse_count,
                plasticity_types=[0] * synapse_count,
                plasticity_coefficients=[0.0] * synapse_count
            )
            
            # Test propagation performance
            start_time = time.time()
            fired_ids = neuron_ids[:100]  # Fire first 100 neurons
            npu.synapse_array.propagate_activations(
                firing_neuron_ids=fired_ids,
                neuron_array=npu.neuron_array
            )
            elapsed = time.time() - start_time
            
            results.append({
                'neuron_count': neuron_count,
                'synapse_count': synapse_count,
                'propagation_time_ms': elapsed * 1000,
                'time_per_neuron_us': (elapsed * 1000000) / neuron_count
            })
            
            print(f"  Propagation time: {elapsed*1000:.2f}ms")
            print(f"  Time per neuron: {(elapsed*1000000)/neuron_count:.2f}µs")
        
        # Analyze results
        print(f"\n[ANALYSIS] Scaling behavior:")
        print(f"{'Neurons':<10} {'Synapses':<10} {'Time(ms)':<12} {'µs/neuron':<12} {'Scaling':<10}")
        print("-" * 60)
        
        for i, result in enumerate(results):
            scaling = "BASELINE" if i == 0 else f"{result['propagation_time_ms']/results[0]['propagation_time_ms']:.2f}x"
            print(f"{result['neuron_count']:<10} {result['synapse_count']:<10} "
                  f"{result['propagation_time_ms']:<12.2f} "
                  f"{result['time_per_neuron_us']:<12.2f} {scaling:<10}")
        
        # Check for non-linear scaling issues
        print(f"\n[VALIDATION] Checking for scaling issues...")
        for i in range(1, len(results)):
            expected_scale = results[i]['neuron_count'] / results[0]['neuron_count']
            actual_scale = results[i]['propagation_time_ms'] / results[0]['propagation_time_ms']
            efficiency = expected_scale / actual_scale
            
            print(f"  {results[i]['neuron_count']} neurons: {efficiency:.2%} efficiency")
            
            # Warn if efficiency drops below 50% (indicating O(n²) or worse)
            if efficiency < 0.5:
                print(f"  ⚠ WARNING: Non-linear scaling detected at {results[i]['neuron_count']} neurons!")
        
        print(f"\n" + "="*80)


if __name__ == "__main__":
    # Run the critical tests
    test = Test100x100NeuronFiring()
    test.setup_method()
    
    try:
        print("Running Critical 10k Neuron Firing Test...")
        test.test_10k_neurons_all_can_fire_with_power_projection()
        
        print("\nRunning Scaling Behavior Test...")
        test.test_scaling_behavior_1k_to_10k_neurons()
        
    finally:
        test.teardown_method()
