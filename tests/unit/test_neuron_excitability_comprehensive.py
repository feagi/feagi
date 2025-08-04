"""
Comprehensive pytest tests for neuron excitability feature.

Tests cover:
1. Basic functionality (0%, 50%, 100% excitability)
2. Backend compatibility (NumPy, SIMD, Numba, GPU)
3. Performance verification
4. Integration with neural update cycle
5. Cortical area-specific behavior
6. Genome validation
7. Edge cases and error handling
"""

import pytest
import numpy as np
import sys
import os

# Add FEAGI to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))

from feagi.bdu.models.neuron import (
    NeuronArray,
    simd_firing_check_with_excitability,
    simd_firing_check,
    NUMBA_AVAILABLE
)
from feagi.evo.genome_validator import validate_cortical_parameters


class TestNeuronExcitabilityBasics:
    """Test basic excitability functionality."""
    
    def test_excitability_array_initialization(self):
        """Test that NeuronArray properly initializes excitability array."""
        neuron_array = NeuronArray(max_neurons=1000, backend="numpy")
        
        # Check array exists and has correct default values
        assert hasattr(neuron_array, 'excitability')
        assert neuron_array.excitability.shape[0] >= 1000
        assert np.all(neuron_array.excitability[:100] == 1.0)
        assert neuron_array.excitability.dtype == np.float32
        
    def test_cortical_area_excitability_setting(self):
        """Test setting excitability for cortical areas."""
        neuron_array = NeuronArray(max_neurons=1000, backend="numpy")
        
        # Set excitability for cortical area 0
        neuron_array.set_cortical_area_excitability(
            cortical_idx=0, start_idx=0, end_idx=500, excitability=0.8
        )
        
        # Verify values
        assert np.all(neuron_array.excitability[0:500] == 0.8)
        assert np.all(neuron_array.excitability[500:1000] == 1.0)  # Unchanged
        
        # Verify area tracking
        probabilistic_areas = neuron_array.get_probabilistic_areas()
        assert 0 in probabilistic_areas
        
    def test_excitability_clamping(self):
        """Test that excitability values are properly clamped."""
        neuron_array = NeuronArray(max_neurons=100, backend="numpy")
        
        # Test clamping to valid range
        neuron_array.set_cortical_area_excitability(0, 0, 25, 1.5)  # > 1.0
        neuron_array.set_cortical_area_excitability(1, 25, 50, -0.5)  # < 0.0
        neuron_array.set_cortical_area_excitability(2, 50, 75, 0.5)  # Valid
        
        assert np.all(neuron_array.excitability[0:25] == 1.0)  # Clamped to 1.0
        assert np.all(neuron_array.excitability[25:50] == 0.0)  # Clamped to 0.0
        assert np.all(neuron_array.excitability[50:75] == 0.5)  # Unchanged


class TestProbabilisticFiring:
    """Test probabilistic firing behavior."""
    
    def test_deterministic_firing(self):
        """Test that excitability = 1.0 always fires when threshold is met."""
        n_neurons = 1000
        membrane_potentials = np.ones(n_neurons, dtype=np.float32) * 2.0  # Above threshold
        thresholds = np.ones(n_neurons, dtype=np.float32) * 1.0
        can_fire_mask = np.ones(n_neurons, dtype=bool)
        excitability = np.ones(n_neurons, dtype=np.float32) * 1.0
        
        fired_mask = simd_firing_check_with_excitability(
            membrane_potentials, thresholds, can_fire_mask, excitability
        )
        
        assert np.sum(fired_mask) == n_neurons, "All neurons should fire with 100% excitability"
        
    def test_zero_excitability(self):
        """Test that excitability = 0.0 never fires."""
        n_neurons = 1000
        membrane_potentials = np.ones(n_neurons, dtype=np.float32) * 2.0  # Above threshold
        thresholds = np.ones(n_neurons, dtype=np.float32) * 1.0
        can_fire_mask = np.ones(n_neurons, dtype=bool)
        excitability = np.zeros(n_neurons, dtype=np.float32)
        
        fired_mask = simd_firing_check_with_excitability(
            membrane_potentials, thresholds, can_fire_mask, excitability
        )
        
        assert np.sum(fired_mask) == 0, "No neurons should fire with 0% excitability"
        
    def test_probabilistic_firing_statistics(self):
        """Test that probabilistic firing follows expected statistics."""
        n_neurons = 10000
        excitability_value = 0.5
        trials = 10
        
        membrane_potentials = np.ones(n_neurons, dtype=np.float32) * 2.0
        thresholds = np.ones(n_neurons, dtype=np.float32) * 1.0
        can_fire_mask = np.ones(n_neurons, dtype=bool)
        excitability = np.ones(n_neurons, dtype=np.float32) * excitability_value
        
        firing_rates = []
        for _ in range(trials):
            fired_mask = simd_firing_check_with_excitability(
                membrane_potentials, thresholds, can_fire_mask, excitability
            )
            firing_rate = np.sum(fired_mask) / n_neurons
            firing_rates.append(firing_rate)
        
        avg_firing_rate = np.mean(firing_rates)
        
        # Should be roughly 50% with some tolerance for randomness
        assert 0.45 <= avg_firing_rate <= 0.55, f"Expected ~50% firing rate, got {avg_firing_rate*100:.1f}%"
        
    def test_threshold_requirement(self):
        """Test that excitability only affects neurons that meet threshold."""
        n_neurons = 1000
        membrane_potentials = np.random.uniform(0.5, 1.5, n_neurons).astype(np.float32)  # Below threshold
        thresholds = np.ones(n_neurons, dtype=np.float32) * 2.0  # High threshold
        can_fire_mask = np.ones(n_neurons, dtype=bool)
        excitability = np.ones(n_neurons, dtype=np.float32) * 1.0  # 100% excitability
        
        fired_mask = simd_firing_check_with_excitability(
            membrane_potentials, thresholds, can_fire_mask, excitability
        )
        
        assert np.sum(fired_mask) == 0, "No neurons should fire when below threshold, regardless of excitability"


class TestBackendCompatibility:
    """Test compatibility across different computational backends."""
    
    def test_numpy_backend(self):
        """Test excitability with NumPy backend."""
        neuron_array = NeuronArray(max_neurons=1000, backend="numpy")
        neuron_array.next_index = 1000
        neuron_array.neuron_count = 1000
        
        # Setup neural state
        neuron_array.membrane_potentials[:1000] = 2.0
        neuron_array.thresholds[:1000] = 1.0
        neuron_array.valid_mask[:1000] = True
        neuron_array.refractory_counters[:1000] = 0
        neuron_array.cortical_idxs[:1000] = 0
        
        # Set 50% excitability
        neuron_array.set_cortical_area_excitability(0, 0, 1000, 0.5)
        
        # Run neural update
        fired_neurons = neuron_array.embedded_optimized_neural_update(timestep=1)
        
        # Should have some but not all neurons firing
        assert 0 < len(fired_neurons) < 1000
        
    @pytest.mark.skipif(not NUMBA_AVAILABLE, reason="Numba not available")
    def test_numba_backend(self):
        """Test excitability with Numba backend."""
        from feagi.bdu.models.neuron import simd_firing_check_with_excitability_numba
        
        n_neurons = 1000
        membrane_potentials = np.ones(n_neurons, dtype=np.float32) * 2.0
        thresholds = np.ones(n_neurons, dtype=np.float32) * 1.0
        can_fire_mask = np.ones(n_neurons, dtype=bool)
        excitability = np.ones(n_neurons, dtype=np.float32) * 0.7
        
        fired_mask = simd_firing_check_with_excitability_numba(
            membrane_potentials, thresholds, can_fire_mask, excitability
        )
        
        firing_rate = np.sum(fired_mask) / n_neurons
        assert 0.6 <= firing_rate <= 0.8  # Should be around 70%
        
    def test_rust_backend_integration(self):
        """Test integration with Rust backend (if available)."""
        try:
            from feagi_rust import Neuron
            
            # Create Rust neuron with excitability
            neuron = Neuron.new_with_excitability(
                id=1, threshold=1.0, decay_rate=0.95, refractory_period=2, excitability=0.8
            )
            
            # Test firing with excitability
            fired_count = 0
            trials = 1000
            
            for _ in range(trials):
                neuron.membrane_potential = 2.0  # Above threshold
                neuron.refractory_countdown = 0   # Can fire
                
                if neuron.update(1.0):  # Input above threshold
                    fired_count += 1
                    
            firing_rate = fired_count / trials
            assert 0.7 <= firing_rate <= 0.9  # Should be around 80%
            
        except ImportError:
            pytest.skip("Rust backend not available")


class TestPerformanceOptimization:
    """Test performance optimization features."""
    
    def test_fast_path_detection(self):
        """Test that fast path is used when all areas have excitability = 1.0."""
        neuron_array = NeuronArray(max_neurons=1000, backend="numpy")
        
        # Set all areas to 100% excitability
        neuron_array.set_cortical_area_excitability(0, 0, 500, 1.0)
        neuron_array.set_cortical_area_excitability(1, 500, 1000, 1.0)
        
        # Should have no probabilistic areas
        probabilistic_areas = neuron_array.get_probabilistic_areas()
        assert len(probabilistic_areas) == 0
        
    def test_mixed_area_detection(self):
        """Test that probabilistic areas are correctly detected."""
        neuron_array = NeuronArray(max_neurons=1000, backend="numpy")
        
        # Set mixed excitability
        neuron_array.set_cortical_area_excitability(0, 0, 500, 1.0)    # Deterministic
        neuron_array.set_cortical_area_excitability(1, 500, 1000, 0.8) # Probabilistic
        
        # Should detect area 1 as probabilistic
        probabilistic_areas = neuron_array.get_probabilistic_areas()
        assert probabilistic_areas == {1}
        
    def test_performance_comparison(self):
        """Test performance difference between deterministic and probabilistic paths."""
        import time
        
        n_neurons = 50000
        iterations = 10
        
        # Setup test data
        membrane_potentials = np.random.uniform(0.5, 2.0, n_neurons).astype(np.float32)
        thresholds = np.ones(n_neurons, dtype=np.float32)
        can_fire_mask = np.ones(n_neurons, dtype=bool)
        
        # Test original SIMD function (baseline)
        start_time = time.perf_counter()
        for _ in range(iterations):
            fired_mask = simd_firing_check(membrane_potentials, thresholds, can_fire_mask)
        baseline_time = time.perf_counter() - start_time
        
        # Test with all excitability = 1.0 (should be fast)
        excitability_deterministic = np.ones(n_neurons, dtype=np.float32)
        start_time = time.perf_counter()
        for _ in range(iterations):
            fired_mask = simd_firing_check_with_excitability(
                membrane_potentials, thresholds, can_fire_mask, excitability_deterministic
            )
        deterministic_time = time.perf_counter() - start_time
        
        # Test with mixed excitability (expected to be slower)
        excitability_mixed = np.random.uniform(0.5, 1.0, n_neurons).astype(np.float32)
        start_time = time.perf_counter()
        for _ in range(iterations):
            fired_mask = simd_firing_check_with_excitability(
                membrane_potentials, thresholds, can_fire_mask, excitability_mixed
            )
        probabilistic_time = time.perf_counter() - start_time
        
        # Verify performance characteristics
        deterministic_overhead = (deterministic_time / baseline_time - 1) * 100
        probabilistic_overhead = (probabilistic_time / baseline_time - 1) * 100
        
        # Deterministic should be reasonably close to baseline
        assert deterministic_overhead < 100, f"Deterministic overhead too high: {deterministic_overhead:.1f}%"
        
        # Probabilistic should be slower than deterministic
        assert probabilistic_time > deterministic_time, "Probabilistic path should be slower"
        
        print(f"Performance Test Results:")
        print(f"  Deterministic overhead: {deterministic_overhead:+.1f}%")
        print(f"  Probabilistic overhead: {probabilistic_overhead:+.1f}%")


class TestGenomeValidation:
    """Test genome validation for excitability parameters."""
    
    def test_valid_excitability_validation(self):
        """Test validation of valid excitability values."""
        blueprint = {
            "test_area": {
                "neuron_excitability": 0.8,
                "firing_threshold": 1.0,
            }
        }
        
        result = validate_cortical_parameters(blueprint)
        assert result["valid"]
        assert len(result["errors"]) == 0
        
    def test_auto_correction_high_excitability(self):
        """Test auto-correction of excitability > 1.0."""
        blueprint = {
            "test_area": {
                "neuron_excitability": 150.0,  # Should be corrected to 1.0
            }
        }
        
        result = validate_cortical_parameters(blueprint)
        assert result["valid"]
        assert len(result["warnings"]) > 0
        assert blueprint["test_area"]["neuron_excitability"] == 1.0
        
    def test_invalid_negative_excitability(self):
        """Test rejection of negative excitability."""
        blueprint = {
            "test_area": {
                "neuron_excitability": -0.5,
            }
        }
        
        result = validate_cortical_parameters(blueprint)
        assert not result["valid"]
        assert len(result["errors"]) > 0
        assert "below minimum" in result["errors"][0].lower()
        
    def test_type_validation(self):
        """Test type validation for excitability."""
        blueprint = {
            "test_area": {
                "neuron_excitability": "invalid_string",
            }
        }
        
        result = validate_cortical_parameters(blueprint)
        assert not result["valid"]
        assert len(result["errors"]) > 0


class TestIntegrationWithNeuralUpdate:
    """Test integration with the complete neural update cycle."""
    
    def test_neural_update_with_excitability(self):
        """Test that excitability works within the full neural update cycle."""
        neuron_array = NeuronArray(max_neurons=1000, backend="numpy")
        neuron_array.next_index = 1000
        neuron_array.neuron_count = 1000
        
        # Setup realistic neural state
        neuron_array.membrane_potentials[:1000] = np.random.uniform(0.8, 1.2, 1000)
        neuron_array.thresholds[:1000] = 1.0
        neuron_array.valid_mask[:1000] = True
        neuron_array.refractory_counters[:1000] = 0
        neuron_array.cortical_idxs[:1000] = 0
        
        # Test with different excitability values
        test_cases = [
            (1.0, "should fire normally"),
            (0.5, "should fire ~50% of eligible neurons"),
            (0.0, "should never fire"),
        ]
        
        for excitability, description in test_cases:
            neuron_array.set_cortical_area_excitability(0, 0, 1000, excitability)
            
            # Reset state
            neuron_array.membrane_potentials[:1000] = 1.5  # Above threshold
            neuron_array.refractory_counters[:1000] = 0
            
            fired_neurons = neuron_array.embedded_optimized_neural_update(timestep=1)
            
            if excitability == 1.0:
                # All eligible neurons should fire
                assert len(fired_neurons) > 900  # Most should fire
            elif excitability == 0.5:
                # About half should fire (with statistical variation)
                assert 300 <= len(fired_neurons) <= 700
            elif excitability == 0.0:
                # None should fire
                assert len(fired_neurons) == 0
                
    def test_cortical_area_isolation(self):
        """Test that excitability affects only the specified cortical area."""
        neuron_array = NeuronArray(max_neurons=1000, backend="numpy")
        neuron_array.next_index = 1000
        neuron_array.neuron_count = 1000
        
        # Setup two cortical areas
        neuron_array.membrane_potentials[:1000] = 2.0  # Above threshold
        neuron_array.thresholds[:1000] = 1.0
        neuron_array.valid_mask[:1000] = True
        neuron_array.refractory_counters[:1000] = 0
        
        # Area 0: first 500 neurons
        neuron_array.cortical_idxs[:500] = 0
        neuron_array.set_cortical_area_excitability(0, 0, 500, 0.0)  # Never fire
        
        # Area 1: last 500 neurons
        neuron_array.cortical_idxs[500:] = 1
        neuron_array.set_cortical_area_excitability(1, 500, 1000, 1.0)  # Always fire
        
        fired_neurons = neuron_array.embedded_optimized_neural_update(timestep=1)
        
        # Only neurons from area 1 should fire
        for neuron_id in fired_neurons:
            # Convert neuron_id to array index to check cortical area
            # This requires accessing the neuron ID to index mapping
            # For simplicity, assume neurons 0-499 are area 0, 500-999 are area 1
            assert neuron_id >= 500, f"Neuron {neuron_id} from area 0 should not fire"


class TestEdgeCases:
    """Test edge cases and error handling."""
    
    def test_empty_cortical_areas(self):
        """Test behavior with no cortical areas defined."""
        neuron_array = NeuronArray(max_neurons=100, backend="numpy")
        
        # Should have no probabilistic areas
        probabilistic_areas = neuron_array.get_probabilistic_areas()
        assert len(probabilistic_areas) == 0
        
    def test_boundary_excitability_values(self):
        """Test boundary values for excitability."""
        neuron_array = NeuronArray(max_neurons=100, backend="numpy")
        
        # Test exactly 0.0 and 1.0
        neuron_array.set_cortical_area_excitability(0, 0, 50, 0.0)
        neuron_array.set_cortical_area_excitability(1, 50, 100, 1.0)
        
        probabilistic_areas = neuron_array.get_probabilistic_areas()
        assert probabilistic_areas == {0}  # Only area 0 is probabilistic
        
    def test_very_small_excitability(self):
        """Test very small but non-zero excitability values."""
        n_neurons = 10000
        membrane_potentials = np.ones(n_neurons, dtype=np.float32) * 2.0
        thresholds = np.ones(n_neurons, dtype=np.float32) * 1.0
        can_fire_mask = np.ones(n_neurons, dtype=bool)
        excitability = np.ones(n_neurons, dtype=np.float32) * 0.001  # 0.1%
        
        # Run multiple trials
        total_fired = 0
        trials = 100
        
        for _ in range(trials):
            fired_mask = simd_firing_check_with_excitability(
                membrane_potentials, thresholds, can_fire_mask, excitability
            )
            total_fired += np.sum(fired_mask)
            
        avg_firing_rate = total_fired / (trials * n_neurons)
        
        # Should be very low but non-zero
        assert 0.0 < avg_firing_rate < 0.01


@pytest.mark.performance
class TestPerformanceBenchmarks:
    """Performance benchmarks for the excitability feature."""
    
    def test_large_scale_performance(self):
        """Test performance with large numbers of neurons."""
        import time
        
        n_neurons = 500000  # Half a million neurons
        neuron_array = NeuronArray(max_neurons=n_neurons, backend="numpy")
        neuron_array.next_index = n_neurons
        neuron_array.neuron_count = n_neurons
        
        # Setup state
        neuron_array.membrane_potentials[:n_neurons] = np.random.uniform(0.5, 2.0, n_neurons)
        neuron_array.thresholds[:n_neurons] = 1.0
        neuron_array.valid_mask[:n_neurons] = True
        neuron_array.refractory_counters[:n_neurons] = 0
        neuron_array.cortical_idxs[:n_neurons] = 0
        
        # Test deterministic case
        neuron_array.set_cortical_area_excitability(0, 0, n_neurons, 1.0)
        
        start_time = time.perf_counter()
        fired_neurons = neuron_array.embedded_optimized_neural_update(timestep=1)
        deterministic_time = time.perf_counter() - start_time
        
        # Test probabilistic case
        neuron_array.set_cortical_area_excitability(0, 0, n_neurons, 0.8)
        
        start_time = time.perf_counter()
        fired_neurons = neuron_array.embedded_optimized_neural_update(timestep=1)
        probabilistic_time = time.perf_counter() - start_time
        
        print(f"Large Scale Performance ({n_neurons:,} neurons):")
        print(f"  Deterministic: {deterministic_time*1000:.1f}ms")
        print(f"  Probabilistic: {probabilistic_time*1000:.1f}ms")
        print(f"  Overhead: {(probabilistic_time/deterministic_time-1)*100:.1f}%")
        
        # Should complete in reasonable time (< 1 second each)
        assert deterministic_time < 1.0
        assert probabilistic_time < 2.0


# Utility functions for running tests
def run_basic_tests():
    """Run basic functionality tests."""
    pytest.main([__file__ + "::TestNeuronExcitabilityBasics", "-v"])

def run_performance_tests():
    """Run performance-focused tests."""
    pytest.main([__file__ + "::TestPerformanceOptimization", "-v"])
    pytest.main([__file__ + "::TestPerformanceBenchmarks", "-v"])

def run_all_tests():
    """Run all tests."""
    pytest.main([__file__, "-v"])


if __name__ == "__main__":
    # Run all tests if called directly
    run_all_tests() 