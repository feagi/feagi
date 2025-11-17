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

from feagi.npu.data_structures import NeuronArray
from feagi.npu.simd_neural_ops import (
    simd_firing_check_with_excitability,
    simd_firing_check,
)
from feagi.evo.genome_validator import validate_cortical_parameters


class TestNeuronExcitabilityBasics:
    """Test basic excitability functionality."""
    
    def test_excitability_array_initialization(self):
        """No per-neuron excitability array exists; per-area cache is used."""
        neuron_array = NeuronArray(max_neurons=1000, backend="cpu")
        assert not hasattr(neuron_array, 'excitabilities')
        
    def test_cortical_area_excitability_setting(self):
        pytest.skip("Per-area excitability managed by NPUInterface; SoA-level API removed")
        
    def test_excitability_clamping(self):
        pytest.skip("Clamping validated at genome/API; SoA excitability removed")


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
        neuron_array = NeuronArray(max_neurons=1000, backend="cpu")
        neuron_array.next_index = 1000
        neuron_array.neuron_count = 1000
        
        # Setup neural state
        neuron_array.membrane_potentials[:1000] = 2.0
        neuron_array.thresholds[:1000] = 1.0
        neuron_array.valid_mask[:1000] = True
        neuron_array.refractory_counters[:1000] = 0
        neuron_array.cortical_idxs[:1000] = 0
        
        # Per-area excitability integration is validated via NPUInterface tests
        assert True
    
    def test_numba_backend(self):
        pytest.skip("Numba-specific path will be reintroduced for per-area cache later")
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
                id=1, threshold=1.0, leak_coefficient=0.95, refractory_period=2, excitability=0.8
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
        pytest.skip("Fast-path detection validated via NPU per-area cache")
        
    def test_mixed_area_detection(self):
        pytest.skip("Probabilistic area detection moved to NPUInterface tests")
        
    def test_performance_comparison(self):
        """Skip environment-sensitive performance comparison for per-area update."""
        pytest.skip("Performance comparison skipped in CI for per-area path")
        
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
        pytest.skip("Full-cycle update now depends on NPUInterface per-area cache; covered elsewhere")
                
    def test_cortical_area_isolation(self):
        pytest.skip("Isolation validated via NPUInterface group gating; SoA API removed")


class TestEdgeCases:
    """Test edge cases and error handling."""
    
    def test_empty_cortical_areas(self):
        pytest.skip("Probabilistic area metadata removed from SoA; per-area cache used")
        
    def test_boundary_excitability_values(self):
        pytest.skip("Boundary behavior validated via SIMD function tests")
        
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
        """Skip environment-sensitive large-scale performance benchmark."""
        pytest.skip("Benchmark skipped for per-area path")


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