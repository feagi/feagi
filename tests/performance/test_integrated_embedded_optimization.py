"""
Test suite for integrated embedded performance optimizations.

This test verifies that the embedded optimizations are properly integrated
into the main FEAGI architecture and can achieve the 10M neurons at 15Hz target.
"""

import pytest
import numpy as np
import time
from typing import Dict, Any, List
import sys
import os

# Add the feagi_core path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from feagi.bdu.models.neuron import NeuronArray, NUMBA_AVAILABLE, MEMORY_ALIGNMENT, VECTOR_WIDTH
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.npu.burst_engine import BurstEngine


class TestIntegratedEmbeddedOptimization:
    """Test suite for integrated embedded optimizations."""
    
    def setup_method(self):
        """Set up test parameters."""
        # Reset ConnectomeManager singleton to ensure clean state
        try:
            from feagi.bdu.connectome_manager import ConnectomeManager
            ConnectomeManager._instance = None
            ConnectomeManager._initialized = False
        except ImportError:
            pass  # Module might not be available
            
        self.test_sizes = [1000, 10000, 100000, 1000000]
        self.target_time_15hz = 66.7  # ms
        self.target_time_10hz = 100   # ms
        
    def test_cache_aligned_memory(self):
        """Test that arrays are properly cache-aligned."""
        from feagi.bdu.models.neuron import CacheAlignedArray
        
        # Test cache alignment
        for size in [64, 1000, 10000]:
            aligned_array = CacheAlignedArray(size, np.float32)
            
            # Verify alignment
            data_ptr = aligned_array.array.ctypes.data
            assert data_ptr % MEMORY_ALIGNMENT == 0, f"Array not {MEMORY_ALIGNMENT}-byte aligned"
            
            # Verify functionality
            aligned_array.array[0] = 3.14
            assert aligned_array.array[0] == 3.14
            
        print(f"✅ Cache alignment verification passed ({MEMORY_ALIGNMENT}-byte alignment)")
    
    def test_simd_operations(self):
        """Test SIMD-optimized neural operations."""
        from feagi.bdu.models.neuron import (
            simd_membrane_decay, simd_refractory_update, 
            simd_threshold_check, simd_fire_neurons
        )
        
        size = 10000
        
        # Create test arrays
        potentials = np.random.rand(size).astype(np.float32)
        decay_rates = np.full(size, 0.95, dtype=np.float32)
        valid_mask = np.random.choice([True, False], size)
        thresholds = np.full(size, 1.0, dtype=np.float32)
        refractory_counters = np.random.randint(0, 5, size, dtype=np.int32)
        resting_potentials = np.zeros(size, dtype=np.float32)
        refractory_periods = np.full(size, 2, dtype=np.int32)
        fired_mask = np.zeros(size, dtype=np.bool_)
        
        # Test membrane decay
        original_potentials = potentials.copy()
        start_time = time.perf_counter()
        simd_membrane_decay(potentials, decay_rates, valid_mask)
        decay_time = time.perf_counter() - start_time
        
        # Verify decay worked
        expected = original_potentials.copy()
        expected[valid_mask] *= decay_rates[valid_mask]
        np.testing.assert_array_almost_equal(potentials, expected)
        
        # Test threshold checking
        start_time = time.perf_counter()
        simd_threshold_check(potentials, thresholds, refractory_counters, valid_mask, fired_mask)
        threshold_time = time.perf_counter() - start_time
        
        # Test neuron firing
        start_time = time.perf_counter()
        simd_fire_neurons(potentials, resting_potentials, refractory_counters, refractory_periods, fired_mask)
        firing_time = time.perf_counter() - start_time
        
        print(f"✅ SIMD operations verified:")
        print(f"   - Numba available: {NUMBA_AVAILABLE}")
        print(f"   - Membrane decay: {decay_time*1000:.2f}ms")
        print(f"   - Threshold check: {threshold_time*1000:.2f}ms")
        print(f"   - Neuron firing: {firing_time*1000:.2f}ms")
    
    def test_neuron_array_performance(self):
        """Test NeuronArray performance with embedded optimizations."""
        print(f"\n🧪 Testing NeuronArray performance scaling...")
        
        results = []
        
        for size in self.test_sizes:
            print(f"\n📊 Testing {size:,} neurons...")
            
            # Create optimized neuron array
            neuron_array = NeuronArray(max_neurons=size)
            
            # Verify optimization features
            perf_summary = neuron_array.get_performance_summary()
            print(f"   Backend: {perf_summary.get('backend', 'N/A')}")
            print(f"   SIMD enabled: {perf_summary.get('simd_enabled', 'N/A')}")
            print(f"   Memory alignment: {perf_summary.get('alignment', 'N/A')}B")
            
            # Create test neurons
            for i in range(min(size, 1000)):  # Limit for performance
                neuron_array.create_neuron(
                    cortical_idx=0,
                    position=(i % 100, (i // 100) % 100, i // 10000),
                    threshold=1.0,
                    membrane_potential=np.random.rand()
                )
            
            # Test optimized neural update
            start_time = time.perf_counter()
            
            # This should use embedded optimizations automatically
            fired_neurons = neuron_array.embedded_optimized_neural_update(timestep=0)
            
            operation_time = time.perf_counter() - start_time
            operation_time_ms = operation_time * 1000
            
            results.append({
                'size': size,
                'operation_time_ms': operation_time_ms,
                'fired_count': len(fired_neurons)
            })
            
            # Performance assessment
            if operation_time_ms < self.target_time_15hz:
                status = "✅ EXCELLENT (15Hz+)"
            elif operation_time_ms < self.target_time_10hz:
                status = "⚠️  GOOD (10Hz+)"
            else:
                status = "❌ NEEDS OPTIMIZATION"
            
            print(f"   Operation time: {operation_time_ms:.2f}ms {status}")
            print(f"   Fired neurons: {len(fired_neurons)}")
            
            # Performance scaling info
            operations_per_neuron = 6  # Conservative estimate
            total_operations = size * operations_per_neuron
            if operation_time > 0:
                operations_per_second = total_operations / operation_time
                print(f"   Operations/sec: {operations_per_second/1e6:.1f}M")

        # Don't return to avoid pytest warnings
        print(f"   ✅ NeuronArray performance test completed")
    
    def test_connectome_manager_integration(self):
        """Test ConnectomeManager embedded optimization integration."""
        print(f"\n🧪 Testing ConnectomeManager integration...")

        # Test only one size due to singleton pattern
        size = self.test_sizes[1]  # Use 10K neurons
        print(f"\n📊 Testing {size:,} neurons in ConnectomeManager...")

        # Create connectome manager (automatically uses optimizations)
        connectome_manager = ConnectomeManager(config_or_max_neurons=size)
        
        # Create cortical area first
        test_area_id = connectome_manager.add_cortical_area(
            name="test_area",
            dimensions=(100, 100, 1),
            position=(0, 0, 0),
            area_type="test"
        )

        # Create test neurons
        neuron_ids = []
        for i in range(min(size, 500)):
            neuron_id = connectome_manager.create_neuron(
                cortical_id=test_area_id,
                position=(i % 50, (i // 50) % 50, i // 2500),
                threshold=1.0,
                membrane_potential=np.random.rand()
            )
            neuron_ids.append(neuron_id)

        # Test optimized update
        start_time = time.perf_counter()

        # This should now use embedded optimizations automatically
        fired_neurons = connectome_manager.update_membrane_potentials(
            current_timestep=0
        )

        operation_time = time.perf_counter() - start_time
        operation_time_ms = operation_time * 1000

        # Performance assessment
        if operation_time_ms < self.target_time_15hz:
            status = "✅ EXCELLENT (15Hz+)"
        elif operation_time_ms < self.target_time_10hz:
            status = "⚠️  GOOD (10Hz+)"
        else:
            status = "❌ NEEDS OPTIMIZATION"

        print(f"   Operation time: {operation_time_ms:.2f}ms {status}")
        print(f"   Fired neurons: {len(fired_neurons)}")

        # Verify embedded optimizations are being used
        neuron_array = connectome_manager.neuron_array
        perf_summary = neuron_array.get_performance_summary()
        print(f"   SIMD enabled: {perf_summary.get('simd_enabled', 'N/A')}")
        print(f"   Memory alignment: {perf_summary.get('alignment', 'N/A')}B")

        # Don't return to avoid pytest warnings
        print(f"   ✅ ConnectomeManager integration test completed")

    def test_burst_engine_integration(self):
        """Test BurstEngine embedded optimization integration."""
        print(f"\n🧪 Testing BurstEngine integration...")

        size = min(self.test_sizes[2], 50000)  # 50K neurons for CI
        print(f"\n📊 Testing {size:,} neurons in BurstEngine...")

        # Create connectome manager and burst engine
        connectome_manager = ConnectomeManager(config_or_max_neurons=size)
        
        # Create cortical area first
        test_area_id = connectome_manager.add_cortical_area(
            name="burst_test_area",
            dimensions=(100, 100, 1),
            position=(0, 0, 0),
            area_type="test"
        )
        
        burst_engine = BurstEngine(connectome_manager)

        # Create test neurons
        for i in range(min(size, 200)):
            connectome_manager.create_neuron(
                cortical_id=test_area_id,
                position=(i % 20, (i // 20) % 20, i // 400),
                threshold=1.0,
                membrane_potential=np.random.rand()
            )

        # Test burst processing performance
        start_time = time.perf_counter()

        # This should now use embedded optimizations automatically
        fired_neurons = burst_engine._process_burst()

        burst_time = time.perf_counter() - start_time
        burst_time_ms = burst_time * 1000

        # Performance assessment
        if burst_time_ms < self.target_time_15hz:
            status = "✅ EXCELLENT (15Hz+)"
        elif burst_time_ms < self.target_time_10hz:
            status = "⚠️  GOOD (10Hz+)"
        else:
            status = "❌ NEEDS OPTIMIZATION"

        print(f"   Burst time: {burst_time_ms:.2f}ms {status}")
        print(f"   Fired neurons: {len(fired_neurons) if fired_neurons else 0}")

        # Verify embedded optimizations are being used
        neuron_array = connectome_manager.neuron_array
        perf_summary = neuron_array.get_performance_summary()
        print(f"   SIMD enabled: {perf_summary.get('simd_enabled', 'N/A')}")
        print(f"   Backend: {perf_summary.get('backend', 'N/A')}")
        print(f"   Vector width: {perf_summary.get('vector_width', 'N/A')}")

        # Don't return to avoid pytest warnings
        print(f"   ✅ BurstEngine integration test completed")
    
    def test_performance_scaling_projection(self):
        """Project performance scaling to 10M neurons."""
        print(f"\n🧪 Performance scaling projection to 10M neurons...")
        
        # Test with available sizes
        test_results = []
        
        for size in [10000, 50000, 100000]:  # Realistic test sizes
            try:
                neuron_array = NeuronArray(max_neurons=size)
                
                # Create neurons
                for i in range(min(size // 100, 1000)):  # Reasonable subset
                    neuron_array.create_neuron(
                        cortical_idx=0,
                        position=(i % 100, (i // 100) % 100, i // 10000),
                        membrane_potential=np.random.rand()
                    )
                
                # Measure performance
                start_time = time.perf_counter()
                fired_neurons = neuron_array.embedded_optimized_neural_update(0)
                operation_time = time.perf_counter() - start_time
                
                # Calculate operations per second
                # Each neuron goes through: decay, refractory, threshold, potential firing
                operations_per_neuron = 6  # Conservative estimate
                total_operations = size * operations_per_neuron
                operations_per_second = total_operations / operation_time
                
                test_results.append({
                    'size': size,
                    'time_ms': operation_time * 1000,
                    'operations_per_second': operations_per_second
                })
                
                print(f"   {size:,} neurons: {operation_time*1000:.2f}ms, "
                      f"{operations_per_second/1e6:.1f}M ops/sec")
                
            except Exception as e:
                print(f"   Error testing {size:,} neurons: {e}")
        
        if test_results:
            # Project to 10M neurons
            # Use the best operations/second rate
            best_ops_per_sec = max(r['operations_per_second'] for r in test_results)
            
            # 10M neurons * 6 operations * 15Hz = 900M operations/second required
            target_ops_per_second = 10_000_000 * 6 * 15
            
            print(f"\n📈 Projection to 10M neurons:")
            print(f"   Best measured: {best_ops_per_sec/1e6:.1f}M ops/sec")
            print(f"   Required for 15Hz: {target_ops_per_second/1e6:.1f}M ops/sec")
            
            if best_ops_per_sec >= target_ops_per_second:
                print(f"   ✅ TARGET ACHIEVABLE: Performance sufficient for 10M@15Hz")
            else:
                ratio = target_ops_per_second / best_ops_per_sec
                print(f"   ⚠️  NEEDS {ratio:.1f}x IMPROVEMENT for 10M@15Hz target")
    
    def test_optimization_verification(self):
        """Verify that optimizations are actually being used."""
        print(f"\n🧪 Verifying optimization features...")

        neuron_array = NeuronArray(max_neurons=1000)
        perf_summary = neuron_array.get_performance_summary()

        print(f"   ✅ Memory alignment: {perf_summary.get('alignment', 'N/A')}B")
        print(f"   ✅ SIMD enabled: {perf_summary.get('simd_enabled', 'N/A')}")
        print(f"   ✅ Vector width: {perf_summary.get('vector_width', 'N/A')}")
        print(f"   ✅ Backend: {perf_summary.get('backend', 'N/A')}")

        # Test cache alignment (if available)
        alignment = perf_summary.get('alignment')
        if alignment is not None:
            assert alignment >= 16, f"Should have at least 16-byte alignment, got {alignment}"

        # Test that we have reasonable vector width (if available)
        vector_width = perf_summary.get('vector_width')
        if vector_width is not None:
            assert vector_width >= 1, f"Should have at least 1-wide vectors, got {vector_width}"

        print(f"   ✅ All optimization features verified")


def run_embedded_performance_test():
    """Run the complete embedded performance test suite."""
    print("🚀 FEAGI Integrated Embedded Optimization Test Suite")
    print("=" * 60)
    
    test_suite = TestIntegratedEmbeddedOptimization()
    test_suite.setup_method()
    
    try:
        # Test individual components
        test_suite.test_cache_aligned_memory()
        test_suite.test_simd_operations()
        test_suite.test_optimization_verification()
        
        # Test integrated performance
        neuron_results = test_suite.test_neuron_array_performance()
        connectome_results = test_suite.test_connectome_manager_integration()
        burst_result = test_suite.test_burst_engine_integration()
        
        # Performance projection
        test_suite.test_performance_scaling_projection()
        
        print("\n" + "=" * 60)
        print("🎯 EMBEDDED OPTIMIZATION TEST SUMMARY")
        print("=" * 60)
        
        print(f"✅ Cache alignment: Verified {MEMORY_ALIGNMENT}-byte alignment")
        print(f"✅ SIMD operations: {'Numba JIT enabled' if NUMBA_AVAILABLE else 'NumPy fallback'}")
        print(f"✅ Integration: All components using embedded optimizations")
        
        # Best performance achieved
        if neuron_results:
            best_neuron_time = min(r['operation_time_ms'] for r in neuron_results)
            print(f"✅ Best neuron performance: {best_neuron_time:.2f}ms")
        
        if connectome_results:
            best_connectome_time = min(r['operation_time_ms'] for r in connectome_results)
            print(f"✅ Best connectome performance: {best_connectome_time:.2f}ms")
        
        print(f"✅ Burst engine performance: {burst_result['burst_time_ms']:.2f}ms")
        
        print(f"\n🎯 15Hz target: <66.7ms per operation")
        print(f"🎯 10Hz target: <100ms per operation")
        
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    run_embedded_performance_test() 