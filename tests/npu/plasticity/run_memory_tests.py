#!/usr/bin/env python3
"""
FEAGI Memory System Test Runner

Comprehensive test runner for all memory system components.
Runs all tests and provides detailed reporting.

Usage:
    python run_memory_tests.py [--verbose] [--performance] [--integration-only]

Version: 3.0
"""

import sys
import time
import argparse
from pathlib import Path

# Add the project root to Python path
project_root = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(project_root))

def run_test_suite(test_name: str, test_class, verbose: bool = False):
    """Run a test suite and report results."""
    
    print(f"\n{'='*60}")
    print(f"🧪 Running {test_name}")
    print(f"{'='*60}")
    
    start_time = time.time()
    
    try:
        # Create test instance
        test_instance = test_class()
        
        # Get all test methods
        test_methods = [method for method in dir(test_instance) 
                       if method.startswith('test_') and callable(getattr(test_instance, method))]
        
        print(f"Found {len(test_methods)} test methods")
        
        passed_tests = 0
        failed_tests = 0
        
        for method_name in test_methods:
            if verbose:
                print(f"\n  🔍 Running {method_name}...")
            
            try:
                # Get method and required fixtures
                method = getattr(test_instance, method_name)
                
                # Create fixtures based on method signature
                fixtures = {}
                if hasattr(test_instance, 'fire_ledger'):
                    fixtures['fire_ledger'] = test_instance.fire_ledger()
                if hasattr(test_instance, 'pattern_config'):
                    fixtures['pattern_config'] = test_instance.pattern_config()
                if hasattr(test_instance, 'memory_neuron_array'):
                    fixtures['memory_neuron_array'] = test_instance.memory_neuron_array()
                if hasattr(test_instance, 'neuron_id_manager'):
                    fixtures['neuron_id_manager'] = test_instance.neuron_id_manager()
                if hasattr(test_instance, 'plasticity_config'):
                    fixtures['plasticity_config'] = test_instance.plasticity_config()
                if hasattr(test_instance, 'npu_interface'):
                    fixtures['npu_interface'] = test_instance.npu_interface()
                if hasattr(test_instance, 'state_manager'):
                    fixtures['state_manager'] = test_instance.state_manager()
                
                # Call method with appropriate fixtures
                import inspect
                sig = inspect.signature(method)
                method_fixtures = {}
                
                for param_name in sig.parameters:
                    if param_name in fixtures:
                        method_fixtures[param_name] = fixtures[param_name]
                
                method(**method_fixtures)
                
                passed_tests += 1
                if verbose:
                    print(f"    ✅ {method_name} PASSED")
                else:
                    print(".", end="", flush=True)
                    
            except Exception as e:
                failed_tests += 1
                if verbose:
                    print(f"    ❌ {method_name} FAILED: {e}")
                else:
                    print("F", end="", flush=True)
        
        if not verbose:
            print()  # New line after dots
        
        end_time = time.time()
        duration = end_time - start_time
        
        print(f"\n📊 {test_name} Results:")
        print(f"   ✅ Passed: {passed_tests}")
        print(f"   ❌ Failed: {failed_tests}")
        print(f"   ⏱️  Duration: {duration:.2f}s")
        
        if failed_tests == 0:
            print(f"   🎉 All {test_name} tests PASSED!")
            return True
        else:
            print(f"   💥 {failed_tests} {test_name} tests FAILED!")
            return False
            
    except Exception as e:
        print(f"❌ Failed to run {test_name}: {e}")
        return False


def run_performance_tests():
    """Run performance-specific tests."""
    
    print(f"\n{'='*60}")
    print("⚡ Running Performance Tests")
    print(f"{'='*60}")
    
    try:
        from test_pattern_detector import TestPatternDetector
        
        test_instance = TestPatternDetector()
        pattern_config = test_instance.pattern_config()
        
        print("🔍 Testing pattern detection performance...")
        test_instance.test_performance_characteristics(pattern_config)
        
        # Additional performance tests
        print("🧠 Testing memory system scalability...")
        
        # Test with large numbers of memory areas
        from feagi.npu.plasticity.pattern_detector import BatchPatternDetector
        from feagi.npu.fire_ledger import FireLedgerInterface
        
        ledger = FireLedgerInterface(default_window_size=20)
        batch_detector = BatchPatternDetector(pattern_config)
        
        # Create 100 memory areas
        memory_areas = {}
        for i in range(100):
            memory_areas[i] = {
                'temporal_depth': 3 + (i % 5),  # Vary temporal depth
                'upstream_areas': [i * 10, i * 10 + 1, i * 10 + 2]
            }
        
        # Create activity patterns
        for timestep in range(10):
            neurons_by_area = {}
            for area_base in range(0, 1000, 10):  # 100 areas
                neurons_by_area[area_base] = [area_base + timestep % 5]
                neurons_by_area[area_base + 1] = [area_base + 1 + timestep % 3]
                neurons_by_area[area_base + 2] = [area_base + 2 + timestep % 4]
            
            ledger.archive_timestep(timestep, neurons_by_area)
        
        # Measure batch detection performance
        start_time = time.time()
        patterns = batch_detector.detect_patterns_batch(ledger, memory_areas, 9)
        end_time = time.time()
        
        duration = end_time - start_time
        patterns_detected = len([p for p in patterns.values() if p is not None])
        
        print(f"   📈 Batch detection: {patterns_detected} patterns in {duration*1000:.2f}ms")
        print(f"   📈 Rate: {patterns_detected/duration:.0f} patterns/second")
        
        if duration < 1.0:  # Should complete in under 1 second
            print("   ✅ Performance test PASSED")
            return True
        else:
            print("   ❌ Performance test FAILED (too slow)")
            return False
            
    except Exception as e:
        print(f"❌ Performance test failed: {e}")
        return False


def run_integration_tests():
    """Run integration tests that test the complete system."""
    
    print(f"\n{'='*60}")
    print("🔗 Running Integration Tests")
    print(f"{'='*60}")
    
    try:
        from test_memory_system import TestMemorySystemIntegration
        
        test_instance = TestMemorySystemIntegration()
        
        # Create all fixtures
        fire_ledger = test_instance.fire_ledger()
        pattern_config = test_instance.pattern_config()
        memory_neuron_array = test_instance.memory_neuron_array()
        plasticity_config = test_instance.plasticity_config()
        
        print("🧠 Testing end-to-end memory formation...")
        test_instance.test_end_to_end_memory_formation(
            fire_ledger, pattern_config, memory_neuron_array, plasticity_config
        )
        
        print("🎯 Testing per-area temporal depth...")
        test_instance.test_per_area_temporal_depth(fire_ledger, pattern_config)
        
        print("🔄 Testing memory neuron lifecycle...")
        test_instance.test_memory_neuron_lifecycle(memory_neuron_array)
        
        print("🆔 Testing neuron ID allocation...")
        neuron_id_manager = test_instance.neuron_id_manager()
        test_instance.test_neuron_id_allocation(neuron_id_manager)
        
        print("🔒 Testing pattern detection determinism...")
        test_instance.test_pattern_detection_determinism(fire_ledger, pattern_config)
        
        print("📊 Testing memory statistics...")
        test_instance.test_memory_statistics(memory_neuron_array)
        
        print("   🎉 All integration tests PASSED!")
        return True
        
    except Exception as e:
        print(f"❌ Integration test failed: {e}")
        return False


def main():
    """Main test runner."""
    
    parser = argparse.ArgumentParser(description="FEAGI Memory System Test Runner")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    parser.add_argument("--performance", "-p", action="store_true", help="Run performance tests")
    parser.add_argument("--integration-only", "-i", action="store_true", help="Run only integration tests")
    
    args = parser.parse_args()
    
    print("🧠 FEAGI Memory System Test Suite")
    print("=" * 60)
    print("Testing comprehensive memory formation capabilities:")
    print("  • Fire Ledger integration")
    print("  • Per-area temporal depth")
    print("  • RoaringBitmap pattern detection")
    print("  • Memory neuron lifecycle")
    print("  • PlasticityService threading")
    print("  • Global unique ID allocation")
    print("  • End-to-end workflows")
    print("=" * 60)
    
    start_time = time.time()
    all_passed = True
    
    if args.integration_only:
        # Run only integration tests
        all_passed = run_integration_tests()
    else:
        # Run individual component tests
        try:
            from test_pattern_detector import TestPatternDetector
            from test_plasticity_service import TestPlasticityService
            from test_memory_system import TestMemorySystemIntegration
            
            # Run all test suites
            test_suites = [
                ("Pattern Detection", TestPatternDetector),
                ("PlasticityService", TestPlasticityService),
                ("Memory System Integration", TestMemorySystemIntegration),
            ]
            
            for test_name, test_class in test_suites:
                success = run_test_suite(test_name, test_class, args.verbose)
                if not success:
                    all_passed = False
            
            # Run performance tests if requested
            if args.performance:
                success = run_performance_tests()
                if not success:
                    all_passed = False
            
            # Always run integration tests at the end
            success = run_integration_tests()
            if not success:
                all_passed = False
                
        except ImportError as e:
            print(f"❌ Failed to import test modules: {e}")
            print("Make sure all test files are in the same directory.")
            return 1
    
    end_time = time.time()
    total_duration = end_time - start_time
    
    print(f"\n{'='*60}")
    print("📋 FINAL RESULTS")
    print(f"{'='*60}")
    print(f"⏱️  Total Duration: {total_duration:.2f}s")
    
    if all_passed:
        print("🎉 ALL MEMORY SYSTEM TESTS PASSED!")
        print("✅ Memory formation system is working correctly")
        print("✅ Ready for production use")
        return 0
    else:
        print("💥 SOME TESTS FAILED!")
        print("❌ Memory system needs attention")
        print("❌ Check failed tests above")
        return 1


if __name__ == "__main__":
    sys.exit(main())
