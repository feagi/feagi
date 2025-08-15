#!/usr/bin/env python3
"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Comprehensive FCL Performance Benchmark Runner

This script orchestrates all FCL performance benchmarks and provides
a unified interface for performance testing, baseline creation, and
regression detection.

Usage:
    python tests/performance/run_fcl_benchmarks.py [options]

Options:
    --create-baseline    Create new performance baselines
    --check-regression   Check for performance regressions
    --full-suite        Run complete benchmark suite
    --quick             Run quick performance check
    --profile           Include detailed profiling
    --save-results      Save results to files
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Any

# Add project root to path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from tests.performance.npu.test_fcl_performance import FCLPerformanceBenchmark
from tests.performance.npu.test_burst_frequency_robustness import BurstFrequencyRobustnessTest
from tests.performance.npu.test_stress_limits import StressTestSuite
from tests.performance.npu.test_neural_propagation_benchmark import NeuralPropagationBenchmark
from tests.performance.system.test_fcl_system_performance import FCLSystemPerformanceBenchmark
from tests.performance.utils.performance_regression import (
    PerformanceRegressionDetector,
    create_performance_baseline,
    detect_performance_regressions
)
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class FCLBenchmarkRunner:
    """Comprehensive FCL benchmark runner and coordinator."""
    
    def __init__(self):
        """Initialize the benchmark runner."""
        self.logger = setup_logger(__name__)
        self.results_dir = Path("tests/performance/logs")
        self.results_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize benchmark suites
        self.fcl_benchmark = FCLPerformanceBenchmark()
        self.frequency_robustness = BurstFrequencyRobustnessTest()
        self.stress_tester = StressTestSuite()
        self.neural_propagation = NeuralPropagationBenchmark()
        self.system_benchmark = FCLSystemPerformanceBenchmark()
        self.regression_detector = PerformanceRegressionDetector()
        
        self.logger.info("FCL Benchmark Runner initialized")
    
    def run_quick_benchmark(self) -> Dict[str, Any]:
        """Run a quick performance check."""
        self.logger.info("Running quick FCL performance check...")
        
        print("🚀 FEAGI FCL Quick Performance Check")
        print("=" * 50)
        
        # Run basic FCL manager benchmarks
        fcl_result = self.fcl_benchmark.benchmark_fcl_manager_operations()
        
        # Extract key metrics
        if fcl_result.metrics:
            avg_time = fcl_result.summary_stats.get('avg_execution_time_ms', 0)
            max_throughput = fcl_result.summary_stats.get('max_throughput_neurons_per_sec', 0)
            
            print(f"\n📊 Quick Results:")
            print(f"   Average FCL Update Time: {avg_time:.3f} ms")
            print(f"   Maximum Throughput: {max_throughput:,.0f} neurons/sec")
            
            # Simple performance assessment
            if avg_time < 1.0:
                print("   ✅ FCL performance: EXCELLENT")
            elif avg_time < 5.0:
                print("   ✅ FCL performance: GOOD")
            elif avg_time < 20.0:
                print("   ⚠️  FCL performance: ACCEPTABLE")
            else:
                print("   ❌ FCL performance: NEEDS OPTIMIZATION")
        
        return {
            'test_type': 'quick',
            'fcl_result': fcl_result.to_dict(),
            'timestamp': time.time()
        }
    
    def run_full_benchmark_suite(self, include_profiling: bool = False, include_frequency_test: bool = True) -> Dict[str, Any]:
        """Run the complete FCL benchmark suite."""
        self.logger.info("Running complete FCL benchmark suite...")
        
        print("🚀 FEAGI FCL Comprehensive Performance Benchmark Suite")
        print("=" * 60)
        
        results = {}
        
        # 1. FCL Component Benchmarks
        print("\n1️⃣ Running FCL Component Benchmarks...")
        fcl_results = []
        
        fcl_results.append(self.fcl_benchmark.benchmark_fcl_manager_operations())
        fcl_results.append(self.fcl_benchmark.benchmark_async_fcl_processor())
        fcl_results.append(self.fcl_benchmark.benchmark_rust_optimized_fcl())
        fcl_results.append(self.fcl_benchmark.benchmark_integration_performance())
        
        # 2. System-Level Benchmarks
        print("\n2️⃣ Running System-Level Benchmarks...")
        system_result = self.system_benchmark.benchmark_end_to_end_burst_processing()
        
        # 3. Frequency Robustness Testing (if requested)
        frequency_results = None
        if include_frequency_test:
            print("\n3️⃣ Running Frequency Robustness Tests...")
            frequency_metrics = self.frequency_robustness.test_frequency_robustness_across_scales(15.0)
            breakdown_points = self.frequency_robustness.identify_frequency_breakdown_points(frequency_metrics)
            self.frequency_robustness.print_frequency_robustness_report(frequency_metrics, breakdown_points, 15.0)
            
            frequency_results = {
                'metrics': [m.to_dict() for m in frequency_metrics],
                'breakdown_points': [bp.__dict__ for bp in breakdown_points],
                'target_frequency': 15.0
            }
        
        # 4. Profiling (if requested)
        profile_data = None
        if include_profiling:
            print("\n4️⃣ Running Detailed Profiling...")
            profile_data = self.fcl_benchmark.profile_fcl_operations()
        
        # Compile results
        results = {
            'test_type': 'full_suite',
            'fcl_component_results': [r.to_dict() for r in fcl_results],
            'system_result': {
                'test_name': system_result.test_name,
                'metrics': [m.__dict__ for m in system_result.metrics],
                'performance_targets': system_result.performance_targets,
                'target_analysis': system_result.target_analysis,
                'bottleneck_analysis': system_result.bottleneck_analysis,
                'recommendations': system_result.recommendations,
                'timestamp': system_result.timestamp
            },
            'frequency_robustness_results': frequency_results,
            'profile_data': profile_data,
            'timestamp': time.time()
        }
        
        # Print comprehensive report
        self._print_comprehensive_report(fcl_results, system_result)
        
        return results
    
    def run_frequency_robustness_test(self, target_frequency: float = 15.0) -> Dict[str, Any]:
        """Run dedicated frequency robustness test."""
        self.logger.info(f"Running frequency robustness test at {target_frequency}Hz...")
        
        print(f"🎯 FEAGI Burst Frequency Robustness Test - {target_frequency}Hz")
        print("=" * 60)
        
        # Run frequency robustness test
        frequency_metrics = self.frequency_robustness.test_frequency_robustness_across_scales(target_frequency)
        breakdown_points = self.frequency_robustness.identify_frequency_breakdown_points(frequency_metrics)
        
        # Print report
        self.frequency_robustness.print_frequency_robustness_report(frequency_metrics, breakdown_points, target_frequency)
        
        # Save results
        results_file = self.frequency_robustness.save_results(frequency_metrics, breakdown_points, target_frequency)
        
        return {
            'test_type': 'frequency_robustness',
            'target_frequency': target_frequency,
            'metrics': [m.to_dict() for m in frequency_metrics],
            'breakdown_points': [bp.__dict__ for bp in breakdown_points],
            'results_file': results_file,
            'timestamp': time.time()
        }
    
    def run_stress_test_suite(self) -> Dict[str, Any]:
        """Run comprehensive stress testing to identify bottlenecks."""
        self.logger.info("Running stress test suite to push system to limits...")
        
        print("🔥 FEAGI STRESS TEST SUITE - FINDING THE BREAKING POINTS")
        print("=" * 60)
        
        # Run complete stress test suite
        results_file = self.stress_tester.run_complete_stress_test_suite()
        
        return {
            'test_type': 'stress_test_suite',
            'results_file': results_file,
            'timestamp': time.time()
        }
    
    def run_neural_propagation_benchmark(self) -> Dict[str, Any]:
        """Run comprehensive neural propagation benchmark with realistic synaptic connectivity."""
        self.logger.info("Running neural propagation benchmark with real synaptic connections...")
        
        print("🧠 FEAGI NEURAL PROPAGATION BENCHMARK - REALISTIC NEURAL COMPUTATION")
        print("=" * 70)
        print("Testing 3-area neural network (A→B→C→A) with block_to_block connectivity")
        print("Systematic parameter sweeps: M dimension, consecutive fires, timestep")
        print("Extreme scenarios: M=2000, timestep=5ms")
        print()
        
        # Run comprehensive neural propagation benchmark
        metrics = self.neural_propagation.run_comprehensive_benchmark()
        
        # Save results
        results_file = self.neural_propagation.save_benchmark_results(metrics)
        
        return {
            'test_type': 'neural_propagation_benchmark',
            'total_scenarios': len(metrics),
            'successful_scenarios': len([m for m in metrics if not m.scenario_name.endswith('_FAILED')]),
            'failed_scenarios': len([m for m in metrics if m.scenario_name.endswith('_FAILED')]),
            'results_file': results_file,
            'timestamp': time.time()
        }
    
    def run_cpu_vs_gpu_comparison(self, M: int, test_duration_sec: float, timestep_sec: float) -> Dict[str, Any]:
        """Run CPU vs GPU performance comparison."""
        
        # Calculate burst count from test duration using the formula:
        # consecutive_fire_count = test_duration / simulation_timestep
        consecutive_fires = max(1, int(test_duration_sec / timestep_sec))
        self.logger.info(f"Calculated burst count: {test_duration_sec}s ÷ {timestep_sec}s = {consecutive_fires} bursts")
        
        # Convert timestep to milliseconds for internal use (FEAGI expects ms)
        timestep_ms = timestep_sec * 1000.0
        
        self.logger.info(f"Running CPU vs GPU comparison: M={M}, duration={test_duration_sec}s, timestep={timestep_sec}s...")
        
        print("🏆 FEAGI CPU vs GPU PERFORMANCE COMPARISON")
        print("=" * 70)
        print(f"Comparing brain development and neural computation performance")
        print(f"Parameters: M={M}, test_duration={test_duration_sec}s, timestep={timestep_sec}s")
        print(f"Calculated Bursts: {consecutive_fires} bursts ({consecutive_fires * timestep_sec:.1f}s)")
        print()
        
        # Run CPU vs GPU comparison
        results = self.neural_propagation.run_cpu_vs_gpu_comparison(M, consecutive_fires, timestep_ms)
        
        # Save results
        results_file = self.neural_propagation.save_benchmark_results(results)
        
        # Create detailed results summary
        detailed_results = {
            'test_type': 'cpu_vs_gpu_comparison',
            'test_parameters': {
                'cortical_dimensions': f"{M}×{M}×1",
                'M': M,
                'total_neurons': 3 * M * M,
                'test_duration_sec': test_duration_sec,
                'simulation_timestep_sec': timestep_sec,
                'simulation_timestep_ms': timestep_ms,  # Keep for compatibility
                'calculated_burst_count': consecutive_fires,
                'target_frequency_hz': 1.0 / timestep_sec,
                'actual_test_duration_sec': consecutive_fires * timestep_sec,
                'backends_tested': ['CPU (PyTorch)', 'GPU (WGPU)']
            },
            'results': {
                'cpu_result': results[0].to_dict() if len(results) > 0 else None,
                'gpu_result': results[1].to_dict() if len(results) > 1 else None
            },
            'comparison_summary': {},
            'results_file': results_file,
            'timestamp': time.time()
        }
        
        # Add comparison summary if both tests succeeded
        if len(results) >= 2 and results[0].total_test_time_ms > 0 and results[1].total_test_time_ms > 0:
            cpu_result = results[0]
            gpu_result = results[1]
            
            detailed_results['comparison_summary'] = {
                'brain_development_speedup': cpu_result.brain_dev_time_ms / gpu_result.brain_dev_time_ms if gpu_result.brain_dev_time_ms > 0 else 0,
                'neural_computation_speedup': cpu_result.neural_comp_time_ms / gpu_result.neural_comp_time_ms if gpu_result.neural_comp_time_ms > 0 else 0,
                'total_speedup': cpu_result.total_test_time_ms / gpu_result.total_test_time_ms if gpu_result.total_test_time_ms > 0 else 0,
                'throughput_ratio': gpu_result.neurons_processed_per_sec / cpu_result.neurons_processed_per_sec if cpu_result.neurons_processed_per_sec > 0 else 0,
                'winner': 'CPU' if cpu_result.total_test_time_ms < gpu_result.total_test_time_ms else 'GPU'
            }
        
        return detailed_results
    
    def create_performance_baselines(self) -> Dict[str, Any]:
        """Create performance baselines for regression testing."""
        self.logger.info("Creating performance baselines...")
        
        print("📊 Creating FCL Performance Baselines")
        print("=" * 40)
        
        # Run benchmarks to establish baselines
        results = self.run_full_benchmark_suite(include_profiling=False)
        
        baselines_created = {}
        
        # Create baselines for FCL component tests
        for fcl_result_dict in results['fcl_component_results']:
            test_name = fcl_result_dict['test_name']
            metrics = fcl_result_dict['metrics']
            
            if metrics:
                baselines = create_performance_baseline(
                    test_name, 
                    metrics,
                    {'creation_timestamp': time.time()}
                )
                baselines_created[test_name] = len(baselines)
                print(f"   ✅ Created {len(baselines)} baselines for {test_name}")
        
        # Create baselines for system tests
        system_metrics = results['system_result']['metrics']
        if system_metrics:
            baselines = create_performance_baseline(
                'system_performance',
                system_metrics,
                {'creation_timestamp': time.time()}
            )
            baselines_created['system_performance'] = len(baselines)
            print(f"   ✅ Created {len(baselines)} baselines for system_performance")
        
        print(f"\n📈 Total baselines created: {sum(baselines_created.values())}")
        
        return {
            'test_type': 'baseline_creation',
            'baselines_created': baselines_created,
            'benchmark_results': results,
            'timestamp': time.time()
        }
    
    def check_performance_regressions(self) -> Dict[str, Any]:
        """Check for performance regressions against baselines."""
        self.logger.info("Checking for performance regressions...")
        
        print("🔍 FCL Performance Regression Analysis")
        print("=" * 40)
        
        # Run current benchmarks
        current_results = self.run_full_benchmark_suite(include_profiling=False)
        
        regression_analyses = []
        
        # Check FCL component regressions
        for fcl_result_dict in current_results['fcl_component_results']:
            test_name = fcl_result_dict['test_name']
            metrics = fcl_result_dict['metrics']
            
            if metrics:
                analysis = detect_performance_regressions(test_name, metrics)
                regression_analyses.append(analysis)
                
                # Print regression report
                self.regression_detector.print_regression_report(analysis)
        
        # Check system performance regressions
        system_metrics = current_results['system_result']['metrics']
        if system_metrics:
            analysis = detect_performance_regressions('system_performance', system_metrics)
            regression_analyses.append(analysis)
            self.regression_detector.print_regression_report(analysis)
        
        # Summary
        total_regressions = sum(a.regressions_detected for a in regression_analyses)
        total_improvements = sum(a.improvements_detected for a in regression_analyses)
        
        print(f"\n📊 Regression Analysis Summary:")
        print(f"   Total Regressions: {total_regressions}")
        print(f"   Total Improvements: {total_improvements}")
        
        if total_regressions == 0:
            print("   ✅ No performance regressions detected!")
        else:
            print(f"   ⚠️  {total_regressions} performance regressions detected")
        
        return {
            'test_type': 'regression_check',
            'regression_analyses': [a.__dict__ for a in regression_analyses],
            'total_regressions': total_regressions,
            'total_improvements': total_improvements,
            'current_results': current_results,
            'timestamp': time.time()
        }
    
    def _print_comprehensive_report(self, fcl_results, system_result):
        """Print comprehensive benchmark report."""
        print("\n" + "="*80)
        print("📊 COMPREHENSIVE FCL PERFORMANCE REPORT")
        print("="*80)
        
        # FCL Component Results Summary
        print(f"\n🔧 FCL COMPONENT PERFORMANCE:")
        print("-" * 50)
        
        for result in fcl_results:
            if not result.metrics:
                continue
            
            print(f"\n📈 {result.test_name.upper()}:")
            for key, value in result.summary_stats.items():
                if 'time' in key:
                    print(f"   {key}: {value:.3f} ms")
                elif 'throughput' in key:
                    print(f"   {key}: {value:,.0f} neurons/sec")
        
        # System Performance Summary
        print(f"\n🖥️  SYSTEM PERFORMANCE:")
        print("-" * 30)
        
        if system_result.metrics:
            best_scenario = min(system_result.metrics, key=lambda m: m.burst_processing_time_ms)
            print(f"   Best Burst Time: {best_scenario.burst_processing_time_ms:.3f} ms")
            print(f"   Best Throughput: {best_scenario.throughput_neurons_per_sec:,.0f} neurons/sec")
            print(f"   Best Frequency: {best_scenario.burst_frequency_hz:.1f} Hz")
        
        # Performance Targets Analysis
        print(f"\n🎯 PERFORMANCE TARGETS:")
        print("-" * 30)
        
        for key, passed in system_result.target_analysis.items():
            status = "✅" if passed else "❌"
            print(f"   {status} {key}")
        
        # Recommendations
        if system_result.recommendations:
            print(f"\n💡 OPTIMIZATION RECOMMENDATIONS:")
            print("-" * 40)
            for i, rec in enumerate(system_result.recommendations, 1):
                print(f"   {i}. {rec}")
        
        print("\n" + "="*80)
    
    def save_results(self, results: Dict[str, Any]) -> str:
        """Save benchmark results to file."""
        timestamp = int(time.time())
        test_type = results.get('test_type', 'benchmark')
        filename = f"fcl_benchmark_{test_type}_{timestamp}.json"
        filepath = self.results_dir / filename
        
        with open(filepath, 'w') as f:
            json.dump(results, f, indent=2, default=str)
        
        self.logger.info(f"Saved benchmark results to {filepath}")
        return str(filepath)


def main():
    """Main entry point for FCL benchmark runner."""
    parser = argparse.ArgumentParser(description="FEAGI FCL Performance Benchmark Runner")
    
    parser.add_argument('--create-baseline', action='store_true',
                       help='Create new performance baselines')
    parser.add_argument('--check-regression', action='store_true',
                       help='Check for performance regressions')
    parser.add_argument('--full-suite', action='store_true',
                       help='Run complete benchmark suite')
    parser.add_argument('--quick', action='store_true',
                       help='Run quick performance check')
    parser.add_argument('--frequency-robustness', action='store_true',
                       help='Run burst frequency robustness test')
    parser.add_argument('--target-frequency', type=float, default=15.0,
                       help='Target frequency for robustness test (default: 15.0 Hz)')
    parser.add_argument('--stress-test', action='store_true',
                       help='Run comprehensive stress testing to find system limits')
    parser.add_argument('--neural-propagation', action='store_true',
                       help='Run neural propagation benchmark with realistic synaptic connectivity')
    parser.add_argument('--cpu-vs-gpu', action='store_true',
                       help='Run CPU vs GPU performance comparison')
    parser.add_argument('--M', type=int, default=100,
                       help='Cortical area dimension for CPU vs GPU test (default: 100)')
    parser.add_argument('--test-duration', type=float, default=60.0,
                       help='Test duration in seconds (default: 60.0 = 1 minute)')
    parser.add_argument('--timestep', type=float, default=0.05,
                       help='Simulation timestep in seconds (default: 0.05 = 50ms)')
    parser.add_argument('--profile', action='store_true',
                       help='Include detailed profiling')
    parser.add_argument('--save-results', action='store_true',
                       help='Save results to files')
    
    args = parser.parse_args()
    
    # Default to quick benchmark if no options specified
    if not any([args.create_baseline, args.check_regression, args.full_suite, args.quick, args.frequency_robustness, args.stress_test, args.neural_propagation, args.cpu_vs_gpu]):
        args.quick = True
    
    runner = FCLBenchmarkRunner()
    results = None
    
    try:
        if args.create_baseline:
            results = runner.create_performance_baselines()
        elif args.check_regression:
            results = runner.check_performance_regressions()
        elif args.full_suite:
            results = runner.run_full_benchmark_suite(include_profiling=args.profile)
        elif args.frequency_robustness:
            results = runner.run_frequency_robustness_test(args.target_frequency)
        elif args.stress_test:
            results = runner.run_stress_test_suite()
        elif args.neural_propagation:
            results = runner.run_neural_propagation_benchmark()
        elif args.cpu_vs_gpu:
            results = runner.run_cpu_vs_gpu_comparison(args.M, args.test_duration, args.timestep)
        elif args.quick:
            results = runner.run_quick_benchmark()
        
        # Save results if requested
        if args.save_results and results:
            results_file = runner.save_results(results)
            print(f"\n💾 Results saved to: {results_file}")
        
        print(f"\n✅ FCL benchmark completed successfully!")
        
    except Exception as e:
        print(f"\n❌ Benchmark failed: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
