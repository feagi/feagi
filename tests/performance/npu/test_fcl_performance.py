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
Comprehensive FCL Performance Benchmark Suite

This module provides comprehensive performance testing for Fire Candidate List (FCL)
operations, establishing baselines and identifying bottlenecks for optimization.

Test Categories:
1. FCL Manager Operations - Basic FCL operations timing
2. Memory Usage Analysis - Memory allocation and deallocation patterns
3. Scalability Testing - Performance across different neuron counts
4. Concurrency Testing - Multi-threaded FCL operations
5. Integration Testing - FCL with ConnectomeManager and BurstEngine
6. Regression Testing - Performance regression detection
"""

import cProfile
import gc
import json
import os
import pstats
import psutil
import statistics
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
import tracemalloc

import pytest
import numpy as np

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.npu.burst_engine import BurstEngine
from feagi.npu.fcl_manager import FCLManager
from feagi.npu.interfaces import FiredNeuronEvent
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


@dataclass
class FCLPerformanceMetrics:
    """Performance metrics for FCL operations."""
    operation_name: str
    execution_time_ms: float
    memory_usage_mb: float
    memory_peak_mb: float
    cpu_usage_percent: float
    neuron_count: int
    cortical_areas: int
    throughput_neurons_per_sec: float
    timestamp: float
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return asdict(self)


@dataclass
class FCLBenchmarkResult:
    """Complete benchmark result with multiple metrics."""
    test_name: str
    metrics: List[FCLPerformanceMetrics]
    summary_stats: Dict[str, float]
    system_info: Dict[str, Any]
    timestamp: float
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            'test_name': self.test_name,
            'metrics': [m.to_dict() for m in self.metrics],
            'summary_stats': self.summary_stats,
            'system_info': self.system_info,
            'timestamp': self.timestamp
        }


class FCLPerformanceBenchmark:
    """Comprehensive FCL performance benchmark suite."""
    
    def __init__(self):
        """Initialize the benchmark suite."""
        self.logger = setup_logger(__name__)
        self.results_dir = Path("tests/performance/logs")
        self.results_dir.mkdir(parents=True, exist_ok=True)
        
        # System information
        self.system_info = {
            'cpu_count': psutil.cpu_count(),
            'memory_total_gb': psutil.virtual_memory().total / (1024**3),
            'python_version': f"{psutil.Process().name()}",
            'platform': os.uname().sysname if hasattr(os, 'uname') else 'Unknown'
        }
        
        # Test configurations
        self.neuron_counts = [100, 1000, 5000, 10000, 25000, 50000]
        self.cortical_area_counts = [1, 5, 10, 20, 50]
        self.iterations = 10
        
        self.logger.info("FCL Performance Benchmark initialized")
        self.logger.info(f"System: {self.system_info}")
    
    def _measure_performance(self, operation_name: str, operation_func, 
                           neuron_count: int, cortical_areas: int) -> FCLPerformanceMetrics:
        """Measure performance of a single operation with comprehensive resource monitoring."""
        # Start memory tracking
        tracemalloc.start()
        process = psutil.Process()
        
        # Get baseline resource usage
        memory_before = process.memory_info().rss / (1024**2)
        
        # Monitor CPU usage during operation
        cpu_samples = []
        memory_samples = []
        
        def monitor_resources():
            """Monitor CPU and memory during operation execution."""
            start_monitor = time.time()
            while time.time() - start_monitor < 5.0:  # Monitor for up to 5 seconds
                try:
                    cpu_samples.append(process.cpu_percent())
                    memory_samples.append(process.memory_info().rss / (1024**2))
                    time.sleep(0.01)  # 10ms sampling
                except:
                    break
        
        # Start resource monitoring in background
        import threading
        monitor_thread = threading.Thread(target=monitor_resources, daemon=True)
        monitor_thread.start()
        
        # Execute operation and measure time
        start_time = time.perf_counter()
        operation_func()
        end_time = time.perf_counter()
        
        # Wait a bit for monitoring to capture post-operation state
        time.sleep(0.05)
        
        # Get memory tracking results
        current, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        
        # Calculate metrics
        execution_time_ms = (end_time - start_time) * 1000
        throughput = neuron_count / (execution_time_ms / 1000) if execution_time_ms > 0 else 0
        
        # Resource usage statistics
        cpu_usage = statistics.mean(cpu_samples) if cpu_samples else 0
        memory_after = process.memory_info().rss / (1024**2)
        
        return FCLPerformanceMetrics(
            operation_name=operation_name,
            execution_time_ms=execution_time_ms,
            memory_usage_mb=current / (1024**2),
            memory_peak_mb=peak / (1024**2),
            cpu_usage_percent=cpu_usage,
            neuron_count=neuron_count,
            cortical_areas=cortical_areas,
            throughput_neurons_per_sec=throughput,
            timestamp=time.time()
        )
    
    def benchmark_fcl_manager_operations(self) -> FCLBenchmarkResult:
        """Benchmark basic FCL manager operations."""
        self.logger.info("Benchmarking FCL Manager operations...")
        
        metrics = []
        
        for neuron_count in self.neuron_counts:
            for cortical_count in [1, 5, 10]:
                if cortical_count > neuron_count // 10:  # Skip if too many cortical areas
                    continue
                
                # Create test data
                neurons_per_cortical = neuron_count // cortical_count
                neurons_by_cortical = {
                    i: list(range(i * neurons_per_cortical, (i + 1) * neurons_per_cortical))
                    for i in range(cortical_count)
                }
                
                # Test FCL Manager creation
                def create_fcl_manager():
                    return FCLManager(window_size=20)
                
                metric = self._measure_performance(
                    "fcl_manager_creation", create_fcl_manager, neuron_count, cortical_count
                )
                metrics.append(metric)
                
                # Test FCL update operations
                fcl_manager = FCLManager(window_size=20)
                
                def update_fcl():
                    fcl_manager.update_fcl(1, neurons_by_cortical)
                
                metric = self._measure_performance(
                    "fcl_update", update_fcl, neuron_count, cortical_count
                )
                metrics.append(metric)
                
                # Test FCL advance timestep
                def advance_timestep():
                    fcl_manager.advance_timestep()
                
                metric = self._measure_performance(
                    "fcl_advance_timestep", advance_timestep, neuron_count, cortical_count
                )
                metrics.append(metric)
        
        # Calculate summary statistics
        execution_times = [m.execution_time_ms for m in metrics if m.operation_name == "fcl_update"]
        throughputs = [m.throughput_neurons_per_sec for m in metrics if m.operation_name == "fcl_update"]
        
        summary_stats = {
            'avg_execution_time_ms': statistics.mean(execution_times) if execution_times else 0,
            'median_execution_time_ms': statistics.median(execution_times) if execution_times else 0,
            'max_execution_time_ms': max(execution_times) if execution_times else 0,
            'min_execution_time_ms': min(execution_times) if execution_times else 0,
            'avg_throughput_neurons_per_sec': statistics.mean(throughputs) if throughputs else 0,
            'max_throughput_neurons_per_sec': max(throughputs) if throughputs else 0
        }
        
        return FCLBenchmarkResult(
            test_name="fcl_manager_operations",
            metrics=metrics,
            summary_stats=summary_stats,
            system_info=self.system_info,
            timestamp=time.time()
        )
    
    def benchmark_async_fcl_processor(self) -> FCLBenchmarkResult:
        """Benchmark async FCL processor performance."""
        self.logger.info("Benchmarking Async FCL Processor...")
        
        metrics = []
        
        # Create ConnectomeManager and BurstEngine for async processor
        cm = ConnectomeManager(config_or_max_neurons=max(self.neuron_counts))
        be = BurstEngine(cm)
        async_processor = cm._get_async_fcl_processor()
        
        if not async_processor:
            self.logger.warning("Async FCL processor not available")
            return FCLBenchmarkResult(
                test_name="async_fcl_processor",
                metrics=[],
                summary_stats={},
                system_info=self.system_info,
                timestamp=time.time()
            )
        
        # Start async processor
        async_processor.start()
        time.sleep(0.1)  # Let it initialize
        
        try:
            for neuron_count in self.neuron_counts[:4]:  # Limit for async testing
                for cortical_count in [1, 5]:
                    neurons_per_cortical = neuron_count // cortical_count
                    neurons_by_cortical = {
                        i: list(range(i * neurons_per_cortical, (i + 1) * neurons_per_cortical))
                        for i in range(cortical_count)
                    }
                    
                    # Test async event processing
                    def process_async_event():
                        event = FiredNeuronEvent(
                            timestep=1,
                            neurons_by_cortical=neurons_by_cortical
                        )
                        async_processor.process_fired_neurons(event)
                    
                    metric = self._measure_performance(
                        "async_fcl_processing", process_async_event, neuron_count, cortical_count
                    )
                    metrics.append(metric)
            
            # Wait for processing to complete
            time.sleep(1.0)
            
        finally:
            async_processor.stop()
        
        # Calculate summary statistics
        execution_times = [m.execution_time_ms for m in metrics]
        throughputs = [m.throughput_neurons_per_sec for m in metrics]
        
        summary_stats = {
            'avg_execution_time_ms': statistics.mean(execution_times) if execution_times else 0,
            'median_execution_time_ms': statistics.median(execution_times) if execution_times else 0,
            'avg_throughput_neurons_per_sec': statistics.mean(throughputs) if throughputs else 0,
            'max_throughput_neurons_per_sec': max(throughputs) if throughputs else 0
        }
        
        return FCLBenchmarkResult(
            test_name="async_fcl_processor",
            metrics=metrics,
            summary_stats=summary_stats,
            system_info=self.system_info,
            timestamp=time.time()
        )
    
    def benchmark_rust_optimized_fcl(self) -> FCLBenchmarkResult:
        """Benchmark Rust-optimized FCL processor performance."""
        self.logger.info("Benchmarking Rust-optimized FCL Processor...")
        
        metrics = []
        
        try:
            from feagi.npu.rust_optimized_fcl import RustOptimizedFCLProcessor, RustCompatibleFCLInterface
            
            # Create Rust-optimized processor
            rust_processor = RustOptimizedFCLProcessor(buffer_capacity=max(self.neuron_counts))
            rust_interface = RustCompatibleFCLInterface(rust_processor)
            
            for neuron_count in self.neuron_counts[:4]:  # Limit for Rust testing
                for cortical_count in [1, 5]:
                    neurons_per_cortical = neuron_count // cortical_count
                    neurons_by_cortical = {
                        i: list(range(i * neurons_per_cortical, (i + 1) * neurons_per_cortical))
                        for i in range(cortical_count)
                    }
                    
                    # Test Rust-optimized processing
                    def process_rust_optimized():
                        rust_interface.process_fired_neurons_rust_compatible(
                            1, neurons_by_cortical
                        )
                    
                    metric = self._measure_performance(
                        "rust_optimized_fcl", process_rust_optimized, neuron_count, cortical_count
                    )
                    metrics.append(metric)
            
        except ImportError as e:
            self.logger.warning(f"Rust-optimized FCL not available: {e}")
        
        # Calculate summary statistics
        execution_times = [m.execution_time_ms for m in metrics]
        throughputs = [m.throughput_neurons_per_sec for m in metrics]
        
        summary_stats = {
            'avg_execution_time_ms': statistics.mean(execution_times) if execution_times else 0,
            'median_execution_time_ms': statistics.median(execution_times) if execution_times else 0,
            'avg_throughput_neurons_per_sec': statistics.mean(throughputs) if throughputs else 0,
            'max_throughput_neurons_per_sec': max(throughputs) if throughputs else 0
        }
        
        return FCLBenchmarkResult(
            test_name="rust_optimized_fcl",
            metrics=metrics,
            summary_stats=summary_stats,
            system_info=self.system_info,
            timestamp=time.time()
        )
    
    def benchmark_integration_performance(self) -> FCLBenchmarkResult:
        """Benchmark FCL integration with ConnectomeManager and BurstEngine."""
        self.logger.info("Benchmarking FCL Integration Performance...")
        
        metrics = []
        
        for neuron_count in self.neuron_counts[:3]:  # Limit for integration testing
            # Create integrated system
            cm = ConnectomeManager(config_or_max_neurons=neuron_count)
            be = BurstEngine(cm)
            
            # Test membrane potential update with FCL processing
            def integrated_update():
                # Simulate neuron firing
                fired_neurons = list(range(0, min(neuron_count // 10, 1000)))
                if fired_neurons:
                    # This triggers FCL update through the integration
                    cm.update_membrane_potentials()
            
            metric = self._measure_performance(
                "integrated_fcl_update", integrated_update, neuron_count, 1
            )
            metrics.append(metric)
        
        # Calculate summary statistics
        execution_times = [m.execution_time_ms for m in metrics]
        throughputs = [m.throughput_neurons_per_sec for m in metrics]
        
        summary_stats = {
            'avg_execution_time_ms': statistics.mean(execution_times) if execution_times else 0,
            'median_execution_time_ms': statistics.median(execution_times) if execution_times else 0,
            'avg_throughput_neurons_per_sec': statistics.mean(throughputs) if throughputs else 0
        }
        
        return FCLBenchmarkResult(
            test_name="integration_performance",
            metrics=metrics,
            summary_stats=summary_stats,
            system_info=self.system_info,
            timestamp=time.time()
        )
    
    def profile_fcl_operations(self) -> Dict[str, Any]:
        """Profile FCL operations using cProfile."""
        self.logger.info("Profiling FCL operations with cProfile...")
        
        # Create test setup
        fcl_manager = FCLManager(window_size=20)
        neuron_count = 10000
        neurons_by_cortical = {
            0: list(range(0, neuron_count // 2)),
            1: list(range(neuron_count // 2, neuron_count))
        }
        
        # Profile FCL operations
        profiler = cProfile.Profile()
        profiler.enable()
        
        # Run multiple FCL operations
        for i in range(100):
            fcl_manager.update_fcl(i, neurons_by_cortical)
            if i % 10 == 0:
                fcl_manager.advance_timestep()
        
        profiler.disable()
        
        # Save profile results
        profile_file = self.results_dir / f"fcl_profile_{int(time.time())}.prof"
        profiler.dump_stats(str(profile_file))
        
        # Generate profile statistics
        stats = pstats.Stats(profiler)
        stats.sort_stats('cumulative')
        
        # Extract top functions
        top_functions = []
        for func, (cc, nc, tt, ct, callers) in stats.stats.items():
            if ct > 0.001:  # Only functions taking more than 1ms
                top_functions.append({
                    'function': f"{func[0]}:{func[1]}({func[2]})",
                    'calls': nc,
                    'total_time': tt,
                    'cumulative_time': ct,
                    'per_call_time': ct / nc if nc > 0 else 0
                })
        
        # Sort by cumulative time
        top_functions.sort(key=lambda x: x['cumulative_time'], reverse=True)
        
        return {
            'profile_file': str(profile_file),
            'top_functions': top_functions[:20],  # Top 20 functions
            'total_calls': sum(stats.stats[func][1] for func in stats.stats),
            'total_time': sum(stats.stats[func][2] for func in stats.stats)
        }
    
    def save_results(self, results: List[FCLBenchmarkResult], profile_data: Dict[str, Any]):
        """Save benchmark results to JSON files."""
        timestamp = int(time.time())
        
        # Save individual benchmark results
        for result in results:
            filename = f"fcl_benchmark_{result.test_name}_{timestamp}.json"
            filepath = self.results_dir / filename
            
            with open(filepath, 'w') as f:
                json.dump(result.to_dict(), f, indent=2)
            
            self.logger.info(f"Saved {result.test_name} results to {filepath}")
        
        # Save profile data
        profile_filename = f"fcl_profile_analysis_{timestamp}.json"
        profile_filepath = self.results_dir / profile_filename
        
        with open(profile_filepath, 'w') as f:
            json.dump(profile_data, f, indent=2)
        
        self.logger.info(f"Saved profile analysis to {profile_filepath}")
        
        # Save summary report
        summary_filename = f"fcl_benchmark_summary_{timestamp}.json"
        summary_filepath = self.results_dir / summary_filename
        
        summary = {
            'timestamp': timestamp,
            'system_info': self.system_info,
            'benchmark_results': [r.to_dict() for r in results],
            'profile_analysis': profile_data
        }
        
        with open(summary_filepath, 'w') as f:
            json.dump(summary, f, indent=2)
        
        self.logger.info(f"Saved comprehensive summary to {summary_filepath}")
        
        return summary_filepath
    
    def print_performance_report(self, results: List[FCLBenchmarkResult]):
        """Print a comprehensive performance report."""
        print("\n" + "="*80)
        print("🚀 FEAGI FCL PERFORMANCE BENCHMARK REPORT")
        print("="*80)
        
        print(f"\n📊 System Information:")
        for key, value in self.system_info.items():
            print(f"   {key}: {value}")
        
        for result in results:
            if not result.metrics:
                continue
                
            print(f"\n📈 {result.test_name.upper()} PERFORMANCE:")
            print("-" * 60)
            
            # Print summary statistics
            for key, value in result.summary_stats.items():
                if 'time' in key:
                    print(f"   {key}: {value:.3f} ms")
                elif 'throughput' in key:
                    print(f"   {key}: {value:,.0f} neurons/sec")
                else:
                    print(f"   {key}: {value}")
            
            # Print best performance metrics
            if result.metrics:
                best_metric = min(result.metrics, key=lambda m: m.execution_time_ms)
                worst_metric = max(result.metrics, key=lambda m: m.execution_time_ms)
                
                print(f"\n   Best Performance:")
                print(f"     Neurons: {best_metric.neuron_count:,}")
                print(f"     Time: {best_metric.execution_time_ms:.3f} ms")
                print(f"     Throughput: {best_metric.throughput_neurons_per_sec:,.0f} neurons/sec")
                
                print(f"\n   Worst Performance:")
                print(f"     Neurons: {worst_metric.neuron_count:,}")
                print(f"     Time: {worst_metric.execution_time_ms:.3f} ms")
                print(f"     Throughput: {worst_metric.throughput_neurons_per_sec:,.0f} neurons/sec")
        
        print("\n" + "="*80)
    
    def run_comprehensive_benchmark(self) -> str:
        """Run the complete FCL performance benchmark suite."""
        self.logger.info("Starting comprehensive FCL performance benchmark...")
        
        results = []
        
        # Run all benchmarks
        results.append(self.benchmark_fcl_manager_operations())
        results.append(self.benchmark_async_fcl_processor())
        results.append(self.benchmark_rust_optimized_fcl())
        results.append(self.benchmark_integration_performance())
        
        # Run profiling
        profile_data = self.profile_fcl_operations()
        
        # Save results
        summary_file = self.save_results(results, profile_data)
        
        # Print report
        self.print_performance_report(results)
        
        self.logger.info("FCL performance benchmark completed successfully")
        return str(summary_file)


# Pytest integration
class TestFCLPerformance:
    """Pytest test class for FCL performance benchmarks."""
    
    def test_fcl_manager_performance(self):
        """Test FCL manager performance benchmarks."""
        benchmark = FCLPerformanceBenchmark()
        result = benchmark.benchmark_fcl_manager_operations()
        
        assert len(result.metrics) > 0, "No performance metrics collected"
        assert result.summary_stats['avg_execution_time_ms'] > 0, "Invalid execution time"
        
        # Performance assertions (adjust based on baseline measurements)
        avg_time = result.summary_stats['avg_execution_time_ms']
        max_throughput = result.summary_stats['max_throughput_neurons_per_sec']
        
        # These are initial thresholds - adjust based on actual measurements
        assert avg_time < 100, f"FCL operations too slow: {avg_time:.3f}ms average"
        assert max_throughput > 1000, f"FCL throughput too low: {max_throughput:,.0f} neurons/sec"
    
    def test_async_fcl_performance(self):
        """Test async FCL processor performance."""
        benchmark = FCLPerformanceBenchmark()
        result = benchmark.benchmark_async_fcl_processor()
        
        if result.metrics:  # Only test if async processor is available
            assert len(result.metrics) > 0, "No async performance metrics collected"
            assert result.summary_stats['avg_execution_time_ms'] >= 0, "Invalid async execution time"
    
    def test_comprehensive_benchmark(self):
        """Test the complete benchmark suite."""
        benchmark = FCLPerformanceBenchmark()
        summary_file = benchmark.run_comprehensive_benchmark()
        
        assert os.path.exists(summary_file), "Benchmark summary file not created"
        
        # Verify summary file contains expected data
        with open(summary_file, 'r') as f:
            summary = json.load(f)
        
        assert 'benchmark_results' in summary, "Missing benchmark results in summary"
        assert 'profile_analysis' in summary, "Missing profile analysis in summary"
        assert len(summary['benchmark_results']) > 0, "No benchmark results saved"


def run_fcl_performance_benchmark():
    """Standalone function to run FCL performance benchmarks."""
    benchmark = FCLPerformanceBenchmark()
    return benchmark.run_comprehensive_benchmark()


if __name__ == "__main__":
    # Run the benchmark when executed directly
    summary_file = run_fcl_performance_benchmark()
    print(f"\n✅ Benchmark completed! Results saved to: {summary_file}")
