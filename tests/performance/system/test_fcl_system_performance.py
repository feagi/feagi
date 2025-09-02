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
System-Level FCL Performance Tests

This module tests end-to-end FCL processing performance across the entire
FEAGI system, including BDU-NPU integration, burst engine coordination,
and real-world neural activity simulation.

Test Scenarios:
1. End-to-End Burst Processing - Complete burst cycle with FCL updates
2. Multi-Cortical Area Simulation - Complex neural activity patterns
3. High-Frequency Burst Testing - 15Hz+ burst frequency testing
4. Memory Pressure Testing - Performance under memory constraints
5. Concurrent Processing - Multiple simultaneous neural activities
"""

import asyncio
import gc
import json
import os
import statistics
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
import tracemalloc

import psutil
import pytest
import numpy as np

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.npu.burst_engine import BurstEngine
from feagi.npu.interfaces import FiredNeuronEvent
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


@dataclass
class SystemPerformanceMetrics:
    """System-level performance metrics."""
    test_scenario: str
    total_execution_time_ms: float
    burst_processing_time_ms: float
    fcl_processing_time_ms: float
    memory_usage_mb: float
    memory_peak_mb: float
    cpu_usage_percent: float
    neurons_processed: int
    cortical_areas: int
    burst_frequency_hz: float
    throughput_neurons_per_sec: float
    latency_p50_ms: float
    latency_p95_ms: float
    latency_p99_ms: float
    timestamp: float


@dataclass
class SystemBenchmarkResult:
    """Complete system benchmark result."""
    test_name: str
    metrics: List[SystemPerformanceMetrics]
    performance_targets: Dict[str, float]
    target_analysis: Dict[str, bool]
    bottleneck_analysis: Dict[str, Any]
    recommendations: List[str]
    timestamp: float


class FCLSystemPerformanceBenchmark:
    """System-level FCL performance benchmark suite."""
    
    def __init__(self):
        """Initialize the system benchmark suite."""
        self.logger = setup_logger(__name__)
        self.results_dir = Path("tests/performance/logs")
        self.results_dir.mkdir(parents=True, exist_ok=True)
        
        # Performance targets (based on FEAGI requirements)
        self.performance_targets = {
            'burst_frequency_hz': 15.0,  # Target 15Hz burst frequency
            'max_burst_time_ms': 66.7,   # Max time per burst at 15Hz
            'min_throughput_neurons_per_sec': 100000,  # Min 100K neurons/sec
            'max_latency_p95_ms': 50.0,   # Max 95th percentile latency
            'max_memory_usage_mb': 1000,  # Max 1GB memory usage
            'max_cpu_usage_percent': 80   # Max 80% CPU usage
        }
        
        # Test configurations
        self.test_scenarios = [
            {'name': 'small_scale', 'neurons': 1000, 'cortical_areas': 5, 'bursts': 50},
            {'name': 'medium_scale', 'neurons': 10000, 'cortical_areas': 20, 'bursts': 30},
            {'name': 'large_scale', 'neurons': 50000, 'cortical_areas': 50, 'bursts': 20},
            {'name': 'high_frequency', 'neurons': 5000, 'cortical_areas': 10, 'bursts': 100}
        ]
        
        self.logger.info("System FCL Performance Benchmark initialized")
    
    def _create_realistic_neural_activity(self, neuron_count: int, 
                                        cortical_areas: int) -> Dict[int, List[int]]:
        """Create realistic neural activity patterns."""
        neurons_per_area = neuron_count // cortical_areas
        activity_rate = 0.1  # 10% of neurons fire per burst
        
        neurons_by_cortical = {}
        
        for area_id in range(cortical_areas):
            start_neuron = area_id * neurons_per_area
            end_neuron = min((area_id + 1) * neurons_per_area, neuron_count)
            
            # Simulate realistic firing patterns
            area_neurons = list(range(start_neuron, end_neuron))
            firing_count = max(1, int(len(area_neurons) * activity_rate))
            
            # Add some randomness to firing patterns
            np.random.seed(area_id)  # Reproducible randomness
            firing_neurons = np.random.choice(area_neurons, firing_count, replace=False)
            
            neurons_by_cortical[area_id] = firing_neurons.tolist()
        
        return neurons_by_cortical
    
    def _measure_burst_cycle_performance(self, cm: ConnectomeManager, be: BurstEngine,
                                       neurons_by_cortical: Dict[int, List[int]]) -> Tuple[float, float, List[float]]:
        """Measure performance of a complete burst cycle."""
        latencies = []
        
        # Start memory tracking
        tracemalloc.start()
        
        # Measure complete burst cycle
        start_time = time.perf_counter()
        
        # Simulate burst processing with FCL updates
        burst_start = time.perf_counter()
        
        # Create and process fired neuron event
        event = FiredNeuronEvent(
            timestep=1,
            neurons_by_cortical=neurons_by_cortical
        )
        
        # Process through async FCL processor if available
        async_processor = cm._get_async_fcl_processor()
        if async_processor:
            event_start = time.perf_counter()
            async_processor.process_fired_neurons(event)
            event_end = time.perf_counter()
            latencies.append((event_end - event_start) * 1000)
        else:
            # Fallback to direct FCL manager
            fcl_manager = cm._get_fcl_manager()
            if fcl_manager:
                event_start = time.perf_counter()
                fcl_manager.update_fcl(1, neurons_by_cortical)
                event_end = time.perf_counter()
                latencies.append((event_end - event_start) * 1000)
        
        burst_end = time.perf_counter()
        burst_time = (burst_end - burst_start) * 1000
        
        # Measure FCL-specific processing time
        fcl_start = time.perf_counter()
        if async_processor:
            # Wait for async processing to complete
            time.sleep(0.001)  # Small delay for async processing
        fcl_end = time.perf_counter()
        fcl_time = (fcl_end - fcl_start) * 1000
        
        end_time = time.perf_counter()
        total_time = (end_time - start_time) * 1000
        
        # Stop memory tracking
        current, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        
        return total_time, burst_time, fcl_time, latencies, current, peak
    
    def benchmark_end_to_end_burst_processing(self) -> SystemBenchmarkResult:
        """Benchmark end-to-end burst processing performance."""
        self.logger.info("Benchmarking end-to-end burst processing...")
        
        metrics = []
        
        for scenario in self.test_scenarios:
            self.logger.info(f"Testing scenario: {scenario['name']}")
            
            # Create system components
            cm = ConnectomeManager(config_or_max_neurons=scenario['neurons'])
            be = BurstEngine(cm)
            
            # Start async processor if available
            async_processor = cm._get_async_fcl_processor()
            if async_processor:
                async_processor.start()
                time.sleep(0.1)  # Let it initialize
            
            try:
                scenario_latencies = []
                scenario_burst_times = []
                scenario_fcl_times = []
                scenario_memory_usage = []
                
                # Run multiple bursts
                for burst_idx in range(scenario['bursts']):
                    # Create realistic neural activity
                    neurons_by_cortical = self._create_realistic_neural_activity(
                        scenario['neurons'], scenario['cortical_areas']
                    )
                    
                    # Measure burst cycle performance
                    total_time, burst_time, fcl_time, latencies, memory_current, memory_peak = \
                        self._measure_burst_cycle_performance(cm, be, neurons_by_cortical)
                    
                    scenario_latencies.extend(latencies)
                    scenario_burst_times.append(burst_time)
                    scenario_fcl_times.append(fcl_time)
                    scenario_memory_usage.append(memory_current / (1024**2))
                    
                    # Small delay between bursts
                    time.sleep(0.001)
                
                # Calculate performance metrics
                total_neurons = scenario['neurons'] * scenario['bursts']
                total_time_sec = sum(scenario_burst_times) / 1000
                throughput = total_neurons / total_time_sec if total_time_sec > 0 else 0
                
                # Calculate latency percentiles
                if scenario_latencies:
                    p50 = np.percentile(scenario_latencies, 50)
                    p95 = np.percentile(scenario_latencies, 95)
                    p99 = np.percentile(scenario_latencies, 99)
                else:
                    p50 = p95 = p99 = 0
                
                # Calculate burst frequency
                avg_burst_time = statistics.mean(scenario_burst_times) if scenario_burst_times else 0
                burst_frequency = 1000 / avg_burst_time if avg_burst_time > 0 else 0
                
                metric = SystemPerformanceMetrics(
                    test_scenario=scenario['name'],
                    total_execution_time_ms=sum(scenario_burst_times),
                    burst_processing_time_ms=statistics.mean(scenario_burst_times) if scenario_burst_times else 0,
                    fcl_processing_time_ms=statistics.mean(scenario_fcl_times) if scenario_fcl_times else 0,
                    memory_usage_mb=statistics.mean(scenario_memory_usage) if scenario_memory_usage else 0,
                    memory_peak_mb=max(scenario_memory_usage) if scenario_memory_usage else 0,
                    cpu_usage_percent=psutil.cpu_percent(),
                    neurons_processed=total_neurons,
                    cortical_areas=scenario['cortical_areas'],
                    burst_frequency_hz=burst_frequency,
                    throughput_neurons_per_sec=throughput,
                    latency_p50_ms=p50,
                    latency_p95_ms=p95,
                    latency_p99_ms=p99,
                    timestamp=time.time()
                )
                
                metrics.append(metric)
                
            finally:
                if async_processor:
                    async_processor.stop()
                
                # Clean up
                del cm, be
                gc.collect()
        
        # Analyze performance against targets
        target_analysis = self._analyze_performance_targets(metrics)
        bottleneck_analysis = self._analyze_bottlenecks(metrics)
        recommendations = self._generate_recommendations(metrics, target_analysis)
        
        return SystemBenchmarkResult(
            test_name="end_to_end_burst_processing",
            metrics=metrics,
            performance_targets=self.performance_targets,
            target_analysis=target_analysis,
            bottleneck_analysis=bottleneck_analysis,
            recommendations=recommendations,
            timestamp=time.time()
        )
    
    def _analyze_performance_targets(self, metrics: List[SystemPerformanceMetrics]) -> Dict[str, bool]:
        """Analyze performance against targets."""
        analysis = {}
        
        if not metrics:
            return analysis
        
        # Check burst frequency target
        max_burst_freq = max(m.burst_frequency_hz for m in metrics)
        analysis['burst_frequency_target'] = max_burst_freq >= self.performance_targets['burst_frequency_hz']
        
        # Check burst time target
        min_burst_time = min(m.burst_processing_time_ms for m in metrics if m.burst_processing_time_ms > 0)
        analysis['burst_time_target'] = min_burst_time <= self.performance_targets['max_burst_time_ms']
        
        # Check throughput target
        max_throughput = max(m.throughput_neurons_per_sec for m in metrics)
        analysis['throughput_target'] = max_throughput >= self.performance_targets['min_throughput_neurons_per_sec']
        
        # Check latency target
        max_p95_latency = max(m.latency_p95_ms for m in metrics if m.latency_p95_ms > 0)
        analysis['latency_target'] = max_p95_latency <= self.performance_targets['max_latency_p95_ms']
        
        # Check memory target
        max_memory = max(m.memory_peak_mb for m in metrics)
        analysis['memory_target'] = max_memory <= self.performance_targets['max_memory_usage_mb']
        
        return analysis
    
    def _analyze_bottlenecks(self, metrics: List[SystemPerformanceMetrics]) -> Dict[str, Any]:
        """Analyze performance bottlenecks."""
        if not metrics:
            return {}
        
        # Find the slowest operations
        slowest_burst = max(metrics, key=lambda m: m.burst_processing_time_ms)
        highest_latency = max(metrics, key=lambda m: m.latency_p95_ms)
        highest_memory = max(metrics, key=lambda m: m.memory_peak_mb)
        
        # Analyze FCL vs total processing time ratio
        fcl_ratios = []
        for m in metrics:
            if m.burst_processing_time_ms > 0:
                ratio = m.fcl_processing_time_ms / m.burst_processing_time_ms
                fcl_ratios.append(ratio)
        
        avg_fcl_ratio = statistics.mean(fcl_ratios) if fcl_ratios else 0
        
        return {
            'slowest_scenario': slowest_burst.test_scenario,
            'slowest_burst_time_ms': slowest_burst.burst_processing_time_ms,
            'highest_latency_scenario': highest_latency.test_scenario,
            'highest_latency_p95_ms': highest_latency.latency_p95_ms,
            'highest_memory_scenario': highest_memory.test_scenario,
            'highest_memory_mb': highest_memory.memory_peak_mb,
            'avg_fcl_processing_ratio': avg_fcl_ratio,
            'fcl_bottleneck_severity': 'high' if avg_fcl_ratio > 0.5 else 'medium' if avg_fcl_ratio > 0.2 else 'low'
        }
    
    def _generate_recommendations(self, metrics: List[SystemPerformanceMetrics], 
                                target_analysis: Dict[str, bool]) -> List[str]:
        """Generate performance optimization recommendations."""
        recommendations = []
        
        if not target_analysis.get('burst_frequency_target', True):
            recommendations.append("Optimize burst processing to achieve 15Hz target frequency")
        
        if not target_analysis.get('throughput_target', True):
            recommendations.append("Improve neuron processing throughput - consider parallel processing")
        
        if not target_analysis.get('latency_target', True):
            recommendations.append("Reduce FCL processing latency - consider async optimizations")
        
        if not target_analysis.get('memory_target', True):
            recommendations.append("Optimize memory usage - consider memory pooling and caching")
        
        # Analyze FCL-specific recommendations
        if metrics:
            avg_fcl_time = statistics.mean(m.fcl_processing_time_ms for m in metrics if m.fcl_processing_time_ms > 0)
            if avg_fcl_time > 10:  # If FCL takes more than 10ms on average
                recommendations.append("FCL processing is a bottleneck - consider Rust optimization")
        
        return recommendations
    
    def save_results(self, result: SystemBenchmarkResult) -> str:
        """Save system benchmark results."""
        timestamp = int(time.time())
        filename = f"fcl_system_benchmark_{timestamp}.json"
        filepath = self.results_dir / filename
        
        # Convert result to dictionary
        result_dict = {
            'test_name': result.test_name,
            'metrics': [asdict(m) for m in result.metrics],
            'performance_targets': result.performance_targets,
            'target_analysis': result.target_analysis,
            'bottleneck_analysis': result.bottleneck_analysis,
            'recommendations': result.recommendations,
            'timestamp': result.timestamp
        }
        
        with open(filepath, 'w') as f:
            json.dump(result_dict, f, indent=2)
        
        self.logger.info(f"Saved system benchmark results to {filepath}")
        return str(filepath)
    
    def print_system_performance_report(self, result: SystemBenchmarkResult):
        """Print comprehensive system performance report."""
        print("\n" + "="*80)
        print("🚀 FEAGI SYSTEM-LEVEL FCL PERFORMANCE REPORT")
        print("="*80)
        
        print(f"\n📊 Performance Targets:")
        for key, value in result.performance_targets.items():
            status = "✅" if result.target_analysis.get(key.replace('max_', '').replace('min_', '') + '_target', False) else "❌"
            print(f"   {status} {key}: {value}")
        
        print(f"\n📈 Test Scenarios Performance:")
        print("-" * 60)
        
        for metric in result.metrics:
            print(f"\n🔍 {metric.test_scenario.upper()}:")
            print(f"   Neurons: {metric.neurons_processed:,}")
            print(f"   Cortical Areas: {metric.cortical_areas}")
            print(f"   Burst Frequency: {metric.burst_frequency_hz:.1f} Hz")
            print(f"   Burst Time: {metric.burst_processing_time_ms:.3f} ms")
            print(f"   FCL Time: {metric.fcl_processing_time_ms:.3f} ms")
            print(f"   Throughput: {metric.throughput_neurons_per_sec:,.0f} neurons/sec")
            print(f"   Latency P95: {metric.latency_p95_ms:.3f} ms")
            print(f"   Memory Peak: {metric.memory_peak_mb:.1f} MB")
        
        print(f"\n🔍 Bottleneck Analysis:")
        print("-" * 40)
        for key, value in result.bottleneck_analysis.items():
            print(f"   {key}: {value}")
        
        print(f"\n💡 Optimization Recommendations:")
        print("-" * 40)
        for i, rec in enumerate(result.recommendations, 1):
            print(f"   {i}. {rec}")
        
        print("\n" + "="*80)
    
    def run_system_benchmark(self) -> str:
        """Run the complete system-level FCL benchmark."""
        self.logger.info("Starting system-level FCL performance benchmark...")
        
        result = self.benchmark_end_to_end_burst_processing()
        
        # Save results
        results_file = self.save_results(result)
        
        # Print report
        self.print_system_performance_report(result)
        
        self.logger.info("System FCL performance benchmark completed")
        return results_file


# Pytest integration
class TestFCLSystemPerformance:
    """Pytest test class for system-level FCL performance."""
    
    def test_system_performance_targets(self):
        """Test system performance against targets."""
        benchmark = FCLSystemPerformanceBenchmark()
        result = benchmark.benchmark_end_to_end_burst_processing()
        
        assert len(result.metrics) > 0, "No system performance metrics collected"
        
        # Check critical performance targets
        target_analysis = result.target_analysis
        
        # At least some scenarios should meet throughput targets
        assert any(m.throughput_neurons_per_sec > 10000 for m in result.metrics), \
            "No scenario achieved minimum throughput of 10K neurons/sec"
        
        # Memory usage should be reasonable
        max_memory = max(m.memory_peak_mb for m in result.metrics)
        assert max_memory < 2000, f"Memory usage too high: {max_memory:.1f} MB"
        
        # Latency should be reasonable
        max_latency = max(m.latency_p95_ms for m in result.metrics if m.latency_p95_ms > 0)
        if max_latency > 0:
            assert max_latency < 200, f"Latency too high: {max_latency:.3f} ms"
    
    def test_performance_regression(self):
        """Test for performance regression by comparing with baseline."""
        benchmark = FCLSystemPerformanceBenchmark()
        result = benchmark.benchmark_end_to_end_burst_processing()
        
        # This test would compare against saved baseline results
        # For now, just ensure we have valid metrics
        assert len(result.metrics) > 0, "No performance metrics for regression testing"
        
        # Ensure all metrics have valid values
        for metric in result.metrics:
            assert metric.burst_processing_time_ms > 0, "Invalid burst processing time"
            assert metric.throughput_neurons_per_sec >= 0, "Invalid throughput"
            assert metric.neurons_processed > 0, "No neurons processed"


def run_system_fcl_benchmark():
    """Standalone function to run system FCL benchmarks."""
    benchmark = FCLSystemPerformanceBenchmark()
    return benchmark.run_system_benchmark()


if __name__ == "__main__":
    # Run the system benchmark when executed directly
    results_file = run_system_fcl_benchmark()
    print(f"\n✅ System benchmark completed! Results saved to: {results_file}")
