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
Burst Frequency Robustness Test Suite

This module tests FEAGI's ability to maintain configured burst frequency
under increasing neural load. It measures frequency deviation, CPU usage,
and memory consumption as neuron firing counts increase.

Key Test Objectives:
1. Measure actual vs configured burst frequency under load
2. Identify the neuron count threshold where frequency drops
3. Monitor CPU and memory usage patterns
4. Detect frequency stability and jitter
5. Validate real-time performance guarantees
"""

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
        return asdict(self)


@dataclass
class FrequencyBreakdownPoint:
    """Point where frequency starts to degrade significantly."""
    neuron_count: int
    frequency_deviation_percent: float
    cpu_usage_percent: float
    memory_usage_mb: float
    burst_time_ms: float
    description: str


class BurstFrequencyRobustnessTest:
    """Comprehensive burst frequency robustness testing."""
    
    def __init__(self):
        """Initialize the frequency robustness test suite."""
        self.logger = setup_logger(__name__)
        self.results_dir = Path("tests/performance/logs")
        self.results_dir.mkdir(parents=True, exist_ok=True)
        
        # Test configurations - progressive neuron count scaling
        self.neuron_count_progression = [
            100, 500, 1000, 2500, 5000, 7500, 10000,
            15000, 20000, 30000, 50000, 75000, 100000,
            150000, 200000, 300000, 500000, 750000, 1000000
        ]
        
        # Frequency configurations to test
        self.test_frequencies = [10.0, 15.0, 20.0, 30.0, 50.0, 100.0]  # Hz
        
        # Performance thresholds
        self.frequency_deviation_thresholds = {
            'acceptable': 5.0,    # 5% deviation acceptable
            'concerning': 15.0,   # 15% deviation concerning
            'critical': 30.0      # 30% deviation critical
        }
        
        # Test parameters
        self.measurement_duration = 10.0  # seconds per test
        self.min_bursts_required = 50    # minimum bursts to measure
        self.resource_monitoring_interval = 0.1  # seconds
        
        # System monitoring
        self.process = psutil.Process()
        
        self.logger.info("Burst Frequency Robustness Test initialized")
    
    def _monitor_system_resources(self, duration: float) -> Dict[str, List[float]]:
        """Monitor CPU and memory usage during test execution."""
        cpu_samples = []
        memory_samples = []
        start_time = time.time()
        
        def monitor():
            while time.time() - start_time < duration:
                try:
                    cpu_percent = self.process.cpu_percent()
                    memory_mb = self.process.memory_info().rss / (1024 * 1024)
                    
                    cpu_samples.append(cpu_percent)
                    memory_samples.append(memory_mb)
                    
                    time.sleep(self.resource_monitoring_interval)
                except Exception as e:
                    self.logger.warning(f"Resource monitoring error: {e}")
                    break
        
        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
        
        return {'cpu': cpu_samples, 'memory': memory_samples}
    
    def _create_firing_pattern(self, neuron_count: int, firing_rate: float = 0.1) -> Dict[int, List[int]]:
        """Create realistic firing pattern for testing."""
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
    
    def _measure_frequency_robustness(self, neuron_count: int, 
                                    target_frequency: float) -> FrequencyRobustnessMetrics:
        """Measure frequency robustness for specific neuron count and frequency."""
        self.logger.info(f"Testing {neuron_count:,} neurons at {target_frequency}Hz")
        
        # Create test system
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
            
            # Start resource monitoring
            resource_data = self._monitor_system_resources(self.measurement_duration)
            
            # Start memory tracking
            tracemalloc.start()
            memory_start = self.process.memory_info().rss / (1024 * 1024)
            
            # Measure burst timing
            burst_times = []
            actual_frequencies = []
            overrun_count = 0
            
            target_burst_interval = 1.0 / target_frequency
            measurement_start = time.perf_counter()
            last_burst_time = measurement_start
            
            # Run bursts for measurement duration
            while (time.perf_counter() - measurement_start < self.measurement_duration and 
                   len(burst_times) < self.min_bursts_required * 10):
                
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
            memory_end = self.process.memory_info().rss / (1024 * 1024)
            
            # Wait for resource monitoring to complete
            time.sleep(0.2)
            
            # Calculate metrics
            measurement_duration = time.perf_counter() - measurement_start
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
            memory_growth_rate = (memory_end - memory_start) / measurement_duration if measurement_duration > 0 else 0
            
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
                measurement_duration_sec=measurement_duration,
                timestamp=time.time()
            )
            
        finally:
            if async_processor:
                async_processor.stop()
            
            # Clean up
            del cm, be
            gc.collect()
    
    def test_frequency_robustness_across_scales(self, target_frequency: float = 15.0) -> List[FrequencyRobustnessMetrics]:
        """Test frequency robustness across different neuron counts."""
        self.logger.info(f"Testing frequency robustness at {target_frequency}Hz across neuron scales")
        
        metrics = []
        
        for neuron_count in self.neuron_count_progression:
            try:
                metric = self._measure_frequency_robustness(neuron_count, target_frequency)
                metrics.append(metric)
                
                # Log progress
                deviation = metric.frequency_deviation_percent
                status = "✅" if deviation < 5 else "⚠️" if deviation < 15 else "❌"
                
                self.logger.info(
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
        
        return metrics
    
    def test_multi_frequency_robustness(self) -> Dict[float, List[FrequencyRobustnessMetrics]]:
        """Test robustness across multiple target frequencies."""
        self.logger.info("Testing frequency robustness across multiple target frequencies")
        
        results = {}
        
        for target_freq in self.test_frequencies:
            self.logger.info(f"Testing target frequency: {target_freq}Hz")
            
            # Test subset of neuron counts for each frequency
            test_counts = [1000, 5000, 10000, 25000, 50000, 100000]
            metrics = []
            
            for neuron_count in test_counts:
                try:
                    metric = self._measure_frequency_robustness(neuron_count, target_freq)
                    metrics.append(metric)
                    
                    # Stop if frequency degradation is too severe
                    if metric.frequency_deviation_percent > 40:
                        break
                        
                except Exception as e:
                    self.logger.error(f"Failed {target_freq}Hz test at {neuron_count:,} neurons: {e}")
                    continue
            
            results[target_freq] = metrics
        
        return results
    
    def identify_frequency_breakdown_points(self, metrics: List[FrequencyRobustnessMetrics]) -> List[FrequencyBreakdownPoint]:
        """Identify points where frequency starts to degrade significantly."""
        breakdown_points = []
        
        for i, metric in enumerate(metrics):
            deviation = metric.frequency_deviation_percent
            
            # Check for significant degradation thresholds
            if deviation >= self.frequency_deviation_thresholds['critical']:
                breakdown_points.append(FrequencyBreakdownPoint(
                    neuron_count=metric.neuron_count,
                    frequency_deviation_percent=deviation,
                    cpu_usage_percent=metric.cpu_usage_avg_percent,
                    memory_usage_mb=metric.memory_usage_avg_mb,
                    burst_time_ms=metric.burst_time_avg_ms,
                    description=f"Critical frequency degradation: {deviation:.1f}% deviation"
                ))
            elif deviation >= self.frequency_deviation_thresholds['concerning']:
                breakdown_points.append(FrequencyBreakdownPoint(
                    neuron_count=metric.neuron_count,
                    frequency_deviation_percent=deviation,
                    cpu_usage_percent=metric.cpu_usage_avg_percent,
                    memory_usage_mb=metric.memory_usage_avg_mb,
                    burst_time_ms=metric.burst_time_avg_ms,
                    description=f"Concerning frequency degradation: {deviation:.1f}% deviation"
                ))
            
            # Check for resource saturation
            if metric.cpu_usage_avg_percent > 90:
                breakdown_points.append(FrequencyBreakdownPoint(
                    neuron_count=metric.neuron_count,
                    frequency_deviation_percent=deviation,
                    cpu_usage_percent=metric.cpu_usage_avg_percent,
                    memory_usage_mb=metric.memory_usage_avg_mb,
                    burst_time_ms=metric.burst_time_avg_ms,
                    description=f"CPU saturation: {metric.cpu_usage_avg_percent:.1f}% usage"
                ))
            
            # Check for memory growth issues
            if metric.memory_growth_rate_mb_per_sec > 10:  # >10MB/sec growth
                breakdown_points.append(FrequencyBreakdownPoint(
                    neuron_count=metric.neuron_count,
                    frequency_deviation_percent=deviation,
                    cpu_usage_percent=metric.cpu_usage_avg_percent,
                    memory_usage_mb=metric.memory_usage_avg_mb,
                    burst_time_ms=metric.burst_time_avg_ms,
                    description=f"Memory leak detected: {metric.memory_growth_rate_mb_per_sec:.1f}MB/sec growth"
                ))
        
        return breakdown_points
    
    def save_results(self, metrics: List[FrequencyRobustnessMetrics], 
                    breakdown_points: List[FrequencyBreakdownPoint],
                    target_frequency: float) -> str:
        """Save frequency robustness test results."""
        timestamp = int(time.time())
        filename = f"frequency_robustness_{target_frequency}hz_{timestamp}.json"
        filepath = self.results_dir / filename
        
        results = {
            'test_type': 'frequency_robustness',
            'target_frequency_hz': target_frequency,
            'metrics': [m.to_dict() for m in metrics],
            'breakdown_points': [asdict(bp) for bp in breakdown_points],
            'thresholds': self.frequency_deviation_thresholds,
            'test_parameters': {
                'neuron_count_progression': self.neuron_count_progression,
                'measurement_duration': self.measurement_duration,
                'min_bursts_required': self.min_bursts_required
            },
            'timestamp': time.time()
        }
        
        with open(filepath, 'w') as f:
            json.dump(results, f, indent=2)
        
        self.logger.info(f"Saved frequency robustness results to {filepath}")
        return str(filepath)
    
    def print_frequency_robustness_report(self, metrics: List[FrequencyRobustnessMetrics],
                                        breakdown_points: List[FrequencyBreakdownPoint],
                                        target_frequency: float):
        """Print comprehensive frequency robustness report."""
        print("\n" + "="*80)
        print(f"🎯 BURST FREQUENCY ROBUSTNESS REPORT - {target_frequency}Hz TARGET")
        print("="*80)
        
        if not metrics:
            print("❌ No metrics collected")
            return
        
        # Summary statistics
        successful_tests = [m for m in metrics if m.frequency_deviation_percent < 15]
        max_stable_neurons = max([m.neuron_count for m in successful_tests]) if successful_tests else 0
        
        print(f"\n📊 SUMMARY:")
        print(f"   Target Frequency: {target_frequency} Hz")
        print(f"   Tests Completed: {len(metrics)}")
        print(f"   Maximum Stable Neuron Count: {max_stable_neurons:,}")
        print(f"   Breakdown Points Detected: {len(breakdown_points)}")
        
        # Performance across scales
        print(f"\n📈 FREQUENCY PERFORMANCE BY SCALE:")
        print("-" * 70)
        print(f"{'Neurons':>10} {'Actual Hz':>10} {'Deviation':>10} {'CPU %':>8} {'Memory MB':>12} {'Status':>8}")
        print("-" * 70)
        
        for metric in metrics:
            deviation = metric.frequency_deviation_percent
            status = "✅ GOOD" if deviation < 5 else "⚠️ WARN" if deviation < 15 else "❌ FAIL"
            
            print(f"{metric.neuron_count:>10,} "
                  f"{metric.actual_frequency_hz:>10.1f} "
                  f"{deviation:>9.1f}% "
                  f"{metric.cpu_usage_avg_percent:>7.1f} "
                  f"{metric.memory_usage_avg_mb:>11.1f} "
                  f"{status:>8}")
        
        # Breakdown points
        if breakdown_points:
            print(f"\n🚨 BREAKDOWN POINTS:")
            print("-" * 60)
            for bp in breakdown_points:
                print(f"   • {bp.neuron_count:,} neurons: {bp.description}")
        else:
            print(f"\n✅ No breakdown points detected within test range")
        
        # Resource usage analysis
        avg_cpu = statistics.mean([m.cpu_usage_avg_percent for m in metrics])
        max_cpu = max([m.cpu_usage_peak_percent for m in metrics])
        avg_memory = statistics.mean([m.memory_usage_avg_mb for m in metrics])
        max_memory = max([m.memory_usage_peak_mb for m in metrics])
        
        print(f"\n💻 RESOURCE USAGE ANALYSIS:")
        print(f"   Average CPU Usage: {avg_cpu:.1f}%")
        print(f"   Peak CPU Usage: {max_cpu:.1f}%")
        print(f"   Average Memory Usage: {avg_memory:.1f} MB")
        print(f"   Peak Memory Usage: {max_memory:.1f} MB")
        
        # Recommendations
        print(f"\n💡 RECOMMENDATIONS:")
        if max_stable_neurons < 50000:
            print("   ⚠️  Consider optimization for larger neuron counts")
        if max_cpu > 80:
            print("   ⚠️  CPU usage approaching limits - consider parallel processing")
        if max_memory > 1000:
            print("   ⚠️  Memory usage high - consider memory optimization")
        if len(breakdown_points) == 0:
            print("   ✅ Excellent frequency stability across all tested scales")
        
        print("\n" + "="*80)


# Pytest integration
class TestBurstFrequencyRobustness:
    """Pytest test class for burst frequency robustness."""
    
    def test_15hz_frequency_robustness(self):
        """Test 15Hz frequency robustness across neuron scales."""
        tester = BurstFrequencyRobustnessTest()
        metrics = tester.test_frequency_robustness_across_scales(15.0)
        
        assert len(metrics) > 0, "No frequency robustness metrics collected"
        
        # Check that we can handle at least 10K neurons at 15Hz with <15% deviation
        large_scale_metrics = [m for m in metrics if m.neuron_count >= 10000]
        if large_scale_metrics:
            best_large_scale = min(large_scale_metrics, key=lambda m: m.frequency_deviation_percent)
            assert best_large_scale.frequency_deviation_percent < 15, \
                f"15Hz frequency not maintained at 10K+ neurons: {best_large_scale.frequency_deviation_percent:.1f}% deviation"
    
    def test_multi_frequency_robustness(self):
        """Test robustness across multiple frequencies."""
        tester = BurstFrequencyRobustnessTest()
        results = tester.test_multi_frequency_robustness()
        
        assert len(results) > 0, "No multi-frequency results collected"
        
        # Each frequency should have some successful tests
        for freq, metrics in results.items():
            assert len(metrics) > 0, f"No metrics for {freq}Hz"
            
            # At least one test should have <20% deviation
            successful_tests = [m for m in metrics if m.frequency_deviation_percent < 20]
            assert len(successful_tests) > 0, f"No successful tests for {freq}Hz"


def run_frequency_robustness_test(target_frequency: float = 15.0) -> str:
    """Standalone function to run frequency robustness test."""
    tester = BurstFrequencyRobustnessTest()
    
    print(f"🚀 Starting Burst Frequency Robustness Test - {target_frequency}Hz")
    
    # Run the test
    metrics = tester.test_frequency_robustness_across_scales(target_frequency)
    breakdown_points = tester.identify_frequency_breakdown_points(metrics)
    
    # Save results
    results_file = tester.save_results(metrics, breakdown_points, target_frequency)
    
    # Print report
    tester.print_frequency_robustness_report(metrics, breakdown_points, target_frequency)
    
    return results_file


if __name__ == "__main__":
    # Run the frequency robustness test when executed directly
    results_file = run_frequency_robustness_test(15.0)
    print(f"\n✅ Frequency robustness test completed! Results saved to: {results_file}")
