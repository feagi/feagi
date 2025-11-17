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
FEAGI Stress Testing - Push System to Limits

This module pushes FEAGI to its absolute limits to identify bottlenecks
and breaking points. It progressively increases load until failure.

Stress Test Categories:
1. Extreme Neuron Count Scaling (up to 10M+ neurons)
2. High-Frequency Burst Testing (up to 1000Hz)
3. Memory Pressure Testing (until OOM)
4. CPU Saturation Testing (until 100% CPU)
5. Concurrent Load Testing (multiple simultaneous operations)
6. Sustained Load Testing (long-duration stress)
"""

import gc
import json
import os
import statistics
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
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
class StressTestMetrics:
    """Metrics for stress testing."""
    test_name: str
    load_parameter: Any  # Could be neuron_count, frequency, etc.
    success: bool
    failure_reason: Optional[str]
    execution_time_ms: float
    cpu_usage_avg_percent: float
    cpu_usage_peak_percent: float
    memory_usage_avg_mb: float
    memory_usage_peak_mb: float
    memory_growth_rate_mb_per_sec: float
    throughput_neurons_per_sec: float
    frequency_achieved_hz: float
    frequency_deviation_percent: float
    overrun_count: int
    error_count: int
    timestamp: float
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return asdict(self)


@dataclass
class BottleneckAnalysis:
    """Analysis of identified bottlenecks."""
    bottleneck_type: str  # 'cpu', 'memory', 'frequency', 'throughput'
    threshold_value: Any
    description: str
    recommendations: List[str]
    severity: str  # 'low', 'medium', 'high', 'critical'


class StressTestSuite:
    """Comprehensive stress testing suite."""
    
    def __init__(self):
        """Initialize the stress test suite."""
        self.logger = setup_logger(__name__)
        self.results_dir = Path("tests/performance/logs")
        self.results_dir.mkdir(parents=True, exist_ok=True)
        
        # System monitoring
        self.process = psutil.Process()
        self.system_memory_gb = psutil.virtual_memory().total / (1024**3)
        self.cpu_count = psutil.cpu_count()
        
        # Stress test parameters
        self.max_test_duration = 300  # 5 minutes max per test
        self.resource_monitoring_interval = 0.1  # 100ms
        
        # Breaking point thresholds
        self.cpu_saturation_threshold = 95.0  # 95% CPU
        self.memory_pressure_threshold = 0.9  # 90% of system memory
        self.frequency_deviation_threshold = 50.0  # 50% frequency deviation
        
        self.logger.info("Stress Test Suite initialized")
        self.logger.info(f"System: {self.cpu_count} CPUs, {self.system_memory_gb:.1f}GB RAM")
    
    def _monitor_system_resources(self, duration: float) -> Dict[str, List[float]]:
        """Monitor system resources during stress test."""
        cpu_samples = []
        memory_samples = []
        start_time = time.time()
        
        def monitor():
            while time.time() - start_time < duration:
                try:
                    # CPU usage
                    cpu_percent = self.process.cpu_percent()
                    cpu_samples.append(cpu_percent)
                    
                    # Memory usage
                    memory_mb = self.process.memory_info().rss / (1024 * 1024)
                    memory_samples.append(memory_mb)
                    
                    # Check for critical resource usage
                    if cpu_percent > self.cpu_saturation_threshold:
                        self.logger.warning(f"CPU saturation detected: {cpu_percent:.1f}%")
                    
                    if memory_mb > (self.system_memory_gb * 1024 * self.memory_pressure_threshold):
                        self.logger.warning(f"Memory pressure detected: {memory_mb:.1f}MB")
                    
                    time.sleep(self.resource_monitoring_interval)
                except Exception as e:
                    self.logger.error(f"Resource monitoring error: {e}")
                    break
        
        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
        monitor_thread.join(duration + 1)  # Wait with timeout
        
        return {'cpu': cpu_samples, 'memory': memory_samples}
    
    def _create_extreme_firing_pattern(self, neuron_count: int, firing_rate: float = 0.2) -> Dict[int, List[int]]:
        """Create extreme firing patterns for stress testing."""
        # Use higher firing rate for stress testing
        cortical_areas = min(max(1, neuron_count // 500), 100)  # More cortical areas
        neurons_per_area = neuron_count // cortical_areas
        
        neurons_by_cortical = {}
        
        for area_id in range(cortical_areas):
            start_neuron = area_id * neurons_per_area
            end_neuron = min((area_id + 1) * neurons_per_area, neuron_count)
            
            area_neurons = list(range(start_neuron, end_neuron))
            firing_count = max(1, int(len(area_neurons) * firing_rate))
            
            # Create varied firing patterns
            np.random.seed(area_id + neuron_count)
            firing_neurons = np.random.choice(area_neurons, firing_count, replace=False)
            
            neurons_by_cortical[area_id] = firing_neurons.tolist()
        
        return neurons_by_cortical
    
    def test_extreme_neuron_scaling(self) -> List[StressTestMetrics]:
        """Test with extreme neuron counts until failure."""
        self.logger.info("🔥 Starting extreme neuron scaling stress test...")
        
        # Progressive neuron counts - push to millions
        neuron_counts = [
            100000, 250000, 500000, 750000, 1000000,  # 100K - 1M
            1500000, 2000000, 3000000, 5000000,       # 1.5M - 5M
            7500000, 10000000, 15000000, 20000000     # 7.5M - 20M
        ]
        
        metrics = []
        
        for neuron_count in neuron_counts:
            self.logger.info(f"Testing {neuron_count:,} neurons...")
            
            try:
                # Start resource monitoring
                start_time = time.perf_counter()
                resource_data = self._monitor_system_resources(30)  # 30 second test
                
                # Start memory tracking
                tracemalloc.start()
                memory_before = self.process.memory_info().rss / (1024 * 1024)
                
                # Create system
                cm = ConnectomeManager(config_or_max_neurons=neuron_count)
                be = BurstEngine(cm)
                
                # Start async processor
                async_processor = cm._get_async_fcl_processor()
                if async_processor:
                    async_processor.start()
                    time.sleep(0.1)
                
                # Create extreme firing pattern
                neurons_by_cortical = self._create_extreme_firing_pattern(neuron_count, 0.2)
                total_firing_neurons = sum(len(neurons) for neurons in neurons_by_cortical.values())
                
                # Execute stress test
                test_start = time.perf_counter()
                
                # Run multiple bursts
                burst_times = []
                for burst_idx in range(10):  # 10 bursts
                    burst_start = time.perf_counter()
                    
                    if async_processor:
                        event = FiredNeuronEvent(
                            timestep=burst_idx,
                            neurons_by_cortical=neurons_by_cortical
                        )
                        async_processor.process_fired_neurons(event)
                    else:
                        fcl_manager = cm._get_fcl_manager()
                        if fcl_manager:
                            fcl_manager.update_fcl(burst_idx, neurons_by_cortical)
                    
                    burst_end = time.perf_counter()
                    burst_times.append((burst_end - burst_start) * 1000)
                
                test_end = time.perf_counter()
                
                # Stop memory tracking
                memory_current, memory_peak = tracemalloc.get_traced_memory()
                tracemalloc.stop()
                memory_after = self.process.memory_info().rss / (1024 * 1024)
                
                # Calculate metrics
                execution_time = (test_end - test_start) * 1000
                throughput = total_firing_neurons * 10 / (execution_time / 1000)  # 10 bursts
                
                # Resource statistics
                cpu_samples = resource_data['cpu']
                memory_samples = resource_data['memory']
                
                cpu_avg = statistics.mean(cpu_samples) if cpu_samples else 0
                cpu_peak = max(cpu_samples) if cpu_samples else 0
                memory_avg = statistics.mean(memory_samples) if memory_samples else 0
                memory_peak_mb = max(memory_samples) if memory_samples else 0
                
                memory_growth_rate = (memory_after - memory_before) / 30  # MB/sec
                
                # Success criteria
                success = (
                    cpu_peak < self.cpu_saturation_threshold and
                    memory_peak_mb < (self.system_memory_gb * 1024 * self.memory_pressure_threshold) and
                    execution_time < 30000  # 30 seconds max
                )
                
                metric = StressTestMetrics(
                    test_name="extreme_neuron_scaling",
                    load_parameter=neuron_count,
                    success=success,
                    failure_reason=None if success else "Resource limits exceeded",
                    execution_time_ms=execution_time,
                    cpu_usage_avg_percent=cpu_avg,
                    cpu_usage_peak_percent=cpu_peak,
                    memory_usage_avg_mb=memory_avg,
                    memory_usage_peak_mb=memory_peak_mb,
                    memory_growth_rate_mb_per_sec=memory_growth_rate,
                    throughput_neurons_per_sec=throughput,
                    frequency_achieved_hz=10.0,  # Approximate
                    frequency_deviation_percent=0.0,
                    overrun_count=0,
                    error_count=0,
                    timestamp=time.time()
                )
                
                metrics.append(metric)
                
                # Log results
                status = "✅" if success else "❌"
                self.logger.info(
                    f"{status} {neuron_count:,} neurons: "
                    f"{execution_time:.1f}ms, "
                    f"CPU: {cpu_peak:.1f}%, "
                    f"Memory: {memory_peak_mb:.1f}MB, "
                    f"Throughput: {throughput:,.0f} neurons/sec"
                )
                
                # Clean up
                if async_processor:
                    async_processor.stop()
                del cm, be
                gc.collect()
                
                # Stop if we failed
                if not success:
                    self.logger.warning(f"Failure at {neuron_count:,} neurons - stopping scaling test")
                    break
                
            except Exception as e:
                self.logger.error(f"Exception at {neuron_count:,} neurons: {e}")
                
                metric = StressTestMetrics(
                    test_name="extreme_neuron_scaling",
                    load_parameter=neuron_count,
                    success=False,
                    failure_reason=str(e),
                    execution_time_ms=0,
                    cpu_usage_avg_percent=0,
                    cpu_usage_peak_percent=0,
                    memory_usage_avg_mb=0,
                    memory_usage_peak_mb=0,
                    memory_growth_rate_mb_per_sec=0,
                    throughput_neurons_per_sec=0,
                    frequency_achieved_hz=0,
                    frequency_deviation_percent=100,
                    overrun_count=0,
                    error_count=1,
                    timestamp=time.time()
                )
                
                metrics.append(metric)
                break
        
        return metrics
    
    def test_high_frequency_limits(self) -> List[StressTestMetrics]:
        """Test high-frequency burst processing until failure."""
        self.logger.info("🔥 Starting high-frequency limits stress test...")
        
        # Progressive frequency scaling
        frequencies = [50, 100, 200, 500, 1000, 2000, 5000]  # Hz
        neuron_count = 10000  # Fixed neuron count
        
        metrics = []
        
        for target_freq in frequencies:
            self.logger.info(f"Testing {target_freq}Hz frequency...")
            
            try:
                # Create system
                cm = ConnectomeManager(config_or_max_neurons=neuron_count)
                be = BurstEngine(cm)
                
                async_processor = cm._get_async_fcl_processor()
                if async_processor:
                    async_processor.start()
                    time.sleep(0.1)
                
                # Create firing pattern
                neurons_by_cortical = self._create_extreme_firing_pattern(neuron_count, 0.15)
                
                # Start resource monitoring
                resource_data = self._monitor_system_resources(10)  # 10 second test
                
                # High-frequency burst test
                target_interval = 1.0 / target_freq
                burst_times = []
                actual_frequencies = []
                overrun_count = 0
                
                test_start = time.perf_counter()
                last_burst_time = test_start
                burst_count = 0
                
                # Run for 10 seconds or until failure
                while (time.perf_counter() - test_start < 10.0 and burst_count < target_freq * 10):
                    next_burst_time = last_burst_time + target_interval
                    current_time = time.perf_counter()
                    
                    # Wait for next burst (if we're ahead)
                    if current_time < next_burst_time:
                        time.sleep(next_burst_time - current_time)
                    
                    # Execute burst
                    burst_start = time.perf_counter()
                    
                    if async_processor:
                        event = FiredNeuronEvent(
                            timestep=burst_count,
                            neurons_by_cortical=neurons_by_cortical
                        )
                        async_processor.process_fired_neurons(event)
                    
                    burst_end = time.perf_counter()
                    burst_duration = burst_end - burst_start
                    burst_times.append(burst_duration * 1000)
                    
                    # Calculate actual frequency
                    if burst_count > 0:
                        actual_interval = burst_end - last_burst_time
                        actual_freq = 1.0 / actual_interval if actual_interval > 0 else 0
                        actual_frequencies.append(actual_freq)
                    
                    # Check for overruns
                    if burst_duration > target_interval:
                        overrun_count += 1
                    
                    last_burst_time = burst_end
                    burst_count += 1
                
                test_end = time.perf_counter()
                
                # Calculate metrics
                execution_time = (test_end - test_start) * 1000
                avg_actual_freq = statistics.mean(actual_frequencies) if actual_frequencies else 0
                frequency_deviation = abs(avg_actual_freq - target_freq) / target_freq * 100
                
                total_neurons_processed = len([n for neurons in neurons_by_cortical.values() for n in neurons]) * burst_count
                throughput = total_neurons_processed / (execution_time / 1000)
                
                # Resource statistics
                cpu_samples = resource_data['cpu']
                memory_samples = resource_data['memory']
                
                cpu_avg = statistics.mean(cpu_samples) if cpu_samples else 0
                cpu_peak = max(cpu_samples) if cpu_samples else 0
                memory_avg = statistics.mean(memory_samples) if memory_samples else 0
                memory_peak_mb = max(memory_samples) if memory_samples else 0
                
                # Success criteria
                success = (
                    frequency_deviation < self.frequency_deviation_threshold and
                    cpu_peak < self.cpu_saturation_threshold and
                    overrun_count < burst_count * 0.1  # Less than 10% overruns
                )
                
                metric = StressTestMetrics(
                    test_name="high_frequency_limits",
                    load_parameter=target_freq,
                    success=success,
                    failure_reason=None if success else f"Frequency deviation: {frequency_deviation:.1f}%",
                    execution_time_ms=execution_time,
                    cpu_usage_avg_percent=cpu_avg,
                    cpu_usage_peak_percent=cpu_peak,
                    memory_usage_avg_mb=memory_avg,
                    memory_usage_peak_mb=memory_peak_mb,
                    memory_growth_rate_mb_per_sec=0,
                    throughput_neurons_per_sec=throughput,
                    frequency_achieved_hz=avg_actual_freq,
                    frequency_deviation_percent=frequency_deviation,
                    overrun_count=overrun_count,
                    error_count=0,
                    timestamp=time.time()
                )
                
                metrics.append(metric)
                
                # Log results
                status = "✅" if success else "❌"
                self.logger.info(
                    f"{status} {target_freq}Hz: "
                    f"Actual: {avg_actual_freq:.1f}Hz ({frequency_deviation:.1f}% deviation), "
                    f"CPU: {cpu_peak:.1f}%, "
                    f"Overruns: {overrun_count}/{burst_count}"
                )
                
                # Clean up
                if async_processor:
                    async_processor.stop()
                del cm, be
                gc.collect()
                
                # Stop if we failed
                if not success:
                    self.logger.warning(f"Failure at {target_freq}Hz - stopping frequency test")
                    break
                
            except Exception as e:
                self.logger.error(f"Exception at {target_freq}Hz: {e}")
                
                metric = StressTestMetrics(
                    test_name="high_frequency_limits",
                    load_parameter=target_freq,
                    success=False,
                    failure_reason=str(e),
                    execution_time_ms=0,
                    cpu_usage_avg_percent=0,
                    cpu_usage_peak_percent=0,
                    memory_usage_avg_mb=0,
                    memory_usage_peak_mb=0,
                    memory_growth_rate_mb_per_sec=0,
                    throughput_neurons_per_sec=0,
                    frequency_achieved_hz=0,
                    frequency_deviation_percent=100,
                    overrun_count=0,
                    error_count=1,
                    timestamp=time.time()
                )
                
                metrics.append(metric)
                break
        
        return metrics
    
    def test_concurrent_load_limits(self) -> List[StressTestMetrics]:
        """Test concurrent processing limits."""
        self.logger.info("🔥 Starting concurrent load limits stress test...")
        
        # Progressive concurrent load
        concurrent_levels = [2, 4, 8, 16, 32, 64]
        neuron_count = 50000
        
        metrics = []
        
        for concurrent_count in concurrent_levels:
            self.logger.info(f"Testing {concurrent_count} concurrent operations...")
            
            try:
                # Create systems
                systems = []
                for i in range(concurrent_count):
                    cm = ConnectomeManager(config_or_max_neurons=neuron_count)
                    be = BurstEngine(cm)
                    async_processor = cm._get_async_fcl_processor()
                    if async_processor:
                        async_processor.start()
                    systems.append((cm, be, async_processor))
                
                time.sleep(0.5)  # Let all systems initialize
                
                # Start resource monitoring
                resource_data = self._monitor_system_resources(15)  # 15 second test
                
                # Concurrent execution function
                def execute_concurrent_load(system_idx):
                    cm, be, async_processor = systems[system_idx]
                    neurons_by_cortical = self._create_extreme_firing_pattern(neuron_count, 0.1)
                    
                    burst_times = []
                    for burst_idx in range(20):  # 20 bursts per system
                        burst_start = time.perf_counter()
                        
                        if async_processor:
                            event = FiredNeuronEvent(
                                timestep=burst_idx,
                                neurons_by_cortical=neurons_by_cortical
                            )
                            async_processor.process_fired_neurons(event)
                        
                        burst_end = time.perf_counter()
                        burst_times.append((burst_end - burst_start) * 1000)
                        
                        time.sleep(0.01)  # Small delay between bursts
                    
                    return burst_times
                
                # Execute concurrent load
                test_start = time.perf_counter()
                
                with ThreadPoolExecutor(max_workers=concurrent_count) as executor:
                    futures = [executor.submit(execute_concurrent_load, i) for i in range(concurrent_count)]
                    results = [future.result() for future in as_completed(futures)]
                
                test_end = time.perf_counter()
                
                # Calculate metrics
                execution_time = (test_end - test_start) * 1000
                all_burst_times = [time for result in results for time in result]
                avg_burst_time = statistics.mean(all_burst_times) if all_burst_times else 0
                
                total_operations = len(all_burst_times)
                total_neurons_processed = neuron_count * 0.1 * total_operations  # 10% firing rate
                throughput = total_neurons_processed / (execution_time / 1000)
                
                # Resource statistics
                cpu_samples = resource_data['cpu']
                memory_samples = resource_data['memory']
                
                cpu_avg = statistics.mean(cpu_samples) if cpu_samples else 0
                cpu_peak = max(cpu_samples) if cpu_samples else 0
                memory_avg = statistics.mean(memory_samples) if memory_samples else 0
                memory_peak_mb = max(memory_samples) if memory_samples else 0
                
                # Success criteria
                success = (
                    cpu_peak < self.cpu_saturation_threshold and
                    memory_peak_mb < (self.system_memory_gb * 1024 * self.memory_pressure_threshold) and
                    avg_burst_time < 1000  # Less than 1 second per burst
                )
                
                metric = StressTestMetrics(
                    test_name="concurrent_load_limits",
                    load_parameter=concurrent_count,
                    success=success,
                    failure_reason=None if success else "Resource saturation",
                    execution_time_ms=execution_time,
                    cpu_usage_avg_percent=cpu_avg,
                    cpu_usage_peak_percent=cpu_peak,
                    memory_usage_avg_mb=memory_avg,
                    memory_usage_peak_mb=memory_peak_mb,
                    memory_growth_rate_mb_per_sec=0,
                    throughput_neurons_per_sec=throughput,
                    frequency_achieved_hz=0,
                    frequency_deviation_percent=0,
                    overrun_count=0,
                    error_count=0,
                    timestamp=time.time()
                )
                
                metrics.append(metric)
                
                # Log results
                status = "✅" if success else "❌"
                self.logger.info(
                    f"{status} {concurrent_count} concurrent: "
                    f"{execution_time:.1f}ms total, "
                    f"CPU: {cpu_peak:.1f}%, "
                    f"Memory: {memory_peak_mb:.1f}MB, "
                    f"Avg burst: {avg_burst_time:.1f}ms"
                )
                
                # Clean up
                for cm, be, async_processor in systems:
                    if async_processor:
                        async_processor.stop()
                    del cm, be
                
                gc.collect()
                
                # Stop if we failed
                if not success:
                    self.logger.warning(f"Failure at {concurrent_count} concurrent - stopping test")
                    break
                
            except Exception as e:
                self.logger.error(f"Exception at {concurrent_count} concurrent: {e}")
                
                metric = StressTestMetrics(
                    test_name="concurrent_load_limits",
                    load_parameter=concurrent_count,
                    success=False,
                    failure_reason=str(e),
                    execution_time_ms=0,
                    cpu_usage_avg_percent=0,
                    cpu_usage_peak_percent=0,
                    memory_usage_avg_mb=0,
                    memory_usage_peak_mb=0,
                    memory_growth_rate_mb_per_sec=0,
                    throughput_neurons_per_sec=0,
                    frequency_achieved_hz=0,
                    frequency_deviation_percent=0,
                    overrun_count=0,
                    error_count=1,
                    timestamp=time.time()
                )
                
                metrics.append(metric)
                break
        
        return metrics
    
    def analyze_bottlenecks(self, all_metrics: List[StressTestMetrics]) -> List[BottleneckAnalysis]:
        """Analyze bottlenecks from stress test results."""
        bottlenecks = []
        
        # Group metrics by test type
        metrics_by_test = {}
        for metric in all_metrics:
            if metric.test_name not in metrics_by_test:
                metrics_by_test[metric.test_name] = []
            metrics_by_test[metric.test_name].append(metric)
        
        # Analyze each test type
        for test_name, metrics in metrics_by_test.items():
            successful_metrics = [m for m in metrics if m.success]
            failed_metrics = [m for m in metrics if not m.success]
            
            if not successful_metrics:
                continue
            
            # Find breaking points
            if failed_metrics:
                first_failure = failed_metrics[0]
                last_success = successful_metrics[-1] if successful_metrics else None
                
                if last_success:
                    # CPU bottleneck analysis
                    if first_failure.cpu_usage_peak_percent > self.cpu_saturation_threshold:
                        bottlenecks.append(BottleneckAnalysis(
                            bottleneck_type="cpu",
                            threshold_value=last_success.load_parameter,
                            description=f"CPU saturation at {first_failure.load_parameter} ({first_failure.cpu_usage_peak_percent:.1f}% CPU)",
                            recommendations=[
                                "Implement parallel processing",
                                "Optimize CPU-intensive operations",
                                "Consider multi-threading improvements"
                            ],
                            severity="high"
                        ))
                    
                    # Memory bottleneck analysis
                    if first_failure.memory_usage_peak_mb > (self.system_memory_gb * 1024 * 0.8):
                        bottlenecks.append(BottleneckAnalysis(
                            bottleneck_type="memory",
                            threshold_value=last_success.load_parameter,
                            description=f"Memory pressure at {first_failure.load_parameter} ({first_failure.memory_usage_peak_mb:.1f}MB)",
                            recommendations=[
                                "Implement memory pooling",
                                "Optimize data structures",
                                "Add memory cleanup mechanisms"
                            ],
                            severity="high"
                        ))
                    
                    # Frequency bottleneck analysis
                    if test_name == "high_frequency_limits" and first_failure.frequency_deviation_percent > 30:
                        bottlenecks.append(BottleneckAnalysis(
                            bottleneck_type="frequency",
                            threshold_value=last_success.load_parameter,
                            description=f"Frequency degradation at {first_failure.load_parameter}Hz ({first_failure.frequency_deviation_percent:.1f}% deviation)",
                            recommendations=[
                                "Optimize burst processing pipeline",
                                "Reduce processing latency",
                                "Implement real-time scheduling"
                            ],
                            severity="medium"
                        ))
        
        return bottlenecks
    
    def save_stress_test_results(self, all_metrics: List[StressTestMetrics], 
                               bottlenecks: List[BottleneckAnalysis]) -> str:
        """Save stress test results."""
        timestamp = int(time.time())
        filename = f"stress_test_results_{timestamp}.json"
        filepath = self.results_dir / filename
        
        results = {
            'test_type': 'stress_test_suite',
            'system_info': {
                'cpu_count': self.cpu_count,
                'memory_gb': self.system_memory_gb,
                'platform': os.uname().sysname if hasattr(os, 'uname') else 'Unknown'
            },
            'metrics': [m.to_dict() for m in all_metrics],
            'bottlenecks': [asdict(b) for b in bottlenecks],
            'thresholds': {
                'cpu_saturation': self.cpu_saturation_threshold,
                'memory_pressure': self.memory_pressure_threshold,
                'frequency_deviation': self.frequency_deviation_threshold
            },
            'timestamp': time.time()
        }
        
        with open(filepath, 'w') as f:
            json.dump(results, f, indent=2)
        
        self.logger.info(f"Saved stress test results to {filepath}")
        return str(filepath)
    
    def print_stress_test_report(self, all_metrics: List[StressTestMetrics], 
                               bottlenecks: List[BottleneckAnalysis]):
        """Print comprehensive stress test report."""
        print("\n" + "="*80)
        print("🔥 FEAGI STRESS TEST REPORT - SYSTEM LIMITS ANALYSIS")
        print("="*80)
        
        # Group metrics by test type
        metrics_by_test = {}
        for metric in all_metrics:
            if metric.test_name not in metrics_by_test:
                metrics_by_test[metric.test_name] = []
            metrics_by_test[metric.test_name].append(metric)
        
        # Report each test type
        for test_name, metrics in metrics_by_test.items():
            successful_metrics = [m for m in metrics if m.success]
            failed_metrics = [m for m in metrics if not m.success]
            
            print(f"\n🧪 {test_name.upper().replace('_', ' ')}:")
            print("-" * 60)
            
            if successful_metrics:
                best_metric = max(successful_metrics, key=lambda m: m.load_parameter)
                print(f"   ✅ Maximum successful load: {best_metric.load_parameter}")
                print(f"   📊 Peak CPU: {best_metric.cpu_usage_peak_percent:.1f}%")
                print(f"   💾 Peak Memory: {best_metric.memory_usage_peak_mb:.1f}MB")
                print(f"   ⚡ Throughput: {best_metric.throughput_neurons_per_sec:,.0f} neurons/sec")
            
            if failed_metrics:
                first_failure = failed_metrics[0]
                print(f"   ❌ First failure at: {first_failure.load_parameter}")
                print(f"   🚫 Failure reason: {first_failure.failure_reason}")
        
        # Bottleneck analysis
        if bottlenecks:
            print(f"\n🚨 BOTTLENECKS IDENTIFIED:")
            print("-" * 40)
            
            for bottleneck in bottlenecks:
                severity_emoji = {"low": "🟢", "medium": "🟡", "high": "🟠", "critical": "🔴"}
                emoji = severity_emoji.get(bottleneck.severity, "⚪")
                
                print(f"\n{emoji} {bottleneck.bottleneck_type.upper()} BOTTLENECK ({bottleneck.severity.upper()}):")
                print(f"   Threshold: {bottleneck.threshold_value}")
                print(f"   Description: {bottleneck.description}")
                print(f"   Recommendations:")
                for rec in bottleneck.recommendations:
                    print(f"     • {rec}")
        else:
            print(f"\n✅ No critical bottlenecks identified within test limits")
        
        # System summary
        all_cpu_peaks = [m.cpu_usage_peak_percent for m in all_metrics if m.success]
        all_memory_peaks = [m.memory_usage_peak_mb for m in all_metrics if m.success]
        all_throughputs = [m.throughput_neurons_per_sec for m in all_metrics if m.success and m.throughput_neurons_per_sec > 0]
        
        if all_cpu_peaks and all_memory_peaks and all_throughputs:
            print(f"\n📈 SYSTEM PERFORMANCE SUMMARY:")
            print(f"   Maximum CPU Usage: {max(all_cpu_peaks):.1f}%")
            print(f"   Maximum Memory Usage: {max(all_memory_peaks):.1f}MB")
            print(f"   Maximum Throughput: {max(all_throughputs):,.0f} neurons/sec")
            print(f"   System Utilization: {max(all_cpu_peaks)/100*100:.1f}% CPU, {max(all_memory_peaks)/(self.system_memory_gb*1024)*100:.1f}% Memory")
        
        print("\n" + "="*80)
    
    def run_complete_stress_test_suite(self) -> str:
        """Run the complete stress test suite."""
        self.logger.info("🔥 Starting complete FEAGI stress test suite...")
        
        print("🔥 FEAGI STRESS TEST SUITE - PUSHING TO THE LIMITS")
        print("=" * 60)
        
        all_metrics = []
        
        # 1. Extreme neuron scaling
        print("\n1️⃣ Extreme Neuron Scaling Test...")
        neuron_metrics = self.test_extreme_neuron_scaling()
        all_metrics.extend(neuron_metrics)
        
        # 2. High-frequency limits
        print("\n2️⃣ High-Frequency Limits Test...")
        frequency_metrics = self.test_high_frequency_limits()
        all_metrics.extend(frequency_metrics)
        
        # 3. Concurrent load limits
        print("\n3️⃣ Concurrent Load Limits Test...")
        concurrent_metrics = self.test_concurrent_load_limits()
        all_metrics.extend(concurrent_metrics)
        
        # 4. Analyze bottlenecks
        print("\n4️⃣ Analyzing Bottlenecks...")
        bottlenecks = self.analyze_bottlenecks(all_metrics)
        
        # 5. Save results
        results_file = self.save_stress_test_results(all_metrics, bottlenecks)
        
        # 6. Print report
        self.print_stress_test_report(all_metrics, bottlenecks)
        
        return results_file


def run_stress_test_suite() -> str:
    """Standalone function to run stress test suite."""
    stress_tester = StressTestSuite()
    return stress_tester.run_complete_stress_test_suite()


if __name__ == "__main__":
    # Run the stress test suite when executed directly
    results_file = run_stress_test_suite()
    print(f"\n🔥 Stress test completed! Results saved to: {results_file}")
