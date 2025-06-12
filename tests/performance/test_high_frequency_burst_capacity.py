#!/usr/bin/env python3
"""
High Frequency Burst Capacity Test - 15Hz Performance Measurement

This test measures how many neuron operations FEAGI can handle per burst
when running at 15Hz (66.7ms per burst), which is 50% faster than the
default 10Hz frequency.

Test objectives:
1. Measure maximum neuron operations per burst at 15Hz
2. Find the performance ceiling before load shedding kicks in
3. Analyze processing time distribution and bottlenecks
4. Generate performance curves for different neuron counts
"""

import json
import statistics
import time
from pathlib import Path
from typing import Any, Dict, List, Tuple
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.core.state_manager import FeagiStateManager
from feagi.npu.burst_engine import BurstEngine
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class HighFrequencyBurstCapacityTest:
    """Test suite for measuring 15Hz burst capacity."""

    def __init__(self):
        self.target_frequency = 15.0  # 15Hz target
        self.max_burst_time = 1.0 / self.target_frequency  # 66.7ms max
        self.test_results = {}

        # EMBEDDED MODE CONSTRAINTS - CRITICAL FOR ACCURATE TESTING
        self.embedded_mode = True
        self.single_cpu_core = True
        self.disable_zmq = True
        self.disable_http = True
        self.single_threaded = True
        self.cpu_core_id = 0  # Restrict to CPU core 0

        # Test configurations - different neuron counts to test
        # TARGET: 10M neuron operations per burst at 15Hz ON SINGLE CPU CORE
        self.test_neuron_counts = [
            1000,  # Baseline
            5000,  # Small scale
            10000,  # 10K
            25000,  # 25K
            50000,  # 50K
            100000,  # 100K
            250000,  # 250K
            500000,  # 500K
            1000000,  # 1M - Major milestone
            2500000,  # 2.5M
            5000000,  # 5M
            7500000,  # 7.5M
            10000000,  # 10M - TARGET!
        ]

        # Performance optimization settings for embedded/single-core
        self.target_operations_per_burst = 10_000_000  # 10M target
        self.enable_simd_optimizations = True  # Critical for single core performance
        self.enable_gpu_acceleration = False  # Not available in embedded mode
        self.monitor_memory_bandwidth = True

        # Burst measurement parameters
        self.measurement_duration = 10.0  # seconds
        self.min_bursts_to_measure = 50  # minimum bursts to collect

    def setup_embedded_environment(self):
        """Configure the test environment for embedded mode constraints."""
        import os

        import psutil

        logger.info("Configuring embedded mode test environment...")

        # 1. Set CPU affinity to single core
        if self.single_cpu_core:
            try:
                current_process = psutil.Process()
                current_process.cpu_affinity([self.cpu_core_id])
                logger.info(f"✅ CPU affinity set to core {self.cpu_core_id}")
            except Exception as e:
                logger.warning(f"⚠️  Could not set CPU affinity: {e}")

        # 2. Disable threading for embedded mode
        if self.single_threaded:
            os.environ["OMP_NUM_THREADS"] = "1"
            os.environ["MKL_NUM_THREADS"] = "1"
            os.environ["NUMEXPR_NUM_THREADS"] = "1"
            os.environ["NUMBA_NUM_THREADS"] = "1"
            logger.info("✅ Single-threaded mode enabled")

        # 3. Configure embedded mode flags
        os.environ["FEAGI_EMBEDDED_MODE"] = "true"
        os.environ["FEAGI_DISABLE_ZMQ"] = "true"
        os.environ["FEAGI_DISABLE_HTTP"] = "true"
        os.environ["FEAGI_SINGLE_CORE"] = "true"
        logger.info("✅ Embedded mode environment variables set")

        # 4. Memory optimizations for embedded
        import gc

        gc.set_threshold(700, 10, 10)  # More aggressive garbage collection
        logger.info("✅ Memory optimizations configured")

        logger.info("🎯 Embedded mode environment ready for 10M neuron test")

        return True

    def setup_test_brain(
        self, neuron_count: int
    ) -> Tuple[ConnectomeManager, BurstEngine]:
        """Set up a test brain with specified neuron count optimized for 10M target."""
        logger.info(
            f"Setting up test brain with {neuron_count:,} neurons (target: {self.target_operations_per_burst:,})"
        )

        # Reset singletons
        BurstEngine.reset_singleton()

        # Create test connectome with optimizations
        connectome = ConnectomeManager.instance()

        # For large neuron counts, create multiple areas to distribute load
        areas_count = max(1, neuron_count // 1000000)  # 1 area per 1M neurons
        neurons_per_area = neuron_count // areas_count

        logger.info(
            f"Creating {areas_count} areas with ~{neurons_per_area:,} neurons each"
        )

        all_neuron_ids = []

        for area_idx in range(areas_count):
            area_id = area_idx + 1

            # Calculate remaining neurons for last area
            if area_idx == areas_count - 1:
                area_neuron_count = neuron_count - (area_idx * neurons_per_area)
            else:
                area_neuron_count = neurons_per_area

            # Optimize area dimensions for memory layout
            area_size = int(np.cbrt(area_neuron_count)) + 1

            area = connectome.add_cortical_area(
                area_id=area_id,
                name=f"PerfTestArea_{area_idx}_{area_neuron_count}",
                area_type="test",
                dimensions=(area_size, area_size, area_size),
                position=(area_idx * 100, 0, 0),  # Spread areas spatially
            )

            # Create neurons in efficient batch
            logger.info(f"Creating {area_neuron_count:,} neurons in area {area_id}")

            positions = []
            for i in range(area_neuron_count):
                x = i % area_size
                y = (i // area_size) % area_size
                z = i // (area_size * area_size)
                positions.append((x, y, z))

            # Use vectorized batch creation for large counts
            if hasattr(connectome, "batch_create_neurons_vectorized"):
                area_neuron_ids = connectome.batch_create_neurons_vectorized(
                    area_id, positions
                )
            elif hasattr(connectome, "batch_create_neurons"):
                area_neuron_ids = connectome.batch_create_neurons(area_id, positions)
            else:
                # Fallback to individual creation (will be slow for large counts)
                area_neuron_ids = []
                batch_size = 1000
                for i in range(0, len(positions), batch_size):
                    batch_positions = positions[i : i + batch_size]
                    for pos in batch_positions:
                        nid = connectome.create_neuron(area_id, pos)
                        area_neuron_ids.append(nid)

            all_neuron_ids.extend(area_neuron_ids)

        # Smart connectivity for large scale - reduce density for larger networks
        if neuron_count <= 10000:
            connectivity_density = 0.01  # 1% for small networks
        elif neuron_count <= 100000:
            connectivity_density = 0.005  # 0.5% for medium networks
        elif neuron_count <= 1000000:
            connectivity_density = 0.001  # 0.1% for large networks
        else:
            connectivity_density = 0.0001  # 0.01% for massive networks (10M target)

        connection_count = int(neuron_count * connectivity_density)
        connection_count = min(
            connection_count, 100000
        )  # Cap at 100K connections for performance

        logger.info(
            f"Creating {connection_count:,} synapses ({connectivity_density:.4%} density)"
        )

        # Batch create synapses for efficiency
        synapse_specs = []
        for _ in range(connection_count):
            pre_neuron = np.random.choice(all_neuron_ids)
            post_neuron = np.random.choice(all_neuron_ids)
            weight = np.random.uniform(0.1, 0.5)
            synapse_specs.append((pre_neuron, post_neuron, weight))

        # Use batch synapse creation if available
        if hasattr(connectome, "batch_create_synapses"):
            connectome.batch_create_synapses(synapse_specs)
        else:
            # Create in batches to avoid memory issues
            batch_size = 1000
            for i in range(0, len(synapse_specs), batch_size):
                batch = synapse_specs[i : i + batch_size]
                for pre, post, weight in batch:
                    try:
                        connectome.create_synapse(pre, post, weight)
                    except Exception:
                        pass  # Skip if connection fails

        # Create burst engine with EMBEDDED MODE configuration
        config = {
            "desired_frequency_hz": self.target_frequency,
            "target_frequency": self.target_frequency,
            "debug_npu": False,
            "performance_monitoring": True,
            # EMBEDDED MODE SETTINGS
            "embedded_mode": True,
            "disable_zmq": True,
            "disable_http_server": True,
            "single_threaded": True,
            "single_core_mode": True,
            # Performance optimizations for single-core
            "simd_profiling": self.enable_simd_optimizations,
            "use_gpu": False,  # No GPU in embedded mode
            "use_multiprocessing": False,  # Single process only
            "batch_size": min(
                10000, neuron_count // 20
            ),  # Smaller batches for single core
            "memory_optimized": True,
            "sequential_processing": True,  # Force sequential execution
        }

        burst_engine = BurstEngine(
            connectome_manager=connectome,
            fcl_manager=connectome.fcl_manager,
            config=config,
        )

        # Mark genome as loaded
        burst_engine.update_with_genome()

        logger.info(
            f"Test brain setup complete: {neuron_count:,} neurons, "
            f"{connection_count:,} synapses, {self.target_frequency}Hz frequency"
        )

        # Calculate target achievement percentage
        target_percentage = (neuron_count / self.target_operations_per_burst) * 100
        logger.info(f"Target achievement: {target_percentage:.1f}% of 10M neuron goal")

        return connectome, burst_engine

    def measure_burst_performance(
        self, burst_engine: BurstEngine, neuron_count: int
    ) -> Dict[str, Any]:
        """Measure burst performance over time with 10M target analysis."""
        target_progress = (neuron_count / self.target_operations_per_burst) * 100
        logger.info(
            f"Measuring burst performance for {neuron_count:,} neurons at {self.target_frequency}Hz "
            f"({target_progress:.1f}% of 10M target)"
        )

        # Enable frequency measurement
        burst_engine.enable_frequency_measurement(True)

        # Collect burst timing data
        burst_times = []
        processing_times = []
        neurons_fired_counts = []
        overrun_count = 0

        measurement_start = time.perf_counter()
        burst_count_start = burst_engine.burst_count

        # Collect data for specified duration
        while (
            time.perf_counter() - measurement_start < self.measurement_duration
            and len(burst_times) < self.min_bursts_to_measure * 2
        ):
            # Run a single test burst
            burst_start = time.perf_counter()
            fired_neurons = burst_engine.run_test()
            burst_end = time.perf_counter()

            burst_duration = burst_end - burst_start
            burst_times.append(burst_duration)
            processing_times.append(burst_duration)  # For this test, they're the same
            neurons_fired_counts.append(len(fired_neurons))

            # Check for timing overruns
            if burst_duration > self.max_burst_time:
                overrun_count += 1

            # Log progress every 10 bursts
            if len(burst_times) % 10 == 0:
                avg_time = np.mean(burst_times[-10:]) * 1000
                logger.info(f"  {len(burst_times)} bursts: {avg_time:.2f}ms avg")

        measurement_duration = time.perf_counter() - measurement_start
        burst_count_end = burst_engine.burst_count
        total_bursts = (
            burst_count_end - burst_count_start + len(burst_times)
        )  # Include test bursts

        # Calculate performance metrics with 10M target analysis
        if not burst_times:
            raise RuntimeError("No burst timing data collected")

        burst_times_ms = [t * 1000 for t in burst_times]

        # Target achievement analysis
        target_progress = (neuron_count / self.target_operations_per_burst) * 100
        avg_burst_time_ms = np.mean(burst_times_ms)
        estimated_time_at_10m = avg_burst_time_ms * (
            self.target_operations_per_burst / neuron_count
        )
        scalability_feasible = estimated_time_at_10m <= (self.max_burst_time * 1000)

        # Memory efficiency (estimate)
        memory_per_neuron_bytes = 64  # Rough estimate
        estimated_memory_10m_gb = (
            self.target_operations_per_burst * memory_per_neuron_bytes
        ) / (1024**3)

        operations_per_second = neuron_count * (len(burst_times) / measurement_duration)
        megaops_per_second = operations_per_second / 1_000_000

        results = {
            # Basic measurements
            "neuron_count": neuron_count,
            "target_frequency_hz": self.target_frequency,
            "max_burst_time_ms": self.max_burst_time * 1000,
            "measurement_duration_s": measurement_duration,
            "total_bursts_measured": len(burst_times),
            # Timing statistics
            "avg_burst_time_ms": avg_burst_time_ms,
            "min_burst_time_ms": np.min(burst_times_ms),
            "max_burst_time_ms": np.max(burst_times_ms),
            "std_burst_time_ms": np.std(burst_times_ms),
            "median_burst_time_ms": np.median(burst_times_ms),
            # Performance metrics
            "overrun_count": overrun_count,
            "overrun_percentage": (overrun_count / len(burst_times)) * 100,
            "avg_actual_frequency_hz": len(burst_times) / measurement_duration,
            "frequency_efficiency": (len(burst_times) / measurement_duration)
            / self.target_frequency,
            # Neural activity vs Processing metrics
            "avg_neurons_fired": np.mean(
                neurons_fired_counts
            ),  # Only neurons that actually fired
            "max_neurons_fired": np.max(
                neurons_fired_counts
            ),  # Peak firing in a single burst
            "total_neurons_fired": np.sum(
                neurons_fired_counts
            ),  # Total firings across all bursts
            # CRITICAL: Operations metrics - THIS IS THE 10M TARGET
            "neuron_operations_per_burst": neuron_count,  # ALL neurons processed per burst
            "neuron_operations_per_second": operations_per_second,  # Total processing load
            "megaops_per_second": megaops_per_second,  # Millions of operations per second
            # What constitutes a "neuron operation":
            # 1. Membrane potential decay (leak)
            # 2. Refractory period update
            # 3. Input current integration
            # 4. Threshold detection
            # 5. Firing logic (if applicable)
            # 6. Synaptic propagation (if fired)
            # = ~6 micro-operations per neuron per burst
            # 10M Target Analysis
            "target_operations_per_burst": self.target_operations_per_burst,
            "target_progress_percent": target_progress,
            "estimated_time_at_10m_ms": estimated_time_at_10m,
            "scalability_feasible": scalability_feasible,
            "estimated_memory_10m_gb": estimated_memory_10m_gb,
            # Performance classification
            "performance_level": self._classify_performance_with_target(
                overrun_count, len(burst_times), target_progress
            ),
        }

        # Add percentile data
        percentiles = [50, 75, 90, 95, 99]
        for p in percentiles:
            results[f"p{p}_burst_time_ms"] = np.percentile(burst_times_ms, p)

        logger.info(
            f"Performance measurement complete: {results['avg_burst_time_ms']:.2f}ms avg, "
            f"{results['overrun_percentage']:.1f}% overruns"
        )

        return results

    def _classify_performance_with_target(
        self, overrun_count: int, total_bursts: int, target_progress: float
    ) -> str:
        """Classify performance level based on timing overruns and 10M target progress."""
        overrun_percentage = (overrun_count / total_bursts) * 100

        # Special classifications for high target progress
        if target_progress >= 100:  # 10M+ neurons achieved!
            if overrun_percentage == 0:
                return "TARGET_ACHIEVED"
            elif overrun_percentage < 10:
                return "TARGET_NEAR"
            else:
                return "TARGET_FAILED"
        elif target_progress >= 75:  # 7.5M+ neurons (close to target)
            if overrun_percentage == 0:
                return "EXCELLENT_SCALING"
            elif overrun_percentage < 5:
                return "GOOD_SCALING"
            else:
                return "POOR_SCALING"
        elif target_progress >= 50:  # 5M+ neurons (halfway to target)
            if overrun_percentage == 0:
                return "EXCELLENT_PROGRESS"
            elif overrun_percentage < 5:
                return "GOOD_PROGRESS"
            else:
                return "POOR_PROGRESS"

        # Standard classifications for lower scales
        if overrun_percentage == 0:
            return "EXCELLENT"
        elif overrun_percentage < 5:
            return "GOOD"
        elif overrun_percentage < 15:
            return "ACCEPTABLE"
        elif overrun_percentage < 30:
            return "POOR"
        else:
            return "FAILING"

    def _classify_performance(self, overrun_count: int, total_bursts: int) -> str:
        """Legacy classification method - use _classify_performance_with_target instead."""
        return self._classify_performance_with_target(overrun_count, total_bursts, 0)

    def find_maximum_capacity(self) -> Dict[str, Any]:
        """Find the maximum neuron count that can be processed at 15Hz in embedded mode."""
        logger.info(
            "🎯 Finding maximum neuron capacity at 15Hz in EMBEDDED MODE (single CPU core)"
        )

        # CRITICAL: Setup embedded environment first
        self.setup_embedded_environment()

        max_capacity_result = None
        last_good_count = 0

        for neuron_count in self.test_neuron_counts:
            try:
                logger.info(f"Testing {neuron_count:,} neurons in embedded mode...")

                # Setup test brain
                connectome, burst_engine = self.setup_test_brain(neuron_count)

                # Measure performance under embedded constraints
                result = self.measure_burst_performance(burst_engine, neuron_count)
                self.test_results[neuron_count] = result

                # Check if performance is acceptable
                if result["performance_level"] in [
                    "EXCELLENT",
                    "GOOD",
                    "ACCEPTABLE",
                    "EXCELLENT_SCALING",
                    "GOOD_SCALING",
                    "EXCELLENT_PROGRESS",
                    "GOOD_PROGRESS",
                    "TARGET_ACHIEVED",
                    "TARGET_NEAR",
                ]:
                    last_good_count = neuron_count
                    max_capacity_result = result
                    target_pct = result.get("target_progress_percent", 0)
                    logger.info(
                        f"✅ {neuron_count:,} neurons: {result['performance_level']} "
                        f"({result['avg_burst_time_ms']:.2f}ms avg, {target_pct:.1f}% of 10M target)"
                    )
                else:
                    logger.warning(
                        f"❌ {neuron_count:,} neurons: {result['performance_level']} "
                        f"({result['overrun_percentage']:.1f}% overruns)"
                    )
                    break

            except Exception as e:
                logger.error(f"💥 Error testing {neuron_count:,} neurons: {e}")
                break
            finally:
                # Cleanup
                try:
                    BurstEngine.reset_singleton()
                    ConnectomeManager.reset_singleton()
                except:
                    pass

        logger.info(
            f"🏁 Embedded mode test complete. Max capacity: {last_good_count:,} neurons"
        )

        return {
            "max_neuron_capacity": last_good_count,
            "max_capacity_result": max_capacity_result,
            "all_results": self.test_results,
            "embedded_mode": True,
            "single_cpu_core": True,
            "target_operations": self.target_operations_per_burst,
        }

    def generate_performance_report(self, results: Dict[str, Any]) -> str:
        """Generate a detailed performance report focused on 10M target."""
        report = []
        report.append("=" * 80)
        report.append(
            "FEAGI 15Hz BURST CAPACITY PERFORMANCE REPORT - 10M TARGET ANALYSIS"
        )
        report.append("=" * 80)
        report.append("")

        max_capacity = results["max_neuron_capacity"]
        max_result = results["max_capacity_result"]

        # Target analysis
        report.append(
            f"🎯 TARGET: {self.target_operations_per_burst:,} neuron operations per burst at 15Hz"
        )
        report.append(
            f"⏱️  TIME BUDGET: {self.max_burst_time * 1000:.1f}ms per burst (66.7ms)"
        )
        report.append("")

        if max_result:
            target_progress = max_result.get("target_progress_percent", 0)
            performance_level = max_result.get("performance_level", "UNKNOWN")
            megaops = max_result.get("megaops_per_second", 0)

            report.append(f"📊 MAXIMUM CAPACITY: {max_capacity:,} neurons at 15Hz")
            report.append(f"   Target Progress: {target_progress:.1f}% of 10M goal")
            report.append(f"   Performance Level: {performance_level}")
            report.append(
                f"   Average burst time: {max_result['avg_burst_time_ms']:.2f}ms"
            )
            report.append(
                f"   Processing efficiency: {max_result['frequency_efficiency']:.1%}"
            )
            report.append(f"   Performance: {megaops:.2f} MOps/sec")

            # Scalability analysis
            if "estimated_time_at_10m_ms" in max_result:
                est_time_10m = max_result["estimated_time_at_10m_ms"]
                feasible = max_result.get("scalability_feasible", False)
                memory_gb = max_result.get("estimated_memory_10m_gb", 0)

                report.append("")
                report.append(f"🔮 SCALABILITY TO 10M TARGET:")
                report.append(f"   Estimated time at 10M: {est_time_10m:.1f}ms")
                report.append(
                    f"   Within time budget: {'✅ YES' if feasible else '❌ NO'}"
                )
                report.append(f"   Estimated memory: {memory_gb:.1f}GB")

            report.append("")

        # Performance progression table
        report.append("📈 PERFORMANCE PROGRESSION:")
        report.append("")
        report.append(
            "Neuron Count | Target % | Avg Time | Performance Level   | MOps/sec"
        )
        report.append("-" * 70)

        all_results = results.get("all_results", {})
        for neuron_count in sorted(all_results.keys()):
            result = all_results[neuron_count]
            target_pct = result.get("target_progress_percent", 0)
            avg_time = result.get("avg_burst_time_ms", 0)
            perf_level = result.get("performance_level", "N/A")[:18]
            megaops = result.get("megaops_per_second", 0)

            report.append(
                f"{neuron_count:>11,} | {target_pct:>7.1f}% | {avg_time:>8.2f}ms | {perf_level:<18} | {megaops:>7.2f}"
            )

        report.append("")
        report.append("=" * 80)

        # Critical insights
        if max_result:
            report.append("🔍 CRITICAL INSIGHTS:")

            if target_progress >= 100:
                report.append(
                    "✅ TARGET ACHIEVED! 10M neuron operations per burst at 15Hz"
                )
            elif target_progress >= 75:
                report.append(
                    f"🔥 Excellent scaling: {target_progress:.1f}% of target achieved"
                )
                report.append("   Minor optimizations may reach 10M target")
            elif target_progress >= 50:
                report.append(
                    f"⚡ Good progress: {target_progress:.1f}% of target achieved"
                )
                report.append("   Significant optimization needed for 10M target")
            elif target_progress >= 25:
                report.append(f"⚠️  Moderate progress: {target_progress:.1f}% of target")
                report.append("   Major architectural changes needed for 10M")
            else:
                report.append(
                    f"❌ Limited scaling: Only {target_progress:.1f}% of 10M target"
                )
                report.append("   GPU/SIMD acceleration essential for 10M")

            # Performance bottleneck analysis
            avg_time = max_result.get("avg_burst_time_ms", 0)
            time_budget = self.max_burst_time * 1000

            if avg_time > time_budget * 0.9:
                report.append("⏰ BOTTLENECK: Processing time near limit")
            elif avg_time > time_budget * 0.7:
                report.append("⚠️  High processing time - optimization recommended")
            else:
                report.append("✅ Processing time within acceptable range")

            report.append("")

        return "\n".join(report)


@pytest.mark.performance
@pytest.mark.slow
def test_15hz_burst_capacity():
    """
    Test FEAGI's maximum neuron capacity at 15Hz burst frequency.

    🎯 TARGET: 10 million neuron operations per burst at 15Hz
    🖥️  CONSTRAINTS: Single CPU core, embedded mode, no ZMQ/HTTP
    ⏱️  TIME BUDGET: 66.7ms per burst

    This test simulates real embedded deployment conditions to measure
    if FEAGI can achieve 10M neuron operations per burst when running
    on a single CPU core without network communication.
    """
    logger.info("=" * 80)
    logger.info("🚀 STARTING FEAGI 15Hz EMBEDDED MODE CAPACITY TEST")
    logger.info("🎯 TARGET: 10M neuron operations per burst")
    logger.info("🖥️  MODE: Single CPU core, no ZMQ, no HTTP")
    logger.info("⏱️  BUDGET: 66.7ms per burst")
    logger.info("=" * 80)

    # Initialize test suite
    test_suite = HighFrequencyBurstCapacityTest()

    try:
        # Run capacity finding test
        results = test_suite.find_maximum_capacity()

        # Generate detailed report
        report = test_suite.generate_performance_report(results)

        # Log results
        logger.info("\n" + report)

        # Save results to file for analysis
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        report_file = (
            f"feagi_core/tests/performance/logs/15hz_embedded_capacity_{timestamp}.txt"
        )

        # Ensure directory exists
        Path(report_file).parent.mkdir(parents=True, exist_ok=True)

        with open(report_file, "w") as f:
            f.write(report)

        logger.info(f"📄 Detailed report saved to: {report_file}")

        # Test assertions for CI/CD
        max_capacity = results["max_neuron_capacity"]
        target_progress = 0

        if results["max_capacity_result"]:
            target_progress = results["max_capacity_result"].get(
                "target_progress_percent", 0
            )

        logger.info(
            f"🏆 FINAL RESULT: {max_capacity:,} neurons ({target_progress:.1f}% of 10M target)"
        )

        # Assertions based on embedded mode expectations
        assert max_capacity > 0, (
            "Should be able to process at least some neurons in embedded mode"
        )

        if target_progress >= 100:
            logger.info(
                "🎉 TARGET ACHIEVED! 10M neuron operations per burst at 15Hz in embedded mode!"
            )
        elif target_progress >= 50:
            logger.info(
                f"⚡ Good progress: {target_progress:.1f}% of 10M target achieved"
            )
        elif target_progress >= 10:
            logger.info(
                f"🔧 Optimization needed: Only {target_progress:.1f}% of 10M target"
            )
        else:
            logger.warning(
                f"⚠️  Significant architectural changes needed: {target_progress:.1f}% of target"
            )

        # Performance recommendations
        if target_progress < 100:
            logger.info("💡 OPTIMIZATION RECOMMENDATIONS:")
            logger.info("   - SIMD vectorization critical for single-core performance")
            logger.info("   - Memory layout optimization for cache efficiency")
            logger.info("   - Algorithm optimization for embedded constraints")

        return results

    except Exception as e:
        logger.error(f"💥 Test failed with error: {e}")
        raise


if __name__ == "__main__":
    # Run the test directly
    test_results = test_15hz_burst_capacity()

    max_capacity = test_results["max_neuron_capacity"]
    print(
        f"\n🎯 RESULT: FEAGI can handle {max_capacity:,} neuron operations per burst at 15Hz"
    )
    print(f"📊 This equals {max_capacity * 15:,} neuron operations per second")
