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
FEAGI Resource Profiler

Detailed profiling tool to identify resource consumption by component.
Critical for embedded device optimization.
"""

import threading
import tracemalloc
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import psutil

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


@dataclass
class ComponentResourceUsage:
    """Resource usage for a specific component."""

    name: str
    memory_mb: float
    cpu_percent: float
    thread_count: int
    file_descriptors: int
    network_connections: int

    def memory_per_neuron_kb(self, neuron_count: int) -> float:
        """Calculate memory usage per neuron in KB."""
        if neuron_count == 0:
            return 0.0
        return (self.memory_mb * 1024) / neuron_count


class ResourceProfiler:
    """
    Detailed resource profiler for FEAGI components.

    Identifies resource consumption by:
    - Process components (API, ZMQ, BurstEngine, etc.)
    - Memory allocation patterns
    - Thread overhead
    - Network resource usage
    """

    def __init__(self):
        self.process = psutil.Process()
        self.baseline_memory = 0.0
        self.baseline_cpu = 0.0
        self.component_snapshots = {}
        self.tracemalloc_enabled = False

    def start_profiling(self):
        """Start detailed profiling with memory tracing."""
        # Enable tracemalloc for detailed memory tracking
        if not tracemalloc.is_tracing():
            tracemalloc.start()
            self.tracemalloc_enabled = True
            logger.info("[SEARCH] Started memory tracing")

        # Capture baseline
        self.baseline_memory = self.process.memory_info().rss / (1024 * 1024)
        self.baseline_cpu = self.process.cpu_percent()

        logger.info(
            f"[STATS] Profiling started - Baseline: {self.baseline_memory:.1f}MB, {self.baseline_cpu:.1f}%"
        )

    def stop_profiling(self):
        """Stop profiling and cleanup."""
        if self.tracemalloc_enabled:
            tracemalloc.stop()
            self.tracemalloc_enabled = False
            logger.info("[SEARCH] Stopped memory tracing")

    def snapshot_component(
        self, component_name: str
    ) -> ComponentResourceUsage:
        """Take a resource snapshot for a specific component."""
        try:
            # Current process stats
            memory_info = self.process.memory_info()
            memory_mb = memory_info.rss / (1024 * 1024)
            cpu_percent = self.process.cpu_percent()

            # Thread count
            thread_count = self.process.num_threads()

            # File descriptors
            try:
                fd_count = (
                    self.process.num_fds()
                    if hasattr(self.process, "num_fds")
                    else 0
                )
            except Exception:
                fd_count = 0

            # Network connections
            try:
                connections = len(self.process.connections())
            except Exception:
                connections = 0

            usage = ComponentResourceUsage(
                name=component_name,
                memory_mb=memory_mb,
                cpu_percent=cpu_percent,
                thread_count=thread_count,
                file_descriptors=fd_count,
                network_connections=connections,
            )

            self.component_snapshots[component_name] = usage
            return usage

        except Exception as e:
            logger.error(f"Error taking snapshot for {component_name}: {e}")
            return ComponentResourceUsage(component_name, 0, 0, 0, 0, 0)

    def get_memory_breakdown(self) -> Dict[str, Any]:
        """Get detailed memory breakdown by allocation source."""
        if not tracemalloc.is_tracing():
            return {"error": "Memory tracing not enabled"}

        try:
            # Get current memory snapshot
            snapshot = tracemalloc.take_snapshot()
            top_stats = snapshot.statistics("lineno")

            breakdown = {
                "total_mb": sum(stat.size for stat in top_stats)
                / (1024 * 1024),
                "top_allocations": [],
            }

            # Get top 20 memory allocations
            for index, stat in enumerate(top_stats[:20]):
                allocation = {
                    "rank": index + 1,
                    "size_mb": stat.size / (1024 * 1024),
                    "count": stat.count,
                    "file": (
                        stat.traceback.format()[-1]
                        if stat.traceback
                        else "unknown"
                    ),
                    "avg_size_bytes": stat.size / stat.count
                    if stat.count > 0
                    else 0,
                }
                breakdown["top_allocations"].append(allocation)

            return breakdown

        except Exception as e:
            logger.error(f"Error getting memory breakdown: {e}")
            return {"error": str(e)}

    def analyze_thread_overhead(self) -> Dict[str, Any]:
        """Analyze thread overhead and identify thread-heavy components."""
        try:
            threads = []
            for thread in threading.enumerate():
                thread_info = {
                    "name": thread.name,
                    "daemon": thread.daemon,
                    "alive": thread.is_alive(),
                }
                threads.append(thread_info)

            return {
                "total_threads": len(threads),
                "daemon_threads": len([t for t in threads if t["daemon"]]),
                "active_threads": len([t for t in threads if t["alive"]]),
                "threads": threads,
            }

        except Exception as e:
            logger.error(f"Error analyzing threads: {e}")
            return {"error": str(e)}

    def get_component_comparison(self) -> List[ComponentResourceUsage]:
        """Get comparison of resource usage between components."""
        return list(self.component_snapshots.values())

    def generate_optimization_report(self, neuron_count: int = 0) -> str:
        """Generate a comprehensive optimization report."""
        report = []
        report.append("FEAGI RESOURCE OPTIMIZATION REPORT")
        report.append("=" * 50)

        # Memory analysis
        memory_per_neuron = (
            self.baseline_memory / neuron_count if neuron_count > 0 else 0
        )
        report.append(f"Memory per neuron: {memory_per_neuron:.2f} MB")
        report.append("Target for embedded: <0.1 MB/neuron")

        if memory_per_neuron > 0.5:  # 500KB per neuron is too much
            report.append(
                f"   CRITICAL: This is {memory_per_neuron / 1:.0f}x too high for embedded devices!"
            )

        # Thread analysis
        thread_count = len(threading.enumerate())
        report.append(f"Thread count: {thread_count}")
        report.append("Target for embedded: <10 threads")

        if thread_count > 20:
            report.append("   Each thread costs ~8MB on embedded systems!")

        # Memory breakdown
        memory_breakdown = self.get_memory_breakdown()
        if "error" not in memory_breakdown:
            report.append("[SAVE] MEMORY ALLOCATION BREAKDOWN:")
            report.append(
                f"   Total traced: {memory_breakdown['total_mb']:.1f}MB"
            )
            report.append("")
            report.append("   Top memory consumers:")

            for alloc in memory_breakdown["top_allocations"][:10]:
                report.append(
                    f"     #{alloc['rank']}: {alloc['size_mb']:.1f}MB - {alloc['file']}"
                )
            report.append("")

        # Component comparison
        if self.component_snapshots:
            report.append("COMPONENT RESOURCE USAGE:")
            components = sorted(
                self.component_snapshots.values(),
                key=lambda x: x.memory_mb,
                reverse=True,
            )

            for comp in components:
                neuron_usage = ""
                if neuron_count > 0:
                    per_neuron = comp.memory_per_neuron_kb(neuron_count)
                    neuron_usage = f" ({per_neuron:.1f}KB/neuron)"

                report.append(
                    f"   {comp.name}: {comp.memory_mb:.1f}MB{neuron_usage}, "
                    f"{comp.cpu_percent:.1f}% CPU, {comp.thread_count} threads"
                )
            report.append("")

        # Optimization recommendations
        report.append("[TARGET] OPTIMIZATION RECOMMENDATIONS:")

        if self.baseline_memory > 1000:  # > 1GB
            report.append("   CRITICAL - Memory usage > 1GB:")
            report.append("     • Implement lazy loading for neural data")
            report.append("     • Use memory-mapped files instead of RAM")
            report.append("     • Implement neural data compression")
            report.append(
                "     • Consider embedded-mode that disables heavy components"
            )

        if thread_count > 10:
            report.append("   CRITICAL - Too many threads:")
            report.append("     • Consolidate background tasks")
            report.append("     • Use async/await instead of threads")
            report.append(
                "     • Implement embedded mode with minimal threads"
            )

        if self.baseline_cpu > 5.0:  # > 5 cores equivalent
            report.append("   CRITICAL - CPU usage too high:")
            report.append("     • Profile and optimize hot paths")
            report.append("     • Reduce logging frequency")
            report.append("     • Implement CPU-efficient neural algorithms")

        report.append("")
        report.append("[TARGET] EMBEDDED DEVICE TARGETS:")
        report.append("   • Memory: <100MB total, <1KB per neuron")
        report.append("   • CPU: <0.5 cores average")
        report.append("   • Threads: <5 total")
        report.append("   • Network: Minimal connections")

        report.append("=" * 80)

        return "\n".join(report)


# Global profiler instance
_global_profiler: Optional[ResourceProfiler] = None


def get_profiler() -> ResourceProfiler:
    """Get the global resource profiler instance."""
    global _global_profiler
    if _global_profiler is None:
        _global_profiler = ResourceProfiler()
    return _global_profiler


def profile_component(component_name: str) -> ComponentResourceUsage:
    """Profile a specific component."""
    profiler = get_profiler()
    return profiler.snapshot_component(component_name)


def start_profiling():
    """Start global profiling."""
    profiler = get_profiler()
    profiler.start_profiling()


def stop_profiling():
    """Stop global profiling."""
    profiler = get_profiler()
    profiler.stop_profiling()


def generate_resource_report(neuron_count: int = 0) -> str:
    """Generate a resource optimization report."""
    profiler = get_profiler()
    return profiler.generate_optimization_report(neuron_count)
