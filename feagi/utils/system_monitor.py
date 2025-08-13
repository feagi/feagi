"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
FEAGI System Resource Monitor

Provides real-time monitoring of system resources using absolute measurements:
- Memory usage in MB (comparable across systems)
- CPU usage in equivalent cores (e.g., "1.2 cores" instead of percentages)
- GPU usage (wgpu-compatible, cross-platform)
- Process-specific resource consumption

Designed for profiling mode (--profile flag) to help developers track resource usage patterns
and identify potential performance bottlenecks or memory leaks across different systems.
"""

import threading
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional

# Standard system monitoring
import psutil

# GPU monitoring (optional dependencies)
try:
    import GPUtil

    GPUTIL_AVAILABLE = True
except ImportError:
    GPUTIL_AVAILABLE = False
    GPUtil = None

# Try to import wgpu for GPU monitoring (if available)
try:
    import wgpu

    WGPU_AVAILABLE = True
except ImportError:
    WGPU_AVAILABLE = False
    wgpu = None

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


@dataclass
class ResourceSnapshot:
    """Snapshot of system resource usage at a point in time."""

    timestamp: datetime
    cpu_cores_used: float  # Equivalent CPU cores being used (absolute)
    memory_mb: float
    gpu_usage: List[Dict[str, Any]]
    process_cpu_cores_used: float  # Process-specific CPU cores equivalent
    process_memory_mb: float

    def format_summary(self) -> str:
        """Format a human-readable summary of resource usage."""
        gpu_info = ""
        if self.gpu_usage:
            gpu_summaries = []
            for i, gpu in enumerate(self.gpu_usage):
                gpu_summaries.append(
                    f"GPU{i}: {gpu.get('utilization', 0):.1f}% ({gpu.get('memory_used', 0):.1f}MB/{gpu.get('memory_total', 0):.1f}MB)"
                )
            gpu_info = f" | {' | '.join(gpu_summaries)}"

        return (
            f"System: CPU {self.cpu_cores_used:.2f} cores | RAM {self.memory_mb:.1f}MB{gpu_info} | "
            f"Threads: {self.thread_count}"
        )


class SystemResourceMonitor:
    """Real-time system resource monitor for FEAGI profiling mode.

    Tracks CPU, memory, and GPU usage both system-wide and process-specific using
    absolute measurements that are comparable across systems:
    - CPU usage in equivalent cores (e.g., "1.2 cores" instead of "50%" on 2-core system)
    - Memory usage in MB (absolute values, not percentages)
    - GPU usage with memory in MB

    Provides periodic reporting and historical tracking for performance analysis.
    """

    def __init__(
        self,
        monitoring_interval: float = 5.0,
        enable_gpu_monitoring: bool = True,
        enable_detailed_logging: bool = True,
        max_history_entries: int = 100,
    ):
        """Initialize the system resource monitor.

        Args:
            monitoring_interval: Seconds between resource checks
            enable_gpu_monitoring: Whether to monitor GPU resources
            enable_detailed_logging: Whether to log detailed resource info
            max_history_entries: Maximum number of historical snapshots to keep
        """
        self.monitoring_interval = monitoring_interval
        self.enable_gpu_monitoring = enable_gpu_monitoring
        self.enable_detailed_logging = enable_detailed_logging
        self.max_history_entries = max_history_entries

        # Runtime state
        self.running = False
        self.monitor_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Resource tracking
        self.current_process = psutil.Process()
        self.resource_history: List[ResourceSnapshot] = []

        # GPU initialization
        self.gpu_available = False

        if self.enable_gpu_monitoring:
            self._initialize_gpu_monitoring()

        logger.info(
            f"[CONFIG] System Resource Monitor initialized (interval: {monitoring_interval}s, GPU: {self.gpu_available})"
        )

    def _initialize_gpu_monitoring(self) -> None:
        """Initialize GPU monitoring capabilities."""
        try:
            if GPUTIL_AVAILABLE:
                # Try GPUtil as fallback
                gpus = GPUtil.getGPUs()
                if gpus:
                    logger.info(
                        f"[CTRL] GPU monitoring enabled via GPUtil ({len(gpus)} devices)"
                    )
                    self.gpu_available = True
            else:
                logger.debug("[CTRL] No GPU monitoring libraries available")
        except Exception as e:
            logger.warning(f"[CTRL] GPU monitoring initialization failed: {e}")
            self.gpu_available = False

    def start(self) -> None:
        """Start the resource monitoring thread."""
        if self.running:
            logger.warning("System resource monitor already running")
            return

        logger.info("[START] Starting system resource monitor...")
        self.running = True
        self._stop_event.clear()

        self.monitor_thread = threading.Thread(
            target=self._monitoring_loop,
            name="SystemResourceMonitor",
            daemon=True,
        )
        self.monitor_thread.start()

        logger.info("[OK] System resource monitor started")

    def stop(self) -> None:
        """Stop the resource monitoring thread."""
        if not self.running:
            return

        logger.info("[HALT] Stopping system resource monitor...")
        self.running = False
        self._stop_event.set()

        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=5.0)
            if self.monitor_thread.is_alive():
                logger.warning("Monitor thread didn't stop gracefully")

        # Cleanup GPU resources
        # No specific cleanup needed for GPUtil or wgpu

        logger.info("[OK] System resource monitor stopped")

    def _monitoring_loop(self) -> None:
        """Main monitoring loop that runs in a separate thread."""
        logger.debug("Resource monitoring loop started")

        while self.running and not self._stop_event.is_set():
            try:
                # Capture resource snapshot
                snapshot = self._capture_resource_snapshot()

                # Store in history
                self.resource_history.append(snapshot)

                # Trim history if needed
                if len(self.resource_history) > self.max_history_entries:
                    self.resource_history.pop(0)

                # Log resource usage
                if self.enable_detailed_logging:
                    logger.info(snapshot.format_summary())

                # Wait for next interval
                if self._stop_event.wait(timeout=self.monitoring_interval):
                    break

            except Exception as e:
                logger.error(f"Error in resource monitoring loop: {e}")
                if self._stop_event.wait(timeout=1.0):
                    break

        logger.debug("Resource monitoring loop stopped")

    def _capture_resource_snapshot(self) -> ResourceSnapshot:
        """Capture a snapshot of current resource usage."""
        timestamp = datetime.now()

        # System-wide CPU and memory
        cpu_percent = psutil.cpu_percent(interval=None)
        cpu_core_count = psutil.cpu_count(
            logical=True
        )  # Get logical CPU count
        cpu_cores_used = (
            cpu_percent / 100.0
        ) * cpu_core_count  # Convert percentage to cores
        memory = psutil.virtual_memory()
        memory_mb = memory.used / (1024 * 1024)

        # Process-specific resources
        try:
            process_cpu_percent = self.current_process.cpu_percent()
            process_cpu_cores_used = (
                process_cpu_percent / 100.0
            ) * cpu_core_count  # Convert to cores
            process_memory = self.current_process.memory_info()
            process_memory_mb = process_memory.rss / (1024 * 1024)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            # Process might have been restarted
            self.current_process = psutil.Process()
            process_cpu_cores_used = 0.0
            process_memory_mb = 0.0

        # GPU usage
        gpu_usage = []
        if self.gpu_available:
            gpu_usage = self._get_gpu_usage()

        return ResourceSnapshot(
            timestamp=timestamp,
            cpu_cores_used=cpu_cores_used,
            memory_mb=memory_mb,
            gpu_usage=gpu_usage,
            process_cpu_cores_used=process_cpu_cores_used,
            process_memory_mb=process_memory_mb,
        )

    def _get_gpu_usage(self) -> List[Dict[str, Any]]:
        """Get current GPU usage information."""
        gpu_info = []

        try:
            if GPUTIL_AVAILABLE:
                # Fallback to GPUtil
                gpus = GPUtil.getGPUs()
                for gpu in gpus:
                    gpu_info.append(
                        {
                            "index": gpu.id,
                            "utilization": gpu.load * 100,
                            "memory_utilization": gpu.memoryUtil * 100,
                            "memory_used": gpu.memoryUsed,
                            "memory_total": gpu.memoryTotal,
                            "memory_percent": gpu.memoryUtil * 100,
                            "temperature": gpu.temperature,
                        }
                    )

        except Exception as e:
            logger.debug(f"Error getting GPU usage: {e}")

        return gpu_info

    def get_current_usage(self) -> Optional[ResourceSnapshot]:
        """Get the most recent resource usage snapshot."""
        if self.resource_history:
            return self.resource_history[-1]
        return None

    def get_usage_summary(self, last_n_entries: int = 10) -> Dict[str, Any]:
        """Get a summary of resource usage over the last N entries."""
        if not self.resource_history:
            return {}

        recent_entries = self.resource_history[-last_n_entries:]

        # Calculate averages
        avg_cpu = sum(entry.cpu_cores_used for entry in recent_entries) / len(
            recent_entries
        )
        avg_memory_mb = sum(entry.memory_mb for entry in recent_entries) / len(
            recent_entries
        )
        avg_process_cpu = sum(
            entry.process_cpu_cores_used for entry in recent_entries
        ) / len(recent_entries)
        avg_process_memory_mb = sum(
            entry.process_memory_mb for entry in recent_entries
        ) / len(recent_entries)

        # Peak values
        peak_cpu = max(entry.cpu_cores_used for entry in recent_entries)
        peak_memory_mb = max(entry.memory_mb for entry in recent_entries)
        peak_process_cpu = max(
            entry.process_cpu_cores_used for entry in recent_entries
        )
        peak_process_memory_mb = max(
            entry.process_memory_mb for entry in recent_entries
        )

        summary = {
            "entries_analyzed": len(recent_entries),
            "time_span_minutes": (
                recent_entries[-1].timestamp - recent_entries[0].timestamp
            ).total_seconds()
            / 60,
            "system": {
                "cpu_avg": avg_cpu,
                "cpu_peak": peak_cpu,
                "memory_mb_avg": avg_memory_mb,
                "memory_mb_peak": peak_memory_mb,
            },
            "process": {
                "cpu_avg": avg_process_cpu,
                "cpu_peak": peak_process_cpu,
                "memory_mb_avg": avg_process_memory_mb,
                "memory_mb_peak": peak_process_memory_mb,
            },
        }

        # GPU summary if available
        if recent_entries[0].gpu_usage:
            gpu_summaries = []
            for gpu_idx in range(len(recent_entries[0].gpu_usage)):
                gpu_utils = [
                    entry.gpu_usage[gpu_idx]["utilization"]
                    for entry in recent_entries
                    if len(entry.gpu_usage) > gpu_idx
                ]
                if gpu_utils:
                    gpu_summaries.append(
                        {
                            "index": gpu_idx,
                            "utilization_avg": sum(gpu_utils) / len(gpu_utils),
                            "utilization_peak": max(gpu_utils),
                        }
                    )
            summary["gpu"] = gpu_summaries

        return summary

    def print_detailed_report(self) -> None:
        """Print a detailed resource usage report to console."""
        if not self.resource_history:
            print("[STATS] No resource usage data available")
            return

        current = self.get_current_usage()
        summary = self.get_usage_summary()

        print("\n" + "=" * 80)
        print("[STATS] FEAGI SYSTEM RESOURCE USAGE REPORT")
        print("=" * 80)

        if current:
            print(
                f"Current Time: {current.timestamp.strftime('%Y-%m-%d %H:%M:%S')}"
            )
            print(f"System CPU: {current.cpu_cores_used:.2f} cores")
            print(f"[BRAIN] System Memory: {current.memory_mb:.1f} MB")
            print(
                f"[FAST] FEAGI Process CPU: {current.process_cpu_cores_used:.2f} cores"
            )
            print(
                f"[SAVE] FEAGI Process Memory: {current.process_memory_mb:.1f} MB"
            )

            if current.gpu_usage:
                print("[CTRL] GPU Usage:")
                for gpu in current.gpu_usage:
                    gpu_text = f"   GPU {gpu['index']}: {gpu['utilization']:.1f}% util, {gpu['memory_used']:.1f}/{gpu['memory_total']:.1f} MB"
                    if gpu.get("temperature"):
                        gpu_text += f", {gpu['temperature']}°C"
                    print(gpu_text)

        if summary:
            print(
                f"\n[UP] Recent Performance (last {summary['entries_analyzed']} snapshots, {summary['time_span_minutes']:.1f} min):"
            )
            print(
                f"   System CPU: avg {summary['system']['cpu_avg']:.2f} cores, peak {summary['system']['cpu_peak']:.2f} cores"
            )
            print(
                f"   System Memory: avg {summary['system']['memory_mb_avg']:.1f} MB, peak {summary['system']['memory_mb_peak']:.1f} MB"
            )
            print(
                f"   FEAGI CPU: avg {summary['process']['cpu_avg']:.2f} cores, peak {summary['process']['cpu_peak']:.2f} cores"
            )
            print(
                f"   FEAGI Memory: avg {summary['process']['memory_mb_avg']:.1f} MB, peak {summary['process']['memory_mb_peak']:.1f} MB"
            )

            if "gpu" in summary:
                print("   GPU Performance:")
                for gpu_summary in summary["gpu"]:
                    print(
                        f"      GPU {gpu_summary['index']}: avg {gpu_summary['utilization_avg']:.1f}%, peak {gpu_summary['utilization_peak']:.1f}%"
                    )

        print("=" * 80 + "\n")


# Global monitor instance for easy access
_global_monitor: Optional[SystemResourceMonitor] = None


def get_system_monitor() -> Optional[SystemResourceMonitor]:
    """Get the global system resource monitor instance."""
    return _global_monitor


def start_system_monitoring(
    monitoring_interval: float = 5.0,
    enable_gpu_monitoring: bool = True,
    enable_detailed_logging: bool = True,
) -> SystemResourceMonitor:
    """Start system resource monitoring for profiling mode.

    Args:
        monitoring_interval: Seconds between resource checks
        enable_gpu_monitoring: Whether to monitor GPU resources
        enable_detailed_logging: Whether to log detailed resource info

    Returns:
        SystemResourceMonitor instance
    """
    global _global_monitor

    if _global_monitor and _global_monitor.running:
        logger.warning("System resource monitor already running")
        return _global_monitor

    _global_monitor = SystemResourceMonitor(
        monitoring_interval=monitoring_interval,
        enable_gpu_monitoring=enable_gpu_monitoring,
        enable_detailed_logging=enable_detailed_logging,
    )

    _global_monitor.start()
    return _global_monitor


def stop_system_monitoring() -> None:
    """Stop the global system resource monitor."""
    global _global_monitor

    if _global_monitor:
        _global_monitor.stop()
        _global_monitor = None


def print_resource_report() -> None:
    """Print a detailed resource usage report to console."""
    if _global_monitor:
        _global_monitor.print_detailed_report()
    else:
        print("[STATS] System resource monitor not running")
