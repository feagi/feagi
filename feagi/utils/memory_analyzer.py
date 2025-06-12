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
FEAGI Memory Analyzer

Analyzes memory usage breakdown to distinguish between neural data
and application overhead. Critical for embedded device optimization.
"""

import gc
import sys
import threading
import time
import tracemalloc
from datetime import datetime
from typing import Any, Dict, List, Optional

import psutil

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class MemoryAnalyzer:
    """Analyzes FEAGI memory usage with detailed breakdown."""

    def __init__(self):
        self.baseline_memory = None
        self.connectome_manager = None
        self.tracking_enabled = False

    def set_connectome_manager(self, connectome_manager):
        """Set the connectome manager for neural data analysis."""
        self.connectome_manager = connectome_manager

    def capture_baseline(self) -> Dict[str, Any]:
        """Capture baseline memory before neural data is loaded."""

        # Force garbage collection
        gc.collect()

        # Get process memory
        process = psutil.Process()
        memory_info = process.memory_info()

        baseline = {
            "timestamp": datetime.now().isoformat(),
            "rss_mb": memory_info.rss / (1024 * 1024),
            "vms_mb": memory_info.vms / (1024 * 1024),
            "thread_count": process.num_threads(),
            "python_objects": len(gc.get_objects()),
            "description": "Baseline memory before neural data loading",
        }

        self.baseline_memory = baseline
        logger.info(f"[STATS] Baseline captured: {baseline['rss_mb']:.1f}MB RSS")

        return baseline

    def estimate_neural_data_memory(self) -> Dict[str, Any]:
        """Estimate memory used specifically by neural data structures."""

        if not self.connectome_manager:
            return {"error": "ConnectomeManager not available"}

        try:
            # Get neuron and synapse counts
            neuron_count = getattr(self.connectome_manager, "neuron_count", 0)
            synapse_count = getattr(self.connectome_manager, "synapse_count", 0)

            # Estimate based on typical data structure sizes
            # These are rough estimates - actual sizes may vary

            # Neuron data (assuming each neuron has ID, position, state, etc.)
            # Rough estimate: 64 bytes per neuron for basic data
            estimated_neuron_memory_bytes = neuron_count * 64

            # Synapse data (assuming source, target, weight, etc.)
            # Rough estimate: 32 bytes per synapse
            estimated_synapse_memory_bytes = synapse_count * 32

            # Fire queue and other neural processing structures
            # Rough estimate: additional 25% overhead
            neural_overhead = (
                estimated_neuron_memory_bytes + estimated_synapse_memory_bytes
            ) * 0.25

            total_neural_bytes = (
                estimated_neuron_memory_bytes
                + estimated_synapse_memory_bytes
                + neural_overhead
            )

            neural_memory = {
                "neuron_count": neuron_count,
                "synapse_count": synapse_count,
                "estimated_neuron_memory_mb": estimated_neuron_memory_bytes
                / (1024 * 1024),
                "estimated_synapse_memory_mb": estimated_synapse_memory_bytes
                / (1024 * 1024),
                "estimated_neural_overhead_mb": neural_overhead / (1024 * 1024),
                "estimated_total_neural_mb": total_neural_bytes / (1024 * 1024),
                "memory_per_neuron_bytes": (
                    total_neural_bytes / neuron_count if neuron_count > 0 else 0
                ),
                "memory_per_neuron_kb": (
                    (total_neural_bytes / neuron_count) / 1024
                    if neuron_count > 0
                    else 0
                ),
                "note": "These are rough estimates based on typical data structure sizes",
            }

            logger.info(
                f"[BRAIN] Estimated neural data: {neural_memory['estimated_total_neural_mb']:.1f}MB "
                f"({neural_memory['memory_per_neuron_kb']:.1f}KB per neuron)"
            )

            return neural_memory

        except Exception as e:
            logger.error(f"Error estimating neural data memory: {e}")
            return {"error": str(e)}

    def get_python_memory_breakdown(self) -> Dict[str, Any]:
        """Get breakdown of Python object memory usage."""

        try:
            # Count objects by type
            obj_counts = {}
            obj_types = {}

            for obj in gc.get_objects():
                obj_type = type(obj).__name__
                obj_counts[obj_type] = obj_counts.get(obj_type, 0) + 1

                # Try to estimate size for common types
                try:
                    if hasattr(obj, "__sizeof__"):
                        size = obj.__sizeof__()
                        if obj_type not in obj_types:
                            obj_types[obj_type] = {"count": 0, "total_size": 0}
                        obj_types[obj_type]["count"] += 1
                        obj_types[obj_type]["total_size"] += size
                except:
                    pass

            # Sort by total size
            sorted_types = sorted(
                obj_types.items(), key=lambda x: x[1]["total_size"], reverse=True
            )[:20]  # Top 20

            breakdown = {
                "total_objects": len(gc.get_objects()),
                "top_memory_consumers": [
                    {
                        "type": obj_type,
                        "count": data["count"],
                        "total_mb": data["total_size"] / (1024 * 1024),
                        "avg_bytes": (
                            data["total_size"] / data["count"]
                            if data["count"] > 0
                            else 0
                        ),
                    }
                    for obj_type, data in sorted_types
                    if data["total_size"] > 0
                ],
            }

            return breakdown

        except Exception as e:
            logger.error(f"Error analyzing Python memory: {e}")
            return {"error": str(e)}

    def analyze_current_memory(self) -> Dict[str, Any]:
        """Perform comprehensive memory analysis."""

        # Force garbage collection first
        gc.collect()

        # Get current process memory
        process = psutil.Process()
        memory_info = process.memory_info()

        current_memory = {
            "timestamp": datetime.now().isoformat(),
            "total_rss_mb": memory_info.rss / (1024 * 1024),
            "total_vms_mb": memory_info.vms / (1024 * 1024),
            "thread_count": process.num_threads(),
            "fd_count": process.num_fds() if hasattr(process, "num_fds") else 0,
        }

        # Calculate overhead if we have baseline
        if self.baseline_memory:
            application_overhead_mb = (
                current_memory["total_rss_mb"] - self.baseline_memory["rss_mb"]
            )
            current_memory["application_overhead_mb"] = max(0, application_overhead_mb)
            current_memory["baseline_rss_mb"] = self.baseline_memory["rss_mb"]

        # Get neural data estimate
        neural_data = self.estimate_neural_data_memory()
        current_memory["neural_data_analysis"] = neural_data

        # Get Python object breakdown
        python_breakdown = self.get_python_memory_breakdown()
        current_memory["python_memory_breakdown"] = python_breakdown

        # Calculate breakdown percentages
        if (
            "estimated_total_neural_mb" in neural_data
            and neural_data["estimated_total_neural_mb"] > 0
        ):
            total_mb = current_memory["total_rss_mb"]
            neural_mb = neural_data["estimated_total_neural_mb"]
            overhead_mb = total_mb - neural_mb

            current_memory["memory_breakdown"] = {
                "total_mb": total_mb,
                "estimated_neural_data_mb": neural_mb,
                "estimated_overhead_mb": max(0, overhead_mb),
                "neural_percentage": (
                    (neural_mb / total_mb) * 100 if total_mb > 0 else 0
                ),
                "overhead_percentage": (
                    (max(0, overhead_mb) / total_mb) * 100 if total_mb > 0 else 0
                ),
            }

            logger.info(
                f"[STATS] Memory breakdown: {total_mb:.1f}MB total = "
                f"{neural_mb:.1f}MB neural ({current_memory['memory_breakdown']['neural_percentage']:.1f}%) + "
                f"{max(0, overhead_mb):.1f}MB overhead ({current_memory['memory_breakdown']['overhead_percentage']:.1f}%)"
            )

        return current_memory

    def start_continuous_monitoring(self, interval_seconds: float = 10.0):
        """Start continuous memory monitoring in background thread."""

        def monitor():
            while self.tracking_enabled:
                try:
                    analysis = self.analyze_current_memory()
                    breakdown = analysis.get("memory_breakdown", {})

                    if breakdown:
                        logger.info(
                            f"[SEARCH] Memory: {breakdown['total_mb']:.1f}MB total "
                            f"({breakdown['estimated_neural_data_mb']:.1f}MB neural + "
                            f"{breakdown['estimated_overhead_mb']:.1f}MB overhead)"
                        )

                    time.sleep(interval_seconds)

                except Exception as e:
                    logger.error(f"Error in memory monitoring: {e}")
                    time.sleep(interval_seconds)

        if not self.tracking_enabled:
            self.tracking_enabled = True
            monitor_thread = threading.Thread(target=monitor, daemon=True)
            monitor_thread.start()
            logger.info(
                f"[SEARCH] Started continuous memory monitoring (interval: {interval_seconds}s)"
            )

    def stop_continuous_monitoring(self):
        """Stop continuous memory monitoring."""
        self.tracking_enabled = False
        logger.info("[HALT] Stopped continuous memory monitoring")


# Global instance
_memory_analyzer = None


def get_memory_analyzer() -> MemoryAnalyzer:
    """Get the global memory analyzer instance."""
    global _memory_analyzer
    if _memory_analyzer is None:
        _memory_analyzer = MemoryAnalyzer()
    return _memory_analyzer


def analyze_memory_usage(connectome_manager=None) -> Dict[str, Any]:
    """Convenience function to analyze current memory usage."""
    analyzer = get_memory_analyzer()
    if connectome_manager:
        analyzer.set_connectome_manager(connectome_manager)
    return analyzer.analyze_current_memory()


def start_memory_monitoring(connectome_manager=None, interval_seconds: float = 10.0):
    """Start continuous memory monitoring."""
    analyzer = get_memory_analyzer()
    if connectome_manager:
        analyzer.set_connectome_manager(connectome_manager)
    analyzer.start_continuous_monitoring(interval_seconds)


def capture_baseline_memory() -> Dict[str, Any]:
    """Capture baseline memory before loading neural data."""
    analyzer = get_memory_analyzer()
    return analyzer.capture_baseline()
