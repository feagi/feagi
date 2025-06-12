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
SIMD Performance Profiler for FEAGI NPU.

This module provides profiling and performance measurement tools for SIMD
operations in neural processing, helping identify optimization opportunities
and monitor performance regressions.

@cursor:simd-profiling
@cursor:performance-monitoring
"""

import logging
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Any, ContextManager, Dict, List, Optional

import numpy as np

from .simd_detection import SIMDBackend, get_simd_detector

logger = logging.getLogger(__name__)


@dataclass
class OperationProfile:
    """Profile data for a single SIMD operation."""

    name: str
    backend: SIMDBackend
    vector_width: int
    total_time: float = 0.0
    call_count: int = 0
    element_count: int = 0
    bytes_processed: int = 0
    cache_misses: int = 0
    simd_efficiency: float = 0.0  # Theoretical vs actual performance

    def __post_init__(self):
        """Calculate derived metrics."""
        self.avg_time = (
            self.total_time / self.call_count if self.call_count > 0 else 0.0
        )
        self.throughput = (
            self.element_count / self.total_time if self.total_time > 0 else 0.0
        )
        self.bandwidth = (
            self.bytes_processed / self.total_time if self.total_time > 0 else 0.0
        )


@dataclass
class SIMDSession:
    """Profiling session for a group of SIMD operations."""

    name: str
    start_time: float = field(default_factory=time.perf_counter)
    end_time: Optional[float] = None
    operations: Dict[str, OperationProfile] = field(default_factory=dict)
    total_elements: int = 0
    total_bytes: int = 0

    @property
    def duration(self) -> float:
        """Get session duration."""
        end = self.end_time or time.perf_counter()
        return end - self.start_time

    @property
    def overall_throughput(self) -> float:
        """Get overall elements processed per second."""
        return self.total_elements / self.duration if self.duration > 0 else 0.0


class SIMDProfiler:
    """Advanced SIMD performance profiler."""

    def __init__(self):
        self.detector = get_simd_detector()
        self.sessions: Dict[str, SIMDSession] = {}
        self.current_session: Optional[SIMDSession] = None
        self._thread_local = threading.local()
        self.enabled = True

    def start_session(self, name: str) -> SIMDSession:
        """Start a new profiling session."""
        if not self.enabled:
            return None

        session = SIMDSession(name=name)
        self.sessions[name] = session
        self.current_session = session
        return session

    def end_session(self, name: Optional[str] = None) -> Optional[SIMDSession]:
        """End a profiling session."""
        if not self.enabled:
            return None

        if name:
            session = self.sessions.get(name)
        else:
            session = self.current_session

        if session:
            session.end_time = time.perf_counter()
            if session == self.current_session:
                self.current_session = None

        return session

    @contextmanager
    def profile_session(self, name: str) -> ContextManager[SIMDSession]:
        """Context manager for profiling sessions."""
        session = self.start_session(name)
        try:
            yield session
        finally:
            self.end_session(name)

    @contextmanager
    def measure(
        self,
        operation_name: str,
        backend: Optional[SIMDBackend] = None,
        element_count: int = 0,
        dtype: str = "float32",
    ) -> ContextManager[None]:
        """Context manager for measuring individual SIMD operations."""
        if not self.enabled or not self.current_session:
            yield
            return

        # Detect backend if not provided
        if backend is None:
            backend = self.detector.get_optimal_backend("general")

        # Calculate bytes processed
        dtype_sizes = {"float32": 4, "float64": 8, "int32": 4, "int64": 8}
        bytes_per_element = dtype_sizes.get(dtype, 4)
        bytes_processed = element_count * bytes_per_element

        # Get or create operation profile
        session = self.current_session
        if operation_name not in session.operations:
            session.operations[operation_name] = OperationProfile(
                name=operation_name,
                backend=backend,
                vector_width=self.detector.capabilities.vector_width,
            )

        profile = session.operations[operation_name]

        # Measure performance
        start_time = time.perf_counter()
        try:
            yield
        finally:
            end_time = time.perf_counter()
            operation_time = end_time - start_time

            # Update profile
            profile.total_time += operation_time
            profile.call_count += 1
            profile.element_count += element_count
            profile.bytes_processed += bytes_processed

            # Update session totals
            session.total_elements += element_count
            session.total_bytes += bytes_processed

            # Calculate SIMD efficiency
            profile.simd_efficiency = self._calculate_simd_efficiency(
                profile, operation_time, element_count
            )

    def _calculate_simd_efficiency(
        self, profile: OperationProfile, operation_time: float, element_count: int
    ) -> float:
        """Calculate SIMD efficiency as ratio of theoretical to actual performance."""
        if operation_time <= 0 or element_count <= 0:
            return 0.0

        # Theoretical performance assumes perfect vectorization
        theoretical_cycles = element_count / profile.vector_width

        # Estimate CPU frequency (rough approximation)
        estimated_frequency = 2.5e9  # 2.5 GHz
        theoretical_time = theoretical_cycles / estimated_frequency

        # Efficiency is theoretical time / actual time
        # Values > 1.0 indicate better than expected performance (cache effects, etc.)
        # Values < 1.0 indicate suboptimal SIMD usage
        efficiency = theoretical_time / operation_time if operation_time > 0 else 0.0

        return min(efficiency, 2.0)  # Cap at 2.0 for realistic values

    def benchmark_operation(
        self,
        operation_name: str,
        operation_func,
        *args,
        iterations: int = 100,
        **kwargs,
    ) -> OperationProfile:
        """Benchmark a specific operation multiple times."""

        with self.profile_session(f"benchmark_{operation_name}") as session:
            for i in range(iterations):
                with self.measure(
                    operation_name, element_count=kwargs.get("element_count", 0)
                ):
                    operation_func(*args, **kwargs)

        return session.operations.get(operation_name)

    def compare_backends(
        self,
        operation_name: str,
        operation_func,
        backends: List[SIMDBackend],
        *args,
        **kwargs,
    ) -> Dict[SIMDBackend, OperationProfile]:
        """Compare performance across different SIMD backends."""
        results = {}

        for backend in backends:
            # This would require backend-specific operation implementations
            # For now, just measure with current backend
            with self.profile_session(f"compare_{backend.value}") as session:
                with self.measure(f"{operation_name}_{backend.value}", backend=backend):
                    operation_func(*args, **kwargs)

                if operation_name in session.operations:
                    results[backend] = session.operations[operation_name]

        return results

    def get_performance_report(
        self, session_name: Optional[str] = None
    ) -> Dict[str, Any]:
        """Generate a comprehensive performance report."""
        if session_name:
            sessions = {session_name: self.sessions[session_name]}
        else:
            sessions = self.sessions

        report = {
            "summary": {
                "total_sessions": len(sessions),
                "overall_duration": sum(s.duration for s in sessions.values()),
                "total_elements": sum(s.total_elements for s in sessions.values()),
                "total_bytes": sum(s.total_bytes for s in sessions.values()),
            },
            "sessions": {},
            "top_operations": [],
            "performance_issues": [],
            "recommendations": [],
        }

        # Analyze each session
        for name, session in sessions.items():
            session_report = {
                "duration": session.duration,
                "throughput": session.overall_throughput,
                "operations": {},
                "bottlenecks": [],
            }

            for op_name, profile in session.operations.items():
                op_report = {
                    "backend": profile.backend.value,
                    "call_count": profile.call_count,
                    "total_time": profile.total_time,
                    "avg_time": profile.avg_time,
                    "throughput": profile.throughput,
                    "bandwidth_mb_s": profile.bandwidth / (1024 * 1024),
                    "simd_efficiency": profile.simd_efficiency,
                    "vector_width": profile.vector_width,
                }
                session_report["operations"][op_name] = op_report

                # Identify bottlenecks
                if profile.simd_efficiency < 0.3:
                    session_report["bottlenecks"].append(
                        {
                            "operation": op_name,
                            "issue": "Low SIMD efficiency",
                            "efficiency": profile.simd_efficiency,
                        }
                    )

                if profile.avg_time > 0.001:  # > 1ms average
                    session_report["bottlenecks"].append(
                        {
                            "operation": op_name,
                            "issue": "High latency",
                            "avg_time": profile.avg_time,
                        }
                    )

            report["sessions"][name] = session_report

        # Generate top operations and recommendations
        self._analyze_performance_patterns(report)

        return report

    def _analyze_performance_patterns(self, report: Dict[str, Any]):
        """Analyze performance patterns and generate recommendations."""
        all_operations = []

        # Collect all operations across sessions
        for session_report in report["sessions"].values():
            for op_name, op_data in session_report["operations"].items():
                all_operations.append((op_name, op_data))

        # Sort by total time (find bottlenecks)
        all_operations.sort(key=lambda x: x[1]["total_time"], reverse=True)

        # Top 5 most time-consuming operations
        report["top_operations"] = [
            {"name": name, **data} for name, data in all_operations[:5]
        ]

        # Performance issue detection
        issues = []
        recommendations = []

        for op_name, op_data in all_operations:
            # Low SIMD efficiency
            if op_data["simd_efficiency"] < 0.5:
                issues.append(
                    {
                        "type": "low_simd_efficiency",
                        "operation": op_name,
                        "efficiency": op_data["simd_efficiency"],
                        "description": f"Operation {op_name} has low SIMD efficiency ({op_data['simd_efficiency']:.2f})",
                    }
                )
                recommendations.append(
                    f"[CONFIG] Optimize {op_name} for better vectorization"
                )

            # High call frequency with low throughput
            if op_data["call_count"] > 100 and op_data["throughput"] < 1000:
                issues.append(
                    {
                        "type": "high_frequency_low_throughput",
                        "operation": op_name,
                        "call_count": op_data["call_count"],
                        "throughput": op_data["throughput"],
                    }
                )
                recommendations.append(
                    f"[START] Consider batching {op_name} operations"
                )

            # Memory bandwidth issues
            if (
                op_data["bandwidth_mb_s"] > 10000
            ):  # > 10 GB/s might indicate memory bound
                recommendations.append(
                    f"[SAVE] {op_name} may be memory-bound, consider cache optimization"
                )

        report["performance_issues"] = issues
        report["recommendations"] = recommendations

    def print_report(self, session_name: Optional[str] = None):
        """Print a formatted performance report."""
        report = self.get_performance_report(session_name)

        print("\n" + "=" * 60)
        print("🧪 SIMD PERFORMANCE REPORT")
        print("=" * 60)

        # Summary
        summary = report["summary"]
        print(f"\n[STATS] Summary:")
        print(f"   Sessions: {summary['total_sessions']}")
        print(f"   Duration: {summary['overall_duration']:.3f}s")
        print(f"   Elements: {summary['total_elements']:,}")
        print(f"   Data: {summary['total_bytes'] / (1024 * 1024):.1f} MB")

        # Top operations
        print(f"\n[DEBUG] Top Operations by Time:")
        for i, op in enumerate(report["top_operations"][:3], 1):
            print(
                f"   {i}. {op['name']}: {op['total_time']:.3f}s "
                f"({op['call_count']} calls, {op['simd_efficiency']:.2f} efficiency)"
            )

        # Performance issues
        if report["performance_issues"]:
            print(f"\n[WARN] Performance Issues:")
            for issue in report["performance_issues"][:3]:
                print(f"   • {issue['description']}")

        # Recommendations
        if report["recommendations"]:
            print(f"\n💡 Recommendations:")
            for rec in report["recommendations"][:3]:
                print(f"   {rec}")

        print("\n" + "=" * 60)

    def reset(self):
        """Reset all profiling data."""
        self.sessions.clear()
        self.current_session = None


# Global profiler instance
_global_profiler = None


def get_profiler() -> SIMDProfiler:
    """Get global SIMD profiler instance."""
    global _global_profiler
    if _global_profiler is None:
        _global_profiler = SIMDProfiler()
    return _global_profiler


@contextmanager
def profile_simd_operation(
    operation_name: str, element_count: int = 0, dtype: str = "float32"
) -> ContextManager[None]:
    """Convenience context manager for profiling SIMD operations."""
    profiler = get_profiler()
    with profiler.measure(operation_name, element_count=element_count, dtype=dtype):
        yield


def benchmark_numpy_operation(
    operation_name: str, operation_func, data_size: int = 10000, iterations: int = 100
):
    """Benchmark a NumPy operation for SIMD performance."""
    profiler = get_profiler()

    # Generate test data
    test_data = np.random.random(data_size).astype(np.float32)

    return profiler.benchmark_operation(
        operation_name,
        operation_func,
        test_data,
        iterations=iterations,
        element_count=data_size,
    )
