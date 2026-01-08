"""
Performance Profiler

Profiles performance of sensory/motor data operations.
"""

import time
from typing import Dict, Any, List, Optional
from dataclasses import dataclass, field
from collections import defaultdict

from feagi.pns.observability.monitor import Monitor


@dataclass
class OperationProfile:
    """Profile for a specific operation."""
    operation_name: str
    call_count: int = 0
    total_time_ms: float = 0.0
    min_time_ms: float = float('inf')
    max_time_ms: float = 0.0
    
    @property
    def avg_time_ms(self) -> float:
        """Average time per call."""
        return self.total_time_ms / self.call_count if self.call_count > 0 else 0.0
    
    def record(self, duration_ms: float):
        """Record an operation duration."""
        self.call_count += 1
        self.total_time_ms += duration_ms
        self.min_time_ms = min(self.min_time_ms, duration_ms)
        self.max_time_ms = max(self.max_time_ms, duration_ms)


@dataclass
class Profile:
    """Overall performance profile."""
    operations: Dict[str, OperationProfile] = field(default_factory=dict)
    
    @property
    def encoding_time_ms(self) -> float:
        """Average encoding time."""
        return self.operations.get('encoding', OperationProfile('encoding')).avg_time_ms
    
    @property
    def serialization_time_ms(self) -> float:
        """Average serialization time."""
        return self.operations.get('serialization', OperationProfile('serialization')).avg_time_ms
    
    @property
    def transmission_time_ms(self) -> float:
        """Average transmission time."""
        return self.operations.get('transmission', OperationProfile('transmission')).avg_time_ms
    
    def get_bottlenecks(self, threshold_ms: float = 10.0) -> List[OperationProfile]:
        """
        Get operations that exceed threshold.
        
        Args:
            threshold_ms: Minimum average time to be considered a bottleneck
        
        Returns:
            List of slow operations
        """
        bottlenecks = [
            op for op in self.operations.values()
            if op.avg_time_ms >= threshold_ms
        ]
        return sorted(bottlenecks, key=lambda x: x.avg_time_ms, reverse=True)
    
    def print_summary(self):
        """Print performance profile summary."""
        print("\n" + "=" * 60)
        print("Performance Profile")
        print("=" * 60)
        
        if not self.operations:
            print("No profiling data collected")
            print("=" * 60 + "\n")
            return
        
        print(f"\n{'Operation':<20} {'Calls':>10} {'Avg (ms)':>12} {'Min (ms)':>12} {'Max (ms)':>12}")
        print("-" * 70)
        
        for op in sorted(self.operations.values(), key=lambda x: x.avg_time_ms, reverse=True):
            print(
                f"{op.operation_name:<20} "
                f"{op.call_count:>10} "
                f"{op.avg_time_ms:>12.2f} "
                f"{op.min_time_ms:>12.2f} "
                f"{op.max_time_ms:>12.2f}"
            )
        
        # Show bottlenecks
        bottlenecks = self.get_bottlenecks(threshold_ms=10.0)
        if bottlenecks:
            print("\n⚠️  Bottlenecks (>10ms average):")
            for op in bottlenecks:
                print(f"  - {op.operation_name}: {op.avg_time_ms:.2f} ms")
        
        print("=" * 60 + "\n")


class Profiler(Monitor):
    """
    Profiles performance of brain_input and brain_output operations.
    
    Tracks timing for:
    - Overall send/receive operations
    - Individual operation phases (if available)
    - Identifies bottlenecks
    
    Example:
        profiler = Profiler()
        brain_input.attach_monitor(profiler)
        
        # ... run agent ...
        
        profile = profiler.get_profile()
        profile.print_summary()
        
        # Identify slow operations
        bottlenecks = profile.get_bottlenecks(threshold_ms=10)
        for op in bottlenecks:
            print(f"Slow: {op.operation_name} - {op.avg_time_ms:.2f} ms")
    """
    
    def __init__(
        self,
        enabled: bool = True,
        track_operations: Optional[List[str]] = None,
        log_level: str = "INFO"
    ):
        """
        Initialize profiler.
        
        Args:
            enabled: Whether profiling is active
            track_operations: Specific operations to track (None = all)
            log_level: Logging level
        """
        super().__init__(enabled=enabled, log_level=log_level)
        
        self.track_operations = track_operations
        self._profiles: Dict[str, OperationProfile] = {}
        self._operation_starts: Dict[str, float] = {}
    
    def _start_operation(self, operation: str):
        """Mark the start of an operation."""
        if self.track_operations and operation not in self.track_operations:
            return
        self._operation_starts[operation] = time.perf_counter()
    
    def _end_operation(self, operation: str):
        """Mark the end of an operation and record duration."""
        if operation not in self._operation_starts:
            return
        
        start_time = self._operation_starts.pop(operation)
        duration_ms = (time.perf_counter() - start_time) * 1000.0
        
        if operation not in self._profiles:
            self._profiles[operation] = OperationProfile(operation)
        
        self._profiles[operation].record(duration_ms)
    
    def on_send_start(self, data: Dict[str, Any]):
        """Mark send operation start."""
        if not self.enabled:
            return
        self._start_operation('send')
    
    def on_send_complete(self, data: Dict[str, Any]):
        """Mark send operation complete and record timing."""
        if not self.enabled:
            return
        
        self._end_operation('send')
        
        # If duration is provided in data, use it directly
        if 'duration_ms' in data:
            if 'send' not in self._profiles:
                self._profiles['send'] = OperationProfile('send')
            self._profiles['send'].record(data['duration_ms'])
        
        # Record sub-operations if available
        if 'encoding_ms' in data:
            if 'encoding' not in self._profiles:
                self._profiles['encoding'] = OperationProfile('encoding')
            self._profiles['encoding'].record(data['encoding_ms'])
        
        if 'serialization_ms' in data:
            if 'serialization' not in self._profiles:
                self._profiles['serialization'] = OperationProfile('serialization')
            self._profiles['serialization'].record(data['serialization_ms'])
        
        if 'transmission_ms' in data:
            if 'transmission' not in self._profiles:
                self._profiles['transmission'] = OperationProfile('transmission')
            self._profiles['transmission'].record(data['transmission_ms'])
    
    def on_receive_start(self, data: Dict[str, Any]):
        """Mark receive operation start."""
        if not self.enabled:
            return
        self._start_operation('receive')
    
    def on_receive_complete(self, data: Dict[str, Any]):
        """Mark receive operation complete and record timing."""
        if not self.enabled:
            return
        
        self._end_operation('receive')
        
        # If duration is provided in data, use it directly
        if 'duration_ms' in data:
            if 'receive' not in self._profiles:
                self._profiles['receive'] = OperationProfile('receive')
            self._profiles['receive'].record(data['duration_ms'])
    
    def get_profile(self) -> Profile:
        """
        Get performance profile.
        
        Returns:
            Profile with all operation timings
        """
        return Profile(operations=dict(self._profiles))
    
    def reset(self):
        """Reset profiling data."""
        self._profiles.clear()
        self._operation_starts.clear()
        self._logger.info("Profiling data reset")

