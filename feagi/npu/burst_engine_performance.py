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
FEAGI Burst Engine Performance Module

This module provides SIMD acceleration, frequency measurement, and performance
monitoring functionality for the BurstEngine.

Features:
- Centralized SIMD configuration integration
- Real-time frequency measurement and analysis
- Performance statistics and monitoring
- Timing buffer management for measurements
- RTOS-compatible performance profiling

This is a mixin class designed to be combined with the core BurstEngine.
"""

import statistics
import time
from typing import Any, Dict, List, Optional

import numpy as np

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

# SIMD and performance imports
try:
    from ..utils.simd_detection import (  # get_backend_selector,  # Unused import removed; get_simd_config,  # Unused import removed
        get_simd_detector,
    )
    from ..utils.simd_profiler import get_profiler, profile_simd_operation
    from .optimized_membrane_operations import SIMDMembraneProcessor

    SIMD_AVAILABLE = True
except ImportError:
    SIMD_AVAILABLE = False


class BurstEnginePerformanceMixin:
    """Performance functionality mixin for BurstEngine.

    Provides SIMD acceleration, frequency measurement, and performance monitoring:
    - Centralized SIMD configuration integration
    - Real-time frequency measurement and analysis
    - Performance statistics and monitoring
    - Timing buffer management for measurements
    - RTOS-compatible performance profiling

    This mixin is designed to work with the core BurstEngine class.
    """

    def __init__(self, *args, **kwargs):
        """Initialize performance functionality.

        Called by main BurstEngine.__init__.
        """
        # DO NOT call super().__init__ in mixins - causes multiple inheritance issues
        # Just initialize our own attributes

        # Initialize frequency measurement system
        self._burst_timing_buffer = []  # Circular buffer for burst durations
        self._processing_timing_buffer = (
            []
        )  # Circular buffer for pure processing durations
        self._timing_buffer_size = 100  # Keep last 100 burst measurements
        self._last_frequency_update = 0.0
        self._frequency_measurement_enabled = (
            False  # Only enable when requested via API
        )

        # Initialize SIMD support
        self._init_simd_support()

        # Performance monitoring
        self.total_neurons_processed = 0
        self.last_performance_report = time.time()

        # SIMD-optimized membrane processor
        if SIMD_AVAILABLE:
            self.membrane_processor = (
                None  # Initialized when capacity is known
            )

        # Runtime configuration (inherited from main BurstEngine config)
        config = getattr(self, "config", {})
        self.use_simd_profiling = config.get("simd_profiling", False)
        self.performance_monitoring = config.get(
            "performance_monitoring", True
        )

        logger.debug("BurstEngine performance mixin initialized")

    def _init_simd_support(self):
        """Initialize SIMD detection and configuration."""
        if SIMD_AVAILABLE:
            try:
                # Use centralized SIMD configuration from State Manager
                from feagi.core.state_manager import get_state_manager

                state_manager = get_state_manager()
                self.simd_config = state_manager.get_simd_configuration()

                if self.simd_config["available"]:
                    # Initialize SIMD components with centralized config
                    self.simd_detector = get_simd_detector()
                    self.simd_profiler = get_profiler()

                    # Log using centralized configuration
                    logger.info(
                        f"[BURST] Using centralized SIMD: {self.simd_config['backend']} "
                        f"(vector_width={self.simd_config['vector_width']}, "
                        f"alignment={self.simd_config['alignment']})"
                    )
                else:
                    logger.info(
                        "[BURST] Centralized SIMD reports: acceleration not available"
                    )
                    self.simd_detector = None
                    self.simd_profiler = None

            except Exception as e:
                logger.warning(
                    f"Failed to access centralized SIMD configuration: {e}"
                )
                self.simd_detector = None
                self.simd_config = {
                    "available": False,
                    "backend": "SCALAR",
                    "vector_width": 1,
                }
        else:
            self.simd_detector = None
            self.simd_config = {
                "available": False,
                "backend": "SCALAR",
                "vector_width": 1,
            }

    def initialize_membrane_processor(self, capacity: int):
        """Initialize SIMD-optimized membrane processor with given capacity."""
        if SIMD_AVAILABLE and self.membrane_processor is None:
            try:
                self.membrane_processor = SIMDMembraneProcessor(
                    capacity=capacity, use_profiling=self.use_simd_profiling
                )
                logger.info(
                    f"[BRAIN] SIMD membrane processor initialized for {capacity} neurons"
                )
            except Exception as e:
                logger.warning(
                    f"Failed to initialize SIMD membrane processor: {e}"
                )
                self.membrane_processor = None

    def measure_actual_frequency(
        self, duration_seconds: float = 5.0, sample_count: int = 100
    ) -> dict:
        """Measure both actual and potential burst frequencies over a specified
        period.

        This is an expensive operation that should only be called on-demand for monitoring
        or debugging purposes. It collects detailed timing data during burst processing.

        Args:
            duration_seconds: How long to collect timing data (default 5 seconds)
            sample_count: Number of burst samples to collect (default 100)

        Returns:
            Dictionary with measurement results including both actual and potential frequencies
        """
        import time

        running = getattr(self, "_running", False)
        if not running:
            raise RuntimeError(
                "Cannot measure frequency - burst engine is not running"
            )

        # Only log detailed frequency measurement start when debugging NPU
        debug_npu = getattr(self, "debug_npu", False)
        if debug_npu:
            logger.info(
                f"Starting frequency measurement for {duration_seconds}s",
                status="[DEBUG]",
            )

        # Enable frequency measurement mode
        old_measurement_enabled = getattr(
            self, "_frequency_measurement_enabled", False
        )
        self._frequency_measurement_enabled = True

        # Clear any existing timing buffers
        self._burst_timing_buffer.clear()
        self._processing_timing_buffer.clear()

        measurement_start = time.perf_counter()
        measurement_end = measurement_start + duration_seconds
        burst_count_start = getattr(self, "burst_count", 0)

        try:
            # Wait for measurements to be collected
            # The timing data will be collected automatically in the main burst loop
            while (
                time.perf_counter() < measurement_end
                and len(self._burst_timing_buffer) < sample_count
            ):
                # RTOS-COMPATIBLE: Replace time.sleep with CPU-friendly yield
                # In RTOS environment, use task yield or timer wait
                import time

                last_check = time.perf_counter()
                while time.perf_counter() - last_check < 0.01:
                    pass  # Busy-wait for 10ms equivalent

                # Safety check - ensure burst engine is still running
                if not getattr(self, "_running", False):
                    raise RuntimeError(
                        "Burst engine stopped during measurement"
                    )

            measurement_actual_duration = (
                time.perf_counter() - measurement_start
            )
            burst_count_end = getattr(self, "burst_count", 0)
            total_bursts_measured = burst_count_end - burst_count_start

            # Calculate frequency metrics from collected timing data
            if (
                not self._burst_timing_buffer
                or not self._processing_timing_buffer
            ):
                raise RuntimeError(
                    "No timing data collected during measurement period"
                )

            # Full cycle timing statistics (includes delays) - for actual frequency
            full_cycle_data_ms = [t * 1000 for t in self._burst_timing_buffer]
            min_cycle_time_ms = min(full_cycle_data_ms)
            max_cycle_time_ms = max(full_cycle_data_ms)
            avg_cycle_time_ms = statistics.mean(full_cycle_data_ms)
            cycle_std_dev_ms = (
                statistics.stdev(full_cycle_data_ms)
                if len(full_cycle_data_ms) > 1
                else 0.0
            )

            # Processing timing statistics (pure processing) - for potential frequency
            processing_data_ms = [
                t * 1000 for t in self._processing_timing_buffer
            ]
            min_processing_time_ms = min(processing_data_ms)
            max_processing_time_ms = max(processing_data_ms)
            avg_processing_time_ms = statistics.mean(processing_data_ms)
            processing_std_dev_ms = (
                statistics.stdev(processing_data_ms)
                if len(processing_data_ms) > 1
                else 0.0
            )

            # Frequency calculations
            avg_cycle_time_seconds = avg_cycle_time_ms / 1000.0
            avg_processing_time_seconds = avg_processing_time_ms / 1000.0

            actual_frequency_hz = (
                1.0 / avg_cycle_time_seconds
                if avg_cycle_time_seconds > 0
                else 0.0
            )
            potential_frequency_hz = (
                1.0 / avg_processing_time_seconds
                if avg_processing_time_seconds > 0
                else 0.0
            )

            # Alternative frequency calculation based on total time
            cycles_per_second = (
                total_bursts_measured / measurement_actual_duration
                if measurement_actual_duration > 0
                else 0.0
            )

            # Get target frequency
            desired_frequency = getattr(self, "desired_frequency", 1.0)

            measurement_result = {
                "actual_frequency_hz": actual_frequency_hz,
                "potential_frequency_hz": potential_frequency_hz,
                "alternative_frequency_hz": cycles_per_second,  # Alternative calculation method
                "target_frequency_hz": desired_frequency,
                "measurement_duration_s": measurement_actual_duration,
                "sample_count": len(self._burst_timing_buffer),
                "total_bursts_measured": total_bursts_measured,
                # Full cycle timing stats (actual frequency)
                "min_cycle_time_ms": min_cycle_time_ms,
                "max_cycle_time_ms": max_cycle_time_ms,
                "avg_cycle_time_ms": avg_cycle_time_ms,
                "cycle_std_dev_ms": cycle_std_dev_ms,
                # Processing timing stats (potential frequency)
                "min_processing_time_ms": min_processing_time_ms,
                "max_processing_time_ms": max_processing_time_ms,
                "avg_processing_time_ms": avg_processing_time_ms,
                "processing_std_dev_ms": processing_std_dev_ms,
                # Performance analysis
                "actual_performance_ratio": (
                    actual_frequency_hz / desired_frequency
                    if desired_frequency > 0
                    else 0.0
                ),
                "potential_performance_ratio": (
                    potential_frequency_hz / desired_frequency
                    if desired_frequency > 0
                    else 0.0
                ),
                "efficiency_ratio": (
                    actual_frequency_hz / potential_frequency_hz
                    if potential_frequency_hz > 0
                    else 0.0
                ),
                "headroom_hz": potential_frequency_hz - desired_frequency,
                # Debug data
                "cycle_timing_data_ms": full_cycle_data_ms[
                    -20:
                ],  # Include last 20 samples for debugging
                "processing_timing_data_ms": processing_data_ms[
                    -20:
                ],  # Include last 20 samples for debugging
            }

            # Only log detailed completion when debugging NPU
            if debug_npu:
                logger.info(
                    f"Frequency measurement complete - Actual: {actual_frequency_hz:.1f}Hz, Potential: {potential_frequency_hz:.1f}Hz (target: {desired_frequency:.1f}Hz)",
                    emoji1="[STATS]",
                )

            return measurement_result

        finally:
            # Restore previous measurement state
            self._frequency_measurement_enabled = old_measurement_enabled

            # Clear timing buffers to free memory
            if not self._frequency_measurement_enabled:
                self._burst_timing_buffer.clear()
                self._processing_timing_buffer.clear()

    def _record_burst_timing(self, burst_duration_seconds: float) -> None:
        """Record burst timing data if frequency measurement is enabled.

        Args:
            burst_duration_seconds: Duration of the burst in seconds
        """
        if not getattr(self, "_frequency_measurement_enabled", False):
            return

        # Add to circular buffer
        self._burst_timing_buffer.append(burst_duration_seconds)

        # Maintain buffer size
        if len(self._burst_timing_buffer) > self._timing_buffer_size:
            self._burst_timing_buffer.pop(0)  # Remove oldest entry

    def _record_processing_timing(
        self, processing_duration_seconds: float
    ) -> None:
        """Record processing timing data if frequency measurement is enabled.

        Args:
            processing_duration_seconds: Duration of the processing in seconds
        """
        if not getattr(self, "_frequency_measurement_enabled", False):
            return

        # Add to circular buffer
        self._processing_timing_buffer.append(processing_duration_seconds)

        # Maintain buffer size
        if len(self._processing_timing_buffer) > self._timing_buffer_size:
            self._processing_timing_buffer.pop(0)  # Remove oldest entry

    def burst(
        self, burst_size: Optional[int] = None, use_gpu: bool = False
    ) -> Dict[str, Any]:
        """Execute a neural processing burst with SIMD optimization.

        Args:
            burst_size: Number of neurons to process (None for auto-sizing)
            use_gpu: Whether to use GPU acceleration

        Returns:
            Burst execution results and performance metrics
        """

        burst_start = time.perf_counter()
        burst_results = {
            "neurons_processed": 0,
            "neurons_fired": 0,
            "processing_time": 0.0,
            "simd_efficiency": 0.0,
            "backend_used": "unknown",
        }

        try:
            # SIMD profiling session
            if SIMD_AVAILABLE and self.use_simd_profiling:
                with self.simd_profiler.profile_session(
                    f"burst_{getattr(self, 'burst_count', 0)}"
                ):
                    burst_results = self._execute_burst_impl(
                        burst_size, use_gpu, burst_start
                    )
            else:
                burst_results = self._execute_burst_impl(
                    burst_size, use_gpu, burst_start
                )

        except Exception as e:
            logger.error(f"Burst execution failed: {e}", exc_info=True)
            burst_results["error"] = str(e)

        # Update performance statistics
        if hasattr(self, "burst_count"):
            self.burst_count += 1
        self.total_neurons_processed += burst_results.get(
            "neurons_processed", 0
        )

        # Periodic performance reporting
        if (
            self.performance_monitoring
            and time.time() - self.last_performance_report > 10.0
        ):
            self._report_performance()
            self.last_performance_report = time.time()

        return burst_results

    def _execute_burst_impl(
        self, burst_size: Optional[int], use_gpu: bool, burst_start: float
    ) -> Dict[str, Any]:
        """Core burst execution implementation with SIMD optimization."""

        results = {
            "neurons_processed": 0,
            "neurons_fired": 0,
            "processing_time": 0.0,
            "simd_efficiency": 0.0,
            "backend_used": "scalar",
        }

        # Determine optimal burst size
        if burst_size is None:
            if (
                SIMD_AVAILABLE
                and hasattr(self, "backend_selector")
                and self.backend_selector
            ):
                burst_size = self.backend_selector.get_chunk_size(
                    10000
                )  # Default processing size
            else:
                burst_size = 1000  # Conservative default

        # Initialize membrane processor if needed
        if SIMD_AVAILABLE and self.membrane_processor is None:
            self.initialize_membrane_processor(burst_size * 2)  # Some headroom

        # Get fire candidates with SIMD optimization
        if SIMD_AVAILABLE and self.use_simd_profiling:
            with profile_simd_operation(
                "fire_candidate_detection", burst_size
            ):
                fire_candidates = self._get_fire_candidates_simd()
        else:
            fire_candidates = (
                self._get_fire_candidates_simd() if SIMD_AVAILABLE else []
            )

        # Process membrane potential updates with SIMD
        if fire_candidates and SIMD_AVAILABLE and self.membrane_processor:
            with profile_simd_operation(
                "membrane_processing", len(fire_candidates)
            ):
                fired_neurons = self._process_membrane_updates_simd(
                    fire_candidates
                )
        else:
            fired_neurons = fire_candidates  # Fallback

        # Update results
        results["neurons_processed"] = burst_size
        results["neurons_fired"] = len(fired_neurons)
        results["processing_time"] = time.perf_counter() - burst_start

        if SIMD_AVAILABLE:
            results["backend_used"] = self.simd_config.get(
                "recommended_backend", "scalar"
            )

            # Get SIMD efficiency from profiler
            if (
                self.use_simd_profiling
                and hasattr(self, "simd_profiler")
                and hasattr(self.simd_profiler, "current_session")
            ):
                session = self.simd_profiler.current_session
                if session and session.operations:
                    avg_efficiency = np.mean(
                        [
                            op.simd_efficiency
                            for op in session.operations.values()
                        ]
                    )
                    results["simd_efficiency"] = avg_efficiency

        return results

    def _get_fire_candidates_simd(self) -> List[int]:
        """Get fire candidates using SIMD-optimized detection."""

        # This would integrate with the actual connectome/GNA
        # For now, return empty list as placeholder
        if hasattr(self, "gna") and hasattr(
            self.gna, "simd_optimized_find_fire_candidates"
        ):
            return self.gna.simd_optimized_find_fire_candidates(
                getattr(self, "burst_count", 0)
            )

        return []

    def _process_membrane_updates_simd(
        self, candidates: List[int]
    ) -> List[int]:
        """Process membrane potential updates using SIMD optimization."""

        if not candidates or not self.membrane_processor:
            return candidates

        try:
            # Convert to arrays for SIMD processing
            candidate_indices = np.array(candidates, dtype=np.int32)
            input_currents = np.ones(
                len(candidates), dtype=np.float32
            )  # Placeholder currents

            # Use SIMD-optimized membrane update
            fired_neurons = self.membrane_processor.vectorized_membrane_update(
                candidate_indices, input_currents
            )

            return fired_neurons.tolist()

        except Exception as e:
            logger.warning(
                f"SIMD membrane processing failed, using fallback: {e}"
            )
            return candidates

    def _report_performance(self):
        """Report SIMD performance statistics."""

        if not SIMD_AVAILABLE:
            return

        try:
            # Get performance stats
            if self.membrane_processor:
                stats = self.membrane_processor.get_performance_stats()
                logger.info(
                    f"[START] SIMD Performance: {stats['avg_neurons_per_update']:.0f} neurons/update, "
                    f"backend: {stats['simd_backend']}, vector_width: {stats['vector_width']}"
                )

            # Get profiler report if available
            if (
                self.use_simd_profiling
                and hasattr(self, "simd_profiler")
                and hasattr(self.simd_profiler, "sessions")
            ):
                if self.simd_profiler.sessions:
                    report = self.simd_profiler.get_performance_report()
                    top_ops = report.get("top_operations", [])
                    if top_ops:
                        logger.info(
                            f"[DEBUG] Top SIMD operation: {top_ops[0]['name']} "
                            f"({top_ops[0]['simd_efficiency']:.2f} efficiency)"
                        )

        except Exception as e:
            logger.debug(f"Performance reporting failed: {e}")

    def get_performance_metrics(self) -> Dict[str, Any]:
        """Get comprehensive performance metrics.

        Returns:
            Dictionary with performance statistics
        """
        metrics = {
            "total_neurons_processed": self.total_neurons_processed,
            "simd_available": SIMD_AVAILABLE
            and self.simd_config.get("available", False),
            "simd_backend": self.simd_config.get("backend", "unknown"),
            "simd_vector_width": self.simd_config.get("vector_width", 1),
            "performance_monitoring_enabled": self.performance_monitoring,
            "frequency_measurement_enabled": self._frequency_measurement_enabled,
        }

        # Add timing buffer statistics if available
        if self._burst_timing_buffer:
            metrics["burst_timing_samples"] = len(self._burst_timing_buffer)
            metrics["avg_burst_time_ms"] = (
                np.mean(self._burst_timing_buffer) * 1000
            )

        if self._processing_timing_buffer:
            metrics["processing_timing_samples"] = len(
                self._processing_timing_buffer
            )
            metrics["avg_processing_time_ms"] = (
                np.mean(self._processing_timing_buffer) * 1000
            )

        # Add SIMD processor statistics if available
        if hasattr(self, "membrane_processor") and self.membrane_processor:
            try:
                simd_stats = self.membrane_processor.get_performance_stats()
                metrics["simd_processor_stats"] = simd_stats
            except Exception as e:
                metrics["simd_processor_error"] = str(e)

        return metrics

    def reset_performance_statistics(self) -> None:
        """Reset all performance statistics and buffers."""
        self.total_neurons_processed = 0
        self.last_performance_report = time.time()
        self._burst_timing_buffer.clear()
        self._processing_timing_buffer.clear()

        # Reset SIMD processor statistics if available
        if hasattr(self, "membrane_processor") and self.membrane_processor:
            try:
                self.membrane_processor.reset_performance_stats()
            except Exception as e:
                logger.warning(f"Failed to reset SIMD processor stats: {e}")

        logger.info("Performance statistics reset")

    def enable_frequency_measurement(self, enabled: bool = True) -> None:
        """Enable or disable frequency measurement.

        Args:
            enabled: Whether to enable frequency measurement
        """
        old_enabled = self._frequency_measurement_enabled
        self._frequency_measurement_enabled = enabled

        if enabled and not old_enabled:
            # Clear buffers when enabling
            self._burst_timing_buffer.clear()
            self._processing_timing_buffer.clear()
            logger.info("Frequency measurement enabled")
        elif not enabled and old_enabled:
            # Clear buffers when disabling to free memory
            self._burst_timing_buffer.clear()
            self._processing_timing_buffer.clear()
            logger.info("Frequency measurement disabled")
