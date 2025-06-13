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
FEAGI Burst Engine Debug Module

This module provides debugging and diagnostics functionality for the BurstEngine.
Includes FQ sampler registration, NPU debugging, and detailed fire queue analysis.

Features:
- Debug fire queue output with detailed breakdown
- FQ sampler registration and monitoring
- Motor/visualization stream debugging
- Memory and performance diagnostics
- NPU-specific debugging when --debug-npu flag is enabled

This is a mixin class designed to be combined with the core BurstEngine.
"""

from typing import Any, Dict, List, Optional

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class BurstEngineDebugMixin:
    """
    Debug functionality mixin for BurstEngine.

    Provides comprehensive debugging and diagnostics capabilities including:
    - Fire queue analysis and visualization
    - FQ sampler monitoring and registration
    - Motor and visualization stream debugging
    - Performance and memory diagnostics
    - NPU-specific debug output when enabled

    This mixin is designed to work with the core BurstEngine class.
    """

    def __init__(self, *args, **kwargs):
        """Initialize debug functionality. Called by main BurstEngine.__init__."""
        # DO NOT call super().__init__ in mixins - causes multiple inheritance issues
        # Just initialize our own attributes

        # FQ Sampler registry for debugging motor and visualization streams
        self._fq_samplers: List[Any] = []  # List of registered FQ samplers

        # Debug configuration (inherited from main BurstEngine)
        # self.debug_npu is set by the main class

        logger.debug("BurstEngine debug mixin initialized")

    def register_fq_sampler(self, fq_sampler: Any) -> None:
        """
        Register an FQ sampler for debugging and monitoring.

        Args:
            fq_sampler: FQSampler instance to register
        """
        if fq_sampler not in self._fq_samplers:
            self._fq_samplers.append(fq_sampler)
            if hasattr(self, "debug_npu") and self.debug_npu:
                logger.debug(
                    f"[DEBUG] NPU DEBUG: Registered FQ sampler - Total FQ samplers: {len(self._fq_samplers)}"
                )

    def unregister_fq_sampler(self, fq_sampler: Any) -> None:
        """
        Unregister an FQ sampler.

        Args:
            fq_sampler: FQSampler instance to unregister
        """
        if fq_sampler in self._fq_samplers:
            self._fq_samplers.remove(fq_sampler)
            if hasattr(self, "debug_npu") and self.debug_npu:
                logger.debug(
                    f"[DEBUG] NPU DEBUG: Unregistered FQ sampler - Total FQ samplers: {len(self._fq_samplers)}"
                )

    def _debug_fire_queue_output(self) -> None:
        """
        Debug fire queue output for NPU debugging.

        When --debug-npu flag is enabled, this method displays detailed information
        about the current fire queue contents including:
        - Global FCL summary
        - Per-cortical area breakdown
        - Neuron firing statistics
        - FQ Sampler status and data (for motor/visualization debugging)
        - FCL Sampler status and data (legacy)
        """
        try:
            burst_count = getattr(self, "burst_count", 0)
            logger.debug(f"\n[DEBUG] ===== NPU DEBUG - BURST {burst_count} =====")

            # Get global FCL
            fcl_manager = getattr(self, "fcl_manager", None)
            if not fcl_manager:
                logger.debug("[DEBUG] No FCL manager available for debug output")
                return

            global_fcl = fcl_manager.get_global_fcl()
            total_firing = len(global_fcl)

            logger.debug("[STATS] Global Fire Summary:")
            logger.debug(f"   Total firing neurons: {total_firing}")

            # Get burst interval if available
            burst_interval = getattr(self, "burst_interval", 1.0)
            logger.debug(f"   Burst frequency: {1.0 / burst_interval:.1f}Hz target")

            if total_firing > 0:
                # Get firing neurons by cortical area
                fcl_by_cortical = fcl_manager.get_fcl_by_cortical()

                logger.debug(
                    f"[BRAIN] Per-Area Breakdown ({len(fcl_by_cortical)} active areas):"
                )

                # Sort areas by number of firing neurons for consistent output
                sorted_areas = sorted(
                    fcl_by_cortical.items(), key=lambda x: len(x[1]), reverse=True
                )

                for cortical_id, area_fcl in sorted_areas:
                    area_count = len(area_fcl)
                    percentage = (
                        (area_count / total_firing) * 100 if total_firing > 0 else 0
                    )

                    # Display first few neurons for small lists, summarize for large ones
                    if area_count <= 10:
                        neuron_list = sorted(list(area_fcl))
                        logger.debug(
                            f"   {cortical_id}: {area_count} neurons ({percentage:.1f}%) - {neuron_list}"
                        )
                    else:
                        neuron_sample = sorted(list(area_fcl))[:5]
                        logger.debug(
                            f"   {cortical_id}: {area_count} neurons ({percentage:.1f}%) - {neuron_sample}... (+{area_count - 5} more)"
                        )

                # Show special area injection info if available (all area types)
                if hasattr(self, "injection_service") and self.injection_service:
                    stats = self.get_injection_statistics()
                    if (
                        "total_injections" in stats
                        and stats.get("total_injections", 0) > 0
                    ):
                        total_neurons = stats.get("total_neurons_injected", 0)
                        total_batches = sum(stats.get("prepared_batches", {}).values())
                        logger.debug(
                            f"[FAST] Special Area Injection: {total_neurons} neurons from {total_batches} batches (all area types)"
                        )
            else:
                logger.debug("   No neurons firing this burst")

            # Show recent firing statistics if available
            if hasattr(fcl_manager, "get_firing_statistics"):
                firing_stats = fcl_manager.get_firing_statistics()
                if firing_stats:
                    logger.debug("[UP] Recent Activity:")
                    logger.debug(
                        f"   Average firing rate: {firing_stats.get('average_firing_rate', 0):.1f} neurons/burst"
                    )
                    logger.debug(
                        f"   Peak firing: {firing_stats.get('peak_firing', 0)} neurons"
                    )

            # === FQ SAMPLER DEBUG INFORMATION ===
            logger.debug("[TARGET] Sampler Debug Information:")

            # FQ Samplers (for motor and visualization)
            if self._fq_samplers:
                logger.debug(f"   FQ Samplers Active: {len(self._fq_samplers)}")
                for i, fq_sampler in enumerate(self._fq_samplers):
                    try:
                        sampler_name = f"FQSampler-{i + 1}"
                        running_status = (
                            "RUNNING"
                            if getattr(fq_sampler, "running", False)
                            else "STOPPED"
                        )
                        sample_freq = getattr(fq_sampler, "sample_frequency", 0)

                        # Get subscriber status
                        viz_subs = getattr(
                            fq_sampler, "_has_visualization_subscribers", False
                        )
                        motor_subs = getattr(
                            fq_sampler, "_has_motor_subscribers", False
                        )

                        logger.debug(
                            f"      {sampler_name}: {running_status} @ {sample_freq:.1f}Hz"
                        )
                        logger.debug(
                            f"         Viz subscribers: {'YES' if viz_subs else 'NO'}"
                        )
                        logger.debug(
                            f"         Motor subscribers: {'YES' if motor_subs else 'NO'}"
                        )

                        # Try to get sample data for current burst
                        if hasattr(fq_sampler, "_get_global_fire_queue_data"):
                            sample_data = fq_sampler._get_global_fire_queue_data()
                            if sample_data and sample_data.get("neuron_ids"):
                                sample_count = len(sample_data["neuron_ids"])
                                logger.debug(
                                    f"         [STATS] Sample data: {sample_count} neurons"
                                )

                                # Show membrane potential range if available
                                if sample_data.get("membrane_potentials"):
                                    potentials = sample_data["membrane_potentials"]
                                    min_pot = min(potentials)
                                    max_pot = max(potentials)
                                    avg_pot = sum(potentials) / len(potentials)
                                    logger.debug(
                                        f"         [BRAIN] Membrane potentials: {min_pot:.2f} - {max_pot:.2f} (avg: {avg_pot:.2f})"
                                    )

                                # Show coordinate range if available
                                if sample_data.get("coordinates"):
                                    coords = sample_data["coordinates"]
                                    if coords:
                                        x_coords = [c[0] for c in coords]
                                        y_coords = [c[1] for c in coords]
                                        z_coords = [c[2] for c in coords]
                                        logger.debug(
                                            f"         Coordinate ranges: X:{min(x_coords)}-{max(x_coords)} Y:{min(y_coords)}-{max(y_coords)} Z:{min(z_coords)}-{max(z_coords)}"
                                        )
                            else:
                                logger.debug(
                                    "         [STATS] Sample data: No neurons firing"
                                )

                        # Check queue status
                        if hasattr(fq_sampler, "output_queue"):
                            try:
                                queue_size = fq_sampler.output_queue.qsize()
                                logger.debug(
                                    f"         Output queue: {queue_size} items"
                                )
                            except Exception:
                                logger.debug("         Output queue: Status unknown")

                    except Exception as sampler_error:
                        logger.debug(
                            f"      FQSampler-{i + 1}: ERROR - {sampler_error}"
                        )
            else:
                logger.debug("   FQ Samplers: NONE REGISTERED")

            # === MOTOR/VISUALIZATION STREAM DEBUGGING ===
            if self._fq_samplers:
                logger.debug("Motor & Visualization Stream Debug:")

                # Check if any samplers are active for motor output
                motor_active_count = 0
                viz_active_count = 0

                for sampler in self._fq_samplers:
                    if getattr(sampler, "_has_motor_subscribers", False):
                        motor_active_count += 1
                    if getattr(sampler, "_has_visualization_subscribers", False):
                        viz_active_count += 1

                logger.debug(f"   Motor stream: {motor_active_count} active samplers")
                logger.debug(
                    f"   Visualization stream: {viz_active_count} active samplers"
                )

                if motor_active_count == 0 and viz_active_count == 0:
                    logger.debug(
                        "   [WARN]  WARNING: No active subscribers - streams may be inactive!"
                    )

                # Sample recent data from each active FQ sampler
                for i, fq_sampler in enumerate(self._fq_samplers):
                    if getattr(fq_sampler, "running", False) and (
                        getattr(fq_sampler, "_has_visualization_subscribers", False)
                        or getattr(fq_sampler, "_has_motor_subscribers", False)
                    ):
                        logger.debug(f"   [STATS] FQSampler-{i + 1} Recent Sample:")

                        # Try to get per-area samples for key areas
                        if (
                            hasattr(fq_sampler, "connectome_manager")
                            and fq_sampler.connectome_manager
                        ):
                            try:
                                # Sample a few key areas (motor and sensory)
                                sample_areas = [
                                    "motor_",
                                    "vision",
                                    "sensor",
                                    "output",
                                    "input",
                                ]  # Common prefixes
                                sampled_any = False

                                # Get FCL by cortical areas for sampling
                                fcl_by_cortical = fcl_manager.get_fcl_by_cortical()
                                for area_id in list(fcl_by_cortical.keys())[
                                    :5
                                ]:  # Sample first 5 active areas
                                    if hasattr(fq_sampler, "_get_area_fire_queue_data"):
                                        area_sample = (
                                            fq_sampler._get_area_fire_queue_data(
                                                area_id
                                            )
                                        )
                                        if area_sample and area_sample.get(
                                            "neuron_ids"
                                        ):
                                            neuron_count = len(
                                                area_sample["neuron_ids"]
                                            )
                                            logger.debug(
                                                f"      {area_id}: {neuron_count} active neurons"
                                            )
                                            sampled_any = True

                                if not sampled_any:
                                    logger.debug("      No area samples available")

                            except Exception as sample_error:
                                logger.debug(f"      Sample error: {sample_error}")

            logger.debug("[DEBUG] ========================================\n")

        except Exception as e:
            logger.error(f"[DEBUG] NPU DEBUG ERROR: Failed to display fire queue - {e}")
            logger.error(f"NPU debug output error: {e}")
            # Include stack trace for debugging
            import traceback

            logger.error("[DEBUG] NPU DEBUG ERROR stack trace:")
            logger.error(traceback.format_exc())

    def get_debug_statistics(self) -> Dict[str, Any]:
        """
        Get comprehensive debug statistics for the burst engine.

        Returns:
            Dictionary containing debug information about samplers, performance, etc.
        """
        stats = {
            "fq_samplers_registered": len(self._fq_samplers),
            "debug_mode_enabled": getattr(self, "debug_npu", False),
            "burst_count": getattr(self, "burst_count", 0),
        }

        # FQ Sampler details
        if self._fq_samplers:
            sampler_stats = []
            for i, sampler in enumerate(self._fq_samplers):
                sampler_info = {
                    "index": i,
                    "running": getattr(sampler, "running", False),
                    "sample_frequency": getattr(sampler, "sample_frequency", 0),
                    "sampling_mode": getattr(sampler, "sampling_mode", "unknown"),
                    "performance_stats": getattr(
                        sampler, "get_performance_stats", lambda: {}
                    )(),
                }
                sampler_stats.append(sampler_info)
            stats["fq_samplers"] = sampler_stats

        # Special area injection statistics if available (all area types)
        if hasattr(self, "get_injection_statistics"):
            try:
                stats["injection_statistics"] = self.get_injection_statistics()
            except Exception as e:
                stats["injection_statistics_error"] = str(e)

        return stats

    def debug_burst_performance(
        self, burst_duration: float, processing_duration: float
    ) -> None:
        """
        Debug burst performance information.

        Args:
            burst_duration: Total burst duration including delays
            processing_duration: Pure processing time without delays
        """
        if not hasattr(self, "debug_npu") or not self.debug_npu:
            return

        burst_count = getattr(self, "burst_count", 0)
        desired_frequency = getattr(self, "desired_frequency", 1.0)

        actual_freq = 1.0 / burst_duration if burst_duration > 0 else 0
        potential_freq = 1.0 / processing_duration if processing_duration > 0 else 0

        logger.debug(f"[DEBUG] BURST PERFORMANCE - Burst {burst_count}:")
        logger.debug(f"   Processing time: {processing_duration * 1000:.2f}ms")
        logger.debug(f"   Total cycle time: {burst_duration * 1000:.2f}ms")
        logger.debug(f"   Target frequency: {desired_frequency:.1f}Hz")
        logger.debug(f"   Actual frequency: {actual_freq:.1f}Hz")
        logger.debug(f"   Potential frequency: {potential_freq:.1f}Hz")
        logger.debug(f"   Performance ratio: {actual_freq / desired_frequency:.2f}")

    def debug_memory_usage(self) -> Optional[Dict[str, Any]]:
        """
        Debug memory usage information.

        Returns:
            Dictionary with memory usage statistics or None if unavailable
        """
        try:
            import psutil

            process = psutil.Process()
            memory_info = process.memory_info()

            memory_stats = {
                "rss_mb": memory_info.rss / 1024 / 1024,  # Resident Set Size
                "vms_mb": memory_info.vms / 1024 / 1024,  # Virtual Memory Size
                "memory_percent": process.memory_percent(),
                "fq_samplers_count": len(self._fq_samplers),
            }

            # Add buffer information if available
            for i, sampler in enumerate(self._fq_samplers):
                if hasattr(sampler, "max_neurons_per_sample"):
                    buffer_size_mb = (
                        sampler.max_neurons_per_sample * 6 * 4 / 1024 / 1024
                    )  # 6 floats * 4 bytes
                    memory_stats[f"sampler_{i}_buffer_mb"] = buffer_size_mb

            if hasattr(self, "debug_npu") and self.debug_npu:
                logger.debug(f"[DEBUG] MEMORY USAGE: {memory_stats}")

            return memory_stats

        except ImportError:
            logger.warning("psutil not available for memory debugging")
            return None
        except Exception as e:
            logger.error(f"Error getting memory usage: {e}")
            return None

    def debug_validate_state(self) -> Dict[str, bool]:
        """
        Validate internal state consistency for debugging.

        Returns:
            Dictionary of validation results
        """
        validation = {
            "fcl_manager_available": hasattr(self, "fcl_manager")
            and self.fcl_manager is not None,
            "connectome_manager_available": hasattr(self, "connectome_manager")
            and self.connectome_manager is not None,
            "state_manager_available": hasattr(self, "state_manager")
            and self.state_manager is not None,
            "fq_samplers_consistent": True,
            "singleton_pattern_ok": True,
        }

        # Validate FQ samplers
        try:
            for sampler in self._fq_samplers:
                if not hasattr(sampler, "sample_frequency"):
                    validation["fq_samplers_consistent"] = False
                    break
        except Exception:
            validation["fq_samplers_consistent"] = False

        # Validate singleton pattern
        try:
            if hasattr(self.__class__, "_instance"):
                validation["singleton_pattern_ok"] = self.__class__._instance is self
        except Exception:
            validation["singleton_pattern_ok"] = False

        if hasattr(self, "debug_npu") and self.debug_npu:
            failed_validations = [k for k, v in validation.items() if not v]
            if failed_validations:
                logger.warning(f"[DEBUG] STATE VALIDATION FAILED: {failed_validations}")
            else:
                logger.debug("[DEBUG] STATE VALIDATION: All checks passed")

        return validation
