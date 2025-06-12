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

"""Brain service for managing FEAGI brain simulation operations."""

import os
from typing import Any, Dict, List, Optional

from ..shared.base_service import BaseService


class BrainService(BaseService):
    """
    Brain service handles brain simulation operations including
    burst engine control, monitoring, and analysis.

    CRITICAL: This service NEVER creates its own BurstEngine instance.
    It always uses the singleton instance from the BurstEngine class.
    """

    def _get_burst_engine(self):
        """Get the singleton burst engine instance. Never creates a new one."""
        try:
            from feagi.npu.burst_engine import BurstEngine

            return BurstEngine.get_instance()
        except Exception as e:
            self.logger.error(f"Error getting burst engine singleton: {str(e)}")
            return None

    def get_burst_engine_status(self) -> Dict[str, Any]:
        """Get current burst engine status."""
        try:
            if not self.state_manager:
                return {
                    "status": "unknown",
                    "error": "State manager not available",
                    "is_running": False,
                }

            # Get the actual burst engine instance to check its _running flag
            burst_engine = self._get_burst_engine()
            if burst_engine:
                # Check the actual burst engine's _running flag (the correct source of truth)
                is_running = burst_engine._running
                current_burst = getattr(burst_engine, "burst_count", 0)
            else:
                # Fallback to state manager if burst engine not available
                is_running = not getattr(self.state_manager, "exit_condition", False)
                current_burst = getattr(self.state_manager, "current_burst_id", 0)

            return {
                "status": "running" if is_running else "stopped",
                "is_running": is_running,  # ✅ FIXED: Include the is_running field
                "current_burst": current_burst,
                "brain_ready": self.state_manager.get_brain_readiness(),
                "genome_loaded": self.state_manager.is_genome_loaded(),
            }
        except Exception as e:
            self.logger.error(f"Error getting burst engine status: {str(e)}")
            return {"status": "error", "error": str(e), "is_running": False}

    def start_burst_engine(self) -> bool:
        """Start the burst engine."""
        # Unconditional debug logging
        try:
            with open("/tmp/feagi_injection_debug.log", "a") as f:
                import datetime

                f.write(
                    f"{datetime.datetime.now()}: Brain service start_burst_engine() called\n"
                )
        except:
            pass

        try:
            if not self.state_manager:
                if os.environ.get("FEAGI_DEBUG_NPU") == "1":
                    print(f"[DEBUG] BRAIN SERVICE: No state manager available")
                return False

            # Get the singleton burst engine instance
            burst_engine = self._get_burst_engine()
            if not burst_engine:
                if os.environ.get("FEAGI_DEBUG_NPU") == "1":
                    print(f"[DEBUG] BRAIN SERVICE: No burst engine instance available")
                self.logger.error("No burst engine instance available")
                return False

            self.logger.debug(
                f"BRAIN SERVICE: Got burst engine instance {burst_engine._instance_id}"
            )
            self.logger.debug(
                f"BRAIN SERVICE: Current _running state: {burst_engine._running}"
            )

            # Check if it's already running
            if burst_engine._running:
                self.logger.debug(
                    "BRAIN SERVICE: Burst engine reports _running=True, skipping start"
                )
                self.logger.info("Burst engine is already running")
                return True

            self.logger.debug(
                "BRAIN SERVICE: Burst engine _running=False, proceeding to start"
            )

            # Clear exit condition to start the burst engine
            self.state_manager.exit_condition = False

            # CRITICAL: Actually start the burst engine main loop in a background thread
            import threading

            def run_burst_engine():
                """Background thread function to run the burst engine main loop"""
                try:
                    self.logger.debug(
                        "BRAIN SERVICE: Background thread starting, about to call burst_engine.run()"
                    )
                    self.logger.info(
                        "BRAIN SERVICE: Starting burst engine main loop in background thread"
                    )
                    burst_engine.run()
                    self.logger.debug("BRAIN SERVICE: burst_engine.run() returned")
                except Exception as e:
                    self.logger.debug(
                        f"BRAIN SERVICE: Exception in burst engine main loop: {str(e)}"
                    )
                    self.logger.error(
                        f"BRAIN SERVICE: Error in burst engine main loop: {str(e)}"
                    )
                    # Set burst engine state to ERROR on exception
                    from feagi.core.state_manager import ServiceState

                    self.state_manager.set_burst_engine_state(ServiceState.ERROR)

            self.logger.debug("BRAIN SERVICE: Creating background thread")

            # Start the burst engine in a daemon thread
            burst_thread = threading.Thread(target=run_burst_engine, daemon=True)
            burst_thread.start()

            self.logger.debug(
                "BRAIN SERVICE: Background thread started, using event-based synchronization..."
            )

            # RTOS-COMPATIBLE: Event-based synchronization instead of sleep polling
            startup_event = threading.Event()
            startup_success = False

            def monitor_startup():
                """Monitor burst engine startup and signal completion"""
                nonlocal startup_success
                # Fixed iteration count for deterministic behavior
                max_iterations = 200  # ~2 seconds at 100Hz check rate
                iteration = 0

                while iteration < max_iterations:
                    # RTOS: Busy-wait with minimal yield instead of sleep
                    # This is deterministic and suitable for real-time systems
                    for _ in range(1000):  # Busy-wait inner loop
                        pass

                    # Check if burst engine is running
                    if burst_engine._running:
                        startup_success = True
                        startup_event.set()
                        return

                    iteration += 1

                    # RTOS: Minimal CPU yield for cooperative multitasking
                    if iteration % 10 == 0:  # Every 10th iteration
                        # Use os.sched_yield() for RTOS compatibility if available
                        try:
                            import os

                            if hasattr(os, "sched_yield"):
                                os.sched_yield()
                        except (ImportError, AttributeError):
                            pass  # No yield available - pure busy wait

                # Timeout reached
                startup_event.set()

            # Start monitoring thread
            monitor_thread = threading.Thread(target=monitor_startup, daemon=True)
            monitor_thread.start()

            # RTOS: Wait for event with timeout (deterministic)
            event_triggered = startup_event.wait(timeout=3.0)  # Max 3 second timeout

            self.logger.debug(
                f"BRAIN SERVICE: Event triggered: {event_triggered}, Success: {startup_success}"
            )

            # Verify startup success
            if startup_success and burst_engine._running:
                self.logger.debug("BRAIN SERVICE: Success! Burst engine is now running")
                self.logger.info(
                    "Burst engine started successfully in background thread"
                )

                # CRITICAL: If there's already a genome loaded, update the burst engine with it
                # This ensures injection service gets initialized for existing genomes
                genome_loaded = (
                    self.state_manager and self.state_manager.is_genome_loaded()
                )
                self.logger.debug(
                    f"BRAIN SERVICE: Checking genome status - loaded: {genome_loaded}"
                )
                # Write to debug file
                try:
                    with open("/tmp/feagi_injection_debug.log", "a") as f:
                        import datetime

                        f.write(
                            f"{datetime.datetime.now()}: Brain service start - genome loaded: {genome_loaded}\n"
                        )
                except:
                    pass

                if genome_loaded:
                    self.logger.debug(
                        "BRAIN SERVICE: Genome already loaded, calling update_with_genome()"
                    )
                    try:
                        with open("/tmp/feagi_injection_debug.log", "a") as f:
                            import datetime

                            f.write(
                                f"{datetime.datetime.now()}: Brain service calling update_with_genome()\n"
                            )
                        burst_engine.update_with_genome()
                        self.logger.info(
                            "Updated burst engine with existing genome - injection service initialized"
                        )
                    except Exception as e:
                        self.logger.warning(
                            f"Failed to update burst engine with existing genome: {str(e)}"
                        )
                        try:
                            with open("/tmp/feagi_injection_debug.log", "a") as f:
                                import datetime

                                f.write(
                                    f"{datetime.datetime.now()}: Brain service error: {str(e)}\n"
                                )
                        except:
                            pass

                return True
            else:
                self.logger.debug(
                    "BRAIN SERVICE: FAILED! Burst engine _running is still False"
                )
                self.logger.error("Failed to start burst engine main loop")
                return False

        except Exception as e:
            self.logger.debug(
                f"BRAIN SERVICE: Exception in start_burst_engine: {str(e)}"
            )
            self.logger.error(f"Error starting burst engine: {str(e)}")
            return False

    def stop_burst_engine(self) -> bool:
        """Stop the burst engine."""
        try:
            if not self.state_manager:
                return False

            # Get the singleton burst engine instance
            burst_engine = self._get_burst_engine()
            if not burst_engine:
                self.logger.error("No burst engine instance available")
                return False

            # Check if it's already stopped
            if not burst_engine._running:
                self.logger.info("Burst engine is already stopped")
                return True

            # Stop the burst engine main loop
            self.logger.info("[DEBUG] BRAIN SERVICE: Stopping burst engine main loop")
            burst_engine.stop()

            # Set exit condition to stop the burst engine
            self.state_manager.exit_condition = True

            # Wait a moment for the thread to stop
            import time

            time.sleep(0.2)  # Allow thread to stop

            # Verify it's stopped
            if not burst_engine._running:
                # Set burst engine state to UNAVAILABLE
                from feagi.core.state_manager import ServiceState

                self.state_manager.set_burst_engine_state(ServiceState.UNAVAILABLE)
                self.logger.info("Burst engine stopped successfully")
                return True
            else:
                self.logger.warning(
                    "Burst engine may still be running after stop request"
                )
                return False

        except Exception as e:
            self.logger.error(f"Error stopping burst engine: {str(e)}")
            return False

    def hold_burst_engine(self) -> bool:
        """Put burst engine on hold (pause neural processing)."""
        try:
            if not self.state_manager:
                return False

            # Check if burst engine is currently running
            from feagi.core.state_manager import ServiceState

            current_state = self.state_manager.get_burst_engine_state()

            if current_state != ServiceState.READY:
                self.logger.warning(
                    f"Cannot hold burst engine - current state: {current_state.name}"
                )
                return False

            # Set engine to ON_HOLD (keeps engine alive but pauses processing)
            self.state_manager.set_burst_engine_state(ServiceState.ON_HOLD)

            self.logger.info("Burst engine put on hold - neural processing paused")
            return True
        except Exception as e:
            self.logger.error(f"Error putting burst engine on hold: {str(e)}")
            return False

    def resume_burst_engine(self) -> bool:
        """Resume burst engine from hold (resume neural processing)."""
        try:
            if not self.state_manager:
                return False

            # Check if burst engine is currently on hold
            from feagi.core.state_manager import ServiceState

            current_state = self.state_manager.get_burst_engine_state()

            if current_state != ServiceState.ON_HOLD:
                self.logger.warning(
                    f"Cannot resume burst engine - current state: {current_state.name}"
                )
                return False

            # Resume processing by setting state back to READY
            self.state_manager.set_burst_engine_state(ServiceState.READY)

            self.logger.info("Burst engine resumed - neural processing active")
            return True
        except Exception as e:
            self.logger.error(f"Error resuming burst engine: {str(e)}")
            return False

    def get_brain_statistics(self) -> Dict[str, Any]:
        """Get comprehensive brain statistics."""
        try:
            if not self.state_manager:
                return {}

            # Get brain stats from state manager
            brain_stats = getattr(self.state_manager, "brain_stats", {})

            # Combine with connectome statistics
            stats = {
                "neuron_count": brain_stats.get("neuron_count", 0),
                "synapse_count": brain_stats.get("synapse_count", 0),
                "cortical_area_count": len(
                    getattr(self.state_manager, "cortical_list", [])
                ),
                "genome_loaded": self.state_manager.is_genome_loaded(),
                "brain_ready": self.state_manager.get_brain_readiness(),
            }

            # Add connectome manager stats if available
            if self._connectome_manager:
                stats.update(
                    {
                        "active_cortical_areas": len(
                            getattr(self._connectome_manager, "cortical_areas", {})
                        ),
                        "current_timestep": getattr(
                            self._connectome_manager, "current_timestep", 0
                        ),
                    }
                )

            return stats
        except Exception as e:
            self.logger.error(f"Error getting brain statistics: {str(e)}")
            return {}

    def get_activity_summary(self, window: int = 10) -> Dict[str, Any]:
        """Get activity summary for the brain over a time window."""
        try:
            if not self._validate_genome_loaded():
                return {}

            # Get current timestep
            current_time = getattr(self._connectome_manager, "current_timestep", 0)

            # Count active neurons across all areas
            total_active = 0
            total_neurons = 0
            area_activity = {}

            for area_idx, area in self._connectome_manager.cortical_areas.items():
                area_neurons = self._connectome_manager.get_neurons_by_area(area_idx)
                area_active = 0

                for neuron_id in area_neurons:
                    neuron_index = self._connectome_manager._neuron_id_to_index.get(
                        neuron_id
                    )
                    if neuron_index is None:
                        continue

                    last_fired = int(self._connectome_manager.last_fired[neuron_index])
                    if last_fired > 0 and (current_time - last_fired) <= window:
                        area_active += 1

                total_active += area_active
                total_neurons += len(area_neurons)

                area_activity[str(area_idx)] = {
                    "name": area.name,
                    "active_neurons": area_active,
                    "total_neurons": len(area_neurons),
                    "activity_ratio": (
                        area_active / len(area_neurons) if area_neurons else 0
                    ),
                }

            return {
                "total_active_neurons": total_active,
                "total_neurons": total_neurons,
                "overall_activity_ratio": (
                    total_active / total_neurons if total_neurons > 0 else 0
                ),
                "current_timestep": current_time,
                "time_window": window,
                "area_activity": area_activity,
            }
        except Exception as e:
            self.logger.error(f"Error getting activity summary: {str(e)}")
            return {}

    def reset_brain_state(self) -> bool:
        """Reset the brain to initial state."""
        try:
            if not self._connectome_manager:
                return False

            # Reset all neural states
            if hasattr(self._connectome_manager, "reset_neural_states"):
                self._connectome_manager.reset_neural_states()

            # Reset timestep
            if hasattr(self._connectome_manager, "current_timestep"):
                self._connectome_manager.current_timestep = 0

            # Clear FCL if available
            if hasattr(self._connectome_manager, "reset_fcl"):
                self._connectome_manager.reset_fcl()

            return True
        except Exception as e:
            self.logger.error(f"Error resetting brain state: {str(e)}")
            return False

    def get_performance_metrics(self) -> Dict[str, Any]:
        """Get brain performance metrics."""
        try:
            if not self.state_manager:
                return {}

            # Get basic performance metrics
            metrics = {
                "burst_duration_ms": getattr(
                    self.state_manager, "last_burst_duration", 0
                ),
                "bursts_per_second": getattr(
                    self.state_manager, "bursts_per_second", 0
                ),
                "neurons_processed": getattr(
                    self.state_manager, "neurons_processed_last_burst", 0
                ),
                "synapses_processed": getattr(
                    self.state_manager, "synapses_processed_last_burst", 0
                ),
            }

            # Add memory usage if available
            try:
                import psutil

                process = psutil.Process()
                metrics.update(
                    {
                        "memory_usage_mb": process.memory_info().rss / 1024 / 1024,
                        "cpu_percent": process.cpu_percent(),
                    }
                )
            except ImportError:
                pass

            return metrics
        except Exception as e:
            self.logger.error(f"Error getting performance metrics: {str(e)}")
            return {}

    def stimulate_neurons(
        self, neuron_ids: List[str], intensity: float = 1.0
    ) -> Dict[str, Any]:
        """Stimulate specific neurons with given intensity."""
        try:
            if not self._validate_genome_loaded():
                return {"success": False, "error": "No genome loaded"}

            stimulated_count = 0
            failed_count = 0

            for neuron_id in neuron_ids:
                try:
                    neuron_index = self._connectome_manager._neuron_id_to_index.get(
                        neuron_id
                    )
                    if neuron_index is None:
                        failed_count += 1
                        continue

                    # Apply stimulation by setting membrane potential
                    self._connectome_manager.membrane_potentials[neuron_index] = (
                        intensity
                    )
                    stimulated_count += 1
                except Exception as e:
                    self.logger.warning(
                        f"Failed to stimulate neuron {neuron_id}: {str(e)}"
                    )
                    failed_count += 1

            return {
                "success": True,
                "stimulated_count": stimulated_count,
                "failed_count": failed_count,
                "total_requested": len(neuron_ids),
                "intensity": intensity,
            }
        except Exception as e:
            self.logger.error(f"Error stimulating neurons: {str(e)}")
            return {"success": False, "error": str(e)}

    def get_burst_engine_config(self) -> Dict[str, Any]:
        """Get burst engine configuration - RTOS-safe."""
        try:
            # Get real configuration from burst engine
            burst_engine = self._get_burst_engine()
            if burst_engine:
                # RTOS-SAFE: Get current frequency configuration
                frequency_config = burst_engine.get_frequency_config()

                # Combine with state manager configuration
                base_config = {
                    "burst_frequency_hz": frequency_config.get(
                        "current_frequency_hz", 10.0
                    ),
                    "burst_interval_seconds": frequency_config.get(
                        "burst_interval_seconds", 0.1
                    ),
                    "target_frequency_hz": frequency_config.get(
                        "target_frequency_hz", 10.0
                    ),
                    "max_neurons_per_burst": (
                        getattr(self.state_manager, "max_neurons_per_burst", 1000)
                        if self.state_manager
                        else 1000
                    ),
                    "burst_timeout_ms": (
                        getattr(self.state_manager, "burst_timeout", 1000)
                        if self.state_manager
                        else 1000
                    ),
                    "auto_restart": (
                        getattr(self.state_manager, "auto_restart", True)
                        if self.state_manager
                        else True
                    ),
                    "performance_mode": (
                        getattr(self.state_manager, "performance_mode", "normal")
                        if self.state_manager
                        else "normal"
                    ),
                }

                return base_config
            else:
                # Fallback configuration when burst engine is not available
                return {
                    "burst_frequency_hz": 10.0,
                    "burst_interval_seconds": 0.1,
                    "target_frequency_hz": 10.0,
                    "max_neurons_per_burst": 1000,
                    "burst_timeout_ms": 1000,
                    "auto_restart": True,
                    "performance_mode": "normal",
                    "error": "Burst engine not available",
                }

        except Exception as e:
            self.logger.error(f"Error getting burst engine config: {str(e)}")
            return {"error": str(e)}

    def get_burst_timer(self) -> float:
        """Get burst timer (stimulation period) from burst engine."""
        try:
            if self.state_manager:
                # Get burst frequency and convert to period in seconds
                frequency = getattr(self.state_manager, "burst_frequency", 1.0)
                if frequency > 0:
                    return 1.0 / frequency
                return 1.0  # Default 1 second period
            return 1.0
        except Exception as e:
            self.logger.error(f"Error getting burst timer: {str(e)}")
            return 1.0
