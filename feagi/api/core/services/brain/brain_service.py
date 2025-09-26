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

"""Brain service for managing FEAGI brain simulation operations."""

from typing import Any, Dict, List

import numpy as np

from ..shared.base_service import BaseService


class BrainService(BaseService):
    """Brain service handles brain simulation operations including burst engine
    control, monitoring, and analysis.

    CRITICAL: This service NEVER creates its own BurstEngine instance.
    It always uses the singleton instance from the BurstEngine class.
    """

    def _get_burst_engine(self):
        """Get the singleton burst engine instance.

        Never creates a new one.
        """
        try:
            from feagi.npu.burst_engine import BurstEngine

            return BurstEngine.get_instance()
        except Exception as e:
            self.logger.error(
                f"Error getting burst engine singleton: {str(e)}"
            )
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
                #  Check the actual burst engine's _running flag (the correct
                #  source of truth)
                is_running = burst_engine._running
                current_burst = getattr(burst_engine, "burst_count", 0)
            else:
                # Fallback to state manager if burst engine not available
                is_running = not getattr(
                    self.state_manager, "exit_condition", False
                )
                current_burst = getattr(
                    self.state_manager, "current_burst_id", 0
                )

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
        # Debug-only logging
        try:
            if self.state_manager.is_debug_npu_enabled():
                import datetime
                import os
                import tempfile

                log_path = os.path.join(
                    tempfile.gettempdir(), "feagi_injection_debug--temp.log"
                )
                with open(log_path, "a") as f:
                    f.write(
                        f"{datetime.datetime.now()}: Brain service start_burst_engine() called\n"
                    )
        except Exception:
            pass

        try:
            if not self.state_manager:
                # Can't check debug flag without state manager
                print("[DEBUG] BRAIN SERVICE: No state manager available")
                return False

            # Get the singleton burst engine instance
            burst_engine = self._get_burst_engine()
            if not burst_engine:
                if self.state_manager.is_debug_npu_enabled():
                    print(
                        "[DEBUG] BRAIN SERVICE: No burst engine instance available"
                    )
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
            result = self.state_manager.set_exit_condition(False)
            if result.is_err:
                self.logger.warning("Failed to clear exit condition")
                # Continue anyway - this is not critical for startup

            #  CRITICAL: Actually start the burst engine main loop in a
            #  background thread
            import threading

            def run_burst_engine():
                """Background thread function to run the burst engine main
                loop."""
                try:
                    self.logger.debug(
                        "BRAIN SERVICE: Background thread starting, about to call burst_engine.run()"
                    )
                    self.logger.info(
                        "BRAIN SERVICE: Starting burst engine main loop in background thread"
                    )
                    burst_engine.run()
                    self.logger.debug(
                        "BRAIN SERVICE: burst_engine.run() returned"
                    )
                except Exception as e:
                    import traceback
                    self.logger.debug(
                        f"BRAIN SERVICE: Exception in burst engine main loop: {str(e)}"
                    )
                    self.logger.error(
                        f"BRAIN SERVICE: Error in burst engine main loop: {str(e)}"
                    )
                    self.logger.error(
                        f"BRAIN SERVICE: Full traceback: {traceback.format_exc()}"
                    )
                    # Set burst engine state to ERROR on exception
                    from feagi.core.state_manager import ServiceState

                    self.state_manager.set_burst_engine_state(
                        ServiceState.ERROR
                    )
                    nonlocal startup_success
                    startup_success = False

            self.logger.debug("BRAIN SERVICE: Creating background thread")

            # Start the burst engine in a daemon thread
            burst_thread = threading.Thread(
                target=run_burst_engine, daemon=True
            )
            burst_thread.start()

            self.logger.debug(
                "BRAIN SERVICE: Background thread started, using event-based synchronization..."
            )

            #  RTOS-COMPATIBLE: Event-based synchronization instead of sleep
            #  polling
            startup_event = threading.Event()
            startup_success = False

            def monitor_startup():
                """Monitor burst engine startup and signal completion."""
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
                        #  Use os.sched_yield() for RTOS compatibility if
                        #  available
                        try:
                            import os

                            if hasattr(os, "sched_yield"):
                                os.sched_yield()
                        except (ImportError, AttributeError):
                            pass  # No yield available - pure busy wait

                # Timeout reached
                startup_event.set()

            # Start monitoring thread
            monitor_thread = threading.Thread(
                target=monitor_startup, daemon=True
            )
            monitor_thread.start()

            # RTOS: Wait for event with timeout (deterministic)
            event_triggered = startup_event.wait(
                timeout=3.0
            )  # Max 3 second timeout

            self.logger.debug(
                f"BRAIN SERVICE: Event triggered: {event_triggered}, Success: {startup_success}"
            )

            # Verify startup success
            if startup_success and burst_engine._running:
                self.logger.debug(
                    "BRAIN SERVICE: Success! Burst engine is now running"
                )
                self.logger.info(
                    "Burst engine started successfully in background thread"
                )

                #  CRITICAL: If there's already a genome loaded, update the
                #  burst engine with it
                #  This ensures injection service gets initialized for existing
                #  genomes
                genome_loaded = (
                    self.state_manager
                    and self.state_manager.is_genome_loaded()
                )
                self.logger.debug(
                    f"BRAIN SERVICE: Checking genome status - loaded: {genome_loaded}"
                )
                # Debug-only file write
                try:
                    if self.state_manager.is_debug_npu_enabled():
                        import datetime
                        import os
                        import tempfile

                        log_path = os.path.join(
                            tempfile.gettempdir(),
                            "feagi_injection_debug--temp.log",
                        )
                        with open(log_path, "a") as f:
                            f.write(
                                f"{datetime.datetime.now()}: Brain service start - genome loaded: {genome_loaded}\n"
                            )
                except Exception:
                    pass

                if genome_loaded:
                    self.logger.debug(
                        "BRAIN SERVICE: Genome already loaded, calling update_with_genome()"
                    )
                    try:
                        if self.state_manager.is_debug_npu_enabled():
                            import datetime
                            import os
                            import tempfile

                            log_path = os.path.join(
                                tempfile.gettempdir(),
                                "feagi_injection_debug--temp.log",
                            )
                            with open(log_path, "a") as f:
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
                            if self.state_manager.is_debug_npu_enabled():
                                import datetime
                                import os
                                import tempfile

                                log_path = os.path.join(
                                    tempfile.gettempdir(),
                                    "feagi_injection_debug--temp.log",
                                )
                                with open(log_path, "a") as f:
                                    f.write(
                                        f"{datetime.datetime.now()}: Brain service error: {str(e)}\n"
                                    )
                        except Exception:
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
            self.logger.info(
                "[DEBUG] BRAIN SERVICE: Stopping burst engine main loop"
            )
            burst_engine.stop()

            # Set exit condition to stop the burst engine
            result = self.state_manager.set_exit_condition(True)
            if result.is_err:
                self.logger.warning("Failed to set exit condition")
                # Continue anyway - still try to stop the engine

            # Wait a moment for the thread to stop
            import time

            try:
                from feagi.config.toml_loader import (
                    get_timeout_config,
                    load_feagi_config,
                )

                cfg = load_feagi_config()
                to = get_timeout_config(cfg)
                delay = max(0.01, float(getattr(to, "thread_join", 0.2)))
            except Exception:
                delay = 0.2  # @architecture:acceptable - emergency fallback
            time.sleep(delay)  # Allow thread to stop

            # Verify it's stopped
            if not burst_engine._running:
                # Set burst engine state to UNAVAILABLE
                from feagi.core.state_manager import ServiceState

                self.state_manager.set_burst_engine_state(
                    ServiceState.UNAVAILABLE
                )
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

            self.logger.info(
                "Burst engine put on hold - neural processing paused"
            )
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
                            getattr(
                                self._connectome_manager, "cortical_areas", {}
                            )
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
            current_time = getattr(
                self._connectome_manager, "current_timestep", 0
            )

            # Count active neurons across all areas
            total_active = 0
            total_neurons = 0
            area_activity = {}

            for (
                area_idx,
                area,
            ) in self._connectome_manager.cortical_areas.items():
                area_neurons = self._connectome_manager.get_neurons_by_area(
                    area_idx
                )
                area_active = 0

                for neuron_id in area_neurons:
                    neuron_index = (
                        self._connectome_manager._neuron_id_to_index.get(
                            neuron_id
                        )
                    )
                    if neuron_index is None:
                        continue

                    last_fired = int(
                        self._connectome_manager.last_fired[neuron_index]
                    )
                    if (
                        last_fired > 0
                        and (current_time - last_fired) <= window
                    ):
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
                        "memory_usage_mb": process.memory_info().rss
                        / 1024
                        / 1024,
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
        """Stimulate specific neurons with given intensity (deterministic NPU path).

        Sets membrane_potential directly on the NPU-owned NeuronArray using
        id-based API. No legacy index mapping or duplicate SoA writes.
        """
        try:
            if not self._validate_genome_loaded():
                return {"success": False, "error": "No genome loaded"}

            # Deterministic single source of truth
            if not hasattr(self._connectome_manager, "neuron_array") or self._connectome_manager.neuron_array is None:
                return {"success": False, "error": "NPU neuron array not available"}

            neuron_array = self._connectome_manager.neuron_array
            if not hasattr(neuron_array, "set_neuron_property"):
                return {"success": False, "error": "NeuronArray.set_neuron_property not supported"}

            stimulated_count = 0
            failed_count = 0

            for neuron_id in neuron_ids:
                try:
                    neuron_array.set_neuron_property(
                        neuron_id, "membrane_potential", float(intensity)
                    )
                    stimulated_count += 1
                except Exception as e:
                    self.logger.warning(
                        f"Failed to stimulate neuron {neuron_id}: {str(e)}"
                    )
                    failed_count += 1

            try:
                from feagi.core.state_manager import FeagiStateManager
                if FeagiStateManager.instance().is_debug_npu_enabled():
                    self.logger.info(
                        f"[SENSORY-DEBUG] Stimulated {stimulated_count}/{len(neuron_ids)} neurons at MP={float(intensity)}"
                    )
            except Exception:
                pass

            return {
                "success": True,
                "stimulated_count": stimulated_count,
                "failed_count": failed_count,
                "total_requested": len(neuron_ids),
                "intensity": float(intensity),
            }
        except Exception as e:
            self.logger.error(f"Error stimulating neurons: {str(e)}")
            return {"success": False, "error": str(e)}

    def stimulate_neurons_unified(
        self, neural_data: Dict[str, Dict[str, np.ndarray]]
    ) -> Dict[str, Any]:
        """Unified neural stimulation using coordinate-based data format.

        SIMD-OPTIMIZED: Uses vectorized numpy operations instead of Python loops.

        Args:
            neural_data: Dictionary with cortical_area_id as keys and coordinate arrays as values:
                {
                    'cortical_area_1': {
                        'coordinates_x': np.array([1, 2, 3, ...], dtype=np.uint16),
                        'coordinates_y': np.array([4, 5, 6, ...], dtype=np.uint16),
                        'coordinates_z': np.array([7, 8, 9, ...], dtype=np.uint16),
                        'membrane_potentials': np.array([0.8, 1.2, 0.9, ...], dtype=np.float32),
                    }
                }

        Returns:
            Dictionary with stimulation results and statistics
        """
        try:
            total_stimulated = 0
            total_failed = 0
            area_results = {}

            # Process each cortical area
            for cortical_id, area_data in neural_data.items():
                try:
                    # Validate required fields
                    required_fields = [
                        "coordinates_x",
                        "coordinates_y",
                        "coordinates_z",
                        "membrane_potentials",
                    ]
                    missing_fields = [
                        field
                        for field in required_fields
                        if field not in area_data
                    ]
                    if missing_fields:
                        area_results[cortical_id] = {
                            "success": False,
                            "error": f"Missing required fields: {missing_fields}",
                        }
                        continue

                    # Extract coordinate arrays (already numpy arrays)
                    coords_x = area_data["coordinates_x"]
                    coords_y = area_data["coordinates_y"]
                    coords_z = area_data["coordinates_z"]
                    potentials = area_data["membrane_potentials"]
                    
                    # DEBUG: Log actual potential values to understand why neurons are firing
                    if len(potentials) > 0:
                        min_pot = float(np.min(potentials))
                        max_pot = float(np.max(potentials))
                        mean_pot = float(np.mean(potentials))
                        self.logger.info(f"[POTENTIAL-DEBUG] {cortical_id}: {len(potentials)} potentials - min={min_pot:.6f}, max={max_pot:.6f}, mean={mean_pot:.6f}, threshold=10000")
                        if max_pot >= 10000:
                            self.logger.info(f"[POTENTIAL-DEBUG] {cortical_id}: HIGH VALUES DETECTED! Max potential {max_pot:.6f} >= threshold 10000")

                    # Validate array lengths match
                    if not (
                        len(coords_x)
                        == len(coords_y)
                        == len(coords_z)
                        == len(potentials)
                    ):
                        area_results[cortical_id] = {
                            "success": False,
                            "error": "Coordinate and potential arrays must have same length",
                        }
                        continue

                    if len(coords_x) == 0:
                        area_results[cortical_id] = {
                            "success": True,
                            "stimulated_count": 0,
                            "failed_count": 0,
                            "unique_coordinates": 0,
                            "total_neurons_found": 0,
                        }
                        continue

                    # SIMD OPTIMIZATION 1: Vectorized coordinate processing
                    #  CRITICAL: Validate coordinate ranges before uint16
                    #  conversion to prevent silent data corruption
                    coords_x_array = np.asarray(coords_x)
                    coords_y_array = np.asarray(coords_y)
                    coords_z_array = np.asarray(coords_z)

                    #  Check for values that would be truncated by uint16
                    #  conversion
                    if (
                        len(coords_x_array) > 0
                        and coords_x_array.max() > 65535
                    ):
                        self.logger.error(
                            f"Area {cortical_id}: X coordinates exceed uint16 range! Max: {coords_x_array.max()}, limit: 65535"
                        )
                        area_results[cortical_id] = {
                            "success": False,
                            "error": f"X coordinates exceed uint16 range (max: {coords_x_array.max()})",
                        }
                        continue

                    if (
                        len(coords_y_array) > 0
                        and coords_y_array.max() > 65535
                    ):
                        self.logger.error(
                            f"Area {cortical_id}: Y coordinates exceed uint16 range! Max: {coords_y_array.max()}, limit: 65535"
                        )
                        area_results[cortical_id] = {
                            "success": False,
                            "error": f"Y coordinates exceed uint16 range (max: {coords_y_array.max()})",
                        }
                        continue

                    if (
                        len(coords_z_array) > 0
                        and coords_z_array.max() > 65535
                    ):
                        self.logger.error(
                            f"Area {cortical_id}: Z coordinates exceed uint16 range! Max: {coords_z_array.max()}, limit: 65535"
                        )
                        area_results[cortical_id] = {
                            "success": False,
                            "error": f"Z coordinates exceed uint16 range (max: {coords_z_array.max()})",
                        }
                        continue

                    # Convert to numpy arrays with validated uint16 conversion
                    coords_x = coords_x_array.astype(np.uint16)
                    coords_y = coords_y_array.astype(np.uint16)
                    coords_z = coords_z_array.astype(np.uint16)
                    potentials = np.asarray(potentials, dtype=np.float32)

                    # Strict path: no converter usage here. Proceed with SIMD/NPU batch mapping only.

                    # SIMD OPTIMIZATION 2: Vectorized unique coordinate finding
                    #  Stack coordinates and find unique positions in one
                    #  operation
                    coordinate_matrix = np.column_stack(
                        (coords_x, coords_y, coords_z)
                    )
                    unique_coords, inverse_indices = np.unique(
                        coordinate_matrix, axis=0, return_inverse=True
                    )

                    #  Convert to set for batch lookup (ConnectomeManager API
                    #  requirement)
                    candidate_positions = set(map(tuple, unique_coords))
                    
                    # Debug: Log sample coordinates being looked up
                    try:
                        from feagi.core.state_manager import FeagiStateManager
                        if FeagiStateManager.instance().is_debug_npu_enabled():
                            sample_coords = list(candidate_positions)[:5]
                            self.logger.info(f"[SENSORY-DEBUG] Looking up {len(candidate_positions)} positions for {cortical_id}: {sample_coords}")
                    except Exception:
                        pass

                    # SIMD-optimized batch lookup: coordinates → neuron_ids
                    #  This uses the existing batch_voxel_to_neuron_lookup
                    #  method
                    neuron_weight_pairs = (
                        self._connectome_manager.batch_voxel_to_neuron_lookup(
                            cortical_id=cortical_id,
                            candidate_positions=candidate_positions,
                            post_synaptic_current=1.0,  # Default weight
                        )
                    )

                    if not neuron_weight_pairs:
                        try:
                            from feagi.core.state_manager import FeagiStateManager
                            if FeagiStateManager.instance().is_debug_npu_enabled():
                                self.logger.info(
                                    f"[SENSORY-DEBUG] No neurons found for {cortical_id} via batch voxel lookup. Skipping FCL injection for this area."
                                )
                        except Exception:
                            pass
                        area_results[cortical_id] = {
                            "success": False,
                            "error": f"No neurons found at coordinates in area {cortical_id}",
                        }
                        continue

                    # Extract neuron IDs directly from batch lookup results
                    neuron_ids_array = np.array(
                        [nid for nid, _ in neuron_weight_pairs], dtype=np.int64
                    )
                    
                    # Accumulate activations for FCL injection (direct from batch lookup)
                    if "activations_for_area" not in locals():
                        activations_for_area = []
                    activations_for_area.extend([int(n) for n in neuron_ids_array.tolist()])
                    
                    area_stimulated = len(neuron_ids_array)
                    area_failed = 0

                    try:
                        from feagi.core.state_manager import FeagiStateManager
                        if FeagiStateManager.instance().is_debug_npu_enabled():
                            self.logger.info(
                                f"[SENSORY-DEBUG] Batch voxel lookup used for {cortical_id}: found={len(neuron_weight_pairs)}"
                            )
                    except Exception:
                        pass

                    # Initialize position to neurons mapping
                    position_to_neurons = {}
                    
                    # Get all neuron positions in batch (if available)
                    if hasattr(
                        self._connectome_manager, "batch_get_neuron_positions"
                    ):
                        # Use batch method if available
                        neuron_positions = self._connectome_manager.batch_get_neuron_positions(
                            neuron_ids_array
                        )
                        for i, neuron_id in enumerate(neuron_ids_array):
                            pos = neuron_positions[i]
                            if pos is not None:
                                pos_tuple = tuple(
                                    pos[:3]
                                )  # Take first 3 elements (x, y, z)
                                if pos_tuple not in position_to_neurons:
                                    position_to_neurons[pos_tuple] = []
                                position_to_neurons[pos_tuple].append(
                                    neuron_id
                                )
                    else:
                        #  Fallback to individual lookups (still better than
                        #  original loops)
                        for neuron_id, _ in neuron_weight_pairs:
                            neuron_pos = (
                                self._connectome_manager.get_neuron_position(
                                    neuron_id
                                )
                            )
                            if neuron_pos:
                                #  Convert from (area_id, x, y, z, idx) format
                                #  to (x, y, z)
                                if len(neuron_pos) >= 4:
                                    pos_tuple = (
                                        neuron_pos[1],
                                        neuron_pos[2],
                                        neuron_pos[3],
                                    )
                                else:
                                    pos_tuple = neuron_pos[:3]

                                if pos_tuple not in position_to_neurons:
                                    position_to_neurons[pos_tuple] = []
                                position_to_neurons[pos_tuple].append(
                                    neuron_id
                                )

                    # Deterministic: accumulate neuron IDs for FCL injection (list form)
                    if len(neuron_ids_array) > 0:
                        if "activations_for_area" not in locals():
                            activations_for_area = []
                        activations_for_area.extend([int(n) for n in neuron_ids_array.tolist()])

                    # area_stimulated and area_failed already set above from batch lookup

                    # activations_for_area already populated above from batch lookup

                    # Process each unique coordinate position
                    for unique_idx, unique_coord in enumerate(unique_coords):
                        coord_tuple = tuple(unique_coord)

                        #  Find all original indices that map to this unique
                        #  coordinate
                        coord_mask = inverse_indices == unique_idx
                        coord_potentials = potentials[coord_mask]

                        # Get neurons at this coordinate
                        neurons_at_coord = position_to_neurons.get(
                            coord_tuple, []
                        )

                        if neurons_at_coord and len(coord_potentials) > 0:
                            # Use the first potential value for this coordinate
                            #  (all coordinates at same position get same
                            #  stimulation)
                            potential_value = float(coord_potentials[0])

                            #  SIMD OPTIMIZATION 5: Batch membrane potential
                            #  update
                            try:
                                if hasattr(
                                    self._connectome_manager, "neuron_array"
                                ):
                                    neuron_array = (
                                        self._connectome_manager.neuron_array
                                    )
                                    if hasattr(
                                        neuron_array,
                                        "batch_update_membrane_potentials",
                                    ):
                                        # Deterministic id-based batch update
                                        neuron_array.batch_update_membrane_potentials(
                                            neurons_at_coord,
                                            [potential_value]
                                            * len(neurons_at_coord),
                                        )
                                        area_stimulated += len(
                                            neurons_at_coord
                                        )
                                    else:
                                        # Deterministic id-based individual updates
                                        for neuron_id in neurons_at_coord:
                                            try:
                                                neuron_array.set_neuron_property(
                                                    neuron_id,
                                                    "membrane_potential",
                                                    potential_value,
                                                )
                                                area_stimulated += 1
                                            except Exception as e:
                                                self.logger.warning(
                                                    f"Failed to stimulate neuron {neuron_id}: {str(e)}"
                                                )
                                                area_failed += 1

                                else:
                                    area_failed += len(neurons_at_coord)
                            except Exception as e:
                                self.logger.warning(
                                    f"Failed to stimulate neurons at {coord_tuple}: {str(e)}"
                                )
                                area_failed += len(neurons_at_coord)

                            # Accumulate ids for FCL injection
                            if neurons_at_coord:
                                activations_for_area.extend(neurons_at_coord)

                    area_results[cortical_id] = {
                        "success": True,
                        "stimulated_count": area_stimulated,
                        "failed_count": area_failed,
                        "unique_coordinates": len(unique_coords),
                        "total_neurons_found": len(neuron_ids_array),
                        "optimization_used": "simd_vectorized",
                    }

                    total_stimulated += area_stimulated
                    total_failed += area_failed

                    # Store per-area activations with potentials to inject later
                    if activations_for_area:
                        if "_pending_activations" not in locals():
                            _pending_activations = {}
                        # Build proper format with coordinates and potentials for injection service
                        _pending_activations[cortical_id] = {
                            "coordinates_x": coords_x,
                            "coordinates_y": coords_y, 
                            "coordinates_z": coords_z,
                            "membrane_potentials": potentials
                        }

                except Exception as e:
                    self.logger.error(
                        f"Error processing area {cortical_id}: {str(e)}"
                    )
                    area_results[cortical_id] = {
                        "success": False,
                        "error": str(e),
                    }
                    continue

            # After all areas processed, inject into FCL via injection service
            injected_count = 0
            try:
                if "_pending_activations" in locals() and _pending_activations:
                    burst_engine = self._get_burst_engine()
                    if burst_engine and hasattr(burst_engine, "injection_service") and burst_engine.injection_service:
                        current_timestep = getattr(
                            self._connectome_manager, "current_timestep", 0
                        )
                        try:
                            from feagi.core.state_manager import FeagiStateManager
                            if FeagiStateManager.instance().is_debug_npu_enabled():
                                # Summarize per-area counts for visibility
                                _area_counts = {aid: (len(v) if isinstance(v, (list, tuple)) else (len(v.get('coordinates_x', [])) if isinstance(v, dict) else 0)) for aid, v in _pending_activations.items()}
                                self.logger.info(
                                    f"[SENSORY-DEBUG] Injecting activations into FCL at t={current_timestep}: areas={list(_pending_activations.keys())} counts={_area_counts}"
                                )
                        except Exception:
                            pass

                        # Debug: Log what we're about to inject (first 10 per area)
                        try:
                            from feagi.core.state_manager import FeagiStateManager
                            if FeagiStateManager.instance().is_debug_npu_enabled():
                                debug_summary = {}
                                for area_id, neuron_list in _pending_activations.items():
                                    if isinstance(neuron_list, list) and len(neuron_list) > 0:
                                        first_10 = neuron_list[:10]
                                        total_count = len(neuron_list)
                                        if total_count > 10:
                                            debug_summary[area_id] = f"{first_10}... (+{total_count-10} more)"
                                        else:
                                            debug_summary[area_id] = first_10
                                    else:
                                        debug_summary[area_id] = neuron_list
                                self.logger.info(f"[SENSORY-DEBUG] About to inject: {debug_summary}")
                        except Exception:
                            pass
                        
                        # Route to BurstEngine buffer; processed via FCLInjector during the burst
                        if hasattr(burst_engine, '_pending_external_activations'):
                            burst_engine._pending_external_activations.update(_pending_activations)
                        else:
                            burst_engine._pending_external_activations = dict(_pending_activations)
                        injected_count = sum(
                            len(v.get('coordinates_x', [])) if isinstance(v, dict) else (len(v) if hasattr(v, '__len__') else 0)
                            for v in _pending_activations.values()
                        )
                        
                        # Debug: Log injection result
                        try:
                            from feagi.core.state_manager import FeagiStateManager
                            if FeagiStateManager.instance().is_debug_npu_enabled():
                                self.logger.info(f"[SENSORY-DEBUG] Injection service returned: {injected_count}")
                        except Exception:
                            pass
                    else:
                        try:
                            from feagi.core.state_manager import FeagiStateManager
                            if FeagiStateManager.instance().is_debug_npu_enabled():
                                self.logger.info(
                                    "[SENSORY-DEBUG] Skipping FCL injection: injection_service not available"
                                )
                        except Exception:
                            pass
            except Exception as inj_err:
                self.logger.error(f"Error injecting sensory activations: {inj_err}")

            return {
                "success": True,
                "total_stimulated": total_stimulated,
                "total_failed": total_failed,
                "areas_processed": len(neural_data),
                "area_results": area_results,
                "method": "unified_coordinate_based_simd_optimized",
                "injected_count": injected_count,
            }

        except Exception as e:
            self.logger.error(f"Error in unified neuron stimulation: {str(e)}")
            return {"success": False, "error": str(e)}

    def get_burst_engine_config(self) -> Dict[str, Any]:
        """Get burst engine configuration - RTOS-safe."""
        try:
            #  Get frequency directly from STATE MANAGER (single source of
            #  truth)
            state_frequency = 10.0  # Emergency fallback
            if self.state_manager:
                try:
                    state_freq = self.state_manager.get_burst_frequency()
                    if state_freq and state_freq > 0:
                        state_frequency = state_freq
                except Exception:
                    pass  # Use fallback

            # Build config from authoritative state manager values
            base_config = {
                "burst_frequency_hz": state_frequency,
                "burst_interval_seconds": 1.0 / state_frequency,
                "target_frequency_hz": state_frequency,
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

        except Exception as e:
            self.logger.error(f"Error getting burst engine config: {str(e)}")
            return {"error": str(e)}

    def get_burst_timer(self) -> float:
        """Get burst timer (stimulation period) from state manager
        (authoritative source)."""
        try:
            if self.state_manager:
                # Read from state_manager - the single source of truth
                frequency = self.state_manager.get_burst_frequency()
                if frequency and frequency > 0:
                    return 1.0 / frequency
                else:
                    # Log but don't raise - return safe default
                    self.logger.warning(f"Invalid burst frequency from state manager: {frequency}Hz - using default 0.1s")
                    return 0.1  # Default 10Hz = 0.1s period
            else:
                self.logger.warning("State manager not available - using default burst timer 0.1s")
                return 0.1
        except Exception as e:
            # Log error but return safe default instead of raising
            self.logger.error(f"Error getting burst timer: {str(e)} - using default 0.1s")
            return 0.1
