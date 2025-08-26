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
FEAGI v1 System API - Single Source of Truth

This module contains the ONLY definitions of system API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""

from typing import Any, Dict, List

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger

from .decorators import system_endpoint
from .schemas import (
    BrainVisualizationRequest,
    ConfigurationResponse,
    CorticalAreaTypesResponse,
    HealthCheckResponse,
    InfluxDBTestResponse,
    SuccessResponse,
    UserPreferencesRequest,
    UserPreferencesResponse,
    VersionsResponse,
    DebugLoggingRequest,
    DebugLoggingResponse,
)

logger = setup_logger(__name__)


class SystemAPI:
    """
    System API - Single Source of Truth for ALL Transports

    Each method in this class is decorated to automatically register
    the endpoint for FastAPI, ZMQ, and any future transport protocols.

    This ensures identical behavior across all transports with zero duplication.
    """

    def __init__(self, core_api_service: CoreAPIService):
        """Initialize with core API service dependency."""
        self.core_api_service = core_api_service

    # ===== User Preferences =====

    @system_endpoint(
        "GET", "/user_preferences", response_model=UserPreferencesResponse
    )
    def get_user_preferences(self) -> UserPreferencesResponse:
        """Get current user preferences."""
        try:
            prefs = self.core_api_service.get_user_preferences()
            return UserPreferencesResponse(
                adv_mode=prefs.get("adv_mode", False),
                ui_magnification=prefs.get("ui_magnification", 1.0),
                auto_pns_area_creation=prefs.get(
                    "auto_pns_area_creation", True
                ),
            )
        except Exception as e:
            logger.error(f"Error getting user preferences: {e}")
            raise ValueError(f"Failed to get user preferences: {str(e)}")

    @system_endpoint(
        "PUT",
        "/user_preferences",
        request_model=UserPreferencesRequest,
        response_model=SuccessResponse,
    )
    def update_user_preferences(
        self, request: UserPreferencesRequest
    ) -> SuccessResponse:
        """Update user preferences."""
        try:
            preferences = {
                "adv_mode": request.adv_mode,
                "ui_magnification": request.ui_magnification,
                "auto_pns_area_creation": request.auto_pns_area_creation,
            }
            success = self.core_api_service.update_user_preferences(
                preferences
            )
            if success:
                return SuccessResponse(
                    message="User preferences updated successfully"
                )
            else:
                raise ValueError("Failed to update user preferences")
        except Exception as e:
            logger.error(f"Error updating user preferences: {e}")
            raise ValueError(f"Failed to update user preferences: {str(e)}")

    # ===== System Information =====

    @system_endpoint("GET", "/versions", response_model=VersionsResponse)
    def get_versions(self) -> VersionsResponse:
        """Get system version information."""
        try:
            versions = self.core_api_service.get_versions()
            return VersionsResponse(
                feagi_core=versions.get("feagi_core", "unknown"),
                python=versions.get("python", "unknown"),
                timestamp=versions.get("timestamp", ""),
                numpy=versions.get("numpy"),
                torch=versions.get("torch"),
            )
        except Exception as e:
            logger.error(f"Error getting versions: {e}")
            raise ValueError(f"Failed to get versions: {str(e)}")

    @system_endpoint(
        "POST",
        "/debug_logging",
        request_model=DebugLoggingRequest,
        response_model=SuccessResponse,
    )
    def set_debug_logging(self, request: DebugLoggingRequest) -> SuccessResponse:
        """Enable/disable debug logging flags live.

        Keys mirror CLI switches; unspecified keys are left unchanged.
        """
        try:
            from feagi.core.state_manager import FeagiStateManager

            sm = FeagiStateManager.instance()
            # Read current flags via API to avoid internal key coupling
            cur_api = sm.is_debug_api_enabled()
            cur_npu = sm.is_debug_npu_enabled()
            cur_bdu = sm.is_debug_bdu_enabled()
            cur_zmq_in = sm.is_debug_zmq_inbound_enabled()
            cur_zmq_out = sm.is_debug_zmq_outbound_enabled()
            cur_mem = sm.is_mem_debug_enabled()

            # Compute new desired state (unspecified keys remain unchanged)
            new_api = cur_api if request.api is None else bool(request.api)
            new_npu = cur_npu if request.npu is None else bool(request.npu)
            new_bdu = cur_bdu if request.bdu is None else bool(request.bdu)
            new_zmq_in = cur_zmq_in if request.zmq_inbound is None else bool(request.zmq_inbound)
            new_zmq_out = cur_zmq_out if request.zmq_outbound is None else bool(request.zmq_outbound)
            new_mem = cur_mem if request.mem is None else bool(request.mem)

            # Update StateManager using its public mapping format
            sm.set_debug_config(
                {
                    "debug": {
                        "api": new_api,
                        "npu": new_npu,
                        "bdu": new_bdu,
                        "zmq_inbound": new_zmq_in,
                        "zmq_outbound": new_zmq_out,
                        "mem_debug": new_mem,
                    }
                }
            )

            # Apply subsystem levels live (logger + handler thresholds)
            from feagi.utils.logger import apply_subsystem_log_levels
            import logging
            # Build a unified debug_cfg snapshot
            debug_cfg = {
                "debug_api": new_api,
                "debug_npu": new_npu,
                "debug_bdu": new_bdu,
                "debug_zmq_inbound": new_zmq_in,
                "debug_zmq_outbound": new_zmq_out,
                "mem_debug": new_mem,
            }
            # Baseline comes from FEAGI_CLI_LOG_LEVEL or defaults to INFO
            baseline = os.environ.get("FEAGI_CLI_LOG_LEVEL", "INFO")
            baseline_level = getattr(logging, baseline.upper(), logging.INFO)
            apply_subsystem_log_levels(debug_cfg, baseline_level)

            # Keep module-specific env overrides in sync for any new loggers
            import os
            os.environ["FEAGI_DEBUG_API"] = "1" if new_api else "0"
            os.environ["FEAGI_DEBUG_NPU"] = "1" if new_npu else "0"
            os.environ["FEAGI_DEBUG_BDU"] = "1" if new_bdu else "0"
            os.environ["FEAGI_DEBUG_ZMQ"] = "1" if (new_zmq_in or new_zmq_out) else "0"
            os.environ["FEAGI_DEBUG_MEM"] = "1" if new_mem else "0"

            return SuccessResponse(message="Debug logging flags updated")
        except Exception as e:
            logger.error(f"Error updating debug logging flags: {e}")
            raise ValueError(f"Failed to update debug logging flags: {str(e)}")

    @system_endpoint(
        "GET",
        "/debug_logging",
        response_model=DebugLoggingResponse,
    )
    def get_debug_logging(self) -> DebugLoggingResponse:
        """Get current debug logging flags from the state manager."""
        try:
            from feagi.core.state_manager import FeagiStateManager

            sm = FeagiStateManager.instance()
            cfg = getattr(sm, "_debug_config", {}) or {}
            return DebugLoggingResponse(
                api=bool(cfg.get("debug_api", False)),
                npu=bool(cfg.get("debug_npu", False)),
                bdu=bool(cfg.get("debug_bdu", False)),
                zmq_inbound=bool(cfg.get("debug_zmq_inbound", False)),
                zmq_outbound=bool(cfg.get("debug_zmq_outbound", False)),
                mem=bool(cfg.get("mem_debug", False)),
            )
        except Exception as e:
            logger.error(f"Error retrieving debug logging flags: {e}")
            raise ValueError(f"Failed to get debug logging flags: {str(e)}")

    @system_endpoint(
        "GET", "/health_check", response_model=HealthCheckResponse
    )
    async def get_health_check(self) -> HealthCheckResponse:
        """Get comprehensive system health information."""
        try:
            health = await self.core_api_service.get_system_health()
            return HealthCheckResponse(
                burst_engine=health.get("burst_engine", False),
                connected_agents=health.get("connected_agents"),
                influxdb_availability=health.get(
                    "influxdb_availability", False
                ),
                neuron_count_max=health.get("neuron_count_max", 0),
                synapse_count_max=health.get("synapse_count_max", 0),
                latest_changes_saved_externally=health.get(
                    "latest_changes_saved_externally", False
                ),
                genome_availability=health.get("genome_availability", False),
                genome_validity=health.get("genome_validity"),
                brain_readiness=health.get("brain_readiness", False),
                fitness=health.get("fitness"),
                cortical_area_count=health.get("cortical_area_count"),
                neuron_count=health.get(
                    "neuron_count"
                ),  # Total neurons (regular + memory)
                memory_neuron_count=health.get(
                    "memory_neuron_count"
                ),  # Memory neurons only
                regular_neuron_count=health.get(
                    "regular_neuron_count"
                ),  # Regular neurons only
                synapse_count=health.get("synapse_count"),
                estimated_brain_size_in_MB=health.get(
                    "estimated_brain_size_in_MB"
                ),
                genome_num=health.get("genome_num"),
                genome_timestamp=health.get("genome_timestamp"),
                simulation_timestep=health.get("simulation_timestep"),
            )
        except Exception as e:
            logger.error(f"Error getting system health: {e}")
            raise ValueError(f"Failed to get system health: {str(e)}")

    @system_endpoint(
        "GET", "/configuration", response_model=ConfigurationResponse
    )
    def get_configuration(self) -> ConfigurationResponse:
        """Get system configuration."""
        try:
            config = self.core_api_service.get_configuration()
            return ConfigurationResponse(config=config)
        except Exception as e:
            logger.error(f"Error getting configuration: {e}")
            raise ValueError(f"Failed to get configuration: {str(e)}")

    # ===== External Services =====

    @system_endpoint(
        "GET", "/db/influxdb/test", response_model=InfluxDBTestResponse
    )
    def test_influxdb(self) -> InfluxDBTestResponse:
        """Test InfluxDB connection."""
        try:
            influx_status = self.core_api_service.test_influxdb()
            if influx_status:
                return InfluxDBTestResponse(
                    status=influx_status.get("status", "unknown"),
                    database=influx_status.get("database", ""),
                    host=influx_status.get("host", ""),
                    port=influx_status.get("port", 0),
                )
            else:
                raise ValueError("InfluxDB service not available")
        except Exception as e:
            logger.error(f"Error testing InfluxDB: {e}")
            raise ValueError(f"InfluxDB test failed: {str(e)}")

    # ===== System Configuration =====

    @system_endpoint(
        "POST", "/circuit_library_path", response_model=SuccessResponse
    )
    def set_circuit_library_path(self, path: str) -> SuccessResponse:
        """Set the circuit library path."""
        try:
            success = self.core_api_service.set_circuit_library_path(path)
            if success:
                return SuccessResponse(
                    message=f"{path} is the new circuit library path."
                )
            else:
                raise ValueError("Failed to set circuit library path")
        except Exception as e:
            logger.error(f"Error setting circuit library path: {e}")
            raise ValueError(f"Failed to set circuit library path: {str(e)}")

    @system_endpoint(
        "GET", "/cortical_area_types", response_model=CorticalAreaTypesResponse
    )
    def get_cortical_area_types(self) -> CorticalAreaTypesResponse:
        """Get available cortical area types."""
        try:
            types = self.core_api_service.get_cortical_area_types()
            return CorticalAreaTypesResponse(types=types)
        except Exception as e:
            logger.error(f"Error getting cortical area types: {e}")
            raise ValueError(f"Failed to get cortical area types: {str(e)}")

    # ===== System Control =====

    @system_endpoint("POST", "/fcl_reset", response_model=SuccessResponse)
    def reset_fcl(self) -> SuccessResponse:
        """Reset the Fire Candidate List."""
        try:
            success = self.core_api_service.reset_fcl()
            if success:
                return SuccessResponse(
                    message="Fire Candidate List reset successfully"
                )
            else:
                raise ValueError("Failed to reset FCL")
        except Exception as e:
            logger.error(f"Error resetting FCL: {e}")
            raise ValueError(f"Failed to reset FCL: {str(e)}")

    # ===== Visualization Configuration =====

    @system_endpoint("GET", "/cortical_area_visualization_skip_rate")
    def get_cortical_area_visualization_skip_rate(self) -> int:
        """Get cortical area visualization skip rate (returns int directly for
        legacy compatibility)."""
        try:
            skip_rate = self.core_api_service.get_visualization_skip_rate()
            return skip_rate
        except Exception as e:
            logger.error(f"Error getting visualization skip rate: {e}")
            raise ValueError(
                f"Failed to get visualization skip rate: {str(e)}"
            )

    @system_endpoint(
        "GET", "/cortical_area_visualization_suppression_threshold"
    )
    def get_cortical_area_visualization_suppression_threshold(self) -> int:
        """Get cortical area visualization suppression threshold (returns int
        directly for legacy compatibility)."""
        try:
            threshold = (
                self.core_api_service.get_visualization_suppression_threshold()
            )
            return threshold
        except Exception as e:
            logger.error(
                f"Error getting visualization suppression threshold: {e}"
            )
            raise ValueError(
                f"Failed to get visualization suppression threshold: {str(e)}"
            )

    @system_endpoint("GET", "/global_activity_visualization")
    def get_global_activity_visualization_info(self) -> Dict[str, Any]:
        """Get global activity visualization settings."""
        try:
            settings = (
                self.core_api_service.get_global_activity_visualization()
            )
            return settings
        except Exception as e:
            logger.error(f"Error getting global activity visualization: {e}")
            raise ValueError(
                f"Failed to get global activity visualization: {str(e)}"
            )

    @system_endpoint("GET", "/global_activity_visualization")
    def get_global_activity_visualization_status(self) -> bool:
        """Get global activity visualization status (returns bool directly for
        legacy compatibility)."""
        try:
            enabled = self.core_api_service.get_global_activity_visualization()
            return enabled
        except Exception as e:
            logger.error(f"Error getting global activity visualization: {e}")
            raise ValueError(
                f"Failed to get global activity visualization: {str(e)}"
            )

    @system_endpoint(
        "PUT",
        "/global_activity_visualization",
        request_model=BrainVisualizationRequest,
        response_model=SuccessResponse,
    )
    def set_global_activity_visualization(
        self, request: BrainVisualizationRequest
    ) -> SuccessResponse:
        """Set global activity visualization settings."""
        try:
            success = self.core_api_service.set_global_activity_visualization(
                request.enabled
            )
            if not success:
                raise ValueError("Failed to set global activity visualization")

            return SuccessResponse(
                message="Global activity visualization updated successfully"
            )
        except Exception as e:
            logger.error(f"Error setting global activity visualization: {e}")
            raise ValueError(
                f"Failed to set global activity visualization: {str(e)}"
            )

    @system_endpoint("GET", "/unique_logs")
    def get_unique_logs_list(self) -> List[str]:
        """Get unique log entries."""
        try:
            logs = self.core_api_service.get_unique_logs()
            return logs
        except Exception as e:
            logger.error(f"Error getting unique logs: {e}")
            raise ValueError(f"Failed to get unique logs: {str(e)}")

    @system_endpoint("GET", "/unique_logs")
    def get_unique_logs_dict(self) -> Dict[str, Any]:
        """Get unique log entries (returns dict format for legacy
        compatibility)."""
        try:
            logs = self.core_api_service.get_unique_logs()
            # Convert list to dict format for legacy compatibility
            return {"logs": logs}
        except Exception as e:
            logger.error(f"Error getting unique logs: {e}")
            raise ValueError(f"Failed to get unique logs: {str(e)}")

    @system_endpoint("GET", "/unique_logs")
    def get_unique_logs_legacy(self) -> Dict[str, Any]:
        """Get unique log entries (returns legacy format
        {"PNS":[],"CNS":[]})."""
        try:
            # Return the exact legacy format
            return {"PNS": [], "CNS": []}
        except Exception as e:
            logger.error(f"Error getting unique logs: {e}")
            raise ValueError(f"Failed to get unique logs: {str(e)}")

    # ===== Legacy/Placeholder Endpoints =====

    @system_endpoint("POST", "/register", response_model=SuccessResponse)
    def register_system(
        self, registration_data: Dict[str, Any]
    ) -> SuccessResponse:
        """System registration (placeholder implementation)."""
        logger.warning("System registration endpoint is not implemented")
        return SuccessResponse(
            message="Warning! This endpoint is not doing anything at this time!"
        )

    @system_endpoint("POST", "/logs", response_model=SuccessResponse)
    def manage_logs(self, log_data: Dict[str, Any]) -> SuccessResponse:
        """Manage system logs."""
        try:
            if not hasattr(
                self.core_api_service._connectome_manager, "api_message_queue"
            ):
                raise ValueError("API message queue not initialized")

            api_message = {"log_management": log_data}
            self.core_api_service._connectome_manager.api_message_queue.put(
                item=api_message
            )
            return SuccessResponse(message="Log management request processed")
        except Exception as e:
            logger.error(f"Error managing logs: {e}")
            raise ValueError(f"Failed to manage logs: {str(e)}")

    @system_endpoint("GET", "/beacon/subscribers")
    def get_beacon_subscribers(self) -> List[str]:
        """Get current beacon subscribers."""
        try:
            state = self.core_api_service.get_state_manager()
            if state and getattr(state, "beacon_sub", None):
                return list(state.beacon_sub)
            else:
                raise ValueError("No subscribers found")
        except Exception as e:
            logger.error(f"Error getting beacon subscribers: {e}")
            raise ValueError(f"Failed to get beacon subscribers: {str(e)}")

    @system_endpoint(
        "POST", "/beacon/subscribe", response_model=SuccessResponse
    )
    def subscribe_to_beacon(self, subscriber_address: str) -> SuccessResponse:
        """Subscribe to beacon notifications."""
        try:
            if not hasattr(
                self.core_api_service._connectome_manager, "api_message_queue"
            ):
                raise ValueError("API message queue not initialized")

            message_dict = {"beacon_sub": subscriber_address}
            self.core_api_service._connectome_manager.api_message_queue.put(
                item=message_dict
            )
            return SuccessResponse(
                message="Subscribed to beacon notifications"
            )
        except Exception as e:
            logger.error(f"Error subscribing to beacon: {e}")
            raise ValueError(f"Failed to subscribe to beacon: {str(e)}")

    @system_endpoint(
        "DELETE", "/beacon/unsubscribe", response_model=SuccessResponse
    )
    def unsubscribe_from_beacon(
        self, subscriber_address: str
    ) -> SuccessResponse:
        """Unsubscribe from beacon notifications."""
        try:
            if not hasattr(
                self.core_api_service._connectome_manager, "api_message_queue"
            ):
                raise ValueError("API message queue not initialized")

            message_dict = {"beacon_unsub": subscriber_address}
            self.core_api_service._connectome_manager.api_message_queue.put(
                item=message_dict
            )
            return SuccessResponse(
                message="Unsubscribed from beacon notifications"
            )
        except Exception as e:
            logger.error(f"Error unsubscribing from beacon: {e}")
            raise ValueError(f"Failed to unsubscribe from beacon: {str(e)}")

    # ===== Legacy Version Endpoint =====

    @system_endpoint("GET", "/version")
    def get_version(self) -> Dict[str, str]:
        """Get FEAGI version (legacy endpoint)."""
        try:
            versions = self.get_versions()
            return {"version": versions.feagi_core}
        except Exception as e:
            logger.error(f"Error getting version: {e}")
            raise ValueError(f"Failed to get version: {str(e)}")

    # ===== FQ Sampler Control =====

    @system_endpoint(
        "POST",
        "/enable_visualization_fq_sampler",
        response_model=SuccessResponse,
    )
    def enable_visualization_fq_sampler(self) -> SuccessResponse:
        """Enable the visualization FQ sampler for brain visualizer
        connectivity."""
        try:
            success = self.core_api_service.enable_visualization_fq_sampler()
            if success:
                return SuccessResponse(
                    message="Visualization FQ sampler enabled successfully"
                )
            else:
                raise ValueError("Failed to enable visualization FQ sampler")
        except Exception as e:
            logger.error(f"Error enabling visualization FQ sampler: {e}")
            raise ValueError(
                f"Failed to enable visualization FQ sampler: {str(e)}"
            )

    @system_endpoint(
        "POST",
        "/disable_visualization_fq_sampler",
        response_model=SuccessResponse,
    )
    def disable_visualization_fq_sampler(self) -> SuccessResponse:
        """Disable the visualization FQ sampler."""
        try:
            success = self.core_api_service.disable_visualization_fq_sampler()
            if success:
                return SuccessResponse(
                    message="Visualization FQ sampler disabled successfully"
                )
            else:
                raise ValueError("Failed to disable visualization FQ sampler")
        except Exception as e:
            logger.error(f"Error disabling visualization FQ sampler: {e}")
            raise ValueError(
                f"Failed to disable visualization FQ sampler: {str(e)}"
            )

    @system_endpoint("GET", "/fq_sampler_status")
    def get_fq_sampler_status(self) -> Dict[str, Any]:
        """Get the current status of all FQ samplers."""
        try:
            return self.core_api_service.get_fq_sampler_status()
        except Exception as e:
            logger.error(f"Error getting FQ sampler status: {e}")
            raise ValueError(f"Failed to get FQ sampler status: {str(e)}")

    # ===== Diagnostics: FCL and Process Manager =====

    @system_endpoint("GET", "/fcl_status")
    def get_fcl_status(self) -> Dict[str, Any]:
        """Return detailed status of the FCL manager (queues/windows by
        cortical)."""
        try:
            fclm = self.core_api_service.get_fcl_manager()
            if not fclm:
                return {
                    "available": False,
                    "error": "FCL manager not available",
                }

            # Basic stats
            status: Dict[str, Any] = {
                "available": True,
                "current_timestep": getattr(fclm, "current_timestep", 0),
                "default_window_size": getattr(fclm, "default_window_size", 0),
                "dynamic_window_sizing": getattr(
                    fclm, "_dynamic_sizing_enabled", False
                ),
                "total_neurons_fired": getattr(fclm, "total_neurons_fired", 0),
            }

            # Active corticals and per-cortical window size (ids as strings)
            active_idx = []
            try:
                active_idx = list(
                    getattr(fclm, "cortical_fcl_history", {}).keys()
                )
            except Exception:
                active_idx = []

            # Resolve cortical_id from idx where possible
            from feagi.bdu.connectome_manager import ConnectomeManager

            cm = ConnectomeManager.instance()
            per_cortical = {}
            for idx in active_idx:
                try:
                    cid = cm.get_cortical_id_for_idx(int(idx)) or str(idx)
                except Exception:
                    cid = str(idx)
                try:
                    w = fclm.get_cortical_window_size(int(idx))
                except Exception:
                    w = status["default_window_size"]
                per_cortical[str(cid)] = {"window_size": int(w)}

            status["corticals"] = per_cortical

            # Memory corticals
            try:
                mem_idxs = list(
                    getattr(fclm, "memory_cortical_indices", set())
                )
                mem_ids = []
                for midx in mem_idxs:
                    try:
                        mem_ids.append(
                            cm.get_cortical_id_for_idx(int(midx)) or str(midx)
                        )
                    except Exception:
                        mem_ids.append(str(midx))
                status["memory_corticals"] = mem_ids
            except Exception:
                status["memory_corticals"] = []

            # Current global FCL occupancy (count only to keep it light)
            try:
                gi = getattr(fclm, "current_window_index", 0)
                ghist = getattr(fclm, "global_fcl_history", [])
                current_global = ghist[gi] if gi < len(ghist) else None
                status["current_fcl_count"] = (
                    int(len(current_global)) if current_global else 0
                )
                status["window_index"] = int(gi)
            except Exception:
                status["current_fcl_count"] = 0

            # Transient recent counts (small N) for quick trend view
            try:
                last_n = 3
                ghist = getattr(fclm, "global_fcl_history", [])
                gi = getattr(fclm, "current_window_index", 0)
                recent = []
                if ghist:
                    for off in range(last_n - 1, -1, -1):
                        idx = gi - off
                        if idx >= 0 and idx < len(ghist):
                            recent.append(int(len(ghist[idx])))
                status["recent_global_counts"] = recent
            except Exception:
                status["recent_global_counts"] = []

            #  Top-K corticals by current activity with recent counts
            #  (transient)
            try:
                top_k = 8
                gi = getattr(fclm, "current_window_index", 0)
                chist = getattr(fclm, "cortical_fcl_history", {})
                # Build list of (cid, current_count)
                current_counts = []
                for cidx, hist in chist.items():
                    try:
                        cur = int(len(hist[gi])) if gi < len(hist) else 0
                    except Exception:
                        cur = 0
                    # Resolve id
                    try:
                        cid = cm.get_cortical_id_for_idx(int(cidx)) or str(
                            cidx
                        )
                    except Exception:
                        cid = str(cidx)
                    current_counts.append((str(cid), cur))
                # Sort and pick top_k
                current_counts.sort(key=lambda t: t[1], reverse=True)
                top = current_counts[:top_k]
                recent_per_cortical: Dict[str, List[int]] = {}
                for cid, _cur in top:
                    try:
                        # Recover index from id
                        # Use mapping: id -> idx
                        idx = cm.cortical_mapping.get_idx(cid)  # type: ignore[attr-defined]
                        hist = chist.get(idx, [])
                        series: List[int] = []
                        for off in range(2, -1, -1):
                            pos = gi - off
                            if (
                                isinstance(hist, list)
                                and pos >= 0
                                and pos < len(hist)
                            ):
                                series.append(int(len(hist[pos])))
                        recent_per_cortical[str(cid)] = series
                    except Exception:
                        recent_per_cortical[str(cid)] = []
                status["top_corticals_recent_counts"] = recent_per_cortical
            except Exception:
                status["top_corticals_recent_counts"] = {}

            return status
        except Exception as e:
            logger.error(f"Error getting FCL status: {e}")
            raise ValueError(f"Failed to get FCL status: {str(e)}")

    @system_endpoint("GET", "/processes")
    def get_active_processes(self) -> Dict[str, Any]:
        """Return details about active FEAGI processes/tasks managed by the
        ProcessManager."""
        try:
            from feagi.process_manager import get_process_manager

            pm = get_process_manager()
            if not pm:
                return {
                    "available": False,
                    "error": "Process manager not available",
                }

            procs: Dict[str, Any] = {}
            processes = getattr(pm, "_processes", {})
            for name, svc in processes.items():
                entry: Dict[str, Any] = {"name": name}
                try:
                    if hasattr(svc, "is_running") and callable(svc.is_running):
                        entry["type"] = "service"
                        entry["running"] = bool(svc.is_running())
                    elif hasattr(svc, "is_alive") and callable(svc.is_alive):
                        entry["type"] = "thread"
                        entry["running"] = bool(svc.is_alive())
                    elif hasattr(svc, "poll") and callable(svc.poll):
                        entry["type"] = "process"
                        entry["exit_code"] = svc.poll()
                        entry["running"] = svc.poll() is None
                    else:
                        entry["type"] = "unknown"
                except Exception:
                    entry["type"] = entry.get("type", "unknown")

                # Optional: quick psutil metrics for processes with pid
                try:
                    import psutil  # noqa: F401

                    pid = getattr(svc, "pid", None)
                    if isinstance(pid, int) and pid > 0:
                        try:
                            p = psutil.Process(pid)
                            mi = p.memory_info()
                            entry["rss_mb"] = round(mi.rss / (1024 * 1024), 2)
                            # Non-blocking CPU snapshot
                            entry["cpu_percent"] = p.cpu_percent(interval=0.0)
                        except Exception:
                            pass
                except Exception:
                    pass

                procs[str(name)] = entry

            # Include sampler presence flags and performance stats (transient)
            summary: Dict[str, Any] = {}
            try:
                viz = getattr(pm, "_viz_fq_sampler", None)
                mot = getattr(pm, "_motor_fq_sampler", None)
                summary["viz_sampler_present"] = bool(viz)
                summary["motor_sampler_present"] = bool(mot)

                # Lightweight stats if method exists
                def _safe_stats(obj):
                    try:
                        fn = getattr(obj, "get_performance_stats", None)
                        if callable(fn):
                            return fn() or {}
                    except Exception:
                        return {}
                    return {}

                summary["viz_stats"] = _safe_stats(viz)
                summary["motor_stats"] = _safe_stats(mot)
            except Exception:
                pass

            # Burst engine status snapshot (transient)
            try:
                be = self.core_api_service.get_burst_engine_status()
                summary["burst_engine"] = {
                    "status": be.get("status", "unknown"),
                    "is_running": be.get("is_running", False),
                }
            except Exception:
                summary["burst_engine"] = {
                    "status": "unknown",
                    "is_running": False,
                }

            return {"available": True, "processes": procs, "summary": summary}
        except Exception as e:
            logger.error(f"Error getting processes: {e}")
            raise ValueError(f"Failed to get processes: {str(e)}")


# ===== Factory Function =====


def create_system_api(core_api_service: CoreAPIService) -> SystemAPI:
    """Factory function to create a SystemAPI instance.

    This function can be used by transport adapters to get a configured
    SystemAPI instance with the required dependencies.
    """
    return SystemAPI(core_api_service)
