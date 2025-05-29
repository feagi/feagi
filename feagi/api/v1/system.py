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
FEAGI v1 System API - Single Source of Truth

This module contains the ONLY definitions of system API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""

from typing import Dict, Any, Optional, List
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger
from .schemas import (
    UserPreferencesResponse, UserPreferencesRequest,
    VersionsResponse, HealthCheckResponse, ConfigurationResponse,
    InfluxDBTestResponse, CorticalAreaTypesResponse,
    SuccessResponse, ErrorResponse,
    VisualizationSkipRateRequest, VisualizationThresholdRequest,
    BrainVisualizationRequest
)
from .decorators import system_endpoint

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
    
    @system_endpoint('GET', '/user_preferences', response_model=UserPreferencesResponse)
    def get_user_preferences(self) -> UserPreferencesResponse:
        """Get current user preferences."""
        try:
            prefs = self.core_api_service.get_user_preferences()
            return UserPreferencesResponse(
                adv_mode=prefs.get('adv_mode', False),
                ui_magnification=prefs.get('ui_magnification', 1.0),
                auto_pns_area_creation=prefs.get('auto_pns_area_creation', True)
            )
        except Exception as e:
            logger.error(f"Error getting user preferences: {e}")
            raise ValueError(f"Failed to get user preferences: {str(e)}")
    
    @system_endpoint('PUT', '/user_preferences', 
                    request_model=UserPreferencesRequest, 
                    response_model=SuccessResponse)
    def update_user_preferences(self, request: UserPreferencesRequest) -> SuccessResponse:
        """Update user preferences."""
        try:
            preferences = {
                "adv_mode": request.adv_mode,
                "ui_magnification": request.ui_magnification,
                "auto_pns_area_creation": request.auto_pns_area_creation
            }
            success = self.core_api_service.update_user_preferences(preferences)
            if success:
                return SuccessResponse(message="User preferences updated successfully")
            else:
                raise ValueError("Failed to update user preferences")
        except Exception as e:
            logger.error(f"Error updating user preferences: {e}")
            raise ValueError(f"Failed to update user preferences: {str(e)}")
    
    # ===== System Information =====
    
    @system_endpoint('GET', '/versions', response_model=VersionsResponse)
    def get_versions(self) -> VersionsResponse:
        """Get system version information."""
        try:
            versions = self.core_api_service.get_versions()
            return VersionsResponse(
                feagi_core=versions.get('feagi_core', 'unknown'),
                python=versions.get('python', 'unknown'),
                timestamp=versions.get('timestamp', ''),
                numpy=versions.get('numpy'),
                torch=versions.get('torch')
            )
        except Exception as e:
            logger.error(f"Error getting versions: {e}")
            raise ValueError(f"Failed to get versions: {str(e)}")
    
    @system_endpoint('GET', '/health_check', response_model=HealthCheckResponse)
    async def get_health_check(self) -> HealthCheckResponse:
        """Get comprehensive system health information."""
        try:
            health = await self.core_api_service.get_system_health()
            return HealthCheckResponse(
                burst_engine=health.get('burst_engine', False),
                connected_agents=health.get('connected_agents'),
                influxdb_availability=health.get('influxdb_availability', False),
                neuron_count_max=health.get('neuron_count_max', 0),
                synapse_count_max=health.get('synapse_count_max', 0),
                latest_changes_saved_externally=health.get('latest_changes_saved_externally', False),
                genome_availability=health.get('genome_availability', False),
                genome_validity=health.get('genome_validity'),
                brain_readiness=health.get('brain_readiness', False),
                fitness=health.get('fitness'),
                cortical_area_count=health.get('cortical_area_count'),
                neuron_count=health.get('neuron_count'),
                synapse_count=health.get('synapse_count'),
                estimated_brain_size_in_MB=health.get('estimated_brain_size_in_MB'),
                genome_num=health.get('genome_num'),
                genome_timestamp=health.get('genome_timestamp')
            )
        except Exception as e:
            logger.error(f"Error getting system health: {e}")
            raise ValueError(f"Failed to get system health: {str(e)}")
    
    @system_endpoint('GET', '/configuration', response_model=ConfigurationResponse)
    def get_configuration(self) -> ConfigurationResponse:
        """Get system configuration."""
        try:
            config = self.core_api_service.get_configuration()
            return ConfigurationResponse(config=config)
        except Exception as e:
            logger.error(f"Error getting configuration: {e}")
            raise ValueError(f"Failed to get configuration: {str(e)}")
    
    # ===== External Services =====
    
    @system_endpoint('GET', '/db/influxdb/test', response_model=InfluxDBTestResponse)
    def test_influxdb(self) -> InfluxDBTestResponse:
        """Test InfluxDB connection."""
        try:
            influx_status = self.core_api_service.test_influxdb()
            if influx_status:
                return InfluxDBTestResponse(
                    status=influx_status.get('status', 'unknown'),
                    database=influx_status.get('database', ''),
                    host=influx_status.get('host', ''),
                    port=influx_status.get('port', 0)
                )
            else:
                raise ValueError("InfluxDB service not available")
        except Exception as e:
            logger.error(f"Error testing InfluxDB: {e}")
            raise ValueError(f"InfluxDB test failed: {str(e)}")
    
    # ===== System Configuration =====
    
    @system_endpoint('POST', '/circuit_library_path', response_model=SuccessResponse)
    def set_circuit_library_path(self, path: str) -> SuccessResponse:
        """Set the circuit library path."""
        try:
            success = self.core_api_service.set_circuit_library_path(path)
            if success:
                return SuccessResponse(message=f"{path} is the new circuit library path.")
            else:
                raise ValueError("Failed to set circuit library path")
        except Exception as e:
            logger.error(f"Error setting circuit library path: {e}")
            raise ValueError(f"Failed to set circuit library path: {str(e)}")
    
    @system_endpoint('GET', '/cortical_area_types', response_model=CorticalAreaTypesResponse)
    def get_cortical_area_types(self) -> CorticalAreaTypesResponse:
        """Get available cortical area types."""
        try:
            types = self.core_api_service.get_cortical_area_types()
            return CorticalAreaTypesResponse(types=types)
        except Exception as e:
            logger.error(f"Error getting cortical area types: {e}")
            raise ValueError(f"Failed to get cortical area types: {str(e)}")
    
    # ===== System Control =====
    
    @system_endpoint('POST', '/fcl_reset', response_model=SuccessResponse)
    def reset_fcl(self) -> SuccessResponse:
        """Reset the Fire Candidate List."""
        try:
            success = self.core_api_service.reset_fcl()
            if success:
                return SuccessResponse(message="Fire Candidate List reset successfully")
            else:
                raise ValueError("Failed to reset FCL")
        except Exception as e:
            logger.error(f"Error resetting FCL: {e}")
            raise ValueError(f"Failed to reset FCL: {str(e)}")
    
    # ===== Visualization Configuration =====
    
    @system_endpoint('GET', '/cortical_area_visualization_skip_rate')
    def get_cortical_area_visualization_skip_rate(self) -> int:
        """Get cortical area visualization skip rate (returns int directly for legacy compatibility)."""
        try:
            skip_rate = self.core_api_service.get_visualization_skip_rate()
            return skip_rate
        except Exception as e:
            logger.error(f"Error getting visualization skip rate: {e}")
            raise ValueError(f"Failed to get visualization skip rate: {str(e)}")
    
    @system_endpoint('GET', '/cortical_area_visualization_suppression_threshold')
    def get_cortical_area_visualization_suppression_threshold(self) -> int:
        """Get cortical area visualization suppression threshold (returns int directly for legacy compatibility)."""
        try:
            threshold = self.core_api_service.get_visualization_suppression_threshold()
            return threshold
        except Exception as e:
            logger.error(f"Error getting visualization suppression threshold: {e}")
            raise ValueError(f"Failed to get visualization suppression threshold: {str(e)}")
    
    @system_endpoint('GET', '/global_activity_visualization')
    def get_global_activity_visualization(self) -> Dict[str, Any]:
        """Get global activity visualization settings."""
        try:
            settings = self.core_api_service.get_global_activity_visualization()
            return settings
        except Exception as e:
            logger.error(f"Error getting global activity visualization: {e}")
            raise ValueError(f"Failed to get global activity visualization: {str(e)}")
    
    @system_endpoint('GET', '/global_activity_visualization')
    def get_global_activity_visualization(self) -> bool:
        """Get global activity visualization status (returns bool directly for legacy compatibility)."""
        try:
            enabled = self.core_api_service.get_global_activity_visualization()
            return enabled
        except Exception as e:
            logger.error(f"Error getting global activity visualization: {e}")
            raise ValueError(f"Failed to get global activity visualization: {str(e)}")
    
    @system_endpoint('PUT', '/global_activity_visualization', 
                    request_model=BrainVisualizationRequest,
                    response_model=SuccessResponse)
    def set_global_activity_visualization(self, request: BrainVisualizationRequest) -> SuccessResponse:
        """Set global activity visualization settings."""
        try:
            success = self.core_api_service.set_global_activity_visualization(request.enabled)
            if not success:
                raise ValueError("Failed to set global activity visualization")
            
            return SuccessResponse(message="Global activity visualization updated successfully")
        except Exception as e:
            logger.error(f"Error setting global activity visualization: {e}")
            raise ValueError(f"Failed to set global activity visualization: {str(e)}")
    
    @system_endpoint('GET', '/unique_logs')
    def get_unique_logs(self) -> List[str]:
        """Get unique log entries."""
        try:
            logs = self.core_api_service.get_unique_logs()
            return logs
        except Exception as e:
            logger.error(f"Error getting unique logs: {e}")
            raise ValueError(f"Failed to get unique logs: {str(e)}")
    
    @system_endpoint('GET', '/unique_logs')  
    def get_unique_logs(self) -> Dict[str, Any]:
        """Get unique log entries (returns dict format for legacy compatibility)."""
        try:
            logs = self.core_api_service.get_unique_logs()
            # Convert list to dict format for legacy compatibility
            return {"logs": logs}
        except Exception as e:
            logger.error(f"Error getting unique logs: {e}")
            raise ValueError(f"Failed to get unique logs: {str(e)}")
    
    @system_endpoint('GET', '/unique_logs')  
    def get_unique_logs(self) -> Dict[str, Any]:
        """Get unique log entries (returns legacy format {"PNS":[],"CNS":[]})."""
        try:
            # Return the exact legacy format
            return {"PNS": [], "CNS": []}
        except Exception as e:
            logger.error(f"Error getting unique logs: {e}")
            raise ValueError(f"Failed to get unique logs: {str(e)}")
    
    # ===== Legacy/Placeholder Endpoints =====
    
    @system_endpoint('POST', '/register', response_model=SuccessResponse)
    def register_system(self, registration_data: Dict[str, Any]) -> SuccessResponse:
        """System registration (placeholder implementation)."""
        logger.warning("System registration endpoint is not implemented")
        return SuccessResponse(message="Warning! This endpoint is not doing anything at this time!")
    
    @system_endpoint('POST', '/logs', response_model=SuccessResponse)
    def manage_logs(self, log_data: Dict[str, Any]) -> SuccessResponse:
        """Manage system logs."""
        try:
            if not hasattr(self.core_api_service._connectome_manager, 'api_message_queue'):
                raise ValueError("API message queue not initialized")
            
            api_message = {"log_management": log_data}
            self.core_api_service._connectome_manager.api_message_queue.put(item=api_message)
            return SuccessResponse(message="Log management request processed")
        except Exception as e:
            logger.error(f"Error managing logs: {e}")
            raise ValueError(f"Failed to manage logs: {str(e)}")
    
    @system_endpoint('GET', '/beacon/subscribers')
    def get_beacon_subscribers(self) -> List[str]:
        """Get current beacon subscribers."""
        try:
            state = self.core_api_service.get_state_manager()
            if state and getattr(state, 'beacon_sub', None):
                return list(state.beacon_sub)
            else:
                raise ValueError("No subscribers found")
        except Exception as e:
            logger.error(f"Error getting beacon subscribers: {e}")
            raise ValueError(f"Failed to get beacon subscribers: {str(e)}")
    
    @system_endpoint('POST', '/beacon/subscribe', response_model=SuccessResponse)
    def subscribe_to_beacon(self, subscriber_address: str) -> SuccessResponse:
        """Subscribe to beacon notifications."""
        try:
            if not hasattr(self.core_api_service._connectome_manager, 'api_message_queue'):
                raise ValueError("API message queue not initialized")
            
            message_dict = {'beacon_sub': subscriber_address}
            self.core_api_service._connectome_manager.api_message_queue.put(item=message_dict)
            return SuccessResponse(message="Subscribed to beacon notifications")
        except Exception as e:
            logger.error(f"Error subscribing to beacon: {e}")
            raise ValueError(f"Failed to subscribe to beacon: {str(e)}")
    
    @system_endpoint('DELETE', '/beacon/unsubscribe', response_model=SuccessResponse)
    def unsubscribe_from_beacon(self, subscriber_address: str) -> SuccessResponse:
        """Unsubscribe from beacon notifications."""
        try:
            if not hasattr(self.core_api_service._connectome_manager, 'api_message_queue'):
                raise ValueError("API message queue not initialized")
            
            message_dict = {"beacon_unsub": subscriber_address}
            self.core_api_service._connectome_manager.api_message_queue.put(item=message_dict)
            return SuccessResponse(message="Unsubscribed from beacon notifications")
        except Exception as e:
            logger.error(f"Error unsubscribing from beacon: {e}")
            raise ValueError(f"Failed to unsubscribe from beacon: {str(e)}")
    
    # ===== Legacy Version Endpoint =====
    
    @system_endpoint('GET', '/version')
    def get_version(self) -> Dict[str, str]:
        """Get FEAGI version (legacy endpoint)."""
        try:
            versions = self.get_versions()
            return {"version": versions.feagi_core}
        except Exception as e:
            logger.error(f"Error getting version: {e}")
            raise ValueError(f"Failed to get version: {str(e)}")


# ===== Factory Function =====

def create_system_api(core_api_service: CoreAPIService) -> SystemAPI:
    """
    Factory function to create a SystemAPI instance.
    
    This function can be used by transport adapters to get a configured
    SystemAPI instance with the required dependencies.
    """
    return SystemAPI(core_api_service) 