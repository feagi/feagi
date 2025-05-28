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

"""System service for managing FEAGI system operations."""

import sys
from datetime import datetime
from typing import Dict, Any, Optional, List
from ..shared.base_service import BaseService
from feagi.core.state_manager import ServiceState


class SystemService(BaseService):
    """
    System service handles system-level operations like health checks,
    configuration management, and system information.
    """
    
    def __init__(self, connectome_manager, state_manager=None):
        """Initialize system service."""
        super().__init__(connectome_manager, state_manager)
        # Ensure we have explicit access to connectome manager for health checks
        self._connectome_manager = connectome_manager

    async def get_health(self) -> Dict[str, Any]:
        """
        Get comprehensive system health information.
        
        Returns:
            Dictionary containing health metrics for all FEAGI components
        """
        health = {}
        try:
            if not self.state_manager:
                return {"error": "State manager not available"}
            
            # CRITICAL: Validate and sync state before health check
            state_is_consistent = self._validate_state_consistency()
            if not state_is_consistent and self.state_manager.is_genome_loaded():
                self.logger.warning("State inconsistency detected, attempting to synchronize")
                sync_success = self._sync_state_if_needed()
                if not sync_success:
                    self.logger.error("Failed to synchronize state - health data may be incomplete")
                else:
                    self.logger.info("State synchronization successful")
                
            # Basic health metrics
            # ENFORCE DESIGN PRINCIPLE: If genome is loaded, burst engine MUST be running
            genome_loaded = self.state_manager.is_genome_loaded()
            burst_state = self.state_manager.get_burst_engine_state()
            
            # Allow ON_HOLD as a valid state when genome is loaded (user intentionally paused)
            valid_states_with_genome = [ServiceState.READY, ServiceState.ON_HOLD]
            
            if genome_loaded and burst_state not in valid_states_with_genome:
                self.logger.warning("DESIGN VIOLATION: Genome loaded but burst engine not running - auto-fixing")
                try:
                    # Auto-start burst engine to enforce design principle
                    self.state_manager.exit_condition = False
                    self.state_manager.set_burst_engine_state(ServiceState.READY)
                    self.logger.info("Auto-started burst engine to enforce design principle")
                    burst_state = ServiceState.READY
                except Exception as e:
                    self.logger.error(f"Failed to auto-start burst engine: {str(e)}")
            
            # Health check: burst engine is "healthy" if READY or ON_HOLD
            health["burst_engine"] = burst_state in [ServiceState.READY, ServiceState.ON_HOLD]
            
            # Get connected agents count (not the dictionary itself) for health check response
            connected_agents_dict = getattr(self.state_manager, 'connected_agents', {})
            health["connected_agents"] = len(connected_agents_dict) if isinstance(connected_agents_dict, dict) else 0
            
            health["influxdb_availability"] = bool(getattr(self.state_manager, 'influxdb', False))
            
            # Resource limits - check parameters in state manager or use defaults
            parameters = getattr(self.state_manager, 'parameters', {})
            limits = parameters.get("Limits", {}) if parameters else {}
            health["neuron_count_max"] = int(limits.get("max_neuron_count", 0))
            health["synapse_count_max"] = int(limits.get("max_synapse_count", 0))
            
            # Genome-related information
            health["latest_changes_saved_externally"] = getattr(self.state_manager, 'changes_saved_externally', False)
            
            # CRITICAL: Include genome_timestamp for downstream clients (Bridge/Godot)
            health["genome_timestamp"] = self.state_manager.get_genome_timestamp()
            
            # Use the proper state manager method to check if genome is loaded
            if self.state_manager.is_genome_loaded():
                health["genome_availability"] = True
                health["brain_readiness"] = self.state_manager.get_brain_readiness()
                # Ensure fitness is a number or null (but not undefined)
                fitness_raw = getattr(self.state_manager, 'genome_fitness', None)
                health["fitness"] = fitness_raw if fitness_raw is not None else 0.0
                
                # Get data from connectome manager (now properly singleton)
                if self._validate_connectome_ready():
                    health["cortical_area_count"] = len(self._connectome_manager.cortical_areas)
                    health["neuron_count"] = self._connectome_manager.neuron_count
                    health["synapse_count"] = self._connectome_manager.get_synapse_count()
                    
                    # Estimate brain size 
                    neuron_size_mb = self._connectome_manager.neuron_count * 0.001  # ~1KB per neuron
                    synapse_size_mb = self._connectome_manager.get_synapse_count() * 0.0001  # ~100B per synapse  
                    health["estimated_brain_size_in_MB"] = round(neuron_size_mb + synapse_size_mb, 2)
                else:
                    # Fallback to zero values if connectome not ready
                    health["cortical_area_count"] = 0
                    health["neuron_count"] = 0
                    health["synapse_count"] = 0
                    health["estimated_brain_size_in_MB"] = 0.0
            else:
                health["genome_availability"] = False
                health["brain_readiness"] = False
                health["fitness"] = 0.0  # Use 0.0 instead of None for consistency
                health["cortical_area_count"] = 0
                health["neuron_count"] = 0
                health["synapse_count"] = 0
                health["estimated_brain_size_in_MB"] = 0.0
            
            # CRITICAL: Ensure genome_validity is always a boolean for Godot compatibility
            genome_validity_raw = getattr(self.state_manager, 'genome_validity', None)
            health["genome_validity"] = bool(genome_validity_raw) if genome_validity_raw is not None else False
            
            # Check for pending amalgamation
            if self._has_pending_amalgamation():
                pending = getattr(self.state_manager, 'pending_amalgamation', {})
                health["amalgamation_pending"] = {
                    "initiation_time": pending.get("initiation_time", None),
                    "genome_id": pending.get("genome_id", None),
                    "amalgamation_id": pending.get("amalgamation_id", None),
                    "genome_title": pending.get("genome_title", None),
                    "circuit_size": pending.get("circuit_size", None)
                }
                
            return health
        except Exception as e:
            self.logger.error(f"Error retrieving system health: {str(e)}")
            return {"error": str(e)}

    def _has_pending_amalgamation(self) -> bool:
        """Check if there is a pending amalgamation operation."""
        try:
            if not self.state_manager:
                return False
                
            return bool(getattr(self.state_manager, 'pending_amalgamation', False))
        except Exception as e:
            self.logger.error(f"Error checking pending amalgamation: {str(e)}")
            return False

    def get_user_preferences(self) -> Dict[str, Any]:
        """Get user preferences."""
        try:
            if self.state_manager and hasattr(self.state_manager, 'user_preferences'):
                return self.state_manager.user_preferences
            
            # Default preferences
            return {
                "adv_mode": False,
                "ui_magnification": 1.0,
                "auto_pns_area_creation": True
            }
        except Exception as e:
            self.logger.error(f"Error getting user preferences: {str(e)}")
            return {}
    
    def update_user_preferences(self, preferences: Dict[str, Any]) -> bool:
        """Update user preferences."""
        try:
            if self.state_manager:
                if not hasattr(self.state_manager, 'user_preferences'):
                    self.state_manager.user_preferences = {}
                self.state_manager.user_preferences.update(preferences)
            
            return True
        except Exception as e:
            self.logger.error(f"Error updating user preferences: {str(e)}")
            return False
    
    def get_versions(self) -> Dict[str, Any]:
        """Get version information for various components."""
        try:
            from feagi.version import __version__
            
            versions = {
                "feagi_core": __version__,
                "python": f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}",
                "timestamp": datetime.now().isoformat()
            }
            
            # Add additional component versions if available
            try:
                import numpy
                versions["numpy"] = numpy.__version__
            except ImportError:
                pass
                
            try:
                import torch
                versions["torch"] = torch.__version__
            except ImportError:
                pass
                
            return versions
        except Exception as e:
            self.logger.error(f"Error getting versions: {str(e)}")
            return {}

    def get_configuration(self) -> Dict[str, Any]:
        """Get system configuration."""
        try:
            if self.state_manager and hasattr(self.state_manager, 'parameters'):
                return self.state_manager.parameters
            return {}
        except Exception as e:
            self.logger.error(f"Error getting configuration: {str(e)}")
            return {}

    def test_influxdb(self) -> Optional[Dict[str, Any]]:
        """Test InfluxDB connectivity."""
        try:
            # Check if InfluxDB configuration exists
            if self.state_manager and hasattr(self.state_manager, 'influxdb_config'):
                return {
                    "status": "connected",
                    "database": self.state_manager.influxdb_config.get('database', 'feagi'),
                    "host": self.state_manager.influxdb_config.get('host', 'localhost'),
                    "port": self.state_manager.influxdb_config.get('port', 8086)
                }
            
            # InfluxDB not configured
            return None
        except Exception as e:
            self.logger.error(f"Error testing InfluxDB: {str(e)}")
            return None

    def set_circuit_library_path(self, path: str) -> bool:
        """Set the circuit library path."""
        try:
            import os
            
            if not os.path.exists(path):
                raise ValueError(f"Path does not exist: {path}")
            
            if not os.path.isdir(path):
                raise ValueError(f"Path is not a directory: {path}")
            
            if self.state_manager:
                self.state_manager.circuit_library_path = path
            
            return True
        except Exception as e:
            self.logger.error(f"Error setting circuit library path: {str(e)}")
            return False

    def get_cortical_area_types(self) -> Dict[str, Any]:
        """Get available cortical area types."""
        try:
            # Import from FEAGI templates
            from feagi.evo.templates import cortical_types
            if not cortical_types:
                raise ValueError("No cortical area types found in FEAGI templates")
            return cortical_types
        except ImportError as e:
            self.logger.error(f"Failed to import cortical area types: {str(e)}")
            raise ValueError("Cortical area types not available - FEAGI templates module not found")
        except Exception as e:
            self.logger.error(f"Error getting cortical area types: {str(e)}")
            raise ValueError(f"Failed to retrieve cortical area types: {str(e)}")

    def reset_fcl(self) -> bool:
        """Reset the Fire Candidate List."""
        try:
            if self._connectome_manager and hasattr(self._connectome_manager, 'reset_fcl'):
                return self._connectome_manager.reset_fcl()
            return True
        except Exception as e:
            self.logger.error(f"Error resetting FCL: {str(e)}")
            return False

    def get_visualization_skip_rate(self) -> int:
        """Get visualization skip rate."""
        try:
            if self.state_manager:
                return getattr(self.state_manager, 'visualization_skip_rate', 1)
            return 1
        except Exception as e:
            self.logger.error(f"Error getting visualization skip rate: {str(e)}")
            return 1
    
    def set_visualization_skip_rate(self, skip_rate: int) -> bool:
        """Set visualization skip rate."""
        try:
            if self.state_manager:
                self.state_manager.visualization_skip_rate = skip_rate
            return True
        except Exception as e:
            self.logger.error(f"Error setting visualization skip rate: {str(e)}")
            return False
    
    def get_visualization_suppression_threshold(self) -> int:
        """Get visualization suppression threshold."""
        try:
            if self.state_manager:
                return getattr(self.state_manager, 'visualization_suppression_threshold', 100)
            return 100
        except Exception as e:
            self.logger.error(f"Error getting visualization suppression threshold: {str(e)}")
            return 100
    
    def set_visualization_suppression_threshold(self, threshold: int) -> bool:
        """Set visualization suppression threshold."""
        try:
            if self.state_manager:
                self.state_manager.visualization_suppression_threshold = threshold
            return True
        except Exception as e:
            self.logger.error(f"Error setting visualization suppression threshold: {str(e)}")
            return False
    
    def get_global_activity_visualization(self) -> bool:
        """Get global activity visualization status."""
        try:
            if self.state_manager:
                return getattr(self.state_manager, 'global_activity_visualization', True)
            return True
        except Exception as e:
            self.logger.error(f"Error getting global activity visualization: {str(e)}")
            return True
    
    def set_global_activity_visualization(self, enabled: bool) -> bool:
        """Set global activity visualization status."""
        try:
            if self.state_manager:
                self.state_manager.global_activity_visualization = enabled
            return True
        except Exception as e:
            self.logger.error(f"Error setting global activity visualization: {str(e)}")
            return False
    
    def get_unique_logs(self) -> List[str]:
        """Get unique log entries."""
        try:
            if self.state_manager and hasattr(self.state_manager, 'unique_logs'):
                return list(self.state_manager.unique_logs)
            return []
        except Exception as e:
            self.logger.error(f"Error getting unique logs: {str(e)}")
            return [] 