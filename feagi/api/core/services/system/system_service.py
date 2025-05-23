"""System service for managing FEAGI system operations."""

import sys
from datetime import datetime
from typing import Dict, Any, Optional
from ..shared.base_service import BaseService


class SystemService(BaseService):
    """
    System service handles system-level operations like health checks,
    configuration management, and system information.
    """
    
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
                
            # Basic health metrics
            health["burst_engine"] = not getattr(self.state_manager, 'exit_condition', False)
            health["connected_agents"] = getattr(self.state_manager, 'connected_agents', None)
            health["influxdb_availability"] = bool(getattr(self.state_manager, 'influxdb', False))
            
            # Resource limits
            limits = getattr(self.state_manager, 'parameters', {}).get("Limits", {})
            health["neuron_count_max"] = int(limits.get("max_neuron_count", 0))
            health["synapse_count_max"] = int(limits.get("max_synapse_count", 0))
            
            # Genome-related information
            health["latest_changes_saved_externally"] = getattr(self.state_manager, 'changes_saved_externally', False)
            
            # Use the proper state manager method to check if genome is loaded
            if self.state_manager.is_genome_loaded():
                health["fitness"] = getattr(self.state_manager, 'genome_fitness', None)
                health["genome_availability"] = True
                
                # Brain statistics
                brain_stats = getattr(self.state_manager, 'brain_stats', {})
                connectome_neuron_count = brain_stats.get("neuron_count", 0)
                connectome_synapse_count = brain_stats.get("synapse_count", 0)
                
                # Estimate brain size in MB using a formula
                connectome_size = 3E-08 * connectome_neuron_count ** 2 + 0.0011 * connectome_neuron_count + 2.9073
                
                health["cortical_area_count"] = len(getattr(self.state_manager, 'cortical_list', []))
                health["neuron_count"] = connectome_neuron_count
                health["synapse_count"] = connectome_synapse_count
                health["estimated_brain_size_in_MB"] = connectome_size
            else:
                health["genome_availability"] = False
                
            health["genome_validity"] = getattr(self.state_manager, 'genome_validity', None)
            health["brain_readiness"] = self.state_manager.get_brain_readiness()
            
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
            return cortical_types
        except ImportError:
            # Fallback types
            return {
                "MEMORY": {"description": "Memory cortical areas"},
                "IPU": {"description": "Input Processing Units"}, 
                "OPU": {"description": "Output Processing Units"},
                "HIDDEN": {"description": "Hidden processing areas"},
                "CUSTOM": {"description": "Custom cortical areas"}
            }
        except Exception as e:
            self.logger.error(f"Error getting cortical area types: {str(e)}")
            return {}

    def reset_fcl(self) -> bool:
        """Reset the Fire Candidate List."""
        try:
            # First try the connectome manager's reset_fcl method
            if hasattr(self._connectome_manager, 'reset_fcl'):
                self._connectome_manager.reset_fcl()
                return True
            # Then try the FCL manager's reset method
            elif hasattr(self._connectome_manager, 'fcl_manager') and hasattr(self._connectome_manager.fcl_manager, 'reset'):
                self._connectome_manager.fcl_manager.reset()
                return True
            else:
                self.logger.warning("Connectome manager does not support FCL reset")
                return False
        except Exception as e:
            self.logger.error(f"Error resetting FCL: {str(e)}")
            return False 