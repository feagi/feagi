"""Brain service for managing FEAGI brain simulation operations."""

from typing import Dict, Any, Optional, List
from ..shared.base_service import BaseService


class BrainService(BaseService):
    """
    Brain service handles brain simulation operations including
    burst engine control, monitoring, and analysis.
    """
    
    def get_burst_engine_status(self) -> Dict[str, Any]:
        """Get current burst engine status."""
        try:
            if not self.state_manager:
                return {"status": "unknown", "error": "State manager not available"}
            
            # Check if burst engine is running
            is_running = not getattr(self.state_manager, 'exit_condition', False)
            
            # Get current burst statistics
            current_burst = getattr(self.state_manager, 'current_burst_id', 0)
            
            return {
                "status": "running" if is_running else "stopped",
                "current_burst": current_burst,
                "brain_ready": self.state_manager.get_brain_readiness(),
                "genome_loaded": self.state_manager.is_genome_loaded()
            }
        except Exception as e:
            self.logger.error(f"Error getting burst engine status: {str(e)}")
            return {"status": "error", "error": str(e)}

    def start_burst_engine(self) -> bool:
        """Start the burst engine."""
        try:
            if not self.state_manager:
                return False
            
            # Clear exit condition to start the burst engine
            self.state_manager.exit_condition = False
            
            # CRITICAL: Set burst engine state to READY 
            from feagi.core.state_manager import ServiceState
            self.state_manager.set_burst_engine_state(ServiceState.READY)
            
            self.logger.info("Burst engine started via BrainService")
            return True
        except Exception as e:
            self.logger.error(f"Error starting burst engine: {str(e)}")
            return False

    def stop_burst_engine(self) -> bool:
        """Stop the burst engine."""
        try:
            if not self.state_manager:
                return False
            
            # Set exit condition to stop the burst engine
            self.state_manager.exit_condition = True
            
            # CRITICAL: Set burst engine state to UNAVAILABLE
            from feagi.core.state_manager import ServiceState
            self.state_manager.set_burst_engine_state(ServiceState.UNAVAILABLE)
            
            self.logger.info("Burst engine stopped via BrainService")
            return True
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
                self.logger.warning(f"Cannot hold burst engine - current state: {current_state.name}")
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
                self.logger.warning(f"Cannot resume burst engine - current state: {current_state.name}")
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
            brain_stats = getattr(self.state_manager, 'brain_stats', {})
            
            # Combine with connectome statistics
            stats = {
                "neuron_count": brain_stats.get("neuron_count", 0),
                "synapse_count": brain_stats.get("synapse_count", 0),
                "cortical_area_count": len(getattr(self.state_manager, 'cortical_list', [])),
                "genome_loaded": self.state_manager.is_genome_loaded(),
                "brain_ready": self.state_manager.get_brain_readiness()
            }
            
            # Add connectome manager stats if available
            if self._connectome_manager:
                stats.update({
                    "active_cortical_areas": len(getattr(self._connectome_manager, 'cortical_areas', {})),
                    "current_timestep": getattr(self._connectome_manager, 'current_timestep', 0)
                })
            
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
            current_time = getattr(self._connectome_manager, 'current_timestep', 0)
            
            # Count active neurons across all areas
            total_active = 0
            total_neurons = 0
            area_activity = {}
            
            for area_idx, area in self._connectome_manager.cortical_areas.items():
                area_neurons = self._connectome_manager.get_neurons_by_area(area_idx)
                area_active = 0
                
                for neuron_id in area_neurons:
                    neuron_index = self._connectome_manager._neuron_id_to_index.get(neuron_id)
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
                    "activity_ratio": area_active / len(area_neurons) if area_neurons else 0
                }
            
            return {
                "total_active_neurons": total_active,
                "total_neurons": total_neurons,
                "overall_activity_ratio": total_active / total_neurons if total_neurons > 0 else 0,
                "current_timestep": current_time,
                "time_window": window,
                "area_activity": area_activity
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
            if hasattr(self._connectome_manager, 'reset_neural_states'):
                self._connectome_manager.reset_neural_states()
            
            # Reset timestep
            if hasattr(self._connectome_manager, 'current_timestep'):
                self._connectome_manager.current_timestep = 0
            
            # Clear FCL if available
            if hasattr(self._connectome_manager, 'reset_fcl'):
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
                "burst_duration_ms": getattr(self.state_manager, 'last_burst_duration', 0),
                "bursts_per_second": getattr(self.state_manager, 'bursts_per_second', 0),
                "neurons_processed": getattr(self.state_manager, 'neurons_processed_last_burst', 0),
                "synapses_processed": getattr(self.state_manager, 'synapses_processed_last_burst', 0)
            }
            
            # Add memory usage if available
            try:
                import psutil
                process = psutil.Process()
                metrics.update({
                    "memory_usage_mb": process.memory_info().rss / 1024 / 1024,
                    "cpu_percent": process.cpu_percent()
                })
            except ImportError:
                pass
            
            return metrics
        except Exception as e:
            self.logger.error(f"Error getting performance metrics: {str(e)}")
            return {}

    def stimulate_neurons(self, neuron_ids: List[str], intensity: float = 1.0) -> Dict[str, Any]:
        """Stimulate specific neurons with given intensity."""
        try:
            if not self._validate_genome_loaded():
                return {"success": False, "error": "No genome loaded"}
            
            stimulated_count = 0
            failed_count = 0
            
            for neuron_id in neuron_ids:
                try:
                    neuron_index = self._connectome_manager._neuron_id_to_index.get(neuron_id)
                    if neuron_index is None:
                        failed_count += 1
                        continue
                    
                    # Apply stimulation by setting membrane potential
                    self._connectome_manager.membrane_potentials[neuron_index] = intensity
                    stimulated_count += 1
                except Exception as e:
                    self.logger.warning(f"Failed to stimulate neuron {neuron_id}: {str(e)}")
                    failed_count += 1
            
            return {
                "success": True,
                "stimulated_count": stimulated_count,
                "failed_count": failed_count,
                "total_requested": len(neuron_ids),
                "intensity": intensity
            }
        except Exception as e:
            self.logger.error(f"Error stimulating neurons: {str(e)}")
            return {"success": False, "error": str(e)}

    def get_burst_engine_config(self) -> Dict[str, Any]:
        """Get burst engine configuration."""
        try:
            if not self.state_manager:
                return {"error": "State manager not available"}
            
            # Get configuration from state manager or use defaults
            config = {
                "burst_frequency_hz": getattr(self.state_manager, 'burst_frequency', 1.0),
                "max_neurons_per_burst": getattr(self.state_manager, 'max_neurons_per_burst', 1000),
                "burst_timeout_ms": getattr(self.state_manager, 'burst_timeout', 1000),
                "auto_restart": getattr(self.state_manager, 'auto_restart', True),
                "performance_mode": getattr(self.state_manager, 'performance_mode', "normal")
            }
            
            return config
        except Exception as e:
            self.logger.error(f"Error getting burst engine config: {str(e)}")
            return {"error": str(e)}

    def get_burst_timer(self) -> float:
        """Get burst timer (stimulation period) from burst engine."""
        try:
            if self.state_manager:
                # Get burst frequency and convert to period in seconds
                frequency = getattr(self.state_manager, 'burst_frequency', 1.0)
                if frequency > 0:
                    return 1.0 / frequency
                return 1.0  # Default 1 second period
            return 1.0
        except Exception as e:
            self.logger.error(f"Error getting burst timer: {str(e)}")
            return 1.0 