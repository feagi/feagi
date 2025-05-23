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
            return True
        except Exception as e:
            self.logger.error(f"Error stopping burst engine: {str(e)}")
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
                    neuron_id_int = int(neuron_id)
                    neuron_index = self._connectome_manager._neuron_id_to_index.get(neuron_id_int)
                    
                    if neuron_index is not None:
                        # Add stimulation to membrane potential
                        current_potential = self._connectome_manager.membrane_potentials[neuron_index]
                        self._connectome_manager.membrane_potentials[neuron_index] = current_potential + intensity
                        stimulated_count += 1
                    else:
                        failed_count += 1
                except ValueError:
                    failed_count += 1
            
            return {
                "success": True,
                "stimulated_neurons": stimulated_count,
                "failed_neurons": failed_count,
                "intensity": intensity
            }
        except Exception as e:
            self.logger.error(f"Error stimulating neurons: {str(e)}")
            return {"success": False, "error": str(e)} 