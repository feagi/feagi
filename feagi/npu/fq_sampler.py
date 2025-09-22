"""
Fire Queue Sampler for FEAGI NPU

Clean implementation that reads from Fire Queue (current timestep only).
Samples firing neurons for visualization, motor output, and external systems.
"""

from typing import Dict, List, Optional, Any
import time
import uuid
from feagi.utils.logger import setup_logger
from .fire_queue import FireQueue, FiringNeuron

logger = setup_logger(__name__)


class FQSampler:
    """Clean Fire Queue Sampler - reads from current Fire Queue only."""
    
    def __init__(self, fire_queue_provider: Any = None, fire_queue: Any = None, sample_frequency_hz: float = 10.0, sampling_mode: str = "visualization", output_queue: Any = None, connectome_manager: Any = None, target_areas: List[str] = None):
        """Initialize FQ Sampler."""
        # Support both fire_queue_provider and fire_queue parameters for compatibility
        if fire_queue is not None:
            self.fire_queue_provider = fire_queue
        else:
            self.fire_queue_provider = fire_queue_provider
        self.sample_frequency_hz = sample_frequency_hz
        self.sample_frequency = sample_frequency_hz  # Compatibility alias 
        self.sampling_mode = sampling_mode
        self.output_queue = output_queue  # Store output_queue for compatibility
        self.samples_taken = 0
        self.last_sample_time = 0.0
        self.sample_interval = 1.0 / sample_frequency_hz if sample_frequency_hz > 0 else 0.1
        
        # Generate unique instance ID for process manager tracking
        self.instance_id = f"fq_sampler_{sampling_mode}_{str(uuid.uuid4())[:8]}"
        
        # Visualization subscriber tracking
        self._has_visualization_subscribers = False
        self._has_motor_subscribers = False
        
        # Compatibility attributes
        self.connectome_manager = connectome_manager
        self.running = False
        self.target_areas = target_areas or []
        
        # FQ Sampler initialized
    
    def sample(self) -> Optional[Dict[str, Any]]:
        """Sample current Fire Queue data organized by cortical areas."""
        current_time = time.time()
        
        # Respect sampling frequency
        if current_time - self.last_sample_time < self.sample_interval:
            return None
        
        # Get current fire queue
        fire_queue = self._get_current_fire_queue()
        if fire_queue is None or fire_queue.is_empty():
            return None
        
        active_areas = fire_queue.get_active_areas()
        
        # Sample all areas for now (can extend with modes)
        result = {}
        sampled_neurons_total = 0
        
        for cortical_idx in active_areas:
            area_data = fire_queue.get_area_fire_queue_dict(cortical_idx)
            if area_data:
                # Use integer cortical_idx directly as key for proper conversion
                result[cortical_idx] = area_data
                
                # COORDINATE DEBUG: Log sample coordinates from each area
                if isinstance(area_data, dict) and 'coordinates_x' in area_data:
                    coords_x = area_data.get('coordinates_x', [])
                    coords_y = area_data.get('coordinates_y', [])
                    coords_z = area_data.get('coordinates_z', [])
                    if coords_x:
                        pass  # Coordinate sampling available
        
        self.samples_taken += 1
        self.last_sample_time = current_time
        
        return result
    
    def _get_current_fire_queue(self) -> Optional[FireQueue]:
        """Get current fire queue from provider."""
        if hasattr(self.fire_queue_provider, 'get_current_fire_queue'):
            return self.fire_queue_provider.get_current_fire_queue()
        elif isinstance(self.fire_queue_provider, FireQueue):
            return self.fire_queue_provider
        return None
    
    def _is_debug_npu_enabled(self) -> bool:
        """Check if NPU debug logging is enabled through the fire queue provider."""
        try:
            # Try to get state manager from fire_queue_provider (usually BurstEngine)
            if hasattr(self.fire_queue_provider, 'state_manager'):
                state_manager = self.fire_queue_provider.state_manager
                if state_manager and hasattr(state_manager, 'is_debug_npu_enabled'):
                    return state_manager.is_debug_npu_enabled()
            
            # Try to get connectome_manager from fire_queue_provider and then state_manager
            if hasattr(self.fire_queue_provider, 'connectome_manager'):
                cm = self.fire_queue_provider.connectome_manager
                if cm and hasattr(cm, 'state_manager'):
                    state_manager = cm.state_manager
                    if state_manager and hasattr(state_manager, 'is_debug_npu_enabled'):
                        return state_manager.is_debug_npu_enabled()
            
            return False
        except Exception:
            return False
    
    def get_area_fire_queue(self, area_id: str) -> Optional[Dict[str, Any]]:
        """Get fire queue data for specific area (compatibility method)."""
        fire_queue = self._get_current_fire_queue()
        if fire_queue is None:
            return None
            
        try:
            cortical_idx = int(area_id.replace('area_', '')) if 'area_' in area_id else int(area_id)
            return fire_queue.get_area_fire_queue_dict(cortical_idx)
        except (ValueError, AttributeError):
            return None
    
    def get_area_fire_queue_zerocopy(self, area_id: str) -> Optional[Dict[str, Any]]:
        """Zero-copy version for high performance."""
        # For now, same as regular method
        # In Rust implementation, this would use zero-copy arrays
        return self.get_area_fire_queue(area_id)
    
    def set_visualization_subscribers(self, has_subscribers: bool):
        """Set whether visualization subscribers are connected.
        
        Args:
            has_subscribers: True if visualization clients are connected
        """
        old_state = self._has_visualization_subscribers
        self._has_visualization_subscribers = has_subscribers
        
        if old_state != has_subscribers:
            pass  # Visualization subscriber state changed
    
    def has_visualization_subscribers(self) -> bool:
        """Check if visualization subscribers are connected."""
        return self._has_visualization_subscribers
    
    def set_sample_frequency(self, frequency_hz: float):
        """Set the sample frequency for the FQ sampler.
        
        Args:
            frequency_hz: New sampling frequency in Hz
        """
        if frequency_hz > 0:
            self.sample_frequency_hz = frequency_hz


    def set_motor_subscribers(self, has_subscribers: bool) -> None:
        """Set whether there are motor subscribers (compatibility method)."""  
        self._has_motor_subscribers = has_subscribers


class UnifiedFQSampler:
    """Compatibility wrapper around FQSampler for legacy process manager integration.
    
    This class provides the same interface as the old UnifiedFQSampler but uses 
    the clean FQSampler implementation underneath. This ensures compatibility
    with existing process manager code while maintaining the clean architecture.
    """
    
    def __init__(self, 
                 fire_queue_provider: Any = None,
                 sample_frequency_hz: float = 10.0,
                 sampling_mode: str = "visualization",
                 output_queue: Optional[Any] = None,
                 connectome_manager: Optional[Any] = None,
                 target_areas: Optional[List[str]] = None,
                 state_manager: Optional[Any] = None,
                 backend: Optional[str] = None):
        """Initialize UnifiedFQSampler with compatibility interface."""
        
        # Create underlying clean FQ sampler
        self._fq_sampler = FQSampler(
            fire_queue_provider=fire_queue_provider,
            sample_frequency_hz=sample_frequency_hz,
            sampling_mode=sampling_mode
        )
        
        # Store additional parameters for compatibility
        self.output_queue = output_queue
        self.connectome_manager = connectome_manager
        self.target_areas = target_areas or []
        self.state_manager = state_manager
        
        # UnifiedFQSampler compatibility wrapper created
    
    # Delegate properties to underlying FQ sampler
    @property
    def instance_id(self) -> str:
        """Get the instance ID."""
        return self._fq_sampler.instance_id
    
    @property
    def sample_frequency_hz(self) -> float:
        """Get the sample frequency."""
        return self._fq_sampler.sample_frequency_hz
    
    @property
    def sampling_mode(self) -> str:
        """Get the sampling mode."""
        return self._fq_sampler.sampling_mode
    
    @property
    def _has_visualization_subscribers(self) -> bool:
        """Get visualization subscriber status - CRITICAL DELEGATION for VisualizationStream."""
        return self._fq_sampler._has_visualization_subscribers
    
    # Delegate methods to underlying FQ sampler
    def sample(self, fire_queue: Any = None) -> Optional[Dict[str, Any]]:
        """Sample fire queue data."""
        if fire_queue is not None:
            # If fire_queue provided, use it for sampling
            return fire_queue.get_firing_neurons_by_area() if hasattr(fire_queue, 'get_firing_neurons_by_area') else {}
        return self._fq_sampler.sample()
    
    def sample_fire_queue(self, fire_queue: Any) -> Dict[int, List[int]]:
        """Sample provided fire queue (legacy compatibility)."""
        if hasattr(fire_queue, 'get_firing_neurons_by_area'):
            return fire_queue.get_firing_neurons_by_area()
        return {}
    
    def get_area_fire_queue(self, area_id: str) -> Optional[Dict[str, Any]]:
        """Get fire queue data for specific area."""
        return self._fq_sampler.get_area_fire_queue(area_id)
    
    def get_area_fire_queue_zerocopy(self, area_id: str) -> Optional[Dict[str, Any]]:
        """Get fire queue data with zero-copy optimization."""
        return self._fq_sampler.get_area_fire_queue_zerocopy(area_id)
    
    def set_visualization_subscribers(self, has_subscribers: bool):
        """Set visualization subscriber status."""
        self._fq_sampler.set_visualization_subscribers(has_subscribers)
    
    def has_visualization_subscribers(self) -> bool:
        """Check if visualization subscribers are connected."""
        return self._fq_sampler.has_visualization_subscribers()
    
    def set_sample_frequency(self, frequency_hz: float):
        """Set the sample frequency for the FQ sampler.
        
        Args:
            frequency_hz: New sampling frequency in Hz
        """
        self._fq_sampler.set_sample_frequency(frequency_hz)
    
    # Additional methods that might be expected by legacy code
    def get_statistics(self) -> Dict[str, Any]:
        """Get sampling statistics."""
        return {
            'instance_id': self.instance_id,
            'sampling_mode': self.sampling_mode,
            'sample_frequency_hz': self.sample_frequency_hz,
            'samples_taken': getattr(self._fq_sampler, 'samples_taken', 0),
            'has_visualization_subscribers': self.has_visualization_subscribers()
        }
