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
    
    def __init__(self, fire_queue_provider: Any, sample_frequency_hz: float = 10.0, sampling_mode: str = "visualization"):
        """Initialize FQ Sampler."""
        self.fire_queue_provider = fire_queue_provider
        self.sample_frequency_hz = sample_frequency_hz
        self.sampling_mode = sampling_mode
        self.samples_taken = 0
        self.last_sample_time = 0.0
        self.sample_interval = 1.0 / sample_frequency_hz if sample_frequency_hz > 0 else 0.1
        
        # Generate unique instance ID for process manager tracking
        self.instance_id = f"fq_sampler_{sampling_mode}_{str(uuid.uuid4())[:8]}"
        
        # Visualization subscriber tracking
        self._has_visualization_subscribers = False
        
        logger.info(f"FQ Sampler initialized: {sampling_mode} mode @ {sample_frequency_hz}Hz, ID: {self.instance_id}")
    
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
        
        # Sample all areas for now (can extend with modes)
        result = {}
        for cortical_idx in fire_queue.get_active_areas():
            area_data = fire_queue.get_area_fire_queue_dict(cortical_idx)
            if area_data:
                area_id = f"area_{cortical_idx}"
                result[area_id] = area_data
        
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
        self._has_visualization_subscribers = has_subscribers
        logger.debug(f"FQ Sampler {self.instance_id}: visualization subscribers = {has_subscribers}")
    
    def has_visualization_subscribers(self) -> bool:
        """Check if visualization subscribers are connected."""
        return self._has_visualization_subscribers


class UnifiedFQSampler:
    """Compatibility wrapper around FQSampler for legacy process manager integration.
    
    This class provides the same interface as the old UnifiedFQSampler but uses 
    the clean FQSampler implementation underneath. This ensures compatibility
    with existing process manager code while maintaining the clean architecture.
    """
    
    def __init__(self, 
                 fire_queue_provider: Any,
                 sample_frequency_hz: float,
                 sampling_mode: str = "visualization",
                 output_queue: Optional[Any] = None,
                 connectome_manager: Optional[Any] = None,
                 target_areas: Optional[List[str]] = None,
                 state_manager: Optional[Any] = None):
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
        
        logger.info(f"UnifiedFQSampler created as compatibility wrapper: {self.instance_id}")
    
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
    
    # Delegate methods to underlying FQ sampler
    def sample(self) -> Optional[Dict[str, Any]]:
        """Sample fire queue data."""
        return self._fq_sampler.sample()
    
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
