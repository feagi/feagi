"""
Fire Queue Sampler for FEAGI NPU

Clean implementation that reads from Fire Queue (current timestep only).
Samples firing neurons for visualization, motor output, and external systems.
"""

from typing import Dict, List, Optional, Any
import time
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
        
        logger.info(f"FQ Sampler initialized: {sampling_mode} mode @ {sample_frequency_hz}Hz")
    
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
