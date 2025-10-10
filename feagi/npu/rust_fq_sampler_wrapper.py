"""
Thin wrapper around Rust FQ Sampler for compatibility with existing Python code.

This provides the same interface as the old Python FQSampler but delegates
all work to the Rust implementation.
"""

from typing import Dict, List, Optional, Any
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class RustFQSamplerWrapper:
    """Compatibility wrapper that makes Rust FQ Sampler look like Python FQSampler."""
    
    def __init__(self, rust_npu_integration):
        """Initialize wrapper.
        
        Args:
            rust_npu_integration: RustNPUIntegration instance
        """
        self._rust_npu = rust_npu_integration
        self.instance_id = "rust_fq_sampler_wrapper"
        
    def sample(self) -> Dict[int, Dict[str, List]]:
        """Sample the Fire Queue (delegates to Rust).
        
        Returns:
            Dict mapping cortical_idx to area data with keys:
            - "neuron_ids"
            - "coordinates_x"
            - "coordinates_y"  
            - "coordinates_z"
            - "membrane_potentials"
            
            Returns empty dict if no data available.
        """
        logger.info("[PY-FQ-WRAPPER] sample() called - delegating to Rust")
        result = self._rust_npu.sample_fire_queue()
        logger.info(f"[PY-FQ-WRAPPER] Rust returned: {result}")
        return result if result is not None else {}
    
    def set_sample_frequency(self, frequency_hz: float):
        """Set sampling frequency (Hz)."""
        self._rust_npu.set_fq_sampler_frequency(frequency_hz)
    
    @property
    def sample_frequency_hz(self) -> float:
        """Get sampling frequency (Hz)."""
        return self._rust_npu.get_fq_sampler_frequency()
    
    def set_visualization_subscribers(self, has_subscribers: bool):
        """Set visualization subscriber state."""
        logger.info(f"[PY-FQ-WRAPPER] set_visualization_subscribers({has_subscribers}) called")
        self._rust_npu.set_visualization_subscribers(has_subscribers)
        logger.info(f"[PY-FQ-WRAPPER] set_visualization_subscribers({has_subscribers}) completed")
    
    def set_motor_subscribers(self, has_subscribers: bool):
        """Set motor subscriber state."""
        self._rust_npu.set_motor_subscribers(has_subscribers)

