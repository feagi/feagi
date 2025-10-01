"""
Enhanced Agent Registration with Capability-Specific Rate Negotiation

This module implements the multi-rate capability architecture where:
1. Agents can request FEAGI to run at specific refresh rates
2. Each capability (sensory, motor, etc.) can have independent polling rates  
3. Rate validation ensures capability rates don't exceed FEAGI global rate
4. Post-registration, capabilities operate at their predefined frequencies
"""

from typing import Dict, Any, Optional, List
from pydantic import BaseModel, Field, validator
from enum import Enum


class CapabilityType(str, Enum):
    """Standard capability types with predefined semantics."""
    SENSORY = "sensory"
    MOTOR = "motor" 
    VISUALIZATION = "visualization"
    CONTROL = "control"
    NEURONS_STREAM = "neurons_stream"  # Legacy compatibility


class CapabilityRateSpec(BaseModel):
    """Rate specification for a single capability."""
    
    capability_type: CapabilityType
    requested_rate_hz: float = Field(
        gt=0, 
        le=1000,  # Reasonable upper bound
        description="Requested polling rate in Hz for this capability"
    )
    required: bool = Field(
        default=True,
        description="Whether this capability is required for agent operation"
    )
    metadata: Optional[Dict[str, Any]] = Field(
        default=None,
        description="Additional capability-specific configuration"
    )
    
    @validator('requested_rate_hz')
    def validate_rate_reasonable(cls, v):
        """Ensure rate is within reasonable bounds."""
        if v < 0.1:
            raise ValueError("Capability rate must be at least 0.1 Hz")
        if v > 1000:
            raise ValueError("Capability rate cannot exceed 1000 Hz")
        return v


class FeagiRateRequest(BaseModel):
    """Request for FEAGI global refresh rate adjustment."""
    
    requested_feagi_rate_hz: float = Field(
        gt=0,
        le=100,  # Reasonable upper bound for FEAGI global rate
        description="Requested FEAGI global refresh rate in Hz"
    )
    justification: Optional[str] = Field(
        default=None,
        description="Reason for requesting specific FEAGI rate"
    )
    
    @validator('requested_feagi_rate_hz') 
    def validate_feagi_rate_reasonable(cls, v):
        """Ensure FEAGI rate is within reasonable bounds."""
        if v < 0.1:
            raise ValueError("FEAGI rate must be at least 0.1 Hz")
        if v > 100:
            raise ValueError("FEAGI rate cannot exceed 100 Hz for stability")
        return v


class EnhancedAgentRegistrationRequest(BaseModel):
    """Enhanced agent registration with capability rate negotiation."""
    
    # Standard registration fields (unchanged)
    agent_type: str
    agent_id: str
    agent_data_port: int
    agent_version: str
    controller_version: str
    agent_ip: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None
    
    # NEW: Rate negotiation fields
    feagi_rate_request: Optional[FeagiRateRequest] = Field(
        default=None,
        description="Optional request to adjust FEAGI global rate"
    )
    capability_rates: List[CapabilityRateSpec] = Field(
        min_items=1,
        description="Rate specifications for each agent capability"
    )
    
    # Legacy compatibility (will be converted to capability_rates)
    capabilities: Optional[Dict[str, Any]] = Field(
        default=None,
        description="Legacy capabilities format (auto-converted)"
    )
    
    @validator('capability_rates')
    def validate_no_duplicate_capabilities(cls, v):
        """Ensure no duplicate capability types."""
        seen_types = set()
        for spec in v:
            if spec.capability_type in seen_types:
                raise ValueError(f"Duplicate capability type: {spec.capability_type}")
            seen_types.add(spec.capability_type)
        return v


class CapabilityRateResult(BaseModel):
    """Result of capability rate negotiation."""
    
    capability_type: CapabilityType
    requested_rate_hz: float
    approved_rate_hz: float  # May be different from requested
    approved: bool
    rejection_reason: Optional[str] = None


class EnhancedAgentRegistrationResponse(BaseModel):
    """Response to enhanced agent registration with rate negotiation results."""
    
    success: bool
    agent_id: str
    message: str
    
    # Rate negotiation results
    feagi_rate_approved: bool
    current_feagi_rate_hz: float
    requested_feagi_rate_hz: Optional[float] = None
    feagi_rate_rejection_reason: Optional[str] = None
    
    capability_results: List[CapabilityRateResult]
    
    # Operational parameters
    approved_capabilities: List[CapabilityType]
    rejected_capabilities: List[CapabilityType] = []
    

class RateValidationError(Exception):
    """Raised when rate validation fails."""
    pass


class CapabilityRateValidator:
    """Validates capability rates against FEAGI global rate and system limits."""
    
    def __init__(self, feagi_global_rate_hz: float):
        self.feagi_global_rate_hz = feagi_global_rate_hz
        
    def validate_capability_rates(
        self, 
        capability_specs: List[CapabilityRateSpec]
    ) -> Dict[CapabilityType, str]:
        """
        Validate all capability rates against FEAGI global rate.
        
        Args:
            capability_specs: List of capability rate specifications
            
        Returns:
            Dict mapping capability types to rejection reasons (empty if approved)
            
        Raises:
            RateValidationError: If validation fails critically
        """
        rejections = {}
        
        for spec in capability_specs:
            rejection_reason = self._validate_single_capability_rate(spec)
            if rejection_reason:
                rejections[spec.capability_type] = rejection_reason
                
        return rejections
    
    def _validate_single_capability_rate(self, spec: CapabilityRateSpec) -> Optional[str]:
        """Validate a single capability rate specification."""
        
        # Rule 1: Capability rate cannot exceed FEAGI global rate
        if spec.requested_rate_hz > self.feagi_global_rate_hz:
            return f"Capability rate {spec.requested_rate_hz}Hz exceeds FEAGI global rate {self.feagi_global_rate_hz}Hz"
        
        # Rule 2: Capability rate should be a reasonable divisor of FEAGI rate for efficiency
        if self.feagi_global_rate_hz % spec.requested_rate_hz != 0:
            # Allow some tolerance for non-perfect divisors
            ratio = self.feagi_global_rate_hz / spec.requested_rate_hz
            if abs(ratio - round(ratio)) > 0.1:
                # Warning but not rejection - just inefficient
                pass
        
        # Rule 3: Very high rates should be questioned for sensory data
        if spec.capability_type == CapabilityType.SENSORY and spec.requested_rate_hz > 60:
            # Allow but warn - many sensors don't need >60Hz
            pass
            
        return None  # Approved
    
    def suggest_optimal_rate(
        self, 
        capability_type: CapabilityType,
        requested_rate_hz: float
    ) -> float:
        """Suggest an optimal rate that's a divisor of FEAGI rate."""
        
        if requested_rate_hz > self.feagi_global_rate_hz:
            return self.feagi_global_rate_hz
            
        # Find the largest divisor of FEAGI rate that's <= requested rate
        optimal_rate = requested_rate_hz
        for divisor in [1, 2, 3, 4, 5, 6, 8, 10, 12, 15, 20, 24, 30, 40, 50, 60]:
            candidate_rate = self.feagi_global_rate_hz / divisor
            if candidate_rate <= requested_rate_hz and candidate_rate >= 0.1:
                optimal_rate = candidate_rate
                break
                
        return optimal_rate
