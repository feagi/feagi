"""
Enhanced Agent Registration API

Implements the multi-rate capability architecture with rate negotiation.
Provides both enhanced registration (with rates) and backward compatibility.

Key Features:
- Per-capability rate negotiation during registration
- FEAGI rate adjustment requests from agents  
- Rate validation against system limits
- Automatic rate optimization for efficiency
- Backward compatibility with legacy registration
"""

from typing import Dict, Any, Optional, List
import time

from fastapi import HTTPException, status
from pydantic import ValidationError

from feagi.utils.logger import setup_logger
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.v1.schemas import (
    AgentRegistrationRequest,
    SuccessResponse
)
from feagi.api.v1.capability_rates import (
    EnhancedAgentRegistrationRequest,
    EnhancedAgentRegistrationResponse,
    CapabilityType,
    CapabilityRateSpec,
    CapabilityRateResult,
    FeagiRateRequest
)
from feagi.core.capability_rate_manager import (
    get_capability_rate_manager,
    AgentCapabilityRate
)
from feagi.core.state_manager import FeagiStateManager

logger = setup_logger(__name__)


class EnhancedAgentRegistrationAPI:
    """Enhanced agent registration with capability rate negotiation."""
    
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service
        self.logger = logger
        
    async def register_agent_enhanced(
        self,
        request: EnhancedAgentRegistrationRequest
    ) -> EnhancedAgentRegistrationResponse:
        """
        Register agent with capability rate negotiation.
        
        This is the new enhanced registration endpoint that supports:
        1. FEAGI rate adjustment requests
        2. Per-capability rate specifications  
        3. Rate validation and optimization
        4. Detailed negotiation results
        """
        try:
            # Get current system state
            state_manager = FeagiStateManager.instance()
            capability_manager = get_capability_rate_manager(state_manager)
            
            if not capability_manager:
                raise HTTPException(
                    status_code=503,
                    detail="Capability rate manager not available"
                )
            
            current_feagi_rate = capability_manager.get_current_feagi_rate_hz()
            
            # Handle FEAGI rate adjustment request
            feagi_rate_approved = True
            feagi_rate_rejection_reason = None
            new_feagi_rate = current_feagi_rate
            
            if request.feagi_rate_request:
                feagi_rate_approved, new_feagi_rate, feagi_rate_rejection_reason = \
                    await self._process_feagi_rate_request(
                        request.feagi_rate_request,
                        current_feagi_rate,
                        state_manager
                    )
            
            # Convert legacy capabilities to capability rate specs if needed
            capability_specs = self._normalize_capability_specs(request)
            
            if not capability_specs:
                raise HTTPException(
                    status_code=400,
                    detail="At least one capability with rate must be specified"
                )
            
            # Register capabilities with rate validation
            approved_configs, rejections = capability_manager.register_agent_capabilities(
                request.agent_id,
                capability_specs
            )
            
            # Check if any required capabilities were rejected
            rejected_required = [
                spec for spec in capability_specs 
                if spec.capability_type in rejections and spec.required
            ]
            
            if rejected_required:
                # Registration fails if required capabilities are rejected
                rejection_details = [
                    f"{spec.capability_type.value}: {rejections[spec.capability_type]}"
                    for spec in rejected_required
                ]
                raise HTTPException(
                    status_code=400,
                    detail=f"Required capabilities rejected: {'; '.join(rejection_details)}"
                )
            
            # Perform standard agent registration
            registration_success = self.core_api_service.register_agent(
                agent_id=request.agent_id,
                agent_type=request.agent_type,
                capabilities=self._build_legacy_capabilities_dict(approved_configs),
                agent_data_port=request.agent_data_port,
                agent_version=request.agent_version,
                controller_version=request.controller_version,
                agent_ip=request.agent_ip
            )
            
            if not registration_success:
                # Cleanup capability registration on failure
                capability_manager.deregister_agent(request.agent_id)
                raise HTTPException(
                    status_code=500,
                    detail="Agent registration failed"
                )
            
            # Build response
            capability_results = []
            approved_capabilities = []
            rejected_capabilities = []
            
            for spec in capability_specs:
                if spec.capability_type in rejections:
                    # Rejected capability
                    result = CapabilityRateResult(
                        capability_type=spec.capability_type,
                        requested_rate_hz=spec.requested_rate_hz,
                        approved_rate_hz=0.0,
                        approved=False,
                        rejection_reason=rejections[spec.capability_type]
                    )
                    rejected_capabilities.append(spec.capability_type)
                else:
                    # Approved capability - find the approved config
                    approved_config = next(
                        (cfg for cfg in approved_configs 
                         if cfg.capability_type == spec.capability_type),
                        None
                    )
                    if approved_config:
                        result = CapabilityRateResult(
                            capability_type=spec.capability_type,
                            requested_rate_hz=spec.requested_rate_hz,
                            approved_rate_hz=approved_config.approved_rate_hz,
                            approved=True
                        )
                        approved_capabilities.append(spec.capability_type)
                
                capability_results.append(result)
            
            response = EnhancedAgentRegistrationResponse(
                success=True,
                agent_id=request.agent_id,
                message=f"Agent registered with {len(approved_capabilities)} capabilities",
                feagi_rate_approved=feagi_rate_approved,
                current_feagi_rate_hz=new_feagi_rate,
                requested_feagi_rate_hz=request.feagi_rate_request.requested_feagi_rate_hz if request.feagi_rate_request else None,
                feagi_rate_rejection_reason=feagi_rate_rejection_reason,
                capability_results=capability_results,
                approved_capabilities=approved_capabilities,
                rejected_capabilities=rejected_capabilities
            )
            
            self.logger.info(
                f"Enhanced registration completed for agent {request.agent_id}: "
                f"{len(approved_capabilities)} capabilities approved, "
                f"{len(rejected_capabilities)} rejected, "
                f"FEAGI rate: {current_feagi_rate}Hz -> {new_feagi_rate}Hz"
            )
            
            return response
            
        except HTTPException:
            raise
        except Exception as e:
            self.logger.error(f"Enhanced agent registration failed for {request.agent_id}: {e}")
            raise HTTPException(
                status_code=500,
                detail=f"Registration failed: {str(e)}"
            )
    
    async def register_agent_legacy(
        self,
        request: AgentRegistrationRequest
    ) -> SuccessResponse:
        """
        Legacy agent registration endpoint for backward compatibility.
        
        Automatically converts legacy capabilities to default rate specs.
        """
        try:
            # Convert legacy request to enhanced request with default rates
            enhanced_request = self._convert_legacy_to_enhanced(request)
            
            # Process as enhanced registration  
            enhanced_response = await self.register_agent_enhanced(enhanced_request)
            
            # Convert back to simple success response for compatibility
            return SuccessResponse(
                success=enhanced_response.success,
                message=enhanced_response.message,
                details={
                    "agent_id": enhanced_response.agent_id,
                    "approved_capabilities": [cap.value for cap in enhanced_response.approved_capabilities]
                }
            )
            
        except HTTPException:
            raise
        except ValidationError as e:
            # Handle Pydantic validation errors with more detailed messages
            error_details = []
            for error in e.errors():
                field = " -> ".join(str(loc) for loc in error['loc'])
                error_details.append(f"{field}: {error['msg']}")
            
            self.logger.error(
                f"Legacy agent registration validation failed for {request.agent_id}: {'; '.join(error_details)}"
            )
            raise HTTPException(
                status_code=400,
                detail=f"Registration validation failed: {'; '.join(error_details)}"
            )
        except Exception as e:
            self.logger.error(f"Legacy agent registration failed for {request.agent_id}: {e}")
            raise HTTPException(
                status_code=500,
                detail=f"Registration failed: {str(e)}"
            )
    
    def _convert_legacy_to_enhanced(
        self, 
        legacy_request: AgentRegistrationRequest
    ) -> EnhancedAgentRegistrationRequest:
        """Convert legacy registration request to enhanced format with default rates."""
        
        # Infer capability types from legacy capabilities dict
        capability_specs = []
        capabilities = legacy_request.capabilities or {}
        seen_capability_types = set()  # Prevent duplicates
        
        # Default rate mapping for legacy capabilities
        default_rates = {
            "sensory": 10.0,  # 10Hz default for sensory
            "motor": 20.0,    # 20Hz default for motor 
            "visualization": 5.0,  # 5Hz default for visualization
            "neurons_stream": 10.0,  # 10Hz for legacy neurons stream
            "control": 1.0    # 1Hz for control messages
        }
        
        for cap_name, cap_config in capabilities.items():
            # Handle special "sensorimotor" capability that combines sensory and motor
            if cap_name.lower() == "sensorimotor":
                # Split sensorimotor into separate sensory and motor capabilities
                sensory_rate = default_rates["sensory"] 
                motor_rate = default_rates["motor"]
                
                # Extract rates from config if provided
                if isinstance(cap_config, dict):
                    if "sensory_rate_hz" in cap_config:
                        try:
                            sensory_rate = float(cap_config["sensory_rate_hz"])
                        except (ValueError, TypeError):
                            pass
                    if "motor_rate_hz" in cap_config:
                        try:
                            motor_rate = float(cap_config["motor_rate_hz"])
                        except (ValueError, TypeError):
                            pass
                    elif "rate_hz" in cap_config:
                        # Use same rate for both if only general rate specified
                        try:
                            rate = float(cap_config["rate_hz"])
                            sensory_rate = motor_rate = rate
                        except (ValueError, TypeError):
                            pass
                
                # Add sensory capability if not already present
                if CapabilityType.SENSORY not in seen_capability_types:
                    capability_specs.append(
                        CapabilityRateSpec(
                            capability_type=CapabilityType.SENSORY,
                            requested_rate_hz=sensory_rate,
                            required=True,
                            metadata={
                                "source": "sensorimotor_split",
                                "original_config": cap_config if isinstance(cap_config, dict) else None
                            }
                        )
                    )
                    seen_capability_types.add(CapabilityType.SENSORY)
                
                # Add motor capability if not already present  
                if CapabilityType.MOTOR not in seen_capability_types:
                    capability_specs.append(
                        CapabilityRateSpec(
                            capability_type=CapabilityType.MOTOR,
                            requested_rate_hz=motor_rate,
                            required=True,
                            metadata={
                                "source": "sensorimotor_split",
                                "original_config": cap_config if isinstance(cap_config, dict) else None
                            }
                        )
                    )
                    seen_capability_types.add(CapabilityType.MOTOR)
                
                continue  # Skip the normal processing for this capability
            
            # Normal capability processing
            cap_types_to_add = []  # List of (cap_type, rate) tuples
            
            # Try to map to standard capability type(s)
            if cap_name.lower() in ["sensory", "sensor", "input", "sensors"]:
                cap_types_to_add.append((CapabilityType.SENSORY, default_rates["sensory"]))
            elif cap_name.lower() in ["motor", "output", "actuator", "motors", "actuators"]:
                cap_types_to_add.append((CapabilityType.MOTOR, default_rates["motor"]))
            elif cap_name.lower() in ["visualization", "viz", "visual", "display"]:
                cap_types_to_add.append((CapabilityType.VISUALIZATION, default_rates["visualization"]))
            elif cap_name.lower() in ["neurons_stream", "neuron_stream", "neural_stream"]:
                cap_types_to_add.append((CapabilityType.NEURONS_STREAM, default_rates["neurons_stream"]))
            elif cap_name.lower() in ["control", "command", "commands"]:
                cap_types_to_add.append((CapabilityType.CONTROL, default_rates["control"]))
            else:
                # Default to sensory for unknown capabilities
                cap_types_to_add.append((CapabilityType.SENSORY, default_rates["sensory"]))
                self.logger.warning(
                    f"Unknown capability type '{cap_name}' for agent {legacy_request.agent_id}, "
                    f"defaulting to sensory"
                )
            
            # Add each capability type if not already present
            for cap_type, default_rate in cap_types_to_add:
                if cap_type not in seen_capability_types:
                    # Check if capability config specifies a rate
                    final_rate = default_rate
                    if isinstance(cap_config, dict) and "rate_hz" in cap_config:
                        try:
                            final_rate = float(cap_config["rate_hz"])
                        except (ValueError, TypeError):
                            pass
                    
                    capability_specs.append(
                        CapabilityRateSpec(
                            capability_type=cap_type,
                            requested_rate_hz=final_rate,
                            required=True,  # Assume all legacy capabilities are required
                            metadata={
                                "source": f"legacy_{cap_name}",
                                "original_config": cap_config if isinstance(cap_config, dict) else None
                            }
                        )
                    )
                    seen_capability_types.add(cap_type)
        
        # Ensure at least one capability is present
        if not capability_specs:
            # Default to sensory capability if no valid capabilities were found
            capability_specs.append(
                CapabilityRateSpec(
                    capability_type=CapabilityType.SENSORY,
                    requested_rate_hz=default_rates["sensory"],
                    required=True,
                    metadata={
                        "source": "fallback_default",
                        "reason": "No valid capabilities found in legacy request"
                    }
                )
            )
            self.logger.warning(
                f"No valid capabilities found for agent {legacy_request.agent_id}, "
                f"added default sensory capability"
            )
        
        # Create enhanced request
        enhanced_request = EnhancedAgentRegistrationRequest(
            agent_type=legacy_request.agent_type,
            agent_id=legacy_request.agent_id,
            agent_data_port=legacy_request.agent_data_port,
            agent_version=legacy_request.agent_version,
            controller_version=legacy_request.controller_version,
            agent_ip=legacy_request.agent_ip,
            metadata=legacy_request.metadata,
            feagi_rate_request=None,  # No FEAGI rate request for legacy
            capability_rates=capability_specs,
            capabilities=legacy_request.capabilities  # Keep original for reference
        )
        
        return enhanced_request
    
    def _normalize_capability_specs(
        self,
        request: EnhancedAgentRegistrationRequest
    ) -> List[CapabilityRateSpec]:
        """Normalize capability specifications from enhanced request."""
        
        if request.capability_rates:
            return request.capability_rates
        elif request.capabilities:
            # Convert legacy capabilities to specs
            legacy_request = AgentRegistrationRequest(
                agent_type=request.agent_type,
                agent_id=request.agent_id,
                agent_data_port=request.agent_data_port,
                agent_version=request.agent_version,
                controller_version=request.controller_version,
                capabilities=request.capabilities,
                agent_ip=request.agent_ip,
                metadata=request.metadata
            )
            enhanced = self._convert_legacy_to_enhanced(legacy_request)
            return enhanced.capability_rates
        else:
            return []
    
    async def _process_feagi_rate_request(
        self,
        rate_request: FeagiRateRequest,
        current_feagi_rate: float,
        state_manager: FeagiStateManager
    ) -> tuple[bool, float, Optional[str]]:
        """
        Process a request to adjust FEAGI global rate.
        
        Returns:
            (approved, new_rate, rejection_reason)
        """
        requested_rate = rate_request.requested_feagi_rate_hz
        
        # Validate rate request
        if requested_rate == current_feagi_rate:
            return True, current_feagi_rate, None
        
        # Rate increase validation (more strict)
        if requested_rate > current_feagi_rate:
            # Check if the increase is reasonable
            rate_increase_factor = requested_rate / current_feagi_rate
            if rate_increase_factor > 2.0:
                return False, current_feagi_rate, f"Rate increase factor {rate_increase_factor:.1f}x exceeds limit of 2.0x"
            
            # Check system capacity (placeholder - would integrate with resource monitoring)
            # For now, allow reasonable increases
            if requested_rate > 50.0:
                return False, current_feagi_rate, f"Requested rate {requested_rate}Hz exceeds system maximum of 50Hz"
        
        # Rate decrease is generally safe
        elif requested_rate < current_feagi_rate:
            if requested_rate < 0.1:
                return False, current_feagi_rate, f"Requested rate {requested_rate}Hz below minimum of 0.1Hz"
        
        # Apply the rate change
        try:
            state_manager.set_burst_frequency(requested_rate)
            self.logger.info(
                f"FEAGI rate changed: {current_feagi_rate}Hz -> {requested_rate}Hz "
                f"(justification: {rate_request.justification or 'none provided'})"
            )
            return True, requested_rate, None
        except Exception as e:
            self.logger.error(f"Failed to set FEAGI rate to {requested_rate}Hz: {e}")
            return False, current_feagi_rate, f"Failed to apply rate change: {str(e)}"
    
    def _build_legacy_capabilities_dict(
        self,
        approved_configs: List[AgentCapabilityRate]
    ) -> Dict[str, Any]:
        """Build legacy capabilities dictionary from approved rate configurations."""
        legacy_capabilities = {}
        
        for config in approved_configs:
            legacy_capabilities[config.capability_type.value] = {
                "enabled": True,
                "rate_hz": config.approved_rate_hz,
                "poll_interval_ms": 1000.0 / config.approved_rate_hz
            }
        
        return legacy_capabilities
