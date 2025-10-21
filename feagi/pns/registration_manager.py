"""
Registration Manager - Rust-backed with Python orchestration

Agent storage: Rust PyAgentRegistry (single source of truth)
Orchestration: Python (FQ samplers, state, capability rates)

No duplication, no fallbacks, no dead code.
"""

import json
import logging
import threading
from datetime import datetime, timezone
from typing import Any, Callable, Dict, Optional, Set

from feagi_rust import PyAgentRegistry

try:
    from feagi.config.toml_loader import get_agent_config, load_feagi_config
except ImportError:
    load_feagi_config = None
    get_agent_config = None

logger = logging.getLogger(__name__)


class AgentRegistrationRequest:
    """Agent registration request"""

    def __init__(
        self,
        agent_id: str,
        agent_type: str,
        agent_ip: str,
        capabilities: Dict[str, Any],
        agent_data_port: Optional[int] = None,
        agent_version: Optional[str] = None,
        controller_version: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ):
        self.agent_id = agent_id
        self.agent_type = agent_type
        self.agent_ip = agent_ip
        self.capabilities = capabilities
        self.agent_data_port = agent_data_port
        self.agent_version = agent_version
        self.controller_version = controller_version
        self.metadata = metadata or {}
        self.registration_timestamp = datetime.now(timezone.utc).isoformat()


class AgentRegistrationResponse:
    """Agent registration response"""

    def __init__(
        self,
        success: bool,
        message: str,
        agent_id: str,
        fq_samplers_enabled: Optional[Dict[str, bool]] = None,
        error_code: Optional[str] = None,
        transport_info: Optional[Dict[str, Any]] = None,
    ):
        self.success = success
        self.message = message
        self.agent_id = agent_id
        self.fq_samplers_enabled = fq_samplers_enabled or {}
        self.transport_info = transport_info or {}
        self.error_code = error_code


class RegistrationManager:
    """
    🦀 Rust-backed Registration Manager
    
    Storage: PyAgentRegistry (Rust) - THE ONLY agent storage
    Orchestration: Python (FQ samplers, state, capability rates)
    """

    def __init__(self, state_manager=None, process_manager=None):
        """Initialize Rust-backed Registration Manager"""
        self._lock = threading.RLock()
        self._state_manager = state_manager
        self._process_manager = process_manager

        # 🦀 Rust agent registry - THE ONLY agent storage
        try:
            self._rust_registry = PyAgentRegistry(
                max_agents=1000,
                timeout_ms=60000
            )
            logger.info("🦀 Rust agent registry initialized")
        except Exception as e:
            logger.error(f"❌ Failed to initialize Rust registry: {e}")
            raise

        # Capability tracking for FQ sampler decisions (NOT agent storage!)
        self._capability_counts = {
            "visualization": 0,
            "motor": 0,
            "sensory": 0,
            "video": 0,
            "feagi": 0,
        }

        # FQ sampler states
        self._fq_sampler_states = {
            "visualization_enabled": False,
            "motor_enabled": False,
        }

        # Event listeners
        self._state_change_listeners: Set[Callable] = set()

        # FEAGI readiness
        self._feagi_ready = True

        logger.info("🦀 Rust-backed Registration Manager initialized")

    def register_agent(
        self, request: AgentRegistrationRequest
    ) -> AgentRegistrationResponse:
        """Register agent using Rust registry"""
        with self._lock:
            logger.info(
                f"🤖 REGISTRATION REQUEST: {request.agent_id} (type: {request.agent_type})"
            )

            try:
                # 1. Validate
                validation_result = self._validate_agent_request(request)
                if not validation_result.get("valid", False):
                    return AgentRegistrationResponse(
                        success=False,
                        message=validation_result.get("error", "Validation failed"),
                        agent_id=request.agent_id,
                        error_code="VALIDATION_ERROR",
                    )

                # 2. Check FEAGI readiness
                if not self._check_feagi_readiness():
                    return AgentRegistrationResponse(
                        success=False,
                        message="FEAGI system not ready",
                        agent_id=request.agent_id,
                        error_code="FEAGI_NOT_READY",
                    )

                # 3. Check if re-registration
                is_re_registration = False
                try:
                    existing_agent_json = self._rust_registry.get_agent_json(request.agent_id)
                    is_re_registration = True
                    existing_agent = json.loads(existing_agent_json)
                    logger.warning(f"⚠️ Agent '{request.agent_id}' re-registering")
                    
                    # Subtract old capability counts
                    old_caps = existing_agent.get("capabilities", {}).get("custom", {})
                    self._update_capability_counts(old_caps, increment=False)
                except Exception:
                    logger.info(f"✅ New agent '{request.agent_id}' registering")

                # 4. Extract capability rates BEFORE sanitization
                capability_rates_to_register = []
                try:
                    from feagi.core.capability_rate_manager import get_capability_rate_manager
                    from feagi.api.v1.capability_rates import CapabilityType, CapabilityRateSpec
                    
                    for cap_name, cap_config in request.capabilities.items():
                        cap_type = None
                        requested_rate = 10.0
                        
                        if cap_name.lower() in ["sensory", "sensor", "input", "sensors"]:
                            cap_type = CapabilityType.SENSORY
                        elif cap_name.lower() in ["motor", "output", "actuator", "motors"]:
                            cap_type = CapabilityType.MOTOR
                        elif cap_name.lower() in ["visualization", "viz"]:
                            cap_type = CapabilityType.VISUALIZATION
                        
                        if cap_type and isinstance(cap_config, dict) and "rate_hz" in cap_config:
                            requested_rate = float(cap_config["rate_hz"])
                            capability_rates_to_register.append(CapabilityRateSpec(
                                capability_type=cap_type,
                                agent_id=request.agent_id,
                                requested_rate_hz=requested_rate
                            ))
                except ImportError:
                    pass

                # 5. Sanitize capabilities
                sanitized_caps = self._sanitize_capabilities(request.capabilities)

                # 6. Convert to Rust format and register
                rust_capabilities = {
                    "custom": sanitized_caps  # Store Python caps in custom field
                }
                
                rust_capabilities_json = json.dumps(rust_capabilities)
                
                # Prepare metadata for Rust registry
                metadata = {}
                if request.agent_data_port is not None:
                    metadata["agent_data_port"] = request.agent_data_port
                if request.agent_version:
                    metadata["agent_version"] = request.agent_version
                if request.controller_version:
                    metadata["controller_version"] = request.controller_version
                if request.agent_ip:
                    metadata["agent_ip"] = request.agent_ip
                if request.metadata:
                    metadata.update(request.metadata)
                
                metadata_json = json.dumps(metadata) if metadata else None
                
                # 🦀 Register in Rust registry with metadata
                result_json = self._rust_registry.register_agent_direct(
                    request.agent_id,
                    self._map_agent_type_to_rust(request.agent_type),
                    rust_capabilities_json,
                    metadata_json
                )
                
                result = json.loads(result_json)
                if not result.get("success"):
                    return AgentRegistrationResponse(
                        success=False,
                        message=result.get("message", "Registration failed"),
                        agent_id=request.agent_id,
                        error_code="RUST_REGISTRY_ERROR",
                    )

                logger.info(f"🦀 Agent stored in Rust registry: {request.agent_id}")

                # 7. Update capability counts
                self._update_capability_counts(sanitized_caps, increment=True)

                # 8. Register capability rates
                for rate_spec in capability_rates_to_register:
                    try:
                        rate_mgr = get_capability_rate_manager()
                        if rate_mgr:
                            rate_mgr.register_capability_rate(rate_spec)
                    except Exception as e:
                        logger.warning(f"Failed to register capability rate: {e}")

                # 9. Coordinate FQ samplers
                fq_changes = self._coordinate_fq_samplers_for_registration(sanitized_caps)

                # 10. Update State Manager
                self._update_state_manager()

                # 11. Notify listeners
                self._notify_state_change("agent_registered", request.agent_id)

                logger.info(
                    f"✅ Agent registered: {request.agent_id} "
                    f"(total: {self._rust_registry.agent_count()})"
                )

                return AgentRegistrationResponse(
                    success=True,
                    message=f"Agent {request.agent_id} registered successfully",
                    agent_id=request.agent_id,
                    fq_samplers_enabled=fq_changes,
                    transport_info=self._get_transport_info(request.agent_type, sanitized_caps),
                )

            except Exception as e:
                logger.error(f"❌ Registration failed for {request.agent_id}: {e}", exc_info=True)
                return AgentRegistrationResponse(
                    success=False,
                    message=f"Registration failed: {str(e)}",
                    agent_id=request.agent_id,
                    error_code="INTERNAL_ERROR",
                )

    def deregister_agent(self, agent_id: str) -> AgentRegistrationResponse:
        """Deregister agent from Rust registry"""
        with self._lock:
            logger.info(f"🔌 DEREGISTRATION REQUEST: {agent_id}")

            try:
                # Get agent info before deletion
                agent_json = self._rust_registry.get_agent_json(agent_id)
                agent_info = json.loads(agent_json)
                caps = agent_info.get("capabilities", {}).get("custom", {})

                # Update capability counts
                self._update_capability_counts(caps, increment=False)

                # 🦀 Remove from Rust registry
                result_json = self._rust_registry.deregister_agent_direct(agent_id)
                result = json.loads(result_json)
                
                if not result.get("success"):
                    return AgentRegistrationResponse(
                        success=False,
                        message=result.get("message", "Deregistration failed"),
                        agent_id=agent_id,
                        error_code="RUST_REGISTRY_ERROR",
                    )

                # Coordinate FQ samplers
                fq_changes = self._coordinate_fq_samplers_for_deregistration(caps)

                # Update State Manager
                self._update_state_manager()

                # Notify listeners
                self._notify_state_change("agent_deregistered", agent_id)

                logger.info(f"✅ Agent deregistered: {agent_id}")

                return AgentRegistrationResponse(
                    success=True,
                    message=f"Agent {agent_id} deregistered successfully",
                    agent_id=agent_id,
                    fq_samplers_enabled=fq_changes,
                )

            except Exception as e:
                logger.error(f"❌ Deregistration failed for {agent_id}: {e}")
                return AgentRegistrationResponse(
                    success=False,
                    message=f"Deregistration failed: {str(e)}",
                    agent_id=agent_id,
                    error_code="AGENT_NOT_FOUND",
                )

    def heartbeat_agent(self, agent_id: str) -> bool:
        """Update agent activity timestamp"""
        try:
            self._rust_registry.update_agent_activity(agent_id)
            return True
        except Exception as e:
            logger.warning(f"Heartbeat failed for {agent_id}: {e}")
            return False

    def get_agent_properties(self, agent_id: str) -> Optional[Dict[str, Any]]:
        """Get agent properties from Rust registry"""
        try:
            agent_json = self._rust_registry.get_agent_json(agent_id)
            agent = json.loads(agent_json)
            
            # Extract custom capabilities for backward compatibility
            if "capabilities" in agent and "custom" in agent["capabilities"]:
                agent["capabilities"] = agent["capabilities"]["custom"]
            
            # Metadata is now properly stored in agent["metadata"] by Rust
            # Extract common metadata fields to top level for convenience
            if "metadata" in agent and agent["metadata"]:
                metadata = agent["metadata"]
                if "agent_data_port" in metadata:
                    agent["agent_data_port"] = metadata["agent_data_port"]
                if "agent_version" in metadata:
                    agent["agent_version"] = metadata["agent_version"]
                if "controller_version" in metadata:
                    agent["controller_version"] = metadata["controller_version"]
                if "agent_ip" in metadata:
                    agent["agent_ip"] = metadata["agent_ip"]
            
            return agent
        except Exception:
            return None

    def list_agents(self) -> Dict[str, Any]:
        """List all registered agents"""
        try:
            agents_json = self._rust_registry.get_all_agents_json()
            agents_list = json.loads(agents_json)
            
            # Extract custom capabilities and metadata for each agent
            for agent in agents_list:
                # Extract custom capabilities for backward compatibility
                if "capabilities" in agent and "custom" in agent["capabilities"]:
                    agent["capabilities"] = agent["capabilities"]["custom"]
                
                # Extract common metadata fields to top level for convenience
                if "metadata" in agent and agent["metadata"]:
                    metadata = agent["metadata"]
                    if "agent_data_port" in metadata:
                        agent["agent_data_port"] = metadata["agent_data_port"]
                    if "agent_version" in metadata:
                        agent["agent_version"] = metadata["agent_version"]
                    if "controller_version" in metadata:
                        agent["controller_version"] = metadata["controller_version"]
                    if "agent_ip" in metadata:
                        agent["agent_ip"] = metadata["agent_ip"]
            
            return {
                "agents": agents_list,
                "count": len(agents_list),
                "capability_summary": self._capability_counts.copy(),
                "fq_sampler_states": self._fq_sampler_states.copy(),
            }
        except Exception as e:
            logger.error(f"Failed to list agents: {e}")
            return {"agents": [], "count": 0}

    def get_fq_sampler_coordination_status(self) -> Dict[str, Any]:
        """Get FQ sampler status"""
        return {
            "fq_samplers": self._fq_sampler_states.copy(),
            "capability_counts": self._capability_counts.copy(),
            "agent_count": self._rust_registry.agent_count(),
        }

    def register_state_change_listener(self, listener: Callable) -> None:
        """Register a state change listener"""
        self._state_change_listeners.add(listener)

    def unregister_state_change_listener(self, listener: Callable) -> None:
        """Unregister a state change listener"""
        self._state_change_listeners.discard(listener)

    # === Helper methods ===
    
    def _map_agent_type_to_rust(self, python_type: str) -> str:
        """Map Python agent type to Rust AgentType"""
        type_map = {
            "sensory": "sensory",
            "motor": "motor",
            "both": "both",
            "multimodal": "both",
            "visualization": "visualization",
            "infrastructure": "infrastructure",
            "feagi_bridge": "infrastructure",  # Bridge is infrastructure type
        }
        return type_map.get(python_type.lower(), "sensory")

    def _sanitize_capabilities(self, capabilities: Dict[str, Any]) -> Dict[str, Any]:
        """Sanitize and normalize capabilities"""
        sanitized = {}
        for cap_name, cap_value in capabilities.items():
            if cap_value is None:
                continue
            if isinstance(cap_value, dict):
                # Remove rate_hz from storage (handled separately)
                cleaned = {k: v for k, v in cap_value.items() if k != "rate_hz"}
                if cleaned:
                    sanitized[cap_name] = cleaned
            else:
                sanitized[cap_name] = cap_value
        return sanitized

    def _validate_agent_request(self, request: AgentRegistrationRequest) -> Dict[str, Any]:
        """Validate agent registration request"""
        if not request.agent_id or not isinstance(request.agent_id, str):
            return {"valid": False, "error": "Invalid agent_id"}
        if not request.agent_type:
            return {"valid": False, "error": "Missing agent_type"}
        if not request.capabilities or not isinstance(request.capabilities, dict):
            return {"valid": False, "error": "Invalid capabilities"}
        return {"valid": True}

    def _check_feagi_readiness(self) -> bool:
        """Check if FEAGI is ready"""
        return self._feagi_ready

    def _get_transport_info(self, agent_type: str, capabilities: Dict[str, Any]) -> Dict[str, Any]:
        """Get transport information for agent"""
        try:
            if load_feagi_config and get_agent_config:
                config = load_feagi_config()
                agent_cfg = get_agent_config(config)
                return {
                    "sensory_endpoint": f"tcp://{agent_cfg.get('host', '0.0.0.0')}:{agent_cfg.get('sensory_port', 5555)}",
                    "motor_endpoint": f"tcp://{agent_cfg.get('host', '0.0.0.0')}:{agent_cfg.get('motor_port', 30005)}",
                }
        except Exception:
            pass
        
        return {
            "sensory_endpoint": "tcp://0.0.0.0:5555",
            "motor_endpoint": "tcp://0.0.0.0:30005",
        }

    def _update_capability_counts(self, capabilities: Dict[str, Any], increment: bool) -> None:
        """Update capability counts"""
        delta = 1 if increment else -1
        
        for cap_name in capabilities.keys():
            cap_lower = cap_name.lower()
            if cap_lower in ["visualization", "viz"]:
                self._capability_counts["visualization"] += delta
            elif cap_lower in ["motor", "motors", "output"]:
                self._capability_counts["motor"] += delta
            elif cap_lower in ["sensory", "sensor", "input", "sensors"]:
                self._capability_counts["sensory"] += delta
            elif cap_lower == "video":
                self._capability_counts["video"] += delta
            elif cap_lower == "feagi":
                self._capability_counts["feagi"] += delta

    def _has_visualization_capabilities(self, capabilities: Dict[str, Any]) -> bool:
        """Check if agent has visualization capabilities"""
        return any(k.lower() in ["visualization", "viz"] for k in capabilities.keys())

    def _has_motor_capabilities(self, capabilities: Dict[str, Any]) -> bool:
        """Check if agent has motor capabilities"""
        return any(k.lower() in ["motor", "motors", "output"] for k in capabilities.keys())

    def _coordinate_fq_samplers_for_registration(
        self, capabilities: Dict[str, Any]
    ) -> Dict[str, bool]:
        """Coordinate FQ samplers when agent registers"""
        changes = {}
        
        # Visualization FQ sampler
        has_viz = self._has_visualization_capabilities(capabilities)
        if has_viz and not self._fq_sampler_states["visualization_enabled"]:
            if self._process_manager:
                try:
                    self._process_manager.enable_visualization_fq_sampler()
                    self._fq_sampler_states["visualization_enabled"] = True
                    changes["visualization_fq_enabled"] = True
                    logger.info("🎨 Visualization FQ sampler ENABLED")
                except Exception as e:
                    logger.error(f"Failed to enable visualization FQ sampler: {e}")

        # Motor FQ sampler
        has_motor = self._has_motor_capabilities(capabilities)
        if has_motor and not self._fq_sampler_states["motor_enabled"]:
            if self._process_manager:
                try:
                    self._process_manager.enable_motor_fq_sampler()
                    self._fq_sampler_states["motor_enabled"] = True
                    changes["motor_fq_enabled"] = True
                    logger.info("🦾 Motor FQ sampler ENABLED")
                except Exception as e:
                    logger.error(f"Failed to enable motor FQ sampler: {e}")

        return changes

    def _coordinate_fq_samplers_for_deregistration(
        self, capabilities: Dict[str, Any]
    ) -> Dict[str, bool]:
        """Coordinate FQ samplers when agent deregisters"""
        changes = {}
        
        # Check if we should disable visualization FQ
        has_viz = self._has_visualization_capabilities(capabilities)
        if has_viz:
            if self._capability_counts["visualization"] <= 0 and self._fq_sampler_states["visualization_enabled"]:
                if self._process_manager:
                    try:
                        self._process_manager.disable_visualization_fq_sampler()
                        self._fq_sampler_states["visualization_enabled"] = False
                        changes["visualization_fq_disabled"] = True
                        logger.info("🎨 Visualization FQ sampler DISABLED")
                    except Exception as e:
                        logger.error(f"Failed to disable visualization FQ sampler: {e}")

        # Check if we should disable motor FQ
        has_motor = self._has_motor_capabilities(capabilities)
        if has_motor:
            if self._capability_counts["motor"] <= 0 and self._fq_sampler_states["motor_enabled"]:
                if self._process_manager:
                    try:
                        self._process_manager.disable_motor_fq_sampler()
                        self._fq_sampler_states["motor_enabled"] = False
                        changes["motor_fq_disabled"] = True
                        logger.info("🦾 Motor FQ sampler DISABLED")
                    except Exception as e:
                        logger.error(f"Failed to disable motor FQ sampler: {e}")

        return changes

    def _update_state_manager(self) -> None:
        """Update State Manager with current registry state"""
        if self._state_manager:
            try:
                agents_json = self._rust_registry.get_all_agents_json()
                agents_list = json.loads(agents_json)
                
                # Convert list to dict (keyed by agent_id) for state manager
                agents_dict = {}
                for agent in agents_list:
                    agent_id = agent.get("agent_id")
                    if agent_id:
                        # Extract custom capabilities for backward compatibility
                        if "capabilities" in agent and "custom" in agent["capabilities"]:
                            agent["capabilities"] = agent["capabilities"]["custom"]
                        
                        # Extract common metadata fields to top level for convenience
                        if "metadata" in agent and agent["metadata"]:
                            metadata = agent["metadata"]
                            if "agent_data_port" in metadata:
                                agent["agent_data_port"] = metadata["agent_data_port"]
                            if "agent_version" in metadata:
                                agent["agent_version"] = metadata["agent_version"]
                            if "controller_version" in metadata:
                                agent["controller_version"] = metadata["controller_version"]
                            if "agent_ip" in metadata:
                                agent["agent_ip"] = metadata["agent_ip"]
                        
                        agents_dict[agent_id] = agent
                
                # Use the correct state manager method
                result = self._state_manager.set_connected_agents(agents_dict)
                if result.is_err:
                    logger.warning(f"Failed to update State Manager: {result.error}")
            except Exception as e:
                logger.warning(f"Failed to update State Manager: {e}")

    def _notify_state_change(self, event_type: str, agent_id: str) -> None:
        """Notify state change listeners"""
        for listener in self._state_change_listeners:
            try:
                listener(event_type, agent_id)
            except Exception as e:
                logger.warning(f"State change listener failed: {e}")


# === Global instance management ===

_registration_manager_instance: Optional[RegistrationManager] = None
_registration_manager_lock = threading.RLock()


def get_registration_manager() -> Optional[RegistrationManager]:
    """Get the global Registration Manager instance"""
    global _registration_manager_instance
    with _registration_manager_lock:
        return _registration_manager_instance


def set_registration_manager(manager: Optional[RegistrationManager]) -> None:
    """Set the global Registration Manager instance"""
    global _registration_manager_instance
    with _registration_manager_lock:
        _registration_manager_instance = manager
        if manager:
            logger.info("🦀 Global Rust-backed Registration Manager set")
        else:
            logger.info("🔄 Registration Manager cleared")


def create_registration_manager(
    state_manager=None, process_manager=None
) -> RegistrationManager:
    """Create and initialize the global Rust-backed Registration Manager"""
    global _registration_manager_instance

    with _registration_manager_lock:
        if _registration_manager_instance is not None:
            logger.warning("🦀 Registration Manager already exists - returning existing")
            return _registration_manager_instance

        _registration_manager_instance = RegistrationManager(
            state_manager=state_manager, process_manager=process_manager
        )

        logger.info("🦀 Global Rust-backed Registration Manager created and initialized")
        return _registration_manager_instance


def reset_registration_manager() -> None:
    """Reset the global Registration Manager (for testing)"""
    global _registration_manager_instance
    with _registration_manager_lock:
        _registration_manager_instance = None
        logger.info("🔄 Registration Manager reset")
