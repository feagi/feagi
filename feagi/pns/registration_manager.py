"""
Registration Manager - Central Authority for Agent Registration and FQ Sampler Coordination

This module implements the Registration Manager, which serves as the single entry point
for all agent registration, deregistration, and capability vetting. It integrates with
the State Manager and Process Manager to provide automatic FQ sampler coordination.

Architecture Flow:
1. Agent connects (ZMQ/Direct) → Registration Manager
2. Registration Manager vets agent properties and checks FEAGI readiness
3. Registration Manager updates State Manager with agent state
4. State Manager changes trigger Process Manager to enable/disable FQ samplers
5. System provides coordinated data flow based on agent capabilities

This design supports both normal ZMQ-based connections and embedded mode direct calls.
"""

import logging
import threading
from datetime import datetime, timezone
from typing import Any, Callable, Dict, Optional, Set

try:
    from feagi.config.toml_loader import get_agent_config, load_feagi_config
except ImportError:
    # Handle cases where configuration might not be available
    load_feagi_config = None
    get_agent_config = None

logger = logging.getLogger(__name__)


class AgentRegistrationRequest:
    """Data structure for agent registration requests."""

    def __init__(
        self,
        agent_id: str,
        agent_type: str = "",
        capabilities: Optional[Dict[str, Any]] = None,
        agent_data_port: Optional[int] = None,
        agent_version: str = "",
        controller_version: str = "",
        agent_ip: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ):
        self.agent_id = agent_id
        self.agent_type = agent_type
        self.capabilities = capabilities or {}
        self.agent_data_port = agent_data_port
        self.agent_version = agent_version
        self.controller_version = controller_version
        
        # Load agent_ip from configuration if not provided
        if agent_ip is None:
            try:
                if load_feagi_config and get_agent_config:
                    config = load_feagi_config()
                    agent_config = get_agent_config(config)
                    agent_ip = agent_config.default_host
                else:
                    agent_ip = "127.0.0.1"  # @architecture:acceptable - emergency fallback
            except Exception as e:
                logger.warning(f"Could not load agent configuration, using fallback: {e}")
                agent_ip = "127.0.0.1"  # @architecture:acceptable - emergency fallback
        
        self.agent_ip = agent_ip
        self.metadata = metadata or {}


class AgentRegistrationResponse:
    """Response structure for agent registration."""

    def __init__(
        self,
        success: bool,
        message: str,
        agent_id: str,
        fq_samplers_enabled: Optional[Dict[str, bool]] = None,
        error_code: Optional[str] = None,
    ):
        self.success = success
        self.message = message
        self.agent_id = agent_id
        self.fq_samplers_enabled = fq_samplers_enabled or {}
        self.error_code = error_code


class RegistrationManager:
    """
    Central Registration Manager for agent lifecycle and FQ sampler coordination.

    This class serves as the single authority for:
    - Agent registration and deregistration
    - Agent capability vetting
    - FEAGI readiness checks
    - State Manager integration
    - FQ sampler coordination via Process Manager

    Supports both ZMQ-based and embedded mode direct function calls.
    """

    def __init__(self, state_manager=None, process_manager=None):
        """
        Initialize Registration Manager.

        Args:
            state_manager: State Manager instance for persistent state
            process_manager: Process Manager instance for FQ sampler control
        """
        self._lock = threading.RLock()
        self._state_manager = state_manager
        self._process_manager = process_manager

        # Agent registry - will be persisted via State Manager
        self._agents: Dict[str, Dict[str, Any]] = {}

        # Capability tracking for FQ sampler decisions
        self._capability_counts = {"visualization": 0, "motor": 0, "sensory": 0}

        # FQ sampler states
        self._fq_sampler_states = {
            "visualization_enabled": False,
            "motor_enabled": False,
        }

        # Event listeners for state changes
        self._state_change_listeners: Set[Callable] = set()

        # FEAGI readiness checks
        self._feagi_ready = True  # Assume ready by default

        logger.info("🏛️ Registration Manager initialized - ready for agent coordination")

    def register_agent(
        self, request: AgentRegistrationRequest
    ) -> AgentRegistrationResponse:
        """
        Register a new agent in the FEAGI system.

        This method handles agent registration with comprehensive state management,
        FQ sampler coordination, and notification of state changes.

        Args:
            request: AgentRegistrationRequest object containing all agent details

        Returns:
            AgentRegistrationResponse with registration status and FQ sampler info
        """
        with self._lock:
            logger.info(
                f"🤖 AGENT REGISTRATION REQUEST: {request.agent_id} (type: {request.agent_type}) with capabilities: {request.capabilities}"
            )

            try:
                # 1. Validate agent request
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
                        message="FEAGI system not ready for agent registration",
                        agent_id=request.agent_id,
                        error_code="FEAGI_NOT_READY",
                    )

                agent_id = request.agent_id
                is_re_registration = agent_id in self._agents

                # 3. Handle re-registration case
                if is_re_registration:
                    existing_agent = self._agents[agent_id]
                    logger.warning(
                        f"⚠️ Agent '{agent_id}' re-registering - overwriting existing registration"
                    )
                    logger.warning(
                        f"    Previous: type={existing_agent.get('agent_type', 'unknown')}, "
                        f"IP={existing_agent.get('agent_ip', 'unknown')}, "
                        f"capabilities={list(existing_agent.get('capabilities', {}).keys())}"
                    )
                    logger.warning(
                        f"    New: type={request.agent_type}, "
                        f"IP={request.agent_ip}, "
                        f"capabilities={list(request.capabilities.keys())}"
                    )

                    # Subtract old capability counts before overwriting
                    self._update_capability_counts(
                        existing_agent.get("capabilities", {}), increment=False
                    )
                else:
                    logger.info(f"✅ New agent '{agent_id}' registering")

                # 4. Create or update agent entry
                agent_data = {
                    "agent_id": agent_id,
                    "agent_type": request.agent_type,
                    "capabilities": request.capabilities,
                    "agent_data_port": request.agent_data_port,
                    "agent_version": request.agent_version,
                    "controller_version": request.controller_version,
                    "agent_ip": request.agent_ip
                    or "127.0.0.1",  # @architecture:acceptable - emergency fallback
                    "status": "active",
                    "registered_at": datetime.now(timezone.utc).isoformat(),
                    "last_seen": datetime.now(timezone.utc).isoformat(),
                    "metadata": request.metadata or {},
                }

                # 5. Store agent data
                self._agents[agent_id] = agent_data

                # 6. Update capability counts (add new capabilities)
                self._update_capability_counts(request.capabilities, increment=True)

                # 7. Coordinate FQ samplers based on new agent capabilities
                fq_coordination_result = self._coordinate_fq_samplers_for_registration(
                    request.capabilities
                )

                # 8. Update State Manager - call register_agent method
                if self._state_manager:
                    try:
                        self._state_manager.register_agent(
                            agent_id=request.agent_id,
                            agent_type=request.agent_type,
                            capabilities=request.capabilities,
                            agent_data_port=request.agent_data_port,
                            agent_version=request.agent_version,
                            controller_version=request.controller_version,
                            agent_ip=request.agent_ip,
                        )
                        # Also update our comprehensive state
                        self._update_state_manager()
                    except Exception as e:
                        logger.error(
                            f"❌ Error calling state manager register_agent: {e}"
                        )

                # 9. Notify state change listeners
                self._notify_state_change(
                    "agent_registered",
                    {
                        "agent_id": agent_id,
                        "agent_type": request.agent_type,
                        "capabilities": request.capabilities,
                        "is_re_registration": is_re_registration,
                    },
                )

                registration_type = (
                    "re-registered (overwritten)"
                    if is_re_registration
                    else "registered"
                )
                success_message = f"Agent '{agent_id}' {registration_type} successfully"

                logger.info(
                    f"✅ {success_message} - FQ samplers coordinated: {fq_coordination_result}"
                )

                return AgentRegistrationResponse(
                    success=True,
                    message=success_message,
                    agent_id=agent_id,
                    fq_samplers_enabled=fq_coordination_result,
                )

            except Exception as e:
                logger.error(f"❌ Error registering agent '{request.agent_id}': {e}")
                return AgentRegistrationResponse(
                    success=False,
                    message=f"Registration failed: {str(e)}",
                    agent_id=request.agent_id,
                    error_code="REGISTRATION_ERROR",
                )

    def deregister_agent(self, agent_id: str) -> AgentRegistrationResponse:
        """
        Deregister an agent from FEAGI system.

        Args:
            agent_id: ID of agent to deregister

        Returns:
            AgentRegistrationResponse with success status and cleanup info
        """
        with self._lock:
            try:
                # 1. Check if agent exists
                if agent_id not in self._agents:
                    return AgentRegistrationResponse(
                        success=False,
                        message=f"Agent '{agent_id}' not found",
                        agent_id=agent_id,
                        error_code="AGENT_NOT_FOUND",
                    )

                # 2. Get agent data before removal
                agent_data = self._agents[agent_id].copy()
                agent_capabilities = agent_data.get("capabilities", {})

                # 3. Remove agent from registry
                del self._agents[agent_id]

                # 4. Update capability counts
                self._update_capability_counts(agent_capabilities, increment=False)

                # 5. Update State Manager - call deregister_agent method
                if self._state_manager:
                    try:
                        self._state_manager.deregister_agent(agent_id)
                        # Also update our comprehensive state
                        self._update_state_manager()
                    except Exception as e:
                        logger.error(
                            f"❌ Error calling state manager deregister_agent: {e}"
                        )

                # 6. Coordinate FQ samplers based on remaining agents
                fq_coordination_result = (
                    self._coordinate_fq_samplers_for_deregistration(agent_capabilities)
                )

                # 7. Notify listeners of state change
                self._notify_state_change("agent_deregistered", agent_data)

                logger.info(f"✅ Agent '{agent_id}' deregistered successfully")

                return AgentRegistrationResponse(
                    success=True,
                    message=f"Agent '{agent_id}' deregistered successfully",
                    agent_id=agent_id,
                    fq_samplers_enabled=fq_coordination_result,
                )

            except Exception as e:
                logger.error(f"❌ Error deregistering agent '{agent_id}': {e}")
                return AgentRegistrationResponse(
                    success=False,
                    message=f"Deregistration failed: {str(e)}",
                    agent_id=agent_id,
                    error_code="DEREGISTRATION_ERROR",
                )

    def heartbeat_agent(self, agent_id: str) -> bool:
        """
        Update agent heartbeat timestamp.

        Args:
            agent_id: ID of agent sending heartbeat

        Returns:
            True if heartbeat processed successfully, False otherwise
        """
        with self._lock:
            if agent_id in self._agents:
                self._agents[agent_id]["last_seen"] = datetime.now(
                    timezone.utc
                ).isoformat()
                return True
            return False

    def get_agent_properties(self, agent_id: str) -> Optional[Dict[str, Any]]:
        """
        Get properties for a specific agent.

        Args:
            agent_id: ID of agent

        Returns:
            Agent properties dictionary or None if not found
        """
        with self._lock:
            return (
                self._agents.get(agent_id, {}).copy()
                if agent_id in self._agents
                else None
            )

    def list_agents(self) -> Dict[str, Any]:
        """
        Get list of all registered agents.

        Returns:
            Dictionary with agents list and summary statistics
        """
        with self._lock:
            agents_list = []
            for agent_data in self._agents.values():
                agents_list.append(
                    {
                        "agent_id": agent_data["agent_id"],
                        "agent_type": agent_data["agent_type"],
                        "capabilities": list(agent_data["capabilities"].keys()),
                        "status": agent_data["status"],
                        "last_seen": agent_data["last_seen"],
                    }
                )

            return {
                "agents": agents_list,
                "summary": {
                    "total_agents": len(self._agents),
                    "visualization_agents": self._capability_counts["visualization"],
                    "motor_agents": self._capability_counts["motor"],
                    "sensory_agents": self._capability_counts["sensory"],
                },
                "fq_sampler_states": self._fq_sampler_states.copy(),
            }

    def get_fq_sampler_coordination_status(self) -> Dict[str, Any]:
        """
        Get current FQ sampler coordination status.

        Returns:
            Dictionary with FQ sampler states and agent coordination info
        """
        with self._lock:
            # Get agents requiring each type of sampler
            viz_agents = [
                aid
                for aid, adata in self._agents.items()
                if self._has_visualization_capabilities(adata.get("capabilities", {}))
            ]
            motor_agents = [
                aid
                for aid, adata in self._agents.items()
                if self._has_motor_capabilities(adata.get("capabilities", {}))
            ]

            return {
                "visualization_fq_sampler": {
                    "enabled": self._fq_sampler_states["visualization_enabled"],
                    "reason": f"{len(viz_agents)} visualization agent(s) connected",
                    "agents_requiring": viz_agents,
                    "sampling_rate": "30Hz",
                    "port": 5562,
                },
                "motor_fq_sampler": {
                    "enabled": self._fq_sampler_states["motor_enabled"],
                    "reason": f"{len(motor_agents)} motor agent(s) connected",
                    "agents_requiring": motor_agents,
                    "sampling_rate": "100Hz",
                    "port": 5564,
                },
                "coordination_summary": {
                    "total_agents": len(self._agents),
                    "fq_samplers_enabled": sum(
                        [1 for enabled in self._fq_sampler_states.values() if enabled]
                    ),
                    "visualization_agents": len(viz_agents),
                    "motor_agents": len(motor_agents),
                    "last_updated": datetime.now(timezone.utc).isoformat(),
                },
            }

    def register_state_change_listener(self, listener: Callable) -> None:
        """
        Register a listener for agent state changes.

        Args:
            listener: Callable that will be notified of state changes
        """
        self._state_change_listeners.add(listener)

    def unregister_state_change_listener(self, listener: Callable) -> None:
        """
        Unregister a state change listener.

        Args:
            listener: Listener to remove
        """
        self._state_change_listeners.discard(listener)

    # --- EMBEDDED MODE INTERFACE ---

    def register_agent_direct(
        self,
        agent_id: str,
        agent_type: str = "",
        capabilities: Optional[Dict[str, Any]] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> AgentRegistrationResponse:
        """
        Direct function call interface for embedded mode agent registration.

        This method provides a simplified interface for embedded mode where
        agents are spawned as direct processes rather than ZMQ connections.

        Args:
            agent_id: Unique identifier for the agent
            agent_type: Type/category of the agent
            capabilities: Dictionary of agent capabilities
            metadata: Additional agent metadata

        Returns:
            AgentRegistrationResponse with registration result
        """
        request = AgentRegistrationRequest(
            agent_id=agent_id,
            agent_type=agent_type,
            capabilities=capabilities or {},
            metadata=metadata or {},
        )

        logger.info(f"🔧 [EMBEDDED MODE] Direct registration for agent '{agent_id}'")
        return self.register_agent(request)

    def deregister_agent_direct(self, agent_id: str) -> AgentRegistrationResponse:
        """
        Direct function call interface for embedded mode agent deregistration.

        Args:
            agent_id: ID of agent to deregister

        Returns:
            AgentRegistrationResponse with deregistration result
        """
        logger.info(f"🔧 [EMBEDDED MODE] Direct deregistration for agent '{agent_id}'")
        return self.deregister_agent(agent_id)

    # --- PRIVATE METHODS ---

    def _validate_agent_request(
        self, request: AgentRegistrationRequest
    ) -> Dict[str, Any]:
        """Validate agent registration request."""
        if not request.agent_id:
            return {
                "valid": False,
                "message": "Agent ID is required",
                "error_code": "MISSING_AGENT_ID",
            }

        if not request.capabilities:
            return {
                "valid": False,
                "message": "At least one capability must be specified",
                "error_code": "MISSING_CAPABILITIES",
            }

        # Validate capability format
        if not isinstance(request.capabilities, dict):
            return {
                "valid": False,
                "message": "Capabilities must be a dictionary",
                "error_code": "INVALID_CAPABILITIES_FORMAT",
            }

        return {"valid": True}

    def _check_feagi_readiness(self) -> bool:
        """Check if FEAGI system is ready for agent registration."""
        # TODO: Implement actual readiness checks
        # - Check if burst engine is running
        # - Check if state manager is available
        # - Check if process manager is available
        # - Check if core services are operational
        return self._feagi_ready

    def _update_capability_counts(
        self, capabilities: Dict[str, Any], increment: bool
    ) -> None:
        """Update capability counters for FQ sampler coordination."""
        delta = 1 if increment else -1

        if self._has_visualization_capabilities(capabilities):
            self._capability_counts["visualization"] = max(
                0, self._capability_counts["visualization"] + delta
            )

        if self._has_motor_capabilities(capabilities):
            self._capability_counts["motor"] = max(
                0, self._capability_counts["motor"] + delta
            )

        if self._has_sensory_capabilities(capabilities):
            self._capability_counts["sensory"] = max(
                0, self._capability_counts["sensory"] + delta
            )

    def _has_visualization_capabilities(self, capabilities: Dict[str, Any]) -> bool:
        """Check if agent has visualization capabilities."""
        return (
            capabilities.get("visualization", False)
            or capabilities.get("3d_visualization", False)
            or capabilities.get("brain_visualizer", False)
            or capabilities.get("neural_visualization", False)
        )

    def _has_motor_capabilities(self, capabilities: Dict[str, Any]) -> bool:
        """
        Check if agent has actual motor control capabilities (not just visualization).

        Brain visualizers that need to see motor data should NOT trigger motor FQ sampler creation.
        Only agents that actually control motors should trigger motor FQ samplers.
        """
        # Explicit motor control capabilities
        has_motor_control = (
            capabilities.get("motor", False)
            or capabilities.get("motor_control", False)
            or capabilities.get("actuator_control", False)
            or capabilities.get("robot_control", False)
        )

        # Check for output capabilities, but exclude visualization-only agents
        has_output = capabilities.get("output", False)
        is_visualizer = (
            capabilities.get("visualization", False)
            or capabilities.get("3d_visualization", False)
            or capabilities.get("brain_visualizer", False)
            or capabilities.get("neural_visualization", False)
        )

        # Only count output as motor capability if it's not a visualization agent
        has_output_control = has_output and not is_visualizer

        # Sensorimotor should only count as motor capability if it explicitly includes motor control
        # Brain visualizers might have sensorimotor for displaying both sensory and motor data
        has_sensorimotor_control = (
            capabilities.get("sensorimotor", False)
            and not is_visualizer
            and (
                capabilities.get("motor", False)
                or capabilities.get("motor_control", False)
            )
        )

        return has_motor_control or has_output_control or has_sensorimotor_control

    def _has_sensory_capabilities(self, capabilities: Dict[str, Any]) -> bool:
        """Check if agent has sensory capabilities."""
        return (
            capabilities.get("sensory", False)
            or capabilities.get("input", False)
            or capabilities.get("sensorimotor", False)
            or capabilities.get("sensory_input", False)
        )

    def _coordinate_fq_samplers_for_registration(
        self, capabilities: Dict[str, Any]
    ) -> Dict[str, bool]:
        """Coordinate FQ samplers when agent registers."""
        fq_results = {}

        if not self._process_manager:
            logger.warning(
                "⚠️ Process Manager not available - FQ samplers not coordinated"
            )
            return fq_results

        try:
            # Handle visualization FQ sampler
            if self._has_visualization_capabilities(capabilities):
                if not self._fq_sampler_states["visualization_enabled"]:
                    # Get frequency from config or use default
                    viz_frequency = 30.0  # Default visualization frequency
                    if hasattr(self._process_manager, "_fq_sampler_config"):
                        viz_frequency = self._process_manager._fq_sampler_config.get(
                            "visualization_frequency", 30.0
                        )
                    
                    # Use minimum of viz_frequency and FEAGI burst frequency
                    # STATE MANAGER is the SINGLE SOURCE OF TRUTH for burst frequency
                    burst_frequency = None
                    original_viz_frequency = viz_frequency
                    
                    try:
                        from feagi.core.state_manager import FeagiStateManager
                        state_manager = FeagiStateManager.instance()
                        burst_frequency = state_manager.get_burst_frequency()
                        
                        if burst_frequency and burst_frequency > 0:
                            viz_frequency = min(viz_frequency, burst_frequency)
                            logger.info(
                                f"🎨 [FREQ-SYNC] STATE MANAGER: Using min(viz={original_viz_frequency}Hz, burst={burst_frequency}Hz) = {viz_frequency}Hz"
                            )
                        else:
                            logger.warning(
                                f"🎨 [FREQ-SYNC] STATE MANAGER: Invalid burst frequency ({burst_frequency}Hz) - using original viz frequency: {viz_frequency}Hz"
                            )
                    except Exception as e:
                        logger.warning(
                            f"🎨 [FREQ-SYNC] STATE MANAGER: Failed to get burst frequency - using original viz frequency: {viz_frequency}Hz. Error: {e}"
                        )

                    success = self._process_manager.create_fq_sampler(
                        "visualization", viz_frequency
                    )
                    if success:
                        self._fq_sampler_states["visualization_enabled"] = True
                        logger.info(
                            "🎨 [ON-DEMAND] Visualization FQ sampler created successfully"
                        )
                    else:
                        logger.error(
                            "🎨 [ERROR] Failed to create visualization FQ sampler"
                        )

                # CRITICAL: Notify ALL existing FQ samplers that visualization client connected
                self._notify_existing_fq_samplers_visualization(True)

                fq_results["visualization"] = self._fq_sampler_states[
                    "visualization_enabled"
                ]

            # Handle motor FQ sampler
            if self._has_motor_capabilities(capabilities):
                logger.info(f"🚗 Agent has motor capabilities: {capabilities}")
                if not self._fq_sampler_states["motor_enabled"]:
                    # Get frequency from config or use default
                    motor_frequency = 100.0  # Default motor frequency
                    if hasattr(self._process_manager, "_fq_sampler_config"):
                        motor_frequency = self._process_manager._fq_sampler_config.get(
                            "motor_frequency", 100.0
                        )

                    success = self._process_manager.create_fq_sampler(
                        "opu", motor_frequency
                    )
                    if success:
                        self._fq_sampler_states["motor_enabled"] = True
                        logger.info(
                            "🚗 [ON-DEMAND] Motor FQ sampler created successfully"
                        )
                    else:
                        logger.error("🚗 [ERROR] Failed to create motor FQ sampler")

                # CRITICAL: Notify ALL existing FQ samplers that motor client connected
                self._notify_existing_fq_samplers_motor(True)

                fq_results["motor"] = self._fq_sampler_states["motor_enabled"]
            else:
                logger.info(f"✅ Agent has NO motor capabilities: {capabilities}")

        except Exception as e:
            logger.error(f"❌ Error coordinating FQ samplers for registration: {e}")

        return fq_results

    def _coordinate_fq_samplers_for_deregistration(
        self, capabilities: Dict[str, Any]
    ) -> Dict[str, bool]:
        """Coordinate FQ samplers when agent deregisters."""
        fq_results = {}

        if not self._process_manager:
            logger.warning(
                "⚠️ Process Manager not available - FQ samplers not coordinated"
            )
            return fq_results

        try:
            # Handle visualization FQ sampler
            if self._has_visualization_capabilities(capabilities):
                if (
                    self._capability_counts["visualization"] == 0
                    and self._fq_sampler_states["visualization_enabled"]
                ):
                    self._process_manager.disable_fq_sampler("visualization")
                    self._fq_sampler_states["visualization_enabled"] = False
                    logger.info(
                        "🎨 Visualization FQ sampler disabled - no visualization agents remain"
                    )
                fq_results["visualization"] = self._fq_sampler_states[
                    "visualization_enabled"
                ]

            # Handle motor FQ sampler
            if self._has_motor_capabilities(capabilities):
                if (
                    self._capability_counts["motor"] == 0
                    and self._fq_sampler_states["motor_enabled"]
                ):
                    self._process_manager.disable_fq_sampler("motor")
                    self._fq_sampler_states["motor_enabled"] = False
                    logger.info("🚗 Motor FQ sampler disabled - no motor agents remain")
                fq_results["motor"] = self._fq_sampler_states["motor_enabled"]

        except Exception as e:
            logger.error(f"❌ Error coordinating FQ samplers for deregistration: {e}")

        return fq_results

    def _notify_existing_fq_samplers_visualization(self, has_clients: bool) -> None:
        """
        Notify ALL existing FQ samplers about visualization client status.

        This is critical because there may be FQ samplers that were created during
        startup or by other processes that don't know about client connections.

        Args:
            has_clients: True if visualization clients are connected, False otherwise
        """
        logger.info(
            f"Notifying all FQ samplers: Visualization clients connected = {has_clients}"
        )

        # Notify Process Manager's FQ samplers
        if self._process_manager:
            if hasattr(self._process_manager, "_viz_fq_sampler"):
                try:
                    self._process_manager._viz_fq_sampler.set_visualization_subscribers(
                        has_clients
                    )
                    logger.info(
                        f"🎨 Notified Process Manager visualization FQ sampler: {has_clients}"
                    )
                except Exception as e:
                    logger.error(f"Error notifying Process Manager viz FQ sampler: {e}")

        # Notify ZMQ Server's FQ samplers
        zmq_server = None
        if self._process_manager:
            zmq_server = self._process_manager.get_zmq_server()

        if zmq_server:
            # Notify visualization stream FQ samplers
            if (
                hasattr(zmq_server, "visualization_stream")
                and zmq_server.visualization_stream
            ):
                if (
                    hasattr(zmq_server.visualization_stream, "fq_sampler")
                    and zmq_server.visualization_stream.fq_sampler
                ):
                    try:
                        zmq_server.visualization_stream.fq_sampler.set_visualization_subscribers(
                            has_clients
                        )
                        logger.info(
                            f"📺 Notified ZMQ visualization stream FQ sampler: {has_clients}"
                        )
                    except Exception as e:
                        logger.error(f"Error notifying ZMQ viz stream FQ sampler: {e}")

    def _notify_existing_fq_samplers_motor(self, has_clients: bool) -> None:
        """
        Notify ALL existing FQ samplers about motor client status.

        Args:
            has_clients: True if motor clients are connected, False otherwise
        """
        logger.info(
            f"🔔 NOTIFYING ALL FQ SAMPLERS: Motor clients connected = {has_clients}"
        )

        # Notify Process Manager's FQ samplers
        if self._process_manager:
            if (
                hasattr(self._process_manager, "_motor_fq_sampler")
                and self._process_manager._motor_fq_sampler
            ):
                try:
                    self._process_manager._motor_fq_sampler.set_motor_subscribers(
                        has_clients
                    )
                    logger.info(
                        f"🚗 Notified Process Manager motor FQ sampler: {has_clients}"
                    )
                except Exception as e:
                    logger.error(
                        f"Error notifying Process Manager motor FQ sampler: {e}"
                    )

    def _update_state_manager(self) -> None:
        """Update State Manager with current agent registry."""
        if self._state_manager:
            try:
                # Update agent count in the state manager
                total_agents = len(self._agents)
                self._state_manager.set_agent_count(total_agents)

                # Note: Registration Manager maintains its own agent registry internally
                # The state manager only tracks high-level counts and states
                logger.debug(
                    f"🔄 State Manager updated: {total_agents} agents registered"
                )
            except Exception as e:
                logger.error(f"❌ Error updating State Manager: {e}")

    def _notify_state_change(self, event_type: str, data: Dict[str, Any]) -> None:
        """Notify registered listeners of state changes."""
        for listener in self._state_change_listeners:
            try:
                listener(event_type, data)
            except Exception as e:
                logger.error(f"❌ Error notifying state change listener: {e}")


# --- GLOBAL INSTANCE ---

_registration_manager_instance: Optional[RegistrationManager] = None
_registration_manager_lock = threading.Lock()


def get_registration_manager() -> Optional[RegistrationManager]:
    """Get the global Registration Manager instance."""
    global _registration_manager_instance
    return _registration_manager_instance


def set_registration_manager(manager: Optional[RegistrationManager]) -> None:
    """
    Set the global Registration Manager instance.

    Args:
        manager: Registration Manager instance to set, or None to clear
    """
    global _registration_manager_instance

    with _registration_manager_lock:
        _registration_manager_instance = manager
        if manager:
            logger.info("🏛️ Registration Manager set as global instance")
        else:
            logger.info("🔄 Registration Manager cleared")


def create_registration_manager(
    state_manager=None, process_manager=None
) -> RegistrationManager:
    """
    Create and initialize the global Registration Manager instance.

    Args:
        state_manager: State Manager instance
        process_manager: Process Manager instance

    Returns:
        RegistrationManager instance
    """
    global _registration_manager_instance

    with _registration_manager_lock:
        if _registration_manager_instance is not None:
            logger.warning(
                "🏛️ Registration Manager already exists - returning existing instance"
            )
            return _registration_manager_instance

        _registration_manager_instance = RegistrationManager(
            state_manager=state_manager, process_manager=process_manager
        )

        logger.info("🏛️ Global Registration Manager created and initialized")
        return _registration_manager_instance


def reset_registration_manager() -> None:
    """Reset the global Registration Manager instance (for testing/cleanup)."""
    global _registration_manager_instance

    with _registration_manager_lock:
        _registration_manager_instance = None
        logger.info("🏛️ Registration Manager reset")
