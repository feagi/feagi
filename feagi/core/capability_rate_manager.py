"""
Capability Rate Manager

Manages per-agent, per-capability polling rates and integrates with FEAGI's
state management system to provide capability-aware scheduling.

Key Features:
- Stores agent capability rates in state manager
- Provides rate-based scheduling for SHM/ZMQ polling loops  
- Validates rate changes against FEAGI global rate
- Supports dynamic rate updates during runtime
"""

import time
import threading
from typing import Dict, List, Optional, Set, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum

from feagi.utils.logger import setup_logger
from feagi.core.state_manager import FeagiStateManager, Result
from feagi.api.v1.capability_rates import (
    CapabilityType, 
    CapabilityRateSpec,
    CapabilityRateValidator,
    RateValidationError
)

logger = setup_logger(__name__)


@dataclass
class AgentCapabilityRate:
    """Rate configuration for a single agent capability."""
    agent_id: str
    capability_type: CapabilityType
    approved_rate_hz: float
    last_poll_time_ns: int = 0
    poll_count: int = 0
    created_at: float = field(default_factory=time.time)
    
    @property
    def poll_interval_ns(self) -> int:
        """Get polling interval in nanoseconds."""
        return int(1_000_000_000 / self.approved_rate_hz)
    
    def should_poll_now(self, current_time_ns: int) -> bool:
        """Check if this capability should be polled now based on its rate."""
        if self.last_poll_time_ns == 0:
            return True
            
        elapsed_ns = current_time_ns - self.last_poll_time_ns
        return elapsed_ns >= self.poll_interval_ns
    
    def mark_polled(self, current_time_ns: int) -> None:
        """Mark that this capability was polled at the given time."""
        self.last_poll_time_ns = current_time_ns
        self.poll_count += 1


@dataclass 
class AgentCapabilityRegistry:
    """Registry of all capabilities for a single agent."""
    agent_id: str
    capabilities: Dict[CapabilityType, AgentCapabilityRate] = field(default_factory=dict)
    registered_at: float = field(default_factory=time.time)
    
    def get_capability_rate(self, capability_type: CapabilityType) -> Optional[AgentCapabilityRate]:
        """Get rate config for a specific capability."""
        return self.capabilities.get(capability_type)
    
    def add_capability(self, rate_config: AgentCapabilityRate) -> None:
        """Add or update a capability rate configuration."""
        self.capabilities[rate_config.capability_type] = rate_config
    
    def remove_capability(self, capability_type: CapabilityType) -> bool:
        """Remove a capability rate configuration."""
        return self.capabilities.pop(capability_type, None) is not None
    
    def get_all_capability_types(self) -> Set[CapabilityType]:
        """Get all registered capability types for this agent.""" 
        return set(self.capabilities.keys())


class CapabilityRateManager:
    """Central manager for all agent capability rates and scheduling."""
    
    def __init__(self, state_manager: FeagiStateManager):
        self.state_manager = state_manager
        self._agent_registries: Dict[str, AgentCapabilityRegistry] = {}
        self._lock = threading.RLock()
        
        # Cache current FEAGI rate for validation
        self._cached_feagi_rate_hz: float = 1.0
        self._last_rate_update_time = 0.0
        self._update_feagi_rate_cache()
        
    def _update_feagi_rate_cache(self) -> None:
        """Update cached FEAGI global rate from state manager."""
        try:
            current_rate = self.state_manager.get_burst_frequency()
            if current_rate and current_rate > 0:
                self._cached_feagi_rate_hz = float(current_rate)
                self._last_rate_update_time = time.time()
            else:
                logger.warning(f"Invalid FEAGI rate from state manager: {current_rate}")
        except Exception as e:
            logger.error(f"Failed to get FEAGI rate from state manager: {e}")
    
    def get_current_feagi_rate_hz(self) -> float:
        """Get current FEAGI global rate, updating cache if stale."""
        current_time = time.time()
        if current_time - self._last_rate_update_time > 5.0:  # Refresh every 5 seconds
            self._update_feagi_rate_cache()
        return self._cached_feagi_rate_hz
    
    def register_agent_capabilities(
        self, 
        agent_id: str,
        capability_specs: List[CapabilityRateSpec]
    ) -> Tuple[List[AgentCapabilityRate], Dict[CapabilityType, str]]:
        """
        Register capability rates for an agent with validation.
        
        Args:
            agent_id: Agent identifier
            capability_specs: List of requested capability rates
            
        Returns:
            Tuple of (approved_configs, rejections_dict)
        """
        with self._lock:
            current_feagi_rate = self.get_current_feagi_rate_hz()
            validator = CapabilityRateValidator(current_feagi_rate)
            
            # Validate all capability rates
            rejections = validator.validate_capability_rates(capability_specs)
            
            # Create registry for this agent
            registry = AgentCapabilityRegistry(agent_id)
            approved_configs = []
            
            for spec in capability_specs:
                if spec.capability_type not in rejections:
                    # Approve with potentially optimized rate
                    approved_rate = validator.suggest_optimal_rate(
                        spec.capability_type, 
                        spec.requested_rate_hz
                    )
                    
                    rate_config = AgentCapabilityRate(
                        agent_id=agent_id,
                        capability_type=spec.capability_type,
                        approved_rate_hz=approved_rate
                    )
                    
                    registry.add_capability(rate_config)
                    approved_configs.append(rate_config)
                    
                    logger.info(
                        f"Approved capability {spec.capability_type} for agent {agent_id}: "
                        f"requested={spec.requested_rate_hz}Hz, approved={approved_rate}Hz"
                    )
            
            # Store registry
            self._agent_registries[agent_id] = registry
            
            # Persist to state manager
            self._persist_agent_capabilities(agent_id, registry)
            
            return approved_configs, rejections
    
    def deregister_agent(self, agent_id: str) -> bool:
        """Remove all capability configurations for an agent."""
        with self._lock:
            registry = self._agent_registries.pop(agent_id, None)
            if registry:
                self._remove_agent_capabilities_from_state(agent_id)
                logger.info(f"Deregistered all capabilities for agent {agent_id}")
                return True
            return False
    
    def get_agents_for_capability_polling(
        self, 
        capability_type: CapabilityType,
        current_time_ns: Optional[int] = None
    ) -> List[AgentCapabilityRate]:
        """
        Get all agents whose specified capability should be polled now.
        
        Args:
            capability_type: Type of capability to poll
            current_time_ns: Current time in nanoseconds (default: now)
            
        Returns:
            List of agent capability configs that should be polled
        """
        if current_time_ns is None:
            current_time_ns = time.time_ns()
        
        agents_to_poll = []
        
        with self._lock:
            for registry in self._agent_registries.values():
                rate_config = registry.get_capability_rate(capability_type)
                if rate_config and rate_config.should_poll_now(current_time_ns):
                    agents_to_poll.append(rate_config)
        
        return agents_to_poll
    
    def mark_capability_polled(
        self, 
        agent_id: str, 
        capability_type: CapabilityType,
        current_time_ns: Optional[int] = None
    ) -> bool:
        """Mark that an agent's capability was successfully polled."""
        if current_time_ns is None:
            current_time_ns = time.time_ns()
            
        with self._lock:
            registry = self._agent_registries.get(agent_id)
            if registry:
                rate_config = registry.get_capability_rate(capability_type)
                if rate_config:
                    rate_config.mark_polled(current_time_ns)
                    return True
        return False
    
    def get_agent_capabilities(self, agent_id: str) -> Optional[AgentCapabilityRegistry]:
        """Get all capability configurations for an agent."""
        with self._lock:
            return self._agent_registries.get(agent_id)
    
    def get_all_registered_agents(self) -> List[str]:
        """Get list of all registered agent IDs."""
        with self._lock:
            return list(self._agent_registries.keys())
    
    def get_capability_statistics(self) -> Dict[str, Any]:
        """Get statistics about registered capabilities and polling rates."""
        with self._lock:
            stats = {
                "total_agents": len(self._agent_registries),
                "feagi_rate_hz": self._cached_feagi_rate_hz,
                "capabilities_by_type": {},
                "rate_distribution": {},
                "agents": {}
            }
            
            # Collect capability statistics
            for agent_id, registry in self._agent_registries.items():
                agent_stats = {
                    "capability_count": len(registry.capabilities),
                    "capabilities": {}
                }
                
                for cap_type, rate_config in registry.capabilities.items():
                    cap_type_str = cap_type.value
                    
                    # Count by capability type
                    if cap_type_str not in stats["capabilities_by_type"]:
                        stats["capabilities_by_type"][cap_type_str] = 0
                    stats["capabilities_by_type"][cap_type_str] += 1
                    
                    # Rate distribution
                    rate_hz = rate_config.approved_rate_hz
                    if rate_hz not in stats["rate_distribution"]:
                        stats["rate_distribution"][rate_hz] = 0
                    stats["rate_distribution"][rate_hz] += 1
                    
                    # Agent capability details
                    agent_stats["capabilities"][cap_type_str] = {
                        "rate_hz": rate_hz,
                        "poll_count": rate_config.poll_count,
                        "last_poll_age_ms": (time.time_ns() - rate_config.last_poll_time_ns) / 1_000_000 if rate_config.last_poll_time_ns > 0 else None
                    }
                
                stats["agents"][agent_id] = agent_stats
            
            return stats
    
    def _persist_agent_capabilities(self, agent_id: str, registry: AgentCapabilityRegistry) -> None:
        """Persist agent capability configuration to state manager."""
        try:
            # Convert to serializable format for state manager
            capability_data = {}
            for cap_type, rate_config in registry.capabilities.items():
                capability_data[cap_type.value] = {
                    "approved_rate_hz": rate_config.approved_rate_hz,
                    "poll_interval_ns": rate_config.poll_interval_ns,
                    "created_at": rate_config.created_at
                }
            
            # Store in state manager's connected agents registry
            connected_agents = self.state_manager.get_connected_agents()
            if agent_id in connected_agents:
                connected_agents[agent_id]["capability_rates"] = capability_data
                self.state_manager.set_connected_agents(connected_agents)
            else:
                logger.warning(f"Agent {agent_id} not found in connected agents registry")
                
        except Exception as e:
            logger.error(f"Failed to persist capabilities for agent {agent_id}: {e}")
    
    def _remove_agent_capabilities_from_state(self, agent_id: str) -> None:
        """Remove agent capability configuration from state manager."""
        try:
            connected_agents = self.state_manager.get_connected_agents()
            if agent_id in connected_agents and "capability_rates" in connected_agents[agent_id]:
                del connected_agents[agent_id]["capability_rates"]
                self.state_manager.set_connected_agents(connected_agents)
        except Exception as e:
            logger.error(f"Failed to remove capabilities for agent {agent_id}: {e}")


# Global instance management
_capability_rate_manager_instance: Optional[CapabilityRateManager] = None
_manager_lock = threading.Lock()


def get_capability_rate_manager(state_manager: Optional[FeagiStateManager] = None) -> Optional[CapabilityRateManager]:
    """Get or create the global capability rate manager instance."""
    global _capability_rate_manager_instance
    
    with _manager_lock:
        if _capability_rate_manager_instance is None:
            if state_manager is None:
                try:
                    state_manager = FeagiStateManager.instance()
                except Exception as e:
                    logger.error(f"Could not get state manager for capability rate manager: {e}")
                    return None
            
            _capability_rate_manager_instance = CapabilityRateManager(state_manager)
            logger.info("Created global capability rate manager instance")
        
        return _capability_rate_manager_instance


def reset_capability_rate_manager() -> None:
    """Reset the global capability rate manager (for testing)."""
    global _capability_rate_manager_instance
    with _manager_lock:
        _capability_rate_manager_instance = None
