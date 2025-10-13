"""
Agent Heartbeat Coordinator - Unified Heartbeat Management System

This module coordinates heartbeat functionality across different agent types,
ensuring consistent agent lifecycle management and preventing premature cleanup.
"""

import asyncio
import logging
import time
import threading
from typing import Dict, Optional, Set, Callable
from datetime import datetime, timezone

from pydantic import BaseModel, Field

logger = logging.getLogger(__name__)


class AgentHeartbeatStatus(BaseModel):
    """Status information for an agent's heartbeat."""
    
    agent_id: str
    agent_type: str
    last_heartbeat: datetime
    heartbeat_interval_sec: float = Field(default=30.0, ge=5.0, le=300.0)
    timeout_threshold_sec: float = Field(default=90.0, ge=15.0, le=900.0)
    consecutive_misses: int = Field(default=0, ge=0)
    is_active: bool = True


class HeartbeatCoordinator:
    """
    Coordinates heartbeat functionality across all agent types.
    
    This coordinator manages:
    - Agent heartbeat registration
    - Timeout monitoring
    - Graceful cleanup coordination
    - Integration with Registration Manager
    """
    
    def __init__(self):
        self._lock = threading.RLock()
        self._agent_statuses: Dict[str, AgentHeartbeatStatus] = {}
        self._cleanup_callbacks: Dict[str, Callable[[str], None]] = {}
        self._monitor_task: Optional[asyncio.Task] = None
        self._running = False
        
        logger.info("💗 Agent Heartbeat Coordinator initialized")
    
    def register_agent_heartbeat(
        self, 
        agent_id: str, 
        agent_type: str,
        heartbeat_interval_sec: float = 30.0,
        timeout_threshold_sec: float = 90.0,
        cleanup_callback: Optional[Callable[[str], None]] = None
    ) -> bool:
        """
        Register an agent for heartbeat monitoring.
        
        Args:
            agent_id: Unique agent identifier
            agent_type: Type of agent (brain_visualizer, video_agent, etc.)
            heartbeat_interval_sec: Expected heartbeat interval
            timeout_threshold_sec: Timeout threshold before cleanup
            cleanup_callback: Optional callback for agent cleanup
            
        Returns:
            True if registration successful
        """
        with self._lock:
            try:
                status = AgentHeartbeatStatus(
                    agent_id=agent_id,
                    agent_type=agent_type,
                    last_heartbeat=datetime.now(timezone.utc),
                    heartbeat_interval_sec=heartbeat_interval_sec,
                    timeout_threshold_sec=timeout_threshold_sec
                )
                
                self._agent_statuses[agent_id] = status
                
                if cleanup_callback:
                    self._cleanup_callbacks[agent_id] = cleanup_callback
                
                logger.info(f"💗 Registered heartbeat monitoring for {agent_type} agent '{agent_id}' "
                          f"(interval={heartbeat_interval_sec}s, timeout={timeout_threshold_sec}s)")
                
                return True
                
            except Exception as e:
                logger.error(f"Failed to register heartbeat for agent {agent_id}: {e}")
                return False
    
    def heartbeat_agent(self, agent_id: str) -> bool:
        """
        Record a heartbeat for an agent.
        
        Args:
            agent_id: Agent identifier
            
        Returns:
            True if heartbeat recorded successfully
        """
        with self._lock:
            if agent_id in self._agent_statuses:
                status = self._agent_statuses[agent_id]
                status.last_heartbeat = datetime.now(timezone.utc)
                status.consecutive_misses = 0
                status.is_active = True
                
                logger.debug(f"💗 Heartbeat recorded for agent '{agent_id}'")
                return True
            
            logger.warning(f"💔 Heartbeat received from unregistered agent '{agent_id}'")
            return False
    
    def deregister_agent_heartbeat(self, agent_id: str) -> bool:
        """
        Remove an agent from heartbeat monitoring.
        
        Args:
            agent_id: Agent identifier
            
        Returns:
            True if deregistration successful
        """
        with self._lock:
            status = self._agent_statuses.pop(agent_id, None)
            self._cleanup_callbacks.pop(agent_id, None)
            
            if status:
                logger.info(f"💔 Deregistered heartbeat monitoring for agent '{agent_id}'")
                return True
            
            return False
    
    def get_agent_status(self, agent_id: str) -> Optional[AgentHeartbeatStatus]:
        """Get heartbeat status for a specific agent."""
        with self._lock:
            return self._agent_statuses.get(agent_id)
    
    def get_all_statuses(self) -> Dict[str, AgentHeartbeatStatus]:
        """Get heartbeat statuses for all registered agents."""
        with self._lock:
            return self._agent_statuses.copy()
    
    def start_monitoring(self) -> None:
        """Start the heartbeat monitoring task."""
        if self._running:
            logger.warning("Heartbeat monitoring already running")
            return
            
        self._running = True
        self._monitor_task = asyncio.create_task(self._monitor_heartbeats())
        logger.info("💗 Started heartbeat monitoring task")
    
    async def stop_monitoring(self) -> None:
        """Stop the heartbeat monitoring task."""
        self._running = False
        
        if self._monitor_task:
            try:
                self._monitor_task.cancel()
                await self._monitor_task
            except asyncio.CancelledError:
                pass
            
        logger.info("💔 Stopped heartbeat monitoring task")
    
    async def _monitor_heartbeats(self) -> None:
        """Monitor agent heartbeats and handle timeouts."""
        while self._running:
            try:
                current_time = datetime.now(timezone.utc)
                agents_to_cleanup = []
                
                with self._lock:
                    for agent_id, status in self._agent_statuses.items():
                        if not status.is_active:
                            continue
                            
                        time_since_heartbeat = (current_time - status.last_heartbeat).total_seconds()
                        
                        if time_since_heartbeat > status.timeout_threshold_sec:
                            # Agent has timed out
                            status.is_active = False
                            status.consecutive_misses += 1
                            agents_to_cleanup.append(agent_id)
                            
                            logger.warning(f"💔 Agent '{agent_id}' ({status.agent_type}) timed out "
                                         f"(inactive for {time_since_heartbeat:.1f}s, "
                                         f"threshold={status.timeout_threshold_sec}s)")
                
                # Execute cleanup callbacks outside the lock
                for agent_id in agents_to_cleanup:
                    try:
                        cleanup_callback = self._cleanup_callbacks.get(agent_id)
                        if cleanup_callback:
                            cleanup_callback(agent_id)
                    except Exception as e:
                        logger.error(f"Error in cleanup callback for agent {agent_id}: {e}")
                
                # Sleep before next check
                await asyncio.sleep(10.0)  # Check every 10 seconds
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in heartbeat monitoring: {e}")
                await asyncio.sleep(5.0)  # Brief pause on error


# Singleton instance
_heartbeat_coordinator: Optional[HeartbeatCoordinator] = None
_coordinator_lock = threading.Lock()


def get_heartbeat_coordinator() -> HeartbeatCoordinator:
    """Get the singleton heartbeat coordinator instance."""
    global _heartbeat_coordinator
    
    with _coordinator_lock:
        if _heartbeat_coordinator is None:
            _heartbeat_coordinator = HeartbeatCoordinator()
        
        return _heartbeat_coordinator
