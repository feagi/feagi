"""
Fault tolerance module for FEAGI process management.

This module provides mechanisms for:
- Process health monitoring
- Automatic recovery from failures
- Checkpointing and state restoration
- Graceful degradation policies
"""
import os
import time
import logging
import threading
from typing import Dict, List, Optional, Any, Callable, Set, Tuple
from dataclasses import dataclass, field
from enum import Enum, auto

logger = logging.getLogger("feagi.fault_tolerance")


class ProcessState(Enum):
    """States a process can be in."""
    STARTING = auto()
    RUNNING = auto()
    WARNING = auto()  # Process is running but showing signs of issues
    CRITICAL = auto()  # Process is running but in critical condition
    FAILED = auto()    # Process has crashed or is unresponsive
    RECOVERING = auto()
    TERMINATED = auto()


class RestartPolicy(Enum):
    """Policies for restarting failed processes."""
    ALWAYS = auto()       # Always restart
    NEVER = auto()        # Never restart
    LIMITED = auto()      # Restart up to a limit
    EXPONENTIAL = auto()  # Restart with exponential backoff


@dataclass
class ProcessHealth:
    """Health status for a monitored process."""
    process_name: str
    pid: int
    state: ProcessState = ProcessState.STARTING
    restart_policy: RestartPolicy = RestartPolicy.ALWAYS
    max_restarts: int = 3
    restart_count: int = 0
    last_restart_time: float = 0.0
    backoff_factor: float = 2.0
    initial_backoff: float = 1.0
    heartbeat_timeout: float = 10.0
    last_heartbeat: float = field(default_factory=time.time)
    resource_usage: Dict[str, Any] = field(default_factory=dict)
    # Critical thresholds
    cpu_warning_threshold: float = 0.9  # 90%
    memory_warning_threshold: float = 0.9  # 90%
    heartbeat_warning_threshold: float = 0.8  # 80% of timeout
    
    def should_restart(self) -> bool:
        """Determine if process should be restarted based on policy."""
        if self.restart_policy == RestartPolicy.NEVER:
            return False
        
        if self.restart_policy == RestartPolicy.ALWAYS:
            return True
        
        if self.restart_policy == RestartPolicy.LIMITED:
            return self.restart_count < self.max_restarts
        
        # EXPONENTIAL policy
        if self.restart_count >= self.max_restarts:
            return False
        
        # Check if enough time has passed since last restart
        if self.last_restart_time == 0.0:
            return True
        
        backoff_time = self.initial_backoff * (self.backoff_factor ** self.restart_count)
        time_since_restart = time.time() - self.last_restart_time
        
        return time_since_restart >= backoff_time
    
    def get_next_backoff_time(self) -> float:
        """Calculate next backoff time for exponential policy."""
        return self.initial_backoff * (self.backoff_factor ** self.restart_count)


class HealthMonitor:
    """
    Monitor process health and implement recovery strategies.
    
    This class tracks process health status, detects failures,
    and initiates recovery actions according to defined policies.
    """
    
    def __init__(self, resource_manager_ref: Any = None):
        """
        Initialize the health monitor.
        
        Args:
            resource_manager_ref: Reference to the ResourceManager
        """
        self.resource_manager = resource_manager_ref
        self._lock = threading.RLock()
        self.monitored_processes: Dict[str, ProcessHealth] = {}
        self.monitor_thread = None
        self.stop_event = threading.Event()
        
        # Register callback functions for different health states
        self.state_handlers: Dict[ProcessState, Callable[[str], None]] = {
            ProcessState.FAILED: self._handle_failed_process,
            ProcessState.WARNING: self._handle_warning_process,
            ProcessState.CRITICAL: self._handle_critical_process,
        }
    
    def register_process(
        self, 
        process_name: str, 
        pid: int, 
        restart_policy: RestartPolicy = RestartPolicy.ALWAYS,
        max_restarts: int = 3,
        heartbeat_timeout: float = 10.0,
    ) -> None:
        """
        Register a process for health monitoring.
        
        Args:
            process_name: Name of the process
            pid: Process ID
            restart_policy: Policy for restarting failed processes
            max_restarts: Maximum number of restart attempts
            heartbeat_timeout: Timeout in seconds for missing heartbeats
        """
        with self._lock:
            health = ProcessHealth(
                process_name=process_name,
                pid=pid,
                restart_policy=restart_policy,
                max_restarts=max_restarts,
                heartbeat_timeout=heartbeat_timeout,
                last_heartbeat=time.time(),
                state=ProcessState.RUNNING
            )
            
            self.monitored_processes[process_name] = health
            logger.info(f"Registered process {process_name} (PID: {pid}) for health monitoring")
    
    def update_heartbeat(self, process_name: str) -> bool:
        """
        Update the heartbeat timestamp for a process.
        
        Args:
            process_name: Name of the process
            
        Returns:
            True if heartbeat was updated, False if process not found
        """
        with self._lock:
            if process_name not in self.monitored_processes:
                return False
            
            health = self.monitored_processes[process_name]
            health.last_heartbeat = time.time()
            
            # If process was in WARNING state due to heartbeat, restore to RUNNING
            if (health.state == ProcessState.WARNING and 
                time.time() - health.last_heartbeat < health.heartbeat_warning_threshold * health.heartbeat_timeout):
                health.state = ProcessState.RUNNING
                logger.info(f"Process {process_name} recovered from heartbeat warning")
            
            return True
    
    def update_resource_usage(self, process_name: str, cpu_usage: float, memory_usage: int) -> bool:
        """
        Update resource usage metrics for a process.
        
        Args:
            process_name: Name of the process
            cpu_usage: CPU usage as a fraction (0.0-1.0)
            memory_usage: Memory usage in bytes
            
        Returns:
            True if metrics were updated, False if process not found
        """
        with self._lock:
            if process_name not in self.monitored_processes:
                return False
            
            health = self.monitored_processes[process_name]
            health.resource_usage["cpu"] = cpu_usage
            health.resource_usage["memory"] = memory_usage
            
            # Check thresholds and update state if needed
            self._check_resource_thresholds(process_name)
            
            return True
    
    def _check_resource_thresholds(self, process_name: str) -> None:
        """
        Check if resource usage exceeds warning thresholds.
        
        Args:
            process_name: Name of the process
        """
        health = self.monitored_processes[process_name]
        current_state = health.state
        
        # Only check if process is RUNNING or in WARNING state
        if current_state not in (ProcessState.RUNNING, ProcessState.WARNING):
            return
        
        cpu_usage = health.resource_usage.get("cpu", 0.0)
        memory_usage = health.resource_usage.get("memory", 0)
        
        # Check for CRITICAL condition
        if cpu_usage >= health.cpu_warning_threshold:
            if current_state != ProcessState.CRITICAL:
                health.state = ProcessState.CRITICAL
                logger.warning(f"Process {process_name} in CRITICAL state: high CPU usage ({cpu_usage:.1%})")
                self._handle_critical_process(process_name)
        
        # Check for WARNING condition
        elif cpu_usage >= health.cpu_warning_threshold * 0.8:
            if current_state == ProcessState.RUNNING:
                health.state = ProcessState.WARNING
                logger.warning(f"Process {process_name} in WARNING state: elevated CPU usage ({cpu_usage:.1%})")
                self._handle_warning_process(process_name)
        
        # If CPU usage is normal but was previously in WARNING, restore to RUNNING
        elif current_state == ProcessState.WARNING:
            health.state = ProcessState.RUNNING
            logger.info(f"Process {process_name} recovered from resource warning")
    
    def start_monitoring(self, check_interval: float = 1.0) -> None:
        """
        Start the health monitoring thread.
        
        Args:
            check_interval: Interval in seconds between health checks
        """
        if self.monitor_thread and self.monitor_thread.is_alive():
            logger.warning("Health monitoring is already running")
            return
        
        self.stop_event.clear()
        self.monitor_thread = threading.Thread(
            target=self._monitoring_loop,
            args=(check_interval,),
            daemon=True,
            name="health-monitor"
        )
        self.monitor_thread.start()
        logger.info("Started health monitoring")
    
    def stop_monitoring(self) -> None:
        """Stop the health monitoring thread."""
        if not self.monitor_thread or not self.monitor_thread.is_alive():
            return
        
        self.stop_event.set()
        self.monitor_thread.join(timeout=5.0)
        logger.info("Stopped health monitoring")
    
    def _monitoring_loop(self, check_interval: float) -> None:
        """
        Main monitoring loop that periodically checks process health.
        
        Args:
            check_interval: Interval in seconds between health checks
        """
        while not self.stop_event.is_set():
            try:
                self._check_all_processes()
            except Exception as e:
                logger.error(f"Error in health monitoring loop: {e}")
            
            # Wait for the next check interval or until stopped
            self.stop_event.wait(check_interval)
    
    def _check_all_processes(self) -> None:
        """Check health status of all monitored processes."""
        with self._lock:
            current_time = time.time()
            
            for process_name, health in list(self.monitored_processes.items()):
                # Skip processes not in RUNNING, WARNING, or CRITICAL state
                if health.state not in (ProcessState.RUNNING, ProcessState.WARNING, ProcessState.CRITICAL):
                    continue
                
                # Check if heartbeat is missing
                time_since_heartbeat = current_time - health.last_heartbeat
                
                if time_since_heartbeat >= health.heartbeat_timeout:
                    # Mark as failed due to missing heartbeat
                    health.state = ProcessState.FAILED
                    logger.error(f"Process {process_name} failed: missing heartbeat for {time_since_heartbeat:.1f}s")
                    self._handle_failed_process(process_name)
                
                elif time_since_heartbeat >= health.heartbeat_warning_threshold * health.heartbeat_timeout:
                    # Mark as warning due to delayed heartbeat
                    if health.state == ProcessState.RUNNING:
                        health.state = ProcessState.WARNING
                        logger.warning(f"Process {process_name} warning: delayed heartbeat for {time_since_heartbeat:.1f}s")
                        self._handle_warning_process(process_name)
                
                # Check process is still alive via OS
                if self.resource_manager:
                    alive = self.resource_manager.is_process_alive(process_name)
                    if not alive and health.state != ProcessState.FAILED:
                        health.state = ProcessState.FAILED
                        logger.error(f"Process {process_name} failed: process not running")
                        self._handle_failed_process(process_name)
    
    def _handle_failed_process(self, process_name: str) -> None:
        """
        Handle a failed process according to its restart policy.
        
        Args:
            process_name: Name of the failed process
        """
        with self._lock:
            if process_name not in self.monitored_processes:
                return
            
            health = self.monitored_processes[process_name]
            
            if health.should_restart() and self.resource_manager:
                # Update restart stats
                health.restart_count += 1
                health.last_restart_time = time.time()
                health.state = ProcessState.RECOVERING
                
                # Log restart attempt
                logger.info(f"Attempting to restart process {process_name} (attempt {health.restart_count}/{health.max_restarts})")
                
                # Request restart from resource manager
                success = self.resource_manager.restart_process(process_name)
                
                if success:
                    logger.info(f"Successfully initiated restart of process {process_name}")
                else:
                    logger.error(f"Failed to restart process {process_name}")
                    health.state = ProcessState.FAILED
            else:
                logger.warning(f"Process {process_name} failed but will not be restarted (policy: {health.restart_policy.name})")
    
    def _handle_warning_process(self, process_name: str) -> None:
        """
        Handle a process in WARNING state.
        
        Args:
            process_name: Name of the process
        """
        # In a warning state, we might want to:
        # - Log additional diagnostics
        # - Reduce workload if possible
        # - Notify administrators
        pass
    
    def _handle_critical_process(self, process_name: str) -> None:
        """
        Handle a process in CRITICAL state.
        
        Args:
            process_name: Name of the process
        """
        # In a critical state, we might want to:
        # - Attempt to reduce its resource usage
        # - Prepare for potential failure
        # - Terminate less critical processes to free resources
        pass
    
    def get_health_status(self, process_name: str) -> Optional[Dict[str, Any]]:
        """
        Get health status for a specific process.
        
        Args:
            process_name: Name of the process
            
        Returns:
            Health status information or None if process is not monitored
        """
        with self._lock:
            if process_name not in self.monitored_processes:
                return None
            
            health = self.monitored_processes[process_name]
            return {
                "process_name": health.process_name,
                "pid": health.pid,
                "state": health.state.name,
                "restart_policy": health.restart_policy.name,
                "restart_count": health.restart_count,
                "last_heartbeat": health.last_heartbeat,
                "time_since_heartbeat": time.time() - health.last_heartbeat,
                "resource_usage": health.resource_usage
            }
    
    def get_all_health_status(self) -> Dict[str, Dict[str, Any]]:
        """
        Get health status for all monitored processes.
        
        Returns:
            Dictionary mapping process names to their health status
        """
        result = {}
        with self._lock:
            for process_name in self.monitored_processes:
                result[process_name] = self.get_health_status(process_name)
        return result 