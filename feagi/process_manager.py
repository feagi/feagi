"""FEAGI Process Manager.

This module provides a centralized Process Manager for FEAGI that implements the
architecture described in feagi_processes.md. It handles starting, monitoring, and
stopping processes according to their priority levels.
"""
import asyncio
from feagi.utils.logger import setup_logger
logger = setup_logger(name="Process Manager")

import os
import signal
import socket
import subprocess
import sys
import threading
import time
from typing import Dict, Any, Optional, List, Tuple, Set
from feagi.npu.burst_engine import FCLSampler
from queue import Queue
import re


# Process priority levels
PRIORITY_CRITICAL = 1  # Real-time critical processes
PRIORITY_IMPORTANT = 2  # Near real-time processes
PRIORITY_BACKGROUND = 3  # Best effort processes

class ProcessManager:
    """
    FEAGI Process Manager.
    
    Responsible for:
    1. Process Creation: Launches processes with appropriate resources and parameters.
    2. Process Monitoring: Monitors process health and performance.
    3. Resource Allocation: Distributes computing resources based on priority.
    4. Fault Tolerance: Restarts failed processes and maintains system integrity.
    """
    
    def __init__(self, connectome=None):
        """Initialize the Process Manager."""
        self._processes = {}
        self._resource_allocations = {}
        self._running = False
        self._monitor_thread = None
        self._core_api = None
        self._zmq_server = None
        
        # Internal references for critical components
        self._burst_engine = None
        self._connectome_manager = connectome
        self._fcl_manager = None
        self._memory_manager = None
        
        # Track which ports are in use
        self._used_ports = set()
        
        self._fcl_sampler = None
        self._fcl_sampler_thread = None
        self._fcl_sampler_queue = None
        
    def find_available_port(self, start_port: int, max_tries: int = 10) -> Optional[int]:
        """
        Find an available port starting from start_port.
        
        Args:
            start_port: The port to start checking from
            max_tries: Maximum number of ports to try
            
        Returns:
            An available port or None if no port was found
        """
        for port_offset in range(max_tries):
            port = start_port + port_offset
            
            # Skip if we already know this port is used
            if port in self._used_ports:
                continue
                
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind(("127.0.0.1", port))
                    self._used_ports.add(port)
                    return port
            except OSError:
                logger.warning(f"Port {port} is already in use, trying next port...")
        
        logger.error(f"Could not find an available port after {max_tries} attempts")
        return None
    
    def init_critical_processes(self, config: Dict[str, Any]) -> bool:
        """
        Initialize Priority 1 (Critical) processes.
        
        These processes are essential for neural simulation and include:
        - Burst Engine
        - Connectome Manager
        - FCL Manager
        - Memory & Learning Manager
        
        Args:
            config: Configuration parameters for the processes
            
        Returns:
            True if successfully initialized, False otherwise
        """
        logger.info("Initializing critical (Priority 1) processes...")
        
        try:
            # Initialize core components - these run in the same process
            # but are conceptually distinct according to the architecture
            
            # Import here to avoid circular imports
            from feagi.core import create_core_api
            
            # Create the core API with all critical components
            self._core_api = create_core_api(self._connectome_manager, config)
            
            # Get references to individual critical components
            if self._core_api:
                self._burst_engine = self._core_api.get_burst_engine()
                self._connectome_manager = self._core_api.get_connectome_manager()
                self._fcl_manager = self._core_api.get_fcl_manager()
                self._memory_manager = self._core_api.get_memory_manager()
            
            logger.info("✓ Critical processes initialized successfully", emoji1="✓ ")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize critical processes: {e}")
            return False
    
    def init_important_processes(self, config: Dict[str, Any]) -> bool:
        """
        Initialize Priority 2 (Important) processes.
        
        These processes handle important but less time-critical operations:
        - FCL Sampler
        - PNS Message Broker (implemented as ZMQ server)
        - Resource Manager
        
        Args:
            config: Configuration parameters for the processes
            
        Returns:
            True if successfully initialized, False otherwise
        """
        logger.info("Initializing important (Priority 2) processes...")
        
        try:
            # --- FCLSampler Integration ---
            from feagi.core.state_manager import FeagiStateManager, ServiceState
            state_manager = FeagiStateManager.instance()
            state_manager.set_fcl_sampler_state(ServiceState.INITIALIZING)
            # Set FCLSampler frequency and consumer in state manager
            sampler_frequency = 20.0  # TODO: Make configurable
            sampler_consumer = 1      # 1=Visualization, 2=Motor, 3=Both (default: Visualization)
            state_manager.set_fcl_sampler_frequency(sampler_frequency)
            state_manager.set_fcl_sampler_consumer(sampler_consumer)
            # Create output queue for visualization/motor consumers
            self._fcl_sampler_queue = Queue(maxsize=50)  # Increased from 10 to 50
            # Use the FCL manager from critical processes
            fcl_manager = self._fcl_manager
            if fcl_manager is None:
                logger.error("FCL Manager not initialized before FCLSampler!")
                state_manager.set_fcl_sampler_state(ServiceState.ERROR)
                return False
            self._fcl_sampler = FCLSampler(
                fcl_manager=fcl_manager,
                sample_frequency_hz=sampler_frequency,
                output_queue=self._fcl_sampler_queue,
                connectome_manager=self._connectome_manager
            )
            self._fcl_sampler_thread = threading.Thread(target=self._fcl_sampler.run, daemon=True)
            self._fcl_sampler_thread.start()
            state_manager.set_fcl_sampler_state(ServiceState.READY)
            logger.info("FCLSampler started successfully.", emoji1="✓ ")
            # --- FCLSampler Integration ---
            # If you add dynamic reconfiguration of frequency/consumer, update state manager here as well.
            
            # Initialize ZMQ server (acts as PNS Message Broker)
            zmq_config = config.get("zmq", {})
            
            # Check and adjust ports if needed
            host = zmq_config.get("host", "127.0.0.1")
            req_rep_port = self.find_available_port(zmq_config.get("req_port", 5555))
            pub_sub_port = self.find_available_port(zmq_config.get("pub_port", 5556))
            push_pull_port = self.find_available_port(zmq_config.get("push_port", 5557))
            sensory_port = self.find_available_port(zmq_config.get("sensory_port", 5558))
            motor_port = self.find_available_port(zmq_config.get("motor_port", 5564))
            control_port = self.find_available_port(zmq_config.get("control_port", 5559))
            vis_base_port = self.find_available_port(zmq_config.get("vis_base_port", 5560))
            
            # Ensure we found available ports
            if not all([req_rep_port, pub_sub_port, push_pull_port, sensory_port, motor_port, control_port, vis_base_port]):
                logger.error("Could not find available ports for ZMQ server")
                return False
                
            # Import here to avoid circular imports
            from feagi.api.zmq.server import ZmqServer
            
            # Create and start the ZMQ server
            self._zmq_server = ZmqServer(
                core_api=self._core_api,
                host=host,
                req_rep_port=req_rep_port,
                pub_sub_port=pub_sub_port,
                push_pull_port=push_pull_port,
                sensory_port=sensory_port,
                motor_port=motor_port,
                control_port=control_port,
                vis_port=vis_base_port,
                fcl_sampler=self._fcl_sampler,
                fcl_sampler_queue=self._fcl_sampler_queue
            )
            
            # Start the ZMQ server
            logger.info("Starting ZMQ server...")
            success = self._zmq_server.start()
            
            if not success:
                logger.error("Failed to start ZMQ server")
                return False
                
            logger.info("✓ Important processes initialized successfully", emoji1="✓ ")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize important processes: {e}")
            from feagi.core.state_manager import FeagiStateManager, ServiceState
            state_manager = FeagiStateManager.instance()
            state_manager.set_fcl_sampler_state(ServiceState.ERROR)
            return False
    
    def init_background_processes(self, config: Dict[str, Any]) -> bool:
        """
        Initialize Priority 3 (Background) processes.
        
        These processes handle optional or background operations:
        - Web Server (REST API)
        - Stem Cell Manager
        - Sleep Manager
        
        Args:
            config: Configuration parameters for the processes
            
        Returns:
            True if successfully initialized, False otherwise.
        """
        logger.info("Initializing background (Priority 3) processes...")
        
        try:
            # Start the REST API server
            api_config = config.get("api", {})
            api_host = api_config.get("host", "127.0.0.1")
            api_port = self.find_available_port(api_config.get("port", 8000))
            api_reload = api_config.get("reload", False)
            
            if not api_port:
                logger.error("Could not find available port for API server")
                return False
                
            # Build the command to start the API server
            cmd = [
                sys.executable, "-m", "uvicorn", 
                "feagi.api.rest.app:create_rest_app", 
                "--host", api_host,
                "--port", str(api_port),
                "--factory"
            ]
            
            if api_reload:
                cmd.append("--reload")
            
            # Set environment variables for ZMQ configuration
            env = os.environ.copy()
            env["FEAGI_INITIALIZED"] = "1"
            
            # Check if ZMQ server is available and set related env vars
            if self._zmq_server:
                env["FEAGI_ZMQ_ENABLED"] = "1"
                env["FEAGI_ZMQ_HOST"] = self._zmq_server.host
                env["FEAGI_ZMQ_REQ_PORT"] = str(self._zmq_server.req_rep_port)
                env["FEAGI_ZMQ_PUB_PORT"] = str(self._zmq_server.pub_sub_port)
                env["FEAGI_ZMQ_PUSH_PORT"] = str(self._zmq_server.push_pull_port)
                env["FEAGI_ZMQ_STREAM_PORT"] = str(self._zmq_server.sensory_port)
            else:
                env["FEAGI_ZMQ_ENABLED"] = "0"
                
            # Set environment variable for core API
            if self._core_api:
                env["FEAGI_CORE_API_AVAILABLE"] = "1"
            else:
                env["FEAGI_CORE_API_AVAILABLE"] = "0"
                
            # Start the API server process
            logger.info(f"Starting FEAGI API server on {api_host}:{api_port}", emoji1="  ")
            
            process = subprocess.Popen(
                cmd, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.STDOUT,
                env=env,
                universal_newlines=True,
                bufsize=1
            )
            
            # Store process information
            self._processes["api_server"] = {
                "process": process,
                "priority": PRIORITY_BACKGROUND,
                "start_time": time.time(),
                "config": api_config
            }
            
            # Wait briefly to see if the server starts up successfully
            time.sleep(0.5)
            rc = process.poll()
            if rc is not None:
                logger.error(f"API server exited immediately with code {rc}")
                logger.error("API server failed to start - check the logs for details")
                self._print_process_output(process)
                return False
                
            # Start a thread to monitor the API server output
            threading.Thread(
                target=self._monitor_process_output,
                args=(process, "api_server"),
                daemon=True
            ).start()
            
            logger.info(f"API server started with PID {process.pid}")
            
            # Initialize additional background processes here in the future
            # such as Stem Cell Manager, Sleep Manager, etc.
            
            logger.info("✓ Background processes initialized successfully", emoji1="✓ ")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize background processes: {e}")
            return False
            
    def _monitor_process_output(self, process, process_name):
        """Monitor and log output from a subprocess."""
        # Pattern to match log lines with emoji and log level
        # Format: [emoji1]  [level]  [timestamp] [label] message
        emoji_log_pattern = re.compile(r'^([^\s]{1,2})\s+(?:INFO|DEBUG|WARNING|ERROR|CRITICAL)\s+(\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2})(?:\s+\[[^\]]+\])?\s*(.*)')
        
        # Fallback pattern for standard log lines without emoji
        std_log_pattern = re.compile(r'^\s*(?:DEBUG|INFO|WARNING|ERROR|CRITICAL)\s+(\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2})(?:\s+\[[^\]]+\])?\s*(.*)')
        
        # Pattern to detect nested logs within a message
        nested_emoji_pattern = re.compile(r'([^\s]{1,2})\s+(?:INFO|DEBUG|WARNING|ERROR|CRITICAL)\s+\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}(?:\s+\[[^\]]+\])?\s*(.*)')
        nested_std_pattern = re.compile(r'(?:DEBUG|INFO|WARNING|ERROR|CRITICAL)\s+\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}(?:\s+\[[^\]]+\])?\s*(.*)')
        
        while True:
            line = process.stdout.readline()
            if not line and process.poll() is not None:
                break
            if line:
                line = line.rstrip()
                
                # First try to match log lines with emoji
                match = emoji_log_pattern.search(line)
                if match:
                    emoji = match.group(1)
                    message = match.group(3).strip()
                    
                    # Check for nested logs in the message
                    nested_emoji_match = nested_emoji_pattern.search(message)
                    if nested_emoji_match:
                        # Use the emoji from the nested log if available
                        nested_emoji = nested_emoji_match.group(1)
                        nested_message = nested_emoji_match.group(2).strip()
                        logger.info(f"{process_name}: {nested_message}", emoji1=nested_emoji)
                    else:
                        nested_std_match = nested_std_pattern.search(message)
                        if nested_std_match:
                            nested_message = nested_std_match.group(1).strip()
                            logger.info(f"{process_name}: {nested_message}", emoji1=emoji)
                        else:
                            logger.info(f"{process_name}: {message}", emoji1=emoji)
                else:
                    # Try standard log pattern
                    std_match = std_log_pattern.search(line)
                    if std_match:
                        message = std_match.group(2).strip()
                        
                        # Check for nested logs in the message
                        nested_emoji_match = nested_emoji_pattern.search(message)
                        if nested_emoji_match:
                            nested_emoji = nested_emoji_match.group(1)
                            nested_message = nested_emoji_match.group(2).strip()
                            logger.info(f"{process_name}: {nested_message}", emoji1=nested_emoji)
                        else:
                            nested_std_match = nested_std_pattern.search(message)
                            if nested_std_match:
                                nested_message = nested_std_match.group(1).strip()
                                logger.info(f"{process_name}: {nested_message}")
                            else:
                                logger.info(f"{process_name}: {message}")
                    else:
                        # If no log pattern matches, log the full line
                        logger.info(f"{process_name} {line}")
        
        rc = process.poll()
        if rc != 0:
            logger.error(f"{process_name} exited with code {rc}")
            
    def _print_process_output(self, process):
        """Print any available output from a process that has terminated."""
        output, _ = process.communicate()
        if output:
            for line in output.splitlines():
                logger.error(f"process output {line}", emoji1="❌")
        else:
            logger.error("No output available from the process")
    
    def start(self, config: Dict[str, Any]) -> bool:
        """
        Start all FEAGI processes in priority order.
        
        Args:
            config: Configuration for all processes
            
        Returns:
            True if all processes started successfully, False otherwise
        """
        logger.info("Starting FEAGI Process Manager...")
        self._running = True
        
        # Start processes in priority order
        # 1. Critical processes (Priority 1)
        if not self.init_critical_processes(config):
            logger.error("Failed to initialize critical processes, aborting startup")
            self.shutdown()
            return False
            
        # 2. Important processes (Priority 2)
        if not self.init_important_processes(config):
            logger.error("Failed to initialize important processes, aborting startup")
            self.shutdown()
            return False
            
        # 3. Background processes (Priority 3)
        if not self.init_background_processes(config):
            logger.error("Failed to initialize background processes, aborting startup")
            self.shutdown()
            return False
            
        # Start monitoring thread
        self._start_monitoring()
        
        logger.info("FEAGI Process Manager started successfully")
        return True
    
    def _start_monitoring(self):
        """Start the process monitoring thread."""
        def monitor_processes():
            while self._running:
                try:
                    self._check_processes()
                except Exception as e:
                    logger.error(f"Error in process monitor: {e}")
                time.sleep(5)  # Check every 5 seconds
                
        self._monitor_thread = threading.Thread(target=monitor_processes, daemon=True)
        self._monitor_thread.start()
    
    def _check_processes(self):
        """Check all processes and restart any that have failed."""
        # Check API server process
        api_process = self._processes.get("api_server")
        if api_process:
            process = api_process["process"]
            if process.poll() is not None:
                # Process has exited
                exit_code = process.poll()
                logger.error(f"API server exited with code {exit_code}")
                
                # TODO: Implement restart logic if needed
                
    def get_core_api(self):
        """Get the Core API instance."""
        return self._core_api
        
    def get_zmq_server(self):
        """Get the ZMQ server instance."""
        return self._zmq_server
        
    def shutdown(self) -> None:
        """Shut down the Process Manager and all managed processes."""
        logger.info("\nShutting down FEAGI servers...")
        
        # First, terminate the API server if it's running
        if "api_server" in self._processes:
            api_process = self._processes["api_server"]["process"]
            if api_process and api_process.poll() is None:
                try:
                    api_process.terminate()
                    api_process.wait(timeout=2)
                except:
                    if api_process.poll() is None:
                        api_process.kill()
                        
        # Next, shut down the ZMQ server
        if self._zmq_server:
            logger.info("Terminating ZMQ server...")
            try:
                self._zmq_server.shutdown()
            except Exception as e:
                logger.error(f"Error shutting down ZMQ server: {e}")
                
        # Finally, clean up any other managed processes
        for name, process_info in self._processes.items():
            if name == "api_server":
                continue  # Already handled above
                
            process = process_info.get("process")
            if process and process.poll() is None:
                try:
                    process.terminate()
                    process.wait(timeout=2)
                except:
                    if process.poll() is None:
                        process.kill()
        
        # Stop FCLSampler if running
        if self._fcl_sampler:
            logger.info("Stopping FCLSampler...")
            self._fcl_sampler.stop()
            if self._fcl_sampler_thread:
                self._fcl_sampler_thread.join(timeout=2)
            self._fcl_sampler = None
            self._fcl_sampler_thread = None
            self._fcl_sampler_queue = None
        
        logger.info("FEAGI servers shut down")
        
    def signal_handler(self, sig, frame):
        """Handle termination signals."""
        self.shutdown()
        
    def update_area_sample_rate(self, cortical_id, rate):
        """
        Notify the running FCLSampler of a per-area sample rate change.
        """
        if self._fcl_sampler is not None:
            self._fcl_sampler.update_area_sample_rate(cortical_id, rate)
        # If FCLSampler is not running, do nothing

# Global instance for the process manager
_process_manager = None

def get_process_manager(connectome=None) -> ProcessManager:
    """Get the global ProcessManager instance."""
    global _process_manager
    if _process_manager is None:
        _process_manager = ProcessManager(connectome=connectome)
    return _process_manager

# Removed the global process_manager instantiation that was here 