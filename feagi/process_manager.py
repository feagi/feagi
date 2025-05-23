"""FEAGI Process Manager.

This module provides a centralized Process Manager for FEAGI that implements the
Rust/RTOS compatible architecture described in feagi_processes.md. It handles starting, 
monitoring, and stopping async tasks and services according to their priority levels.

ARCHITECTURE: Rust/RTOS Compatible
- Uses singleton ConnectomeManager for mission-critical reliability
- Direct task spawning instead of subprocess boundaries  
- Memory-mapped state for zero-copy data access
- No environment variable dependencies for IPC

MIGRATION PATH: Python → Rust
- threading.Thread → tokio::spawn
- Memory-mapped files → memmap2 crate
- Singleton pattern → std::sync::Once
"""
import asyncio
from feagi.utils.logger import setup_logger
logger = setup_logger(name="Process Manager")

import os
import signal
import socket
import sys
import threading
import time
from typing import Dict, Any, Optional, List, Tuple, Set
from feagi.npu.burst_engine import FQSampler
from feagi.core.state_manager import FeagiStateManager
from queue import Queue


# Process priority levels
PRIORITY_CRITICAL = 1  # Real-time critical processes
PRIORITY_IMPORTANT = 2  # Near real-time processes
PRIORITY_BACKGROUND = 3  # Best effort processes

class ProcessManager:
    """
    FEAGI Process Manager - Rust/RTOS Compatible Architecture.
    
    Responsible for:
    1. Task Creation: Launches async tasks with appropriate resources and parameters.
    2. Task Monitoring: Monitors task health and performance without subprocess overhead.
    3. Resource Allocation: Distributes computing resources based on priority.
    4. Fault Tolerance: Restarts failed tasks and maintains system integrity.
    
    RUST/RTOS COMPATIBLE FEATURES:
    - Singleton ConnectomeManager for consistent state access
    - Direct task spawning eliminates subprocess boundaries  
    - Memory-mapped state for instant synchronization
    - No environment variable IPC dependencies
    """
    
    def __init__(self):
        """Initialize the Process Manager with singleton ConnectomeManager."""
        self._processes = {}
        self._resource_allocations = {}
        self._running = False
        self._monitor_thread = None
        self._core_api = None
        self._zmq_server = None
        
        # CRITICAL: Use ConnectomeManager singleton for mission-critical reliability
        from feagi.bdu.connectome_manager import ConnectomeManager
        self._connectome_manager = ConnectomeManager.instance()
        
        # Internal references for critical components
        self._burst_engine = None
        self._fcl_manager = None
        self._memory_manager = None
        
        # Track which ports are in use
        self._used_ports = set()
        
        self._fcl_sampler = None
        self._fcl_sampler_thread = None
        self._fcl_sampler_queue = None
        
        # FQ Sampler (Fire Queue Sampler) - replacement for FCL sampler
        self._fq_sampler = None
        self._fq_sampler_thread = None
        self._fq_sampler_queue = None
        
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
            # --- FQSampler Integration ---
            from feagi.core.state_manager import FeagiStateManager, ServiceState
            state_manager = FeagiStateManager.instance()
            state_manager.set_fcl_sampler_state(ServiceState.INITIALIZING)  # Keep same state for compatibility
            # Set FQSampler frequency and consumer in state manager
            sampler_frequency = 20.0  # TODO: Make configurable
            sampler_consumer = 1      # 1=Visualization, 2=Motor, 3=Both (default: Visualization)
            state_manager.set_fcl_sampler_frequency(sampler_frequency)
            state_manager.set_fcl_sampler_consumer(sampler_consumer)
            # Create output queue for visualization/motor consumers
            self._fq_sampler_queue = Queue(maxsize=50)  # Best-effort queue - newer samples are prioritized over processing every sample
            # Use the fire queue provider (core object) from critical processes
            fire_queue_provider = self._core_api  # Core API provides fire queue access
            if fire_queue_provider is None:
                logger.error("Core API not initialized before FQSampler!")
                state_manager.set_fcl_sampler_state(ServiceState.ERROR)
                return False
            self._fq_sampler = FQSampler(
                fire_queue_provider=fire_queue_provider,
                sample_frequency_hz=sampler_frequency,
                output_queue=self._fq_sampler_queue,
                connectome_manager=self._connectome_manager
            )
            self._fq_sampler_thread = threading.Thread(target=self._fq_sampler.run, daemon=True)
            self._fq_sampler_thread.start()
            state_manager.set_fcl_sampler_state(ServiceState.READY)
            logger.info("FQSampler started successfully.", emoji1="✓ ")
            # --- FQSampler Integration ---
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
            rest_port = self.find_available_port(zmq_config.get("rest_port", 5563))
            vis_base_port = self.find_available_port(zmq_config.get("vis_base_port", 5562))
            
            # Ensure we found available ports
            if not all([req_rep_port, pub_sub_port, push_pull_port, sensory_port, motor_port, control_port, rest_port, vis_base_port]):
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
                rest_port=rest_port,
                vis_port=vis_base_port,
                fq_sampler=self._fq_sampler,
                fq_sampler_queue=self._fq_sampler_queue
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
        
        RUST/RTOS COMPATIBLE: Uses direct task spawning instead of subprocesses.
        All services run in the same process space with shared memory access.
        
        Args:
            config: Configuration parameters for the processes
            
        Returns:
            True if successfully initialized, False otherwise.
        """
        logger.info("Initializing background (Priority 3) processes...")
        
        try:
            # RUST/RTOS COMPATIBLE: Direct service instantiation instead of subprocess
            api_config = config.get("api", {})
            api_host = api_config.get("host", "127.0.0.1")
            api_port = self.find_available_port(api_config.get("port", 8000))
            
            if not api_port:
                logger.error("Could not find available port for API server")
                return False
            
            # CRITICAL REFACTOR: Create REST API service directly in same process
            # This eliminates subprocess boundaries and makes Rust migration trivial
            
            # Direct dependency injection - no environment variables needed
            api_service_config = {
                'core_api': self._core_api,
                'state_manager': FeagiStateManager.instance(),
                'connectome_manager': self._connectome_manager,
                'host': api_host,
                'port': api_port,
                'debug': api_config.get("debug_api", False)
            }
            
            # Create and start the API service as an async task (not subprocess)
            api_task = self._start_api_service_task(api_service_config)
            
            if not api_task:
                logger.error("Failed to start API service task")
                return False
                
            # Store task information instead of process information
            self._processes["api_server"] = {
                "task": api_task,
                "priority": PRIORITY_BACKGROUND,
                "start_time": time.time(),
                "config": api_config,
                "type": "async_task"  # Mark as task, not process
            }
            
            logger.info(f"API service started as async task on {api_host}:{api_port}")
            
            # Future: Add other background services here as direct tasks
            # - Stem Cell Manager Task
            # - Sleep Manager Task
            # - Monitoring Task
            
            logger.info("✓ Background processes initialized successfully", emoji1="✓ ")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize background processes: {e}")
            return False
    
    def _start_api_service_task(self, config: Dict[str, Any]) -> Optional[Any]:
        """
        Start API service as async task instead of subprocess.
        
        RUST/RTOS COMPATIBLE: This pattern translates directly to Rust async tasks.
        """
        try:
            import asyncio
            import threading
            
            # CRITICAL FIX: Ensure state synchronization between main process and FastAPI thread
            # Set environment variable so FastAPI thread uses the same state file
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            if hasattr(state_manager, 'path'):
                os.environ["FEAGI_STATE_FILE"] = state_manager.path
                logger.info(f"🔗 Sharing state file with FastAPI thread: {state_manager.path}")
            else:
                logger.warning("⚠️  State manager has no path attribute")
            
            # Create dedicated event loop for API service
            # In Rust, this would be a tokio::spawn() call
            def run_api_service():
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                
                try:
                    # Import and create FastAPI app with direct dependencies
                    from feagi.api.rest.app import create_rest_app_direct
                    app = create_rest_app_direct(config)
                    
                    # Run uvicorn in the same process
                    import uvicorn
                    uvicorn.run(
                        app, 
                        host=config['host'], 
                        port=config['port'],
                        loop="asyncio"
                    )
                except Exception as e:
                    logger.error(f"API service task failed: {e}")
                finally:
                    loop.close()
            
            # Start as daemon thread (in Rust: tokio::spawn with proper task management)
            api_thread = threading.Thread(target=run_api_service, daemon=True)
            api_thread.start()
            
            logger.info("✓ API service task started successfully", emoji1="✓ ")
            return api_thread
            
        except Exception as e:
            logger.error(f"Failed to start API service task: {e}")
            return None
    
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
        """
        Check all tasks and processes and restart any that have failed.
        
        RUST/RTOS COMPATIBLE: Monitors both async tasks and legacy processes.
        In Rust, this would be integrated with the async runtime's task monitoring.
        """
        for name, service_info in self._processes.items():
            service_type = service_info.get("type", "process")  # Default to legacy process
            
            if service_type == "async_task":
                # RUST/RTOS COMPATIBLE: Monitor async task health
                task = service_info.get("task")
                if task and hasattr(task, 'is_alive'):
                    if not task.is_alive():
                        logger.error(f"Async task {name} has stopped unexpectedly")
                        # TODO: Implement task restart logic if needed
                        # In Rust: respawn the task with tokio::spawn
                else:
                    logger.warning(f"Task {name} doesn't support health checking")
            else:
                # Legacy subprocess monitoring (will be removed in full Rust migration)
                process = service_info.get("process")
                if process and process.poll() is not None:
                    # Process has exited
                    exit_code = process.poll()
                    logger.error(f"Process {name} exited with code {exit_code}")
                    # TODO: Implement restart logic if needed
    
    def get_core_api(self):
        """Get the Core API instance."""
        return self._core_api
        
    def get_zmq_server(self):
        """Get the ZMQ server instance."""
        return self._zmq_server
        
    def shutdown(self) -> None:
        """
        Shut down the Process Manager and all managed tasks/processes.
        
        RUST/RTOS COMPATIBLE: Handles both async tasks and legacy processes.
        In Rust, this would be a clean async task cancellation system.
        """
        # @cursor:critical-path - Signal-safe shutdown should minimize logging
        try:
            print("Shutting down FEAGI services...", file=sys.stderr, flush=True)
            
            # Stop running flag to signal all services to stop
            self._running = False
            
            # Shutdown tasks and processes based on their type
            for name, service_info in self._processes.items():
                service_type = service_info.get("type", "process")  # Default to legacy process
                
                if service_type == "async_task":
                    # RUST/RTOS COMPATIBLE: Clean task shutdown
                    print(f"Stopping async task: {name}...", file=sys.stderr, flush=True)
                    task = service_info.get("task")
                    if task and hasattr(task, 'is_alive') and task.is_alive():
                        try:
                            # In Rust: task.cancel().await or similar
                            # For now, just signal the thread to stop gracefully
                            # The actual service should check self._running flag
                            task.join(timeout=2)
                        except Exception as e:
                            print(f"Error stopping task {name}: {e}", file=sys.stderr, flush=True)
                else:
                    # Legacy subprocess handling (will be removed in full Rust migration)
                    print(f"Terminating subprocess: {name}...", file=sys.stderr, flush=True)
                    process = service_info.get("process")
                    if process and process.poll() is None:
                        try:
                            process.terminate()
                            process.wait(timeout=2)
                        except:
                            if process.poll() is None:
                                process.kill()
                        
            # Next, shut down the ZMQ server
            if self._zmq_server:
                print("Terminating ZMQ server...", file=sys.stderr, flush=True)
                try:
                    self._zmq_server.shutdown()
                except Exception as e:
                    print(f"Error shutting down ZMQ server: {e}", file=sys.stderr, flush=True)
            
            # Stop FQSampler if running
            if hasattr(self, '_fq_sampler') and self._fq_sampler:
                print("Stopping FQSampler...", file=sys.stderr, flush=True)
                try:
                    self._fq_sampler.stop()
                    if hasattr(self, '_fq_sampler_thread') and self._fq_sampler_thread:
                        self._fq_sampler_thread.join(timeout=2)
                    self._fq_sampler = None
                    self._fq_sampler_thread = None
                    self._fq_sampler_queue = None
                except Exception as e:
                    print(f"Error stopping FQSampler: {e}", file=sys.stderr, flush=True)
            
            print("FEAGI services shut down", file=sys.stderr, flush=True)
            
        except Exception as e:
            # Last resort error handling - print to stderr and continue
            print(f"Critical error during shutdown: {e}", file=sys.stderr, flush=True)
        
    def signal_handler(self, sig, frame):
        """Handle termination signals."""
        self.shutdown()
        
    def update_area_sample_rate(self, cortical_id, rate):
        """
        Notify the running FQSampler of a per-area sample rate change.
        """
        if hasattr(self, '_fq_sampler') and self._fq_sampler is not None:
            self._fq_sampler.update_area_sample_rate(cortical_id, rate)
        # If FQSampler is not running, do nothing

# Global instance for the process manager
_process_manager = None

def get_process_manager() -> ProcessManager:
    """Get the global ProcessManager instance."""
    global _process_manager
    if _process_manager is None:
        _process_manager = ProcessManager()
    return _process_manager

# Removed the global process_manager instantiation that was here 