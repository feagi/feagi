"""Resource Manager for FEAGI.

This module handles the process orchestration, resource allocation, and initialization
of critical data structures in FEAGI.
"""
import os
from feagi.utils.logger import setup_logger
logger = setup_logger()
import multiprocessing as mp
import threading
import time
import weakref
from typing import Dict, List, Optional, Union, Callable, Any, Set
from dataclasses import dataclass, field

from feagi.utils.data_structures import RustCompatible, rust_field, OwnershipType


# Thread safety lock for resource management
_resource_lock = threading.RLock()


@dataclass
class ProcessInfo(RustCompatible):
    """
    Information about a running process.
    
    process_name: Name of the process
    pid: Process ID
    cpu_allocation: Number of CPU cores allocated
    is_running: Whether the process is currently running
    start_time: Time when the process was started
    """
    
    process_name: str
    pid: int
    # Thread-safe field that can be mutated
    last_heartbeat: float = rust_field(ownership=OwnershipType.MUTABLE_BORROWED, thread_safe=True)
    cpu_allocation: int = 1
    is_running: bool = True
    start_time: float = field(default_factory=time.time)
    # Self-contained references that don't need to be tracked by Rust
    _process_ref: Any = field(default=None, repr=False)


@dataclass
class ResourceAllocation(RustCompatible):
    """
    Resource allocation information.
    
    cpu_cores: List of CPU core IDs allocated
    memory_mb: Amount of memory allocated in MB
    gpu_id: ID of the allocated GPU (if any)
    """
    
    cpu_cores: List[int] = field(default_factory=list)
    memory_mb: int = 0
    gpu_id: Optional[int] = None


class ResourceManager:
    """
    Manages resources and processes for FEAGI.
    
    This class is responsible for:
    1. Starting, terminating, and orchestrating FEAGI processes
    2. Allocating CPU/GPU resources
    3. Initializing critical data structures
    4. Monitoring process health
    
    Thread safety:
    - All public methods are thread-safe
    - Resources and process registries are protected by locks
    """
    
    # Use a class variable to keep track of instances
    _instances = weakref.WeakValueDictionary()
    
    def __init__(self, config: Optional[Dict] = None):
        """
        Initialize the Resource Manager.
        
        Args:
            config: Optional configuration dictionary
        """
        # Use a singleton pattern to avoid multiple instances
        with _resource_lock:
            self.config = config or {}
            # Thread safety
            self._lock = threading.RLock()
            # Set up logger early
            self.logger = setup_logger("feagi.resource_mgr")
            # Use weak references to processes to avoid memory leaks
            self.processes: Dict[str, weakref.ReferenceType] = {}
            self.threads: Dict[str, weakref.ReferenceType] = {}
            # Strong references to process info
            self.process_info: Dict[str, ProcessInfo] = {}
            self.resources = self._detect_resources()
            # Allocated resources
            self.allocated_cpu_cores: Set[int] = set()
            self.allocated_gpu_ids: Set[int] = set()
            
            ResourceManager._instances[id(self)] = self
    
    @classmethod
    def get_instance(cls, config: Optional[Dict] = None) -> 'ResourceManager':
        """
        Get or create the ResourceManager instance.
        
        Args:
            config: Optional configuration dictionary
            
        Returns:
            ResourceManager instance
        """
        # Return the first instance if it exists, otherwise create a new one
        with _resource_lock:
            if not cls._instances:
                return cls(config)
            return next(iter(cls._instances.values()))
    
    def _detect_resources(self) -> Dict[str, Any]:
        """
        Detect available computing resources.
        
        Returns:
            Dictionary containing information about available resources
        """
        with self._lock:
            resources = {
                "cpu_count": os.cpu_count() or 1,
                "cpu_cores": list(range(os.cpu_count() or 1)),
                "gpu_available": False,
                "gpu_count": 0,
                "webgpu_available": False,
                "metal_available": False,
                "memory": self._get_available_memory(),
            }
            
            # Try to detect GPU resources if available
            try:
                import torch
                resources["gpu_available"] = torch.cuda.is_available()
                resources["gpu_count"] = torch.cuda.device_count() if resources["gpu_available"] else 0
                if resources["gpu_available"]:
                    resources["gpu_info"] = [
                        {
                            "id": i,
                            "name": torch.cuda.get_device_name(i),
                            "memory": torch.cuda.get_device_properties(i).total_memory,
                        }
                        for i in range(resources["gpu_count"])
                    ]
                
                # Check for Apple Metal support
                if hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
                    resources["metal_available"] = True
                    self.logger.info("Apple Metal (MPS) backend is available")
            except ImportError:
                self.logger.warning("PyTorch not available. GPU detection skipped.")
            
            # Check for WebGPU support
            try:
                import wgpu
                resources["webgpu_available"] = True
                
                # Get WebGPU adapter information if possible
                try:
                    if hasattr(wgpu, 'gpu') and hasattr(wgpu.gpu, 'request_adapter_sync'):
                        adapter = wgpu.gpu.request_adapter_sync()
                        if adapter:
                            # In newer wgpu versions, adapter_info might not be available
                            # Instead, just record that we have a valid adapter
                            resources["webgpu_info"] = {
                                "name": "WebGPU Adapter",
                                "driver": "Unknown",
                                "adapter_type": "Unknown"
                            }
                            self.logger.info(f"WebGPU adapter detected successfully")
                except Exception as e:
                    self.logger.warning(f"WebGPU adapter detection error: {e}")
            except ImportError:
                self.logger.debug("WebGPU not available.")
            
            return resources
    
    def _get_available_memory(self) -> int:
        """
        Get available system memory in bytes.
        
        Returns:
            Available memory in bytes
        """
        try:
            import psutil
            return psutil.virtual_memory().available
        except ImportError:
            self.logger.warning("psutil not available. Memory detection will be limited.")
            return 0
    
    def start_process(
        self, 
        name: str, 
        target: Callable, 
        args: tuple = (), 
        kwargs: Dict = None,
        cpu_allocation: int = 1,
    ) -> bool:
        """
        Start a new process with the given parameters.
        
        This method is thread-safe.
        
        Args:
            name: Name of the process
            target: Function to execute in the process
            args: Arguments to pass to the function
            kwargs: Keyword arguments to pass to the function
            cpu_allocation: Number of CPU cores to allocate
            
        Returns:
            True if the process was started successfully, False otherwise
        """
        with self._lock:
            # Check if process is already running
            proc_ref = self.processes.get(name)
            if proc_ref and proc_ref() and proc_ref().is_alive():
                self.logger.warning(f"Process {name} is already running.")
                return False
            
            # Allocate resources
            allocation = self._allocate_resources(name, cpu_allocation)
            if not allocation:
                self.logger.error(f"Failed to allocate resources for process {name}")
                return False
            
            self.logger.info(f"Starting process: {name}")
            process = mp.Process(
                name=name,
                target=target,
                args=args,
                kwargs=kwargs or {},
                daemon=True,
            )
            process.start()
            
            # Store process info
            process_info = ProcessInfo(
                process_name=name,
                pid=process.pid,
                cpu_allocation=cpu_allocation,
                is_running=True,
                start_time=time.time(),
                last_heartbeat=time.time(),
                _process_ref=process,
            )
            
            # Store weak reference to process
            self.processes[name] = weakref.ref(process)
            self.process_info[name] = process_info
            
            return True
    
    def _allocate_resources(self, process_name: str, cpu_count: int) -> Optional[ResourceAllocation]:
        """
        Allocate resources for a process.
        
        This method is internally thread-safe.
        
        Args:
            process_name: Name of the process
            cpu_count: Number of CPU cores to allocate
            
        Returns:
            ResourceAllocation if successful, None otherwise
        """
        with self._lock:
            available_cpus = set(self.resources["cpu_cores"]) - self.allocated_cpu_cores
            
            if len(available_cpus) < cpu_count:
                self.logger.warning(
                    f"Requested {cpu_count} CPUs for {process_name}, but only {len(available_cpus)} available. "
                    f"Allocating all available CPUs."
                )
                cpu_count = len(available_cpus)
            
            if cpu_count == 0:
                return None
            
            # Allocate CPU cores
            allocated_cores = list(available_cpus)[:cpu_count]
            self.allocated_cpu_cores.update(allocated_cores)
            
            self.logger.info(f"Allocated CPU cores {allocated_cores} for {process_name}")
            
            # Create resource allocation
            allocation = ResourceAllocation(
                cpu_cores=allocated_cores,
                memory_mb=1024,  # Default allocation, could be refined
                gpu_id=None,  # Default to no GPU
            )
            
            return allocation
    
    def start_thread(self, name: str, target: Callable, args: tuple = (), kwargs: Dict = None) -> bool:
        """
        Start a new thread with the given parameters.
        
        This method is thread-safe.
        
        Args:
            name: Name of the thread
            target: Function to execute in the thread
            args: Arguments to pass to the function
            kwargs: Keyword arguments to pass to the function
            
        Returns:
            True if the thread was started successfully, False otherwise
        """
        with self._lock:
            # Check if thread is already running
            thread_ref = self.threads.get(name)
            if thread_ref and thread_ref() and thread_ref().is_alive():
                self.logger.warning(f"Thread {name} is already running.")
                return False
            
            self.logger.info(f"Starting thread: {name}")
            thread = threading.Thread(
                name=name,
                target=target,
                args=args,
                kwargs=kwargs or {},
                daemon=True,
            )
            thread.start()
            
            # Store weak reference to thread
            self.threads[name] = weakref.ref(thread)
            
            return True
    
    def terminate_process(self, name: str) -> bool:
        """
        Terminate a running process.
        
        This method is thread-safe.
        
        Args:
            name: Name of the process to terminate
            
        Returns:
            True if the process was terminated successfully, False otherwise
        """
        with self._lock:
            # Get process reference
            proc_ref = self.processes.get(name)
            if not proc_ref or not proc_ref():
                self.logger.warning(f"Process {name} not found.")
                return False
            
            process = proc_ref()
            if not process.is_alive():
                self.logger.warning(f"Process {name} is not running.")
                # Clean up resources anyway
                self._cleanup_process_resources(name)
                return False
            
            self.logger.info(f"Terminating process: {name}")
            try:
                process.terminate()
                process.join(timeout=2)
                
                if process.is_alive():
                    self.logger.warning(f"Process {name} did not terminate gracefully. Killing it.")
                    process.kill()
                    process.join(timeout=1)
            except Exception as e:
                self.logger.error(f"Error terminating process {name}: {e}")
            
            # Clean up resources
            self._cleanup_process_resources(name)
            
            return True
    
    def _cleanup_process_resources(self, name: str) -> None:
        """
        Clean up resources allocated to a process.
        
        This method is internally thread-safe.
        
        Args:
            name: Name of the process
        """
        with self._lock:
            # Get process info
            process_info = self.process_info.pop(name, None)
            if process_info:
                # Release CPU cores
                self.allocated_cpu_cores -= set(process_info.cpu_allocation)
                
                # Release GPU if allocated
                if hasattr(process_info, 'gpu_id') and process_info.gpu_id is not None:
                    self.allocated_gpu_ids.discard(process_info.gpu_id)
            
            # Remove process reference
            self.processes.pop(name, None)
    
    def monitor_processes(self, interval: float = 5.0) -> None:
        """
        Start monitoring the health of all processes.
        
        This method is thread-safe.
        
        Args:
            interval: Monitoring interval in seconds
        """
        def _monitor() -> None:
            while True:
                with self._lock:
                    current_time = time.time()
                    for name, proc_ref in list(self.processes.items()):
                        if proc_ref and proc_ref():
                            process = proc_ref()
                            # Check if process is alive
                            if not process.is_alive():
                                self.logger.error(f"Process {name} died unexpectedly.")
                                self._cleanup_process_resources(name)
                        else:
                            # Process reference is dead
                            self.logger.warning(f"Process reference for {name} is invalid.")
                            self._cleanup_process_resources(name)
                            
                        # Check heartbeat for zombie processes
                        if name in self.process_info:
                            process_info = self.process_info[name]
                            if current_time - process_info.last_heartbeat > interval * 3:
                                self.logger.warning(f"Process {name} heartbeat timeout. Marking as dead.")
                                self._cleanup_process_resources(name)
                
                time.sleep(interval)
        
        self.start_thread("process_monitor", _monitor)
    
    def get_process_info(self, name: str) -> Optional[ProcessInfo]:
        """
        Get information about a running process.
        
        This method is thread-safe.
        
        Args:
            name: Name of the process
            
        Returns:
            ProcessInfo if the process exists, None otherwise
        """
        with self._lock:
            return self.process_info.get(name)
    
    def update_process_heartbeat(self, name: str) -> bool:
        """
        Update the heartbeat for a process.
        
        This method is thread-safe and should be called periodically by processes.
        
        Args:
            name: Name of the process
            
        Returns:
            True if the heartbeat was updated, False otherwise
        """
        with self._lock:
            if name in self.process_info:
                self.process_info[name].last_heartbeat = time.time()
                return True
            return False
    
    def initialize_critical_structures(self) -> bool:
        """
        Initialize critical data structures required by FEAGI.
        
        This method is thread-safe.
        
        Returns:
            True if initialization was successful, False otherwise
        """
        with self._lock:
            self.logger.info("Initializing critical data structures...")
            try:
                # Placeholder for actual initialization logic
                # This will be implemented as the specific data structures are defined
                return True
            except Exception as e:
                self.logger.error(f"Failed to initialize critical data structures: {e}")
                return False
    
    def shutdown(self) -> None:
        """
        Shutdown all processes and threads.
        
        This method is thread-safe.
        """
        with self._lock:
            self.logger.info("Shutting down FEAGI Resource Manager...")
            
            # Terminate all processes
            for name in list(self.processes.keys()):
                self.terminate_process(name)
            
            # No explicit termination for threads as they are daemons
            self.logger.info("Shutdown complete.") 