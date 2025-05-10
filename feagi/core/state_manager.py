"""
FEAGI Global State Manager

Provides a high-performance memory-mapped state management system
for tracking FEAGI's internal states with near-zero overhead access.
"""

import ctypes
import mmap
import os
from enum import IntEnum
from typing import Optional
import time


# ===== State Definitions =====
class GenomeState(IntEnum):
    MISSING = 0
    LOADING = 1
    LOADED = 2
    SAVING = 3
    ERROR = 4


class ConnectomeState(IntEnum):
    MISSING = 0
    INITIALIZING = 1
    UPDATING = 2
    READY = 3
    SNAPSHOTTING = 4
    ERROR = 5


class ServiceState(IntEnum):
    UNAVAILABLE = 0
    INITIALIZING = 1
    READY = 2
    DEGRADED = 3
    ERROR = 4


class SimulationState(IntEnum):
    STOPPED = 0
    PAUSED = 1
    RUNNING = 2
    STEPPING = 3


# ===== Raw Memory Structure =====
class FeagiStateStruct(ctypes.Structure):
    _fields_ = [
        ("genome_state", ctypes.c_uint8),
        ("connectome_state", ctypes.c_uint8),
        ("api_state", ctypes.c_uint8),
        ("zmq_state", ctypes.c_uint8),
        ("agent_count", ctypes.c_uint32),
        ("burst_engine_state", ctypes.c_uint8),
        ("burst_frequency", ctypes.c_float),
        ("simulation_state", ctypes.c_uint8),
        ("fcl_sampler_state", ctypes.c_uint8),
        ("fcl_sampler_frequency", ctypes.c_float),
        ("fcl_sampler_consumer", ctypes.c_uint8),
        ("state_version", ctypes.c_uint64),
    ]


class FeagiStateManager:
    _instance = None
    _default_dir = "/tmp"

    @classmethod
    def instance(cls, path: Optional[str] = None):
        """Singleton accessor for the state manager"""
        if cls._instance is None:
            if path is None:
                timestamp = int(time.time())
                path = f"{cls._default_dir}/feagi_state_{timestamp}.bin"
            cls._instance = cls(path)
        return cls._instance

    def __init__(self, path: str):
        """Initialize state manager with memory mapping to specified file path"""
        size = ctypes.sizeof(FeagiStateStruct)
        # Create file if it doesn't exist or resize if too small
        if not os.path.exists(path) or os.path.getsize(path) != size:
            with open(path, "wb") as f:
                f.write(b"\0" * size)
        self.file = open(path, "r+b")
        self.mm = mmap.mmap(self.file.fileno(), size)
        self.state_ptr = ctypes.pointer(
            FeagiStateStruct.from_buffer(self.mm)
        )
        self.path = path

    def cleanup(self):
        """Clean up resources and delete the state file on shutdown"""
        try:
            if hasattr(self, 'mm') and self.mm:
                self.mm.close()
            if hasattr(self, 'file') and self.file:
                self.file.close()
            if os.path.exists(self.path):
                os.remove(self.path)
        except Exception:
            pass

    def __del__(self):
        self.cleanup()

    # ===== Genome State =====
    def get_genome_state(self) -> GenomeState:
        """Get current genome state as enum value"""
        raw_value = self.state_ptr.contents.genome_state
        return GenomeState(raw_value)
    
    def set_genome_state(self, state: GenomeState) -> None:
        """Set genome state using enum value"""
        self.state_ptr.contents.genome_state = int(state)
        self.state_ptr.contents.state_version += 1
    
    # ===== Connectome State =====
    def get_connectome_state(self) -> ConnectomeState:
        """Get current connectome state as enum value"""
        raw_value = self.state_ptr.contents.connectome_state
        return ConnectomeState(raw_value)
    
    def set_connectome_state(self, state: ConnectomeState) -> None:
        """Set connectome state using enum value"""
        self.state_ptr.contents.connectome_state = int(state)
        self.state_ptr.contents.state_version += 1

    # ===== API State =====
    def get_api_state(self) -> ServiceState:
        """Get current API state as enum value"""
        raw_value = self.state_ptr.contents.api_state
        return ServiceState(raw_value)
    
    def set_api_state(self, state: ServiceState) -> None:
        """Set API state using enum value"""
        self.state_ptr.contents.api_state = int(state)
        self.state_ptr.contents.state_version += 1

    # ===== ZMQ State =====
    def get_zmq_state(self) -> ServiceState:
        """Get current ZMQ state as enum value"""
        raw_value = self.state_ptr.contents.zmq_state
        return ServiceState(raw_value)
    
    def set_zmq_state(self, state: ServiceState) -> None:
        """Set ZMQ state using enum value"""
        self.state_ptr.contents.zmq_state = int(state)
        self.state_ptr.contents.state_version += 1

    # ===== Agent Count =====
    def get_agent_count(self) -> int:
        """Get current number of registered agents"""
        return self.state_ptr.contents.agent_count
    
    def set_agent_count(self, count: int) -> None:
        """Set current number of registered agents"""
        self.state_ptr.contents.agent_count = count
        self.state_ptr.contents.state_version += 1

    # ===== Burst Engine State =====
    def get_burst_engine_state(self) -> ServiceState:
        """Get current burst engine state as enum value"""
        raw_value = self.state_ptr.contents.burst_engine_state
        return ServiceState(raw_value)
    
    def set_burst_engine_state(self, state: ServiceState) -> None:
        """Set burst engine state using enum value"""
        self.state_ptr.contents.burst_engine_state = int(state)
        self.state_ptr.contents.state_version += 1

    # ===== Burst Frequency =====
    def get_burst_frequency(self) -> float:
        """Get current burst frequency in Hz"""
        return self.state_ptr.contents.burst_frequency
    
    def set_burst_frequency(self, frequency: float) -> None:
        """Set current burst frequency in Hz"""
        self.state_ptr.contents.burst_frequency = frequency
        self.state_ptr.contents.state_version += 1

    # ===== Simulation State =====
    def get_simulation_state(self) -> SimulationState:
        """Get current simulation state as enum value"""
        raw_value = self.state_ptr.contents.simulation_state
        return SimulationState(raw_value)
    
    def set_simulation_state(self, state: SimulationState) -> None:
        """Set simulation state using enum value"""
        self.state_ptr.contents.simulation_state = int(state)
        self.state_ptr.contents.state_version += 1
        
    # ===== FCLSampler State =====
    def get_fcl_sampler_state(self) -> ServiceState:
        """Get current FCLSampler state as enum value"""
        raw_value = self.state_ptr.contents.fcl_sampler_state
        return ServiceState(raw_value)
    def set_fcl_sampler_state(self, state: ServiceState) -> None:
        """Set FCLSampler state using enum value"""
        self.state_ptr.contents.fcl_sampler_state = int(state)
        self.state_ptr.contents.state_version += 1

    # ===== FCLSampler Frequency =====
    def get_fcl_sampler_frequency(self) -> float:
        """Get current FCLSampler frequency in Hz"""
        return self.state_ptr.contents.fcl_sampler_frequency
    def set_fcl_sampler_frequency(self, frequency: float) -> None:
        """Set FCLSampler frequency in Hz"""
        self.state_ptr.contents.fcl_sampler_frequency = frequency
        self.state_ptr.contents.state_version += 1

    # ===== FCLSampler Consumer =====
    def get_fcl_sampler_consumer(self) -> int:
        """Get current FCLSampler consumer code (1=Visualization, 2=Motor, 3=Both, etc.)"""
        return self.state_ptr.contents.fcl_sampler_consumer
    def set_fcl_sampler_consumer(self, consumer: int) -> None:
        """Set FCLSampler consumer code (1=Visualization, 2=Motor, 3=Both, etc.)"""
        self.state_ptr.contents.fcl_sampler_consumer = consumer
        self.state_ptr.contents.state_version += 1

    # ===== State Version =====
    def get_state_version(self) -> int:
        """Get current state version (increments on any state change)"""
        return self.state_ptr.contents.state_version
        
    # ===== Disk Operations =====
    def sync_to_disk(self) -> None:
        """Force state to be written to disk"""
        self.mm.flush()
        
    # ===== High-level status helpers =====
    def is_genome_loaded(self) -> bool:
        """Check if genome is fully loaded"""
        return self.get_genome_state() == GenomeState.LOADED
        
    def is_connectome_ready(self) -> bool:
        """Check if connectome is ready for operation"""
        return self.get_connectome_state() == ConnectomeState.READY
        
    def is_burst_engine_ready(self) -> bool:
        """Check if burst engine is ready"""
        return self.get_burst_engine_state() == ServiceState.READY
        
    def is_simulation_running(self) -> bool:
        """Check if simulation is currently running"""
        return self.get_simulation_state() == SimulationState.RUNNING 