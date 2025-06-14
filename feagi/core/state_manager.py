"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
FEAGI Global State Manager

Provides a high-performance memory-mapped state management system
for tracking FEAGI's internal states with near-zero overhead access.

Logging Patterns:
- All state change logs should go through _log_state_change function
- Never call logger directly with emoji parameter outside of _log_state_change
- State transitions are logged with appropriate emojis
"""

import ctypes
import datetime
import mmap
import os
import tempfile
import threading
import time
from contextlib import contextmanager
from enum import Enum, IntEnum
from pathlib import Path
from typing import Optional

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


# def feagi_logger(name="app", level=logging.DEBUG):
#     LEVEL_MAP = {
#         "DEBUG":    "DEBUG   ",
#         "INFO":     "INFO    ",
#         "WARNING":  "WARNING ",
#         "ERROR":    "ERROR   ",
#         "CRITICAL": "CRITICL ",
#     }
#
#     class BuiltinFormatter(logging.Formatter):
#         def format(self, record):
#             emoji1 = getattr(record, 'emoji1', '')
#             emoji2 = getattr(record, 'emoji2', '')
#             # Ensure emoji block is 4 characters wide
#             emoji_block = f"{emoji1}{emoji2}".ljust(4)
#
#             level = LEVEL_MAP.get(record.levelname, record.levelname.ljust(8))
#             timestamp = self.formatTime(record, self.datefmt)
#             message = record.getMessage()
#
#             return f"{emoji_block}{level} {timestamp} {message}"
#
#     class EmojiAdapter(logging.LoggerAdapter):
#         def process(self, msg, kwargs):
#             emoji1 = kwargs.pop('emoji1', '')
#             emoji2 = kwargs.pop('emoji2', '')
#             kwargs.setdefault('extra', {})['emoji1'] = emoji1
#             kwargs['extra']['emoji2'] = emoji2
#             return msg, kwargs
#
#     logger = logging.getLogger(name)
#     if not logger.handlers:
#         handler = logging.StreamHandler()
#         formatter = BuiltinFormatter(datefmt="%Y-%m-%d %H:%M:%S")
#         handler.setFormatter(formatter)
#         logger.addHandler(handler)
#         logger.setLevel(level)
#
#     return EmojiAdapter(logger, {})
#
# flog = feagi_logger()


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


class ServiceState(Enum):
    UNAVAILABLE = "UNAVAILABLE"
    INITIALIZING = "INITIALIZING"
    READY = "READY"
    DEGRADED = "DEGRADED"
    ERROR = "ERROR"
    UNINITIALIZED = "UNINITIALIZED"
    FAILED = "FAILED"
    STOPPED = "STOPPED"
    SYNCING = "SYNCING"
    SYNC_COMPLETE = "SYNC_COMPLETE"
    SYNC_ERROR = "SYNC_ERROR"
    ON_HOLD = "ON_HOLD"


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
        (
            "burst_frequency",
            ctypes.c_float,
        ),  # Target/assigned frequency from genome/user
        ("simulation_state", ctypes.c_uint8),
        ("fq_sampler_state", ctypes.c_uint8),
        ("fq_sampler_frequency", ctypes.c_float),
        ("fq_sampler_consumer", ctypes.c_uint8),
        ("state_version", ctypes.c_uint64),
        ("genome_counter", ctypes.c_uint32),
        ("brain_readiness", ctypes.c_uint8),  # 0 = False, 1 = True
        ("test_visualization_mode", ctypes.c_uint8),  # 0 = False, 1 = True
        (
            "genome_timestamp",
            ctypes.c_uint64,
        ),  # Timestamp (milliseconds) when genome was last loaded/changed
        # Simple agent counts for FQ sampler compatibility (kept in binary structure)
        (
            "agents_with_visualization",
            ctypes.c_uint32,
        ),  # Count of agents with visualization capability
        ("agents_with_motor", ctypes.c_uint32),  # Count of agents with motor capability
        (
            "agents_with_sensory",
            ctypes.c_uint32,
        ),  # Count of agents with sensory capability
        (
            "last_agent_registry_update",
            ctypes.c_uint64,
        ),  # Timestamp of last agent registry change
        # Legacy fields (kept for compatibility)
        (
            "visualization_client_count",
            ctypes.c_uint32,
        ),  # Number of connected visualization clients
        ("motor_client_count", ctypes.c_uint32),  # Number of connected motor clients
        ("visualization_sampling_enabled", ctypes.c_uint8),  # 0 = False, 1 = True
        ("motor_sampling_enabled", ctypes.c_uint8),  # 0 = False, 1 = True
        # SIMD Configuration (centralized detection results)
        ("simd_available", ctypes.c_uint8),  # 0 = False, 1 = True
        (
            "simd_backend",
            ctypes.c_uint8,
        ),  # Backend enum value (0=scalar, 1=SSE2, 2=AVX, etc.)
        ("simd_vector_width", ctypes.c_uint8),  # Vector width (4, 8, 16, 32)
        ("simd_supports_avx", ctypes.c_uint8),  # 0 = False, 1 = True
        ("simd_supports_avx2", ctypes.c_uint8),  # 0 = False, 1 = True
        ("simd_supports_avx512", ctypes.c_uint8),  # 0 = False, 1 = True
        ("simd_alignment", ctypes.c_uint8),  # Memory alignment requirement (16, 32, 64)
        ("simd_initialization_timestamp", ctypes.c_uint64),  # When SIMD was detected
    ]


# Define a mapping between integer values and ServiceState values
_SERVICE_STATE_VALUES = {
    0: "UNAVAILABLE",
    1: "INITIALIZING",
    2: "READY",
    3: "DEGRADED",
    4: "ERROR",
    5: "UNINITIALIZED",
    6: "FAILED",
    7: "STOPPED",
    8: "SYNCING",
    9: "SYNC_COMPLETE",
    10: "SYNC_ERROR",
    11: "ON_HOLD",
}

# And the reverse mapping
_SERVICE_STATE_INTS = {v: k for k, v in _SERVICE_STATE_VALUES.items()}

# SIMD Backend mappings for centralized state management
_SIMD_BACKEND_VALUES = {
    0: "SCALAR",
    1: "SSE2",
    2: "AVX",
    3: "AVX2",
    4: "AVX512",
    5: "NEON",
    6: "SVE",
    7: "GPU_CUDA",
    8: "GPU_WEBGPU",
    9: "GPU_OPENCL",
}

_SIMD_BACKEND_INTS = {v: k for k, v in _SIMD_BACKEND_VALUES.items()}


# Agent Capability Flags for centralized agent registry
class AgentCapability:
    """Bit flags for agent capabilities"""

    NONE = 0
    VISUALIZATION = 1  # Can receive visualization data
    MOTOR = 2  # Can receive motor data
    SENSORY = 4  # Can send sensory data
    BRAIN_CONTROL = 8  # Can control brain parameters
    GENOME_EDIT = 16  # Can modify genome
    ALL = 31  # All capabilities combined


# Agent Type mappings
_AGENT_TYPE_VALUES = {
    0: "UNKNOWN",
    1: "BRIDGE",
    2: "CONNECTOR",
    3: "EXTERNAL",
    4: "INTERNAL",
    5: "SIMULATOR",
    6: "VISUALIZER",
}

_AGENT_TYPE_INTS = {v: k for k, v in _AGENT_TYPE_VALUES.items()}


class ServiceState(Enum):
    UNAVAILABLE = "UNAVAILABLE"
    INITIALIZING = "INITIALIZING"
    READY = "READY"
    DEGRADED = "DEGRADED"
    ERROR = "ERROR"
    UNINITIALIZED = "UNINITIALIZED"
    FAILED = "FAILED"
    STOPPED = "STOPPED"
    SYNCING = "SYNCING"
    SYNC_COMPLETE = "SYNC_COMPLETE"
    SYNC_ERROR = "SYNC_ERROR"
    ON_HOLD = "ON_HOLD"

    @classmethod
    def _missing_(cls, value):
        # Convert integers to their string values
        if isinstance(value, int) and value in _SERVICE_STATE_VALUES:
            return cls(_SERVICE_STATE_VALUES[value])
        return None

    def __int__(self):
        """Convert the enum to its integer representation."""
        for k, v in _SERVICE_STATE_VALUES.items():
            if v == self.value:
                return k
        return 0  # Default to UNAVAILABLE

    def __hash__(self):
        """Make ServiceState hashable."""
        return hash(self.value)

    def __eq__(self, other):
        """Properly compare ServiceState with other types."""
        if isinstance(other, int):
            return self.value == _SERVICE_STATE_VALUES.get(other)
        elif isinstance(other, str):
            return self.value == other
        elif isinstance(other, ServiceState):
            return self.value == other.value
        return False


class FeagiStateManager:
    """
    RUST/RTOS COMPATIBLE: High-performance memory-mapped state management system.

    This class provides near-zero overhead state synchronization between services
    using memory-mapped files. The design translates directly to Rust using:
    - memmap2 crate for memory mapping
    - std::sync::atomic for atomic operations
    - std::sync::Once for singleton pattern

    Key Rust/RTOS benefits:
    - No garbage collection interference
    - Deterministic memory access patterns
    - Lock-free state synchronization
    - Cross-process shared memory support
    """

    _instance = None
    _default_dir = tempfile.gettempdir()  # Cross-platform temp directory

    @classmethod
    def instance(cls, path: Optional[str] = None):
        """
        RUST/RTOS COMPATIBLE: Singleton accessor for the state manager.

        In Rust, this would use std::sync::Once for thread-safe initialization.
        """
        if cls._instance is None:
            if path is None:
                # CRITICAL FIX: Check for shared state file from environment (subprocess mode)
                import os

                shared_state_file = os.environ.get("FEAGI_STATE_FILE")
                if shared_state_file:
                    path = shared_state_file
                    logger.info(
                        f"[LINK] Using shared state file from environment: {path}"
                    )
                else:
                    # Create new state file (main process mode) - cross-platform temp directory
                    timestamp = int(time.time())
                    path = os.path.join(
                        cls._default_dir, f"feagi_state_{timestamp}.bin"
                    )
                    logger.info(f"🏠 Creating new state file: {path}")
            cls._instance = cls(path)
        return cls._instance

    def __init__(self, path: str):
        """
        RUST/RTOS COMPATIBLE: Initialize state manager with memory mapping.

        In Rust, this would use:
        ```rust
        use memmap2::MmapMut;
        use std::fs::OpenOptions;

        let file = OpenOptions::new().read(true).write(true).create(true).open(path)?;
        let mmap = unsafe { MmapMut::map_mut(&file)? };
        ```
        """
        import ctypes
        import os

        # Ensure the directory exists
        state_dir = Path(path).parent
        if not state_dir.exists():
            try:
                state_dir.mkdir(parents=True, exist_ok=True)
                logger.info(f"Created state directory: {state_dir}")
            except Exception as e:
                logger.error(f"Failed to create state directory {state_dir}: {e}")
                raise

        size = ctypes.sizeof(FeagiStateStruct)

        # Create file if it doesn't exist or resize if too small
        try:
            if not os.path.exists(path) or os.path.getsize(path) != size:
                with open(path, "wb") as f:
                    f.write(b"\0" * size)
                logger.debug(f"Created/resized state file: {path} ({size} bytes)")
        except Exception as e:
            logger.error(f"Failed to create state file {path}: {e}")
            raise

        # Open the file for memory mapping
        try:
            self.file = open(path, "r+b")
            self.mm = mmap.mmap(self.file.fileno(), size)
            self.state_ptr = ctypes.pointer(FeagiStateStruct.from_buffer(self.mm))
            self.path = path
        except Exception as e:
            logger.error(f"Failed to initialize memory mapping for {path}: {e}")
            if hasattr(self, "mm"):
                self.mm.close()
            if hasattr(self, "file"):
                self.file.close()
            raise

        # Add synchronization tracking
        self.genome_sync_state = ServiceState.UNINITIALIZED
        self.pending_sync_operations = []
        self.sync_observers = []

        # Add notification hooks
        self._notification_callbacks = {
            "genome": [],
            "connectome": [],
            "burst_engine": [],
            "simulation": [],
        }

        # Add frequency measurement history (in-memory only, not persisted)
        # Format: {timestamp: {"frequency_hz": float, "measurement_duration_s": float, "performance_status": str}}
        self._frequency_measurement_history = {}
        self._max_frequency_history_entries = 100  # Keep last 100 measurements

        # Comprehensive agent registry (not in binary structure - Python only)
        self._agent_registry = {
            "connected_visualization_agents": set(),
            "connected_sensorimotor_agents": set(),
            "agent_properties": {},
        }
        self._agent_registry_lock = threading.Lock()

        # Debug configuration (RTOS-compatible, in-memory only)
        self._debug_config = {
            "api": False,
            "npu": False,
            "zmq_outbound": False,
            "zmq_inbound": False,
        }

    def cleanup(self):
        """Clean up resources and delete the state file on shutdown"""
        try:
            if hasattr(self, "mm") and self.mm:
                self.mm.close()
            if hasattr(self, "file") and self.file:
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
        old = GenomeState(self.state_ptr.contents.genome_state)
        self.state_ptr.contents.genome_state = int(state)
        self.state_ptr.contents.state_version += 1
        self._log_state_change("GenomeState", old, state)
        self._notify_state_change("genome", old, state)

    # ===== Connectome State =====
    def get_connectome_state(self) -> ConnectomeState:
        """Get current connectome state as enum value"""
        raw_value = self.state_ptr.contents.connectome_state
        return ConnectomeState(raw_value)

    def set_connectome_state(self, state: ConnectomeState) -> None:
        """Set connectome state using enum value"""
        old = ConnectomeState(self.state_ptr.contents.connectome_state)
        self.state_ptr.contents.connectome_state = int(state)
        self.state_ptr.contents.state_version += 1
        self._log_state_change("ConnectomeState", old, state)
        self._notify_state_change("connectome", old, state)

    # ===== API State =====
    def get_api_state(self) -> ServiceState:
        """Get the current API service state."""
        raw_value = self.state_ptr.contents.api_state
        return ServiceState(_SERVICE_STATE_VALUES.get(raw_value, "UNAVAILABLE"))

    def set_api_state(self, state: ServiceState) -> None:
        """Set the API service state."""
        self._verify_enum(state, ServiceState)
        old_state = self.get_api_state()
        self.state_ptr.contents.api_state = int(state)
        self.state_ptr.contents.state_version += 1
        self._log_state_change("APIState", old_state, state)
        self._notify_state_change("API", old_state, state)

    # ===== ZMQ State =====
    def get_zmq_state(self) -> ServiceState:
        """Get the current ZMQ service state."""
        raw_value = self.state_ptr.contents.zmq_state
        return ServiceState(_SERVICE_STATE_VALUES.get(raw_value, "UNAVAILABLE"))

    def set_zmq_state(self, state: ServiceState) -> None:
        """Set the ZMQ service state."""
        self._verify_enum(state, ServiceState)
        old_state = self.get_zmq_state()
        self.state_ptr.contents.zmq_state = int(state)
        self.state_ptr.contents.state_version += 1
        self._log_state_change("ZMQState", old_state, state)
        self._notify_state_change("ZMQ", old_state, state)

    # ===== Agent Count =====
    def get_agent_count(self) -> int:
        """Get current number of registered agents"""
        return self.state_ptr.contents.agent_count

    def set_agent_count(self, count: int) -> None:
        """Set current number of registered agents"""
        old_count = self.state_ptr.contents.agent_count
        self.state_ptr.contents.agent_count = count
        self.state_ptr.contents.state_version += 1
        self._log_state_change("AgentCount", old_count, count)

    # ===== Comprehensive Agent Registry =====
    def register_agent(
        self,
        agent_id: str,
        agent_type: str = None,
        capabilities: dict = None,
        agent_data_port: int = None,
        agent_version: str = None,
        controller_version: str = None,
        agent_ip: str = None,
    ) -> None:
        """
        Register an agent with full capability structure and metadata.

        Args:
            agent_id: Unique identifier for the agent
            agent_type: Type of agent (optional, will be determined from capabilities if not provided)
            capabilities: Full capabilities dictionary structure
            agent_data_port: Port number for agent data communication
            agent_version: Version of the agent software
            controller_version: Version of the controller software
            agent_ip: IP address of the agent (optional)
        """
        with self._agent_registry_lock:
            timestamp = time.time()

            # Determine agent type from capabilities if not provided
            if not agent_type and capabilities:
                agent_type = self._determine_agent_type(capabilities)
            elif not agent_type:
                agent_type = "unknown"

            # Generate router address if IP and port are provided
            agent_router_address = None
            if agent_ip and agent_data_port:
                agent_router_address = f"tcp://{agent_ip}:{agent_data_port}"

            # Add to agent properties with full metadata
            self._agent_registry["agent_properties"][agent_id] = {
                "capabilities": capabilities or {},
                "type": agent_type,
                "agent_data_port": agent_data_port,
                "agent_version": agent_version,
                "controller_version": controller_version,
                "agent_ip": agent_ip,
                "agent_router_address": agent_router_address,
                "connection_log": [{"event": "connected", "timestamp": timestamp}],
                "last_activity": timestamp,
                "registration_timestamp": timestamp,
            }

            # Add to appropriate connected agent sets based on capabilities
            if self._has_visualization_capabilities(capabilities or {}):
                self._agent_registry["connected_visualization_agents"].add(agent_id)

            if self._has_sensorimotor_capabilities(capabilities or {}):
                self._agent_registry["connected_sensorimotor_agents"].add(agent_id)

            # Update binary structure counts for FQ sampler compatibility
            self._update_agent_counts()
            self.state_ptr.contents.last_agent_registry_update = int(timestamp * 1000)
            self.state_ptr.contents.state_version += 1

            logger.info(
                f"Registered agent {agent_id} (type: {agent_type}, port: {agent_data_port})"
            )

    def unregister_agent(self, agent_id: str) -> None:
        """
        Unregister an agent and remove from all tracking.

        Args:
            agent_id: Unique identifier for the agent to remove
        """
        with self._agent_registry_lock:
            if agent_id not in self._agent_registry["agent_properties"]:
                logger.warning(f"Attempted to unregister unknown agent: {agent_id}")
                return

            timestamp = time.time()

            # Log disconnection
            self._agent_registry["agent_properties"][agent_id]["connection_log"].append(
                {"event": "disconnected", "timestamp": timestamp}
            )

            # Remove from connected sets
            self._agent_registry["connected_visualization_agents"].discard(agent_id)
            self._agent_registry["connected_sensorimotor_agents"].discard(agent_id)

            # Remove from agent properties
            del self._agent_registry["agent_properties"][agent_id]

            # Update binary structure counts
            self._update_agent_counts()
            self.state_ptr.contents.last_agent_registry_update = int(timestamp * 1000)
            self.state_ptr.contents.state_version += 1

            logger.info(f"Unregistered agent {agent_id}")

    def get_connected_agents(self, capability_type: str = None) -> set:
        """
        Get set of connected agent IDs, optionally filtered by capability type.

        Args:
            capability_type: Optional filter ("visualization", "sensorimotor", None for all)

        Returns:
            Set of agent IDs
        """
        with self._agent_registry_lock:
            if capability_type == "visualization":
                return self._agent_registry["connected_visualization_agents"].copy()
            elif capability_type == "sensorimotor":
                return self._agent_registry["connected_sensorimotor_agents"].copy()
            else:
                # Return all connected agents
                all_agents = set()
                all_agents.update(
                    self._agent_registry["connected_visualization_agents"]
                )
                all_agents.update(self._agent_registry["connected_sensorimotor_agents"])
                return all_agents

    def get_agent_properties(self, agent_id: str) -> dict:
        """
        Get full properties for a specific agent.

        Args:
            agent_id: Agent identifier

        Returns:
            Dictionary with agent properties or None if not found
        """
        with self._agent_registry_lock:
            return self._agent_registry["agent_properties"].get(agent_id, {}).copy()

    def get_agent_registry_summary(self) -> dict:
        """
        Get comprehensive summary of agent registry state.

        Returns:
            Dictionary with registry state including counts and agent lists
        """
        with self._agent_registry_lock:
            return {
                "connected_visualization_agents": list(
                    self._agent_registry["connected_visualization_agents"]
                ),
                "connected_sensorimotor_agents": list(
                    self._agent_registry["connected_sensorimotor_agents"]
                ),
                "total_agents": len(self._agent_registry["agent_properties"]),
                "agent_count_viz": len(
                    self._agent_registry["connected_visualization_agents"]
                ),
                "agent_count_sensorimotor": len(
                    self._agent_registry["connected_sensorimotor_agents"]
                ),
                "last_update": self.state_ptr.contents.last_agent_registry_update,
            }

    # FQ Sampler compatibility methods (use binary structure counts)
    def has_visualization_agents(self) -> bool:
        """Check if any agents with visualization capability are connected"""
        return self.state_ptr.contents.agents_with_visualization > 0

    def has_motor_agents(self) -> bool:
        """Check if any agents with motor capability are connected (alias for sensorimotor)"""
        return self.state_ptr.contents.agents_with_motor > 0

    def has_sensory_agents(self) -> bool:
        """Check if any agents with sensory capability are connected"""
        return self.state_ptr.contents.agents_with_sensory > 0

    def get_agents_with_visualization(self) -> int:
        """Get count of agents with visualization capability"""
        return self.state_ptr.contents.agents_with_visualization

    def get_agents_with_motor(self) -> int:
        """Get count of agents with motor capability"""
        return self.state_ptr.contents.agents_with_motor

    def get_agents_with_sensory(self) -> int:
        """Get count of agents with sensory capability"""
        return self.state_ptr.contents.agents_with_sensory

    # Helper methods
    def _has_visualization_capabilities(self, capabilities: dict) -> bool:
        """Check if capabilities dictionary indicates visualization support"""
        if isinstance(capabilities, dict):
            return "visualization" in capabilities
        return False

    def _has_sensorimotor_capabilities(self, capabilities: dict) -> bool:
        """Check if capabilities dictionary indicates sensorimotor support"""
        # Look for input/output capabilities in the structure
        if isinstance(capabilities, dict):
            return "input" in capabilities or "output" in capabilities
        return False

    def _determine_agent_type(self, capabilities: dict) -> str:
        """Determine agent type based on capabilities structure"""
        if not isinstance(capabilities, dict):
            return "unknown"

        if "visualization" in capabilities:
            return "visualization"
        elif "input" in capabilities or "output" in capabilities:
            return "sensorimotor"
        elif "timeseries_database" in capabilities:
            return "database"
        else:
            return "unknown"

    def _update_agent_counts(self) -> None:
        """Update binary structure counts based on current registry state"""
        self.state_ptr.contents.agents_with_visualization = len(
            self._agent_registry["connected_visualization_agents"]
        )
        self.state_ptr.contents.agents_with_motor = len(
            self._agent_registry["connected_sensorimotor_agents"]
        )
        self.state_ptr.contents.agents_with_sensory = len(
            self._agent_registry["connected_sensorimotor_agents"]
        )
        self.state_ptr.contents.agent_count = len(
            self._agent_registry["agent_properties"]
        )

    # ===== Burst Engine State =====
    def get_burst_engine_state(self) -> ServiceState:
        """Get the current burst engine state."""
        raw_value = self.state_ptr.contents.burst_engine_state
        return ServiceState(_SERVICE_STATE_VALUES.get(raw_value, "UNAVAILABLE"))

    def set_burst_engine_state(self, state: ServiceState) -> None:
        """Set the burst engine state."""
        self._verify_enum(state, ServiceState)
        old_state = self.get_burst_engine_state()
        # Convert to int for storage
        int_value = 0  # Default to UNAVAILABLE
        for k, v in _SERVICE_STATE_VALUES.items():
            if v == state.value:
                int_value = k
                break

        self.state_ptr.contents.burst_engine_state = int_value
        self.state_ptr.contents.state_version += 1
        self._log_state_change("BurstEngineState", old_state, state)
        # Use the category key from the notification callbacks dict
        self._notify_state_change("burst_engine", old_state, state)

    # ===== Burst Frequency =====
    def get_burst_frequency(self) -> float:
        """Get current target/assigned burst frequency in Hz (from genome/user settings)"""
        return self.state_ptr.contents.burst_frequency

    def set_burst_frequency(self, frequency: float) -> None:
        """Set target/assigned burst frequency in Hz (from genome/user settings)"""
        old_frequency = self.state_ptr.contents.burst_frequency
        self.state_ptr.contents.burst_frequency = frequency
        self.state_ptr.contents.state_version += 1

        # Always log frequency changes since this is important for monitoring
        self._log_state_change(
            "BurstFrequency", f"{old_frequency:.1f}Hz", f"{frequency:.1f}Hz"
        )

    # ===== Simulation State =====
    def get_simulation_state(self) -> SimulationState:
        """Get current simulation state as enum value"""
        raw_value = self.state_ptr.contents.simulation_state
        return SimulationState(raw_value)

    def set_simulation_state(self, state: SimulationState) -> None:
        """Set simulation state using enum value"""
        old = SimulationState(self.state_ptr.contents.simulation_state)
        self.state_ptr.contents.simulation_state = int(state)
        self.state_ptr.contents.state_version += 1
        self._log_state_change("SimulationState", old, state)
        self._notify_state_change("simulation", old, state)

    # ===== FQSampler State =====
    def get_fq_sampler_state(self) -> ServiceState:
        """Get the current FQ sampler state."""
        raw_value = self.state_ptr.contents.fq_sampler_state
        return ServiceState(_SERVICE_STATE_VALUES.get(raw_value, "UNAVAILABLE"))

    def set_fq_sampler_state(self, state: ServiceState) -> None:
        """Set the FQ sampler state."""
        self._verify_enum(state, ServiceState)
        old_state = self.get_fq_sampler_state()
        self.state_ptr.contents.fq_sampler_state = int(state)
        self.state_ptr.contents.state_version += 1
        self._log_state_change("FQSamplerState", old_state, state)
        self._notify_state_change("FQ Sampler", old_state, state)

    # ===== FQSampler Frequency =====
    def get_fq_sampler_frequency(self) -> float:
        """Get current FQSampler frequency in Hz"""
        return self.state_ptr.contents.fq_sampler_frequency

    def set_fq_sampler_frequency(self, frequency: float) -> None:
        """Set FQSampler frequency in Hz"""
        old_frequency = self.state_ptr.contents.fq_sampler_frequency
        self.state_ptr.contents.fq_sampler_frequency = frequency
        self.state_ptr.contents.state_version += 1
        self._log_state_change(
            "FQSamplerFrequency", f"{old_frequency:.1f}Hz", f"{frequency:.1f}Hz"
        )

    # ===== FQSampler Consumer =====
    def get_fq_sampler_consumer(self) -> int:
        """Get current FQSampler consumer code (1=Visualization, 2=Motor, 3=Both, etc.)"""
        return self.state_ptr.contents.fq_sampler_consumer

    def set_fq_sampler_consumer(self, consumer: int) -> None:
        """Set FQSampler consumer code (1=Visualization, 2=Motor, 3=Both, etc.)"""
        old_consumer = self.state_ptr.contents.fq_sampler_consumer
        self.state_ptr.contents.fq_sampler_consumer = consumer
        self.state_ptr.contents.state_version += 1
        consumer_names = {1: "Visualization", 2: "Motor", 3: "Both", 0: "None"}
        old_name = consumer_names.get(old_consumer, f"Code{old_consumer}")
        new_name = consumer_names.get(consumer, f"Code{consumer}")
        self._log_state_change("FQSamplerConsumer", old_name, new_name)

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
        """Check if genome is in LOADED state"""
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

    def get_genome_counter(self) -> int:
        """Get the current genome counter value"""
        return self.state_ptr.contents.genome_counter

    def increment_genome_counter(self) -> None:
        """Increment the genome counter by 1"""
        old_counter = self.state_ptr.contents.genome_counter
        self.state_ptr.contents.genome_counter += 1
        self.state_ptr.contents.state_version += 1
        self._log_state_change(
            "GenomeCounter", old_counter, self.state_ptr.contents.genome_counter
        )
        self.sync_to_disk()

    def get_brain_readiness(self) -> bool:
        """Get the brain readiness flag (True if brain is ready)"""
        return bool(self.state_ptr.contents.brain_readiness)

    def set_brain_readiness(self, ready: bool) -> None:
        """Set the brain readiness flag (True if brain is ready)"""
        old = bool(self.state_ptr.contents.brain_readiness)
        self.state_ptr.contents.brain_readiness = 1 if ready else 0
        self.state_ptr.contents.state_version += 1
        self._log_state_change("BrainReadiness", old, ready)

    def get_genome_timestamp(self) -> int:
        """Get the genome timestamp (milliseconds since epoch when genome was last loaded/changed)"""
        return self.state_ptr.contents.genome_timestamp

    def set_genome_timestamp(self, timestamp: int) -> None:
        """Set the genome timestamp (milliseconds since epoch when genome was last loaded/changed)"""
        old = self.state_ptr.contents.genome_timestamp
        self.state_ptr.contents.genome_timestamp = timestamp
        self.state_ptr.contents.state_version += 1
        # Convert timestamps to readable format for logging
        old_readable = (
            datetime.datetime.fromtimestamp(old / 1000).strftime("%Y-%m-%d %H:%M:%S")
            if old > 0
            else "None"
        )
        new_readable = datetime.datetime.fromtimestamp(timestamp / 1000).strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        self._log_state_change("GenomeTimestamp", old_readable, new_readable)

    def get_test_visualization_mode(self) -> bool:
        """Get the test visualization mode flag (True if test visualization is enabled)"""
        return bool(self.state_ptr.contents.test_visualization_mode)

    def set_test_visualization_mode(self, enabled: bool) -> None:
        """
        Set the test visualization mode flag.

        When enabled, visualization data will be logged for debugging purposes
        even if no ZMQ clients are connected.

        Args:
            enabled: True to enable test visualization mode, False to disable
        """
        old = bool(self.state_ptr.contents.test_visualization_mode)
        self.state_ptr.contents.test_visualization_mode = 1 if enabled else 0
        self.state_ptr.contents.state_version += 1
        if old != enabled:
            self._log_state_change("TestVisualizationMode", old, enabled)

    def get_connectome(self):
        """Get the current connectome instance"""
        try:
            # Check if embedded mode is enabled
            import os

            if os.environ.get("FEAGI_EMBEDDED_MODE", "0") == "1":
                logger.debug("Embedded mode: Skipping connectome dependency injection")
                return None

            from feagi.api.rest.dependencies import get_connectome

            return get_connectome()
        except (ImportError, RuntimeError):
            logger.warning("Failed to get connectome from dependencies")
            return None

    def register_sync_observer(self, observer):
        """Register an observer for genome sync events"""
        self.sync_observers.append(observer)

    def set_genome_sync_state(self, state, details=None):
        """Update the genome synchronization state"""
        old_state = self.genome_sync_state
        self.genome_sync_state = state
        logger.info(
            f"Genome-Connectome sync state changed: {old_state} → {state}",
            status="[PROC]",
        )

        # Notify observers
        for observer in self.sync_observers:
            observer.on_sync_state_change(old_state, state, details)

    def begin_genome_transaction(self):
        """Begin a new genome modification transaction"""
        from feagi.core.genome_transaction import GenomeTransaction

        self.set_genome_sync_state(ServiceState.SYNCING)
        return GenomeTransaction(self)

    def begin_genome_transaction_context(self):
        """Context manager for genome transactions"""
        from feagi.core.genome_transaction import GenomeTransaction

        @contextmanager
        def transaction_context():
            transaction = GenomeTransaction(self)
            try:
                yield transaction
                transaction.commit()
            except Exception:
                transaction.rollback()
                raise

        return transaction_context()

    def register_notification_callback(self, state_type: str, callback):
        """Register a callback to be notified when a particular state changes.

        Args:
            state_type: The state to monitor (e.g. "genome", "connectome")
            callback: Function to call when state changes
        """
        if state_type in self._notification_callbacks:
            self._notification_callbacks[state_type].append(callback)
            return True
        return False

    def _notify_state_change(self, state_type, old_state, new_state):
        """
        Notify all registered callbacks about a state change.

        This method intentionally avoids using the emoji parameter in logging
        to ensure compatibility with all loggers.

        Args:
            state_type: The type of state that changed (e.g., "genome")
            old_state: The previous state
            new_state: The new state
        """
        if state_type in self._notification_callbacks:
            for callback in self._notification_callbacks[state_type]:
                try:
                    callback(old_state, new_state)
                except Exception as e:
                    # Do NOT use the emoji parameter here
                    logger.error(f"[WARN] Error in notification callback: {e}")

    def _verify_enum(self, state, enum_type):
        if not isinstance(state, enum_type):
            raise ValueError(f"{state} is not a valid {enum_type.__name__}")

    # ===== Frequency Measurement History =====
    def add_frequency_measurement(
        self,
        actual_frequency_hz: float,
        potential_frequency_hz: float,
        measurement_duration_s: float,
        metadata: Optional[dict] = None,
    ) -> None:
        """
        Add a frequency measurement to the history.

        Args:
            actual_frequency_hz: The measured actual frequency including delays to maintain target
            potential_frequency_hz: The maximum potential frequency without artificial delays
            measurement_duration_s: How long the measurement took to complete
            metadata: Optional additional measurement metadata
        """
        import time

        timestamp = time.time()
        target_frequency = self.get_burst_frequency()

        # Calculate performance ratios and statuses
        actual_ratio = (
            actual_frequency_hz / target_frequency if target_frequency > 0 else 0.0
        )
        potential_ratio = (
            potential_frequency_hz / target_frequency if target_frequency > 0 else 0.0
        )

        # Assess performance status based on actual frequency
        if actual_ratio >= 0.95:
            status = "OPTIMAL"  # Within 5% of target
        elif actual_ratio >= 0.8:
            status = "GOOD"  # Within 20% of target
        elif actual_ratio >= 0.5:
            status = "DEGRADED"  # 50-80% of target
        else:
            status = "POOR"  # Below 50% of target

        # Assess system capability based on potential frequency
        if potential_ratio >= 2.0:
            capability = "HIGH"  # Can run at 2x+ target
        elif potential_ratio >= 1.5:
            capability = "GOOD"  # Can run at 1.5x+ target
        elif potential_ratio >= 1.0:
            capability = "ADEQUATE"  # Can meet target
        else:
            capability = "LIMITED"  # Cannot meet target even at full speed

        measurement_entry = {
            "actual_frequency_hz": actual_frequency_hz,
            "potential_frequency_hz": potential_frequency_hz,
            "target_frequency_hz": target_frequency,
            "actual_performance_ratio": actual_ratio,
            "potential_performance_ratio": potential_ratio,
            "performance_status": status,
            "system_capability": capability,
            "frequency_gap_hz": target_frequency - actual_frequency_hz,
            "potential_headroom_hz": potential_frequency_hz - target_frequency,
            "measurement_duration_s": measurement_duration_s,
            "metadata": metadata or {},
        }

        # Add to history
        self._frequency_measurement_history[timestamp] = measurement_entry

        # Maintain history size limit
        if (
            len(self._frequency_measurement_history)
            > self._max_frequency_history_entries
        ):
            # Remove oldest entries
            oldest_timestamps = sorted(self._frequency_measurement_history.keys())[
                : -self._max_frequency_history_entries
            ]
            for old_timestamp in oldest_timestamps:
                del self._frequency_measurement_history[old_timestamp]

        # Update state version
        self.state_ptr.contents.state_version += 1

        # Only log detailed frequency measurements when debugging NPU
        if self.is_debug_npu_enabled():
            logger.info(
                f"Frequency measurement recorded - Actual: {actual_frequency_hz:.1f}Hz, Potential: {potential_frequency_hz:.1f}Hz ({status})",
                status="[STATS]",
            )
        else:
            # For normal operation, only log significant measurements or changes
            if status in ["POOR", "DEGRADED"] or (
                hasattr(self, "_last_logged_status")
                and self._last_logged_status != status
            ):
                logger.info(
                    f"Performance status: {status} - Actual: {actual_frequency_hz:.1f}Hz",
                    status="[STATS]",
                )
                self._last_logged_status = status

    def get_frequency_measurement_history(self, limit: Optional[int] = None) -> dict:
        """
        Get frequency measurement history.

        Args:
            limit: Maximum number of recent measurements to return (None for all)

        Returns:
            Dictionary with timestamps as keys and measurement data as values
        """
        if limit is None:
            return self._frequency_measurement_history.copy()

        # Return most recent measurements
        sorted_timestamps = sorted(
            self._frequency_measurement_history.keys(), reverse=True
        )
        limited_timestamps = sorted_timestamps[:limit]

        return {
            ts: self._frequency_measurement_history[ts] for ts in limited_timestamps
        }

    def get_latest_frequency_measurement(self) -> Optional[dict]:
        """
        Get the most recent frequency measurement.

        Returns:
            Latest measurement data or None if no measurements exist
        """
        if not self._frequency_measurement_history:
            return None

        latest_timestamp = max(self._frequency_measurement_history.keys())
        latest_measurement = self._frequency_measurement_history[
            latest_timestamp
        ].copy()
        latest_measurement["timestamp"] = latest_timestamp

        return latest_measurement

    def trigger_frequency_measurement(
        self, measurement_duration_s: float = 5.0, sample_count: int = 100
    ) -> dict:
        """
        Trigger an on-demand frequency measurement via the burst engine.

        This method is expensive and should only be called when needed for monitoring
        or debugging purposes. It will enable frequency measurement in the burst engine
        for a specified duration, then calculate the average actual frequency.

        Args:
            measurement_duration_s: How long to measure frequency (default 5 seconds)
            sample_count: Number of burst samples to collect (default 100 bursts)

        Returns:
            Dictionary with measurement results
        """
        # Get the burst engine instance
        try:
            from feagi.npu.burst_engine import BurstEngine

            burst_engine = BurstEngine.get_instance()

            if burst_engine is None:
                raise RuntimeError("No burst engine instance available")

            if not burst_engine._running:
                raise RuntimeError("Burst engine is not running")

            # Only log detailed measurement triggers when debugging NPU
            if self.is_debug_npu_enabled():
                logger.info(
                    f"Starting frequency measurement ({measurement_duration_s}s, {sample_count} samples)",
                    status="[DEBUG]",
                )

            # Trigger measurement in burst engine
            measurement_result = burst_engine.measure_actual_frequency(
                duration_seconds=measurement_duration_s, sample_count=sample_count
            )

            # Add to history
            self.add_frequency_measurement(
                actual_frequency_hz=measurement_result["actual_frequency_hz"],
                potential_frequency_hz=measurement_result["potential_frequency_hz"],
                measurement_duration_s=measurement_result["measurement_duration_s"],
                metadata={
                    "sample_count": measurement_result["sample_count"],
                    "min_cycle_time_ms": measurement_result.get("min_cycle_time_ms", 0),
                    "max_cycle_time_ms": measurement_result.get("max_cycle_time_ms", 0),
                    "avg_cycle_time_ms": measurement_result.get("avg_cycle_time_ms", 0),
                    "cycle_std_dev_ms": measurement_result.get("cycle_std_dev_ms", 0),
                    "min_processing_time_ms": measurement_result.get(
                        "min_processing_time_ms", 0
                    ),
                    "max_processing_time_ms": measurement_result.get(
                        "max_processing_time_ms", 0
                    ),
                    "avg_processing_time_ms": measurement_result.get(
                        "avg_processing_time_ms", 0
                    ),
                    "processing_std_dev_ms": measurement_result.get(
                        "processing_std_dev_ms", 0
                    ),
                    "efficiency_ratio": measurement_result.get("efficiency_ratio", 0),
                    "headroom_hz": measurement_result.get("headroom_hz", 0),
                },
            )

            return measurement_result

        except Exception as e:
            logger.error(f"Failed to trigger frequency measurement: {e}")
            raise

    def get_frequency_status_summary(self) -> dict:
        """
        Get a summary of frequency measurements for monitoring/debugging.

        Returns:
            Dictionary with current status and recent measurement trends
        """
        target_frequency = self.get_burst_frequency()
        latest_measurement = self.get_latest_frequency_measurement()

        summary = {
            "target_frequency_hz": target_frequency,
            "has_measurements": len(self._frequency_measurement_history) > 0,
            "total_measurements": len(self._frequency_measurement_history),
            "latest_measurement": latest_measurement,
        }

        if latest_measurement:
            summary.update(
                {
                    "current_actual_frequency_hz": latest_measurement[
                        "actual_frequency_hz"
                    ],
                    "current_potential_frequency_hz": latest_measurement[
                        "potential_frequency_hz"
                    ],
                    "current_performance_ratio": latest_measurement[
                        "actual_performance_ratio"
                    ],
                    "current_potential_ratio": latest_measurement[
                        "potential_performance_ratio"
                    ],
                    "current_performance_status": latest_measurement[
                        "performance_status"
                    ],
                    "current_system_capability": latest_measurement[
                        "system_capability"
                    ],
                    "current_efficiency_ratio": (
                        latest_measurement["actual_frequency_hz"]
                        / latest_measurement["potential_frequency_hz"]
                        if latest_measurement["potential_frequency_hz"] > 0
                        else 0
                    ),
                    "current_headroom_hz": latest_measurement["potential_headroom_hz"],
                    "measurement_age_seconds": time.time()
                    - latest_measurement["timestamp"],
                }
            )

            # Add trend analysis if we have multiple measurements
            if len(self._frequency_measurement_history) >= 3:
                recent_measurements = list(
                    self.get_frequency_measurement_history(5).values()
                )
                recent_actual_frequencies = [
                    m["actual_frequency_hz"] for m in recent_measurements
                ]
                recent_potential_frequencies = [
                    m["potential_frequency_hz"] for m in recent_measurements
                ]

                # Simple trend analysis for both frequencies
                if len(recent_actual_frequencies) >= 2:
                    actual_trend = (
                        "IMPROVING"
                        if recent_actual_frequencies[0] > recent_actual_frequencies[-1]
                        else (
                            "DECLINING"
                            if recent_actual_frequencies[0]
                            < recent_actual_frequencies[-1]
                            else "STABLE"
                        )
                    )
                    potential_trend = (
                        "IMPROVING"
                        if recent_potential_frequencies[0]
                        > recent_potential_frequencies[-1]
                        else (
                            "DECLINING"
                            if recent_potential_frequencies[0]
                            < recent_potential_frequencies[-1]
                            else "STABLE"
                        )
                    )

                    summary["actual_frequency_trend"] = actual_trend
                    summary["potential_frequency_trend"] = potential_trend
                    summary["recent_avg_actual_frequency_hz"] = sum(
                        recent_actual_frequencies
                    ) / len(recent_actual_frequencies)
                    summary["recent_avg_potential_frequency_hz"] = sum(
                        recent_potential_frequencies
                    ) / len(recent_potential_frequencies)

        return summary

    # ===== SIMD Configuration Management =====
    def initialize_simd_configuration(self) -> bool:
        """
        Initialize centralized SIMD configuration during FEAGI startup.

        Performs hardware detection once and stores results in shared state.
        Returns True if SIMD is available, False otherwise.

        RTOS/Rust Compatible: Single detection, cached results.
        """
        self._log_state_message("[INIT]", "Starting SIMD/GPU backend detection...")

        try:
            # Import SIMD detection (may fail in SIMD-less environments)
            from feagi.utils.simd_detection import get_simd_detector

            detector = get_simd_detector()
            caps = detector.capabilities
            backend = detector.get_optimal_backend()

            # Store SIMD availability
            self.state_ptr.contents.simd_available = 1

            # Map backend to integer
            backend_name = backend.value if hasattr(backend, "value") else str(backend)
            self.state_ptr.contents.simd_backend = _SIMD_BACKEND_INTS.get(
                backend_name, 0
            )

            # Store capabilities
            self.state_ptr.contents.simd_vector_width = caps.vector_width
            self.state_ptr.contents.simd_supports_avx = 1 if caps.avx else 0
            self.state_ptr.contents.simd_supports_avx2 = 1 if caps.avx2 else 0
            self.state_ptr.contents.simd_supports_avx512 = 1 if caps.avx512f else 0
            self.state_ptr.contents.simd_alignment = detector.get_memory_alignment()
            self.state_ptr.contents.simd_initialization_timestamp = int(
                time.time() * 1000
            )

            self.state_ptr.contents.state_version += 1

            # Comprehensive backend logging
            self._log_state_message("[BACKEND]", "🚀 SIMD/GPU Detection Complete")
            self._log_state_message(
                "[BACKEND]", f"├─ Platform: {caps.platform} ({caps.architecture})"
            )
            self._log_state_message("[BACKEND]", f"├─ Optimal Backend: {backend_name}")
            self._log_state_message(
                "[BACKEND]", f"├─ Vector Width: {caps.vector_width} (SIMD parallelism)"
            )
            self._log_state_message(
                "[BACKEND]",
                f"├─ Memory Alignment: {self.state_ptr.contents.simd_alignment} bytes",
            )

            # CPU SIMD capabilities
            simd_features = []
            if caps.sse2:
                simd_features.append("SSE2")
            if caps.avx:
                simd_features.append("AVX")
            if caps.avx2:
                simd_features.append("AVX2")
            if caps.avx512f:
                simd_features.append("AVX512F")
            if caps.neon:
                simd_features.append("NEON")
            if caps.sve:
                simd_features.append("SVE")

            if simd_features:
                self._log_state_message(
                    "[BACKEND]", f"├─ CPU SIMD Features: {', '.join(simd_features)}"
                )
            else:
                self._log_state_message(
                    "[BACKEND]", "├─ CPU SIMD Features: None (scalar only)"
                )

            # GPU capabilities
            gpu_features = []
            if caps.cuda_available:
                gpu_features.append("CUDA")
            if caps.webgpu_available:
                gpu_features.append("WebGPU")

            if gpu_features:
                self._log_state_message(
                    "[BACKEND]", f"├─ GPU Features: {', '.join(gpu_features)}"
                )
            else:
                self._log_state_message("[BACKEND]", "├─ GPU Features: None detected")

            # Performance expectations
            if caps.vector_width >= 16:
                perf_tier = "High-Performance"
            elif caps.vector_width >= 8:
                perf_tier = "Standard"
            elif caps.vector_width >= 4:
                perf_tier = "Basic SIMD"
            else:
                perf_tier = "Scalar (No SIMD)"

            self._log_state_message("[BACKEND]", f"└─ Performance Tier: {perf_tier}")

            # Final summary
            self._log_state_message(
                "[SIMD]",
                f"✅ Acceleration enabled: {backend_name} backend ready for neural processing",
            )
            return True

        except ImportError:
            # SIMD not available - set defaults
            self.state_ptr.contents.simd_available = 0
            self.state_ptr.contents.simd_backend = 0  # SCALAR
            self.state_ptr.contents.simd_vector_width = 1
            self.state_ptr.contents.simd_supports_avx = 0
            self.state_ptr.contents.simd_supports_avx2 = 0
            self.state_ptr.contents.simd_supports_avx512 = 0
            self.state_ptr.contents.simd_alignment = 8  # Standard alignment
            self.state_ptr.contents.simd_initialization_timestamp = int(
                time.time() * 1000
            )

            self.state_ptr.contents.state_version += 1
            self._log_state_message(
                "[BACKEND]", "⚠️ SIMD/GPU acceleration not available"
            )
            self._log_state_message("[BACKEND]", "├─ Backend: SCALAR (CPU-only)")
            self._log_state_message(
                "[BACKEND]", "├─ Reason: SIMD detection module not found"
            )
            self._log_state_message("[BACKEND]", "└─ Performance: Basic CPU processing")
            self._log_state_message(
                "[SIMD]", "❌ Using scalar fallback for neural processing"
            )
            return False

        except Exception as e:
            # Error during detection - safe fallback
            self.state_ptr.contents.simd_available = 0
            self.state_ptr.contents.simd_backend = 0
            self.state_ptr.contents.simd_vector_width = 1
            self.state_ptr.contents.simd_supports_avx = 0
            self.state_ptr.contents.simd_supports_avx2 = 0
            self.state_ptr.contents.simd_supports_avx512 = 0
            self.state_ptr.contents.simd_alignment = 8
            self.state_ptr.contents.simd_initialization_timestamp = int(
                time.time() * 1000
            )

            self.state_ptr.contents.state_version += 1
            self._log_state_message("[BACKEND]", f"❌ SIMD/GPU detection failed: {e}")
            self._log_state_message("[BACKEND]", "├─ Backend: SCALAR (CPU-only)")
            self._log_state_message("[BACKEND]", "├─ Reason: Hardware detection error")
            self._log_state_message("[BACKEND]", "└─ Performance: Basic CPU processing")
            self._log_state_message(
                "[SIMD]", "❌ Using scalar fallback for neural processing"
            )
            return False

    def get_simd_configuration(self) -> dict:
        """
        Get centralized SIMD configuration.

        Returns dictionary with SIMD capabilities for component use.
        Components should use this instead of doing their own detection.
        """
        return {
            "available": bool(self.state_ptr.contents.simd_available),
            "backend": _SIMD_BACKEND_VALUES.get(
                self.state_ptr.contents.simd_backend, "SCALAR"
            ),
            "vector_width": self.state_ptr.contents.simd_vector_width,
            "supports_avx": bool(self.state_ptr.contents.simd_supports_avx),
            "supports_avx2": bool(self.state_ptr.contents.simd_supports_avx2),
            "supports_avx512": bool(self.state_ptr.contents.simd_supports_avx512),
            "alignment": self.state_ptr.contents.simd_alignment,
            "initialization_timestamp": self.state_ptr.contents.simd_initialization_timestamp,
            "backend_int": self.state_ptr.contents.simd_backend,  # For compatibility
        }

    def is_simd_available(self) -> bool:
        """Check if SIMD acceleration is available."""
        return bool(self.state_ptr.contents.simd_available)

    def get_simd_backend(self) -> str:
        """Get the optimal SIMD backend name."""
        return _SIMD_BACKEND_VALUES.get(self.state_ptr.contents.simd_backend, "SCALAR")

    def get_simd_vector_width(self) -> int:
        """Get SIMD vector width."""
        return self.state_ptr.contents.simd_vector_width

    def get_simd_alignment(self) -> int:
        """Get required memory alignment for SIMD operations."""
        return self.state_ptr.contents.simd_alignment

    def log_startup_summary(self) -> None:
        """
        Log a comprehensive startup summary of all FEAGI states.

        Call this after FEAGI initialization is complete to provide
        a complete overview of the system configuration.
        """
        self._log_state_message("[STARTUP]", "🏁 FEAGI Initialization Summary")
        self._log_state_message("[STARTUP]", "=" * 50)

        # Core states
        genome_state = self.get_genome_state()
        connectome_state = self.get_connectome_state()
        simulation_state = self.get_simulation_state()

        self._log_state_message("[STARTUP]", f"🧬 Genome State: {genome_state.name}")
        self._log_state_message(
            "[STARTUP]", f"🕸️  Connectome State: {connectome_state.name}"
        )
        self._log_state_message(
            "[STARTUP]", f"🧪 Simulation State: {simulation_state.name}"
        )
        self._log_state_message(
            "[STARTUP]", f"🧠 Brain Ready: {self.get_brain_readiness()}"
        )

        # Service states
        api_state = self.get_api_state()
        zmq_state = self.get_zmq_state()
        burst_state = self.get_burst_engine_state()
        fq_state = self.get_fq_sampler_state()

        self._log_state_message("[STARTUP]", f"🌐 REST API: {api_state.name}")
        self._log_state_message("[STARTUP]", f"⚡ ZMQ Service: {zmq_state.name}")
        self._log_state_message("[STARTUP]", f"💥 Burst Engine: {burst_state.name}")
        self._log_state_message("[STARTUP]", f"🎯 FQ Sampler: {fq_state.name}")

        # Performance configuration
        burst_freq = self.get_burst_frequency()
        fq_freq = self.get_fq_sampler_frequency()
        agent_count = self.get_agent_count()

        self._log_state_message("[STARTUP]", f"⚡ Burst Frequency: {burst_freq:.1f}Hz")
        self._log_state_message("[STARTUP]", f"🎯 FQ Frequency: {fq_freq:.1f}Hz")
        self._log_state_message("[STARTUP]", f"🤖 Connected Agents: {agent_count}")

        # SIMD/GPU summary
        simd_config = self.get_simd_configuration()
        if simd_config["available"]:
            self._log_state_message(
                "[STARTUP]",
                f"🚀 Acceleration: {simd_config['backend']} (Vector Width: {simd_config['vector_width']})",
            )
        else:
            self._log_state_message(
                "[STARTUP]", "🐌 Acceleration: SCALAR (No SIMD/GPU)"
            )

        # Test modes
        test_viz = self.get_test_visualization_mode()
        if test_viz:
            self._log_state_message("[STARTUP]", "🧪 Test Visualization: ENABLED")

        # State version for debugging
        version = self.get_state_version()
        self._log_state_message("[STARTUP]", f"📊 State Version: {version}")

        self._log_state_message("[STARTUP]", "=" * 50)
        self._log_state_message("[STARTUP]", "✅ FEAGI is ready for neural simulation!")

    def _log_state_change(self, state_type: str, old_value, new_value):
        """
        Log state changes in the expected format: [state manager] {prior state} >> {new state}

        ENFORCED USAGE: This method requires exactly 3 parameters for actual state transitions.
        For general logging messages, use _log_state_message() instead.

        Args:
            state_type: Type of state that changed (e.g., "GenomeState", "ConnectomeState")
            old_value: Previous state value
            new_value: New state value
        """
        # Enforce correct usage pattern - this method is ONLY for state transitions
        if state_type.startswith("[") and state_type.endswith("]"):
            raise ValueError(
                f"Invalid usage: _log_state_change() called with category tag '{state_type}'. "
                f"Use _log_state_message() for general logging with category tags. "
                f"_log_state_change() is reserved for actual state transitions only."
            )

        if old_value != new_value:
            old_str = str(old_value) if old_value is not None else "None"
            new_str = str(new_value) if new_value is not None else "None"
            logger.info(f"[state manager] {state_type}: {old_str} >> {new_str}")

    def _log_state_message(self, category: str, message: str):
        """
        Log informational state messages with category prefixes.

        Args:
            category: Category tag like "[DNA]", "[NET]", "[SIMD]"
            message: The main log message
        """
        logger.info(f"{category} {message}")

    # ===== CRITICAL SERVICE READINESS GATE =====

    def are_critical_services_ready(self) -> bool:
        """
        Check if ALL critical services are ready for operation.

        This is the master gate that prevents FQ sampler registration
        until all Priority 1 critical services are operational.

        Returns:
            bool: True if all critical services are ready, False otherwise
        """
        critical_status = self.get_critical_services_status()

        # All services must be in READY state
        ready = all(status == ServiceState.READY for status in critical_status.values())

        if not ready:
            # Log which services are not ready (but only once every 5 seconds to avoid spam)
            import time

            current_time = time.time()
            if (
                not hasattr(self, "_last_critical_check_log")
                or current_time - self._last_critical_check_log > 5.0
            ):
                not_ready = [
                    service
                    for service, status in critical_status.items()
                    if status != ServiceState.READY
                ]
                self._log_state_message(
                    "CRITICAL", f"Services not ready: {', '.join(not_ready)}"
                )
                self._last_critical_check_log = current_time

        return ready

    def get_critical_services_status(self) -> dict:
        """
        Get the status of all critical services.

        Returns:
            dict: Service name -> ServiceState mapping
        """
        return {
            "burst_engine": self.get_burst_engine_state(),
            "connectome": (
                ServiceState.READY
                if self.is_connectome_ready()
                else ServiceState.UNAVAILABLE
            ),
            "genome": (
                ServiceState.READY
                if self.is_genome_loaded()
                else ServiceState.UNAVAILABLE
            ),
            "state_manager": ServiceState.READY,  # Always ready if this method is called
            "zmq_server": self.get_zmq_state(),
            "api_server": self.get_api_state(),
        }

    def wait_for_critical_services(
        self, timeout_seconds: float = 30.0, check_interval: float = 0.5
    ) -> bool:
        """
        Wait for all critical services to become ready.

        Args:
            timeout_seconds: Maximum time to wait in seconds
            check_interval: How often to check service status in seconds

        Returns:
            bool: True if all services became ready within timeout, False if timeout
        """
        import time

        start_time = time.time()

        self._log_state_message(
            "CRITICAL", f"Waiting for critical services (timeout: {timeout_seconds}s)"
        )

        while time.time() - start_time < timeout_seconds:
            if self.are_critical_services_ready():
                elapsed = time.time() - start_time
                self._log_state_message(
                    "CRITICAL", f"All critical services ready after {elapsed:.1f}s"
                )
                return True

            # Log progress every 5 seconds
            elapsed = time.time() - start_time
            if elapsed % 5.0 < check_interval:
                status = self.get_critical_services_status()
                not_ready = [s for s, st in status.items() if st != ServiceState.READY]
                self._log_state_message(
                    "CRITICAL",
                    f"Still waiting for: {', '.join(not_ready)} ({elapsed:.1f}s elapsed)",
                )

            time.sleep(check_interval)

        # Timeout reached
        status = self.get_critical_services_status()
        not_ready = [s for s, st in status.items() if st != ServiceState.READY]
        self._log_state_message(
            "CRITICAL",
            f"TIMEOUT after {timeout_seconds}s - Services not ready: {', '.join(not_ready)}",
        )
        return False

    def set_system_ready_for_fq_samplers(self, ready: bool) -> None:
        """
        Mark the system as ready (or not ready) for FQ sampler operations.

        This is set by the Process Manager once all critical services
        have been verified as operational.

        Args:
            ready: True if system is ready for FQ samplers, False otherwise
        """
        old_value = self.get_brain_readiness()
        self.set_brain_readiness(ready)

        if ready and not old_value:
            self._log_state_message(
                "CRITICAL", "System now READY for FQ sampler registration"
            )
        elif not ready and old_value:
            self._log_state_message(
                "CRITICAL", "System BLOCKED from FQ sampler registration"
            )

    def is_system_ready_for_fq_samplers(self) -> bool:
        """
        Check if the system is ready for FQ sampler operations.

        This combines both critical service readiness AND explicit
        system readiness flag set by Process Manager.

        Returns:
            bool: True if ready for FQ samplers, False otherwise
        """
        return self.are_critical_services_ready() and self.get_brain_readiness()

    # ===== END CRITICAL SERVICE READINESS GATE =====

    # ===== Debug Configuration =====
    def set_debug_config(self, config: dict) -> None:
        """
        Set debug configuration from loaded FEAGI config.

        RTOS-compatible: Stores flags in memory for zero-overhead access.

        Args:
            config: Configuration dictionary with debug section
        """
        debug_section = config.get("debug", {})
        self._debug_config.update(
            {
                "api": debug_section.get("api", False),
                "npu": debug_section.get("npu", False),
                "bdu": debug_section.get("bdu", False),
                "zmq_outbound": debug_section.get("zmq_outbound", False),
                "zmq_inbound": debug_section.get("zmq_inbound", False),
            }
        )

        if any(self._debug_config.values()):
            enabled_flags = [k for k, v in self._debug_config.items() if v]
            logger.info(f"[DEBUG] Debug flags enabled: {enabled_flags}")

    def is_debug_npu_enabled(self) -> bool:
        """Check if NPU debug logging is enabled."""
        return self._debug_config.get("npu", False)

    def is_debug_api_enabled(self) -> bool:
        """Check if API debug logging is enabled."""
        return self._debug_config.get("api", False)

    def is_debug_zmq_outbound_enabled(self) -> bool:
        """Check if ZMQ outbound debug logging is enabled."""
        return self._debug_config.get("zmq_outbound", False)

    def is_debug_zmq_inbound_enabled(self) -> bool:
        """Check if ZMQ inbound debug logging is enabled."""
        return self._debug_config.get("zmq_inbound", False)

    def is_debug_bdu_enabled(self) -> bool:
        """Check if BDU (Brain Development Unit) debug logging is enabled."""
        return self._debug_config.get("bdu", False)

    def get_debug_config(self) -> dict:
        """Get current debug configuration."""
        return self._debug_config.copy()


def get_state_manager():
    """Get the singleton instance of FeagiStateManager"""
    return FeagiStateManager.instance()
