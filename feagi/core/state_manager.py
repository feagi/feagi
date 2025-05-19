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
import mmap
import os
from enum import IntEnum, Enum
from typing import Optional
import time
from feagi.utils.logger import setup_logger
import datetime
from contextlib import contextmanager

# Need to add this at the top of the file with the other imports
logger = setup_logger()


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
        ("genome_counter", ctypes.c_uint32),
        ("brain_readiness", ctypes.c_uint8),  # 0 = False, 1 = True
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
    10: "SYNC_ERROR"
}

# And the reverse mapping
_SERVICE_STATE_INTS = {v: k for k, v in _SERVICE_STATE_VALUES.items()}

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

        # Add synchronization tracking
        self.genome_sync_state = ServiceState.UNINITIALIZED
        self.pending_sync_operations = []
        self.sync_observers = []

        # Add notification hooks
        self._notification_callbacks = {
            "genome": [],
            "connectome": [],
            "burst_engine": [],
            "simulation": []
        }

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
        old = GenomeState(self.state_ptr.contents.genome_state)
        self.state_ptr.contents.genome_state = int(state)
        self.state_ptr.contents.state_version += 1
        logger.info(f"Genome state changed: {old.name} → {state.name}", emoji1="🧬")
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
        logger.info(f"Connectome state changed: {old.name} → {state.name}", emoji1="🕸️")
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
        logger.info(f"REST API state changed: {old_state.name} → {state.name}", emoji1="🚦")
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
        logger.info(f"ZMQ state changed: {old_state.name} → {state.name}", emoji1="📬")
        self._notify_state_change("ZMQ", old_state, state)

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
        logger.info(f"Burst Engine state changed: {old_state.name} → {state.name}", emoji1="💥")
        # Use the category key from the notification callbacks dict
        self._notify_state_change("burst_engine", old_state, state)

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
        old = SimulationState(self.state_ptr.contents.simulation_state)
        self.state_ptr.contents.simulation_state = int(state)
        self.state_ptr.contents.state_version += 1
        logger.info(f"Simulation state changed: {old.name} → {state.name}", emoji1="🧪")
        self._notify_state_change("simulation", old, state)

    # ===== FCLSampler State =====
    def get_fcl_sampler_state(self) -> ServiceState:
        """Get the current FCL sampler state."""
        raw_value = self.state_ptr.contents.fcl_sampler_state
        return ServiceState(_SERVICE_STATE_VALUES.get(raw_value, "UNAVAILABLE"))
    
    def set_fcl_sampler_state(self, state: ServiceState) -> None:
        """Set the FCL sampler state."""
        self._verify_enum(state, ServiceState)
        old_state = self.get_fcl_sampler_state()
        self.state_ptr.contents.fcl_sampler_state = int(state)
        self.state_ptr.contents.state_version += 1
        logger.info(f"FCLSampler state changed: {old_state.name} → {state.name}", emoji1="🎯")
        self._notify_state_change("FCL Sampler", old_state, state)

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
        self.state_ptr.contents.genome_counter += 1
        self.state_ptr.contents.state_version += 1
        logger.info(f"Genome counter incremented to {self.state_ptr.contents.genome_counter}")
        self.sync_to_disk()

    def get_brain_readiness(self) -> bool:
        """Get the brain readiness flag (True if brain is ready)"""
        return bool(self.state_ptr.contents.brain_readiness)

    def set_brain_readiness(self, ready: bool) -> None:
        """Set the brain readiness flag (True if brain is ready)"""
        old = bool(self.state_ptr.contents.brain_readiness)
        self.state_ptr.contents.brain_readiness = 1 if ready else 0
        self.state_ptr.contents.state_version += 1
        logger.info(f"Brain readiness changed: {old} → {ready}", emoji1="🧠")

    def get_connectome(self):
        """Get the current connectome instance"""
        try:
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
        logger.info(f"Genome-Connectome sync state changed: {old_state} → {state}", emoji1="🔄")
        
        # Notify observers
        for observer in self.sync_observers:
            observer.on_sync_state_change(old_state, state, details)
            
    def begin_genome_transaction(self):
        """Begin a new genome modification transaction"""
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
                    logger.error(f"⚠️ Error in notification callback: {e}")

    def _verify_enum(self, state, enum_type):
        if not isinstance(state, enum_type):
            raise ValueError(f"{state} is not a valid {enum_type.__name__}")

# def _log_state_change(emoji: str, message: str):
#     """
#     Central function for logging state changes with emoji support.
#
#     This function handles emoji logging in a way that works with both:
#     1. Custom loggers that support the emoji parameter
#     2. Standard loggers that don't support this parameter
#
#     All code that logs state changes should use this function rather than
#     calling logger.info() directly.
#
#     Args:
#         emoji: The emoji to prefix the log message with
#         message: The log message content
#     """
#     try:
#         # Try with emoji parameter first
#         logging.getLogger(__name__).info(message, emoji=emoji)
#     except TypeError:
#         # Fall back to standard logging if emoji not supported
#         logging.getLogger(__name__).info(f"{emoji} {message}")
#
#     # Always also log with standard formatting for consistency
#     icon = f"{emoji:<2}" if emoji else "   "
#     timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
#     log_msg = f"{icon} [{timestamp}] {message}"
#     logger.info(log_msg)

def get_state_manager():
    """Get the singleton instance of FeagiStateManager"""
    return FeagiStateManager.instance() 