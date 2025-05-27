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
import mmap
import os
from enum import IntEnum, Enum
from typing import Optional
import time
from feagi.utils.logger import setup_logger
import datetime
from contextlib import contextmanager

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
        ("burst_frequency", ctypes.c_float),            # Target/assigned frequency from genome/user
        ("simulation_state", ctypes.c_uint8),
        ("fq_sampler_state", ctypes.c_uint8),
        ("fq_sampler_frequency", ctypes.c_float),
        ("fq_sampler_consumer", ctypes.c_uint8),
        ("state_version", ctypes.c_uint64),
        ("genome_counter", ctypes.c_uint32),
        ("brain_readiness", ctypes.c_uint8),  # 0 = False, 1 = True
        ("test_visualization_mode", ctypes.c_uint8),  # 0 = False, 1 = True
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
    11: "ON_HOLD"
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
    _default_dir = "/tmp"

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
                    logger.info(f"🔗 Using shared state file from environment: {path}")
                else:
                    # Create new state file (main process mode)
                    timestamp = int(time.time())
                    path = f"{cls._default_dir}/feagi_state_{timestamp}.bin"
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
        
        # Add frequency measurement history (in-memory only, not persisted)
        # Format: {timestamp: {"frequency_hz": float, "measurement_duration_s": float, "performance_status": str}}
        self._frequency_measurement_history = {}
        self._max_frequency_history_entries = 100  # Keep last 100 measurements

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
        """Get current target/assigned burst frequency in Hz (from genome/user settings)"""
        return self.state_ptr.contents.burst_frequency
    
    def set_burst_frequency(self, frequency: float) -> None:
        """Set target/assigned burst frequency in Hz (from genome/user settings)"""
        self.state_ptr.contents.burst_frequency = frequency
        self.state_ptr.contents.state_version += 1
        
        # Only log frequency changes when debugging NPU
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            logger.info(f"Target burst frequency set to {frequency:.1f}Hz", emoji1="⚡")

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
        _log_state_change("🧪", f"Simulation state changed: {old.name} → {state.name}")
        
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
        logger.info(f"FQSampler state changed: {old_state.name} → {state.name}", emoji1="🎯")
        self._notify_state_change("FQ Sampler", old_state, state)

    # ===== FQSampler Frequency =====
    def get_fq_sampler_frequency(self) -> float:
        """Get current FQSampler frequency in Hz"""
        return self.state_ptr.contents.fq_sampler_frequency
    def set_fq_sampler_frequency(self, frequency: float) -> None:
        """Set FQSampler frequency in Hz"""
        self.state_ptr.contents.fq_sampler_frequency = frequency
        self.state_ptr.contents.state_version += 1

    # ===== FQSampler Consumer =====
    def get_fq_sampler_consumer(self) -> int:
        """Get current FQSampler consumer code (1=Visualization, 2=Motor, 3=Both, etc.)"""
        return self.state_ptr.contents.fq_sampler_consumer
    def set_fq_sampler_consumer(self, consumer: int) -> None:
        """Set FQSampler consumer code (1=Visualization, 2=Motor, 3=Both, etc.)"""
        self.state_ptr.contents.fq_sampler_consumer = consumer
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
            logger.info(f"Test visualization mode changed: {old} → {enabled}", emoji1="🧮")

    def get_connectome(self):
        """Get the current connectome instance"""
        try:
            # Check if embedded mode is enabled
            import os
            if os.environ.get('FEAGI_EMBEDDED_MODE', '0') == '1':
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

    # ===== Frequency Measurement History =====
    def add_frequency_measurement(self, actual_frequency_hz: float, potential_frequency_hz: float, 
                                 measurement_duration_s: float, metadata: Optional[dict] = None) -> None:
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
        actual_ratio = actual_frequency_hz / target_frequency if target_frequency > 0 else 0.0
        potential_ratio = potential_frequency_hz / target_frequency if target_frequency > 0 else 0.0
        
        # Assess performance status based on actual frequency
        if actual_ratio >= 0.95:
            status = "OPTIMAL"  # Within 5% of target
        elif actual_ratio >= 0.8:
            status = "GOOD"     # Within 20% of target
        elif actual_ratio >= 0.5:
            status = "DEGRADED" # 50-80% of target
        else:
            status = "POOR"     # Below 50% of target
        
        # Assess system capability based on potential frequency
        if potential_ratio >= 2.0:
            capability = "HIGH"      # Can run at 2x+ target
        elif potential_ratio >= 1.5:
            capability = "GOOD"      # Can run at 1.5x+ target
        elif potential_ratio >= 1.0:
            capability = "ADEQUATE"  # Can meet target
        else:
            capability = "LIMITED"   # Cannot meet target even at full speed
        
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
            "metadata": metadata or {}
        }
        
        # Add to history
        self._frequency_measurement_history[timestamp] = measurement_entry
        
        # Maintain history size limit
        if len(self._frequency_measurement_history) > self._max_frequency_history_entries:
            # Remove oldest entries
            oldest_timestamps = sorted(self._frequency_measurement_history.keys())[:-self._max_frequency_history_entries]
            for old_timestamp in oldest_timestamps:
                del self._frequency_measurement_history[old_timestamp]
        
        # Update state version
        self.state_ptr.contents.state_version += 1
        
        # Only log detailed frequency measurements when debugging NPU
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            logger.info(f"Frequency measurement recorded - Actual: {actual_frequency_hz:.1f}Hz, Potential: {potential_frequency_hz:.1f}Hz ({status})", emoji1="📊")
        else:
            # For normal operation, only log significant measurements or changes
            if status in ["POOR", "DEGRADED"] or (hasattr(self, '_last_logged_status') and self._last_logged_status != status):
                logger.info(f"Performance status: {status} - Actual: {actual_frequency_hz:.1f}Hz", emoji1="📊")
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
        sorted_timestamps = sorted(self._frequency_measurement_history.keys(), reverse=True)
        limited_timestamps = sorted_timestamps[:limit]
        
        return {ts: self._frequency_measurement_history[ts] for ts in limited_timestamps}
    
    def get_latest_frequency_measurement(self) -> Optional[dict]:
        """
        Get the most recent frequency measurement.
        
        Returns:
            Latest measurement data or None if no measurements exist
        """
        if not self._frequency_measurement_history:
            return None
            
        latest_timestamp = max(self._frequency_measurement_history.keys())
        latest_measurement = self._frequency_measurement_history[latest_timestamp].copy()
        latest_measurement["timestamp"] = latest_timestamp
        
        return latest_measurement
    
    def trigger_frequency_measurement(self, measurement_duration_s: float = 5.0, 
                                    sample_count: int = 100) -> dict:
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
            if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                logger.info(f"Starting frequency measurement ({measurement_duration_s}s, {sample_count} samples)", emoji1="🔬")
            
            # Trigger measurement in burst engine
            measurement_result = burst_engine.measure_actual_frequency(
                duration_seconds=measurement_duration_s,
                sample_count=sample_count
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
                    "min_processing_time_ms": measurement_result.get("min_processing_time_ms", 0),
                    "max_processing_time_ms": measurement_result.get("max_processing_time_ms", 0),
                    "avg_processing_time_ms": measurement_result.get("avg_processing_time_ms", 0),
                    "processing_std_dev_ms": measurement_result.get("processing_std_dev_ms", 0),
                    "efficiency_ratio": measurement_result.get("efficiency_ratio", 0),
                    "headroom_hz": measurement_result.get("headroom_hz", 0)
                }
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
            "latest_measurement": latest_measurement
        }
        
        if latest_measurement:
            summary.update({
                "current_actual_frequency_hz": latest_measurement["actual_frequency_hz"],
                "current_potential_frequency_hz": latest_measurement["potential_frequency_hz"],
                "current_performance_ratio": latest_measurement["actual_performance_ratio"],
                "current_potential_ratio": latest_measurement["potential_performance_ratio"],
                "current_performance_status": latest_measurement["performance_status"],
                "current_system_capability": latest_measurement["system_capability"],
                "current_efficiency_ratio": latest_measurement["actual_frequency_hz"] / latest_measurement["potential_frequency_hz"] if latest_measurement["potential_frequency_hz"] > 0 else 0,
                "current_headroom_hz": latest_measurement["potential_headroom_hz"],
                "measurement_age_seconds": time.time() - latest_measurement["timestamp"]
            })
            
            # Add trend analysis if we have multiple measurements
            if len(self._frequency_measurement_history) >= 3:
                recent_measurements = list(self.get_frequency_measurement_history(5).values())
                recent_actual_frequencies = [m["actual_frequency_hz"] for m in recent_measurements]
                recent_potential_frequencies = [m["potential_frequency_hz"] for m in recent_measurements]
                
                # Simple trend analysis for both frequencies
                if len(recent_actual_frequencies) >= 2:
                    actual_trend = "IMPROVING" if recent_actual_frequencies[0] > recent_actual_frequencies[-1] else \
                                  "DECLINING" if recent_actual_frequencies[0] < recent_actual_frequencies[-1] else "STABLE"
                    potential_trend = "IMPROVING" if recent_potential_frequencies[0] > recent_potential_frequencies[-1] else \
                                     "DECLINING" if recent_potential_frequencies[0] < recent_potential_frequencies[-1] else "STABLE"
                    
                    summary["actual_frequency_trend"] = actual_trend
                    summary["potential_frequency_trend"] = potential_trend
                    summary["recent_avg_actual_frequency_hz"] = sum(recent_actual_frequencies) / len(recent_actual_frequencies)
                    summary["recent_avg_potential_frequency_hz"] = sum(recent_potential_frequencies) / len(recent_potential_frequencies)
        
        return summary

def get_state_manager():
    """Get the singleton instance of FeagiStateManager"""
    return FeagiStateManager.instance() 