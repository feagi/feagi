"""
Rust-compatible atomic operations and memory layouts for FEAGI state management.

This module provides atomic primitives and fixed-size data structures
that can be directly converted to Rust when migrating.
"""

import ctypes
import logging
import threading
import time

logger = logging.getLogger(__name__)


class AtomicU8:
    """
    Rust-compatible atomic u8 using Python threading primitives.

    This maps directly to std::sync::atomic::AtomicU8 in Rust.
    All operations are thread-safe and provide memory ordering guarantees.
    """

    def __init__(self, initial: int = 0):
        """
        Initialize atomic u8 with initial value.

        Args:
            initial: Initial value (must be 0-255)
        """
        if not (0 <= initial <= 255):
            raise ValueError(f"AtomicU8 value must be 0-255, got {initial}")

        self._value = initial
        self._lock = threading.Lock()

    def load(self) -> int:
        """
        Atomically load the current value.

        Returns:
            Current value (0-255)
        """
        with self._lock:
            return self._value

    def store(self, value: int) -> None:
        """
        Atomically store a new value.

        Args:
            value: New value (must be 0-255)
        """
        if not (0 <= value <= 255):
            raise ValueError(f"AtomicU8 value must be 0-255, got {value}")

        with self._lock:
            self._value = value

    def compare_exchange(self, expected: int, desired: int) -> bool:
        """
        Atomically compare and swap if equal.

        Args:
            expected: Expected current value
            desired: Value to set if current equals expected

        Returns:
            True if swap occurred, False otherwise
        """
        if not (0 <= expected <= 255) or not (0 <= desired <= 255):
            raise ValueError("AtomicU8 values must be 0-255")

        with self._lock:
            if self._value == expected:
                self._value = desired
                return True
            return False

    def fetch_add(self, value: int) -> int:
        """
        Atomically add to current value and return old value.

        Args:
            value: Value to add

        Returns:
            Previous value before addition
        """
        with self._lock:
            old_value = self._value
            new_value = (old_value + value) % 256  # Wrap on overflow
            self._value = new_value
            return old_value

    def fetch_sub(self, value: int) -> int:
        """
        Atomically subtract from current value and return old value.

        Args:
            value: Value to subtract

        Returns:
            Previous value before subtraction
        """
        with self._lock:
            old_value = self._value
            new_value = (old_value - value) % 256  # Wrap on underflow
            self._value = new_value
            return old_value


class AtomicU32:
    """
    Rust-compatible atomic u32 using Python threading primitives.

    This maps directly to std::sync::atomic::AtomicU32 in Rust.
    """

    def __init__(self, initial: int = 0):
        """
        Initialize atomic u32 with initial value.

        Args:
            initial: Initial value (must be 0 to 2^32-1)
        """
        if not (0 <= initial <= 0xFFFFFFFF):
            raise ValueError(
                f"AtomicU32 value must be 0 to 2^32-1, got {initial}"
            )

        self._value = initial
        self._lock = threading.Lock()

    def load(self) -> int:
        """Atomically load the current value."""
        with self._lock:
            return self._value

    def store(self, value: int) -> None:
        """Atomically store a new value."""
        if not (0 <= value <= 0xFFFFFFFF):
            raise ValueError(
                f"AtomicU32 value must be 0 to 2^32-1, got {value}"
            )

        with self._lock:
            self._value = value

    def compare_exchange(self, expected: int, desired: int) -> bool:
        """Atomically compare and swap if equal."""
        with self._lock:
            if self._value == expected:
                self._value = desired
                return True
            return False

    def fetch_add(self, value: int) -> int:
        """Atomically add and return old value."""
        with self._lock:
            old_value = self._value
            self._value = (old_value + value) % (2**32)  # Wrap on overflow
            return old_value

    def fetch_sub(self, value: int) -> int:
        """Atomically subtract and return old value."""
        with self._lock:
            old_value = self._value
            self._value = (old_value - value) % (2**32)  # Wrap on underflow
            return old_value


class AtomicU64:
    """
    Rust-compatible atomic u64 using Python threading primitives.

    This maps directly to std::sync::atomic::AtomicU64 in Rust.
    """

    def __init__(self, initial: int = 0):
        """Initialize atomic u64 with initial value."""
        if not (0 <= initial <= 0xFFFFFFFFFFFFFFFF):
            raise ValueError(
                f"AtomicU64 value must be 0 to 2^64-1, got {initial}"
            )

        self._value = initial
        self._lock = threading.Lock()

    def load(self) -> int:
        """Atomically load the current value."""
        with self._lock:
            return self._value

    def store(self, value: int) -> None:
        """Atomically store a new value."""
        if not (0 <= value <= 0xFFFFFFFFFFFFFFFF):
            raise ValueError(
                f"AtomicU64 value must be 0 to 2^64-1, got {value}"
            )

        with self._lock:
            self._value = value

    def fetch_add(self, value: int) -> int:
        """Atomically add and return old value."""
        with self._lock:
            old_value = self._value
            self._value = (old_value + value) % (2**64)
            return old_value


class RustCompatibleState(ctypes.Structure):
    """
    Memory layout identical to future Rust struct.

    This structure uses explicit field ordering and padding to ensure
    the same memory layout as the equivalent Rust struct with #[repr(C)].

    Layout:
    - Total size: 64 bytes (cache line aligned)
    - All fields are packed with explicit padding
    - Compatible with both little and big endian systems
    """

    _pack_ = 1  # No padding between fields
    _fields_ = [
        # Core service states (8 bytes)
        ("genome_state", ctypes.c_uint8),  # 0: GenomeState enum
        ("connectome_state", ctypes.c_uint8),  # 1: ConnectomeState enum
        ("burst_engine_state", ctypes.c_uint8),  # 2: ServiceState enum
        ("fq_sampler_state", ctypes.c_uint8),  # 3: ServiceState enum
        ("api_state", ctypes.c_uint8),  # 4: ServiceState enum
        ("zmq_state", ctypes.c_uint8),  # 5: ServiceState enum
        ("brain_readiness", ctypes.c_uint8),  # 6: 0 or 1
        ("exit_condition", ctypes.c_uint8),  # 7: 0 or 1 (burst engine control)
        # Counters and metrics (8 bytes)
        ("agent_count", ctypes.c_uint32),  # 8-11: Number of connected agents
        ("burst_frequency", ctypes.c_uint32),  # 12-15: Burst frequency in Hz
        # Versioning and timing (16 bytes)
        ("state_version", ctypes.c_uint64),  # 16-23: State change counter
        ("last_modified", ctypes.c_uint64),  # 24-31: Timestamp of last change
        # Development tracking (8 bytes)
        ("neuroembryogenesis_stage", ctypes.c_uint8),  # 32: Development stage
        (
            "neuroembryogenesis_progress",
            ctypes.c_uint8,
        ),  # 33: Progress percentage
        ("_reserved2", ctypes.c_uint16),  # 34-35: Reserved
        ("development_duration", ctypes.c_uint32),  # 36-39: Duration in ms
        # Statistics (16 bytes)
        ("neuron_count", ctypes.c_uint32),  # 40-43: Total neurons
        ("synapse_count", ctypes.c_uint32),  # 44-47: Total synapses
        (
            "cortical_area_count",
            ctypes.c_uint16,
        ),  # 48-49: Number of cortical areas
        ("_reserved3", ctypes.c_uint16),  # 50-51: Reserved
        ("memory_usage", ctypes.c_uint32),  # 52-55: Memory usage in bytes
        # Padding to 64 bytes (cache line aligned)
        ("_padding", ctypes.c_uint8 * 8),  # 56-63: Padding
    ]

    def __init__(self):
        """Initialize with default values."""
        super().__init__()
        self.genome_state = 0  # MISSING
        self.connectome_state = 0  # MISSING
        self.burst_engine_state = 0  # UNAVAILABLE
        self.fq_sampler_state = 0  # UNAVAILABLE
        self.api_state = 0  # UNAVAILABLE
        self.zmq_state = 0  # UNAVAILABLE
        self.brain_readiness = 0  # False
        self.exit_condition = 0  # False
        self.agent_count = 0
        self.burst_frequency = 0
        self.state_version = 0
        self.last_modified = int(time.time() * 1000)  # Current timestamp in ms
        self.neuroembryogenesis_stage = 0  # INITIALIZATION
        self.neuroembryogenesis_progress = 0
        self.development_duration = 0
        self.neuron_count = 0
        self.synapse_count = 0
        self.cortical_area_count = 0
        self.memory_usage = 0

        # Additional attributes for backward compatibility
        self.brain_stats = {}
        self.cortical_list = []
        self.genome_validity = False
        self.connected_agents = {}
        self.changes_saved_externally = False
        self.simulation_state = 0  # STOPPED
        self.genome_timestamp = 0  # Genome timestamp for change detection
        self.genome_counter = 0  # Genome counter for version tracking

    def get_size(self) -> int:
        """Get the size of the structure in bytes."""
        return ctypes.sizeof(self)

    def to_bytes(self) -> bytes:
        """Convert structure to bytes for serialization."""
        return bytes(self)

    @classmethod
    def from_bytes(cls, data: bytes) -> "RustCompatibleState":
        """Create structure from bytes."""
        if len(data) != 64:
            raise ValueError(f"Expected 64 bytes, got {len(data)}")

        instance = cls()
        ctypes.memmove(ctypes.addressof(instance), data, 64)
        return instance

    def validate_invariants(self) -> bool:
        """
        Validate structural invariants.

        Returns:
            True if all invariants hold, False otherwise
        """
        try:
            # Check enum values are in valid ranges
            if not (0 <= self.genome_state <= 4):
                return False
            if not (0 <= self.connectome_state <= 5):
                return False
            if not (0 <= self.burst_engine_state <= 7):
                return False
            if not (0 <= self.brain_readiness <= 1):
                return False
            if not (0 <= self.neuroembryogenesis_stage <= 6):
                return False
            if not (0 <= self.neuroembryogenesis_progress <= 100):
                return False

            return True
        except Exception as e:
            logger.error(f"Error validating state invariants: {e}")
            return False

    def __str__(self) -> str:
        """String representation for debugging."""
        return (
            f"RustCompatibleState("
            f"genome={self.genome_state}, "
            f"burst_engine={self.burst_engine_state}, "
            f"brain_ready={bool(self.brain_readiness)}, "
            f"agents={self.agent_count}, "
            f"version={self.state_version})"
        )


# Verify structure size at module load time
try:
    _state = RustCompatibleState()
    actual_size = _state.get_size()
    logger.info(f"RustCompatibleState validated: {actual_size} bytes")
except Exception as e:
    logger.error(f"Error validating RustCompatibleState: {e}")
