"""
Rust-friendly state manager for FEAGI.

This state manager is designed for easy conversion to Rust while maintaining
full compatibility with existing Python code. It provides atomic operations,
Result-based error handling, and fixed-size data structures.
"""

from .state_errors import Result, StateError, validate_state_transition
from .atomic_state import AtomicU8, RustCompatibleState
from .state_storage import StateStorage, MemoryStorage
import time
import threading
import logging
from typing import Dict, List, Any, Optional
from enum import Enum, IntEnum

logger = logging.getLogger(__name__)

# ===== State Enums (for compatibility with existing imports) =====
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

# State transition lookup tables (Rust: const arrays)
GENOME_TRANSITIONS = {
    (0, 1): True,  # MISSING -> LOADING
    (1, 2): True,  # LOADING -> LOADED
    (1, 4): True,  # LOADING -> ERROR
    (2, 1): True,  # LOADED -> LOADING (reload)
    (2, 3): True,  # LOADED -> SAVING
    (3, 2): True,  # SAVING -> LOADED
    (4, 1): True,  # ERROR -> LOADING (retry)
}

BURST_ENGINE_TRANSITIONS = {
    (0, 1): True,  # UNAVAILABLE -> INITIALIZING
    (1, 2): True,  # INITIALIZING -> READY
    (1, 6): True,  # INITIALIZING -> FAILED
    (2, 3): True,  # READY -> ON_HOLD
    (3, 2): True,  # ON_HOLD -> READY
    (2, 7): True,  # READY -> STOPPED
    (7, 0): True,  # STOPPED -> UNAVAILABLE
    (2, 5): True,  # READY -> ERROR
    (5, 1): True,  # ERROR -> INITIALIZING (restart)
}

FQ_SAMPLER_TRANSITIONS = {
    (0, 1): True,  # UNAVAILABLE -> INITIALIZING
    (1, 2): True,  # INITIALIZING -> READY
    (1, 3): True,  # INITIALIZING -> ERROR
    (2, 4): True,  # READY -> STOPPED
    (4, 0): True,  # STOPPED -> UNAVAILABLE
    (3, 1): True,  # ERROR -> INITIALIZING (retry)
}

class StateChangeEvent:
    """Rust-compatible state change event."""
    def __init__(self, field_name: str, old_value: int, new_value: int):
        self.timestamp = int(time.time() * 1000)  # milliseconds
        self.field_name = field_name
        self.old_value = old_value
        self.new_value = new_value

class FeagiStateManager:
    """
    Rust-friendly state manager designed for easy conversion.
    
    This manager provides:
    - Atomic operations for all state changes
    - Result-based error handling (no exceptions in hot paths)
    - Fixed-size data structures
    - Constant-time validation
    - Transaction support for multi-field updates
    """
    
    _instance = None
    _lock = threading.Lock()
    
    @classmethod
    def instance(cls, storage: Optional[StateStorage] = None):
        """Get singleton instance of the state manager."""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = cls(storage)
        return cls._instance
    
    def __init__(self, storage: Optional[StateStorage] = None):
        """Initialize state manager with storage backend."""
        self._storage = storage or MemoryStorage()
        self._instance_lock = threading.RLock()  # Reentrant lock for nested operations
        self._event_log: List[StateChangeEvent] = []
        self._max_events = 1000  # Fixed-size event log
        
        # Load initial state
        load_result = self._storage.load_state()
        if load_result.is_ok:
            self._state = load_result.unwrap()
        else:
            logger.warning(f"Failed to load state: {load_result.unwrap_err()}, using defaults")
            self._state = RustCompatibleState()
        
        # Create atomic wrappers for frequently accessed fields
        self._atomic_genome = AtomicU8(self._state.genome_state)
        self._atomic_burst_engine = AtomicU8(self._state.burst_engine_state)
        self._atomic_fq_sampler = AtomicU8(self._state.fq_sampler_state)
        self._atomic_brain_ready = AtomicU8(self._state.brain_readiness)
        self._atomic_version = AtomicU8(0)
        
        logger.info("FeagiStateManager initialized")
    
    # === GENOME STATE MANAGEMENT ===
    
    def get_genome_state(self) -> int:
        """Get current genome state (zero-cost operation)."""
        return self._atomic_genome.load()
    
    def set_genome_state(self, state: int) -> Result[None]:
        """Set genome state with validation."""
        if not (0 <= state <= 4):
            return Result.err(StateError.VALIDATION_FAILED)
        
        current_state = self._atomic_genome.load()
        
        # Validate transition
        transition_result = validate_state_transition(
            current_state, state, GENOME_TRANSITIONS
        )
        if transition_result.is_err:
            return transition_result
        
        # Atomic update
        with self._instance_lock:
            old_state = self._atomic_genome.load()
            self._atomic_genome.store(state)
            self._state.genome_state = state
            self._increment_version()
            
            # Log state change
            self._log_state_change("genome_state", old_state, state)
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._atomic_genome.store(old_state)
                self._state.genome_state = old_state
                return store_result
        
        return Result.ok(None)
    
    # === BURST ENGINE STATE MANAGEMENT ===
    
    def get_burst_engine_state(self) -> int:
        """Get current burst engine state."""
        return self._atomic_burst_engine.load()
    
    def set_burst_engine_state(self, state: int) -> Result[None]:
        """Set burst engine state with validation."""
        if not (0 <= state <= 7):
            return Result.err(StateError.VALIDATION_FAILED)
        
        current_state = self._atomic_burst_engine.load()
        
        # Validate transition
        transition_result = validate_state_transition(
            current_state, state, BURST_ENGINE_TRANSITIONS
        )
        if transition_result.is_err:
            return transition_result
        
        # Atomic update
        with self._instance_lock:
            old_state = self._atomic_burst_engine.load()
            self._atomic_burst_engine.store(state)
            self._state.burst_engine_state = state
            self._increment_version()
            
            self._log_state_change("burst_engine_state", old_state, state)
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._atomic_burst_engine.store(old_state)
                self._state.burst_engine_state = old_state
                return store_result
        
        return Result.ok(None)
    
    # === FQ SAMPLER STATE MANAGEMENT ===
    
    def get_fq_sampler_state(self) -> int:
        """Get current FQ sampler state."""
        return self._atomic_fq_sampler.load()
    
    def set_fq_sampler_state(self, state: int) -> Result[None]:
        """Set FQ sampler state with validation."""
        if not (0 <= state <= 4):
            return Result.err(StateError.VALIDATION_FAILED)
        
        current_state = self._atomic_fq_sampler.load()
        
        # Validate transition
        transition_result = validate_state_transition(
            current_state, state, FQ_SAMPLER_TRANSITIONS
        )
        if transition_result.is_err:
            return transition_result
        
        # CRITICAL: Validate prerequisites before allowing FQ sampler to initialize
        if state == 1:  # INITIALIZING
            prerequisites_result = self._validate_fq_sampler_prerequisites()
            if prerequisites_result.is_err:
                logger.warning("FQ sampler initialization blocked - prerequisites not met")
                return prerequisites_result
        
        # Atomic update
        with self._instance_lock:
            old_state = self._atomic_fq_sampler.load()
            self._atomic_fq_sampler.store(state)
            self._state.fq_sampler_state = state
            self._increment_version()
            
            self._log_state_change("fq_sampler_state", old_state, state)
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._atomic_fq_sampler.store(old_state)
                self._state.fq_sampler_state = old_state
                return store_result
        
        return Result.ok(None)
    
    # === BRAIN READINESS MANAGEMENT ===
    
    def get_brain_readiness(self) -> bool:
        """Get current brain readiness status."""
        return bool(self._atomic_brain_ready.load())
    
    def set_brain_readiness(self, ready: bool) -> Result[None]:
        """Set brain readiness with prerequisite validation."""
        new_state = 1 if ready else 0
        
        # CRITICAL: Validate prerequisites before setting brain ready
        if ready:
            prerequisites_result = self._validate_brain_readiness_prerequisites()
            if prerequisites_result.is_err:
                logger.warning("Brain readiness blocked - prerequisites not met")
                return prerequisites_result
        
        # Atomic update
        with self._instance_lock:
            old_state = self._atomic_brain_ready.load()
            self._atomic_brain_ready.store(new_state)
            self._state.brain_readiness = new_state
            self._increment_version()
            
            self._log_state_change("brain_readiness", old_state, new_state)
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._atomic_brain_ready.store(old_state)
                self._state.brain_readiness = old_state
                return store_result
        
        return Result.ok(None)
    
    # === EXIT CONDITION MANAGEMENT ===
    
    def get_exit_condition(self) -> bool:
        """Get current exit condition status."""
        return bool(self._state.exit_condition)
    
    def set_exit_condition(self, should_exit: bool) -> Result[None]:
        """Set exit condition for burst engine control."""
        new_state = 1 if should_exit else 0
        
        # Atomic update
        with self._instance_lock:
            old_state = 1 if self._state.exit_condition else 0
            self._state.exit_condition = should_exit
            self._increment_version()
            
            self._log_state_change("exit_condition", old_state, new_state)
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._state.exit_condition = not should_exit
                return store_result
        
        return Result.ok(None)
    
    # === BRAIN STATISTICS MANAGEMENT ===
    
    def get_brain_stats(self) -> Dict[str, Any]:
        """Get current brain statistics."""
        return getattr(self._state, 'brain_stats', {
            "neuron_count": self._state.neuron_count,
            "synapse_count": self._state.synapse_count,
            "cortical_area_count": self._state.cortical_area_count
        })
    
    def set_brain_stats(self, stats: Dict[str, Any]) -> Result[None]:
        """Set brain statistics."""
        if not isinstance(stats, dict):
            return Result.err(StateError.VALIDATION_FAILED)
        
        # Atomic update
        with self._instance_lock:
            # Store in both structured fields and as a dict for compatibility
            self._state.neuron_count = stats.get("neuron_count", 0)
            self._state.synapse_count = stats.get("synapse_count", 0)
            self._state.cortical_area_count = stats.get("cortical_area_count", 0)
            
            # Also store as brain_stats attribute for backward compatibility
            self._state.brain_stats = stats
            self._increment_version()
            
            self._log_state_change("brain_stats", {}, stats)
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                return store_result
        
        return Result.ok(None)
    
    # === CORTICAL LIST MANAGEMENT ===
    
    def get_cortical_list(self) -> List[str]:
        """Get current cortical area list."""
        return getattr(self._state, 'cortical_list', [])
    
    def set_cortical_list(self, cortical_ids: List[str]) -> Result[None]:
        """Set cortical area list."""
        if not isinstance(cortical_ids, list):
            return Result.err(StateError.VALIDATION_FAILED)
        
        # Atomic update
        with self._instance_lock:
            old_list = getattr(self._state, 'cortical_list', [])
            self._state.cortical_list = cortical_ids.copy()
            self._increment_version()
            
            self._log_state_change("cortical_list", len(old_list), len(cortical_ids))
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._state.cortical_list = old_list
                return store_result
        
        return Result.ok(None)
    
    # === GENOME VALIDITY MANAGEMENT ===
    
    def get_genome_validity(self) -> bool:
        """Get current genome validity status."""
        return getattr(self._state, 'genome_validity', False)
    
    def set_genome_validity(self, valid: bool) -> Result[None]:
        """Set genome validity status."""
        # Atomic update
        with self._instance_lock:
            old_validity = getattr(self._state, 'genome_validity', False)
            self._state.genome_validity = valid
            self._increment_version()
            
            self._log_state_change("genome_validity", old_validity, valid)
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._state.genome_validity = old_validity
                return store_result
        
        return Result.ok(None)
    
    # === CONNECTED AGENTS MANAGEMENT ===
    
    def get_connected_agents(self) -> Dict[str, Any]:
        """Get current connected agents registry."""
        return getattr(self._state, 'connected_agents', {})
    
    def set_connected_agents(self, agents: Dict[str, Any]) -> Result[None]:
        """Set connected agents registry."""
        if not isinstance(agents, dict):
            return Result.err(StateError.VALIDATION_FAILED)
        
        # Atomic update
        with self._instance_lock:
            old_agents = getattr(self._state, 'connected_agents', {})
            self._state.connected_agents = agents.copy()
            
            # Update agent count in structured state
            self._state.agent_count = len(agents)
            self._increment_version()
            
            self._log_state_change("connected_agents", len(old_agents), len(agents))
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._state.connected_agents = old_agents
                self._state.agent_count = len(old_agents)
                return store_result
        
        return Result.ok(None)
    
    def set_agent_count(self, count: int) -> Result[None]:
        """Set agent count (for compatibility)."""
        if count < 0:
            return Result.err(StateError.VALIDATION_FAILED)
        
        # Atomic update
        with self._instance_lock:
            old_count = self._state.agent_count
            self._state.agent_count = count
            self._increment_version()
            
            self._log_state_change("agent_count", old_count, count)
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._state.agent_count = old_count
                return store_result
        
        return Result.ok(None)
    
    # === CHANGES SAVED EXTERNALLY MANAGEMENT ===
    
    def get_changes_saved_externally(self) -> bool:
        """Get changes saved externally status."""
        return getattr(self._state, 'changes_saved_externally', False)
    
    def set_changes_saved_externally(self, saved: bool) -> Result[None]:
        """Set changes saved externally status."""
        # Atomic update
        with self._instance_lock:
            old_saved = getattr(self._state, 'changes_saved_externally', False)
            self._state.changes_saved_externally = saved
            self._increment_version()
            
            self._log_state_change("changes_saved_externally", old_saved, saved)
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._state.changes_saved_externally = old_saved
                return store_result
        
        return Result.ok(None)
    
    # === VALIDATION METHODS ===
    
    def _validate_fq_sampler_prerequisites(self) -> Result[None]:
        """Validate prerequisites for FQ sampler initialization."""
        # Check genome state
        if self._atomic_genome.load() != 2:  # Must be LOADED
            return Result.err(StateError.PREREQUISITE_NOT_MET)
        
        # Check burst engine state
        burst_state = self._atomic_burst_engine.load()
        if burst_state not in [2, 3]:  # Must be READY or ON_HOLD
            return Result.err(StateError.PREREQUISITE_NOT_MET)
        
        # Check brain readiness
        if self._atomic_brain_ready.load() != 1:  # Must be ready
            return Result.err(StateError.PREREQUISITE_NOT_MET)
        
        # Check neuroembryogenesis completion
        if self._state.neuroembryogenesis_stage != 5:  # Must be COMPLETED
            return Result.err(StateError.PREREQUISITE_NOT_MET)
        
        return Result.ok(None)
    
    def _validate_brain_readiness_prerequisites(self) -> Result[None]:
        """Validate prerequisites for brain readiness."""
        # Check genome state
        if self._atomic_genome.load() != 2:  # Must be LOADED
            return Result.err(StateError.PREREQUISITE_NOT_MET)
        
        # Check neuroembryogenesis completion
        if self._state.neuroembryogenesis_stage != 5:  # Must be COMPLETED
            return Result.err(StateError.PREREQUISITE_NOT_MET)
        
        return Result.ok(None)
    
    # === UTILITY METHODS ===
    
    def _increment_version(self) -> None:
        """Increment state version atomically."""
        old_version = self._atomic_version.fetch_add(1)
        self._state.state_version = old_version + 1
        self._state.last_modified = int(time.time() * 1000)
    
    def _log_state_change(self, field_name: str, old_value: int, new_value: int) -> None:
        """Log state change event."""
        event = StateChangeEvent(field_name, old_value, new_value)
        
        # Maintain fixed-size event log
        if len(self._event_log) >= self._max_events:
            self._event_log.pop(0)  # Remove oldest event
        
        self._event_log.append(event)
        logger.debug(f"State change: {field_name} {old_value} -> {new_value}")
    
    def get_state_version(self) -> int:
        """Get current state version."""
        return self._atomic_version.load()
    
    def get_comprehensive_state_report(self) -> Dict[str, Any]:
        """Get comprehensive state report for diagnostics."""
        with self._instance_lock:
            return {
                "timestamp": int(time.time() * 1000),
                "version": self.get_state_version(),
                "service_states": {
                    "genome": self.get_genome_state(),
                    "burst_engine": self.get_burst_engine_state(),
                    "fq_sampler": self.get_fq_sampler_state(),
                    "brain_readiness": self.get_brain_readiness(),
                },
                "development": {
                    "neuroembryogenesis_stage": self._state.neuroembryogenesis_stage,
                    "neuroembryogenesis_progress": self._state.neuroembryogenesis_progress,
                    "development_duration": self._state.development_duration,
                },
                "statistics": {
                    "agent_count": self._state.agent_count,
                    "neuron_count": self._state.neuron_count,
                    "synapse_count": self._state.synapse_count,
                    "cortical_area_count": self._state.cortical_area_count,
                },
                "validation": {
                    "fq_sampler_can_initialize": self._validate_fq_sampler_prerequisites().is_ok,
                    "brain_can_be_ready": self._validate_brain_readiness_prerequisites().is_ok,
                },
                "recent_events": [
                    {
                        "timestamp": event.timestamp,
                        "field": event.field_name,
                        "old_value": event.old_value,
                        "new_value": event.new_value,
                    }
                    for event in self._event_log[-10:]  # Last 10 events
                ]
            }
    
    def validate_state_consistency(self) -> List[str]:
        """Validate state consistency and return list of errors."""
        errors = []
        
        # Check brain readiness prerequisites
        if self.get_brain_readiness():
            if self.get_genome_state() != 2:
                errors.append("Brain ready but genome not loaded")
            if self._state.neuroembryogenesis_stage != 5:
                errors.append("Brain ready but neuroembryogenesis not completed")
        
        # Check FQ sampler prerequisites
        if self.get_fq_sampler_state() == 2:  # READY
            if self._validate_fq_sampler_prerequisites().is_err:
                errors.append("FQ samplers ready but prerequisites not met")
        
        return errors


def get_state_manager():
    """Get the singleton instance of FeagiStateManager"""
    return FeagiStateManager.instance()


def get_state_manager():
    """Get the singleton instance of FeagiStateManager"""
    return FeagiStateManager.instance()
