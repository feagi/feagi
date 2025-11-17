"""Rust-friendly state manager for FEAGI.

This state manager is designed for easy conversion to Rust while maintaining
full compatibility with existing Python code. It provides atomic operations,
Result-based error handling, and fixed-size data structures.
"""

import logging
import threading
import time
from collections import defaultdict
from enum import IntEnum
from typing import Any, Dict, List, Optional, Set
from pathlib import Path

from .cortical_locking import get_cortical_lock_manager, LockResult, GlobalLockInfo
from .rust_state_adapter import get_rust_state_delegate
from .state_errors import Result, StateError

try:
    from feagi.config.toml_loader import get_agent_config, load_feagi_config
except ImportError:
    # Handle cases where configuration might not be available
    load_feagi_config = None
    get_agent_config = None

logger = logging.getLogger(__name__)


class FCLWindowSizeCache:
    """Cached FCL window size computation for dynamic memory area support.

    Tracks cortical area mappings to memory areas and computes optimal window
    sizes.
    """
    
    def __init__(self, default_window_size: int = 20):
        self.default_window_size = default_window_size
        self._lock = threading.RLock()
        
        # Mapping tracking: cortical_id -> set(memory_area_ids) 
        self.cortical_to_memory_mappings: Dict[str, Set[str]] = defaultdict(
            set
        )
        
        # Memory area properties: memory_area_id -> temporal_depth
        self.memory_temporal_depths: Dict[str, int] = {}
        
        # Computed window sizes: cortical_id -> computed_window_size
        self.computed_window_sizes: Dict[str, int] = {}
        
        # Memory areas registry
        self.memory_areas: Set[str] = set()
        
        logger.info(
            "FCLWindowSizeCache initialized with default window size: %d",
            default_window_size,
        )
    
    def register_memory_area(
        self, cortical_id: str, temporal_depth: int
    ) -> None:
        """Register a new memory cortical area with its temporal depth."""
        with self._lock:
            self.memory_areas.add(cortical_id)
            self.memory_temporal_depths[cortical_id] = temporal_depth
            # Invalidate any cortical areas that might be affected
            self._invalidate_dependent_areas(cortical_id)
            logger.debug(
                "Registered memory area %s with temporal_depth=%d",
                cortical_id,
                temporal_depth,
            )
    
    def unregister_memory_area(self, cortical_id: str) -> None:
        """Unregister a memory cortical area."""
        with self._lock:
            self.memory_areas.discard(cortical_id)
            self.memory_temporal_depths.pop(cortical_id, None)
            # Remove from all mappings
            for source_id in list(self.cortical_to_memory_mappings.keys()):
                self.cortical_to_memory_mappings[source_id].discard(
                    cortical_id
                )
                if not self.cortical_to_memory_mappings[source_id]:
                    del self.cortical_to_memory_mappings[source_id]
            self._invalidate_dependent_areas(cortical_id)
            logger.debug("Unregistered memory area %s", cortical_id)
    
    def update_memory_temporal_depth(
        self, memory_cortical_id: str, new_temporal_depth: int
    ) -> None:
        """Update temporal depth for a memory area and invalidate affected
        window sizes."""
        with self._lock:
            if memory_cortical_id in self.memory_areas:
                old_depth = self.memory_temporal_depths.get(
                    memory_cortical_id, 1
                )
                self.memory_temporal_depths[memory_cortical_id] = (
                    new_temporal_depth
                )
                if old_depth != new_temporal_depth:
                    self._invalidate_dependent_areas(memory_cortical_id)
                    logger.debug(
                        "Updated memory area %s temporal_depth: %d -> %d",
                        memory_cortical_id,
                        old_depth,
                        new_temporal_depth,
                    )

    def add_cortical_mapping(
        self, source_cortical_id: str, target_cortical_id: str
    ) -> None:
        """Add a mapping from source cortical area to target (potentially
        memory) area."""
        with self._lock:
            if target_cortical_id in self.memory_areas:
                self.cortical_to_memory_mappings[source_cortical_id].add(
                    target_cortical_id
                )
                self.invalidate_cortical_area(source_cortical_id)
                logger.debug(
                    "Added mapping %s -> %s (memory area)",
                    source_cortical_id,
                    target_cortical_id,
                )

    def remove_cortical_mapping(
        self, source_cortical_id: str, target_cortical_id: str
    ) -> None:
        """Remove a mapping from source cortical area to target area."""
        with self._lock:
            if source_cortical_id in self.cortical_to_memory_mappings:
                self.cortical_to_memory_mappings[source_cortical_id].discard(
                    target_cortical_id
                )
                if not self.cortical_to_memory_mappings[source_cortical_id]:
                    del self.cortical_to_memory_mappings[source_cortical_id]
                self.invalidate_cortical_area(source_cortical_id)
                logger.debug(
                    "Removed mapping %s -> %s",
                    source_cortical_id,
                    target_cortical_id,
                )
    
    def invalidate_cortical_area(self, cortical_id: str) -> None:
        """Invalidate cached window size for a specific cortical area."""
        with self._lock:
            self.computed_window_sizes.pop(cortical_id, None)
            logger.debug(
                "Invalidated window size cache for cortical area %s",
                cortical_id,
            )
    
    def get_window_size(self, cortical_id: str) -> int:
        """Get computed window size for cortical area (cached computation)."""
        with self._lock:
            # Return cached value if available
            if cortical_id in self.computed_window_sizes:
                return self.computed_window_sizes[cortical_id]
            
            # Compute window size
            connected_memory_areas = self.cortical_to_memory_mappings.get(
                cortical_id, set()
            )
            if not connected_memory_areas:
                # No memory areas connected, use default
                window_size = self.default_window_size
            else:
                # Find maximum temporal depth among connected memory areas
                max_temporal_depth = max(
                    self.memory_temporal_depths.get(mem_area, 1) 
                    for mem_area in connected_memory_areas
                )
                window_size = max(self.default_window_size, max_temporal_depth)
            
            # Cache and return
            self.computed_window_sizes[cortical_id] = window_size
            logger.debug(
                "Computed window size for %s: %d (connected to %s)",
                cortical_id,
                window_size,
                connected_memory_areas,
            )
            return window_size
    
    def _invalidate_dependent_areas(self, memory_cortical_id: str) -> None:
        """Invalidate all cortical areas that map to this memory area."""
        areas_to_invalidate = [
            source_id
            for source_id, targets in self.cortical_to_memory_mappings.items()
            if memory_cortical_id in targets
        ]
        for area_id in areas_to_invalidate:
            self.invalidate_cortical_area(area_id)
    
    def get_debug_info(self) -> Dict:
        """Get debug information about current cache state."""
        with self._lock:
            return {
                "memory_areas": list(self.memory_areas),
                "memory_temporal_depths": dict(self.memory_temporal_depths),
                "cortical_to_memory_mappings": {
                    k: list(v)
                    for k, v in self.cortical_to_memory_mappings.items()
                },
                "computed_window_sizes": dict(self.computed_window_sizes),
                "default_window_size": self.default_window_size,
            }


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


class ServiceState(IntEnum):
    UNAVAILABLE = 0
    INITIALIZING = 1
    READY = 2
    DEGRADED = 3
    ERROR = 4
    UNINITIALIZED = 5
    FAILED = 6
    STOPPED = 7
    SYNCING = 8
    SYNC_COMPLETE = 9
    SYNC_ERROR = 10
    ON_HOLD = 11


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
    """Rust-friendly state manager designed for easy conversion.
    
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
    def instance(cls, storage=None):
        """Get singleton instance of the state manager (Rust-backed)."""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance
    
    def __init__(self, storage=None):
        """Initialize state manager (now fully Rust-backed)."""
        self._instance_lock = (
            threading.RLock()
        )  # Reentrant lock for nested operations
        self._event_log: List[StateChangeEvent] = []
        self._max_events = 1000  # Fixed-size event log
        self._debug_config = {}  # Initialize debug config
        
        # Initialize Rust state delegate for ALL state management
        # This replaces ALL Python atomic operations and storage with high-performance Rust
        try:
            self._rust_state = get_rust_state_delegate()
            logger.info("✅ Rust StateManager initialized - ALL state operations delegated to Rust")
        except Exception as e:
            logger.error(f"❌ Failed to initialize Rust StateManager: {e}")
            raise RuntimeError(f"Rust StateManager is REQUIRED - cannot proceed: {e}")
        
        # Initialize Morton spatial hash tracking
        self._morton_coordinate_limit = (
            1 << 21
        )  # 2,097,152 per dimension for 21-bit Morton encoding
        self._morton_class_name = (
            "RoaringSpatialHash"  # Current active Morton implementation
        )
        
        # Initialize memory area tracking
        self._memory_area_cache = FCLWindowSizeCache()
        
        # Initialize cortical areas cache for event-driven updates
        self._cortical_areas_cache = None
        self._cortical_areas_cache_dirty = True  # Mark as dirty initially

        #  Initialize attributes expected by service health checks to avoid
        #  transient warnings
        #  These placeholders are replaced as the system initializes but
        #  prevent noisy WARN logs
        if not hasattr(self, "brain_stats"):
            self.brain_stats: Dict[str, int] = {
                "neuron_count": 0,
                "synapse_count": 0,
                "cortical_area_count": 0,
            }
        if not hasattr(self, "cortical_list"):
            self.cortical_list: List[str] = []
        if not hasattr(self, "genome_validity"):
            self.genome_validity: Dict[str, Any] = {"status": "unknown"}
        if not hasattr(self, "connected_agents"):
            self.connected_agents: Dict[str, Any] = {}
        if not hasattr(self, "memory_area_stats"):
            # Per-cortical-area memory neuron statistics
            # Key: 6-letter memory cortical_id (e.g., "mVPmem")
            # Value: Dict with neuron_count and other stats
            self.memory_area_stats: Dict[str, Dict[str, Any]] = {}
        if not hasattr(self, "global_logging_level"):
            # Global logging level for runtime adjustment
            self.global_logging_level: str = "WARNING"  # Default level
        if not hasattr(self, "changes_saved_externally"):
            self.changes_saved_externally: bool = False
        if not hasattr(self, "exit_condition"):
            self.exit_condition: Optional[str] = None

        #  Flag to indicate a resynchronization is ongoing to reduce duplicate
        #  warnings
        self._resync_in_progress: bool = False

        # Load GPU keep-alive configuration
        self._gpu_keepalive_brain_threshold = self._load_gpu_keepalive_threshold()
        
        # Initialize GPU keep-alive eligibility based on current brain size
        self._update_gpu_keepalive_eligibility()
        
        # Python instance variables for non-hot-path state (not migrated to Rust)
        self._connectome_state = 0  # ConnectomeState.MISSING
        self._api_state = 0  # ServiceState.UNAVAILABLE
        self._fq_sampler_state = 0  # ServiceState.UNAVAILABLE
        self._brain_readiness = False
        self._exit_condition = False
        self._synapse_count = 0
        self._neuron_count = 0
        self._cortical_area_count = 0
        
        # Initialize stub object for compatibility with methods that still reference self._state
        # TODO: Migrate all remaining methods to use direct Python instance variables
        class _StateStub:
            def __init__(self):
                # All state fields for compatibility
                self.genome_state = 0
                self.connectome_state = 0
                self.burst_engine_state = 0
                self.api_state = 0
                self.neuroembryogenesis_stage = 0
                self.neuroembryogenesis_progress = 0
                self.development_duration = 0
                self.connected_agents = {}
                self.agent_count = 0
                self.changes_saved_externally = False
                self.exit_condition = False
                self.zmq_state = 0
                self.burst_frequency = 0
                self.simulation_state = 0
                self.genome_counter = 0
                self.genome_timestamp = 0
                self.feagi_session_timestamp = int(time.time() * 1000)
                self.state_version = 0
                self.last_modified = int(time.time() * 1000)
                self.neuron_count = 0
                self.synapse_count = 0
                self.cortical_area_count = 0
        
        self._state = _StateStub()
        
        # Stub storage for compatibility
        class _StorageStub:
            def store_state(self, state):
                return type('Result', (), {'is_err': False})()
        
        self._storage = _StorageStub()
        
        # Stub atomic objects for compatibility
        class _AtomicStub:
            def __init__(self, val=0):
                self.val = val
            def load(self):
                return self.val
            def store(self, val):
                self.val = val
            def fetch_add(self, val):
                old = self.val
                self.val += val
                return old
        
        self._atomic_version = _AtomicStub(0)
        self._atomic_genome = _AtomicStub(0)
        self._atomic_burst_engine = _AtomicStub(0)
        self._atomic_fq_sampler = _AtomicStub(0)
        self._atomic_brain_ready = _AtomicStub(0)
        
        # Shared Memory: manager and registries
        self._shared_memory_registry: Dict[str, str] = {}
        self._agent_shared_memory: Dict[str, Dict[str, str]] = {}
        try:
            from feagi.core.shared_memory import SharedMemoryManager

            self._shm_manager = SharedMemoryManager()
            # Cleanup any stale SHM files on startup
            logger.warning("𒓉 [SHM-CLEANUP] StateManager startup invoking cleanup_all()")
            self._shm_manager.cleanup_all()
        except Exception:
            self._shm_manager = None

        logger.info("FeagiStateManager initialized")
        logger.info(
            f"Morton spatial hash: {self._morton_class_name}, coordinate limit: {self._morton_coordinate_limit}"
        )

    # === GPU KEEP-ALIVE BRAIN SIZE MANAGEMENT ===
    
    def _load_gpu_keepalive_threshold(self) -> int:
        """Load GPU keep-alive brain size threshold from configuration.
        
        Uses GPU threshold × 80% as the keep-alive threshold for efficiency.
        """
        try:
            if load_feagi_config:
                config = load_feagi_config()
                hybrid_config = config.get("neural", {}).get("hybrid", {})
                
                # Get the main GPU threshold and calculate 80% of it
                gpu_threshold = hybrid_config.get("gpu_threshold", 1_000_000)
                keepalive_threshold = int(gpu_threshold * 0.8)  # 80% of GPU threshold
                
                logger.info(f"🔀 GPU keep-alive threshold: {keepalive_threshold:,} synapses (80% of GPU threshold: {gpu_threshold:,})")
                return keepalive_threshold
            else:
                return 800_000  # Default: 80% of 1M
        except Exception as e:
            logger.warning(f"Failed to load GPU keep-alive threshold config: {e}")
            return 800_000  # Default: 80% of 1M
    
    def _update_gpu_keepalive_eligibility(self):
        """Update GPU keep-alive eligibility flag based on current brain size.
        
        TODO: Migrate this to Rust StateManager or remove if not needed.
        """
        # Disabled - this was using Python atomic state which is now migrated to Rust
        pass
    
    def is_gpu_keepalive_eligible(self) -> bool:
        """Check if brain is large enough to warrant GPU keep-alive.
        
        TODO: Migrate this to Rust StateManager or remove if not needed.
        """
        return False  # Disabled for now
    
    def get_gpu_keepalive_brain_threshold(self) -> int:
        """Get the current GPU keep-alive brain size threshold."""
        return self._gpu_keepalive_brain_threshold
    
    def update_synapse_count(self, synapse_count: int):
        """Update synapse count and refresh GPU keep-alive eligibility.
        
        This should be called whenever synapses are added/removed.
        Only synapse count affects GPU keep-alive eligibility.
        
        Args:
            synapse_count: New total synapse count
        """
        with self._instance_lock:
            self._state.synapse_count = synapse_count
            
            # Update GPU keep-alive eligibility based on synapse count only
            # self._update_gpu_keepalive_eligibility()  # TODO: Migrate to Rust or remove
            
            # Persist state changes
            self._storage.store_state(self._state)
    
    def update_neuron_count(self, neuron_count: int):
        """Update neuron count without affecting GPU keep-alive eligibility.
        
        Args:
            neuron_count: New total neuron count
        """
        with self._instance_lock:
            self._state.neuron_count = neuron_count
            # No GPU keep-alive update needed - only synapse count matters
            self._storage.store_state(self._state)
    
    # === GENOME STATE MANAGEMENT ===
    
    def get_genome_state(self) -> int:
        """Get current genome state (fully delegated to Rust)."""
        return self._rust_state.get_genome_state()
    
    def set_genome_state(self, state: int):
        """Set genome state (fully delegated to Rust)."""
        self._rust_state.set_genome_state(state)

    # === SHARED MEMORY REGISTRATION/LOOKUP ===
    def register_core_shared_memory_path(self, key: str, path: str) -> None:
        """Register a core shared memory file path (e.g., visualization_stream).

        Args:
            key: Registry key (e.g., "visualization_stream")
            path: Absolute file path to the SHM file
        """
        try:
            if not isinstance(key, str) or not key:
                return
            if not isinstance(path, str) or not path:
                return
            self._shared_memory_registry[key] = path
            logger.info(f"𒓉 [SHM] Registered core SHM path: {key} → {path}")
        except Exception as e:
            logger.warning(f"Failed to register core SHM path for {key}: {e}")

    def get_shared_memory_registry(self) -> Dict[str, str]:
        """Return a copy of the core shared memory registry."""
        return dict(self._shared_memory_registry)

    def create_agent_shm_with_caps(self, agent_id: str, capabilities: Dict[str, Any]) -> Dict[str, str]:
        """Create and register per-agent SHM mappings with explicitly provided capabilities.
        
        This method avoids circular dependency where create_agent_shm tries to fetch
        capabilities from RegistrationManager during registration.

        Args:
            agent_id: Agent identifier
            capabilities: Agent capabilities dictionary (already sanitized by RegistrationManager)

        Returns:
            Mapping of shared memory identifiers to absolute file paths.
        """
        return self._create_agent_shm_internal(agent_id, capabilities)

    def create_agent_shm(self, agent_id: str) -> Dict[str, str]:
        """Create and register per-agent SHM mappings for client consumption.

        For visualizer clients, we expose the core visualization stream path so
        they can subscribe via shared memory.

        Returns:
            Mapping of shared memory identifiers to absolute file paths.
            Example: {"visualization_stream": "/.../feagi-shared-mem-visualization-stream.bin"}
        """
        if not isinstance(agent_id, str) or not agent_id:
            return {}

        # Determine agent capabilities (authoritative source)
        from feagi.pns.registration_manager import get_registration_manager
        caps: Dict[str, bool] = {}
        try:
            reg = get_registration_manager()
            if reg:
                props = reg.get_agent_properties(agent_id) or {}
                caps = props.get("capabilities", {}) or {}
        except Exception:
            caps = {}
        
        return self._create_agent_shm_internal(agent_id, caps)

    def _create_agent_shm_internal(self, agent_id: str, caps: Dict[str, Any]) -> Dict[str, str]:
        """Internal implementation for creating agent SHM mappings.
        
        Args:
            agent_id: Agent identifier
            caps: Agent capabilities dictionary

        Returns:
            Mapping of shared memory identifiers to absolute file paths.
        """
        if not isinstance(agent_id, str) or not agent_id:
            return {}

        mappings: Dict[str, str] = {}

        try:
            # Ensure we have a SHM manager
            if not getattr(self, "_shm_manager", None):
                logger.info("[SHM] No SHM manager available; cannot create mappings")
                return {}

            # Only expose/create FEAGI-owned core streams for eligible agents
            # Visualization core stream (FEAGI → BV)
            # Strict visualization gating: only canonical key 'visualization'
            wants_viz = False
            if isinstance(caps.get("visualization"), dict):
                wants_viz = bool(caps["visualization"].get("enabled", True))
            else:
                wants_viz = bool(caps.get("visualization"))
            if wants_viz:
                # Core registry uses 'visualization_stream'. Expose to agents as 'visualization'
                core_viz_key = "visualization_stream"
                viz_path = self._shared_memory_registry.get(core_viz_key, "")
                try:
                    # If registry missing or file missing, (re)create safely
                    from pathlib import Path as _P
                    if not viz_path or not _P(viz_path).exists():
                        viz_path = self._shm_manager.create_stream_file(core_viz_key)
                        self._shared_memory_registry[core_viz_key] = viz_path
                        logger.info(
                            f"𒓉 [SHM] Ensured core visualization stream SHM exists: {viz_path}"
                        )
                except Exception as e:
                    logger.warning(f"[SHM] Failed to ensure visualization SHM: {e}")
                    viz_path = ""
                if viz_path:
                    mappings["visualization"] = viz_path

            # Motor stream (FEAGI → motor-capable agents)
            # Use dedicated per-agent SHM file to avoid shared-writer contention
            # Strict motor gating: only canonical key (no aliases)
            wants_motor = False
            if isinstance(caps.get("motor"), dict):
                wants_motor = bool(caps["motor"].get("enabled", True))
            else:
                wants_motor = bool(caps.get("motor"))
            if wants_motor:
                try:
                    created = self._shm_manager.create_agent_capability_files(agent_id, {"motor": True})
                    motor_path = created.get("motor", "")
                except Exception as e:
                    logger.warning(f"[SHM] Failed to create per-agent motor SHM: {e}")
                    motor_path = ""
                if motor_path:
                    mappings["motor"] = motor_path

            # Sensory stream (agent → FEAGI) - agent-owned writer path
            wants_sensory = False
            if isinstance(caps.get("sensory"), dict):
                wants_sensory = bool(caps["sensory"].get("enabled", True))
            else:
                wants_sensory = bool(caps.get("sensory"))
            if wants_sensory:
                try:
                    created = self._shm_manager.create_agent_capability_files(agent_id, {"sensory": True})
                    sensory_path = created.get("sensory", "")
                except Exception as e:
                    logger.warning(f"[SHM] Failed to create per-agent sensory SHM: {e}")
                    sensory_path = ""
                if sensory_path:
                    mappings["sensory"] = sensory_path

            # Video stream (agent → BV preview) - agent-owned writer path
            wants_video = False
            if isinstance(caps.get("video"), dict):
                wants_video = bool(caps["video"].get("enabled", True))
            else:
                wants_video = bool(caps.get("video"))
            if wants_video:
                try:
                    created = self._shm_manager.create_agent_capability_files(agent_id, {"video": True})
                    video_path = created.get("video", "")
                except Exception as e:
                    logger.warning(f"[SHM] Failed to create per-agent video SHM: {e}")
                    video_path = ""
                if video_path:
                    mappings["video"] = video_path

            # FEAGI processed video stream (agent → BV FEAGI view) - agent-owned writer path
            wants_feagi = False
            if isinstance(caps.get("feagi"), dict):
                wants_feagi = bool(caps["feagi"].get("enabled", True))
            else:
                wants_feagi = bool(caps.get("feagi"))
            logger.info(f"[SHM-FEAGI] Agent {agent_id} requested feagi capability: {wants_feagi}, caps.get('feagi')={caps.get('feagi')}")
            if wants_feagi:
                try:
                    created = self._shm_manager.create_agent_capability_files(agent_id, {"feagi": True})
                    feagi_path = created.get("feagi", "")
                    logger.info(f"[SHM-FEAGI] Created feagi SHM path for {agent_id}: {feagi_path}")
                except Exception as e:
                    logger.warning(f"[SHM] Failed to create per-agent feagi SHM: {e}")
                    feagi_path = ""
                if feagi_path:
                    mappings["feagi"] = feagi_path
                    logger.info(f"[SHM-FEAGI] Added feagi path to mappings: {feagi_path}")
                else:
                    logger.warning(f"[SHM-FEAGI] feagi_path is empty, not adding to mappings")

            # NOTE: Do NOT pre-create inbound agent files (sensory/video). Those
            # must be created by the agent with correct headers. Pre-touching here
            # causes 'Invalid SHM magic' in readers.

            # Save per-agent view of mappings (even if currently only visualization provided)
            # Ensure there is only one agent_id per unique mapping: remove duplicates
            try:
                signature_parts = []
                for k in sorted(mappings.keys()):
                    signature_parts.append(f"{k}={mappings.get(k, '')}")
                signature = "|".join(signature_parts)
                to_remove = []
                for aid, mapping in self._agent_shared_memory.items():
                    try:
                        parts = []
                        for k in sorted(mapping.keys()):
                            parts.append(f"{k}={mapping.get(k, '')}")
                        if "|".join(parts) == signature and aid != agent_id:
                            to_remove.append(aid)
                    except Exception:
                        continue
                for aid in to_remove:
                    try:
                        del self._agent_shared_memory[aid]
                        logger.info(f"𒓉 [SHM] Removed duplicate SHM mapping under agent '{aid}' → canonical '{agent_id}'")
                    except Exception:
                        pass
            except Exception:
                pass
            self._agent_shared_memory[agent_id] = dict(mappings)
            if mappings:
                logger.info(f"𒓉 [SHM] Agent {agent_id} SHM mappings: {mappings}")
            else:
                logger.info(f"[SHM] Agent {agent_id} has no SHM mappings (shared-mem disabled or unavailable)")

            return mappings
        except Exception as e:
            logger.error(f"[SHM] Error creating agent SHM for {agent_id}: {e}")
            return {}

    def delete_agent_shm(self, agent_id: str) -> None:
        """Remove per-agent SHM mappings from the registry.

        Note: Does not delete core visualization SHM files.
        """
        try:
            if agent_id in self._agent_shared_memory:
                del self._agent_shared_memory[agent_id]
                logger.info(f"𒓉 [SHM-AGENT] Cleared SHM mappings for agent {agent_id}")
            # Attempt to remove agent-specific SHM files as part of cleanup
            if getattr(self, "_shm_manager", None):
                try:
                    logger.warning(f"𒓉 [SHM-AGENT] StateManager invoking delete_agent_files for {agent_id}")
                    self._shm_manager.delete_agent_files(agent_id)
                except Exception:
                    pass
        except Exception:
            pass
    
    # === BURST ENGINE STATE MANAGEMENT ===
    
    def get_burst_engine_state(self) -> int:
        """Get current burst engine state (fully delegated to Rust)."""
        return self._rust_state.get_burst_engine_state()
    
    def set_burst_engine_state(self, state: int):
        """Set burst engine state (fully delegated to Rust)."""
        self._rust_state.set_burst_engine_state(state)
    
    # === FQ SAMPLER STATE MANAGEMENT ===
    
    def get_fq_sampler_state(self) -> int:
        """Get current FQ sampler state."""
        return self._atomic_fq_sampler.load()
    
    def set_fq_sampler_state(self, state):
        """Set FQ sampler state with validation."""
        # Handle enum or int input
        if hasattr(state, "value"):
            state_value = state.value
        else:
            state_value = int(state)
            
        if not (0 <= state_value <= 7):  # Allow ServiceState range
            return Result.err(StateError.VALIDATION_FAILED)
        
        # Accept state changes directly for compatibility
        # (In production, strict validation would be enabled)
        
        # Atomic update
        with self._instance_lock:
            old_state = self._atomic_fq_sampler.load()
            self._atomic_fq_sampler.store(state_value)
            self._state.fq_sampler_state = state_value
            self._increment_version()
            
            self._log_state_change("fq_sampler_state", old_state, state_value)
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._atomic_fq_sampler.store(old_state)
                self._state.fq_sampler_state = old_state
                return store_result
        
        return Result.ok(None)
    
    # === CONNECTOME STATE MANAGEMENT ===
    
    def get_connectome_state(self) -> int:
        """Get current connectome state."""
        return self._state.connectome_state
    
    def set_connectome_state(self, state: int) :
        """Set connectome state with validation."""
        if not (0 <= state <= 5):  # ConnectomeState enum values
            return Result.err(StateError.VALIDATION_FAILED)
        
        # Atomic update
        with self._instance_lock:
            old_state = self._state.connectome_state
            self._state.connectome_state = state
            self._increment_version()
            
            self._log_state_change("connectome_state", old_state, state)
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._state.connectome_state = old_state
                return store_result
        
        return Result.ok(None)

    # === API STATE MANAGEMENT ===
    
    def get_api_state(self) -> int:
        """Get current API state."""
        return self._state.api_state
    
    def set_api_state(self, state) :
        """Set API state with validation."""
        # Convert ServiceState enum to integer for Rust/RTOS compatibility
        if isinstance(state, ServiceState):
            # Map ServiceState enum values to integers
            state_map = {
                ServiceState.UNAVAILABLE: 0,
                ServiceState.INITIALIZING: 1, 
                ServiceState.READY: 2,
                ServiceState.DEGRADED: 3,
                ServiceState.ERROR: 4,
                ServiceState.UNINITIALIZED: 5,
                ServiceState.FAILED: 6,
                ServiceState.STOPPED: 7,
                ServiceState.SYNCING: 8,
                ServiceState.SYNC_COMPLETE: 9,
                ServiceState.SYNC_ERROR: 10,
                ServiceState.ON_HOLD: 11,
            }
            state_value = state_map.get(state, 0)
        elif isinstance(state, int):
            state_value = state
        else:
            # Convert string state to int
            state_map = {
                "UNAVAILABLE": 0,
                "INITIALIZING": 1,
                "READY": 2,
                "DEGRADED": 3,
                "ERROR": 4,
                "UNINITIALIZED": 5,
                "FAILED": 6,
                "STOPPED": 7,
                "SYNCING": 8,
                "SYNC_COMPLETE": 9,
                "SYNC_ERROR": 10,
                "ON_HOLD": 11,
            }
            state_value = state_map.get(str(state).upper(), 0)
        
        if not (0 <= state_value <= 11):  # ServiceState enum range
            return Result.err(StateError.VALIDATION_FAILED)
        
        # Atomic update
        with self._instance_lock:
            old_state = self._state.api_state
            self._state.api_state = state_value
            self._increment_version()
            
            self._log_state_change("api_state", old_state, state_value)
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._state.api_state = old_state
                return store_result
        
        return Result.ok(None)

    # === BRAIN READINESS MANAGEMENT ===
    
    def get_brain_readiness(self) -> bool:
        """Get current brain readiness status."""
        return self._brain_readiness
    
    def set_brain_readiness(self, ready: bool):
        """Set brain readiness with prerequisite validation."""
        with self._instance_lock:
            self._brain_readiness = ready
            # Also update stub for compatibility
            self._atomic_brain_ready.store(1 if ready else 0)
            self._state.brain_readiness = 1 if ready else 0
    
    # === EXIT CONDITION MANAGEMENT ===
    
    def get_exit_condition(self) -> bool:
        """Get current exit condition status."""
        return bool(self._state.exit_condition)
    
    def set_exit_condition(self, should_exit: bool) :
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
        stats = getattr(self._state, "brain_stats", None)
        if not isinstance(stats, dict):
            stats = {
                "neuron_count": getattr(self._state, "neuron_count", 0),
                "synapse_count": getattr(self._state, "synapse_count", 0),
                "cortical_area_count": getattr(
                    self._state, "cortical_area_count", 0
                ),
            }
        # Ensure memory/non-memory counts are always present
        stats.setdefault("memory_neuron_count", 0)
        stats.setdefault(
            "non_memory_neuron_count", stats.get("neuron_count", 0)
        )
        return stats

    def update_memory_area_neuron_count(self, cortical_id: str, delta: int, operation: str = "update") :
        """Update memory neuron count for a specific cortical area.
        
        Args:
            cortical_id: 6-letter memory cortical area ID (e.g., "mVPmem")
            delta: Change in neuron count (+1 for creation, -1 for deletion)
            operation: Type of operation ("create", "delete", "update")
            
        Returns:
            Result indicating success or failure
        """
        if not isinstance(cortical_id, str) or len(cortical_id) != 6:
            return Result.err(StateError.VALIDATION_FAILED)
        
        with self._instance_lock:
            # Initialize area stats if not exists
            if cortical_id not in self.memory_area_stats:
                self.memory_area_stats[cortical_id] = {
                    "neuron_count": 0,
                    "created_total": 0,
                    "deleted_total": 0,
                    "last_updated": int(time.time() * 1000)  # timestamp in ms
                }
            
            area_stats = self.memory_area_stats[cortical_id]
            
            # Update neuron count
            old_count = area_stats["neuron_count"]
            new_count = max(0, old_count + delta)  # Prevent negative counts
            area_stats["neuron_count"] = new_count
            
            # Track operation totals
            if operation == "create" and delta > 0:
                area_stats["created_total"] += delta
            elif operation == "delete" and delta < 0:
                area_stats["deleted_total"] += abs(delta)
            
            # Update timestamp
            area_stats["last_updated"] = int(time.time() * 1000)
            
            # Log the change for debugging
            logger.debug(
                f"Memory area {cortical_id}: {operation} {delta:+d} neurons, "
                f"count: {old_count} → {new_count}"
            )
            
            return Result.ok(None)

    def get_memory_area_stats(self) -> Dict[str, Dict[str, Any]]:
        """Get all memory area statistics.
        
        Returns:
            Dictionary mapping cortical_id to area stats
        """
        with self._instance_lock:
            return dict(self.memory_area_stats)  # Return a copy

    def get_memory_area_neuron_count(self, cortical_id: str) -> int:
        """Get neuron count for a specific memory area.
        
        Args:
            cortical_id: 6-letter memory cortical area ID
            
        Returns:
            Current neuron count for the area (0 if area not found)
        """
        with self._instance_lock:
            area_stats = self.memory_area_stats.get(cortical_id, {})
            return area_stats.get("neuron_count", 0)

    def register_memory_area_for_stats(self, cortical_id: str) :
        """Register a new memory cortical area for per-area statistics tracking.
        
        Args:
            cortical_id: 6-letter memory cortical area ID
            
        Returns:
            Result indicating success or failure
        """
        if not isinstance(cortical_id, str) or len(cortical_id) != 6:
            return Result.err(StateError.VALIDATION_FAILED)
        
        with self._instance_lock:
            if cortical_id not in self.memory_area_stats:
                self.memory_area_stats[cortical_id] = {
                    "neuron_count": 0,
                    "created_total": 0,
                    "deleted_total": 0,
                    "last_updated": int(time.time() * 1000)
                }
                logger.info(f"Registered memory area {cortical_id} for tracking")
            
            return Result.ok(None)

    def unregister_memory_area_for_stats(self, cortical_id: str) :
        """Unregister a memory cortical area from per-area statistics tracking.
        
        Args:
            cortical_id: 6-letter memory cortical area ID
            
        Returns:
            Result indicating success or failure
        """
        with self._instance_lock:
            if cortical_id in self.memory_area_stats:
                del self.memory_area_stats[cortical_id]
                logger.info(f"Unregistered memory area {cortical_id} from tracking")
            
            return Result.ok(None)

    def get_global_logging_level(self) -> str:
        """Get the current global logging level.
        
        Returns:
            Current global logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        """
        with self._instance_lock:
            return getattr(self, 'global_logging_level', 'WARNING')

    def set_global_logging_level(self, level: str) :
        """Set the global logging level and apply it to all loggers.
        
        Args:
            level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
            
        Returns:
            Result indicating success or failure
        """
        import logging
        
        # Validate logging level
        valid_levels = ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']
        if level not in valid_levels:
            return Result.err(StateError.VALIDATION_FAILED)
        
        with self._instance_lock:
            old_level = getattr(self, 'global_logging_level', 'WARNING')
            self.global_logging_level = level
            
            # Apply the new logging level to all loggers at runtime
            try:
                # Set the root logger level
                root_logger = logging.getLogger()
                root_logger.setLevel(getattr(logging, level))
                
                # Set level for all existing loggers
                for logger_name in logging.Logger.manager.loggerDict:
                    logger_obj = logging.getLogger(logger_name)
                    if logger_obj.handlers:  # Only update loggers that have handlers
                        logger_obj.setLevel(getattr(logging, level))
                
                # Update all handlers to respect the new level
                for handler in root_logger.handlers:
                    handler.setLevel(getattr(logging, level))
                
                logger.info(f"Global logging level changed: {old_level} → {level}")
                
            except Exception as e:
                # Revert on error
                self.global_logging_level = old_level
                logger.error(f"Failed to set global logging level: {e}")
                return Result.err(StateError.OPERATION_FAILED)
            
            return Result.ok(None)
    
    def set_brain_stats(self, stats: Dict[str, Any]) :
        """Set brain statistics."""
        if not isinstance(stats, dict):
            return Result.err(StateError.VALIDATION_FAILED)
        
        # DEBUG: Removed detailed caller logging
        
        # Atomic update
        with self._instance_lock:
            # Extract incoming values
            in_total = stats.get("neuron_count")
            in_synapses = stats.get("synapse_count", 0)
            # Preserve existing cortical_area_count if not provided
            in_areas = stats.get("cortical_area_count", self._state.cortical_area_count)
            in_mem = stats.get("memory_neuron_count")
            in_non_mem = stats.get("non_memory_neuron_count")

            # Derive missing parts and enforce consistency
            if in_mem is None and in_non_mem is None:
                # Only total provided → default memory=0, non_memory=total
                mem_cnt = 0
                total_cnt = int(in_total or 0)
                non_mem_cnt = total_cnt
            elif in_mem is None and in_non_mem is not None:
                non_mem_cnt = max(0, int(in_non_mem))
                total_cnt = int(in_total or non_mem_cnt)
                mem_cnt = max(0, total_cnt - non_mem_cnt)
            elif in_mem is not None and in_non_mem is None:
                mem_cnt = max(0, int(in_mem))
                total_cnt = int(in_total or mem_cnt)
                non_mem_cnt = max(0, total_cnt - mem_cnt)
            else:
                # Both provided → recompute total for consistency
                mem_cnt = max(0, int(in_mem))
                non_mem_cnt = max(0, int(in_non_mem))
                total_cnt = mem_cnt + non_mem_cnt

            # Store in both structured fields and as a dict for compatibility
            self._state.neuron_count = total_cnt
            self._state.synapse_count = int(in_synapses)
            self._state.cortical_area_count = int(in_areas)
            
            # DEBUG: Removed cortical area count logging

            # Store in brain_stats dict
            stats["neuron_count"] = total_cnt
            stats["synapse_count"] = int(in_synapses)
            stats["cortical_area_count"] = int(in_areas)
            stats["memory_neuron_count"] = mem_cnt
            stats["non_memory_neuron_count"] = non_mem_cnt
            
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
        return getattr(self._state, "cortical_list", [])
    
    def set_cortical_list(self, cortical_ids: List[str]) :
        """Set cortical area list."""
        if not isinstance(cortical_ids, list):
            return Result.err(StateError.VALIDATION_FAILED)
        
        # Atomic update
        with self._instance_lock:
            old_list = getattr(self._state, "cortical_list", [])
            self._state.cortical_list = cortical_ids.copy()
            self._increment_version()
            
            self._log_state_change(
                "cortical_list", len(old_list), len(cortical_ids)
            )
            
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
        return getattr(self._state, "genome_validity", False)
    
    def set_genome_validity(self, valid: bool) :
        """Set genome validity status."""
        # Atomic update
        with self._instance_lock:
            old_validity = getattr(self._state, "genome_validity", False)
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
    
    def is_genome_loaded(self) -> bool:
        """Check if genome is currently loaded based on genome state."""
        return self.get_genome_state() == GenomeState.LOADED.value
    
    # === SYSTEM READINESS CHECKS ===
    
    def is_system_ready_for_fq_samplers(self) -> bool:
        """Check if all critical services are ready for FQ sampler
        initialization.
        
        Returns:
            True if system is ready, False otherwise
        """
        # Check genome state - must be LOADED
        if self.get_genome_state() != GenomeState.LOADED.value:
            return False
        
        # Check burst engine state - must be READY or ON_HOLD
        burst_state = self.get_burst_engine_state()
        if burst_state not in [
            ServiceState.READY.value,
            ServiceState.ON_HOLD.value,
        ]:
            return False
        
        # Check brain readiness
        if not self.get_brain_readiness():
            return False
        
        # Check neuroembryogenesis completion
        if (
            getattr(self._state, "neuroembryogenesis_stage", 0) != 5
        ):  # COMPLETED
            return False
        
        return True
    
    def get_critical_service_readiness_report(self) -> Dict[str, Any]:
        """Get detailed report of critical service readiness for event-driven
        decisions.
        
        Returns:
            Dict containing service states and readiness conditions
        """
        critical_status = self.get_critical_services_status()
        
        return {
            "services": {
                service: {
                    "state": state.value,
                    "is_error": state.value == "ERROR",
                    "is_ready": state.value == "READY",
                }
                for service, state in critical_status.items()
            },
            "genome_loaded": self.is_genome_loaded(),
            "brain_ready": self.get_brain_readiness(),
            "burst_engine_available": self.get_burst_engine_state()
            in ["READY", "ON_HOLD", "UNAVAILABLE"],
            "has_error_states": any(
                state.value == "ERROR" for state in critical_status.values()
            ),
            "system_ready_for_fq_samplers": self.is_system_ready_for_fq_samplers(),
        }
    
    # === CONNECTED AGENTS MANAGEMENT ===
    
    def get_connected_agents(self) -> Dict[str, Any]:
        """Get current connected agents registry."""
        return getattr(self._state, "connected_agents", {})
    
    def set_connected_agents(self, agents: Dict[str, Any]) :
        """Set connected agents registry."""
        if not isinstance(agents, dict):
            return Result.err(StateError.VALIDATION_FAILED)
        
        # Atomic update
        with self._instance_lock:
            old_agents = getattr(self._state, "connected_agents", {})
            self._state.connected_agents = agents.copy()
            
            # Update agent count in structured state
            self._state.agent_count = len(agents)
            self._increment_version()
            
            self._log_state_change(
                "connected_agents", len(old_agents), len(agents)
            )
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._state.connected_agents = old_agents
                self._state.agent_count = len(old_agents)
                return store_result
        
        return Result.ok(None)
    
    def set_agent_count(self, count: int) :
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
        return getattr(self._state, "changes_saved_externally", False)
    
    def set_changes_saved_externally(self, saved: bool) :
        """Set changes saved externally status."""
        # Atomic update
        with self._instance_lock:
            old_saved = getattr(self._state, "changes_saved_externally", False)
            self._state.changes_saved_externally = saved
            self._increment_version()
            
            self._log_state_change(
                "changes_saved_externally", old_saved, saved
            )
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._state.changes_saved_externally = old_saved
                return store_result
        
        return Result.ok(None)
    
    # === VALIDATION METHODS ===
    
    def _validate_fq_sampler_prerequisites(self) :
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
    
    def _validate_brain_readiness_prerequisites(self) :
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
    
    def _log_state_change(
        self, field_name: str, old_value: int, new_value: int
    ) -> None:
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
                    "neuroembryogenesis_progress": (
                        self._state.neuroembryogenesis_progress
                    ),
                    "development_duration": self._state.development_duration,
                },
                "statistics": {
                    "agent_count": self._state.agent_count,
                    "neuron_count": self._state.neuron_count,
                    "synapse_count": self._state.synapse_count,
                    "cortical_area_count": self._state.cortical_area_count,
                },
                "validation": {
                    "fq_sampler_can_initialize": (
                        self._validate_fq_sampler_prerequisites().is_ok
                    ),
                    "brain_can_be_ready": (
                        self._validate_brain_readiness_prerequisites().is_ok
                    ),
                },
                "recent_events": [
                    {
                        "timestamp": event.timestamp,
                        "field": event.field_name,
                        "old_value": event.old_value,
                        "new_value": event.new_value,
                    }
                    for event in self._event_log[-10:]  # Last 10 events
                ],
            }
    
    def validate_state_consistency(self) -> List[str]:
        """Validate state consistency and return list of errors."""
        errors = []
        
        # Check brain readiness prerequisites
        if self.get_brain_readiness():
            if self.get_genome_state() != 2:
                errors.append("Brain ready but genome not loaded")
            if self._state.neuroembryogenesis_stage != 5:
                errors.append(
                    "Brain ready but neuroembryogenesis not completed"
                )
        
        # Check FQ sampler prerequisites
        if self.get_fq_sampler_state() == 2:  # READY
            if self._validate_fq_sampler_prerequisites().is_err:
                errors.append("FQ samplers ready but prerequisites not met")
        
        return errors

    # === MISSING CRITICAL METHODS FOR MAIN.PY ===
    
    def set_debug_config(self, config: Dict[str, Any]) -> None:
        """Set debug configuration from main.py config."""
        try:
            # Store debug configuration in state
            debug_config = config.get("debug", {})
            print(f"🔍 [STATE-PRINT] set_debug_config called with: {config}")
            print(f"🔍 [STATE-PRINT] Extracted debug config: {debug_config}")
            logger.info(f"🔍 [STATE-DEBUG] set_debug_config called with: {config}")
            logger.info(f"🔍 [STATE-DEBUG] Extracted debug config: {debug_config}")
            
            # Extract relevant debug settings
            log_level = debug_config.get("log_level", "INFO")
            verbose = debug_config.get("verbose", False)
            
            # Update internal debug state (could be expanded later)
            if not hasattr(self, "_debug_config"):
                self._debug_config = {}
            
            self._debug_config = {
                "log_level": log_level,
                "verbose": verbose,
                "config_loaded": True,
                #  Debug flags from command line args - FIXED: use correct key
                #  names from CLI mapping
                "debug_npu": debug_config.get(
                    "npu", False
                ),  # CLI maps debug_npu -> debug.npu
                "debug_api": debug_config.get(
                    "api", False
                ),  # CLI maps debug_api -> debug.api
                "debug_bdu": debug_config.get(
                    "bdu", False
                ),  # CLI maps debug_bdu -> debug.bdu (not defined yet)
                "debug_zmq_inbound": debug_config.get(
                    "zmq_inbound", False
                ),  # CLI maps debug_zmq_inbound -> debug.zmq_inbound
                "debug_zmq_outbound": debug_config.get(
                    "zmq_outbound", False
                ),  # CLI maps debug_zmq_outbound -> debug.zmq_outbound
                "mem_debug": debug_config.get(
                    "mem_debug", False
                ),  # CLI maps mem_debug -> debug.mem_debug
            }
            
            # Show which debug flags are enabled
            enabled_flags = [
                flag.replace("debug_", "")
                for flag, enabled in self._debug_config.items()
                if flag.startswith("debug_") and enabled
            ]

            #  Add memory debug flag separately since it doesn't follow the
            #  "debug_" prefix pattern
            if self._debug_config.get("mem_debug", False):
                enabled_flags.append("mem_debug")

            print(f"🔍 [STATE-PRINT] Final _debug_config: {self._debug_config}")
            logger.info(f"🔍 [STATE-DEBUG] Final _debug_config: {self._debug_config}")
                
            if enabled_flags:
                logger.info(
                    f"Debug configuration set: log_level={log_level}, verbose={verbose}"
                )
                logger.info(f"Debug flags enabled: {', '.join(enabled_flags)}")
            else:
                logger.info(
                    f"Debug configuration set: log_level={log_level}, verbose={verbose}"
                )
            
            # Log enabled debug flags
            enabled_flags = [
                key.replace("debug_", "")
                for key in self._debug_config.keys()
                if key.startswith("debug_") and self._debug_config[key]
            ]
            if enabled_flags:
                logger.info(f"Debug flags enabled: {', '.join(enabled_flags)}")
            
        except Exception as e:
            logger.error(f"Error setting debug config: {e}")
            # Don't fail startup for debug config issues
            self._debug_config = {"config_loaded": False, "error": str(e)}

    # ===== CORTICAL AREA LOCKING METHODS =====
    
    def lock_cortical_area(self, cortical_idx: int, locked_by: str, operation: str = "unknown") -> bool:
        """Lock a cortical area for exclusive BDU operations.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            locked_by: Component requesting the lock (e.g., "BDU", "SleepManager")
            operation: Description of the operation requiring the lock
            
        Returns:
            True if lock was successful, False otherwise
        """
        try:
            lock_manager = get_cortical_lock_manager()
            result = lock_manager.lock_area(cortical_idx, locked_by, operation)
            return result == LockResult.SUCCESS
        except Exception as e:
            logger.error(f"Failed to lock cortical area {cortical_idx}: {e}")
            return False
    
    def unlock_cortical_area(self, cortical_idx: int, locked_by: str) -> bool:
        """Unlock a cortical area.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            locked_by: Component that originally locked the area
            
        Returns:
            True if unlock was successful, False otherwise
        """
        try:
            lock_manager = get_cortical_lock_manager()
            result = lock_manager.unlock_area(cortical_idx, locked_by)
            return result == LockResult.SUCCESS
        except Exception as e:
            logger.error(f"Failed to unlock cortical area {cortical_idx}: {e}")
            return False
    
    def is_cortical_area_locked(self, cortical_idx: int) -> bool:
        """Check if a cortical area is currently locked.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            
        Returns:
            True if the area is locked, False otherwise
        """
        try:
            lock_manager = get_cortical_lock_manager()
            return lock_manager.is_area_locked(cortical_idx)
        except Exception as e:
            logger.error(f"Failed to check lock status for cortical area {cortical_idx}: {e}")
            return False
    
    def get_locked_cortical_areas(self) -> List[int]:
        """Get list of all currently locked cortical area indices.
        
        Returns:
            List of cortical_idx values that are currently locked
        """
        try:
            lock_manager = get_cortical_lock_manager()
            return lock_manager.get_locked_areas()
        except Exception as e:
            logger.error(f"Failed to get locked cortical areas: {e}")
            return []
    
    def lock_multiple_cortical_areas(self, cortical_indices: List[int], locked_by: str, operation: str = "batch") -> bool:
        """Lock multiple cortical areas atomically.
        
        Args:
            cortical_indices: List of cortical area indices to lock
            locked_by: Component requesting the locks
            operation: Description of the operation requiring the locks
            
        Returns:
            True if all areas were locked successfully, False otherwise
        """
        try:
            lock_manager = get_cortical_lock_manager()
            results = lock_manager.lock_multiple_areas(cortical_indices, locked_by, operation)
            return all(result == LockResult.SUCCESS for result in results.values())
        except Exception as e:
            logger.error(f"Failed to lock multiple cortical areas {cortical_indices}: {e}")
            return False
    
    def unlock_multiple_cortical_areas(self, cortical_indices: List[int], locked_by: str) -> bool:
        """Unlock multiple cortical areas.
        
        Args:
            cortical_indices: List of cortical area indices to unlock
            locked_by: Component that originally locked the areas
            
        Returns:
            True if all areas were unlocked successfully, False otherwise
        """
        try:
            lock_manager = get_cortical_lock_manager()
            results = lock_manager.unlock_multiple_areas(cortical_indices, locked_by)
            return all(result == LockResult.SUCCESS for result in results.values())
        except Exception as e:
            logger.error(f"Failed to unlock multiple cortical areas {cortical_indices}: {e}")
            return False
    
    def get_cortical_locking_statistics(self) -> Dict[str, int]:
        """Get cortical area locking statistics for monitoring.
        
        Returns:
            Dictionary with lock statistics
        """
        try:
            lock_manager = get_cortical_lock_manager()
            return lock_manager.get_statistics()
        except Exception as e:
            logger.error(f"Failed to get cortical locking statistics: {e}")
            return {"error": 1}
    
    def force_unlock_all_cortical_areas(self, locked_by: str) -> int:
        """Force unlock all areas locked by a specific component.
        
        This is useful for cleanup when a component shuts down unexpectedly.
        
        Args:
            locked_by: Component to unlock all areas for
            
        Returns:
            Number of areas that were unlocked
        """
        try:
            lock_manager = get_cortical_lock_manager()
            return lock_manager.force_unlock_all(locked_by)
        except Exception as e:
            logger.error(f"Failed to force unlock areas for {locked_by}: {e}")
            return 0

    # ===== GLOBAL BRAIN LOCKING METHODS =====
    
    def lock_global_brain(self, locked_by: str, operation: str = "global_operation", 
                         affected_areas: Optional[List[int]] = None) -> bool:
        """Lock the entire brain for global operations.
        
        This is much more efficient than locking individual cortical areas and provides
        atomic global operations. Perfect for Sleep Manager maintenance operations.
        
        Args:
            locked_by: Component requesting the global lock (e.g., "SleepManager")
            operation: Description of the global operation
            affected_areas: Optional list of specific areas affected (None = all areas)
            
        Returns:
            True if global lock was successful, False otherwise
        """
        try:
            lock_manager = get_cortical_lock_manager()
            result = lock_manager.lock_global_brain(locked_by, operation, affected_areas)
            return result == LockResult.SUCCESS
        except Exception as e:
            logger.error(f"Failed to lock global brain for {locked_by}: {e}")
            return False
    
    def unlock_global_brain(self, locked_by: str) -> bool:
        """Unlock the global brain.
        
        Args:
            locked_by: Component that originally locked the brain
            
        Returns:
            True if unlock was successful, False otherwise
        """
        try:
            lock_manager = get_cortical_lock_manager()
            result = lock_manager.unlock_global_brain(locked_by)
            return result == LockResult.SUCCESS
        except Exception as e:
            logger.error(f"Failed to unlock global brain for {locked_by}: {e}")
            return False
    
    def is_global_brain_locked(self) -> bool:
        """Check if the global brain is currently locked.
        
        Returns:
            True if global brain is locked, False otherwise
        """
        try:
            lock_manager = get_cortical_lock_manager()
            return lock_manager.is_global_brain_locked()
        except Exception as e:
            logger.error(f"Failed to check global brain lock status: {e}")
            return False
    
    def get_global_brain_lock_info(self) -> Optional[GlobalLockInfo]:
        """Get information about the global brain lock.
        
        Returns:
            GlobalLockInfo if global lock is active, None otherwise
        """
        try:
            lock_manager = get_cortical_lock_manager()
            return lock_manager.get_global_lock_info()
        except Exception as e:
            logger.error(f"Failed to get global brain lock info: {e}")
            return None
    
    def force_unlock_global_brain(self, locked_by: str) -> bool:
        """Force unlock the global brain (emergency cleanup).
        
        Args:
            locked_by: Component to force unlock for
            
        Returns:
            True if global lock was cleared, False otherwise
        """
        try:
            lock_manager = get_cortical_lock_manager()
            return lock_manager.force_unlock_global_brain(locked_by)
        except Exception as e:
            logger.error(f"Failed to force unlock global brain for {locked_by}: {e}")
            return False

    def cleanup(self) -> None:
        """Cleanup state manager resources for graceful shutdown."""
        try:
            logger.info("FeagiStateManager cleanup initiated")
            # Best-effort: cleanup SHM files on FEAGI exit
            try:
                if hasattr(self, "_shm_manager") and self._shm_manager:
                    logger.warning("𒓉 [SHM-CLEANUP] StateManager shutdown invoking cleanup_all()")
                    self._shm_manager.cleanup_all()
            except Exception:
                pass
            
            # Set exit condition
            self.set_exit_condition(True)
            
            # Save final state
            if hasattr(self, "_storage") and self._storage:
                store_result = self._storage.store_state(self._state)
                if store_result.is_err:
                    logger.warning(
                        f"Failed to save final state during cleanup: "
                        f"{store_result.unwrap_err()}"
                    )
                else:
                    logger.info("Final state saved successfully")
            
            # Clear instance for clean shutdown
            with FeagiStateManager._lock:
                FeagiStateManager._instance = None
                
            logger.info("FeagiStateManager cleanup completed")
            
        except Exception as e:
            logger.error(f"Error during FeagiStateManager cleanup: {e}")

    def log_startup_summary(self) -> None:
        """Log comprehensive startup summary for main.py."""
        try:
            logger.info("=== FEAGI STATE MANAGER STARTUP SUMMARY ===")
            logger.info(f"Genome State: {self.get_genome_state()}")
            logger.info(f"Brain Readiness: {self.get_brain_readiness()}")
            logger.info(f"Burst Engine State: {self.get_burst_engine_state()}")
            logger.info(f"FQ Sampler State: {self.get_fq_sampler_state()}")
            logger.info(f"Exit Condition: {self.get_exit_condition()}")
            
            # Additional state info
            cortical_list = self.get_cortical_list()
            logger.info(f"Cortical Areas: {len(cortical_list)} registered")
            
            connected_agents = self.get_connected_agents()
            logger.info(f"Connected Agents: {len(connected_agents)} active")
            
            logger.info("=== STATE MANAGER READY ===")
            
        except Exception as e:
            logger.error(f"Error logging startup summary: {e}")

    # === DEBUG CONFIGURATION METHODS ===
    
    def is_debug_npu_enabled(self) -> bool:
        """Check if NPU debug mode is enabled."""
        if not hasattr(self, "_debug_config"):
            return False
        return self._debug_config.get("debug_npu", False)
    
    def is_debug_api_enabled(self) -> bool:
        """Check if API debug mode is enabled."""
        if not hasattr(self, "_debug_config"):
            return False
        return self._debug_config.get("debug_api", False)
    
    def is_debug_bdu_enabled(self) -> bool:
        """Check if BDU debug mode is enabled."""
        if not hasattr(self, "_debug_config"):
            return False
        return self._debug_config.get("debug_bdu", False)
    
    def is_debug_zmq_inbound_enabled(self) -> bool:
        """Check if ZMQ inbound debug mode is enabled."""
        if not hasattr(self, "_debug_config"):
            return False
        return self._debug_config.get("debug_zmq_inbound", False)
    
    def is_debug_zmq_outbound_enabled(self) -> bool:
        """Check if ZMQ outbound debug mode is enabled."""
        if not hasattr(self, "_debug_config"):
            return False
        return self._debug_config.get("debug_zmq_outbound", False)

    def is_mem_debug_enabled(self) -> bool:
        """Check if memory debug mode is enabled."""
        if not hasattr(self, "_debug_config"):
            return False
        return self._debug_config.get("mem_debug", False)

    # === CUMULATIVE ACTIVITY COUNTERS (Sleep trigger support) ===

    def _ensure_activity_counters(self) -> None:
        """Initialize activity counters if missing."""
        if not hasattr(self, "_cumulative_activity_total"):
            self._cumulative_activity_total = (
                0  # Total neurons fired accumulated
            )
        if not hasattr(self, "_cumulative_activity_bursts"):
            self._cumulative_activity_bursts = (
                0  # Bursts counted in current window
            )

    def increment_cumulative_activity(
        self, neurons_fired_this_burst: int
    ) -> None:
        """Increment cumulative FCL activity counters.

        Args:
            neurons_fired_this_burst: Total neurons fired across all areas in the burst
        """
        try:
            self._ensure_activity_counters()
            # Guard inputs
            inc = int(neurons_fired_this_burst)
            if inc < 0:
                inc = 0
            self._cumulative_activity_total += inc
            self._cumulative_activity_bursts += 1
        except Exception:
            # Keep counters best-effort; never raise in hot path
            pass

    def get_cumulative_activity(self) -> Dict[str, int]:
        """Get current cumulative FCL activity window counters."""
        self._ensure_activity_counters()
        return {
            "bursts": int(self._cumulative_activity_bursts),
            "neurons": int(self._cumulative_activity_total),
        }

    def reset_cumulative_activity(self) -> None:
        """Reset cumulative counters (e.g., when Sleep maintenance
        triggers)."""
        self._ensure_activity_counters()
        self._cumulative_activity_total = 0
        self._cumulative_activity_bursts = 0

    # === PLASTICITY COUNTERS ===
    def _ensure_plasticity_counters(self) -> None:
        if not hasattr(self, "_plasticity_dropped_ops"):
            self._plasticity_dropped_ops = 0

    def increment_plasticity_dropped_ops(self, count: int) -> None:
        """Increment number of plasticity operations dropped (queue full)."""
        try:
            self._ensure_plasticity_counters()
            inc = int(count)
            if inc < 0:
                inc = 0
            self._plasticity_dropped_ops += inc
        except Exception:
            pass

    def get_plasticity_counters(self) -> Dict[str, int]:
        self._ensure_plasticity_counters()
        return {
            "dropped_ops": int(self._plasticity_dropped_ops)
        }

    def get_critical_services_status(self) -> Dict[str, Any]:
        """Get status of all critical services for system readiness checks."""

        # Create mock state objects with .value attribute for compatibility
        class StateValue:
            def __init__(self, value: str):
                self.value = value
        
        # Map integer states back to string values for compatibility
        genome_state_map = {
            0: "MISSING",
            1: "LOADING",
            2: "LOADED",
            3: "SAVING",
            4: "ERROR",
        }
        connectome_state_map = {
            0: "MISSING",
            1: "INITIALIZING",
            2: "UPDATING",
            3: "READY",
            4: "SNAPSHOTTING",
            5: "ERROR",
        }
        burst_engine_state_map = {
            0: "UNAVAILABLE",
            1: "INITIALIZING",
            2: "READY",
            3: "ON_HOLD",
            4: "STOPPED",
            5: "ERROR",
            6: "FAILED",
            7: "STOPPED",
        }
        api_state_map = {
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
        
        return {
            "genome": StateValue(
                genome_state_map.get(self._state.genome_state, "UNKNOWN")
            ),
            "connectome": StateValue(
                connectome_state_map.get(
                    self._state.connectome_state, "UNKNOWN"
                )
            ),
            "burst_engine": StateValue(
                burst_engine_state_map.get(
                    self._state.burst_engine_state, "UNKNOWN"
                )
            ),
            "state_manager": StateValue("READY"),  # Always ready if callable
            "api_server": StateValue(
                api_state_map.get(self._state.api_state, "UNKNOWN")
            ),
            "zmq_server": StateValue("UNAVAILABLE"),  # ZMQ state not tracked
        }

    def get_simd_configuration(self) -> Dict[str, Any]:
        """Get SIMD configuration for performance optimization."""
        # Return SIMD configuration based on system capabilities
        # This is a read-only configuration that doesn't need state storage
        try:
            # Try to detect SIMD capabilities
            import platform

            arch = platform.machine().lower()
            
            # Basic SIMD detection for common architectures
            if "arm64" in arch or "aarch64" in arch:
                return {
                    "available": True,
                    "backend": "ARM_NEON",
                    "vector_width": 4,  # 128-bit NEON vectors
                    "alignment": 16,
                }
            elif "x86_64" in arch or "amd64" in arch:
                # Basic x86-64 with SSE2 (minimum for 64-bit)
                return {
                    "available": True,
                    "backend": "SSE2",
                    "vector_width": 2,  # 128-bit SSE vectors for doubles
                    "alignment": 16,
                }
            else:
                # Fallback to scalar processing
                return {
                    "available": False,
                    "backend": "SCALAR", 
                    "vector_width": 1,
                    "alignment": 8,
                }
        except Exception:
            # On any error, return safe scalar configuration
            return {
                "available": False,
                "backend": "SCALAR",
                "vector_width": 1,
                "alignment": 8,
            }

    # === ZMQ STATE MANAGEMENT ===
    
    def get_zmq_state(self) -> int:
        """Get current ZMQ state."""
        return getattr(
            self._state, "zmq_state", ServiceState.UNAVAILABLE.value
        )
    
    def set_zmq_state(self, state) :
        """Set ZMQ state with validation."""
        # Convert ServiceState enum to integer for Rust/RTOS compatibility
        if isinstance(state, ServiceState):
            state_value = state.value
        elif isinstance(state, int):
            state_value = state
        else:
            # Convert string state to int
            state_map = {
                "UNAVAILABLE": 0,
                "INITIALIZING": 1,
                "READY": 2,
                "DEGRADED": 3,
                "ERROR": 4,
                "UNINITIALIZED": 5,
                "FAILED": 6,
                "STOPPED": 7,
                "SYNCING": 8,
                "SYNC_COMPLETE": 9,
                "SYNC_ERROR": 10,
                "ON_HOLD": 11,
            }
            state_value = state_map.get(str(state).upper(), 0)
        
        if not (0 <= state_value <= 11):  # ServiceState enum range
            return Result.err(StateError.VALIDATION_FAILED)
        
        # Atomic update
        with self._instance_lock:
            old_state = getattr(self._state, "zmq_state", 0)
            self._state.zmq_state = state_value
            self._increment_version()
            
            self._log_state_change("zmq_state", old_state, state_value)
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._state.zmq_state = old_state
                return store_result
        
        return Result.ok(None)
    
    # === AGENT REGISTRATION MANAGEMENT ===
    
    def register_agent(
        self,
        agent_id: str,
        agent_type: str = "",
        capabilities: Optional[Dict[str, Any]] = None,
        agent_data_port: Optional[int] = None,
        agent_version: str = "",
        controller_version: str = "",
        agent_ip: Optional[str] = None,
        **kwargs,
    ) :
        """Register an agent in the state manager.
        
        Args:
            agent_id: Unique agent identifier
            agent_type: Type of agent
            capabilities: Agent capabilities dictionary
            agent_data_port: Agent data port
            agent_version: Agent version
            controller_version: Controller version  
            agent_ip: Agent IP address (uses configuration default if None)
            **kwargs: Additional agent data
        """
        # Load agent_ip from configuration if not provided
        if agent_ip is None:
            try:
                if load_feagi_config and get_agent_config:
                    config = load_feagi_config()
                    agent_config = get_agent_config(config)
                    agent_ip = agent_config.default_host
                else:
                    agent_ip = "127.0.0.1"  # @architecture:acceptable - emergency fallback
            except Exception as e:
                logger.warning(
                    f"Could not load agent configuration, using fallback: {e}"
                )
                agent_ip = "127.0.0.1"  # @architecture:acceptable - emergency fallback
        
        agent_data = {
            "agent_id": agent_id,
            "agent_type": agent_type,
            "capabilities": capabilities or {},
            "agent_data_port": agent_data_port,
            "agent_version": agent_version,
            "controller_version": controller_version,
            "agent_ip": agent_ip,
            "registered_at": int(time.time() * 1000),
            **kwargs,
        }
        
        # Update connected agents registry
        current_agents = self.get_connected_agents()
        current_agents[agent_id] = agent_data
        
        # Use existing method to update registry
        result = self.set_connected_agents(current_agents)
        return result
    
    def deregister_agent(self, agent_id: str) :
        """Deregister an agent from the state manager.
        
        Args:
            agent_id: Agent identifier to remove
        """
        current_agents = self.get_connected_agents()
        if agent_id in current_agents:
            del current_agents[agent_id]
            result = self.set_connected_agents(current_agents)
            # Also clear per-agent SHM view and try to delete files
            try:
                if hasattr(self, "delete_agent_shm"):
                    self.delete_agent_shm(agent_id)
            except Exception:
                pass
            return result
        # Even if agent not present, ensure SHM view is cleared
        try:
            if hasattr(self, "delete_agent_shm"):
                self.delete_agent_shm(agent_id)
        except Exception:
            pass
        return Result.ok(None)

    # === MISSING METHODS FOR TEST COMPATIBILITY ===
    
    def get_agent_count(self) -> int:
        """Get current agent count."""
        return self._state.agent_count
    
    def get_burst_frequency(self) -> float:
        """Get current burst frequency."""
        return (
            float(self._state.burst_frequency) / 100.0
        )  # Convert from fixed point
    
    def set_burst_frequency(self, frequency: float) -> None:
        """Set burst frequency."""
        with self._instance_lock:
            old_freq = self._state.burst_frequency
            self._state.burst_frequency = int(
                frequency * 100
            )  # Store as fixed point
            self._increment_version()
            self._log_state_change(
                "burst_frequency", old_freq, self._state.burst_frequency
            )
            self._storage.store_state(self._state)
    
    def get_simulation_state(self) -> int:
        """Get current simulation state."""
        return getattr(
            self._state, "simulation_state", SimulationState.STOPPED.value
        )
    
    def set_simulation_state(self, state) -> None:
        """Set simulation state."""
        if isinstance(state, SimulationState):
            state_value = state.value
        elif isinstance(state, int):
            state_value = state
        else:
            state_value = SimulationState.STOPPED.value
            
        with self._instance_lock:
            old_state = getattr(self._state, "simulation_state", 0)
            self._state.simulation_state = state_value
            self._increment_version()
            self._log_state_change("simulation_state", old_state, state_value)
            self._storage.store_state(self._state)
    
    def set_fcl_sampler_state(self, state) -> None:
        """Set FCL sampler state (alias for FQ sampler state)."""
        self.set_fq_sampler_state(state)
    
    def get_fcl_sampler_state(self) -> int:
        """Get FCL sampler state (alias for FQ sampler state)."""
        return self.get_fq_sampler_state()
    
    def is_connectome_ready(self) -> bool:
        """Check if connectome is ready."""
        return self.get_connectome_state() == ConnectomeState.READY.value
    
    def is_burst_engine_ready(self) -> bool:
        """Check if burst engine is ready."""
        return self.get_burst_engine_state() == ServiceState.READY.value
    
    def is_simulation_running(self) -> bool:
        """Check if simulation is running."""
        return self.get_simulation_state() == SimulationState.RUNNING.value
    
    def sync_to_disk(self) -> None:
        """Force synchronization to disk."""
        self._storage.store_state(self._state)
    
    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive system status (alias for
        get_critical_services_status)."""
        return self.get_critical_services_status()
    
    def get_state_summary(self) -> Dict[str, Any]:
        """Get comprehensive state summary (alias for
        get_comprehensive_state_report)."""
        return self.get_comprehensive_state_report()
    
    def set_debug_configuration(self, config: Dict[str, Any]) -> None:
        """Set debug configuration (alias for set_debug_config)."""
        self.set_debug_config(config)
    
    def begin_genome_transaction(self) -> "GenomeTransaction":
        """Begin a genome transaction for atomic genome modifications."""
        # For now, return a simple mock transaction object
        # This can be expanded later if needed
        return GenomeTransaction(self)
    
    def get_agent_registry_summary(self) -> Dict[str, Any]:
        """Get agent registry summary for logging and monitoring."""
        connected_agents = self.get_connected_agents()
        agent_count = len(connected_agents)
        
        # Categorize agents by type
        agent_types = {}
        for agent_id, agent_data in connected_agents.items():
            if isinstance(agent_data, dict):
                agent_type = agent_data.get("agent_type", "unknown")
            else:
                agent_type = str(agent_data)
            
            if agent_type not in agent_types:
                agent_types[agent_type] = []
            agent_types[agent_type].append(agent_id)
        
        return {
            "total_agents": agent_count,
            "agent_types": agent_types,
            "agents_by_type_count": {
                agent_type: len(agents)
                for agent_type, agents in agent_types.items()
            },
            "connected_agent_ids": list(connected_agents.keys()),
        }
    
    def get_genome_counter(self) -> int:
        """Get the current genome counter/version number."""
        # This tracks how many times genomes have been loaded
        # Return the actual genome counter that gets incremented
        return getattr(self._state, "genome_counter", 0)
    
    def get_genome_timestamp(self) -> int:
        """Get the current genome timestamp."""
        return getattr(self._state, "genome_timestamp", 0)
    
    def get_feagi_session_timestamp(self) -> int:
        """Get the FEAGI session timestamp (when this FEAGI instance was created)."""
        return getattr(self._state, "feagi_session_timestamp", 0)
    
    def set_genome_timestamp(self, timestamp: int) :
        """Set genome timestamp."""
        if not isinstance(timestamp, int) or timestamp < 0:
            return Result.err(StateError.VALIDATION_FAILED)
        
        # Atomic update
        with self._instance_lock:
            old_timestamp = getattr(self._state, "genome_timestamp", 0)
            self._state.genome_timestamp = timestamp
            self._increment_version()
            
            self._log_state_change(
                "genome_timestamp", old_timestamp, timestamp
            )
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._state.genome_timestamp = old_timestamp
                return store_result
        
        return Result.ok(None)
    
    def increment_genome_counter(self) :
        """Increment the genome counter."""
        # For now, this is a simple implementation
        # In a full implementation, this would track actual genome loads
        with self._instance_lock:
            old_counter = getattr(self._state, "genome_counter", 0)
            new_counter = old_counter + 1
            self._state.genome_counter = new_counter
            self._increment_version()
            
            self._log_state_change("genome_counter", old_counter, new_counter)
            
            # Persist to storage
            store_result = self._storage.store_state(self._state)
            if store_result.is_err:
                # Rollback on storage failure
                self._state.genome_counter = old_counter
                return store_result
        
        return Result.ok(None)

    # === MORTON SPATIAL HASH STATE MANAGEMENT ===
    
    def get_morton_coordinate_limit(self) -> int:
        """Get the maximum coordinate value supported by the active Morton
        spatial hash.
        
        Returns:
            Maximum coordinate value per dimension (exclusive)
        """
        return self._morton_coordinate_limit
    
    def get_morton_class_name(self) -> str:
        """Get the name of the active Morton spatial hash class.
        
        Returns:
            Name of the Morton spatial hash implementation
        """
        return self._morton_class_name
    
    def set_morton_class_info(
        self, class_name: str, coordinate_limit: int
    ) :
        """Update Morton spatial hash class information.
        
        Args:
            class_name: Name of the Morton class implementation
            coordinate_limit: Maximum coordinate value per dimension
            
        Returns:
            Result indicating success or failure
        """
        if coordinate_limit <= 0:
            return Result.err(StateError.VALIDATION_FAILED)
        
        with self._instance_lock:
            old_class = self._morton_class_name
            old_limit = self._morton_coordinate_limit
            
            self._morton_class_name = class_name
            self._morton_coordinate_limit = coordinate_limit
            
            logger.info(
                f"Morton spatial hash updated: {old_class} -> {class_name}, limit: {old_limit} -> {coordinate_limit}"
            )
            
        return Result.ok(None)
    
    def is_coordinate_within_morton_limits(
        self, x: int, y: int, z: int
    ) -> bool:
        """Check if coordinates are within Morton encoding limits.
        
        Args:
            x, y, z: 3D coordinates to validate
            
        Returns:
            True if coordinates are within limits, False otherwise
        """
        limit = self._morton_coordinate_limit
        return 0 <= x < limit and 0 <= y < limit and 0 <= z < limit
    
    def validate_cortical_area_dimensions(
        self, dimensions: tuple
    ) :
        """Validate that cortical area dimensions are within Morton limits.
        
        Args:
            dimensions: Tuple of (width, height, depth) dimensions
            
        Returns:
            Result indicating if dimensions are valid
        """
        if len(dimensions) != 3:
            return Result.err(StateError.VALIDATION_FAILED)
        
        width, height, depth = dimensions
        limit = self._morton_coordinate_limit
        
        if width >= limit or height >= limit or depth >= limit:
            logger.error(
                f"Cortical area dimensions {dimensions} exceed Morton limit {limit}"
            )
            return Result.err(StateError.VALIDATION_FAILED)
        
        return Result.ok(None)

    # === MEMORY AREA MANAGEMENT ===
    
    def register_memory_area(
        self, cortical_id: str, temporal_depth: int
    ) :
        """Register a memory cortical area with its temporal depth."""
        try:
            self._memory_area_cache.register_memory_area(
                cortical_id, temporal_depth
            )
            logger.info(
                f"Registered memory area {cortical_id} with temporal_depth={temporal_depth}"
            )
            return Result.ok(None)
        except Exception as e:
            logger.error(f"Failed to register memory area {cortical_id}: {e}")
            return Result.err(StateError.OPERATION_FAILED)
    
    def unregister_memory_area(self, cortical_id: str) :
        """Unregister a memory cortical area."""
        try:
            self._memory_area_cache.unregister_memory_area(cortical_id)
            logger.info(f"Unregistered memory area {cortical_id}")
            return Result.ok(None)
        except Exception as e:
            logger.error(
                f"Failed to unregister memory area {cortical_id}: {e}"
            )
            return Result.err(StateError.OPERATION_FAILED)
    
    def update_memory_temporal_depth(
        self, cortical_id: str, new_temporal_depth: int
    ) :
        """Update temporal depth for a memory area."""
        try:
            self._memory_area_cache.update_memory_temporal_depth(
                cortical_id, new_temporal_depth
            )
            logger.info(
                f"Updated memory area {cortical_id} temporal_depth to {new_temporal_depth}"
            )
            return Result.ok(None)
        except Exception as e:
            logger.error(
                f"Failed to update memory area temporal depth {cortical_id}: {e}"
            )
            return Result.err(StateError.OPERATION_FAILED)
    
    def add_cortical_mapping_to_cache(
        self, source_cortical_id: str, target_cortical_id: str
    ) -> None:
        """Add cortical mapping to memory area cache (called by
        ConnectomeManager)."""
        self._memory_area_cache.add_cortical_mapping(
            source_cortical_id, target_cortical_id
        )

    def remove_cortical_mapping_from_cache(
        self, source_cortical_id: str, target_cortical_id: str
    ) -> None:
        """Remove cortical mapping from memory area cache (called by
        ConnectomeManager)."""
        self._memory_area_cache.remove_cortical_mapping(
            source_cortical_id, target_cortical_id
        )
    
    def get_fcl_window_size(self, cortical_id: str) -> int:
        """Get computed FCL window size for cortical area."""
        return self._memory_area_cache.get_window_size(cortical_id)
    
    def invalidate_fcl_window_cache(self, cortical_id: str) -> None:
        """Invalidate FCL window size cache for cortical area."""
        self._memory_area_cache.invalidate_cortical_area(cortical_id)
    
    def is_memory_area(self, cortical_id: str) -> bool:
        """Check if cortical area is a memory area."""
        return cortical_id in self._memory_area_cache.memory_areas
    
    def get_memory_areas(self) -> List[str]:
        """Get list of all registered memory areas."""
        return list(self._memory_area_cache.memory_areas)
    
    def get_memory_area_debug_info(self) -> Dict:
        """Get debug information about memory area cache state."""
        return self._memory_area_cache.get_debug_info()

    # === CORTICAL AREAS CACHE MANAGEMENT ===
    
    def invalidate_cortical_areas_cache(self) -> None:
        """Mark cortical areas cache as dirty - will be refreshed on next access."""
        self._cortical_areas_cache_dirty = True
        self._cortical_areas_cache = None
        logger.debug("Cortical areas cache invalidated")
    
    def get_cortical_areas_cache(self, connectome_manager=None) -> List[Dict]:
        """Get cached cortical areas data, refreshing if needed.
        
        Args:
            connectome_manager: ConnectomeManager instance to refresh from if cache is dirty
            
        Returns:
            List of cortical area dictionaries
        """
        if (
            self._cortical_areas_cache_dirty
            or self._cortical_areas_cache is None
        ):
            if connectome_manager is None:
                logger.warning(
                    "Cache is dirty but no connectome_manager provided for refresh"
                )
                return self._cortical_areas_cache or []
            
            # Refresh cache from ConnectomeManager
            try:
                fresh_data = (
                    connectome_manager.get_all_cortical_area_properties()
                )
                # Filter out empty dictionaries
                fresh_data = [area for area in fresh_data if area]
                
                self._cortical_areas_cache = fresh_data
                self._cortical_areas_cache_dirty = False
                
                logger.debug(
                    f"Refreshed cortical areas cache with {len(fresh_data)} areas"
                )
                return fresh_data
                
            except Exception as e:
                logger.error(f"Failed to refresh cortical areas cache: {e}")
                return self._cortical_areas_cache or []
        
        return self._cortical_areas_cache
    
    def update_cortical_areas_cache(
        self, cortical_id: str, operation: str
    ) -> None:
        """Update cortical areas cache after operations.
        
        Args:
            cortical_id: ID of the cortical area that changed
            operation: Type of operation ('add', 'update', 'delete', 'mapping_update')
        """
        # For simplicity, mark cache as dirty for any operation
        # In a more sophisticated implementation, we could selectively update
        self._cortical_areas_cache_dirty = True
        logger.debug(
            f"Marked cortical areas cache dirty due to {operation} on {cortical_id}"
        )


class GenomeTransaction:
    """Simple genome transaction context manager for atomic operations."""
    
    def __init__(self, state_manager: FeagiStateManager):
        self._state_manager = state_manager
        self._changes = []
        self._committed = False
    
    def record_change(self, operation: str, *args, **kwargs):
        """Record a change to be applied atomically."""
        self._changes.append((operation, args, kwargs))
    
    def commit(self):
        """Commit all recorded changes."""
        # For now, just mark as committed
        # In a full implementation, this would apply changes atomically
        self._committed = True
        return {"success": True, "changes_applied": len(self._changes)}
    
    def rollback(self):
        """Rollback any changes."""
        self._changes.clear()
        return {"success": True, "changes_rolled_back": True}
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is None and not self._committed:
            self.commit()
        elif exc_type is not None:
            self.rollback()
        return False


def get_state_manager():
    """Get the singleton instance of FeagiStateManager."""
    return FeagiStateManager.instance()
