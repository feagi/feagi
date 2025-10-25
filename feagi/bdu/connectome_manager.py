"""ConnectomeManager for the BDU optimized for GPU processing.

This module provides a GPU-optimized implementation of the connectome manager
using NumPy arrays and sparse matrices for efficient data processing and
transfer to GPU memory.
"""

import logging
from collections import defaultdict
from enum import Enum
from typing import Any, Dict, List, Optional, Set, Tuple, Union

import numpy as np

from feagi.bdu.cortical_mapping import BiDirectionalCorticalMap
from feagi.core.state_manager import get_state_manager
from feagi.bdu.models.cortical_area import CorticalArea

# Import models
from feagi.npu.data_structures import BackendType

# Import utility functions
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class NeuronPropertyType(Enum):
    """Types of neuron properties that can be accessed/modified."""

    MEMBRANE_POTENTIAL = "membrane_potential"
    RESTING_POTENTIAL = "resting_potential"
    THRESHOLD = "threshold"
    REFRACTORY_PERIOD = "refractory_period"
    SNOOZE_PERIOD = "snooze_period"  # Extended refractory period
    LEAK_COEFFICIENT = "leak_coefficient"
    CORTICAL_IDX = "cortical_idx"
    POSITION = "position"
    FIRING = "firing"
    REFRACTORY_COUNTER = "refractory_counter"
    ACTIVE = "is_active"


class ConnectomeManager:
    """Manager for creating and manipulating the neural connectome with GPU/CPU
    optimization.

    This high-performance ConnectomeManager uses Structure of Arrays (SoA) format
    for neuron storage, providing massive memory efficiency improvements over
    the legacy dictionary-based approach.

    Features:
    - 90% memory reduction vs legacy implementation
    - GPU acceleration when available (CUDA, Metal, WebGPU)
    - CPU fallback with vectorized NumPy operations
    - SIMD-optimized operations on supported platforms
    - Drop-in replacement for legacy ConnectomeManager

    Backend Selection (automatic):
    1. PyTorch (GPU if available, CPU fallback)
    2. CuPy (CUDA GPU)
    3. WebGPU (when available)
    4. NumPy (CPU, always available)

    This class implements a singleton pattern to ensure mission-critical reliability
    and performance in FEAGI's multi-process architecture.
    """

    # Singleton pattern implementation
    _instance = None
    _initialized = False
    
    # Debug: Log class definition
    logger.info("🔧 ConnectomeManager class being defined")

    @classmethod
    def instance(
        cls,
        config_or_max_neurons=10_000_000,
        max_synapses=100_000_000,
        backend=None,
    ):
        """Get the singleton instance of ConnectomeManager.

        Args:
            config_or_max_neurons: Either a FeagiConfig object or the maximum number of neurons (only used on first call)
            max_synapses: Maximum number of synapses (only used if first parameter is an integer)
            backend: Backend type to use ('numpy', 'pytorch', 'cupy', 'wgpu', or auto) - only used on first call

        Returns:
            The singleton ConnectomeManager instance
        """
        if cls._instance is None:
            logger.info(f"🔧 Creating NEW ConnectomeManager singleton with backend: '{backend}'")
            cls._instance = cls.__new__(cls)
            cls._instance.__init__(
                config_or_max_neurons, max_synapses, backend
            )
            cls._initialized = True
            logger.info(
                "Created singleton ConnectomeManager instance (optimized SoA version)",
                status="[TARGET]",
            )
        else:
            logger.warning(f"🔧 ConnectomeManager singleton already exists! Ignoring backend: '{backend}'")
            logger.warning(f"🔧 Existing NPU Interface: {getattr(cls._instance, '_npu_interface', 'NOT_SET')}")
            from feagi.core.state_manager import get_state_manager

            state_manager = get_state_manager()
            if state_manager.is_debug_bdu_enabled():
                logger.debug(
                    "[BDU-DEBUG] Returning existing ConnectomeManager singleton",
                    status="[LINK]",
                )
        return cls._instance

    @classmethod
    def reset_singleton(cls):
        """Reset the singleton instance for testing purposes.

        This method is used by tests to ensure clean state between test runs.
        """
        cls._instance = None
        cls._initialized = False
        logger.debug("Reset ConnectomeManager singleton for testing")

    def __new__(cls, *args, **kwargs):
        """Override __new__ to enforce singleton pattern."""
        import traceback
        logger.info(f"🔧 ConnectomeManager.__new__ called with args: {args}, kwargs: {kwargs}")
        logger.info(f"🔧 Call stack:\n{traceback.format_stack()[-3:-1]}")
        
        if cls._instance is not None and cls._initialized:
            logger.warning(
                "Attempted to create multiple ConnectomeManager instances - returning singleton",
                status="[LINK]",
            )
            return cls._instance
        # Create new instance if none exists or if reset was called
        logger.info("🔧 Creating new ConnectomeManager instance via __new__")
        instance = super().__new__(cls)
        cls._instance = instance
        return instance

    def __init__(
        self,
        config_or_max_neurons: Union[Dict[str, Any], int],
        max_synapses: int = 100_000_000,
        backend: str = "cpu",

    ):
        """Initialize the ConnectomeManager.

        Args:
            config_or_max_neurons: Either a configuration dictionary or maximum number of neurons
            max_synapses: Maximum number of synapses per neuron (default: 100M)
            backend: Backend to use for computations ("cpu" or "cuda")

        """
        # Debug: Print to stderr to bypass logging issues
        import sys
        print(f"🔧 ConnectomeManager.__init__ called on {id(self)} with backend: '{backend}'", file=sys.stderr, flush=True)
        
        # Initialize logger
        self.logger = logging.getLogger(__name__)

        logger.info(f"🔧 ConnectomeManager.__init__ called with backend: '{backend}'")
        logger.info(f"🔧 _initialized flag: {ConnectomeManager._initialized}")

        if ConnectomeManager._initialized:
            logger.info("🔧 ConnectomeManager already initialized, but checking NPU Interface...")
            # Even if already initialized, ensure NPU Interface is set up if it wasn't before
            if not hasattr(self, '_npu_interface') or self._npu_interface is None:
                logger.info(f"🔧 NPU Interface not set, initializing with backend: '{backend}'")
                self._initialize_npu_interface(backend)
            else:
                logger.info(f"🔧 NPU Interface already exists: {self._npu_interface}")
            return

        # Handle legacy parameter passing and dynamic sizing
        # Store these values to pass to NPU Interface later
        if isinstance(config_or_max_neurons, dict):
            config = config_or_max_neurons
            self._max_neurons_config = config.get("max_neurons", 10_000_000)
            max_synapses = config.get("max_synapses", max_synapses)
            backend = config.get("backend", backend)
        elif hasattr(config_or_max_neurons, "get"):  # FeagiConfig object
            config = config_or_max_neurons
            # NEW: Dynamic sizing based on genome stats and configuration
            self._max_neurons_config = self._calculate_neuron_space(config)
            max_synapses = self._calculate_synapse_space(config, max_synapses)
            backend = config.get("connectome.backend", backend)
        else:
            self._max_neurons_config = config_or_max_neurons

        self._max_synapses_config = max_synapses

        #  CRITICAL: Do NOT create neuron array here - NPU Interface owns it
        #  ConnectomeManager will get reference to NPU Interface's neuron array
        #  when set_npu_interface() is called
        self.neuron_array = None  # Will be set by NPU Interface

        #  Neuron ID counter will be initialized when NPU Interface is set
        #  This prevents memory corruption from reusing IDs as indices

        # CRITICAL: Initialize NPU Interface immediately to own all data structures
        # This must happen BEFORE neurogenesis so that neuron creation works
        logger.info(f"🔧 ConnectomeManager initializing with backend: '{backend}'")
        print(f"🔧 About to initialize NPU Interface with backend: '{backend}'", file=sys.stderr, flush=True)
        self._initialize_npu_interface(backend)
        print(f"🔧 NPU Interface initialized: {self._npu_interface}", file=sys.stderr, flush=True)

        # Memory area tracking for pattern processing
        self.memory_areas: Set[str] = set()
        self.memory_area_upstream_mappings: Dict[str, Set[str]] = defaultdict(
            set
        )  # memory_area -> upstream areas

        # Initialize cortical areas and brain regions
        self.cortical_areas: Dict[str, CorticalArea] = {}
        self.brain_regions = {}
        self.region_area_map = {}
        
        # Initialize hierarchical brain region system
        from feagi.bdu.models.brain_region_hierarchy import BrainRegionHierarchy
        self.brain_region_hierarchy = BrainRegionHierarchy()
        
        # Initialize Rust Morton spatial hash for ultra-fast position lookups
        self._rust_morton_hash = None
        try:
            from feagi_bdu import PyMortonSpatialHash
            self._rust_morton_hash = PyMortonSpatialHash()
            logger.info("🦀 Rust Morton spatial hash enabled for ConnectomeManager")
        except ImportError as e:
            logger.debug(f"Rust Morton hash not available: {e}")

        # Initialize connectivity rules and cortical connections storage
        self.connectivity_rules = {}
        self.cortical_connections = {}

        #  Core area reservations - cortical_idx=0 for "_death", cortical_idx=1
        #  for "_power"
        self.reserved_cortical_areas = {"_death": 0, "_power": 1}

        # Initialize bidirectional cortical mapping
        self.cortical_mapping = BiDirectionalCorticalMap()

        # REMOVED: Legacy _neuron_to_position dictionary (dead code)
        # NPU interface owns neuron_to_position mapping as single source of truth

        #  Cache management for property-based access (prevents memory
        #  corruption)
        self._cache_invalidated = True

        # Neuron ID management - delegate to NeuronArray
        self.next_neuron_id = 1

        # CRITICAL: Do NOT create synapse array here - NPU Interface owns it
        # ARCHITECTURE: Synapses managed directly by Rust NPU (no Python synapse_array)
        # Access via self._npu_interface.rust_npu.* methods

        #  FCL manager is now owned by NPU BurstEngine, not BDU ConnectomeManager
        #  This maintains backward compatibility for any code that expects fcl_manager attribute
        self.fcl_manager = None  # Will be set by NPU when BurstEngine is created
        
        # NPU interface reference - NPU is PRIMARY OWNER of synaptic updates
        # NOTE: _npu_interface_internal is already set by _initialize_npu_interface() above

        # Initialize active neurons tracking
        self.active_neurons = np.zeros(self.max_neurons, dtype=np.bool_)
        self.current_timestep = 0



        # Initialize state manager for brain size tracking
        self.state_manager = get_state_manager()

        # Backward compatibility for tests - store the instance
        ConnectomeManager._instance = self
        
        ConnectomeManager._initialized = True
        
        logger.info(
            "✅ ConnectomeManager initialized with Rust NPU backend",
            status="[OK]",
        )

    #  ============================================================================
    # Neuron Existence Checks - Delegates to Rust NPU (Single Source of Truth)
    #  ============================================================================

    def has_neuron(self, neuron_id: int) -> bool:
        """Check if a neuron ID exists.
        
        ARCHITECTURE: neuron_id == array_index, so just check if neuron exists in Rust NPU.
        """
        if self._npu_interface:
            return self._npu_interface.neuron_exists(neuron_id)
        if hasattr(self, "memory_neuron_array") and self.memory_neuron_array:
            return neuron_id in self.memory_neuron_array.neuron_id_to_index
        return False

    def _get_fcl_manager(self):
        """Get FCL manager from NPU BurstEngine.
        
        Since FCL manager is now owned by NPU, we need to get it from BurstEngine.
        This maintains backward compatibility for BDU code that needs FCL access.
        
        Returns:
            FCL manager instance from NPU, or None if not available
        """
        if self.fcl_manager is not None:
            return self.fcl_manager
            
        # 🦀 RUST: FCL is now managed directly by Rust NPU
        # Python FCL manager is deprecated - FCL operations go through NPU Interface
        return None
    
    def _get_async_fcl_processor(self):
        """Get async FCL processor from NPU BurstEngine.
        
        🦀 RUST: Async FCL processing is now handled by Rust NPU.
        Python async processor is deprecated.
        
        Returns:
            None (deprecated - Rust handles FCL processing)
        """
        return None

    # ======================================================================
    # DYNAMIC SIZING METHODS - GENOME-BASED MEMORY OPTIMIZATION
    # ======================================================================

    def resize_for_genome(self, genome_data: Dict[str, Any]) -> bool:
        """Resize the connectome based on genome requirements.

        This method is called after genome loading to optimize memory usage
        based on actual genome requirements.

        Args:
            genome_data: Loaded genome data containing stats

        Returns:
            True if resizing was successful, False otherwise
        """
        try:
            # ✅ RUST NPU: Skip dynamic resizing - capacity is fixed at initialization
            if self._npu_interface and hasattr(self._npu_interface, '_rust_npu_integration'):
                logger.info(
                    "🦀 [RUST-NPU] Skipping dynamic resize - capacity is fixed at initialization",
                    status="[OK]"
                )
                logger.info(
                    f"ℹ️  [DYNAMIC SIZING] Connectome size is optimal (capacity: {self.max_neurons:,} neurons, {self.max_synapses:,} synapses)"
                )
                return True
            
            # Extract genome stats
            stats = genome_data.get("stats", {})
            genome_neuron_count = stats.get("innate_neuron_count", 0)
            genome_synapse_count = stats.get("innate_synapse_count", 0)

            if genome_neuron_count == 0:
                logger.warning(
                    "⚠️  [DYNAMIC SIZING] No genome neuron count found - keeping current size"
                )
                return False

            # Get configuration values (use defaults if not available)
            min_neuron_space = 100_000  # Default minimum
            min_synapse_space = 500_000  # Default minimum
            buffer_multiplier = 1.5  # Default 50% buffer

            # Calculate optimal sizes
            buffered_neuron_requirement = int(
                genome_neuron_count * buffer_multiplier
            )
            buffered_synapse_requirement = int(
                genome_synapse_count * buffer_multiplier
            )

            optimal_neuron_size = max(
                buffered_neuron_requirement, min_neuron_space
            )
            optimal_synapse_size = max(
                buffered_synapse_requirement, min_synapse_space
            )

            logger.info(
                "🧠 [DYNAMIC SIZING] Genome-based connectome resizing:"
            )
            logger.info(f"   Genome neurons: {genome_neuron_count:,}")
            logger.info(f"   Genome synapses: {genome_synapse_count:,}")
            logger.info(f"   Current neuron capacity: {self.max_neurons:,}")
            logger.info(f"   Optimal neuron capacity: {optimal_neuron_size:,}")

            #  CRITICAL FIX: Skip dynamic sizing if neurons already exist
            #  (e.g., during test mode)
            #  This prevents spatial hash corruption and neuron loss during
            #  genome loading
            current_neuron_count = self.get_neuron_count()
            if current_neuron_count > 0:
                logger.info(
                    f"⚠️  [DYNAMIC SIZING] Skipping resize - {current_neuron_count:,} neurons already exist"
                )
                logger.info(
                    "ℹ️  [DYNAMIC SIZING] This prevents spatial hash corruption during test mode"
                )
                return False

            #  Check if resizing is beneficial (reduce by at least 50% or
            #  increase if needed)
            if (
                optimal_neuron_size < self.max_neurons * 0.5
                or optimal_neuron_size > self.max_neurons
            ):
                logger.info(
                    f"🔄 [DYNAMIC SIZING] Resizing connectome from {self.max_neurons:,} to {optimal_neuron_size:,} neurons"
                )

                # Store old values for comparison
                old_neuron_capacity = self.max_neurons

                # Update capacity
                self.max_neurons = optimal_neuron_size
                self.max_synapses = optimal_synapse_size

                # ARCHITECTURE: Neurons and synapses managed by Rust NPU (no Python arrays)
                # Rust NPU is reinitialized via NPU Interface, not here
                # Note: No need to clear neuron_id_to_index - Rust NPU handles this internally

                logger.info(
                    "✅ [DYNAMIC SIZING] Connectome resized successfully!"
                )
                logger.info(
                    f"   Memory savings: {(old_neuron_capacity - optimal_neuron_size) / old_neuron_capacity * 100:.1f}%"
                )
                logger.info(
                    f"   New matrix size: {optimal_neuron_size:,} x {optimal_neuron_size:,}"
                )

                return True
            else:
                logger.info(
                    f"ℹ️  [DYNAMIC SIZING] Current size {self.max_neurons:,} is already optimal - no resizing needed"
                )
                return False

        except Exception as e:
            logger.error(f"❌ [DYNAMIC SIZING] Error resizing connectome: {e}")
            return False

    def _calculate_neuron_space(self, config) -> int:
        """Calculate optimal neuron space based on genome stats and
        configuration.

        Logic:
        1. Read genome stats to get actual neuron requirements
        2. Apply buffer multiplier (default 50% more)
        3. Ensure minimum space from configuration
        4. Return the larger of buffered requirement vs minimum

        Args:
            config: FeagiConfig object with genome and connectome settings

        Returns:
            Optimal neuron space size
        """
        # Get configuration values
        min_neuron_space = config.get("connectome.min_neuron_space", 100_000)
        buffer_multiplier = config.get("connectome.buffer_multiplier", 1.5)

        # Try to get genome stats
        genome_neuron_count = self._get_genome_neuron_count(config)

        if genome_neuron_count > 0:
            # Calculate buffered requirement
            buffered_requirement = int(genome_neuron_count * buffer_multiplier)
            # Return the larger of buffered requirement vs minimum
            optimal_size = max(buffered_requirement, min_neuron_space)

            logger.info(
                f"🧠 [DYNAMIC SIZING] Genome neurons: {genome_neuron_count:,}"
            )
            logger.info(
                f"🧠 [DYNAMIC SIZING] Buffered requirement: {buffered_requirement:,} (x{buffer_multiplier})"
            )
            logger.info(
                f"🧠 [DYNAMIC SIZING] Minimum configured: {min_neuron_space:,}"
            )
            logger.info(
                f"🧠 [DYNAMIC SIZING] Optimal neuron space: {optimal_size:,}"
            )

            return optimal_size
        else:
            # Fallback to minimum if no genome stats available
            logger.warning(
                f"⚠️  [DYNAMIC SIZING] No genome stats available, using minimum: {min_neuron_space:,}"
            )
            return min_neuron_space

    def _calculate_synapse_space(
        self, config, default_max_synapses: int
    ) -> int:
        """Calculate optimal synapse space based on genome stats and
        configuration.

        Args:
            config: FeagiConfig object with genome and connectome settings
            default_max_synapses: Default max synapses value

        Returns:
            Optimal synapse space size
        """
        # Get configuration values
        min_synapse_space = config.get("connectome.min_synapse_space", 500_000)
        buffer_multiplier = config.get("connectome.buffer_multiplier", 1.5)

        # Try to get genome stats
        genome_synapse_count = self._get_genome_synapse_count(config)

        if genome_synapse_count > 0:
            # Calculate buffered requirement
            buffered_requirement = int(
                genome_synapse_count * buffer_multiplier
            )
            # Return the larger of buffered requirement vs minimum
            optimal_size = max(buffered_requirement, min_synapse_space)

            logger.info(
                f"🔗 [DYNAMIC SIZING] Genome synapses: {genome_synapse_count:,}"
            )
            logger.info(
                f"🔗 [DYNAMIC SIZING] Buffered requirement: {buffered_requirement:,} (x{buffer_multiplier})"
            )
            logger.info(
                f"🔗 [DYNAMIC SIZING] Minimum configured: {min_synapse_space:,}"
            )
            logger.info(
                f"🔗 [DYNAMIC SIZING] Optimal synapse space: {optimal_size:,}"
            )

            return optimal_size
        else:
            # Fallback to minimum if no genome stats available
            logger.warning(
                f"⚠️  [DYNAMIC SIZING] No genome stats available, using minimum: {min_synapse_space:,}"
            )
            return min_synapse_space

    def _calculate_memory_neuron_space(self, config) -> int:
        """Calculate optimal memory neuron space based on configuration.

        Args:
            config: FeagiConfig object with connectome settings

        Returns:
            Optimal memory neuron space size
        """
        # Get configuration values
        min_memory_neuron_space = config.get(
            "connectome.min_memory_neuron_space", 50_000
        )

        logger.info(
            f"🧠 [MEMORY SIZING] Memory neuron space: {min_memory_neuron_space:,}"
        )

        return min_memory_neuron_space

    def _get_genome_neuron_count(self, config) -> int:
        """Extract neuron count from genome stats."""
        try:
            # Try to get from currently loaded genome
            genome_data = config.get("genome.data", {})
            if isinstance(genome_data, dict):
                stats = genome_data.get("stats", {})
                if isinstance(stats, dict):
                    return stats.get("innate_neuron_count", 0)

            # Try alternative paths
            neuron_count = config.get("genome.stats.innate_neuron_count", 0)
            if neuron_count > 0:
                return neuron_count

            return 0
        except Exception as e:
            logger.warning(
                f"⚠️  [DYNAMIC SIZING] Error reading genome neuron count: {e}"
            )
            return 0

    def _get_genome_synapse_count(self, config) -> int:
        """Extract synapse count from genome stats."""
        try:
            # Try to get from currently loaded genome
            genome_data = config.get("genome.data", {})
            if isinstance(genome_data, dict):
                stats = genome_data.get("stats", {})
                if isinstance(stats, dict):
                    return stats.get("innate_synapse_count", 0)

            # Try alternative paths
            synapse_count = config.get("genome.stats.innate_synapse_count", 0)
            if synapse_count > 0:
                return synapse_count

            return 0
        except Exception as e:
            logger.warning(
                f"⚠️  [DYNAMIC SIZING] Error reading genome synapse count: {e}"
            )
            return 0

    # ======================================================================
    # PHASE 1 & 2: O(1) CORTICAL ID/IDX CONVERSION METHODS
    # ======================================================================

    def get_cortical_idx_for_id(self, cortical_id: str) -> Optional[int]:
        """Get cortical_idx from cortical_id.

        This method is the ONLY way to access the BiDirectionalCorticalMap.
        It provides a clean interface for ID to index mapping.

        Args:
            cortical_id: String identifier for cortical area

        Returns:
            Integer cortical_idx if found, None otherwise
        """
        # CRITICAL FIX: Strip literal quotes that may be embedded in cortical_id
        # This handles cases where cortical_id comes in as "'iic400'" instead of "iic400"
        cleaned_cortical_id = cortical_id.strip("'\"") if cortical_id else cortical_id
        
        result = self.cortical_mapping.get_idx(cleaned_cortical_id)
        if result is None:
            # Check if cortical mapping is empty and try to rebuild it
            try:
                all_mappings = self.cortical_mapping.get_all_mappings() if hasattr(self.cortical_mapping, 'get_all_mappings') else {}
                all_ids = list(all_mappings.keys())
                
                # Fallback: Check if cleaned cortical_id exists in get_all_mappings
                if cleaned_cortical_id in all_mappings:
                    expected_idx = all_mappings[cleaned_cortical_id]
                    logger.info(f"[CORTICAL-MAP] SUCCESS: After cleaning quotes, {cleaned_cortical_id} found -> {expected_idx}")
                    # Return the result since we found it after cleaning
                    return expected_idx
                
                # Not found even after cleaning
                logger.error(f"[CORTICAL-MAP] {cortical_id} (cleaned: {cleaned_cortical_id}) not found. Available IDs: {all_ids[:10]}")
            except Exception as e:
                logger.error(f"[CORTICAL-MAP] Error during debug logging for {cortical_id}: {e}")
        return result

    def get_cortical_id_for_idx(self, cortical_idx: int) -> Optional[str]:
        """Get cortical_id from cortical_idx.

        This method is the ONLY way to access the BiDirectionalCorticalMap.
        It provides a clean interface for index to ID mapping.

        Args:
            cortical_idx: Integer index for cortical area

        Returns:
            String cortical_id if found, None otherwise
        """
        return self.cortical_mapping.get_id(cortical_idx)

    # ======================================================================
    # OPERATIONAL HELPERS: STRICT IPU/OPU CLASSIFICATION
    # ======================================================================
    def is_opu(self, cortical_id_or_idx) -> bool:
        """Return True iff the area's cortical_group is exactly 'OPU'.

        Args:
            cortical_id_or_idx: Cortical area ID (str) or index (int)
        """
        try:
            if isinstance(cortical_id_or_idx, int):
                cid = self.get_cortical_id_for_idx(int(cortical_id_or_idx))
            else:
                cid = str(cortical_id_or_idx)
            if not cid:
                return False
            info = self.get_cortical_area_properties(cid) or {}
            group = str(info.get("cortical_group", "")).upper()
            return group == "OPU"
        except Exception:
            return False

    def is_ipu(self, cortical_id_or_idx) -> bool:
        """Return True iff the area's cortical_group is exactly 'IPU'.

        Args:
            cortical_id_or_idx: Cortical area ID (str) or index (int)
        """
        try:
            if isinstance(cortical_id_or_idx, int):
                cid = self.get_cortical_id_for_idx(int(cortical_id_or_idx))
            else:
                cid = str(cortical_id_or_idx)
            if not cid:
                return False
            info = self.get_cortical_area_properties(cid) or {}
            group = str(info.get("cortical_group", "")).upper()
            return group == "IPU"
        except Exception:
            return False

    def list_opu_areas(self) -> list:
        """Return a list of cortical IDs whose cortical_group is 'OPU'."""
        result = []
        try:
            for cid in list(self.cortical_areas.keys()):
                try:
                    if self.is_opu(cid):
                        result.append(cid)
                except Exception:
                    continue
        except Exception:
            pass
        return result

    def list_ipu_areas(self) -> list:
        """Return a list of cortical IDs whose cortical_group is 'IPU'."""
        result = []
        try:
            for cid in list(self.cortical_areas.keys()):
                try:
                    if self.is_ipu(cid):
                        result.append(cid)
                except Exception:
                    continue
        except Exception:
            pass
        return result

    def validate_cortical_mapping(self) -> bool:
        """Validate that cortical mapping is consistent with cortical_areas.

        Returns:
            True if consistent, False otherwise
        """
        try:
            is_consistent, errors = (
                self.cortical_mapping.validate_consistency()
            )

            if not is_consistent:
                logger.error(
                    f"Cortical mapping inconsistency detected: {errors}"
                )
                return False

            # Validate against cortical_areas using vectorized operations
            cortical_ids = list(self.cortical_areas.keys())
            if cortical_ids:  # Only validate if we have cortical areas
                area_indices = np.array(
                    [
                        self.cortical_areas[cid].cortical_idx
                        for cid in cortical_ids
                    ]
                )
                mapped_indices = np.array(
                    [
                        self.cortical_mapping.get_idx(cid)
                        for cid in cortical_ids
                    ]
                )

                # Vectorized comparison - GPU/SIMD friendly
                mismatches = area_indices != mapped_indices
                if np.any(mismatches):
                    mismatch_positions = np.where(mismatches)[0]
                    mismatch_ids = [
                        cortical_ids[i] for i in mismatch_positions
                    ]
                    for i, cortical_id in enumerate(mismatch_ids):
                        pos = mismatch_positions[i]
                        logger.error(
                            f"Mapping mismatch for {cortical_id}: mapping={mapped_indices[pos]}, area={area_indices[pos]}"
                        )
                    return False

            return True

        except Exception as e:
            logger.error(f"Error validating cortical mapping: {e}")
            return False

    def _sync_cortical_mapping(
        self, cortical_id: str, cortical_idx: int
    ) -> None:
        """Synchronize cortical mapping between ID and index.

        This is an internal method that should only be called by ConnectomeManager.
        It ensures the BiDirectionalCorticalMap stays in sync with the connectome.

        Args:
            cortical_id: String identifier for cortical area
            cortical_idx: Integer index for cortical area
        """
        logger.info(f"[CORTICAL-MAP] _sync_cortical_mapping called: {cortical_id} -> {cortical_idx}")
        success = self.cortical_mapping.add_mapping(cortical_id, cortical_idx)
        if success:
            logger.info(f"[CORTICAL-MAP] Successfully added mapping: {cortical_id} -> {cortical_idx}")
        else:
            logger.error(f"[CORTICAL-MAP] FAILED to add mapping: {cortical_id} -> {cortical_idx}")
            # Verify what went wrong
            logger.error(f"[CORTICAL-MAP] Input validation: cortical_id='{cortical_id}' (len={len(cortical_id) if cortical_id else 0}), cortical_idx={cortical_idx}")
            if cortical_idx in (0, 1) and cortical_id not in ("_death", "_power"):
                logger.error(f"[CORTICAL-MAP] Rejected: Trying to use reserved cortical_idx {cortical_idx} for non-core area {cortical_id}")
        
        # Verify the mapping was actually added
        verification = self.cortical_mapping.get_idx(cortical_id)
        if verification != cortical_idx:
            logger.error(f"[CORTICAL-MAP] VERIFICATION FAILED: get_idx({cortical_id}) returned {verification}, expected {cortical_idx}")
        else:
            logger.info(f"[CORTICAL-MAP] Verification passed: get_idx({cortical_id}) = {cortical_idx}")

    def _remove_cortical_mapping(self, cortical_id: str) -> None:
        """Remove cortical mapping for an area.

        This is an internal method that should only be called by ConnectomeManager.
        It ensures the BiDirectionalCorticalMap stays in sync with the connectome.

        Args:
            cortical_id: String identifier for cortical area to remove
        """
        self.cortical_mapping.remove_by_id(cortical_id)

    def _find_next_available_cortical_idx(self) -> int:
        """Dynamically find the next available cortical_idx using proper state
        management.

        This method scans existing cortical areas to find the next available index,
        respecting reserved cortical_idx constraints (0 for _death, 1 for _power).

        Returns:
            Next available cortical_idx starting from 2 (after reserved indices)
        """
        try:
            # Collect all currently used cortical_idx values
            used_indices = set()

            #  Add reserved indices (always reserved regardless of whether
            #  areas exist)
            reserved_indices = set(self.reserved_cortical_areas.values())
            used_indices.update(reserved_indices)

            # Scan existing cortical areas to find used indices
            if hasattr(self, "cortical_areas") and self.cortical_areas:
                for area in self.cortical_areas.values():
                    if (
                        hasattr(area, "cortical_idx")
                        and area.cortical_idx is not None
                    ):
                        used_indices.add(area.cortical_idx)

            #  Find the next available index starting from the minimum allowed
            #  (2)
            min_non_reserved_idx = (
                max(reserved_indices) + 1 if reserved_indices else 0
            )
            next_idx = min_non_reserved_idx

            while next_idx in used_indices:
                next_idx += 1

            from feagi.core.state_manager import get_state_manager

            state_manager = get_state_manager()
            if state_manager.is_debug_bdu_enabled():
                logger.debug(
                    f"[BDU-DEBUG] cortical_idx allocation: reserved={reserved_indices}, "
                    f"used={sorted(used_indices)}, next_available={next_idx}"
                )

            return next_idx

        except Exception as e:
            # Fallback to safe default if state scanning fails
            logger.warning(
                f"Failed to scan cortical_idx state, using safe fallback: {e}"
            )
            return (
                max(self.reserved_cortical_areas.values()) + 1
                if self.reserved_cortical_areas
                else 2
            )

    def rebuild_cortical_mapping_from_existing_areas(self) -> bool:
        """Retroactively synchronize BiDirectionalCorticalMap from existing
        cortical areas.

        This fixes systems that were loaded from disk before BiDirectionalCorticalMap
        was implemented, ensuring mapping consistency.

        Returns:
            True if rebuild was successful, False otherwise
        """
        logger.info(
            "🔧 RETROACTIVE MAPPING: Rebuilding BiDirectionalCorticalMap from existing cortical areas"
        )

        if not hasattr(self, "cortical_areas") or not self.cortical_areas:
            logger.warning(
                "❌ RETROACTIVE MAPPING: No cortical areas found to rebuild mapping from"
            )
            return False

        rebuild_count = 0
        failed_count = 0

        for cortical_id, area_obj in self.cortical_areas.items():
            try:
                # Get cortical_idx from the area object
                cortical_idx = getattr(area_obj, "cortical_idx", None)

                if cortical_idx is None:
                    # Area doesn't have cortical_idx assigned - assign one
                    if cortical_id in self.reserved_cortical_areas:
                        cortical_idx = self.reserved_cortical_areas[
                            cortical_id
                        ]
                        logger.info(
                            f"🔧 RETROACTIVE MAPPING: Assigning reserved cortical_idx={cortical_idx} to core area '{cortical_id}'"
                        )
                    else:
                        cortical_idx = self._find_next_available_cortical_idx()
                        logger.info(
                            f"🔧 RETROACTIVE MAPPING: Assigning new cortical_idx={cortical_idx} to area '{cortical_id}' (dynamically allocated)"
                        )

                    # Update the area object
                    area_obj.cortical_idx = cortical_idx

                # Synchronize the mapping
                success = self.cortical_mapping.add_mapping(
                    cortical_id, cortical_idx
                )
                if success:
                    rebuild_count += 1
                    logger.info(
                        f"✅ RETROACTIVE MAPPING: Synchronized {cortical_id} ↔ {cortical_idx}"
                    )
                else:
                    failed_count += 1
                    logger.error(
                        f"❌ RETROACTIVE MAPPING: Failed to sync {cortical_id} ↔ {cortical_idx}"
                    )

            except Exception as e:
                failed_count += 1
                logger.error(
                    f"❌ RETROACTIVE MAPPING: Exception syncing {cortical_id}: {e}"
                )

        # Validate the mapping
        is_consistent, errors = self.cortical_mapping.validate_consistency()
        if not is_consistent:
            logger.error(
                f"❌ RETROACTIVE MAPPING: Validation failed: {errors}"
            )
            return False

        logger.info(
            f"✅ RETROACTIVE MAPPING: Successfully rebuilt {rebuild_count} mappings, {failed_count} failed"
        )
        return failed_count == 0

    @property
    def _npu_interface(self):
        """Property to track access to NPU interface."""
        return self._npu_interface_internal
    
    @_npu_interface.setter
    def _npu_interface(self, value):
        """Property to track setting of NPU interface."""
        self._npu_interface_internal = value

    def _initialize_npu_interface(self, backend: str):
        """Initialize NPU Interface with the specified backend.

        Args:
            backend: Backend string ("cpu", "cuda", "wgpu", etc.) or None
        """
        from feagi.npu.interface import NPUInterface
        from feagi.npu.data_structures import BackendType
        
        # Handle None backend (use CPU as default)
        if backend is None:
            backend = "cpu"
            logger.info("🔧 Backend was None, defaulting to 'cpu'")
        
        # Create NPU Interface as single source of truth for neural data
        backend_map = {
            "cpu": BackendType.CPU,
            "cuda": BackendType.CUDA, 
            "wgpu": BackendType.WGPU,
            "auto": BackendType.CPU  # Map 'auto' to CPU for now
        }
        npu_backend = backend_map.get(backend, BackendType.CPU)
        
        logger.info(f"🔧 Creating NPU Interface with Rust NPU (capacity: {self._max_neurons_config:,} neurons, {self._max_synapses_config:,} synapses)")
        
        # CRITICAL: Pass calculated capacities to NPUInterface
        # NPUInterface will immediately create Rust NPU with these capacities
        self._npu_interface = NPUInterface(
            backend=npu_backend,
            max_neurons=self._max_neurons_config,
            max_synapses=self._max_synapses_config
        )
        self._npu_interface.set_connectome_manager(self)
        
        # ✅ Rust NPU is ALREADY CREATED with proper capacity
        # neuron_array and synapse_array live ONLY in Rust memory (SoA, GPU-friendly)
        # Neuroembryogenesis can now directly fill the pre-allocated Rust arrays
        
        logger.info("✅ NPU Interface initialized with Rust NPU")
        logger.info(f"   Backend: {npu_backend.value}")
        logger.info(f"   Neuron capacity: {self._max_neurons_config:,}")
        logger.info(f"   Synapse capacity: {self._max_synapses_config:,}")
    
    def set_npu_interface(self, npu_interface):
        """Set NPU interface as primary owner of synaptic updates.
        
        This method is now mainly for backward compatibility since NPU Interface
        is initialized during ConnectomeManager creation.

        Args:
            npu_interface: NPUInterface instance that will own synaptic updates
        """
        try:
            # Always adopt the provided NPU interface to ensure single source of truth
            self._npu_interface = npu_interface

            logger.info("✅ NPU interface set - ConnectomeManager uses clean API delegation")
        except Exception as e:
            logger.error(f"Failed to set NPU interface on ConnectomeManager: {e}")

    def sync_cortical_areas_to_npu(self) -> None:
        """Synchronize cortical area registry into the active NPU interface.

        Deterministic, no fallbacks. Copies minimal metadata required by NPU hot paths:
        - cortical_idx (int)
        - dimensions (tuple[int,int,int])
        - area_type: "regular" | "memory"
        - cortical_id (6-char string) for rare lookups (e.g., special areas)
        """
        if not hasattr(self, "_npu_interface") or self._npu_interface is None:
            raise RuntimeError("NPU Interface not available for cortical area sync")

        # Guard against missing cortical_areas structure
        areas = getattr(self, "cortical_areas", None)
        if not areas:
            return

        from feagi.npu.interface import OperationResult

        try:
            from feagi.core.state_manager import FeagiStateManager
            debug_enabled = FeagiStateManager.instance().is_debug_npu_enabled()
        except Exception:
            debug_enabled = False

        logger.info(
            f"[NPU-SYNC] Starting cortical area sync: npu_id={id(self._npu_interface)}, areas={len(areas)}"
        )

        synced_new = 0
        updated_existing = 0
        for cortical_id, area in areas.items():
            try:
                cortical_idx = getattr(area, "cortical_idx")
                dimensions = getattr(area, "dimensions")
                area_type = (
                    "memory" if getattr(area, "area_type", "regular") == "memory" else "regular"
                )

                result = self._npu_interface.create_cortical_area(
                    cortical_idx=cortical_idx,
                    dimensions=dimensions,
                    area_type=area_type,
                    cortical_id=cortical_id,
                )
                # Treat INVALID_INPUT as already-registered and UPSET metadata
                if result == OperationResult.SUCCESS:
                    synced_new += 1
                    if debug_enabled:
                        logger.info(
                            f"[NPU-SYNC] Registered area id='{cortical_id}' idx={cortical_idx} dims={dimensions} type={area_type}"
                        )
                elif result == OperationResult.INVALID_INPUT:
                    # @ruff-skip: critical hotfix - cleanup task: NPU-SYNC-UPTS-001
                    # Area exists; ensure cortical_id (and optionally dims/type) are set for lookups like '_power'
                    try:
                        if cortical_idx in self._npu_interface.cortical_areas:
                            entry = self._npu_interface.cortical_areas[cortical_idx]
                            changed = False
                            if entry.get("cortical_id") != cortical_id:
                                entry["cortical_id"] = cortical_id
                                changed = True
                            # Keep dims/type authoritative from NPU unless missing
                            if "dimensions" not in entry or entry["dimensions"] is None:
                                entry["dimensions"] = dimensions
                                changed = True
                            if "type" not in entry or entry["type"] not in ("regular", "memory"):
                                entry["type"] = area_type
                                changed = True
                            if changed:
                                updated_existing += 1
                                if debug_enabled:
                                    logger.info(
                                        f"[NPU-SYNC] Updated existing area metadata id='{cortical_id}' idx={cortical_idx}"
                                    )
                        else:
                            # Inconsistent state: INVALID_INPUT but not present; log for investigation
                            logger.warning(
                                f"[NPU-SYNC] INVALID_INPUT for idx={cortical_idx} but not found in NPU registry"
                            )
                    except Exception as inner_e:
                        logger.error(f"[NPU-SYNC] Error updating existing area '{cortical_id}': {inner_e}")
            except Exception as e:
                logger.error(f"Failed to sync cortical area '{cortical_id}': {e}")
                raise

        logger.info(
            f"[NPU-SYNC] Synchronized new={synced_new}, updated={updated_existing}, total_areas={len(areas)} into NPU registry"
        )

    def update_membrane_potentials(
        self, decay_factor=None, current_timestep=None
    ) -> List[int]:
        """Update membrane potentials using embedded optimizations.

        This method now uses the ultra-high-performance embedded-optimized neural update
        from the NeuronArray, providing:
        - SIMD-vectorized operations
        - Cache-aligned memory access
        - Block-sparse connectivity optimization
        - Zero-allocation operation paths

        Designed for 10M neurons at 15Hz on single-core embedded systems.

        Args:
            decay_factor: Optional decay factor (for backward compatibility)
            current_timestep: Current simulation timestep (optional)

        Returns:
            List of neuron IDs that fired
        """
        # Debug-only block gated by state manager
        try:
            from feagi.core.state_manager import FeagiStateManager
            if FeagiStateManager.instance().is_debug_npu_enabled():
                logger.debug("[CONNECTOME-DEBUG] === UPDATE_MEMBRANE_POTENTIALS CALLED ===")
                logger.debug(f"[CONNECTOME-DEBUG] decay_factor: {decay_factor}")
                logger.debug(f"[CONNECTOME-DEBUG] current_timestep: {current_timestep}")
                logger.debug(f"[CONNECTOME-DEBUG] Has NPU interface: {hasattr(self, '_npu_interface') and self._npu_interface is not None}")
        except Exception:
            pass
        # NO BACKWARD COMPATIBILITY - NPU has 100% exclusive ownership
        if decay_factor is not None and isinstance(decay_factor, (int, float)):
            raise RuntimeError(
                "Legacy BDU neural processing is prohibited. "
                "NPU has 100% exclusive ownership of neural processing. "
                "Update tests to use NPU-based neural processing."
            )

        # Set current timestep
        if current_timestep is not None:
            self.current_timestep = current_timestep

        # CRITICAL: NPU has 100% exclusive ownership of neural processing
        # NO FALLBACKS - NPU is REQUIRED for neural processing
        if not hasattr(self, '_npu_interface') or not self._npu_interface:
            raise RuntimeError(
                "NPU interface required - NPU has 100% exclusive ownership of neural processing. "
                "BDU only handles synaptogenesis and synaptic pruning. "
                "Configure NPU interface before neural processing."
            )
        
        try:
            from feagi.core.state_manager import FeagiStateManager
            if FeagiStateManager.instance().is_debug_npu_enabled():
                logger.debug("[CONNECTOME-DEBUG] === DELEGATING TO NPU ===")
                logger.debug(f"[CONNECTOME-DEBUG] Current timestep: {self.current_timestep}")
                logger.debug(f"[CONNECTOME-DEBUG] NPU interface available: {self._npu_interface is not None}")
        except Exception:
            pass

        # NPU has 100% exclusive ownership of neural processing AND synaptic propagation
        result = self._npu_interface.process_neural_burst(self.current_timestep)
        try:
            from feagi.core.state_manager import FeagiStateManager
            if FeagiStateManager.instance().is_debug_npu_enabled():
                logger.debug(f"[CONNECTOME-DEBUG] NPU returned fired neurons: {result}")
        except Exception:
            pass
        return result

    # ----------------------------------------------------------------------
    # Synapse Storage Methods
    # ----------------------------------------------------------------------

    # Legacy sparse matrix methods removed - replaced with NPU SynapseArray

    # ----------------------------------------------------------------------
    # Neuron CRUD Operations
    # ----------------------------------------------------------------------

    def create_neuron(
        self,
        cortical_id: str,
        position: Tuple[int, int, int],
        threshold: float = 1.0,
        membrane_potential: float = 0.0,
        resting_potential: float = 0.0,
        leak_coefficient: float = 0.0,
        refractory_period: int = 1,
        properties: Optional[Dict[str, Any]] = None,
        cortical_idx: Optional[int] = None,
    ) -> int:
        """Create a new neuron in the specified cortical area.

        Args:
            cortical_id: ID of the cortical area
            position: 3D coordinates within the cortical area (x, y, z)
            threshold: Firing threshold potential
            membrane_potential: Initial membrane potential
            resting_potential: Base membrane potential
            leak_coefficient: Leak coefficient (0.0-1.0, percentage of potential lost per burst)
            refractory_period: Number of timesteps after firing during which the neuron cannot fire
            properties: Additional properties for the neuron (optional)
            cortical_idx: Integer index of the cortical area (optional, will be determined from cortical_id if not provided)

        Returns:
            Unique ID of the created neuron

        Raises:
            ValueError: If the cortical_id doesn't exist
            ValueError: If the position is outside the area's boundaries
        """
        # Validate the cortical area exists
        if cortical_id not in self.cortical_areas:
            raise ValueError(f"Cortical area '{cortical_id}' does not exist")

        area = self.cortical_areas[cortical_id]

        # Validate position
        if not area.contains_position(position):
            raise ValueError(
                f"Position {position} is outside the bounds of area {area.name}"
            )

        # Use the provided cortical_idx or get it from the area
        if cortical_idx is None:
            cortical_idx = area.cortical_idx

        # CRITICAL: Use NPU Interface CRUD methods
        if not self._npu_interface:
            raise RuntimeError("NPU interface not configured - cannot create neurons")

        return self._create_neuron_via_npu(
            cortical_idx=cortical_idx,
            position=position,
            threshold=threshold,
            membrane_potential=membrane_potential,
            resting_potential=resting_potential,
            leak_coefficient=leak_coefficient,
            refractory_period=refractory_period,
            properties=properties,
        )

    def get_neuron(self, neuron_id: int) -> Dict[str, Any]:
        """Get information about a specific neuron.

        Args:
            neuron_id: ID of the neuron

        Returns:
            Dictionary with neuron properties

        Raises:
            KeyError: If the neuron_id doesn't exist
        """
        # ✅ RUST NPU: Use NPUInterface API (single source of truth)
        npu = getattr(self, '_npu_interface', None)
        if npu is None:
            raise RuntimeError("NPU Interface not available")
        
        if not npu.neuron_exists(neuron_id):
            raise KeyError(f"Neuron {neuron_id} does not exist")

        # Get position from NPUInterface
        position = npu.get_neuron_position(neuron_id)
        if position is None:
            raise KeyError(f"Neuron {neuron_id} position not found")

        # Get cortical_idx from NPUInterface
        cortical_idx = npu.get_neuron_cortical_idx(neuron_id)
        if cortical_idx is None:
            raise KeyError(f"Neuron {neuron_id} cortical_idx not found")

        # Find the corresponding cortical_id using mapping
        cortical_id = self.cortical_mapping.get_id(cortical_idx)
        if cortical_id is None:
            raise RuntimeError(
                f"CRITICAL: cortical_idx={cortical_idx} not found in mapping - system corruption detected"
            )

        result = {
            "cortical_id": cortical_id,  # String identifier (for backward compatibility)
            "cortical_idx": cortical_idx,  # Integer index (for internal use)
            "position": position,
            "threshold": 1.0,  # Static genome value
            "membrane_potential": 0.0,  # Live state not exposed via API
            "resting_potential": 0.0,  # Static genome value
            "leak_coefficient": 0.0,  # Static genome value
            "refractory_period": 0,  # Static genome value
            "refractory_counter": 0,  # Live state not exposed via API
            "properties": {},  # We don't store additional properties in the optimized version
        }

        return result

    def get_neuron_properties(self, neuron_id: int) -> Dict[str, Any]:
        """Get all properties of a specific neuron including synaptic
        connections.

        Args:
            neuron_id: ID of the neuron

        Returns:
            Dictionary with all neuron properties including incoming and outgoing synapses

        Raises:
            KeyError: If the neuron_id doesn't exist
        """
        # Get basic neuron properties
        properties = self.get_neuron(neuron_id)

        # Add synaptic connection information
        try:
            # Get outgoing synapses (this neuron -> other neurons)
            outgoing_synapses = self.get_outgoing_connections(neuron_id)
            properties["outgoing_synapses"] = [
                {"target_neuron_id": target_id, "weight": float(weight)}
                for target_id, weight in outgoing_synapses
            ]

            # Get incoming synapses (other neurons -> this neuron)
            incoming_synapses = self.get_incoming_connections(neuron_id)
            properties["incoming_synapses"] = [
                {"source_neuron_id": source_id, "weight": float(weight)}
                for source_id, weight in incoming_synapses
            ]

            # Add synapse counts for quick reference
            properties["synapse_counts"] = {
                "outgoing": len(outgoing_synapses),
                "incoming": len(incoming_synapses),
                "total": len(outgoing_synapses) + len(incoming_synapses),
            }

        except Exception as e:
            #  If synapse lookup fails, still return basic properties but log
            #  the error
            self.logger.warning(
                f"Failed to retrieve synapse information for neuron {neuron_id}: {e}"
            )
            properties["outgoing_synapses"] = []
            properties["incoming_synapses"] = []
            properties["synapse_counts"] = {
                "outgoing": 0,
                "incoming": 0,
                "total": 0,
            }

        return properties

    def get_neuron_property(
        self, neuron_id: int, property_name: Union[str, NeuronPropertyType]
    ) -> Any:
        """Get a specific property of a neuron.

        Args:
            neuron_id: ID of the neuron
            property_name: Name or enum of the property to get

        Returns:
            Value of the property

        Raises:
            KeyError: If the neuron_id doesn't exist
            KeyError: If the property doesn't exist
        """
        if not self._npu_interface.neuron_exists(neuron_id):
            raise KeyError(f"Neuron {neuron_id} does not exist")

        # Handle property name as either string or enum
        if isinstance(property_name, NeuronPropertyType):
            property_name = property_name.value

        # CRITICAL FIX: Read from NPU if it's the primary owner
        if hasattr(self, '_npu_processor') and self._npu_processor is not None:
            # NPU is primary owner - read from NPU arrays directly
            if neuron_id in self._npu_processor.neurons.neuron_id_to_index:
                idx = self._npu_processor.neurons.neuron_id_to_index[neuron_id]
                
                if property_name == "membrane_potential":
                    return float(self._npu_processor.neurons.membrane_potentials[idx])
                elif property_name == "threshold":
                    return float(self._npu_processor.neurons.thresholds[idx])
                elif property_name == "leak_coefficient":
                    return float(self._npu_processor.neurons.leak_coefficients[idx])
                elif property_name == "resting_potential":
                    return float(self._npu_processor.neurons.resting_potentials[idx])
                elif property_name == "refractory_period":
                    return int(self._npu_processor.neurons.refractory_periods[idx])
                elif property_name == "refractory_counter":
                    return int(self._npu_processor.neurons.refractory_counters[idx])
                else:
                    raise RuntimeError(f"Unknown neuron property '{property_name}' - NPU does not support this property")
            else:
                raise RuntimeError(f"Neuron {neuron_id} not found in NPU - BDU fallback prohibited")
        else:
            # NPU not configured - NO FALLBACKS
            raise RuntimeError(
                "NPU processor required - NPU has 100% exclusive ownership of neuron properties. "
                "Configure NPU processor before accessing neuron properties."
            )

    def set_neuron_property(
        self,
        neuron_id: int,
        property_name: Union[str, NeuronPropertyType],
        value: Any,
    ) -> None:
        """Set a specific property of a neuron.

        Args:
            neuron_id: ID of the neuron
            property_name: Name or enum of the property to set
            value: New value for the property

        Raises:
            KeyError: If the neuron_id doesn't exist
            KeyError: If the property doesn't exist
        """
        if not self._npu_interface.neuron_exists(neuron_id):
            raise KeyError(f"Neuron {neuron_id} does not exist")

        # Handle property name as either string or enum
        if isinstance(property_name, NeuronPropertyType):
            property_name = property_name.value

        # CRITICAL FIX: Synchronize with NPU if it's the primary owner
        if hasattr(self, '_npu_processor') and self._npu_processor is not None:
            # NPU is primary owner - update NPU arrays directly
            if neuron_id in self._npu_processor.neurons.neuron_id_to_index:
                idx = self._npu_processor.neurons.neuron_id_to_index[neuron_id]
                
                if property_name == "membrane_potential":
                    self._npu_processor.neurons.membrane_potentials[idx] = float(value)
                    logger.debug(f"[NPU-SYNC] Updated neuron {neuron_id} membrane_potential = {value}")
                elif property_name == "threshold":
                    self._npu_processor.neurons.thresholds[idx] = float(value)
                elif property_name == "leak_coefficient":
                    self._npu_processor.neurons.leak_coefficients[idx] = float(value)
                elif property_name == "resting_potential":
                    self._npu_processor.neurons.resting_potentials[idx] = float(value)
                elif property_name == "refractory_period":
                    self._npu_processor.neurons.refractory_periods[idx] = int(value)
                elif property_name == "refractory_counter":
                    self._npu_processor.neurons.refractory_counters[idx] = int(value)
                else:
                    logger.warning(f"Unknown property for NPU sync: {property_name}")
                
                # ✅ Rust NPU is the single source of truth - no backward compatibility layer needed
            else:
                logger.warning(f"Neuron {neuron_id} not found in NPU")
        else:
            # ✅ Rust NPU is required for neuron property updates
            logger.warning(f"NPU not configured - cannot update neuron property {property_name}")

    def get_neurons_by_cortical_area(self, cortical_id: str) -> List[int]:
        """Get all neurons in a specific cortical area using GPU/SIMD-optimized
        vectorized operations.

        VECTORIZED VERSION: Leverages Structure of Arrays (SoA) design for maximum performance.
        - Fully vectorized NumPy operations (GPU/SIMD friendly)
        - O(1) lookup using pre-built index-to-ID lookup array
        - No Python loops or dictionary iterations
        - MEMORY AREA SUPPORT: Also checks MemoryNeuronArray for memory areas

        Args:
            cortical_id: ID of the cortical area

        Returns:
            List of neuron IDs in the area (regular + memory neurons)

        Raises:
            KeyError: If the cortical_id doesn't exist
        """
        if cortical_id not in self.cortical_areas:
            raise KeyError(f"Cortical area {cortical_id} does not exist")

        # Get cortical_idx from cortical mapping
        cortical_idx = self.cortical_mapping.get_idx(cortical_id)
        if cortical_idx is None:
            return []

        # ✅ Use NPU interface clean API
        if self._npu_interface:
            return self._npu_interface.get_neurons_in_cortical_area(cortical_idx)
        else:
            raise RuntimeError("NPU interface not available")

    def get_cortical_area_for_neuron(self, neuron_id: int) -> str:
        """Get the ID of the cortical area containing a neuron using
        translation layer.

        Args:
            neuron_id: ID of the neuron

        Returns:
            ID of the cortical area containing the neuron

        Raises:
            KeyError: If the neuron_id doesn't exist
        """
        # ✅ Use NPU interface clean API
        if self._npu_interface:
            cortical_idx = self._npu_interface.get_neuron_cortical_idx(neuron_id)
            if cortical_idx is None:
                raise KeyError(f"Neuron {neuron_id} does not exist")
        else:
            raise RuntimeError("NPU interface not available")

        # Use cortical mapping to find cortical_id
        cortical_id = self.cortical_mapping.get_id(cortical_idx)
        if cortical_id is None:
            raise RuntimeError(
                f"CRITICAL: cortical_idx={cortical_idx} not found in mapping - system corruption detected"
            )

        return cortical_id

    # For backward compatibility, maintain the old method name
    def get_area_for_neuron(self, neuron_id: int) -> str:
        """Get the ID of the cortical area containing a neuron (backward
        compatibility).

        Args:
            neuron_id: ID of the neuron

        Returns:
            ID of the cortical area containing the neuron
        """
        return self.get_cortical_area_for_neuron(neuron_id)

    def get_neurons_by_area_list(self, cortical_id: str) -> List[int]:
        """Get all neuron IDs in a specific cortical area (list variant).

        Args:
            cortical_id: ID of the cortical area

        Returns:
            List of neuron IDs in the area
        """
        return self.get_neurons_by_cortical_area(cortical_id)

    def get_neurons_by_cortical_idx(self, cortical_idx: int) -> List[int]:
        """Get all neurons in a cortical area by cortical_idx (integer).

        Args:
            cortical_idx: Integer cortical index (0 for _death, 1 for _power, etc.)

        Returns:
            List of neuron IDs in the area

        Raises:
            KeyError: If no area with the specified cortical_idx exists
        """
        # Use O(1) translation layer to find cortical_id
        cortical_id = self.cortical_mapping.get_id(cortical_idx)
        if cortical_id is None:
            raise KeyError(
                f"No cortical area found with cortical_idx={cortical_idx}"
            )

        return self.get_neurons_by_cortical_area(cortical_id)

    def delete_neuron(self, neuron_id: int) -> None:
        """Delete a neuron and all its connections.

        Args:
            neuron_id: ID of the neuron to delete

        Raises:
            ValueError: If the neuron doesn't exist
        """
        # Check if neuron exists
        if not self._npu_interface.neuron_exists(neuron_id):
            raise ValueError(f"Neuron {neuron_id} does not exist")

        # Get the neuron's index (neuron_id == index in Rust NPU)
        neuron_index = neuron_id  # Identity mapping

        # Delete outgoing synapses (synapses where this neuron is pre-synaptic)
        outgoing_connections = self.get_outgoing_connections(neuron_id)
        for target_id, _ in outgoing_connections:
            self.remove_synapse(neuron_id, target_id)

        #  Delete incoming synapses (synapses where this neuron is
        #  post-synaptic)
        incoming_connections = self.get_incoming_connections(neuron_id)
        for source_id, _ in incoming_connections:
            self.remove_synapse(source_id, neuron_id)

        # ✅ Mark neuron as inactive in Rust NPU
        if self._npu_interface and self._npu_interface.rust_npu:
            self._npu_interface.rust_npu.delete_neuron(neuron_id)
        
        # Note: No need to remove from mappings - Rust NPU handles this internally

    def get_neuron_position(self, neuron_id: int) -> Tuple[int, int, int]:
        """Get the position of a neuron.

        Args:
            neuron_id: ID of the neuron

        Returns:
            3D coordinates of the neuron

        Raises:
            KeyError: If the neuron_id doesn't exist
        """
        # ✅ Use NPU interface clean API
        if self._npu_interface:
            position = self._npu_interface.get_neuron_position(neuron_id)
            if position is None:
                raise KeyError(f"Neuron {neuron_id} does not exist")
            return position
        else:
            raise RuntimeError("NPU interface not available")

    # ----------------------------------------------------------------------
    # Synapse Management Methods
    # ----------------------------------------------------------------------

    def create_synapse(
        self,
        pre_neuron_id: int,
        post_neuron_id: int,
        weight: float,
        is_plastic: bool = False,
        plasticity_coeff: float = 0.0,
        plasticity_decay: float = 0.0,
        **kwargs,
    ) -> bool:
        """Create a synapse between two neurons using high-performance
        NPU SynapseArray.

        Args:
            pre_neuron_id: ID of the presynaptic neuron
            post_neuron_id: ID of the postsynaptic neuron
            weight: Synaptic weight
            is_plastic: Whether the synapse is plastic
            plasticity_coeff: Coefficient for plasticity
            plasticity_decay: Decay rate for plasticity
            **kwargs: Additional synapse properties

        Returns:
            True if the synapse was created, False if it already existed

        Raises:
            KeyError: If either neuron doesn't exist
        """
        # Check both neurons exist (via Rust NPU)
        if not self.has_neuron(pre_neuron_id):
            raise KeyError(
                f"Pre-synaptic neuron {pre_neuron_id} does not exist"
            )
        if not self.has_neuron(post_neuron_id):
            raise KeyError(
                f"Post-synaptic neuron {post_neuron_id} does not exist"
            )

        # Use new NPU SynapseArray for O(1) synapse creation
        synapse_type_int = 3 if is_plastic else 0  # 3=PLASTIC, 0=EXCITATORY

        # Extract conductance from genome or use default
        conductance = kwargs.get('conductance', 1.0)  # Default from genome template postsynaptic_current
        delay = kwargs.get('delay', 1)  # Default 1 timestep delay
        
        # ARCHITECTURE: Use Rust NPU directly (no deprecated synapse_array)
        success = self._npu_interface.rust_npu.add_synapse(
            source=pre_neuron_id,
            target=post_neuron_id,
            weight=weight,
            synapse_type=synapse_type_int,
            delay=delay,
            conductance=conductance,
            plasticity_coeff=plasticity_coeff
        )
        
        # Update state manager with new synapse count (optimized - synapse count only)
        if success:
            self._update_synapse_count_only()
        
        return success

    def batch_create_synapses(
        self, synapse_specs: List[Tuple[int, int, float]]
    ) -> int:
        """Create multiple synapses using Rust NPU.

        Args:
            synapse_specs: List of tuples (pre_neuron_id, post_neuron_id, weight)

        Returns:
            Number of synapses successfully created
        """
        # ✅ RUST NPU: Use NPUInterface API (single source of truth)
        if not hasattr(self, "_npu_interface") or self._npu_interface is None:
            raise RuntimeError("NPU Interface is not initialized")

        if not synapse_specs:
            logger.warning("No synapse specifications provided")
            return 0

        # PERFORMANCE: Skip Python-side validation - let Rust NPU handle it
        # Previous code validated ~2M neurons for large mappings (2M FFI calls!)
        # Rust NPU can validate internally during synapse creation (much faster)
        # Invalid synapses will be reported in result.failed_indices
        
        # Convert to NPUInterface SynapseCreationRequest format
        from feagi.npu.interface import SynapseCreationRequest
        
        request = SynapseCreationRequest(
            source_neuron_ids=[spec[0] for spec in synapse_specs],
            target_neuron_ids=[spec[1] for spec in synapse_specs],
            weights=[int(spec[2]) for spec in synapse_specs],  # Convert to u8
        )
        
        # Call NPUInterface to create synapses in Rust NPU
        # Rust will validate neurons exist and report failures efficiently
        result = self._npu_interface.create_synapses_batch(request)
        created_count = result.successful_count
        
        # Log validation failures if any
        if result.failed_indices:
            failure_count = len(result.failed_indices)
            if failure_count > 10:
                logger.warning(f"Failed to create {failure_count} synapses (first 10 indices: {result.failed_indices[:10]})")
            else:
                logger.warning(f"Failed to create {failure_count} synapses at indices: {result.failed_indices}")
        
        # Update state manager with new synapse count
        if created_count > 0:
            self._update_synapse_count_only()
        
        return created_count

    def _update_synapse_count_only(self):
        """Update state manager with current synapse count only.
        
        Only synapse count affects GPU keep-alive eligibility, so we avoid
        the overhead of updating neuron count unnecessarily.
        """
        try:
            # ARCHITECTURE: Use Rust NPU directly (no deprecated synapse_array)
            current_synapse_count = self._npu_interface.rust_npu.get_synapse_count()
            
            # Only update synapse count - more efficient
            self.state_manager.update_synapse_count(current_synapse_count)
            
            # ✅ REMOVED: GPU keep-alive irrelevant with Rust NPU (no Python backend)
                
        except Exception as e:
            self.logger.warning(f"Failed to update synapse count in state manager: {e}")
    
    def _update_neuron_count_only(self):
        """Update state manager with current neuron count only.
        
        This is separate from synapse updates to avoid unnecessary overhead.
        """
        try:
            current_neuron_count = self.get_neuron_count()  # Use the safe method
            self.state_manager.update_neuron_count(current_neuron_count)
        except Exception as e:
            self.logger.warning(f"Failed to update neuron count in state manager: {e}")
    
    def _update_brain_statistics(self):
        """Update state manager with comprehensive brain statistics.
        
        This includes cortical area count, neuron count, and synapse count.
        Used when cortical areas are added/removed.
        """
        try:
            # Get current counts
            cortical_area_count = len(self.cortical_areas)
            neuron_count = self.get_neuron_count()  # Use the safe method
            # ARCHITECTURE: Use Rust NPU directly (no deprecated synapse_array)
            synapse_count = self._npu_interface.rust_npu.get_synapse_count() if self._npu_interface else 0
            
            # Calculate memory vs regular neuron counts
            memory_neuron_count = 0
            regular_neuron_count = 0
            
            if self._npu_interface:
                for area_id, area in self.cortical_areas.items():
                    try:
                        neurons = self._npu_interface.get_neurons_by_area(area.cortical_idx)
                        neuron_count_in_area = len(neurons)
                        
                        if area.area_type == "memory":
                            memory_neuron_count += neuron_count_in_area
                        else:
                            regular_neuron_count += neuron_count_in_area
                    except Exception as e:
                        self.logger.debug(f"Could not count neurons in area {area_id}: {e}")
            
            # Update state manager with comprehensive brain stats
            brain_stats = {
                "cortical_area_count": cortical_area_count,
                "neuron_count": neuron_count,
                "synapse_count": synapse_count,
                "memory_neuron_count": memory_neuron_count,
                "regular_neuron_count": regular_neuron_count,
            }
            
            logger.info(
                f"📊 Updating state manager: {neuron_count} neurons "
                f"({regular_neuron_count} regular, {memory_neuron_count} memory), "
                f"{synapse_count} synapses, {cortical_area_count} areas"
            )
            
            result = self.state_manager.set_brain_stats(brain_stats)
            if result.is_err:
                self.logger.warning(f"Failed to set brain stats in state manager: {result.err}")
            else:
                self.logger.info(f"✅ State manager updated successfully: {brain_stats}")
                
            # Also update cortical list
            cortical_ids = list(self.cortical_areas.keys())
            cortical_result = self.state_manager.set_cortical_list(cortical_ids)
            if cortical_result.is_err:
                self.logger.warning(f"Failed to set cortical list in state manager: {cortical_result.err}")
            else:
                self.logger.debug(f"Updated cortical list: {len(cortical_ids)} areas")
                
        except Exception as e:
            self.logger.warning(f"Failed to update brain statistics in state manager: {e}")
            import traceback
            self.logger.warning(f"Traceback: {traceback.format_exc()}")

    #  Legacy optimized method removed - NPU SynapseArray is inherently
    #  optimized

    def remove_synapse(self, pre_neuron_id: int, post_neuron_id: int) -> bool:
        """Remove a synapse between two neurons using NPU SynapseArray.

        Args:
            pre_neuron_id: ID of the presynaptic neuron
            post_neuron_id: ID of the postsynaptic neuron

        Returns:
            True if the synapse was removed, False if it didn't exist

        Raises:
            KeyError: If either neuron doesn't exist
        """
        # Check both neurons exist using NPU-owned mapping
        if not self.has_neuron(pre_neuron_id):
            raise KeyError(
                f"Pre-synaptic neuron {pre_neuron_id} does not exist"
            )
        if not self.has_neuron(post_neuron_id):
            raise KeyError(
                f"Post-synaptic neuron {post_neuron_id} does not exist"
            )

        # ARCHITECTURE: Use Rust NPU directly (no deprecated synapse_array)
        success = self._npu_interface.rust_npu.remove_synapse(pre_neuron_id, post_neuron_id)
        
        # Update state manager with new synapse count (optimized - synapse count only)
        if success:
            self._update_synapse_count_only()
        
        return success

    def get_synapse_weight(
        self, pre_neuron_id: int, post_neuron_id: int
    ) -> float:
        """Get the weight of a synapse between two neurons using
        NPU SynapseArray.

        Args:
            pre_neuron_id: ID of the presynaptic neuron
            post_neuron_id: ID of the postsynaptic neuron

        Returns:
            Weight of the synapse, or 0.0 if it doesn't exist

        Raises:
            KeyError: If either neuron doesn't exist
        """
        # Check both neurons exist using NPU-owned mapping
        if not self.has_neuron(pre_neuron_id):
            raise KeyError(
                f"Pre-synaptic neuron {pre_neuron_id} does not exist"
            )
        if not self.has_neuron(post_neuron_id):
            raise KeyError(
                f"Post-synaptic neuron {post_neuron_id} does not exist"
            )

        # ARCHITECTURE: Use Rust NPU directly (no deprecated synapse_array)
        # Get all outgoing synapses from source and find the target
        outgoing = self._npu_interface.rust_npu.get_outgoing_synapses(pre_neuron_id)
        for target, weight, conductance, synapse_type in outgoing:
            if target == post_neuron_id:
                return weight
        return None  # Synapse not found

    def update_synapse_weight(
        self, pre_neuron_id: int, post_neuron_id: int, new_weight: float
    ) -> bool:
        """Update the weight of a synapse between two neurons using
        NPU SynapseArray.

        Args:
            pre_neuron_id: ID of the presynaptic neuron
            post_neuron_id: ID of the postsynaptic neuron
            new_weight: New weight for the synapse

        Returns:
            True if the synapse was updated, False if it didn't exist

        Raises:
            KeyError: If either neuron doesn't exist
        """
        # Check both neurons exist using NPU-owned mapping
        if not self.has_neuron(pre_neuron_id):
            raise KeyError(
                f"Pre-synaptic neuron {pre_neuron_id} does not exist"
            )
        if not self.has_neuron(post_neuron_id):
            raise KeyError(
                f"Post-synaptic neuron {post_neuron_id} does not exist"
            )

        # ARCHITECTURE: Use Rust NPU directly (no deprecated synapse_array)
        # Note: Rust expects u8 weight (0-255)
        weight_u8 = int(max(0, min(255, new_weight)))
        return self._npu_interface.rust_npu.update_synapse_weight(
            pre_neuron_id, post_neuron_id, weight_u8
        )

    def get_outgoing_connections(
        self, neuron_id: int
    ) -> List[Tuple[int, float]]:
        """Get all outgoing connections from a neuron using NPU SynapseArray.

        Args:
            neuron_id: ID of the neuron

        Returns:
            List of (target_neuron_id, weight) tuples

        Raises:
            KeyError: If the neuron doesn't exist
        """
        # Validate existence using NPU mapping
        if not self.has_neuron(neuron_id):
            raise KeyError(f"Neuron {neuron_id} does not exist")

        # ARCHITECTURE: Use Rust NPU directly (no deprecated synapse_array)
        outgoing = self._npu_interface.rust_npu.get_outgoing_synapses(neuron_id)
        # Convert from (target, weight, conductance, synapse_type) to (target, weight)
        return [(target, float(weight)) for target, weight, _, _ in outgoing]

    def get_incoming_connections(
        self, neuron_id: int
    ) -> List[Tuple[int, float]]:
        """Get all incoming connections to a neuron using NPU SynapseArray.

        Args:
            neuron_id: ID of the neuron

        Returns:
            List of (source_neuron_id, weight) tuples

        Raises:
            KeyError: If the neuron doesn't exist
        """
        if not self._npu_interface.neuron_exists(neuron_id):
            raise KeyError(f"Neuron {neuron_id} does not exist")

        # ARCHITECTURE: Use Rust NPU directly (no deprecated synapse_array)
        incoming = self._npu_interface.rust_npu.get_incoming_synapses(neuron_id)
        # Convert from (source, weight, conductance, synapse_type) to (source, weight)
        return [(source, float(weight)) for source, weight, _, _ in incoming]

    def get_synapse_count(self) -> int:
        """Get the total number of synapses in the connectome using
        NPU SynapseArray.

        Returns:
            Number of synapses
        """
        # ARCHITECTURE: Use Rust NPU directly (no deprecated synapse_array)
        if hasattr(self, "_npu_interface") and self._npu_interface:
            return self._npu_interface.rust_npu.get_synapse_count()
        return 0

    def _update_without_firing(self) -> List[int]:
        """Update membrane potentials when no neurons are firing.

        This method:
        1. Decays membrane potentials
        2. Decrements refractory counters
        3. Checks for neurons that now exceed their threshold

        Returns:
            List of neuron IDs that fired
        """
        # Move to the next FCL window
        fcl_manager = self._get_fcl_manager()
        if fcl_manager:
            fcl_manager.advance_timestep()

        # ✅ REMOVED: Old Python firing logic - Rust NPU handles all neural dynamics
        # The Rust NPU's process_burst() handles:
        #   - Membrane potential decay/leak
        #   - Threshold checking
        #   - Firing decisions
        #   - FCL management

        #  Convert fired indices to neuron IDs - CRITICAL FIX: Use correct
        #  vectorized method
        if fired_indices:
            fired_neuron_ids = self._vectorized_index_to_neuron_id(
                np.array(fired_indices)
            )
            # Convert to Python list of integers for FCL compatibility
            fired_neuron_ids = fired_neuron_ids.tolist()
        else:
            fired_neuron_ids = []

        # Increment timestep
        self.current_timestep += 1

        return fired_neuron_ids

    # More methods would follow for cortical areas, brain regions, etc.
    #  For this initial implementation, we're focusing on neuron and synapse
    #  management

    # ----------------------------------------------------------------------
    # Cortical Area Management Methods
    # ----------------------------------------------------------------------

    def add_cortical_area(
        self,
        name: str,
        dimensions: Tuple[int, int, int],
        position: Tuple[int, int, int],
        area_type: str = "custom",
        properties: Optional[Dict[str, Any]] = None,
        cortical_id: Optional[str] = None,
    ) -> str:
        """Add a new cortical area to the connectome.

        Args:
            name: Human-readable name for this area
            dimensions: 3D dimensions of the area (width, height, depth)
            position: 3D coordinates of the area's origin in the brain space
            area_type: Type of cortical area (e.g., "sensory", "motor", "custom")
            properties: Additional properties for the area (optional)
            cortical_id: Unique identifier for this area (optional, generated if not provided)

        Returns:
            Unique ID of the created cortical area

        Raises:
            ValueError: If an area with the same name already exists or dimensions exceed Morton limits
        """
        logger.info(f"[CORTICAL-MAP] add_cortical_area called: name='{name}', cortical_id='{cortical_id}', dimensions={dimensions}, position={position}")
        
        # Validate cortical area dimensions against Morton spatial hash limits
        from feagi.core.state_manager import get_state_manager

        state_manager = get_state_manager()

        validation_result = state_manager.validate_cortical_area_dimensions(
            dimensions
        )
        if validation_result.is_err:
            morton_limit = state_manager.get_morton_coordinate_limit()
            morton_class = state_manager.get_morton_class_name()
            raise ValueError(
                f"Cortical area dimensions {dimensions} exceed {morton_class} coordinate limits. "
                f"Maximum allowed per dimension: {morton_limit}. "
                f"Consider using smaller cortical areas or upgrading to 64-bit Morton encoding."
            )

        # Check if an area with this name already exists
        for area in self.cortical_areas.values():
            if area.name == name:
                raise ValueError(
                    f"Cortical area with name '{name}' already exists"
                )

        # CRITICAL FIX: Check if an area with this cortical_id already exists
        #  This prevents duplicate creation of core areas (_power, _death)
        #  which would
        #  cause multiple areas to share the same cortical_idx, leading to
        #  neuron corruption
        if cortical_id and cortical_id in self.cortical_areas:
            logger.info(
                f"Cortical area '{cortical_id}' already exists, returning existing area ID"
            )
            return cortical_id

        # Check for reserved core areas and assign appropriate cortical_idx
        if cortical_id in self.reserved_cortical_areas:
            #  This is a core area (_power or _death) - use reserved
            #  cortical_idx
            cortical_idx = self.reserved_cortical_areas[cortical_id]
            logger.info(
                f"Assigning reserved cortical_idx={cortical_idx} to core area '{cortical_id}'"
            )
        else:
            # Regular area - find next available cortical_idx dynamically
            cortical_idx = self._find_next_available_cortical_idx()
            logger.debug(
                f"Assigned cortical_idx={cortical_idx} to area '{cortical_id}' (dynamically allocated)"
            )

        #  CRITICAL COLLISION DETECTION: Ensure no two areas can share the same
        #  cortical_idx
        #  This is essential for one-to-one mapping between cortical_id and
        #  cortical_idx
        existing_area_with_same_idx = None
        for existing_cortical_id, existing_area in self.cortical_areas.items():
            if (
                hasattr(existing_area, "cortical_idx")
                and existing_area.cortical_idx == cortical_idx
            ):
                existing_area_with_same_idx = existing_cortical_id
                break

        if existing_area_with_same_idx:
            error_msg = (
                f"🚨 CRITICAL cortical_idx COLLISION DETECTED!\n"
                f"   Attempted: cortical_id='{cortical_id}' → cortical_idx={cortical_idx}\n"
                f"   Existing: cortical_id='{existing_area_with_same_idx}' → cortical_idx={cortical_idx}\n"
                f"   This would corrupt neuron assignments and break the one-to-one mapping!\n"
                f"   Reserved areas: {self.reserved_cortical_areas}\n"
                f"   next_cortical_idx: {self.next_cortical_idx}"
            )
            logger.error(error_msg, status="[CRITICAL]")
            raise ValueError(
                f"cortical_idx collision: cortical_idx={cortical_idx} already assigned to '{existing_area_with_same_idx}'"
            )

        # Lock this cortical area index during creation to prevent concurrent reads/writes
        lock_acquired = False
        try:
            try:
                lock_acquired = state_manager.lock_cortical_area(
                    cortical_idx, "BDU", operation="corticogenesis:add_area"
                )
            except Exception as lock_e:
                logger.warning(
                    f"[LOCK] Failed to acquire lock for cortical_idx={cortical_idx}: {lock_e}"
                )

            # Create the cortical area with assigned cortical_idx
            area = CorticalArea(
                name=name,
                dimensions=dimensions,
                position=position,
                area_type=area_type,
                properties=properties or {},
                cortical_id=cortical_id,
                cortical_idx=cortical_idx,  # Now we assign reserved or next available index
            )

            # Add to cortical areas dict
            self.cortical_areas[area.id] = area

            # Synchronize bidirectional cortical mapping
            self._sync_cortical_mapping(area.id, cortical_idx)
        finally:
            # Always release the lock
            try:
                if lock_acquired:
                    state_manager.unlock_cortical_area(cortical_idx, "BDU")
            except Exception as unlock_e:
                logger.warning(
                    f"[LOCK] Failed to release lock for cortical_idx={cortical_idx}: {unlock_e}"
                )


        logger.debug(
            f"Added cortical area '{name}' with ID {area.id} and cortical_idx {cortical_idx}"
        )

        # Update state manager with current brain statistics
        try:
            self._update_brain_statistics()
        except Exception as e:
            logger.warning(
                f"Failed to update brain statistics after adding {area.id}: {e}"
            )

        # Ensure NPU registry is synced so lookups by cortical_id work immediately
        try:
            if hasattr(self, "sync_cortical_areas_to_npu") and hasattr(self, "_npu_interface") and self._npu_interface is not None:
                self.sync_cortical_areas_to_npu()
        except Exception as sync_err:
            logger.warning(f"[NPU-SYNC] Failed to sync cortical areas after add_cortical_area({area.id}): {sync_err}")

        # Trigger brain region mapping validation after adding cortical area
        try:
            self._trigger_brain_region_validation()
        except Exception as validation_err:
            logger.warning(f"[BRAIN REGIONS] Failed to validate brain region mappings after adding {area.id}: {validation_err}")

        return area.id

    def get_cortical_area(self, cortical_id: str) -> CorticalArea:
        """Get cortical area by ID.

        Args:
            cortical_id: ID of the cortical area

        Returns:
            CorticalArea object

        Raises:
            KeyError: If the cortical_id doesn't exist
        """
        if cortical_id not in self.cortical_areas:
            raise KeyError(f"Cortical area {cortical_id} does not exist")

        return self.cortical_areas[cortical_id]

    def get_cortical_area_by_name(self, name: str) -> Optional[CorticalArea]:
        """Get a cortical area by its name.

        Args:
            name: Name of the cortical area

        Returns:
            The cortical area object, or None if not found
        """
        for area in self.cortical_areas.values():
            if area.name == name:
                return area

        return None

    def get_cortical_area_names(self) -> List[str]:
        """Get the names of all cortical areas.

        Returns:
            List of area names
        """
        return [area.name for area in self.cortical_areas.values()]

    # === MEMORY AREA MANAGEMENT ===

    def register_memory_area(
        self, cortical_id: str, temporal_depth: int
    ) -> bool:
        """Register a cortical area as a memory area with temporal depth.

        Args:
            cortical_id: ID of the cortical area
            temporal_depth: Temporal depth for pattern recognition

        Returns:
            True if successful, False if area doesn't exist
        """
        import sys
        debug_mem = '--debug-mem' in sys.argv
        
        if debug_mem:
            print(f"[DEBUG-MEM] ConnectomeManager.register_memory_area called: cortical_id={cortical_id}, temporal_depth={temporal_depth}")
        
        logger.info(
            f"[MEMORY-REG] Starting registration for cortical_id={cortical_id}, temporal_depth={temporal_depth}"
        )

        if cortical_id not in self.cortical_areas:
            logger.error(
                f"Cannot register memory area: cortical area {cortical_id} not found"
            )
            logger.debug(
                f"Available cortical areas: {list(self.cortical_areas.keys())}"
            )
            return False

        logger.info(
            f"[MEMORY-REG] Cortical area {cortical_id} found, adding to memory_areas set"
        )
        self.memory_areas.add(cortical_id)
        logger.info(
            f"[MEMORY-REG] memory_areas set now contains: {self.memory_areas}"
        )

        # CRITICAL FIX: Initialize upstream mappings for this memory area
        if cortical_id not in self.memory_area_upstream_mappings:
            self.memory_area_upstream_mappings[cortical_id] = set()
            logger.info(
                f"[MEMORY-REG] Initialized upstream mappings for {cortical_id}"
            )

        # CRITICAL FIX: Scan existing cortical mappings for connections to this memory area
        # This handles cases where regular cortical mappings target memory areas
        self._scan_and_convert_memory_mappings(cortical_id)

        #  Update StateManager cache - but don't fail registration if
        #  StateManager fails
        state_manager_success = False
        try:
            logger.info("[MEMORY-REG] Importing StateManager...")
            from feagi.core.state_manager import get_state_manager

            logger.info("[MEMORY-REG] Getting StateManager instance...")
            state_manager = get_state_manager()
            logger.info(
                f"[MEMORY-REG] Calling state_manager.register_memory_area({cortical_id}, {temporal_depth})"
            )
            result = state_manager.register_memory_area_for_stats(cortical_id)
            logger.info(
                f"[MEMORY-REG] StateManager call returned: {type(result)}"
            )
            logger.info(f"[MEMORY-REG] Result.is_err: {result.is_err}")

            if result.is_err:
                error_msg = result.unwrap_err()
                logger.warning(
                    f"StateManager registration failed: {error_msg}"
                )
                logger.warning(
                    "[MEMORY-REG] Continuing with memory area registration despite StateManager failure"
                )
            else:
                logger.info(
                    "[MEMORY-REG] StateManager registration successful"
                )
                state_manager_success = True
        except Exception as e:
            logger.warning(
                f"Error registering memory area with StateManager: {e}"
            )
            logger.exception("[MEMORY-REG] Full exception trace:")
            logger.warning(
                "[MEMORY-REG] Continuing with memory area registration despite StateManager failure"
            )

        #  CRITICAL FIX: Don't fail memory area registration if StateManager
        #  fails
        # The memory area should still be functional for cortical mappings
        logger.info(
            f"Registered memory area {cortical_id} with temporal_depth={temporal_depth}"
        )
        logger.info(
            f"[MEMORY-REG] StateManager success: {state_manager_success}"
        )
        logger.info(
            f"[MEMORY-REG] Final memory_areas set: {self.memory_areas}"
        )

        # CRITICAL FIX: Register memory area with FCL manager
        # This ensures memory areas are handled correctly by FQ Sampler
        try:
            area = self.cortical_areas[cortical_id]
            cortical_idx = area.cortical_idx

            #  FCL window size should be at least temporal_depth for pattern
            #  recognition
            # Use default FCL window size as minimum to avoid issues
            fcl_manager = self._get_fcl_manager()
            default_window_size = 20  # Fallback if FCL manager not available
            if fcl_manager and hasattr(fcl_manager, 'default_window_size'):
                default_window_size = fcl_manager.default_window_size
            
            fcl_window_size = max(temporal_depth, default_window_size)

            logger.info(
                f"[MEMORY-REG] Registering cortical_idx={cortical_idx} with FCL manager (window_size={fcl_window_size})"
            )
            if fcl_manager:
                fcl_manager.register_memory_cortical(
                    cortical_idx, fcl_window_size
                )
            logger.info(
                f"[MEMORY-REG] Successfully registered cortical_idx={cortical_idx} with FCL manager"
            )

        except Exception as e:
            logger.error(
                f"[MEMORY-REG] Failed to register memory area {cortical_id} with FCL manager: {e}"
            )
            logger.exception("[MEMORY-REG] FCL registration exception:")
            # Don't fail the entire registration if FCL registration fails

        # CRITICAL FIX: Register memory area with PlasticityService
        # This is the missing link that prevents memory neurons from being created!
        try:
            if debug_mem:
                print(f"[DEBUG-MEM] Attempting to register with PlasticityService...")
            
            # 🦀 RUST: PlasticityService integration needs update for Rust burst engine
            # For now, skip PlasticityService registration (will be added in Rust integration)
            plasticity_service = None
            
            if plasticity_service:
                # Get cortical area index and upstream areas
                area = self.cortical_areas[cortical_id]
                area_idx = area.cortical_idx
                
                # Get upstream areas from memory mappings
                upstream_areas = list(self.memory_area_upstream_mappings.get(cortical_id, set()))
                
                if debug_mem:
                    print(f"[DEBUG-MEM] Registering with PlasticityService: area_idx={area_idx}, temporal_depth={temporal_depth}, upstream_areas={upstream_areas}")
                
                success = plasticity_service.register_memory_area(
                    area_idx=area_idx,
                    temporal_depth=temporal_depth,
                    upstream_areas=upstream_areas
                )
                
                if success:
                    if debug_mem:
                        print(f"[DEBUG-MEM] ✅ Successfully registered memory area {cortical_id} with PlasticityService!")
                    logger.info(f"[MEMORY-REG] Successfully registered {cortical_id} with PlasticityService")
                else:
                    if debug_mem:
                        print(f"[DEBUG-MEM] ❌ Failed to register memory area {cortical_id} with PlasticityService")
                    logger.warning(f"[MEMORY-REG] Failed to register {cortical_id} with PlasticityService")
            else:
                if debug_mem:
                    print(f"[DEBUG-MEM] ❌ PlasticityService not available - this is why memory neurons won't be created!")
                logger.warning("[MEMORY-REG] PlasticityService not available for memory area registration")
                
        except Exception as e:
            if debug_mem:
                print(f"[DEBUG-MEM] ❌ Exception registering with PlasticityService: {e}")
            logger.warning(f"[MEMORY-REG] Failed to register with PlasticityService: {e}")
            # Don't fail the entire registration if PlasticityService registration fails

        return True

    def unregister_memory_area(self, cortical_id: str) -> bool:
        """Unregister a memory area.

        Args:
            cortical_id: ID of the memory area to unregister

        Returns:
            True if successful
        """
        self.memory_areas.discard(cortical_id)

        # Remove from upstream mappings
        for memory_area in list(self.memory_area_upstream_mappings.keys()):
            if memory_area == cortical_id:
                del self.memory_area_upstream_mappings[memory_area]

        # Update StateManager cache
        try:
            from feagi.core.state_manager import get_state_manager

            state_manager = get_state_manager()
            result = state_manager.unregister_memory_area(cortical_id)
            if result.is_err:
                logger.warning(
                    f"Failed to unregister memory area from StateManager: {result.unwrap_err()}"
                )
        except Exception as e:
            logger.warning(
                f"Error unregistering memory area from StateManager: {e}"
            )

        logger.info(f"Unregistered memory area {cortical_id}")
        return True

    def add_memory_area_mapping(
        self, source_cortical_id: str, target_cortical_id: str
    ) -> None:
        """Add a mapping to a memory area and update FCL window cache.

        Args:
            source_cortical_id: Source cortical area
            target_cortical_id: Target cortical area (memory area)
        """
        # Check if debug-mem is enabled
        import sys
        debug_mem = '--debug-mem' in sys.argv
        try:
            from feagi.core.state_manager import get_state_manager
            state_manager = get_state_manager()
            mem_debug = state_manager.is_mem_debug_enabled() if state_manager else False
        except Exception:
            mem_debug = False
        
        if debug_mem:
            print(f"[DEBUG-MEM] ConnectomeManager.add_memory_area_mapping called: {source_cortical_id} -> {target_cortical_id}")
            
        if mem_debug:
            logger.info(
                f"🔗 [MEMORY-DEBUG] add_memory_area_mapping() called: {source_cortical_id} -> {target_cortical_id}"
            )
            logger.info(
                f"🔗 [MEMORY-DEBUG] Current memory_areas set: {self.memory_areas}"
            )
            logger.info(
                f"🔗 [MEMORY-DEBUG] Is {target_cortical_id} in memory_areas? {target_cortical_id in self.memory_areas}"
            )
            logger.info(
                f"🔗 [MEMORY-DEBUG] Available cortical areas: {list(self.cortical_areas.keys())}"
            )
        else:
            logger.info(
                f"[MEMORY-MAPPING] Called add_memory_area_mapping({source_cortical_id} -> {target_cortical_id})"
            )

        if target_cortical_id in self.memory_areas:
            if mem_debug:
                logger.info(
                    f"🔗 [MEMORY-DEBUG] Target {target_cortical_id} is a registered memory area, adding upstream mapping..."
                )
            self.memory_area_upstream_mappings[target_cortical_id].add(
                source_cortical_id
            )
            if mem_debug:
                logger.info(
                    f"🔗 [MEMORY-DEBUG] Updated upstream mappings: {dict(self.memory_area_upstream_mappings)}"
                )
            else:
                logger.info(
                    f"[MEMORY-MAPPING] Updated upstream mappings: {dict(self.memory_area_upstream_mappings)}"
                )

            # Update StateManager cache
            try:
                logger.info("[MEMORY-MAPPING] Updating StateManager cache...")
                from feagi.core.state_manager import get_state_manager

                state_manager = get_state_manager()
                state_manager.add_cortical_mapping_to_cache(
                    source_cortical_id, target_cortical_id
                )
                logger.debug(
                    f"Added mapping {source_cortical_id} -> {target_cortical_id} (memory area)"
                )
                logger.info(
                    "[MEMORY-MAPPING] StateManager cache updated successfully"
                )
            except Exception as e:
                logger.warning(
                    f"Error updating StateManager mapping cache: {e}"
                )
                logger.exception(
                    "[MEMORY-MAPPING] StateManager cache update exception:"
                )

            # CRITICAL FIX: Update PlasticityService with new upstream mapping
            try:
                if debug_mem:
                    print(f"[DEBUG-MEM] Updating PlasticityService with new upstream mapping: {source_cortical_id} -> {target_cortical_id}")
                
                # 🦀 RUST: PlasticityService integration needs update for Rust burst engine
                # For now, skip PlasticityService update
                plasticity_service = None
                
                if plasticity_service:
                    # Get cortical area index for the memory area
                    target_area = self.cortical_areas.get(target_cortical_id)
                    source_area = self.cortical_areas.get(source_cortical_id)
                    
                    if target_area and source_area:
                        target_area_idx = target_area.cortical_idx
                        source_area_idx = source_area.cortical_idx
                        
                        # Get current upstream areas for this memory area
                        current_upstream = list(self.memory_area_upstream_mappings.get(target_cortical_id, set()))
                        upstream_indices = []
                        for upstream_id in current_upstream:
                            upstream_area = self.cortical_areas.get(upstream_id)
                            if upstream_area:
                                upstream_indices.append(upstream_area.cortical_idx)
                        
                        if debug_mem:
                            print(f"[DEBUG-MEM] Re-registering memory area {target_cortical_id} (idx={target_area_idx}) with updated upstream areas: {upstream_indices}")
                        
                        # Re-register the memory area with updated upstream areas
                        temporal_depth = target_area.temporal_depth if hasattr(target_area, 'temporal_depth') else 3
                        success = plasticity_service.register_memory_area(
                            area_idx=target_area_idx,
                            temporal_depth=temporal_depth,
                            upstream_areas=upstream_indices
                        )
                        
                        if success:
                            if debug_mem:
                                print(f"[DEBUG-MEM] ✅ Successfully updated PlasticityService with upstream mapping")
                            logger.info(f"[MEMORY-MAPPING] Updated PlasticityService with upstream mapping: {source_cortical_id} -> {target_cortical_id}")
                        else:
                            if debug_mem:
                                print(f"[DEBUG-MEM] ❌ Failed to update PlasticityService with upstream mapping")
                            logger.warning(f"[MEMORY-MAPPING] Failed to update PlasticityService with upstream mapping")
                    else:
                        if debug_mem:
                            print(f"[DEBUG-MEM] ❌ Could not find cortical areas: target={target_area is not None}, source={source_area is not None}")
                else:
                    if debug_mem:
                        print(f"[DEBUG-MEM] ❌ PlasticityService not available for upstream mapping update")
                    logger.warning("[MEMORY-MAPPING] PlasticityService not available for upstream mapping update")
                    
            except Exception as e:
                if debug_mem:
                    print(f"[DEBUG-MEM] ❌ Exception updating PlasticityService with upstream mapping: {e}")
                logger.warning(f"[MEMORY-MAPPING] Failed to update PlasticityService with upstream mapping: {e}")

            # CRITICAL FIX: Update MemoryProcessor with new upstream mapping
            try:
                logger.info("[MEMORY-MAPPING] Updating MemoryProcessor...")
                # 🦀 RUST: MemoryProcessor integration needs update for Rust burst engine
                # For now, skip MemoryProcessor update
                burst_engine = None
                if burst_engine and hasattr(burst_engine, 'memory_processor') and burst_engine.memory_processor:
                    #  CRITICAL: Ensure memory area is registered with
                    #  MemoryProcessor
                    if (
                        target_cortical_id
                        not in burst_engine.memory_processor.active_memory_areas
                    ):
                        logger.warning(
                            f"[MEMORY-MAPPING] Memory area {target_cortical_id} not in active_memory_areas, registering now..."
                        )

                        # Get memory area properties for registration
                        area = self.cortical_areas.get(target_cortical_id)
                        if area and hasattr(area, "properties"):
                            props = area.properties or {}
                            temporal_depth = props.get("temporal_depth", 1)
                            init_lifespan = props.get("init_lifespan", 9)
                            lifespan_growth_rate = props.get(
                                "lifespan_growth_rate", 1.0
                            )
                            longterm_threshold = props.get(
                                "longterm_mem_threshold", 100
                            )

                            registered = burst_engine.memory_processor.register_memory_area(
                                cortical_id=target_cortical_id,
                                temporal_depth=temporal_depth,
                                initial_lifespan=init_lifespan,
                                lifespan_growth_rate=lifespan_growth_rate,
                                longterm_threshold=longterm_threshold,
                                upstream_areas=set(),
                            )
                            if registered:
                                logger.info(
                                    f"[MEMORY-MAPPING] Successfully registered {target_cortical_id} with MemoryProcessor"
                                )
                            else:
                                logger.error(
                                    f"[MEMORY-MAPPING] Failed to register {target_cortical_id} with MemoryProcessor"
                                )
                        else:
                            logger.error(
                                f"[MEMORY-MAPPING] Cannot get properties for memory area {target_cortical_id}"
                            )

                    # Get current upstream areas for this memory area
                    current_upstream = self.memory_area_upstream_mappings[
                        target_cortical_id
                    ]
                    #  Update MemoryProcessor with the complete set of upstream
                    #  areas
                    burst_engine.memory_processor.update_memory_area_upstream(
                        target_cortical_id, current_upstream
                    )
                    logger.debug(
                        f"Updated MemoryProcessor upstream areas for {target_cortical_id}: {current_upstream}"
                    )
                    logger.info(
                        "[MEMORY-MAPPING] MemoryProcessor updated successfully"
                    )

                    # ENHANCED DEBUG: Check final state
                    logger.info(
                        f"[MEMORY-MAPPING] Final MemoryProcessor active areas: {list(burst_engine.memory_processor.active_memory_areas)}"
                    )
                else:
                    logger.warning(
                        "BurstEngine or MemoryProcessor not available for upstream mapping update"
                    )
                    logger.info(
                        f"[MEMORY-MAPPING] BurstEngine available: {burst_engine is not None}"
                    )
                    if burst_engine:
                        logger.info(
                            f"[MEMORY-MAPPING] MemoryProcessor available: {hasattr(burst_engine, 'memory_processor') and burst_engine.memory_processor is not None}"
                        )
            except Exception as e:
                logger.warning(
                    f"Error updating MemoryProcessor upstream mapping: {e}"
                )
                logger.exception(
                    "[MEMORY-MAPPING] MemoryProcessor update exception:"
                )

            logger.info(
                "[MEMORY-MAPPING] Memory area mapping completed successfully"
            )
        else:
            logger.error(
                f"[MEMORY-MAPPING] Target cortical area {target_cortical_id} is NOT in memory_areas set!"
            )
            logger.error(
                f"[MEMORY-MAPPING] Available memory areas: {list(self.memory_areas)}"
            )
            logger.error(
                "[MEMORY-MAPPING] This will cause the memory mapping to be silently ignored!"
            )

    def remove_memory_area_mapping(
        self, source_cortical_id: str, target_cortical_id: str
    ) -> None:
        """Remove a mapping from a memory area and update FCL window cache.

        Args:
            source_cortical_id: Source cortical area
            target_cortical_id: Target cortical area (memory area)
        """
        if target_cortical_id in self.memory_area_upstream_mappings:
            self.memory_area_upstream_mappings[target_cortical_id].discard(
                source_cortical_id
            )

            # Update StateManager cache
            try:
                from feagi.core.state_manager import get_state_manager

                state_manager = get_state_manager()
                state_manager.remove_cortical_mapping_from_cache(
                    source_cortical_id, target_cortical_id
                )
                logger.debug(
                    f"Removed mapping {source_cortical_id} -> {target_cortical_id} (memory area)"
                )
            except Exception as e:
                logger.warning(
                    f"Error updating StateManager mapping cache: {e}"
                )

    def is_memory_area(self, cortical_id: str) -> bool:
        """Check if a cortical area is a memory area."""
        return cortical_id in self.memory_areas

    def get_memory_areas(self) -> List[str]:
        """Get list of all memory area IDs."""
        return list(self.memory_areas)
    
    def _get_memory_neurons_by_cortical_area(self, cortical_id: str, cortical_idx: int) -> List[int]:
        """Get memory neurons for a specific memory cortical area.
        
        Args:
            cortical_id: Memory cortical area ID
            cortical_idx: Fast integer index for the cortical area
            
        Returns:
            List of memory neuron IDs in the area
        """
        if not hasattr(self, 'memory_neuron_array') or self.memory_neuron_array is None:
            return []
            
        memory_neurons = []
        
        # Search through active memory neurons
        for idx in range(self.memory_neuron_array.count):
            if (self.memory_neuron_array.valid_mask[idx] and 
                self.memory_neuron_array.is_active[idx] and
                self.memory_neuron_array.cortical_idxs[idx] == cortical_idx):
                
                # Get neuron ID from index mapping
                neuron_id = self.memory_neuron_array.index_to_neuron_id.get(idx)
                if neuron_id is not None:
                    memory_neurons.append(neuron_id)
        
        return memory_neurons

    def get_upstream_areas_for_memory(
        self, memory_cortical_id: str
    ) -> Set[str]:
        """Get upstream cortical areas for a memory area."""
        return self.memory_area_upstream_mappings.get(
            memory_cortical_id, set()
        )

    def _scan_and_convert_memory_mappings(self, memory_cortical_id: str) -> None:
        """Scan existing cortical mappings and convert connections to memory area.
        
        This method finds all regular cortical mappings that target the given memory area
        and converts them to memory area mappings. This handles cases where the genome
        defines regular cortical connections to memory areas.
        
        Args:
            memory_cortical_id: The memory area to scan for incoming connections
        """
        # Check if debug-mem is enabled
        try:
            from feagi.core.state_manager import get_state_manager
            state_manager = get_state_manager()
            mem_debug = state_manager.is_mem_debug_enabled() if state_manager else False
        except Exception:
            mem_debug = False
            
        if mem_debug:
            logger.info(f"🔍 [MEMORY-DEBUG] Scanning cortical mappings for connections to memory area: {memory_cortical_id}")
            
        converted_count = 0
        
        # Scan all cortical areas for mappings that target this memory area
        for source_area_id, area in self.cortical_areas.items():
            if hasattr(area, 'properties') and 'mapping' in area.properties:
                mapping = area.properties['mapping']
                
                if mem_debug:
                    logger.info(f"🔍 [MEMORY-DEBUG] Checking area {source_area_id} mapping: {mapping}")
                
                # Handle different mapping formats
                targets_to_check = []
                
                if isinstance(mapping, dict):
                    # Complex mapping format: {dst_area: [connection_specs]}
                    targets_to_check = list(mapping.keys())
                elif isinstance(mapping, list):
                    # Simple list format: [dst_area1, dst_area2, ...]
                    targets_to_check = mapping
                
                if mem_debug:
                    logger.info(f"🔍 [MEMORY-DEBUG] Targets to check for {source_area_id}: {targets_to_check}")
                
                # Check if this source area maps to our memory area
                if memory_cortical_id in targets_to_check:
                    if mem_debug:
                        logger.info(f"🔍 [MEMORY-DEBUG] Found cortical mapping: {source_area_id} -> {memory_cortical_id}")
                        
                    # Add to memory area upstream mappings
                    if memory_cortical_id not in self.memory_area_upstream_mappings:
                        self.memory_area_upstream_mappings[memory_cortical_id] = set()
                        
                    if source_area_id not in self.memory_area_upstream_mappings[memory_cortical_id]:
                        self.memory_area_upstream_mappings[memory_cortical_id].add(source_area_id)
                        converted_count += 1
                        
                        if mem_debug:
                            logger.info(f"🔗 [MEMORY-DEBUG] Converted cortical mapping to memory mapping: {source_area_id} -> {memory_cortical_id}")
                        else:
                            logger.info(f"[MEMORY-MAPPING] Converted cortical mapping to memory mapping: {source_area_id} -> {memory_cortical_id}")
                            
        if converted_count > 0:
            if mem_debug:
                logger.info(f"🔗 [MEMORY-DEBUG] Converted {converted_count} cortical mappings to memory mappings for {memory_cortical_id}")
                logger.info(f"🔗 [MEMORY-DEBUG] Final upstream mappings: {dict(self.memory_area_upstream_mappings)}")
            else:
                logger.info(f"[MEMORY-MAPPING] Converted {converted_count} cortical mappings to memory mappings for {memory_cortical_id}")
        elif mem_debug:
            logger.info(f"🔍 [MEMORY-DEBUG] No cortical mappings found targeting memory area {memory_cortical_id}")

    def rescan_all_memory_mappings(self) -> None:
        """Rescan all memory areas for cortical mappings.
        
        This method should be called after genome loading is complete to ensure
        all cortical mappings to memory areas are properly converted.
        """
        # Check if debug-mem is enabled
        try:
            from feagi.core.state_manager import get_state_manager
            state_manager = get_state_manager()
            mem_debug = state_manager.is_mem_debug_enabled() if state_manager else False
        except Exception:
            mem_debug = False
            
        if mem_debug:
            logger.info("🔍 [MEMORY-DEBUG] Rescanning all memory areas for cortical mappings...")
            logger.info(f"🔍 [MEMORY-DEBUG] Memory areas to rescan: {list(self.memory_areas)}")
            
        # total_converted metric removed (unused)
        for memory_area_id in self.memory_areas:
            if mem_debug:
                logger.info(f"🔍 [MEMORY-DEBUG] Rescanning memory area: {memory_area_id}")
            self._scan_and_convert_memory_mappings(memory_area_id)
            
        if mem_debug:
            logger.info(f"🔍 [MEMORY-DEBUG] Rescan complete. Final upstream mappings: {dict(self.memory_area_upstream_mappings)}")

    def get_all_cortical_ids(self) -> List[str]:
        """Get all cortical area IDs (6-character strings).

        Returns:
            List of cortical area IDs
        """
        try:
            # Prefer mapping: id -> idx
            if hasattr(self, "cortical_mapping") and self.cortical_mapping:
                all_mappings = self.cortical_mapping.get_all_mappings()
                # Keys are cortical IDs
                return sorted(list(all_mappings.keys()))
            # Fallback to extracting IDs from area objects
            ids: List[str] = []
            for idx, area in getattr(self, "cortical_areas", {}).items():
                cid = getattr(area, "cortical_id", None)
                if cid:
                    ids.append(cid)
            return sorted(ids)
        except Exception:
            # Last resort: empty list on failure
            return []

    def get_all_cortical_indices(self) -> List[int]:
        """Get all cortical area indices (integers) used by the FCL.

        Returns:
            List of cortical area indices
        """
        if hasattr(self, "cortical_mapping") and self.cortical_mapping:
            all_mappings = self.cortical_mapping.get_all_mappings()
            return sorted(list(all_mappings.values()))
        return []

    def get_max_cortical_area_dimensions(self) -> Tuple[int, int, int]:
        """Calculate the maximum dimensions across all cortical areas.

        This method provides the maximum cortical area dimensions for spatial hash
        cache sizing and other components that need to know the largest coordinate
        space required by any cortical area.

        IMPORTANT: Returns cortical area dimensions only (local coordinate space),
        NOT global brain positioning coordinates.

        Returns:
            Tuple of (max_x, max_y, max_z) dimensions across all cortical areas
        """
        max_dims = [1, 1, 1]  # Minimum reasonable dimensions

        for area in self.cortical_areas.values():
            dims = area.dimensions
            max_dims[0] = max(max_dims[0], dims[0])
            max_dims[1] = max(max_dims[1], dims[1])
            max_dims[2] = max(max_dims[2], dims[2])

        from feagi.core.state_manager import get_state_manager

        state_manager = get_state_manager()
        if state_manager.is_debug_bdu_enabled():
            logger.debug(
                f"[BDU-DEBUG] Maximum cortical area dimensions: {max_dims[0]}x{max_dims[1]}x{max_dims[2]}"
            )
        return tuple(max_dims)


    def _convert_hierarchical_to_flat_parameters(self, hierarchical_params: Dict[str, Any]) -> Dict[str, Any]:
        """Convert hierarchical genome property names to API format.
        
        Returns BOTH canonical names AND legacy names for backward compatibility.
        Canonical names are the source of truth for API consumers.
        
        Args:
            hierarchical_params: Parameters in hierarchical genome format
            
        Returns:
            Parameters with both canonical and legacy names
        """
        # Canonical property names (used by API consumers)
        canonical_names = {
            "consecutive_fire_cnt_max": "consecutive_fire_cnt_max",
            "firing_threshold": "firing_threshold",
            "refractory_period": "refractory_period",
            "leak_coefficient": "leak_coefficient",
            "snooze_length": "neuron_snooze_period",  # Canonical: neuron_snooze_period
            "neuron_excitability": "neuron_excitability",
        }
        
        # Legacy aliases (for backward compatibility only - will be phased out)
        legacy_aliases = {
            "consecutive_fire_cnt_max": "c_fr_c",
            "synapse_attractivity": "synatt", 
            "postsynaptic_current": "pstcr",
            "postsynaptic_current_max": "pstcrm",
            "firing_threshold": "fire_t",
            "refractory_period": "refrac",
            "leak_coefficient": "leak_c",
            "leak_variability": "leak_v",
            "snooze_length": "snooze",
            "degeneration": "de_gen",
            "psp_uniform_distribution": "pspuni",
            "firing_threshold_increment_x": "ftincx",
            "firing_threshold_increment_y": "ftincy", 
            "firing_threshold_increment_z": "ftincz",
            "firing_threshold_limit": "fthlim",
            "mp_charge_accumulation": "mp_acc",
            "mp_driven_psp": "mp_psp",
            "longterm_mem_threshold": "mem__t",
            "lifespan_growth_rate": "mem_gr",
            "init_lifespan": "mem_ls",
            "neuron_excitability": "excite",
        }
        
        flat_params = {}
        
        # Process each hierarchical property
        for hierarchical_key, hierarchical_value in hierarchical_params.items():
            # Always add canonical name (source of truth)
            if hierarchical_key in canonical_names:
                canonical_key = canonical_names[hierarchical_key]
                flat_params[canonical_key] = hierarchical_value
            
            # Add legacy alias for backward compatibility
            if hierarchical_key in legacy_aliases:
                legacy_key = legacy_aliases[hierarchical_key]
                flat_params[legacy_key] = hierarchical_value
            
            # If not in either mapping, keep as-is
            if hierarchical_key not in canonical_names and hierarchical_key not in legacy_aliases:
                flat_params[hierarchical_key] = hierarchical_value
                
        return flat_params

    def get_cortical_area_properties(self, cortical_id: str) -> Dict[str, Any]:
        """Get properties of a cortical area.

        This is the SINGLE SOURCE OF TRUTH for cortical area properties.
        All other components must use this method to access cortical properties.

        Args:
            cortical_id: String identifier for cortical area

        Returns:
            Dictionary containing cortical area properties:
            - id: Cortical area ID
            - cortical_idx: Integer index for the area
            - name: Area name
            - coordinates: 3D position (x,y,z)
            - dimensions: 3D dimensions (width,height,depth)
            - type: Area type
            - parameters: Area-specific parameters including mappings
            - neuron_count: Number of neurons in the area
        """
        try:
            # Enforce mapping↔area consistency first
            idx_from_map = self.cortical_mapping.get_idx(cortical_id)
            if idx_from_map is None:
                raise KeyError(f"Cortical ID '{cortical_id}' not found in mapping")
            area = self.get_cortical_area(cortical_id)
            if not area:
                raise KeyError(
                    f"Cortical area object not found for '{cortical_id}' (idx={idx_from_map})"
                )

            # Get cortical_idx through the mapping
            cortical_idx = self.cortical_mapping.get_idx(cortical_id)

            # Build complete properties dictionary with safe type conversion
            try:
                # Safely convert coordinates and dimensions to integers
                # Treat placeholder strings (e.g., 'x','y','z') as absent without error logs
                coordinates = []
                for i, x in enumerate(area.position):
                    try:
                        if isinstance(x, (int, float)):
                            coordinates.append(int(x))
                            continue
                        value_str = str(x).strip().lower()
                        if value_str in ("x", "y", "z", "", "none"):
                            coordinates.append(0)
                        else:
                            coordinates.append(int(float(value_str)))
                    except Exception:
                        # Downgrade noisy logs; placeholders are common from clients
                        try:
                            from feagi.core.state_manager import FeagiStateManager

                            if FeagiStateManager.instance().is_debug_npu_enabled():
                                self.logger.debug(
                                    f"[SANITIZE] position[{i}]='{x}' for area {cortical_id} -> 0"
                                )
                        except Exception:
                            pass
                        coordinates.append(0)

                dimensions = []
                for i, x in enumerate(area.dimensions):
                    try:
                        if isinstance(x, (int, float)):
                            dimensions.append(int(x))
                            continue
                        value_str = str(x).strip().lower()
                        if value_str in (
                            "w",
                            "h",
                            "d",
                            "width",
                            "height",
                            "depth",
                            "",
                            "none",
                        ):
                            dimensions.append(1)
                        else:
                            dimensions.append(int(float(value_str)))
                    except Exception:
                        try:
                            from feagi.core.state_manager import FeagiStateManager

                            if FeagiStateManager.instance().is_debug_npu_enabled():
                                self.logger.debug(
                                    f"[SANITIZE] dimensions[{i}]='{x}' for area {cortical_id} -> 1"
                                )
                        except Exception:
                            pass
                        dimensions.append(1)

                # Safe neuron count (handle None from NPU variant)
                _neuron_ids_for_count = self.get_neurons_by_area(cortical_id)
                if _neuron_ids_for_count is None:
                    _neuron_ids_for_count = []

                # Convert hierarchical genome properties to flat API parameter format
                hierarchical_params = area.properties.copy() if area.properties else {}
                flat_parameters = self._convert_hierarchical_to_flat_parameters(hierarchical_params)
                
                # Derive canonical cortical_group strictly from parameters/type
                try:
                    raw_group = (
                        flat_parameters.get("cortical_group")
                        or flat_parameters.get("group")
                        or getattr(area, "area_type", "")
                    )
                except Exception:
                    raw_group = ""
                cortical_group = str(raw_group).upper() if raw_group else "CUSTOM"

                properties = {
                    "id": cortical_id,
                    "cortical_idx": (
                        int(cortical_idx) if cortical_idx is not None else None
                    ),
                    "name": area.name,
                    "coordinates": tuple(coordinates),
                    "dimensions": tuple(dimensions),
                    "type": area.area_type,
                    "cortical_group": cortical_group,
                    "parameters": flat_parameters,
                    "neuron_count": int(len(_neuron_ids_for_count)),
                }
            except Exception as conversion_error:
                self.logger.error(
                    f"Error during property conversion for area {cortical_id}: {conversion_error}"
                )
                self.logger.error(
                    f"Area position type: {type(area.position)}, value: {area.position}"
                )
                self.logger.error(
                    f"Area dimensions type: {type(area.dimensions)}, value: {area.dimensions}"
                )
                raise conversion_error

            #  CRITICAL FIX: Preserve existing mapping data from
            #  NeuroEmbryogenesis
            #  The area.properties["mapping"] may contain mapping
            #  specifications that haven't
            #  been converted to actual synaptic connections yet (e.g., memory
            #  mappings)
            existing_mapping = (
                area.properties.get("mapping", {}) if area.properties else {}
            )

            # Ensure mapping information is included in parameters
            if "mapping" not in properties["parameters"]:
                properties["parameters"]["mapping"] = {}

            # Start with existing mapping data from NeuroEmbryogenesis
            outgoing_mappings = existing_mapping.copy()

            #  Update mapping information in parameters (preserving
            #  NeuroEmbryogenesis data only - no expensive synapse operations)
            properties["parameters"]["mapping"] = outgoing_mappings

            #  Convert all numpy types to native Python types for JSON
            #  serialization
            return self._convert_numpy_types_to_python(properties)
        except KeyError:
            self.logger.warning(f"Cortical area {cortical_id} not found")
            return {}
        except Exception as e:
            self.logger.error(
                f"Error getting properties for area {cortical_id}: {e}"
            )
            return {}

    def _extract_neuron_properties_for_area(
        self, cortical_id: str
    ) -> Dict[str, Any]:
        """Extract actual neuron properties from the neuron array for a
        cortical area.

        For regular cortical areas: Returns representative neuron properties like excitability,
        threshold, etc. by sampling neurons in the area and computing averages.

        For memory cortical areas: Returns properties from the area's template configuration
        since memory areas don't have regular neurons.
        """
        try:
            area = self.get_cortical_area(cortical_id)
            if not area:
                return {}

            # Check if this is a memory cortical area
            is_memory_area = (
                area.properties
                and area.properties.get("sub_group_id") == "MEMORY"
            )

            if is_memory_area:
                #  For memory areas, return properties from the area's
                #  configuration
                #  Memory areas don't have regular neurons, so we use template
                #  properties
                memory_properties = {
                    # Standard neuron properties (from memory template)
                    "neuron_excitability": area.properties.get(
                        "neuron_excitability", 1.0
                    ),
                    "firing_threshold": area.properties.get(
                        "firing_threshold", 1.0
                    ),
                    "refractory_period": area.properties.get(
                        "refractory_period", 0
                    ),
                    "leak_coefficient": area.properties.get(
                        "leak_coefficient", 0.0
                    ),
                    # Memory-specific properties
                    "init_lifespan": area.properties.get("init_lifespan", 9),
                    "lifespan_growth_rate": area.properties.get(
                        "lifespan_growth_rate", 1.0
                    ),
                    "longterm_mem_threshold": area.properties.get(
                        "longterm_mem_threshold", 100
                    ),
                    "temporal_depth": area.properties.get("temporal_depth", 1),
                    "sub_group_id": "MEMORY",
                }
                self.logger.debug(
                    f"Extracted memory area properties for {cortical_id}: {memory_properties}"
                )
                return memory_properties

            #  For regular cortical areas, extract properties from actual
            #  neurons
            cortical_idx = area.cortical_idx
            # Return properties from cortical area (genome-derived, authoritative)
            # Neuron-level property extraction should use Rust NPU APIs if needed
            area_props_ex = area.properties.get("neuron_excitability", 1.0) if hasattr(area, "properties") and area.properties else 1.0
            area_props_threshold = area.properties.get("firing_threshold", 1.0) if hasattr(area, "properties") and area.properties else 1.0
            area_props_refractory = area.properties.get("refractory_period", 1) if hasattr(area, "properties") and area.properties else 1
            area_props_leak = area.properties.get("leak_coefficient", 0.0) if hasattr(area, "properties") and area.properties else 0.0

            return {
                "neuron_excitability": float(area_props_ex),
                "firing_threshold": float(area_props_threshold),
                "refractory_period": int(area_props_refractory),
                "leak_coefficient": float(area_props_leak),  # Already in 0-100 range from genome
            }

        except Exception as e:
            self.logger.error(
                f"Error extracting neuron properties for area {cortical_id}: {e}"
            )
            return {
                "neuron_excitability": 0.0,
                "firing_threshold": 0.0,
                "refractory_period": 0,
                "leak_coefficient": 0.0,
            }

    def update_cortical_area_properties(
        self, cortical_id: str, property_updates: Dict[str, Any]
    ) -> bool:
        """Update properties of a cortical area.

        This method ensures ConnectomeManager stays synchronized with genome changes.
        Called by GenomeService after genome updates to maintain consistency.

        Args:
            cortical_id: String identifier for cortical area
            property_updates: Dictionary of property_name -> new_value

        Returns:
            True if update successful, False otherwise
        """
        try:
            if cortical_id not in self.cortical_areas:
                self.logger.error(
                    f"Cannot update properties: Cortical area {cortical_id} not found"
                )
                return False

            area = self.cortical_areas[cortical_id]

            # Ensure area has properties dictionary - PRESERVE existing properties
            existing_props_count = len(getattr(area, 'properties', {})) if hasattr(area, 'properties') and area.properties else 0
            existing_has_destinations = hasattr(area, 'properties') and area.properties and 'cortical_destinations' in area.properties
            
            if not hasattr(area, "properties") or area.properties is None:
                area.properties = {}
            
            self.logger.info(f"🔄 [CONNECTOME-UPDATE] {cortical_id}: existing={existing_props_count} props, has_destinations={existing_has_destinations}")
            self.logger.info(f"🔄 [CONNECTOME-UPDATE] {cortical_id}: updating {len(property_updates)} properties: {list(property_updates.keys())}")

            # Update each property - write BOTH canonical and legacy names
            updated_properties = []
            for prop_name, new_value in property_updates.items():
                # Handle special property name mappings
                if prop_name == "neuron_consecutive_fire_count":
                    # Canonical + legacy
                    area.properties["consecutive_fire_cnt_max"] = new_value
                    area.properties["c_fr_c"] = new_value
                    updated_properties.append(
                        f"consecutive_fire_cnt_max={new_value}"
                    )
                elif prop_name == "cortical_name":
                    area.name = str(new_value)
                    updated_properties.append(f"name='{new_value}'")
                elif prop_name == "neuron_fire_threshold":
                    # Canonical + legacy
                    area.properties["firing_threshold"] = float(new_value)
                    area.properties["fire_t"] = float(new_value)
                    updated_properties.append(f"firing_threshold={float(new_value)}")
                elif prop_name == "neuron_fire_threshold_increment":
                    try:
                        inc = list(new_value)
                        x = float(inc[0]) if len(inc) > 0 else 0.0
                        y = float(inc[1]) if len(inc) > 1 else 0.0
                        z = float(inc[2]) if len(inc) > 2 else 0.0
                        area.properties["ftincx"] = x
                        area.properties["ftincy"] = y
                        area.properties["ftincz"] = z
                        updated_properties.append(f"ftincx={x}, ftincy={y}, ftincz={z}")
                    except Exception:
                        pass
                elif prop_name == "neuron_firing_threshold_limit":
                    area.properties["fthlim"] = int(new_value)
                    updated_properties.append(f"fthlim={int(new_value)}")
                elif prop_name == "neuron_refractory_period":
                    # Canonical + legacy
                    area.properties["refractory_period"] = int(new_value)
                    area.properties["refrac"] = int(new_value)
                    updated_properties.append(f"refractory_period={int(new_value)}")
                elif prop_name == "neuron_leak_coefficient":
                    # Canonical + legacy
                    area.properties["leak_coefficient"] = float(new_value)
                    area.properties["leak_c"] = float(new_value)
                    updated_properties.append(f"leak_coefficient={float(new_value)}")
                elif prop_name == "neuron_leak_variability":
                    area.properties["leak_v"] = float(new_value)
                    updated_properties.append(f"leak_v={float(new_value)}")
                elif prop_name == "neuron_snooze_period":
                    # Canonical + legacy
                    area.properties["neuron_snooze_period"] = int(new_value)
                    area.properties["snooze"] = int(new_value)
                    updated_properties.append(f"neuron_snooze_period={int(new_value)}")
                elif prop_name == "neuron_post_synaptic_potential":
                    area.properties["pstcr"] = float(new_value)
                    updated_properties.append(f"pstcr={float(new_value)}")
                elif prop_name == "neuron_post_synaptic_potential_max":
                    area.properties["pstcrm"] = float(new_value)
                    updated_properties.append(f"pstcrm={float(new_value)}")
                elif prop_name == "neuron_psp_uniform_distribution":
                    area.properties["pspuni"] = bool(new_value)
                    updated_properties.append(f"pspuni={bool(new_value)}")
                elif prop_name == "neuron_mp_charge_accumulation":
                    area.properties["mp_acc"] = bool(new_value)
                    updated_properties.append(f"mp_acc={bool(new_value)}")
                elif prop_name == "neuron_mp_driven_psp":
                    area.properties["mp_psp"] = bool(new_value)
                    updated_properties.append(f"mp_psp={bool(new_value)}")
                elif prop_name == "neuron_excitability":
                    # Canonical + legacy
                    ex = max(0.0, min(1.0, float(new_value)))
                    area.properties["neuron_excitability"] = ex
                    area.properties["excite"] = ex
                    updated_properties.append(f"neuron_excitability={ex}")
                else:
                    # Direct property update
                    area.properties[prop_name] = new_value
                    updated_properties.append(f"{prop_name}={new_value}")

            if updated_properties:
                try:
                    from feagi.core.state_manager import FeagiStateManager
                    if FeagiStateManager.instance().is_debug_bdu_enabled():
                        self.logger.info(
                            f"[BDU-DEBUG] Updated cortical area {cortical_id}: "
                            f"{', '.join(updated_properties)}"
                        )
                except Exception:
                    pass

                # CRITICAL DEBUG: Check if cortical_destinations survived the update
                final_has_destinations = 'cortical_destinations' in area.properties
                final_destinations_count = len(area.properties.get('cortical_destinations', {}))
                self.logger.info(f"🔄 [CONNECTOME-UPDATE] {cortical_id}: AFTER update - has_destinations={final_has_destinations}, count={final_destinations_count}")
                
                if existing_has_destinations and not final_has_destinations:
                    self.logger.error(f"💥 [CONNECTOME-UPDATE] CORRUPTION DETECTED: {cortical_id} lost cortical_destinations during update!")

                # Update StateManager cortical areas cache
                try:
                    from feagi.core.state_manager import get_state_manager

                    state_manager = get_state_manager()
                    state_manager.update_cortical_areas_cache(
                        cortical_id, "update"
                    )
                except Exception as e:
                    self.logger.warning(
                        f"Failed to update cortical areas cache after updating {cortical_id}: {e}"
                    )

            return True

        except Exception as e:
            self.logger.error(
                f"Failed to update cortical area {cortical_id} properties: {e}"
            )
            return False

    def _convert_numpy_types_to_python(self, obj: Any) -> Any:
        """Convert numpy types to native Python types for JSON serialization.

        Args:
            obj: Object that may contain numpy types

        Returns:
            Object with numpy types converted to native Python types
        """
        import numpy as np

        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {
                key: self._convert_numpy_types_to_python(value)
                for key, value in obj.items()
            }
        elif isinstance(obj, (list, tuple)):
            converted_items = [
                self._convert_numpy_types_to_python(item) for item in obj
            ]
            return type(obj)(
                converted_items
            )  # Preserve original type (list or tuple)
        else:
            return obj

    def get_all_cortical_area_properties(self) -> List[Dict[str, Any]]:
        """Get properties of all cortical areas.

        Returns:
            List of dictionaries with area properties
        """
        result = []
        for cortical_id in self.cortical_areas.keys():
            try:
                area_props = self.get_cortical_area_properties(cortical_id)
                if area_props:  # Only add non-empty dictionaries
                    result.append(area_props)
            except Exception as e:
                self.logger.error(
                    f"Error getting properties for area {cortical_id}: {e}"
                )
                continue  # Skip this area and continue with others
        return result

    def delete_cortical_area(
        self, cortical_id: str, delete_neurons: bool = True
    ) -> bool:
        """Delete a cortical area from the connectome.

        Args:
            cortical_id: ID of the cortical area
            delete_neurons: Whether to also delete all neurons in the area

        Returns:
            True if the area was deleted, False otherwise

        Raises:
            KeyError: If the cortical_id doesn't exist
        """
        if cortical_id not in self.cortical_areas:
            raise KeyError(f"Cortical area {cortical_id} does not exist")

        area = self.cortical_areas[cortical_id]

        # Get neurons in this area before deletion to avoid lookup issues
        try:
            neurons_to_delete = self.get_neurons_by_cortical_area(cortical_id)
        except Exception as e:
            logger.warning(
                f"Could not get neurons for area {cortical_id} during deletion: {e}"
            )
            neurons_to_delete = []

        # Delete neurons if requested
        if delete_neurons and neurons_to_delete:
            logger.info(
                f"Deleting {len(neurons_to_delete)} neurons from cortical area {cortical_id}"
            )
            for neuron_id in neurons_to_delete:
                try:
                    self.delete_neuron(neuron_id)
                except (KeyError, ValueError) as e:
                    # Neuron may have been already deleted or corrupted
                    from feagi.core.state_manager import get_state_manager

                    state_manager = get_state_manager()
                    if state_manager.is_debug_bdu_enabled():
                        logger.debug(
                            f"[BDU-DEBUG] Could not delete neuron {neuron_id}: {e}"
                        )
                    pass

        # Remove from any brain region using vectorized search
        region_ids = list(self.region_area_map.keys())
        area_sets = [self.region_area_map[rid] for rid in region_ids]

        # Vectorized search for regions containing this cortical area
        containing_regions = [
            rid
            for rid, area_set in zip(region_ids, area_sets)
            if cortical_id in area_set
        ]

        # Remove from all containing regions
        for region_id in containing_regions:
            self.region_area_map[region_id].discard(cortical_id)

        # Clean up bidirectional cortical mapping
        self._remove_cortical_mapping(cortical_id)

        # Remove area
        area_name = area.name
        del self.cortical_areas[cortical_id]

        # CRITICAL: Invalidate lookup arrays after bulk neuron deletion
        if delete_neurons and neurons_to_delete:
            logger.info(
                f"Deleted {len(neurons_to_delete)} neurons (neuron IDs managed by Rust NPU)"
            )

        logger.info(f"Deleted cortical area {cortical_id} ({area_name})")

        # Update StateManager cortical areas cache
        try:
            from feagi.core.state_manager import get_state_manager

            state_manager = get_state_manager()
            state_manager.update_cortical_areas_cache(cortical_id, "delete")
        except Exception as e:
            self.logger.warning(
                f"Failed to update cortical areas cache after deleting {cortical_id}: {e}"
            )

        # Trigger brain region mapping validation after deleting cortical area
        try:
            self._trigger_brain_region_validation()
        except Exception as validation_err:
            logger.warning(f"[BRAIN REGIONS] Failed to validate brain region mappings after deleting {cortical_id}: {validation_err}")

        return True

    # ----------------------------------------------------------------------
    # Brain Region Management Methods
    # ----------------------------------------------------------------------

    def add_brain_region(
        self,
        name: str,
        region_type: str = "custom",
        properties: Optional[Dict[str, Any]] = None,
        region_id: Optional[str] = None,
    ) -> str:
        """Add a new brain region to organize cortical areas.

        Args:
            name: Human-readable name for this region
            region_type: Type of brain region (e.g., "sensory", "motor", "association")
            properties: Additional properties for the region (optional)
            region_id: Unique identifier for this region (optional, generated if not provided)

        Returns:
            ID of the created brain region

        Raises:
            ValueError: If a region with the same name already exists
        """
        # Check if region with same name already exists
        for region in self.brain_regions.values():
            if region["name"] == name:
                raise ValueError(
                    f"Brain region with name '{name}' already exists"
                )

        # Generate ID if not provided
        if region_id is None:
            import uuid

            region_id = str(uuid.uuid4())

        # Create region
        self.brain_regions[region_id] = {
            "name": name,
            "region_type": region_type,
            "properties": properties or {},
        }

        # Initialize area map for this region
        self.region_area_map[region_id] = set()

        logger.info(f"Added brain region '{name}' with ID {region_id}")
        return region_id

    def get_brain_region(self, region_id: str) -> Dict[str, Any]:
        """Get information about a brain region.

        Args:
            region_id: ID of the brain region

        Returns:
            Dictionary with region properties

        Raises:
            KeyError: If the region_id doesn't exist
        """
        if region_id not in self.brain_regions:
            raise KeyError(f"Brain region {region_id} does not exist")

        return self.brain_regions[region_id].copy()

    def get_brain_region_by_name(
        self, name: str
    ) -> Optional[Tuple[str, Dict[str, Any]]]:
        """Get a brain region by name.

        Args:
            name: Name of the brain region

        Returns:
            Tuple of (region_id, region_data) if found, None otherwise
        """
        for region_id, region in self.brain_regions.items():
            if region["name"] == name:
                return (region_id, region.copy())

        return None

    def update_brain_region(
        self, region_id: str, updates: Dict[str, Any]
    ) -> bool:
        """Update properties of a brain region.

        Args:
            region_id: ID of the brain region
            updates: Dictionary of properties to update

        Returns:
            True if the region was updated, False otherwise

        Raises:
            KeyError: If the region_id doesn't exist
        """
        if region_id not in self.brain_regions:
            raise KeyError(f"Brain region {region_id} does not exist")

        region = self.brain_regions[region_id]

        # Update properties
        if "name" in updates and updates["name"] != region["name"]:
            # Check for name conflicts
            for other_id, other_region in self.brain_regions.items():
                if (
                    other_id != region_id
                    and other_region["name"] == updates["name"]
                ):
                    return False
            region["name"] = updates["name"]

        if "region_type" in updates:
            region["region_type"] = updates["region_type"]

        if "properties" in updates:
            if isinstance(updates["properties"], dict):
                region["properties"].update(updates["properties"])
            else:
                region["properties"] = updates["properties"]

        logger.info(f"Updated brain region {region_id} ({region['name']})")
        return True

    def on_cortical_mapping_created(self, source_area_id: str, target_area_id: str) -> None:
        """Handle automatic I/O designation when a cortical mapping is created.
        
        This method implements the rule:
        When connecting area A to area B:
        - If the brain region which area A belongs to is not in the ancestry tree 
          of area B, then area A will be designated as an output area for area A's 
          region and area B will be designated as an input area for area B's region.
        
        Args:
            source_area_id: Source cortical area ID
            target_area_id: Target cortical area ID
        """
        try:
            # CRITICAL FIX: Ensure brain region hierarchy is loaded before I/O designation
            # This handles the case where mappings are created but hierarchy wasn't loaded at startup
            if not self.brain_region_hierarchy.nodes:
                logger.info("🔄 Brain region hierarchy empty during mapping creation - loading from genome")
                try:
                    from feagi.core.state_manager import FeagiStateManager
                    state_manager = FeagiStateManager.instance()
                    
                    if hasattr(state_manager, 'genome') and state_manager.genome:
                        self.brain_region_hierarchy.load_from_genome(state_manager.genome)
                        logger.info(f"✅ Loaded brain region hierarchy: {len(self.brain_region_hierarchy.nodes)} regions, {len(self.brain_region_hierarchy.area_to_region)} areas")
                    else:
                        logger.warning("Cannot load brain region hierarchy: no genome in StateManager")
                        return  # Can't do I/O designation without hierarchy
                except Exception as load_error:
                    logger.error(f"Failed to load brain region hierarchy: {load_error}")
                    return  # Can't do I/O designation without hierarchy
            
            # Check if we should designate I/O based on hierarchy rules
            should_output, should_input = self.brain_region_hierarchy.should_designate_io(
                source_area_id, target_area_id
            )
            
            if not should_output and not should_input:
                logger.debug(f"No I/O designation needed for mapping {source_area_id} -> {target_area_id}")
                return
            
            # Get regions for both areas
            source_region_id = self.brain_region_hierarchy.get_region_for_area(source_area_id)
            target_region_id = self.brain_region_hierarchy.get_region_for_area(target_area_id)
            
            changes_made = False
            
            # Designate source area as output in its region
            if should_output and source_region_id:
                success = self.brain_region_hierarchy.add_output_area(source_region_id, source_area_id)
                if success:
                    logger.info(f"Designated {source_area_id} as OUTPUT in region {source_region_id}")
                    changes_made = True
            
            # Designate target area as input in its region
            if should_input and target_region_id:
                success = self.brain_region_hierarchy.add_input_area(target_region_id, target_area_id)
                if success:
                    logger.info(f"Designated {target_area_id} as INPUT in region {target_region_id}")
                    changes_made = True
            
            # Sync changes back to genome if any were made
            if changes_made:
                self._sync_hierarchy_to_genome()
                
            # Trigger full brain region mapping validation
            self._trigger_brain_region_validation()
                
        except Exception as e:
            logger.error(f"Error in automatic I/O designation for {source_area_id} -> {target_area_id}: {e}")
    
    def _sync_hierarchy_to_genome(self) -> None:
        """Sync brain region hierarchy changes back to genome."""
        try:
            # Get current genome from StateManager
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            
            if hasattr(state_manager, 'genome') and state_manager.genome:
                # Sync hierarchy changes to genome
                self.brain_region_hierarchy.sync_to_genome(state_manager.genome)
                
                # Also update local brain_regions for consistency
                if "brain_regions" in state_manager.genome:
                    self.brain_regions.update(state_manager.genome["brain_regions"])
                
                logger.debug("Synced brain region hierarchy changes to genome")
            else:
                logger.warning("Cannot sync hierarchy changes: no genome in StateManager")
                
        except Exception as e:
            logger.error(f"Error syncing hierarchy to genome: {e}")

    def _trigger_brain_region_validation(self) -> None:
        """Trigger comprehensive brain region mapping validation.
        
        This method creates a NeuroEmbryogenesis instance to run the full
        brain region mapping validation that normally happens at the end
        of brain development.
        """
        try:
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            
            if not hasattr(state_manager, 'genome') or not state_manager.genome:
                logger.warning("Cannot trigger brain region validation: no genome in StateManager")
                return
                
            # Create NeuroEmbryogenesis instance for validation
            from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
            embryo = NeuroEmbryogenesis(self, config=None)
            
            # Run the brain region mapping validation
            success = embryo._validate_and_update_brain_region_mappings()
            
            if success:
                logger.info("🧠 [BRAIN REGIONS] Triggered brain region mapping validation completed")
                
                # Reload hierarchy with updated mappings
                if hasattr(self, 'brain_region_hierarchy'):
                    self.brain_region_hierarchy.load_from_genome(state_manager.genome)
            else:
                logger.warning("Brain region mapping validation failed")
                
        except Exception as e:
            logger.error(f"Error triggering brain region validation: {e}")

    def delete_brain_region(
        self, region_id: str, delete_areas: bool = False
    ) -> bool:
        """Delete a brain region.

        Args:
            region_id: ID of the brain region
            delete_areas: Whether to also delete all areas in the region

        Returns:
            True if the region was deleted, False otherwise

        Raises:
            KeyError: If the region_id doesn't exist
        """
        if region_id not in self.brain_regions:
            raise KeyError(f"Brain region {region_id} does not exist")

        # Get areas in this region
        areas_to_delete = list(self.region_area_map.get(region_id, set()))

        # Delete areas if requested
        if delete_areas:
            for cortical_id in areas_to_delete:
                try:
                    self.delete_cortical_area(cortical_id)
                except KeyError:
                    # Area may have been already deleted
                    pass
        else:
            # Just remove the association
            for cortical_id in areas_to_delete:
                if cortical_id in self.cortical_areas:
                    if (
                        "region_id"
                        in self.cortical_areas[cortical_id].properties
                    ):
                        del self.cortical_areas[cortical_id].properties[
                            "region_id"
                        ]

        # Remove region from tracking
        region_name = self.brain_regions[region_id]["name"]
        del self.brain_regions[region_id]
        if region_id in self.region_area_map:
            del self.region_area_map[region_id]

        logger.info(f"Deleted brain region {region_id} ({region_name})")
        return True

    def assign_area_to_region(self, cortical_id: str, region_id: str) -> bool:
        """Assign a cortical area to a brain region.

        Args:
            cortical_id: ID of the cortical area
            region_id: ID of the brain region

        Returns:
            True if the area was assigned, False otherwise

        Raises:
            KeyError: If either the cortical_id or region_id doesn't exist
        """
        if cortical_id not in self.cortical_areas:
            raise KeyError(f"Cortical area {cortical_id} does not exist")

        if region_id not in self.brain_regions:
            raise KeyError(f"Brain region {region_id} does not exist")

        # Check if area is already in another region and remove if so
        area = self.cortical_areas[cortical_id]
        current_region_id = area.properties.get("region_id")
        if current_region_id and current_region_id in self.region_area_map:
            self.region_area_map[current_region_id].discard(cortical_id)

        # Assign to new region
        area.properties["region_id"] = region_id
        if region_id not in self.region_area_map:
            self.region_area_map[region_id] = set()
        self.region_area_map[region_id].add(cortical_id)

        logger.info(
            f"Assigned cortical area {cortical_id} ({area.name}) to brain region {region_id} ({self.brain_regions[region_id]['name']})"
        )
        
        return True

    def remove_cortical_area_from_region(
        self, cortical_id: str, region_id: str
    ) -> bool:
        """Remove a cortical area from a brain region.

        Args:
            cortical_id: ID of the cortical area
            region_id: ID of the brain region

        Returns:
            True if the area was removed, False otherwise

        Raises:
            KeyError: If either the cortical_id or region_id doesn't exist
        """
        if cortical_id not in self.cortical_areas:
            raise KeyError(f"Cortical area {cortical_id} does not exist")

        if region_id not in self.brain_regions:
            raise KeyError(f"Brain region {region_id} does not exist")

        # Remove from region mapping
        if region_id in self.region_area_map:
            self.region_area_map[region_id].discard(cortical_id)

        # Remove region_id from area properties
        area = self.cortical_areas[cortical_id]
        if (
            "region_id" in area.properties
            and area.properties["region_id"] == region_id
        ):
            del area.properties["region_id"]

        logger.info(
            f"Removed cortical area {cortical_id} ({area.name}) from brain region {region_id}"
        )
        
        return True

    def plan_clone_cortical_area(
        self,
        source_area_id: str,
        coordinates_3d: Optional[List[int]] = None,
        coordinates_2d: Optional[List[int]] = None,
    ) -> Dict[str, Any]:
        """Prepare a clone plan for a cortical area without performing writes.

        Uses existing property/mapping readers to avoid duplicating logic.
        """
        if source_area_id not in self.cortical_areas:
            raise KeyError(f"Cortical area {source_area_id} does not exist")

        source_props = self.get_cortical_area_properties(source_area_id)
        if not source_props:
            raise ValueError(f"Failed to retrieve properties for {source_area_id}")

        params = source_props.get("parameters", {}) or {}
        dims_tuple = tuple(source_props.get("dimensions", (1, 1, 1)))
        pos_tuple = tuple(source_props.get("coordinates", (0, 0, 0)))

        # Parent region
        parent_region_id: Optional[str] = None
        try:
            if hasattr(self, "brain_region_hierarchy") and self.brain_region_hierarchy:
                parent_region_id = self.brain_region_hierarchy.get_region_for_area(source_area_id)
        except Exception:
            parent_region_id = None
        if not parent_region_id:
            parent_region_id = params.get("parent_region_id")
        if not parent_region_id:
            raise ValueError("Cannot determine parent region for clone plan")

        # 3D coords
        if coordinates_3d is None:
            sx, sy, sz = int(pos_tuple[0]), int(pos_tuple[1]), int(pos_tuple[2])
            coordinates_3d = [sx + 1, sy, sz]
        else:
            coordinates_3d = [int(coordinates_3d[0]), int(coordinates_3d[1]), int(coordinates_3d[2])]

        # 2D coords
        if coordinates_2d is None:
            c2d = params.get("coordinates_2d")
            if not c2d:
                x2d = params.get("2dcorx")
                y2d = params.get("2dcory")
                if x2d is not None and y2d is not None:
                    c2d = [int(x2d), int(y2d)]
            if isinstance(c2d, (list, tuple)) and len(c2d) >= 2:
                coordinates_2d = [int(c2d[0]) + 1, int(c2d[1])]
            else:
                coordinates_2d = [0, 0]
        else:
            coordinates_2d = [int(coordinates_2d[0]), int(coordinates_2d[1])]

        # Derive canonical cortical_group with strict, deterministic mapping
        raw_group = (
            params.get("cortical_group")
            or params.get("group")
            or source_props.get("cortical_group")
            or source_props.get("type")
            or source_props.get("group_id")
            or ""
        )
        cortical_group = str(raw_group).upper() if raw_group else "CUSTOM"
        sub_group_id = params.get("sub_group_id") or params.get("cortical_sub_group") or source_props.get("subgroup")
        is_memory = sub_group_id == "MEMORY"

        dims_dict = {"width": int(dims_tuple[0]), "height": int(dims_tuple[1]), "depth": int(dims_tuple[2])}

        pv = None
        if not is_memory:
            pv = params.get("per_voxel_neuron_cnt") or params.get("neurons_per_voxel")
            if pv is not None:
                pv = int(pv)

        outgoing_mapping: Dict[str, Any] = {}
        try:
            if isinstance(params.get("mapping"), dict):
                outgoing_mapping = params.get("mapping")
        except Exception:
            outgoing_mapping = {}

        incoming_sources: List[Dict[str, Any]] = []
        try:
            for other_area_id in list(self.cortical_areas.keys()):
                if other_area_id == source_area_id:
                    continue
                try:
                    oprops = self.get_cortical_area_properties(other_area_id)
                    oparams = (oprops or {}).get("parameters", {}) or {}
                    omap = oparams.get("mapping", {}) or {}
                    if isinstance(omap, dict) and source_area_id in omap:
                        incoming_sources.append({
                            "src": other_area_id,
                            "connections": omap.get(source_area_id, []),
                        })
                except Exception:
                    continue
        except Exception:
            incoming_sources = []

        has_recursive = False
        try:
            has_recursive = isinstance(outgoing_mapping, dict) and source_area_id in outgoing_mapping
        except Exception:
            has_recursive = False

        return {
            "parent_region_id": parent_region_id,
            "dimensions": dims_dict,
            "coordinates_3d": coordinates_3d,
            "coordinates_2d": coordinates_2d,
            "cortical_group": cortical_group,
            "sub_group_id": sub_group_id or "CUSTOM",
            "is_memory": bool(is_memory),
            "per_voxel_neuron_cnt": pv,
            "outgoing_mapping": outgoing_mapping,
            "incoming_sources": incoming_sources,
            "has_recursive": has_recursive,
        }

    def get_areas_in_region(self, region_id: str) -> List[str]:
        """Get all cortical areas in a brain region.

        Args:
            region_id: ID of the brain region

        Returns:
            List of cortical area IDs in the region

        Raises:
            KeyError: If the region_id doesn't exist
        """
        if region_id not in self.brain_regions:
            raise KeyError(f"Brain region {region_id} does not exist")

        return list(self.region_area_map.get(region_id, set()))

    def get_neurons_in_region(self, region_id: str) -> List[int]:
        """Get all neurons in a brain region.

        Args:
            region_id: ID of the brain region

        Returns:
            List of neuron IDs in the region

        Raises:
            KeyError: If the region_id doesn't exist
        """
        if region_id not in self.brain_regions:
            raise KeyError(f"Brain region {region_id} does not exist")

        neuron_ids = []
        for cortical_id in self.region_area_map.get(region_id, set()):
            neuron_ids.extend(self.get_neurons_by_cortical_area(cortical_id))

        return neuron_ids

    # Backward compatibility aliases
    def remove_area_from_region(
        self, cortical_id: str, region_id: str
    ) -> bool:
        """Alias for remove_cortical_area_from_region for backward
        compatibility."""
        return self.remove_cortical_area_from_region(cortical_id, region_id)

    # Property to maintain backward compatibility with the existing API
    @property
    def _neuron_id_to_index(self):
        return self.neuron_id_to_index

    def update_neuron_position(
        self, neuron_id: int, new_position: Tuple[int, int, int]
    ) -> bool:
        """NOT IMPLEMENTED: Dynamic neuron position updates not yet supported in Rust NPU.

        Args:
            neuron_id: ID of the neuron
            new_position: New 3D coordinates within the cortical area (x, y, z)

        Returns:
            False (not implemented)

        Raises:
            NotImplementedError: Position updates require Rust NPU implementation
        """
        raise NotImplementedError(
            "Dynamic neuron position updates not yet supported in Rust NPU. "
            "Requires implementation in Rust ConnectomeManager."
        )

    def batch_create_neurons(
        self,
        cortical_id: str,
        positions: List[Tuple[int, int, int]],
        threshold: float,
        membrane_potential: float,
        resting_potential: float,
        leak_coefficient: float,
        leak_variability: float = 0.0,
        refractory_period: int = 1,
        excitability: float = 1.0,
        consecutive_fire_limit: int = 10,
        snooze_period: int = 0,
        properties: Optional[Dict[str, Any]] = None,
        cortical_idx: Optional[int] = None,
    ) -> List[int]:
        """Create multiple neurons in a cortical area in a batch operation.

        This is more efficient than creating neurons one by one, especially for large batches.

        Args:
            cortical_id: ID of the cortical area
            positions: List of 3D coordinates (x, y, z) for each neuron
            threshold: Firing threshold potential (can be a single value or a list)
            membrane_potential: Initial membrane potential (can be a single value or a list)
            resting_potential: Base membrane potential (can be a single value or a list)
            leak_coefficient: Leak coefficient 0.0-1.0 (can be a single value or a list)
            leak_variability: Standard deviation for leak variation during neurogenesis (0.0-1.0)
            refractory_period: Number of timesteps after firing during which neurons cannot fire (can be a single value or a list)
            excitability: Neuron excitability 0.0-1.0 (can be a single value or a list)
            consecutive_fire_limit: Maximum consecutive fires before extended refractory (can be a single value or a list)
            snooze_period: Extended refractory period after hitting consecutive fire limit (can be a single value or a list)
            properties: Additional properties for the neurons (optional)
            cortical_idx: Integer index of the cortical area (optional, will be determined from cortical_id if not provided)

        Returns:
            List of neuron IDs for the created neurons

        Raises:
            ValueError: If the cortical_id doesn't exist
            ValueError: If any position is outside the area's boundaries
            ValueError: If there are duplicate positions
        """
        # Validate area exists
        if cortical_id not in self.cortical_areas:
            raise ValueError(f"Cortical area {cortical_id} does not exist")

        area = self.cortical_areas[cortical_id]

        # CRITICAL FIX: Handle empty positions list 
        if not positions:
            self.logger.warning(
                f"batch_create_neurons called with empty positions list for {cortical_id}. "
                f"This typically indicates an expansion logic error."
            )
            return []  # Return empty list instead of causing indexing errors

        # Validate positions
        for pos in positions:
            if not area.contains_position(pos):
                raise ValueError(
                    f"Position {pos} is outside the bounds of area {area.name}"
                )

        # FEAGI DESIGN: Multiple neurons can share the same position (voxel)
        # This is essential for neuron density > 1 and memory areas
        # Removed duplicate position validation as it's incorrect for FEAGI's architecture

        # Use the provided cortical_idx or get it from the area
        if cortical_idx is None:
            cortical_idx = area.cortical_idx

        # ARCHITECTURE COMPLIANCE: Delegate neuron creation to Rust NPU through NPU interface
        # ConnectomeManager prepares the request, Rust NPU manages IDs and storage
        count = len(positions)

        # Normalize per-neuron lists - ALL PARAMETERS FROM GENOME
        thresholds_list = (
            [threshold] * count if isinstance(threshold, (int, float)) else list(threshold)
        )
        mp_list = (
            [membrane_potential] * count if isinstance(membrane_potential, (int, float)) else list(membrane_potential)
        )
        rp_list = (
            [resting_potential] * count if isinstance(resting_potential, (int, float)) else list(resting_potential)
        )
        
        # Apply leak_variability to introduce per-neuron variation (neurogenesis only)
        import numpy as np
        if leak_variability > 0.0:
            # Generate per-neuron leak coefficients with Gaussian variation
            leak_list = np.clip(
                np.random.normal(leak_coefficient, leak_variability, count),
                0.0,  # Minimum: no leak
                1.0   # Maximum: full leak
            ).tolist()
        else:
            leak_list = (
                [leak_coefficient] * count if isinstance(leak_coefficient, (int, float)) else list(leak_coefficient)
            )
        
        refr_list = (
            [refractory_period] * count if isinstance(refractory_period, int) else list(refractory_period)
        )
        excitability_list = (
            [excitability] * count if isinstance(excitability, (int, float)) else list(excitability)
        )
        consecutive_fire_limits_list = (
            [consecutive_fire_limit] * count if isinstance(consecutive_fire_limit, int) else list(consecutive_fire_limit)
        )
        snooze_periods_list = (
            [snooze_period] * count if isinstance(snooze_period, int) else list(snooze_period)
        )

        # Delegate to Rust NPU through NPU interface
        # Rust NPU manages neuron IDs, positions, and all neuron data
        from feagi.npu.interface import NeuronCreationRequest
        
        request = NeuronCreationRequest(
            cortical_idx=cortical_idx,
            positions=positions,
            neuron_types=[0] * count,
            initial_potentials=mp_list,
            thresholds=thresholds_list,
            leak_coefficients=leak_list,
            refractory_periods=refr_list,
            excitabilities=excitability_list,
            resting_potentials=rp_list,
            consecutive_fire_limits=consecutive_fire_limits_list,
            snooze_periods=snooze_periods_list
        )
        
        result = self._npu_interface.create_neurons_batch(request)
        
        from feagi.npu.interface import OperationResult
        if result.result != OperationResult.SUCCESS:
            logger.error(f"Failed to create neurons in Rust NPU: {result.error_message}")
            return []
        
        # Get neuron IDs assigned by Rust NPU
        neuron_ids = result.data.get("neuron_ids", []) if result.data else []
        
        if not neuron_ids:
            logger.error("Rust NPU did not return neuron IDs")
            return []

        # ARCHITECTURE: All neuron data now in Rust NPU (single source of truth)
        # Python CorticalArea tracking removed - it was redundant and caused 4s delays
        # Rust NPU owns: neuron IDs, positions, cortical areas, all properties
        # Python CorticalArea object is legacy - only keeping for genome compatibility
        
        # Update state manager with new neuron count
        if len(neuron_ids) > 0:
            self._update_neuron_count_only()
        
        return neuron_ids

    def batch_update_neuron_properties(
        self,
        neuron_ids: List[int],
        property_name: Union[str, NeuronPropertyType],
        values: Union[List[float], List[int], float, int],
    ) -> bool:
        """Update a property for multiple neurons at once in a vectorized
        operation.

        Args:
            neuron_ids: List of neuron IDs to update
            property_name: Name or enum of the property to update
            values: Either a list of values (one per neuron) or a single value for all neurons

        Returns:
            True if successful, False otherwise

        Raises:
            ValueError: If neuron IDs are invalid or property doesn't exist
        """
        # Convert string property name to enum if needed
        if isinstance(property_name, str):
            try:
                property_name = NeuronPropertyType(property_name)
            except ValueError as err:
                raise ValueError(
                    f"Unknown neuron property: {property_name}"
                ) from err

        # Check for unsupported properties early (before value validation)
        if property_name == NeuronPropertyType.POSITION:
            raise NotImplementedError(
                "Batch update of neuron positions is not supported. "
                "Position cannot be changed after neuron creation as per user requirements."
            )

        # Validate neuron IDs via NPU interface (neuron_id == index in Rust NPU)
        valid_mask = np.zeros(len(neuron_ids), dtype=bool)
        for i, neuron_id in enumerate(neuron_ids):
            if self._npu_interface.neuron_exists(neuron_id):
                valid_mask[i] = True

        if not np.any(valid_mask):
            logger.warning(
                f"None of the provided neuron IDs exist: {neuron_ids}"
            )
            return False

        # Get indices for valid neuron IDs (identity mapping: neuron_id == index)
        valid_ids = np.array(neuron_ids)[valid_mask]
        indices = valid_ids  # In Rust NPU, neuron_id == index

        # Handle single value vs. list of values
        if isinstance(values, (int, float)):
            # Single value for all neurons
            update_values = np.full(len(indices), values)
        else:
            # List of values (one per neuron)
            if len(values) != len(neuron_ids):
                raise ValueError(
                    f"Length of values ({len(values)}) must match length of neuron_ids ({len(neuron_ids)})"
                )
            update_values = np.array(values)[valid_mask]

        # Call appropriate Rust NPU batch update function via NPUInterface
        valid_ids_list = valid_ids.tolist()
        
        # Access Rust NPU through NPUInterface
        rust_npu = self._npu_interface.rust_npu
        
        try:
            if property_name == NeuronPropertyType.REFRACTORY_PERIOD:
                # Round before converting to int (7.9 → 8, not 7)
                values_u16 = np.round(update_values).astype(np.uint16).tolist()
                updated_count = rust_npu.batch_update_refractory_period(valid_ids_list, values_u16)
            elif property_name == NeuronPropertyType.THRESHOLD:
                values_f32 = update_values.astype(np.float32).tolist()
                updated_count = rust_npu.batch_update_threshold(valid_ids_list, values_f32)
            elif property_name == NeuronPropertyType.LEAK_COEFFICIENT:
                values_f32 = update_values.astype(np.float32).tolist()
                updated_count = rust_npu.batch_update_leak_coefficient(valid_ids_list, values_f32)
            elif property_name == NeuronPropertyType.MEMBRANE_POTENTIAL:
                values_f32 = update_values.astype(np.float32).tolist()
                updated_count = rust_npu.batch_update_membrane_potential(valid_ids_list, values_f32)
            elif property_name == NeuronPropertyType.RESTING_POTENTIAL:
                values_f32 = update_values.astype(np.float32).tolist()
                updated_count = rust_npu.batch_update_resting_potential(valid_ids_list, values_f32)
            elif property_name == NeuronPropertyType.SNOOZE_PERIOD:
                # Snooze period (extended refractory) - stored as u16
                values_u16 = np.round(update_values).astype(np.uint16).tolist()
                updated_count = rust_npu.batch_update_snooze_period(valid_ids_list, values_u16)
            else:
                logger.warning(
                    f"⚠️ Property {property_name} batch update not yet implemented in Rust NPU"
                )
                raise NotImplementedError(
                    f"Batch neuron property updates for {property_name} not yet supported in Rust NPU."
                )
                
            logger.info(f"✅ Batch updated {updated_count} neurons for property {property_name}")
            return updated_count > 0
            
        except Exception as e:
            logger.error(f"Failed to batch update {property_name}: {e}")
            raise

    def batch_get_neuron_properties(
        self,
        neuron_ids: List[int],
        property_name: Union[str, NeuronPropertyType],
    ) -> np.ndarray:
        """Get a property for multiple neurons at once.

        Args:
            neuron_ids: List of neuron IDs to query
            property_name: Name or enum of the property to get

        Returns:
            NumPy array of property values for the specified neurons

        Raises:
            ValueError: If property doesn't exist
        """
        # Convert string property name to enum if needed
        if isinstance(property_name, str):
            try:
                property_name = NeuronPropertyType(property_name)
            except ValueError as err:
                raise ValueError(
                    f"Unknown neuron property: {property_name}"
                ) from err

        # Handle empty list
        if not neuron_ids:
            return np.array([])

        # Get indices for valid neuron IDs, with -1 for invalid IDs (neuron_id == index in Rust NPU)
        indices = np.array(
            [nid if self._npu_interface.neuron_exists(nid) else -1 for nid in neuron_ids]
        )
        valid_mask = indices >= 0

        # Initialize result with NaN for invalid indices
        result = np.full(len(neuron_ids), np.nan)

        # Query properties from Rust NPU one by one
        # TODO: Optimize with batched getter in Rust
        rust_npu = self._npu_interface.rust_npu
        
        for i, neuron_id in enumerate(neuron_ids):
            if not valid_mask[i]:
                continue  # Skip invalid neurons
            
            try:
                value = None
                if property_name == NeuronPropertyType.REFRACTORY_PERIOD:
                    value = rust_npu.get_neuron_refractory_period(neuron_id)
                elif property_name == NeuronPropertyType.THRESHOLD:
                    value = rust_npu.get_neuron_threshold(neuron_id)
                elif property_name == NeuronPropertyType.LEAK_COEFFICIENT:
                    value = rust_npu.get_neuron_leak_coefficient(neuron_id)
                elif property_name == NeuronPropertyType.MEMBRANE_POTENTIAL:
                    value = rust_npu.get_neuron_membrane_potential(neuron_id)
                elif property_name == NeuronPropertyType.RESTING_POTENTIAL:
                    value = rust_npu.get_neuron_resting_potential(neuron_id)
                elif property_name == NeuronPropertyType.EXCITABILITY:
                    value = rust_npu.get_neuron_excitability(neuron_id)
                elif property_name == NeuronPropertyType.CONSECUTIVE_FIRE_LIMIT:
                    value = rust_npu.get_neuron_consecutive_fire_limit(neuron_id)
                else:
                    logger.warning(f"⚠️ Unsupported property for batch get: {property_name}")
                
                # Handle Optional (None means neuron not found or invalid)
                if value is not None:
                    result[i] = value
            except AttributeError as e:
                logger.warning(f"⚠️ Rust NPU getter not available for {property_name}: {e}")
            except Exception as e:
                logger.warning(f"⚠️ Failed to get property {property_name} for neuron {neuron_id}: {e}")

        return result

    def batch_add_synapses(
        self,
        pre_neurons: List[int],
        post_neurons: List[int],
        weights: Union[List[float], float],
        delays: Union[List[int], int] = 1,
    ) -> List[bool]:
        """Add multiple synapses at once.

        Args:
            pre_neurons: List of pre-synaptic neuron IDs
            post_neurons: List of post-synaptic neuron IDs
            weights: Either a list of weights (one per synapse) or a single weight for all synapses
            delays: Either a list of delays (one per synapse) or a single delay for all synapses

        Returns:
            List of booleans indicating which synapses were successfully added
        """
        if len(pre_neurons) != len(post_neurons):
            raise ValueError(
                f"pre_neurons ({len(pre_neurons)}) and post_neurons ({len(post_neurons)}) must have the same length"
            )

        # Handle single weight vs. list of weights
        if isinstance(weights, (int, float)):
            weights = [weights] * len(pre_neurons)
        elif len(weights) != len(pre_neurons):
            raise ValueError(
                f"weights ({len(weights)}) must have the same length as pre_neurons ({len(pre_neurons)})"
            )

        # Handle single delay vs. list of delays
        if isinstance(delays, int):
            delays = [delays] * len(pre_neurons)
        elif len(delays) != len(pre_neurons):
            raise ValueError(
                f"delays ({len(delays)}) must have the same length as pre_neurons ({len(pre_neurons)})"
            )

        # Add each synapse and track success
        results = []
        for pre, post, weight, delay in zip(
            pre_neurons, post_neurons, weights, delays
        ):
            results.append(self.add_synapse(pre, post, weight, delay))

        # Force re-indexing of CSR matrices
        self._csr_matrix_outdated = True

        return results

    def vectorized_cortical_area_operations(
        self, operation: str, cortical_ids: List[str], **kwargs
    ) -> Dict[str, Any]:
        """Perform vectorized operations on multiple cortical areas
        efficiently.

        Args:
            operation: Type of operation ('count_neurons', 'get_activity', 'update_properties', etc.)
            cortical_ids: List of cortical area IDs to operate on
            **kwargs: Operation-specific parameters

        Returns:
            Dictionary mapping cortical_id -> operation result
        """
        results = {}

        if operation == "count_neurons":
            # Vectorized neuron counting
            for cortical_id in cortical_ids:
                if cortical_id in self.area_neuron_masks:
                    # Use efficient numpy sum on boolean mask
                    neuron_count = np.sum(self.area_neuron_masks[cortical_id])
                    results[cortical_id] = int(neuron_count)
                else:
                    results[cortical_id] = 0

        elif operation == "resize":
            # Resize multiple areas at once
            new_dimensions = kwargs.get("dimensions")
            if not new_dimensions:
                raise ValueError(
                    "New dimensions required for resize operation"
                )

            for cortical_id in cortical_ids:
                if cortical_id not in self.cortical_areas:
                    results[cortical_id] = {
                        "success": False,
                        "reason": "Area not found",
                    }
                    continue

                area = self.cortical_areas[cortical_id]
                old_dimensions = area.dimensions

                # Update area dimensions
                area.dimensions = new_dimensions

                # ✅ Use Rust NPU to find neurons outside new bounds
                cortical_idx = self.cortical_mapping.get_index(cortical_id)
                if cortical_idx is not None:
                    # Get all neurons in this cortical area from Rust NPU
                    neuron_positions = self._npu_interface.rust_npu.get_neuron_positions_in_cortical_area(cortical_idx)
                    
                    removed_neuron_ids = []
                    for neuron_id, x, y, z in neuron_positions:
                        if (
                            x >= new_dimensions[0]
                            or y >= new_dimensions[1]
                            or z >= new_dimensions[2]
                        ):
                            # This neuron is now outside bounds - delete it
                            self.delete_neuron(neuron_id)
                            removed_neuron_ids.append(neuron_id)
                else:
                    removed_neuron_ids = []

                results[cortical_id] = {
                    "success": True,
                    "old_dimensions": old_dimensions,
                    "new_dimensions": new_dimensions,
                    "removed_neurons": removed_neuron_ids,
                }

        elif operation == "move":
            # Move multiple areas at once
            new_position = kwargs.get("position")
            if not new_position:
                raise ValueError("New position required for move operation")

            for cortical_id in cortical_ids:
                if cortical_id not in self.cortical_areas:
                    results[cortical_id] = {
                        "success": False,
                        "reason": "Area not found",
                    }
                    continue

                area = self.cortical_areas[cortical_id]
                old_position = area.position

                # Update area position
                area.position = new_position

                results[cortical_id] = {
                    "success": True,
                    "old_position": old_position,
                    "new_position": new_position,
                }

        elif operation == "get_bounds":
            # Get position bounds for multiple areas at once
            for cortical_id in cortical_ids:
                if cortical_id not in self.cortical_areas:
                    results[cortical_id] = {
                        "success": False,
                        "reason": "Area not found",
                    }
                    continue

                area = self.cortical_areas[cortical_id]

                # Calculate bounds
                min_pos = area.position
                max_pos = tuple(
                    p + d for p, d in zip(area.position, area.dimensions)
                )

                results[cortical_id] = {
                    "success": True,
                    "min_bounds": min_pos,
                    "max_bounds": max_pos,
                    "dimensions": area.dimensions,
                }

        else:
            raise ValueError(f"Unsupported operation: {operation}")

        return results

    def apply_rule_batch(
        self,
        rule_ids: List[str],
        weight_override: Optional[float] = None,
        max_synapses: int = 1_000_000,  # Increased from 10,000 to 1M for large cortical areas
    ) -> Dict[str, int]:
        """Apply multiple connectivity rules at once using vectorized
        operations.

        Args:
            rule_ids: List of connectivity rule IDs to apply
            weight_override: Override the weight specified in the rules (optional)
            max_synapses: Maximum number of synapses to create (prevents excessive connections)

        Returns:
            Dictionary mapping rule IDs to number of synapses created

        Raises:
            KeyError: If any rule_id doesn't exist
        """
        results = {}

        # Group rules by type for vectorized processing
        rules_by_type = {}
        for rule_id in rule_ids:
            if rule_id not in self.connectivity_rules:
                raise KeyError(f"Connectivity rule {rule_id} does not exist")

            rule = self.connectivity_rules[rule_id]
            if not rule["enabled"]:
                results[rule_id] = 0
                continue

            rule_type = rule["rule_type"]
            if rule_type not in rules_by_type:
                rules_by_type[rule_type] = []

            rules_by_type[rule_type].append((rule_id, rule))

        # Process rules by type using specialized vectorized implementations
        for rule_type, rules in rules_by_type.items():
            if rule_type == "one-to-one":
                results.update(
                    self._apply_one_to_one_rules_batch(
                        rules, weight_override, max_synapses
                    )
                )
            elif rule_type == "all-to-all":
                results.update(
                    self._apply_all_to_all_rules_batch(
                        rules, weight_override, max_synapses
                    )
                )
            elif rule_type == "probabilistic":
                results.update(
                    self._apply_probabilistic_rules_batch(
                        rules, weight_override, max_synapses
                    )
                )
            elif rule_type == "distance":
                results.update(
                    self._apply_distance_rules_batch(
                        rules, weight_override, max_synapses
                    )
                )
            elif rule_type == "random-subset":
                results.update(
                    self._apply_random_subset_rules_batch(
                        rules, weight_override, max_synapses
                    )
                )
            else:
                # Fallback to individual application
                for rule_id, _rule in rules:
                    created_count = self.apply_connectivity_rule(
                        rule_id, weight_override, max_synapses
                    )
                    results[rule_id] = created_count

        return results

    def _apply_one_to_one_rules_batch(
        self, rules, weight_override, max_synapses
    ):
        """Apply one-to-one rules in batch."""
        results = {}

        for rule_id, rule in rules:
            source_cortical_id = rule["source_cortical_id"]
            target_cortical_id = rule["target_cortical_id"]

            # Get neurons for both areas
            source_neurons = self.get_neurons_by_cortical_area(
                source_cortical_id
            )
            target_neurons = self.get_neurons_by_cortical_area(
                target_cortical_id
            )

            if not source_neurons or not target_neurons:
                results[rule_id] = 0
                continue

            # Check dimensions
            source_area = self.cortical_areas[source_cortical_id]
            target_area = self.cortical_areas[target_cortical_id]

            if source_area.dimensions != target_area.dimensions:
                results[rule_id] = 0
                continue

            # Determine weight
            weight = (
                weight_override
                if weight_override is not None
                else rule["parameters"].get("weight", 1.0)
            )

            # Prepare synapse specs
            synapse_specs = []
            max_connections = min(
                max_synapses, min(len(source_neurons), len(target_neurons))
            )

            # ✅ Sort neurons by position for consistent mapping (using Rust NPU)
            source_positions = []
            for neuron_id in source_neurons[:max_connections]:
                pos = self.get_neuron_position(neuron_id)  # Use existing method that queries Rust NPU
                source_positions.append((neuron_id, pos))

            target_positions = []
            for neuron_id in target_neurons[:max_connections]:
                pos = self.get_neuron_position(neuron_id)  # Use existing method that queries Rust NPU
                target_positions.append((neuron_id, pos))

            # Sort by position
            source_positions.sort(key=lambda x: (x[1][0], x[1][1], x[1][2]))
            target_positions.sort(key=lambda x: (x[1][0], x[1][1], x[1][2]))

            # Create connections
            for i in range(min(len(source_positions), len(target_positions))):
                source_id = source_positions[i][0]
                target_id = target_positions[i][0]
                synapse_specs.append((source_id, target_id, weight))

            # Create synapses in batch
            created_count = self.batch_create_synapses(synapse_specs)
            results[rule_id] = created_count

        return results

    def _apply_all_to_all_rules_batch(
        self, rules, weight_override, max_synapses
    ):
        """Apply all-to-all rules in batch."""
        results = {}

        for rule_id, rule in rules:
            source_cortical_id = rule["source_cortical_id"]
            target_cortical_id = rule["target_cortical_id"]

            # Get neurons for both areas
            source_neurons = self.get_neurons_by_cortical_area(
                source_cortical_id
            )
            target_neurons = self.get_neurons_by_cortical_area(
                target_cortical_id
            )

            if not source_neurons or not target_neurons:
                results[rule_id] = 0
                continue

            # Determine weight
            weight = (
                weight_override
                if weight_override is not None
                else rule["parameters"].get("weight", 1.0)
            )

            # Calculate total possible connections
            total_possible = len(source_neurons) * len(target_neurons)

            # Check if we need to sample
            if total_possible <= max_synapses:
                # Create all connections
                synapse_specs = []
                for source_id in source_neurons:
                    for target_id in target_neurons:
                        synapse_specs.append((source_id, target_id, weight))
                        if len(synapse_specs) >= max_synapses:
                            break
                    if len(synapse_specs) >= max_synapses:
                        break
            else:
                # Sample connections
                sample_count = min(max_synapses, total_possible)
                source_ids = np.random.choice(
                    source_neurons, size=sample_count, replace=True
                )
                target_ids = np.random.choice(
                    target_neurons, size=sample_count, replace=True
                )

                synapse_specs = [
                    (source_ids[i], target_ids[i], weight)
                    for i in range(sample_count)
                ]

            # Create synapses in batch
            created_count = self.batch_create_synapses(synapse_specs)
            results[rule_id] = created_count

        return results

    def _apply_probabilistic_rules_batch(
        self, rules, weight_override, max_synapses
    ):
        """Apply probabilistic rules in batch."""
        results = {}

        for rule_id, rule in rules:
            source_cortical_id = rule["source_cortical_id"]
            target_cortical_id = rule["target_cortical_id"]

            # Get neurons for both areas
            source_neurons = self.get_neurons_by_cortical_area(
                source_cortical_id
            )
            target_neurons = self.get_neurons_by_cortical_area(
                target_cortical_id
            )

            if not source_neurons or not target_neurons:
                results[rule_id] = 0
                continue

            # Determine weight and probability
            weight = (
                weight_override
                if weight_override is not None
                else rule["parameters"].get("weight", 1.0)
            )
            probability = rule["parameters"].get("probability", 0.1)

            # Calculate total possible and expected connections
            total_possible = len(source_neurons) * len(target_neurons)
            expected_count = int(total_possible * probability)
            actual_count = min(expected_count, max_synapses)

            # Sample connections
            if actual_count > 0:
                source_ids = np.random.choice(
                    source_neurons, size=actual_count, replace=True
                )
                target_ids = np.random.choice(
                    target_neurons, size=actual_count, replace=True
                )

                synapse_specs = [
                    (source_ids[i], target_ids[i], weight)
                    for i in range(actual_count)
                ]

                # Create synapses in batch
                created_count = self.batch_create_synapses(synapse_specs)
                results[rule_id] = created_count
            else:
                results[rule_id] = 0

        return results

    def _apply_distance_rules_batch(
        self, rules, weight_override, max_synapses
    ):
        """Apply distance-based rules in batch."""
        results = {}

        for rule_id, rule in rules:
            source_cortical_id = rule["source_cortical_id"]
            target_cortical_id = rule["target_cortical_id"]

            # Get neurons for both areas
            source_neurons = self.get_neurons_by_cortical_area(
                source_cortical_id
            )
            target_neurons = self.get_neurons_by_cortical_area(
                target_cortical_id
            )

            if not source_neurons or not target_neurons:
                results[rule_id] = 0
                continue

            # Determine weight and max distance
            weight = (
                weight_override
                if weight_override is not None
                else rule["parameters"].get("weight", 1.0)
            )
            max_distance = rule["parameters"].get("max_distance", 5.0)
            scale_by_distance = rule["parameters"].get(
                "scale_by_distance", False
            )

            # Get areas
            source_area = self.cortical_areas[source_cortical_id]
            target_area = self.cortical_areas[target_cortical_id]

            # ✅ Get positions in global coordinates (using Rust NPU)
            source_global_positions = {}
            for neuron_id in source_neurons:
                local_pos = self.get_neuron_position(neuron_id)  # Use existing method that queries Rust NPU
                global_pos = tuple(
                    lp + ap for lp, ap in zip(local_pos, source_area.position)
                )
                source_global_positions[neuron_id] = global_pos

            target_global_positions = {}
            for neuron_id in target_neurons:
                local_pos = self.get_neuron_position(neuron_id)  # Use existing method that queries Rust NPU
                global_pos = tuple(
                    lp + ap for lp, ap in zip(local_pos, target_area.position)
                )
                target_global_positions[neuron_id] = global_pos

            # Calculate distances using vectorized operations for efficiency
            total_possible = len(source_neurons) * len(target_neurons)

            if (
                total_possible > 100000
            ):  # Switch to sampling for large networks
                # Sample random pairs and check distances
                candidates = 0
                max_candidates = min(100000, total_possible)
                synapse_specs = []

                while (
                    len(synapse_specs) < max_synapses
                    and candidates < max_candidates
                ):
                    source_id = np.random.choice(source_neurons)
                    target_id = np.random.choice(target_neurons)

                    source_pos = source_global_positions[source_id]
                    target_pos = target_global_positions[target_id]

                    # Calculate Euclidean distance
                    distance = np.sqrt(
                        sum(
                            (a - b) ** 2
                            for a, b in zip(source_pos, target_pos)
                        )
                    )

                    if distance <= max_distance:
                        if scale_by_distance:
                            distance_weight = 1.0 - (distance / max_distance)
                            synapse_specs.append(
                                (
                                    source_id,
                                    target_id,
                                    weight * distance_weight,
                                )
                            )
                        else:
                            synapse_specs.append(
                                (source_id, target_id, weight)
                            )

                    candidates += 1
            else:
                # Vectorized distance calculation for smaller networks
                source_ids = np.array(list(source_neurons))
                target_ids = np.array(list(target_neurons))

                # Convert positions to arrays for vectorized operations
                source_pos_array = np.array(
                    [source_global_positions[nid] for nid in source_ids]
                )
                target_pos_array = np.array(
                    [target_global_positions[nid] for nid in target_ids]
                )

                # Limit to manageable subsets if needed
                max_sources = min(1000, len(source_ids))
                max_targets = min(1000, len(target_ids))

                source_pos_array = source_pos_array[:max_sources]
                source_ids = source_ids[:max_sources]
                target_pos_array = target_pos_array[:max_targets]
                target_ids = target_ids[:max_targets]

                # Calculate distances (still O(n²) but vectorized)
                synapse_specs = []

                for _i, (source_id, source_pos) in enumerate(
                    zip(source_ids, source_pos_array)
                ):
                    # Calculate distances to all targets at once
                    distances = np.sqrt(
                        np.sum((target_pos_array - source_pos) ** 2, axis=1)
                    )

                    # Find targets within max distance
                    within_distance = distances <= max_distance
                    valid_targets = target_ids[within_distance]
                    valid_distances = distances[within_distance]

                    # Add connections
                    for _j, (target_id, distance) in enumerate(
                        zip(valid_targets, valid_distances)
                    ):
                        if scale_by_distance:
                            distance_weight = 1.0 - (distance / max_distance)
                            synapse_specs.append(
                                (
                                    source_id,
                                    target_id,
                                    weight * distance_weight,
                                )
                            )
                        else:
                            synapse_specs.append(
                                (source_id, target_id, weight)
                            )

                        if len(synapse_specs) >= max_synapses:
                            break

                    if len(synapse_specs) >= max_synapses:
                        break

            # Create synapses in batch
            created_count = self.batch_create_synapses(synapse_specs)
            results[rule_id] = created_count

        return results

    def _apply_random_subset_rules_batch(
        self, rules, weight_override, max_synapses
    ):
        """Apply random-subset rules in batch."""
        results = {}

        for rule_id, rule in rules:
            source_cortical_id = rule["source_cortical_id"]
            target_cortical_id = rule["target_cortical_id"]

            # Get neurons for both areas
            source_neurons = self.get_neurons_by_cortical_area(
                source_cortical_id
            )
            target_neurons = self.get_neurons_by_cortical_area(
                target_cortical_id
            )

            if not source_neurons or not target_neurons:
                results[rule_id] = 0
                continue

            # Determine weight and number of targets
            weight = (
                weight_override
                if weight_override is not None
                else rule["parameters"].get("weight", 1.0)
            )
            num_targets = min(
                rule["parameters"].get("num_targets", 5), len(target_neurons)
            )

            # Limit source neurons to avoid excessive computation
            max_sources = min(len(source_neurons), max_synapses // num_targets)

            # Prepare all synapse specs at once
            synapse_specs = []

            # For each source neuron, randomly select target neurons
            for source_id in source_neurons[:max_sources]:
                targets = np.random.choice(
                    target_neurons, num_targets, replace=False
                )

                for target_id in targets:
                    synapse_specs.append((source_id, target_id, weight))

                if len(synapse_specs) >= max_synapses:
                    break

            # Create synapses in batch
            created_count = self.batch_create_synapses(synapse_specs)
            results[rule_id] = created_count

        return results

    def add_neuron(
        self,
        cortical_id: Optional[str] = None,
        position: Optional[Tuple[int, int, int]] = None,
        threshold: float = 1.0,
        membrane_potential: float = 0.0,
        resting_potential: float = 0.0,
        leak_coefficient: float = 0.0,
        refractory_period: int = 1,
        properties: Optional[Dict[str, Any]] = None,
    ) -> int:
        """Create a new neuron and add it to the network.

        This is an alias for create_neuron to maintain compatibility with tests.

        Args:
            cortical_id: ID of the cortical area this neuron belongs to
            position: 3D coordinates (x, y, z)
            threshold: Firing threshold
            membrane_potential: Initial membrane potential
            resting_potential: Resting potential
            leak_coefficient: Leak coefficient (0.0-1.0, percentage of potential lost per burst)
            refractory_period: Refractory period in timesteps
            properties: Additional properties to set

        Returns:
            ID of the created neuron
        """
        if position is None:
            position = (0, 0, 0)

        #  For test compatibility, create a temporary cortical area if none is
        #  provided
        if cortical_id is None:
            # Create a test cortical area if it doesn't exist
            test_cortical_id = "TEST__"
            if test_cortical_id not in self.cortical_areas:
                self.add_cortical_area(
                    name="Test Area",
                    dimensions=(100, 100, 100),
                    position=(0, 0, 0),
                    area_type="test",
                    cortical_id=test_cortical_id,
                )
            cortical_id = test_cortical_id

        return self.create_neuron(
            cortical_id=cortical_id,
            position=position,
            threshold=threshold,
            membrane_potential=membrane_potential,
            resting_potential=resting_potential,
            leak_coefficient=leak_coefficient,
            refractory_period=refractory_period,
            properties=properties,
        )

    def add_neurons(
        self,
        count: int,
        cortical_id: Optional[str] = None,
        position: Optional[Tuple[int, int, int]] = None,
        threshold: float = 1.0,
        membrane_potential: float = 0.0,
        resting_potential: float = 0.0,
        leak_coefficient: float = 0.0,
        refractory_period: int = 1,
        properties: Optional[Dict[str, Any]] = None,
    ) -> List[int]:
        """Create multiple neurons with the same properties.

        Args:
            count: Number of neurons to create
            cortical_id: ID of the cortical area these neurons belong to
            position: Base 3D coordinates (x, y, z) - if provided, neurons will be placed sequentially from this position
            threshold: Firing threshold
            membrane_potential: Initial membrane potential
            resting_potential: Resting potential
            leak_coefficient: Leak coefficient (0.0-1.0, percentage of potential lost per burst)
            refractory_period: Refractory period in timesteps
            properties: Additional properties to set

        Returns:
            List of IDs of the created neurons
        """
        if position is None:
            position = (0, 0, 0)

        #  For test compatibility, create a temporary cortical area if none is
        #  provided
        if cortical_id is None:
            # Create a test cortical area if it doesn't exist
            test_cortical_id = "TEST__"
            if test_cortical_id not in self.cortical_areas:
                self.add_cortical_area(
                    name="Test Area",
                    dimensions=(100, 100, 100),
                    position=(0, 0, 0),
                    area_type="test",
                    cortical_id=test_cortical_id,
                )
            cortical_id = test_cortical_id

        # Generate sequential positions if base position is provided
        positions = []
        x, y, z = position
        for i in range(count):
            positions.append((x + i, y, z))

        return self.batch_create_neurons(
            cortical_id=cortical_id,
            positions=positions,
            threshold=threshold,
            membrane_potential=membrane_potential,
            resting_potential=resting_potential,
            leak_coefficient=leak_coefficient,
            refractory_period=refractory_period,
            excitability=1.0,  # Default excitability
            consecutive_fire_limit=10,  # Default consecutive fire limit
            properties=properties,
        )

    def add_synapse(
        self,
        pre_neuron: int,
        post_neuron: int,
        weight: float,
        is_plastic: bool = False,
        plasticity_coeff: float = 0.0,
        plasticity_decay: float = 0.0,
        **kwargs,
    ) -> bool:
        """Add a synapse between two neurons.

        This is an alias for create_synapse to maintain compatibility with tests.

        Args:
            pre_neuron: ID of the pre-synaptic neuron
            post_neuron: ID of the post-synaptic neuron
            weight: Synapse weight
            is_plastic: Whether the synapse is plastic (can change weight)
            plasticity_coeff: Coefficient for plasticity
            plasticity_decay: Decay rate for plasticity
            **kwargs: Additional properties for the synapse

        Returns:
            True if synapse was created, False if it already existed
        """
        return self.create_synapse(
            pre_neuron_id=pre_neuron,
            post_neuron_id=post_neuron,
            weight=weight,
            is_plastic=is_plastic,
            plasticity_coeff=plasticity_coeff,
            plasticity_decay=plasticity_decay,
            **kwargs,
        )

    @property
    def neuron_count(self) -> int:
        """Get the total number of neurons in the connectome."""
        return self.get_neuron_count()

    @property
    def synapse_count(self) -> int:
        """Get the total number of synapses in the connectome using Rust NPU."""
        # ARCHITECTURE: Use Rust NPU directly (no deprecated synapse_array)
        return self._npu_interface.rust_npu.get_synapse_count()

    @property
    def is_initialized(self) -> bool:
        """Check if the connectome is initialized with cortical areas.

        A connectome is considered initialized when:
        1. It has cortical areas loaded
        2. The basic structures are in place

        This property is required for API brain running checks.
        """
        return len(self.cortical_areas) > 0

    def has_synapse(self, pre_neuron: int, post_neuron: int) -> bool:
        """Check if a synapse exists between two neurons using
        NPU SynapseArray.

        Args:
            pre_neuron: ID of the pre-synaptic neuron
            post_neuron: ID of the post-synaptic neuron

        Returns:
            True if the synapse exists, False otherwise
        """
        # Check if both neurons exist (use NPU interface)
        if (
            not self._npu_interface.neuron_exists(pre_neuron)
            or not self._npu_interface.neuron_exists(post_neuron)
        ):
            return False

        # ARCHITECTURE: Use Rust NPU directly (no deprecated synapse_array)
        # Check if synapse exists by querying outgoing synapses
        outgoing = self._npu_interface.rust_npu.get_outgoing_synapses(pre_neuron)
        return any(target == post_neuron for target, _, _, _ in outgoing)

    def update_neuron_property(
        self, neuron_id: int, property_name: str, value: Any
    ) -> bool:
        """Update a property of a neuron.

        This is an alias for set_neuron_property to maintain compatibility with tests.

        Args:
            neuron_id: ID of the neuron
            property_name: Name of the property to update
            value: New value for the property

        Returns:
            True if the property was updated, False otherwise
        """
        try:
            self.set_neuron_property(neuron_id, property_name, value)
            return True
        except (ValueError, KeyError):
            return False

    def delete_synapse(self, pre_neuron: int, post_neuron: int) -> bool:
        """Delete a synapse between two neurons.

        This is an alias for remove_synapse to maintain compatibility with tests.

        Args:
            pre_neuron: ID of the pre-synaptic neuron
            post_neuron: ID of the post-synaptic neuron

        Returns:
            True if the synapse was removed, False if it didn't exist
        """
        return self.remove_synapse(pre_neuron, post_neuron)


    @property
    def next_neuron_index(self) -> int:
        """Alias for next_neuron_id for backward compatibility with tests."""
        return self.next_neuron_id


    def delete_neurons(self, neuron_ids: List[int]) -> int:
        """Delete multiple neurons at once.

        Args:
            neuron_ids: List of neuron IDs to delete

        Returns:
            Number of neurons successfully deleted
        """
        # Vectorized bulk neuron deletion for RTOS/GPU compliance
        neuron_ids_array = np.array(neuron_ids, dtype=np.int32)
        valid_neuron_mask = np.array(
            [self._npu_interface.neuron_exists(nid) for nid in neuron_ids_array]
        )
        valid_neuron_ids = neuron_ids_array[valid_neuron_mask]

        deleted_count = 0
        if len(valid_neuron_ids) > 0:
            # Vectorized index lookup (neuron_id == index in Rust NPU)
            indices_to_delete = valid_neuron_ids  # Identity mapping

            # ✅ Use Rust NPU for neuron deletion
            for neuron_id in valid_neuron_ids:
                try:
                    self.delete_neuron(int(neuron_id))
                    deleted_count += 1
                except (ValueError, KeyError) as e:
                        logger.warning(
                            f"Failed to delete neuron {neuron_id}: {e}"
                        )

        # Report invalid neuron IDs for debugging
        invalid_count = len(neuron_ids) - len(valid_neuron_ids)
        if invalid_count > 0:
            logger.warning(
                f"Attempted to delete {invalid_count} non-existent neurons"
            )

        return deleted_count

    def delete_synapses(self, synapse_specs: List[Tuple[int, int]]) -> int:
        """Delete multiple synapses at once.

        Args:
            synapse_specs: List of (pre_neuron_id, post_neuron_id) tuples

        Returns:
            Number of synapses successfully deleted
        """
        deleted_count = 0
        for pre_id, post_id in synapse_specs:
            try:
                if self.remove_synapse(pre_id, post_id):
                    deleted_count += 1
            except (ValueError, KeyError) as e:
                logger.warning(
                    f"Failed to delete synapse {pre_id}->{post_id}: {e}"
                )

        return deleted_count

    def _clear_existing_brain_data(self) -> Dict[str, int]:
        """Clear all existing brain data (neurons, synapses, cortical areas).

        This method provides efficient memory reset for genome reloading.

        Returns:
            Dictionary with counts of cleared items
        """
        # 1. Count what we're clearing for reporting
        cortical_areas_cleared = len(getattr(self, "cortical_areas", {}))
        neurons_cleared = (
            self.get_neuron_count() if hasattr(self, "neuron_array") else 0
        )
        synapses_cleared = (
            self.get_synapse_count() if hasattr(self, "synapse_matrix") else 0
        )

        # 2. Clear cortical areas and related data structures
        if hasattr(self, "cortical_areas"):
            self.cortical_areas.clear()
        if hasattr(self, "area_neuron_masks"):
            self.area_neuron_masks.clear()
        
        # Clear neuron position cache for all areas
        self.clear_neuron_position_cache()
        
        # Clear Rust Morton hash
        if self._rust_morton_hash is not None:
            self._rust_morton_hash.clear()

        #  NOTE: No longer managing next_cortical_idx counter since we use
        #  dynamic allocation
        #  The _find_next_available_cortical_idx() method handles cortical_idx
        #  assignment by
        # scanning existing areas and respecting reserved indices

        # 3. Clear brain regions
        if hasattr(self, "brain_regions"):
            self.brain_regions.clear()
        if hasattr(self, "region_area_map"):
            self.region_area_map.clear()

        # 4. Reset neuron array efficiently
        if hasattr(self, "neuron_array"):
            try:
                # ✅ RUST NPU: Skip array manipulation - Rust NPU will be re-initialized
                # The Rust NPU is completely rebuilt during genome load via
                # BurstEngine.reinitialize_rust_npu(), so no manual clearing needed.
                if self._npu_interface and hasattr(self._npu_interface, '_rust_npu_integration'):
                    logger.info(
                        "🦀 [RUST-NPU] Skipping neuron array reset - Rust NPU will be re-initialized during genome load",
                        status="[OK]",
                    )
                    # Note: neuron_id tracking now in Rust NPU only (no Python dictionaries to clear)
                else:
                    # Legacy Python neuron array reset (should not be reached)
                    logger.warning("Legacy neuron array reset path - this should not be reached with Rust NPU")
                    
            except Exception as e:
                logger.warning(f"Error during neuron data reset: {e}")

        # 5. Clear all ID mappings in one operation
        if hasattr(self, "neuron_id_to_index"):
            self.neuron_id_to_index.clear()
        if hasattr(self, "index_to_neuron_id"):
            self.index_to_neuron_id.clear()

        # 6. Reset neuron counter
        if hasattr(self, "next_neuron_id"):
            self.next_neuron_id = 1  # Start from 1, not 0

        # 7. CRITICAL FIX: Clear cortical mapping to prevent stale area references
        if hasattr(self, "cortical_mapping"):
            self.cortical_mapping.clear(preserve_core_areas=True)
            logger.info(
                "Cleared cortical mapping (preserved core areas _death and _power)",
                status="[OK]",
            )

        # 8. Clear memory area tracking to prevent stale memory area references
        memory_areas_cleared = 0
        mappings_cleared = 0
        if hasattr(self, "memory_areas"):
            memory_areas_cleared = len(self.memory_areas)
            self.memory_areas.clear()
            logger.info(
                f"Cleared {memory_areas_cleared} memory areas from tracking set",
                status="[OK]",
            )
        if hasattr(self, "memory_area_upstream_mappings"):
            mappings_cleared = len(self.memory_area_upstream_mappings)
            self.memory_area_upstream_mappings.clear()
            logger.info(
                f"Cleared {mappings_cleared} memory area upstream mappings",
                status="[OK]",
            )

        # 9. CRITICAL FIX: Clear FCL state via NPU-owned managers to prevent stale cortical indices
        fcl_cleared = False
        try:
            # Prefer clearing through NPU BurstEngine if available
            burst_engine = getattr(self, "_npu_interface", None)
            if burst_engine and hasattr(burst_engine, "fcl_manager") and burst_engine.fcl_manager:
                fcl = burst_engine.fcl_manager
                if hasattr(fcl, "clear_all_fcl_history"):
                    fcl.clear_all_fcl_history()
                    fcl_cleared = True
                elif hasattr(fcl, "clear_all_window_caches"):
                    fcl.clear_all_window_caches()
                    fcl_cleared = True
            
            # Legacy local reference fallback
            if not fcl_cleared and hasattr(self, "fcl_manager") and self.fcl_manager:
                if hasattr(self.fcl_manager, "clear_all_fcl_history"):
                    self.fcl_manager.clear_all_fcl_history()
                    fcl_cleared = True
                elif hasattr(self.fcl_manager, "clear_all_window_caches"):
                    self.fcl_manager.clear_all_window_caches()
                    fcl_cleared = True

            if fcl_cleared:
                logger.info(
                    "Cleared FCL caches/history to prevent stale cortical indices",
                    status="[OK]",
                )
            else:
                logger.warning(
                    "FCL manager found but could not be cleared - may cause stale cortical index warnings"
                )
        except Exception as e:
            logger.warning(f"Error clearing FCL state: {e}")

        #  CRITICAL: Ensure NeuronArray and ConnectomeManager counters are
        #  synchronized
        # ✅ REMOVED: Neuron ID tracking now in Rust NPU only
        # No need to sync Python-side counters

        # 7. Clear synapse matrix efficiently
        if hasattr(self, "synapse_matrix"):
            try:
                # Reset sparse matrix to empty state
                if hasattr(self.synapse_matrix, "data"):
                    self.synapse_matrix.data = np.array([], dtype=np.float32)
                if hasattr(self.synapse_matrix, "indices"):
                    self.synapse_matrix.indices = np.array([], dtype=np.int32)
                if hasattr(self.synapse_matrix, "indptr"):
                    # Reset indptr to all zeros for empty CSR matrix
                    self.synapse_matrix.indptr = np.zeros(
                        self.max_neurons + 1, dtype=np.int32
                    )

                logger.info("Reset synapse matrix efficiently", status="[OK]")
            except Exception as e:
                logger.warning(f"Error resetting synapse matrix: {e}")

        # 8. Clear other neuron-related data structures
        if hasattr(self, "active_neurons"):
            self.active_neurons = np.zeros(self.max_neurons, dtype=bool)

        logger.info(
            f"Cleared {cortical_areas_cleared} cortical areas, {neurons_cleared} neurons, {synapses_cleared} synapses",
            status="[OK]",
        )
        
        # CRITICAL: Reset state manager counts to 0 after clearing brain data
        try:
            self.state_manager.set_brain_stats({
                "neuron_count": 0,
                "synapse_count": 0,
                "cortical_area_count": 0,
                "memory_neuron_count": 0,
                "non_memory_neuron_count": 0,
            })
            logger.info("✅ Reset state manager brain statistics to 0", status="[OK]")
        except Exception as e:
            logger.warning(f"Failed to reset state manager counts: {e}")

        return {
            "cortical_areas_cleared": cortical_areas_cleared,
            "neurons_cleared": neurons_cleared,
            "synapses_cleared": synapses_cleared,
            "cortical_mapping_cleared": True,
            "memory_areas_cleared": memory_areas_cleared,
            "memory_mappings_cleared": mappings_cleared,
            "fcl_manager_cleared": fcl_cleared,
        }

    def _ensure_brain_regions_structure(
        self, genome_data: Dict[str, Any]
    ) -> None:
        """Ensure that brain_regions structure exists in the genome and
        ConnectomeManager.

        This method automatically creates a default "root" brain region if none exists,
        ensuring compatibility with cortical area creation APIs and proper organization.

        Args:
            genome_data: The genome data being loaded
        """
        logger.info("[BRAIN REGIONS] Checking brain regions structure...")

        # Check if brain_regions exists in genome data
        if "brain_regions" not in genome_data:
            logger.info(
                "[BRAIN REGIONS] No brain_regions found in genome - creating default structure"
            )
            genome_data["brain_regions"] = {}

        # Ensure root region exists
        if "root" not in genome_data["brain_regions"]:
            logger.info("[BRAIN REGIONS] Creating default 'root' brain region")

            #  Get existing cortical areas from blueprint to assign to root
            #  region
            existing_areas = []
            blueprint = genome_data.get("blueprint", {})
            if blueprint:
                #  ARCHITECTURE: Only support hierarchical format (single
                #  source of truth)
                #  Flat format support removed - all genomes converted to
                #  hierarchical in GenomeService
                existing_areas = list(blueprint.keys())
                logger.info(
                    f"[BRAIN REGIONS] Found {len(existing_areas)} cortical areas in hierarchical blueprint: {existing_areas}"
                )

            # Automatically assign IPU/OPU areas as inputs/outputs
            auto_inputs = []
            auto_outputs = []
            blueprint = genome_data.get("blueprint", {})
            
            for area_id in existing_areas:
                area_props = blueprint.get(area_id, {})
                # Check both 'group' and 'cortical_group' for compatibility
                area_group = area_props.get("group", area_props.get("cortical_group", "")).upper()
                
                if area_group == "IPU":
                    auto_inputs.append(area_id)
                elif area_group == "OPU":
                    auto_outputs.append(area_id)

            # Create default root region with existing areas and automatic I/O
            genome_data["brain_regions"]["root"] = {
                "title": "Root Brain Region",
                "description": "Default root region for brain organization",
                "parent_region_id": None,
                "coordinate_2d": [0, 0],
                "coordinate_3d": [0, 0, 0],
                "areas": existing_areas,
                "regions": [],  # Will be populated as child regions are created
                "inputs": auto_inputs,
                "outputs": auto_outputs,
                "signature": "",
            }

            logger.info(
                f"[BRAIN REGIONS] Created root region with {len(existing_areas)} cortical areas"
            )

        # Initialize ConnectomeManager's brain_regions from genome
        if not hasattr(self, "brain_regions"):
            self.brain_regions = {}

        # Sync ConnectomeManager's brain_regions with genome
        self.brain_regions.update(genome_data["brain_regions"])
        
        # Load hierarchical brain region system
        try:
            self.brain_region_hierarchy.load_from_genome(genome_data)
            logger.info("[BRAIN REGIONS] Loaded hierarchical brain region system")
        except Exception as e:
            logger.error(f"[BRAIN REGIONS] Failed to load hierarchy: {e}")
            # Continue without hierarchy - graceful degradation

        #  ARCHITECTURE: StateManager is the single source of truth for genome
        #  data
        # No local genome reference needed - always access through StateManager

        logger.info(
            f"[BRAIN REGIONS] Brain regions structure ensured - {len(self.brain_regions)} regions available"
        )

    def prepare_for_new_genome(
        self, genome_data: Dict[str, Any], save_current_state: bool = True
    ) -> Dict[str, Any]:
        """Prepare connectome for loading a new genome.

        This method handles the complete preparation process:
        1. Detect existing brain state
        2. Save current state if requested
        3. Clear all existing data
        4. Ensure sufficient memory capacity
        5. Reset all counters and indices

        Args:
            genome_data: The genome data that will be loaded
            save_current_state: Whether to save the current brain state before clearing

        Returns:
            Dictionary with preparation results and saved state info
        """
        logger.info(
            "PREPARE FOR NEW GENOME: Starting complete brain reset process",
            status="[BRAIN]",
        )

        # STEP 1: DETECT EXISTING BRAIN STATE
        has_existing_brain = (
            hasattr(self, "cortical_areas")
            and len(getattr(self, "cortical_areas", {})) > 0
        )

        # Initialize saved state info
        saved_state_info = None

        if has_existing_brain:
            existing_area_count = (
                len(self.cortical_areas)
                if hasattr(self, "cortical_areas")
                else 0
            )
            logger.info(
                f"Step 2: Had existing brain with {existing_area_count} cortical areas (now cleared)"
            )

            # STEP 3: SAVE CURRENT STATE IF REQUESTED
            if save_current_state:
                try:
                    #  Save current brain state (placeholder - implement actual
                    #  save logic)
                    saved_state_info = {
                        "filename": "brain_state_backup.json",
                        "timestamp": "now",
                    }
                    logger.info(
                        f"Current brain state saved: {saved_state_info['filename']}",
                        status="[OK]",
                    )
                except Exception:
                    logger.warning(
                        "Failed to save current brain state - proceeding anyway",
                        status="[WARN]",
                    )
        else:
            logger.info(
                "Step 2: No existing brain found - proceeding with fresh initialization"
            )

        # STEP 4: CLEAR EXISTING BRAIN DATA (ALWAYS!)
        #  Even if no existing brain, we need to reset counters from any
        #  previous state
        logger.info("Step 4: Clearing/resetting all brain data and arrays")
        clear_results = self._clear_existing_brain_data()
        logger.info(
            f"Cleared/reset {clear_results['cortical_areas_cleared']} cortical areas, "
            f"{clear_results['neurons_cleared']} neurons, {clear_results['synapses_cleared']} synapses",
            status="[OK]",
        )

        # STEP 5: ENSURE BRAIN REGIONS STRUCTURE EXISTS
        logger.info("Step 5: Ensuring brain regions structure exists")
        self._ensure_brain_regions_structure(genome_data)

        # STEP 6: CHECK MEMORY CAPACITY AND REALLOCATE IF NEEDED
        logger.info("Step 6: Checking memory capacity requirements")
        # Estimate memory requirements from genome (simplified)
        estimated_neurons = (
            len(genome_data.get("blueprint", {}).get("cortical_areas", {}))
            * 1000
        )
        estimated_synapses = estimated_neurons * 10

        capacity_results = {
            "reallocated": False,
            "max_neurons": self.max_neurons,
            "max_synapses": self.max_synapses,
        }

        if (
            estimated_neurons > self.max_neurons
            or estimated_synapses > self.max_synapses
        ):
            logger.info(
                f"Reallocating memory: {estimated_neurons} neurons, {estimated_synapses} synapses"
            )
            # Note: Actual reallocation would happen here
            capacity_results["reallocated"] = True
            capacity_results["max_neurons"] = max(
                self.max_neurons, estimated_neurons
            )
            capacity_results["max_synapses"] = max(
                self.max_synapses, estimated_synapses
            )

        if capacity_results["reallocated"]:
            logger.info(
                f"Reallocated connectome with {capacity_results['max_neurons']} neurons, "
                f"{capacity_results['max_synapses']} synapses",
                status="[OK]",
            )

            #  CRITICAL: After reallocation, ensure NeuronArray is in pristine
            #  state (through NPU Interface)
            if self._npu_interface and self._npu_interface.neuron_array:
                neuron_array = self._npu_interface.neuron_array
                neuron_array.next_index = 0
                neuron_array.neuron_count = 0
                neuron_array.free_indices = set()
                # ✅ Neuron ID tracking now in Rust NPU only
                logger.info(
                    "Post-reallocation completed (neuron IDs managed by Rust NPU)",
                    status="[OK]",
                )
        else:
            logger.info(
                f"Using existing capacity: {capacity_results['max_neurons']} neurons, "
                f"{capacity_results['max_synapses']} synapses",
                status="[OK]",
            )

        logger.info(
            "PREPARE FOR NEW GENOME: Brain preparation completed successfully",
            status="[TARGET]",
        )

        return {
            "success": True,
            "had_existing_brain": has_existing_brain,
            "saved_state_info": saved_state_info,
            "clear_results": clear_results,
            "capacity_results": capacity_results,
            "message": "Connectome prepared for new genome loading",
        }

    @property
    def neuron_array(self):
        """DEPRECATED: Legacy neuron_array access - Requires Rust NPU refactor.
        
        This property exists ONLY for backward compatibility with legacy code.
        49 instances remain in ConnectomeManager that need refactoring.
        
        ⚠️ WARNING: Direct neuron_array access is DEPRECATED!
        ✅ Use Rust NPU methods instead via _npu_interface.rust_npu
        
        This stub allows old code to continue functioning while we migrate,
        but logs warnings to identify what needs refactoring.
        """
        import warnings
        import traceback
        
        # Log which method is accessing neuron_array (for refactoring tracking)
        stack = traceback.extract_stack()
        caller = stack[-2]  # Get caller's location
        logger.warning(
            f"⚠️ [DEPRECATED] neuron_array accessed from {caller.filename}:{caller.lineno} "
            f"in {caller.name}() - Requires Rust NPU refactor"
        )
        
        # Return a minimal compatibility stub for transitional period
        # This will be REMOVED when Rust ConnectomeManager is implemented
        class _DeprecatedNeuronArrayStub:
            """TEMPORARY stub until Rust ConnectomeManager migration.
            
            Provides minimal compatibility for code paths that still access neuron_array.
            Logs all accesses to track what needs refactoring.
            
            ⚠️ DO NOT add new code that depends on this!
            ✅ This will be REMOVED in Rust ConnectomeManager migration.
            """
            def __init__(self, npu_interface):
                self._npu = npu_interface
            
            def __getattr__(self, name):
                # Log access for tracking (helps identify what needs Rust implementation)
                logger.debug(
                    f"⚠️ [DEPRECATED-BRIDGE] neuron_array.{name} accessed - "
                    f"Will be removed in Rust ConnectomeManager"
                )
                
                # Raise error - forces explicit handling
                raise AttributeError(
                    f"neuron_array.{name} not available in Rust NPU. "
                    f"This will be implemented in Rust ConnectomeManager migration. "
                    f"If critical, contact team to prioritize."
                )
        
        return _DeprecatedNeuronArrayStub(self._npu_interface)
    
    @neuron_array.setter
    def neuron_array(self, value):
        """DEPRECATED: Ignore neuron_array assignments during migration.
        
        Legacy code tries to set self.neuron_array during initialization.
        We ignore these assignments since Rust NPU is the source of truth.
        """
        if value is not None:
            logger.debug(
                f"⚠️ [DEPRECATED] Attempted to assign neuron_array = {type(value).__name__} - "
                f"Ignored (Rust NPU is source of truth)"
            )
        # Silently ignore - no-op

    def _invalidate_mapping_cache(self):
        """Cache invalidation no longer needed - direct delegation to NeuronArray"""
        # PERFORMANCE: Invalidate spatial index when neuron structure changes
        if hasattr(self, "_spatial_index"):
            self._spatial_index.clear()
        pass

    def _vectorized_index_to_neuron_id(self, indices):
        """Vectorized index-to-neuron-ID conversion for performance-critical
        paths.

        Args:
            indices: Single index (int) or array of indices (np.ndarray)

        Returns:
            Single neuron ID (int) or array of neuron IDs (np.ndarray)
        """
        if isinstance(indices, (int, np.integer)):
            # ✅ Single index lookup via NPU interface
            return self.index_to_neuron_id.get(indices, -1)
        else:
            # ✅ Batch lookup using NPU interface mapping
            mapping = self.index_to_neuron_id
            mapped = []
            for idx in list(np.asarray(indices)):
                nid = mapping.get(int(idx))
                if nid is not None:
                    mapped.append(int(nid))
            return np.array(mapped, dtype=np.int64)

    # ======================================================================
    # QUERY METHODS FOR TESTS
    # ======================================================================

    def query_neurons_by_threshold_range(
        self, min_threshold: float, max_threshold: float
    ) -> List[int]:
        """Query neurons by threshold range.

        Args:
            min_threshold: Minimum threshold value (inclusive)
            max_threshold: Maximum threshold value (inclusive)

        Returns:
            List of neuron IDs with thresholds in the specified range
        """
        neuron_ids = []
        # ✅ Get all valid neuron IDs from Rust NPU (single source of truth)
        if self._npu_interface:
            neuron_count = self._npu_interface.rust_npu.get_neuron_count()
            for neuron_id in range(neuron_count):
                if not self._npu_interface.neuron_exists(neuron_id):
                    continue
                threshold = self.get_neuron_property(
                    neuron_id, NeuronPropertyType.THRESHOLD
                )
                if min_threshold <= threshold <= max_threshold:
                    neuron_ids.append(neuron_id)
        return neuron_ids

    def query_neurons_by_area_and_position(
        self,
        cortical_id: str,
        x_range: Tuple[int, int],
        y_range: Tuple[int, int],
        z_range: Tuple[int, int] = None,
    ) -> List[int]:
        """Query neurons by cortical area and position range.

        Args:
            cortical_id: ID of the cortical area
            x_range: Tuple of (min_x, max_x) inclusive
            y_range: Tuple of (min_y, max_y) inclusive
            z_range: Optional tuple of (min_z, max_z) inclusive

        Returns:
            List of neuron IDs in the specified area and position range
        """
        neurons_in_area = self.get_neurons_by_cortical_area(cortical_id)
        matching_neurons = []

        for neuron_id in neurons_in_area:
            x, y, z = self.get_neuron_position(neuron_id)
            if (
                x_range[0] <= x <= x_range[1]
                and y_range[0] <= y <= y_range[1]
                and (z_range is None or z_range[0] <= z <= z_range[1])
            ):
                matching_neurons.append(neuron_id)

        return matching_neurons

    def check_neuron_index_uniqueness(self) -> bool:
        """Check that all neuron indices are unique.
        
        ARCHITECTURE NOTE: In Rust NPU, neuron_id == index always (identity mapping),
        so indices are unique by definition. This method is kept for compatibility.

        Returns:
            True (always, as neuron_id == index in Rust NPU)
        """
        return True

    def get_neurons_at_position(
        self, cortical_id: str, position: Tuple[int, int, int]
    ) -> List[int]:
        """Get neurons at a specific position in a cortical area.

        PERFORMANCE: Optimized O(1) lookup using spatial indexing instead of O(N) linear search.
        This eliminates the 8-second bottleneck in synaptogenesis.

        Args:
            cortical_id: ID of the cortical area
            position: Tuple of (x, y, z) coordinates

        Returns:
            List of neuron IDs at the specified position
        """
        #  CRITICAL PERFORMANCE FIX: Use spatial indexing instead of linear
        #  search
        try:
            # Get cortical area to access spatial index
            cortical_area = self.get_cortical_area(cortical_id)
            if hasattr(cortical_area, "get_neurons_at_position"):
                # Use the cortical area's optimized spatial lookup
                return cortical_area.get_neurons_at_position(position)

            # Fallback: Build spatial index on-demand if not available
            if not hasattr(self, "_spatial_index"):
                self._spatial_index = {}

            # Check if we have cached spatial index for this area
            if cortical_id not in self._spatial_index:
                self._build_spatial_index_for_area(cortical_id)

            # O(1) lookup using spatial index
            area_index = self._spatial_index[cortical_id]
            return area_index.get(position, [])

        except Exception as e:
            self.logger.warning(
                f"Spatial index lookup failed for {cortical_id} at {position}: {e}"
            )
            # Emergency fallback to linear search (should rarely happen)
            return self._linear_search_neurons_at_position(
                cortical_id, position
            )

    def _build_spatial_index_for_area(self, cortical_id: str) -> None:
        """Build spatial index for fast position-based neuron lookups.

        PERFORMANCE: This is called once per area and provides O(1) lookups thereafter.
        """
        try:
            self._spatial_index[cortical_id] = {}
            neurons_in_area = self.get_neurons_by_cortical_area(cortical_id)

            # Build position -> [neuron_ids] mapping
            for neuron_id in neurons_in_area:
                position = self.get_neuron_position(neuron_id)
                if position not in self._spatial_index[cortical_id]:
                    self._spatial_index[cortical_id][position] = []
                self._spatial_index[cortical_id][position].append(neuron_id)

        except Exception as e:
            self.logger.error(
                f"Failed to build spatial index for {cortical_id}: {e}"
            )
            self._spatial_index[cortical_id] = {}

    def _linear_search_neurons_at_position(
        self, cortical_id: str, position: Tuple[int, int, int]
    ) -> List[int]:
        """Fallback linear search method (original implementation).

        PERFORMANCE: This is the slow O(N) method that caused 8-second delays.
        Only used as emergency fallback.
        """
        neurons_in_area = self.get_neurons_by_cortical_area(cortical_id)
        matching_neurons = []

        for neuron_id in neurons_in_area:
            neuron_position = self.get_neuron_position(neuron_id)
            if neuron_position == position:
                matching_neurons.append(neuron_id)

        return matching_neurons

    def get_connection_matrix(
        self, source_area_id: str, target_area_id: str
    ) -> Dict[str, Any]:
        """Get connection matrix between two cortical areas.

        Args:
            source_area_id: ID of the source cortical area
            target_area_id: ID of the target cortical area

        Returns:
            Dictionary containing connection matrix information including:
            - source_neurons: List of source neuron IDs
            - target_neurons: List of target neuron IDs
            - connections: List of (source_idx, target_idx, weight) tuples
            - total_connections: Total number of connections
            - total_weight: Sum of all connection weights
        """
        try:
            # Get source and target neurons
            source_neurons = self.get_neurons_by_area(source_area_id)
            target_neurons = self.get_neurons_by_area(target_area_id)

            if not source_neurons or not target_neurons:
                return {
                    "source_neurons": [],
                    "target_neurons": [],
                    "connections": [],
                    "total_connections": 0,
                    "total_weight": 0.0,
                }

            # Get source and target indices
            source_indices = self._get_neuron_indices(source_neurons)
            target_indices = self._get_neuron_indices(target_neurons)

            # Get connection matrix
            connections = []
            total_weight = 0.0

            for src_idx in source_indices:
                # Convert index back to neuron ID for get_outgoing_connections
                src_neuron_id = self.index_to_neuron_id.get(src_idx)
                if src_neuron_id is None:
                    continue

                # Get outgoing connections for this neuron
                outgoing = self.get_outgoing_connections(src_neuron_id)
                if outgoing:
                    for dst_neuron_id, weight in outgoing:
                        # Convert target neuron ID to index for comparison
                        dst_idx = self.neuron_id_to_index.get(dst_neuron_id)
                        if dst_idx is not None and dst_idx in target_indices:
                            #  Return neuron IDs instead of indices for
                            #  consistency
                            connections.append(
                                (src_neuron_id, dst_neuron_id, weight)
                            )
                            total_weight += weight

            return {
                "source_neurons": source_neurons,
                "target_neurons": target_neurons,
                "connections": connections,
                "total_connections": len(connections),
                "total_weight": total_weight,
            }

        except Exception as e:
            self.logger.error(f"Error getting connection matrix: {str(e)}")
            return {
                "source_neurons": [],
                "target_neurons": [],
                "connections": [],
                "total_connections": 0,
                "total_weight": 0.0,
            }

    def _get_neuron_indices(self, neuron_ids: List[int]) -> List[int]:
        """Get neuron indices for a list of neuron IDs.

        Args:
            neuron_ids: List of neuron IDs to get indices for

        Returns:
            List of neuron indices (identity mapping: neuron_id == index in Rust NPU)
        """
        try:
            # In Rust NPU, neuron_id == index (identity mapping)
            indices = []
            for neuron_id in neuron_ids:
                if self._npu_interface.neuron_exists(neuron_id):
                    indices.append(neuron_id)  # Identity mapping
            return indices
        except Exception as e:
            self.logger.error(f"Error getting neuron indices: {str(e)}")
            return []

    # ======================================================================
    # CONNECTIVITY RULES MANAGEMENT
    # ======================================================================

    def add_connectivity_rule(
        self,
        name: str,
        source_area_id: str,
        target_area_id: str,
        rule_type: str,
        parameters: Dict[str, Any],
        enabled: bool = True,
        rule_id: Optional[str] = None,
    ) -> str:
        """Add a connectivity rule between two cortical areas.

        Args:
            name: Human-readable name for the rule
            source_area_id: Source cortical area ID
            target_area_id: Target cortical area ID
            rule_type: Type of rule ('one-to-one', 'all-to-all', 'probabilistic', 'distance', 'random-subset')
            parameters: Rule-specific parameters
            enabled: Whether the rule is enabled
            rule_id: Optional specific rule ID (auto-generated if None)

        Returns:
            The rule ID

        Raises:
            ValueError: If areas don't exist or rule_type is invalid
        """
        import time
        import uuid

        # Validate areas exist
        if source_area_id not in self.cortical_areas:
            raise ValueError(f"Source area {source_area_id} does not exist")
        if target_area_id not in self.cortical_areas:
            raise ValueError(f"Target area {target_area_id} does not exist")

        # Validate rule type
        valid_types = [
            "one-to-one",
            "all-to-all",
            "probabilistic",
            "distance",
            "random-subset",
        ]
        if rule_type not in valid_types:
            raise ValueError(
                f"Invalid rule type {rule_type}. Must be one of: {valid_types}"
            )

        # Generate rule ID if not provided
        if rule_id is None:
            rule_id = str(uuid.uuid4())

        # Store the rule
        self.connectivity_rules[rule_id] = {
            "name": name,
            "source_cortical_id": source_area_id,  # Use cortical_id for consistency
            "target_cortical_id": target_area_id,  # Use cortical_id for consistency
            "rule_type": rule_type,
            "parameters": parameters.copy(),
            "enabled": enabled,
            "created_at": time.time(),
        }

        logger.info(
            f"Added connectivity rule '{name}' ({rule_id}) from {source_area_id} to {target_area_id}"
        )
        return rule_id

    def get_connectivity_rule(self, rule_id: str) -> Dict[str, Any]:
        """Get a connectivity rule by ID.

        Args:
            rule_id: Rule ID

        Returns:
            Rule dictionary

        Raises:
            KeyError: If rule doesn't exist
        """
        if rule_id not in self.connectivity_rules:
            raise KeyError(f"Connectivity rule {rule_id} does not exist")
        return self.connectivity_rules[rule_id].copy()

    def update_connectivity_rule(
        self, rule_id: str, updates: Dict[str, Any]
    ) -> bool:
        """Update a connectivity rule.

        Args:
            rule_id: Rule ID
            updates: Dictionary of updates to apply

        Returns:
            True if updated successfully

        Raises:
            KeyError: If rule doesn't exist
        """
        if rule_id not in self.connectivity_rules:
            raise KeyError(f"Connectivity rule {rule_id} does not exist")

        # Apply updates
        for key, value in updates.items():
            if key in self.connectivity_rules[rule_id]:
                self.connectivity_rules[rule_id][key] = value

        logger.info(f"Updated connectivity rule {rule_id}")
        return True

    def delete_connectivity_rule(self, rule_id: str) -> bool:
        """Delete a connectivity rule.

        Args:
            rule_id: Rule ID

        Returns:
            True if deleted successfully

        Raises:
            KeyError: If rule doesn't exist
        """
        if rule_id not in self.connectivity_rules:
            raise KeyError(f"Connectivity rule {rule_id} does not exist")

        del self.connectivity_rules[rule_id]
        logger.info(f"Deleted connectivity rule {rule_id}")
        return True

    def get_connectivity_rules_for_areas(
        self, source_area_id: str = None, target_area_id: str = None
    ) -> List[str]:
        """Get connectivity rules for specific areas.

        Args:
            source_area_id: Source area ID (optional)
            target_area_id: Target area ID (optional)

        Returns:
            List of rule IDs matching the criteria
        """
        matching_rules = []
        for rule_id, rule in self.connectivity_rules.items():
            if source_area_id and rule["source_cortical_id"] != source_area_id:
                continue
            if target_area_id and rule["target_cortical_id"] != target_area_id:
                continue
            matching_rules.append(rule_id)
        return matching_rules

    def apply_connectivity_rule(
        self,
        rule_id: str,
        weight_override: Optional[float] = None,
        max_synapses: int = 1_000_000,  # Increased from 10,000 to 1M for large cortical areas
    ) -> int:
        """Apply a single connectivity rule.

        Args:
            rule_id: Rule ID to apply
            weight_override: Override the weight specified in the rule
            max_synapses: Maximum number of synapses to create

        Returns:
            Number of synapses created

        Raises:
            KeyError: If rule doesn't exist
        """
        if rule_id not in self.connectivity_rules:
            raise KeyError(f"Connectivity rule {rule_id} does not exist")

        rule = self.connectivity_rules[rule_id]
        if not rule["enabled"]:
            return 0

        # Use the existing batch application logic
        results = self.apply_rule_batch(
            [rule_id], weight_override, max_synapses
        )
        return results.get(rule_id, 0)

    # ======================================================================
    # CORTICAL CONNECTIONS MANAGEMENT
    # ======================================================================

    def add_cortical_connection(
        self,
        name: str,
        source_area_id: str,
        target_area_id: str,
        properties: Optional[Dict[str, Any]] = None,
        connection_id: Optional[str] = None,
    ) -> str:
        """Add a cortical connection between two areas.

        Args:
            name: Human-readable name for the connection
            source_area_id: Source cortical area ID
            target_area_id: Target cortical area ID
            properties: Optional properties dictionary
            connection_id: Optional specific connection ID

        Returns:
            The connection ID

        Raises:
            ValueError: If areas don't exist
        """
        import time
        import uuid

        # Validate areas exist
        if source_area_id not in self.cortical_areas:
            raise ValueError(f"Source area {source_area_id} does not exist")
        if target_area_id not in self.cortical_areas:
            raise ValueError(f"Target area {target_area_id} does not exist")

        # Generate connection ID if not provided
        if connection_id is None:
            connection_id = str(uuid.uuid4())

        # Store the connection
        self.cortical_connections[connection_id] = {
            "name": name,
            "source_area_id": source_area_id,
            "target_area_id": target_area_id,
            "properties": properties.copy() if properties else {},
            "synapse_count": 0,
            "created_at": time.time(),
        }

        logger.info(
            f"Added cortical connection '{name}' ({connection_id}) from {source_area_id} to {target_area_id}"
        )
        return connection_id

    def get_cortical_connection(self, connection_id: str) -> Dict[str, Any]:
        """Get a cortical connection by ID.

        Args:
            connection_id: Connection ID

        Returns:
            Connection dictionary

        Raises:
            KeyError: If connection doesn't exist
        """
        if connection_id not in self.cortical_connections:
            raise KeyError(
                f"Cortical connection {connection_id} does not exist"
            )
        return self.cortical_connections[connection_id].copy()

    def update_cortical_connection(
        self, connection_id: str, updates: Dict[str, Any]
    ) -> bool:
        """Update a cortical connection.

        Args:
            connection_id: Connection ID
            updates: Dictionary of updates to apply

        Returns:
            True if updated successfully

        Raises:
            KeyError: If connection doesn't exist
        """
        if connection_id not in self.cortical_connections:
            raise KeyError(
                f"Cortical connection {connection_id} does not exist"
            )

        # Apply updates
        for key, value in updates.items():
            if key == "properties" and isinstance(value, dict):
                # Merge properties instead of replacing
                self.cortical_connections[connection_id]["properties"].update(
                    value
                )
            else:
                self.cortical_connections[connection_id][key] = value

        logger.info(f"Updated cortical connection {connection_id}")

        # Update StateManager cortical areas cache for mapping changes
        try:
            from feagi.core.state_manager import get_state_manager

            state_manager = get_state_manager()
            connection = self.cortical_connections[connection_id]
            # Invalidate cache for both source and target areas
            state_manager.update_cortical_areas_cache(
                connection["source_area_id"], "mapping_update"
            )
            state_manager.update_cortical_areas_cache(
                connection["target_area_id"], "mapping_update"
            )
        except Exception as e:
            self.logger.warning(
                f"Failed to update cortical areas cache after connection update {connection_id}: {e}"
            )

        return True

    def delete_cortical_connection(
        self, connection_id: str, delete_synapses: bool = False
    ) -> bool:
        """Delete a cortical connection.

        Args:
            connection_id: Connection ID
            delete_synapses: Whether to delete associated synapses

        Returns:
            True if deleted successfully

        Raises:
            KeyError: If connection doesn't exist
        """
        if connection_id not in self.cortical_connections:
            raise KeyError(
                f"Cortical connection {connection_id} does not exist"
            )

        connection = self.cortical_connections[connection_id]

        if delete_synapses:
            # Delete synapses between the areas
            source_neurons = self.get_neurons_by_cortical_area(
                connection["source_area_id"]
            )
            target_neurons = self.get_neurons_by_cortical_area(
                connection["target_area_id"]
            )

            deleted_count = 0
            for source_id in source_neurons:
                for target_id in target_neurons:
                    if self.has_synapse(source_id, target_id):
                        self.remove_synapse(source_id, target_id)
                        deleted_count += 1

            logger.info(
                f"Deleted {deleted_count} synapses for connection {connection_id}"
            )

        del self.cortical_connections[connection_id]
        logger.info(f"Deleted cortical connection {connection_id}")
        return True

    def update_synapse_count_for_connection(self, connection_id: str) -> int:
        """Update and return the synapse count for a connection.

        Args:
            connection_id: Connection ID

        Returns:
            Number of synapses in the connection

        Raises:
            KeyError: If connection doesn't exist
        """
        if connection_id not in self.cortical_connections:
            raise KeyError(
                f"Cortical connection {connection_id} does not exist"
            )

        connection = self.cortical_connections[connection_id]
        source_neurons = self.get_neurons_by_cortical_area(
            connection["source_area_id"]
        )
        target_neurons = self.get_neurons_by_cortical_area(
            connection["target_area_id"]
        )

        synapse_count = 0
        for source_id in source_neurons:
            for target_id in target_neurons:
                if self.has_synapse(source_id, target_id):
                    synapse_count += 1

        self.cortical_connections[connection_id][
            "synapse_count"
        ] = synapse_count
        return synapse_count

    def get_connection_statistics(self, connection_id: str) -> Dict[str, Any]:
        """Get statistics for a cortical connection.

        Args:
            connection_id: Connection ID

        Returns:
            Dictionary with connection statistics

        Raises:
            KeyError: If connection doesn't exist
        """
        if connection_id not in self.cortical_connections:
            raise KeyError(
                f"Cortical connection {connection_id} does not exist"
            )

        connection = self.cortical_connections[connection_id]
        source_neurons = self.get_neurons_by_cortical_area(
            connection["source_area_id"]
        )
        target_neurons = self.get_neurons_by_cortical_area(
            connection["target_area_id"]
        )

        synapse_count = 0
        total_weight = 0.0
        weights = []

        for source_id in source_neurons:
            for target_id in target_neurons:
                if self.has_synapse(source_id, target_id):
                    weight = self.get_synapse_weight(source_id, target_id)
                    synapse_count += 1
                    total_weight += weight
                    weights.append(weight)

        avg_weight = total_weight / synapse_count if synapse_count > 0 else 0.0

        return {
            "synapse_count": synapse_count,
            "avg_weight": avg_weight,
            "total_weight": total_weight,
            "source_neuron_count": len(source_neurons),
            "target_neuron_count": len(target_neurons),
        }

    def apply_connection_weight_change(
        self, connection_id: str, weight_multiplier: float
    ) -> int:
        """Apply a weight change to all synapses in a connection.

        Args:
            connection_id: Connection ID
            weight_multiplier: Multiplier to apply to all weights

        Returns:
            Number of synapses modified

        Raises:
            KeyError: If connection doesn't exist
        """
        if connection_id not in self.cortical_connections:
            raise KeyError(
                f"Cortical connection {connection_id} does not exist"
            )

        connection = self.cortical_connections[connection_id]
        source_neurons = self.get_neurons_by_cortical_area(
            connection["source_area_id"]
        )
        target_neurons = self.get_neurons_by_cortical_area(
            connection["target_area_id"]
        )

        modified_count = 0
        for source_id in source_neurons:
            for target_id in target_neurons:
                if self.has_synapse(source_id, target_id):
                    current_weight = self.get_synapse_weight(
                        source_id, target_id
                    )
                    new_weight = current_weight * weight_multiplier
                    self.update_synapse_weight(
                        source_id, target_id, new_weight
                    )
                    modified_count += 1

        logger.info(
            f"Applied weight multiplier {weight_multiplier} to {modified_count} synapses in connection {connection_id}"
        )
        return modified_count

    def get_connections_by_area(
        self, area_id: str, as_source: bool = True, as_target: bool = True
    ) -> List[str]:
        """Get connections involving a specific area.

        Args:
            area_id: Cortical area ID
            as_source: Include connections where this area is the source
            as_target: Include connections where this area is the target

        Returns:
            List of connection IDs
        """
        matching_connections = []
        for connection_id, connection in self.cortical_connections.items():
            if as_source and connection["source_area_id"] == area_id:
                matching_connections.append(connection_id)
            elif as_target and connection["target_area_id"] == area_id:
                matching_connections.append(connection_id)
        return matching_connections

    # ======================================================================
    # SAVE/LOAD FUNCTIONALITY
    # ======================================================================

    def save(self, filename: str) -> bool:
        """Save the connectome to a file.

        Args:
            filename: Path to save the connectome

        Returns:
            True if saved successfully

        Raises:
            Exception: If save fails
        """
        import pickle

        try:
            # Prepare data to save
            save_data = {
                "cortical_areas": {
                    cid: area.to_dict()
                    for cid, area in self.cortical_areas.items()
                },
                "brain_regions": self.brain_regions.copy(),
                "region_area_map": self.region_area_map.copy(),
                "connectivity_rules": self.connectivity_rules.copy(),
                "cortical_connections": self.cortical_connections.copy(),
                "neuron_data": self._serialize_neuron_data(),
                "synapse_data": self._serialize_synapse_data(),
                "metadata": {
                    "version": "1.0",
                    "max_neurons": self.max_neurons,
                    "max_synapses": self.max_synapses,
                    "neuron_count": self.get_neuron_count(),
                    "synapse_count": self.get_synapse_count(),
                },
            }

            # Save to file
            with open(filename, "wb") as f:
                pickle.dump(save_data, f)

            logger.info(f"Saved connectome to {filename}")
            return True

        except Exception as e:
            logger.error(f"Failed to save connectome to {filename}: {e}")
            raise

    @classmethod
    def load(cls, filename: str) -> "ConnectomeManager":
        """Load a connectome from a file.

        Args:
            filename: Path to load the connectome from

        Returns:
            Loaded ConnectomeManager instance

        Raises:
            Exception: If load fails
        """
        import pickle

        try:
            # Load data from file
            with open(filename, "rb") as f:
                save_data = pickle.load(f)

            # Create new ConnectomeManager instance
            metadata = save_data.get("metadata", {})
            max_neurons = metadata.get("max_neurons", 10_000_000)
            max_synapses = metadata.get("max_synapses", 100_000_000)

            # Reset singleton to allow loading
            cls.reset_singleton()
            connectome = cls(max_neurons, max_synapses)

            # Restore cortical areas
            for cid, area_data in save_data.get("cortical_areas", {}).items():
                from feagi.bdu.models.cortical_area import CorticalArea

                area = CorticalArea.from_dict(area_data)
                connectome.cortical_areas[cid] = area
                # Only sync mapping if cortical_idx is valid
                if area.cortical_idx is not None:
                    connectome._sync_cortical_mapping(cid, area.cortical_idx)

            # Restore brain regions and mappings
            connectome.brain_regions = save_data.get("brain_regions", {})
            connectome.region_area_map = save_data.get("region_area_map", {})
            connectome.connectivity_rules = save_data.get(
                "connectivity_rules", {}
            )
            connectome.cortical_connections = save_data.get(
                "cortical_connections", {}
            )

            # Restore neuron and synapse data
            connectome._deserialize_neuron_data(
                save_data.get("neuron_data", {})
            )
            connectome._deserialize_synapse_data(
                save_data.get("synapse_data", {})
            )

            logger.info(f"Loaded connectome from {filename}")
            return connectome

        except Exception as e:
            logger.error(f"Failed to load connectome from {filename}: {e}")
            raise

    def _serialize_neuron_data(self) -> Dict[str, Any]:
        """Serialize neuron data for saving."""
        try:
            # ✅ Get all valid neuron IDs from Rust NPU (single source of truth)
            neuron_data = {}
            if self._npu_interface:
                neuron_count = self._npu_interface.rust_npu.get_neuron_count()
                for neuron_id in range(neuron_count):
                    if not self._npu_interface.neuron_exists(neuron_id):
                        continue
                    neuron_data[neuron_id] = self.get_neuron(neuron_id)

            return {
                "neurons": neuron_data,
                "next_neuron_id": self.next_neuron_id,  # Use ConnectomeManager's counter
            }

        except Exception as e:
            logger.error(f"Error serializing neuron data: {e}")
            return {"neurons": {}, "next_neuron_id": 1}

    def _serialize_synapse_data(self) -> Dict[str, Any]:
        """Serialize synapse data for saving."""
        try:
            synapses = []

            # ✅ Get all synapses from Rust NPU (single source of truth)
            if self._npu_interface:
                neuron_count = self._npu_interface.rust_npu.get_neuron_count()
                for neuron_id in range(neuron_count):
                    if not self._npu_interface.neuron_exists(neuron_id):
                        continue
                    outgoing = self.get_outgoing_connections(neuron_id)
                    for target_id, weight in outgoing:
                        synapses.append(
                            {
                                "pre_neuron_id": neuron_id,
                                "post_neuron_id": target_id,
                                "weight": weight,
                            }
                        )

            return {"synapses": synapses}

        except Exception as e:
            logger.error(f"Failed to serialize synapse data: {e}")
            return {}

    def _deserialize_neuron_data(self, data: Dict[str, Any]) -> bool:
        """Deserialize neuron data from saved state."""
        try:
            neuron_data = data.get("neurons", {})
            next_neuron_id = data.get("next_neuron_id", 1)

            logger.info(f"Deserializing {len(neuron_data)} neurons...")

            # Restore neurons
            for neuron_id_str, neuron_props in neuron_data.items():
                neuron_id = int(neuron_id_str)

                # ✅ Check neuron existence via Rust NPU (single source of truth)
                if not self.has_neuron(neuron_id):
                    logger.warning(
                        f"Neuron {neuron_id} not found in Rust NPU, skipping"
                    )
                    continue

                # Restore properties
                self.set_neuron_property(
                    neuron_id,
                    NeuronPropertyType.MEMBRANE_POTENTIAL,
                    neuron_props.get("membrane_potential", 0.0),
                )
                # ... other properties ...

            # ✅ REMOVED: Neuron ID tracking now in Rust NPU only
            # No need to restore Python-side counters

            return True

        except Exception as e:
            logger.error(f"Error deserializing neuron data: {e}")
            return False

    def _deserialize_synapse_data(self, synapse_data: Dict[str, Any]) -> None:
        """Deserialize synapse data after loading."""
        try:
            synapses = synapse_data.get("synapses", [])

            # Recreate synapses
            for synapse in synapses:
                self.create_synapse(
                    pre_neuron_id=synapse["pre_neuron_id"],
                    post_neuron_id=synapse["post_neuron_id"],
                    weight=synapse["weight"],
                )

        except Exception as e:
            logger.error(f"Failed to deserialize synapse data: {e}")

    def batch_voxel_to_neuron_lookup(
        self,
        cortical_id: str,
        candidate_positions: Set[Tuple[int, int, int]],
        post_synaptic_current: float = 1.0,
    ) -> List[Tuple[int, float]]:
        """Batch lookup using Rust Morton spatial hash for O(1) per-position lookup.

        Deterministically finds neurons in `cortical_id` whose
        positions match any in `candidate_positions`.
        
        Performance: O(K) where K = len(candidate_positions) with Rust Morton hash
        Fallback: O(N) where N = neurons in area (cached approach)
        """
        try:
            if not candidate_positions:
                return []
            
            # Try Rust Morton spatial hash first (O(1) per position)
            if hasattr(self, '_rust_morton_hash') and self._rust_morton_hash is not None:
                found: List[Tuple[int, float]] = []
                for x, y, z in candidate_positions:
                    neurons = self._rust_morton_hash.get_neurons_at_coordinate(cortical_id, x, y, z)
                    for neuron_id in neurons:
                        found.append((neuron_id, float(post_synaptic_current)))
                return found
            
            # Fallback: Optimized NPU-based lookup with O(N) set membership checking
            npu = getattr(self, "_npu_interface", None)
            if npu is None:
                logger.error(f"[BATCH-LOOKUP] NPU Interface required for voxel lookup")
                return []

            cortical_idx = self.get_cortical_idx_for_id(cortical_id)
            if cortical_idx is None:
                logger.error(f"[BATCH-LOOKUP] No cortical_idx found for {cortical_id}")
                return []
            
            rust_npu = npu.rust_npu
            if rust_npu is None:
                logger.error(f"[BATCH-LOOKUP] Rust NPU not available")
                return []
            
            # PERFORMANCE: Cache neuron positions per cortical area
            cache_key = f"_neuron_positions_cache_{cortical_idx}"
            if not hasattr(self, cache_key):
                neuron_positions = rust_npu.get_neuron_positions_in_cortical_area(cortical_idx)
                if not neuron_positions:
                    return []
                setattr(self, cache_key, neuron_positions)
            else:
                neuron_positions = getattr(self, cache_key)
            
            # Build target position set for O(1) membership checks
            targets = set(candidate_positions)
            
            # Single pass: match neurons to target positions
            found: List[Tuple[int, float]] = []
            
            for neuron_id, x, y, z in neuron_positions:
                position = (int(x), int(y), int(z))
                if position in targets:
                    found.append((int(neuron_id), float(post_synaptic_current)))
            
            return found

        except Exception as e:
            logger.error(f"Error in batch voxel lookup: {e}")
            logger.exception("Full stack trace:")
            return []

    def populate_morton_hash_from_existing_neurons(self):
        """Populate Rust Morton hash from all existing neurons.
        
        Called after genome loading to enable fast spatial lookups.
        """
        if self._rust_morton_hash is None:
            return
        
        try:
            npu = getattr(self, "_npu_interface", None)
            if npu is None or npu.rust_npu is None:
                return
            
            # Get all neurons from all cortical areas
            for cortical_id, area in self.cortical_areas.items():
                cortical_idx = self.get_cortical_idx_for_id(cortical_id)
                if cortical_idx is None:
                    continue
                
                # Get all neuron positions for this area
                neuron_positions = npu.rust_npu.get_neuron_positions_in_cortical_area(cortical_idx)
                
                # Add each to Morton hash
                for neuron_id, x, y, z in neuron_positions:
                    self._rust_morton_hash.add_neuron(cortical_id, int(x), int(y), int(z), int(neuron_id))
            
            logger.info(
                f"🦀 Rust Morton hash populated successfully for {len(self.cortical_areas)} cortical areas"
            )
        except Exception as e:
            logger.error(f"Failed to populate Morton hash: {e}")
    
    def clear_neuron_position_cache(self, cortical_id: Optional[str] = None):
        """Clear cached neuron positions for a cortical area or all areas.
        
        Should be called after:
        - Adding/removing neurons
        - Modifying cortical areas
        - Loading a new genome
        
        Args:
            cortical_id: Specific area to clear, or None to clear all caches
        """
        if cortical_id:
            cortical_idx = self.get_cortical_idx_for_id(cortical_id)
            if cortical_idx is not None:
                cache_key = f"_neuron_positions_cache_{cortical_idx}"
                if hasattr(self, cache_key):
                    delattr(self, cache_key)
        else:
            # Clear all position caches
            attrs_to_delete = [attr for attr in dir(self) if attr.startswith("_neuron_positions_cache_")]
            for attr in attrs_to_delete:
                try:
                    delattr(self, attr)
                except AttributeError:
                    pass
            logger.debug(f"[CACHE] Cleared {len(attrs_to_delete)} neuron position caches")
    
    # ======================================================================
    # CORTICAL AREA DIMENSION VALIDATION
    # ======================================================================

    def get_max_allowable_cortical_area_dimensions(
        self,
    ) -> Tuple[int, int, int]:
        """Get the maximum allowable cortical area dimensions based on Morton
        spatial hash limits.

        Returns:
            Tuple of (max_width, max_height, max_depth) that can be safely created
        """
        from feagi.core.state_manager import get_state_manager

        state_manager = get_state_manager()

        morton_limit = state_manager.get_morton_coordinate_limit()
        #  Morton limit is per coordinate, so cortical area dimensions must be
        #  less than this
        max_dimension = morton_limit - 1  # Leave room for 0-based indexing

        return (max_dimension, max_dimension, max_dimension)

    def validate_cortical_area_dimensions_safe(
        self, dimensions: Tuple[int, int, int]
    ) -> bool:
        """Safely validate cortical area dimensions without raising exceptions.

        Args:
            dimensions: Tuple of (width, height, depth) dimensions

        Returns:
            True if dimensions are within Morton limits, False otherwise
        """
        try:
            from feagi.core.state_manager import get_state_manager

            state_manager = get_state_manager()

            validation_result = (
                state_manager.validate_cortical_area_dimensions(dimensions)
            )
            return validation_result.is_ok
        except Exception as e:
            logger.error(f"Error validating cortical area dimensions: {e}")
            return False


    # Removed duplicate get_synapse_count/get_neuron_count/get_neurons_by_area at end of file (consolidated above)

    @property
    def max_neurons(self) -> int:
        """Get maximum neuron capacity from NPU Interface.
        
        Returns:
            Maximum number of neurons that can be stored
        """
        # ✅ Use NPU interface max_neurons directly (no proxy)
        if self._npu_interface:
            return self._npu_interface.max_neurons
        return 0

    @property
    def max_synapses(self) -> int:
        """Get maximum synapse capacity from NPU Interface.
        
        Returns:
            Maximum number of synapses that can be stored
        """
        # ARCHITECTURE: Use Rust NPU directly (no deprecated synapse_array)
        if self._npu_interface:
            return self._npu_interface.max_synapses
        return 0

    def get_neurons_by_area(self, cortical_id: str) -> Optional[List[int]]:
        """Get all neuron IDs in a cortical area by cortical_id.
        
        MEMORY AREA SUPPORT: Now checks both regular and memory neurons.
        
        Args:
            cortical_id: String identifier (e.g., "_power", "mVPmem")
            
        Returns:
            List of neuron IDs in the area (regular + memory neurons), or None if area not found
        """
        if not self._npu_interface:
            return None

        cortical_idx = self._npu_interface.get_cortical_idx_by_id(cortical_id)
        if cortical_idx is None:
            return None

        # MEMORY AREA SUPPORT: Check if this is a memory area
        is_memory_area = self.is_memory_area(cortical_id)
        
        if is_memory_area:
            # For memory areas, get memory neurons from MemoryNeuronArray
            memory_neurons = self._get_memory_neurons_by_cortical_area(cortical_id, cortical_idx)
            return memory_neurons if memory_neurons else []
        
        # Regular areas: delegate to NPU interface
        return self._npu_interface.get_neurons_by_area(cortical_idx)

    def debug_cortical_areas(self) -> Dict[str, Any]:
        """Debug method to show all cortical areas and their neuron counts.
        
        Returns:
            Dictionary with area information for debugging
        """
        if not self._npu_interface:
            return {"error": "NPU Interface not available"}
        return self._npu_interface.debug_cortical_areas()

    def _create_neuron_via_npu(
        self,
        cortical_idx: int,
        position: Tuple[int, int, int],
        threshold: float,
        membrane_potential: float,
        resting_potential: float,
        leak_coefficient: float,
        refractory_period: int,
        properties: Optional[Dict[str, Any]],
    ) -> int:
        """Create a single neuron via Rust NPU.

        ARCHITECTURE: Delegates to NPU interface which manages all neuron data.
        ConnectomeManager does not duplicate NPU's internal mappings.

        Args:
            cortical_idx: Area index
            position: (x, y, z)
            threshold: threshold
            membrane_potential: initial potential
            resting_potential: resting potential
            leak_coefficient: leak coefficient (0.0-1.0)
            refractory_period: refractory period
            properties: optional extra properties (unused here)

        Returns:
            Newly created neuron ID
        """
        if not self._npu_interface:
            raise RuntimeError("NPU interface not configured - cannot create neurons")
        
        # Create neuron through Rust NPU
        # NPU interface handles all ID generation and mapping internally
        neuron_id = self._npu_interface.rust_npu.add_neuron(
            threshold=float(threshold),
            leak_coefficient=float(leak_coefficient),
            resting_potential=float(resting_potential),
            neuron_type=0,  # Default type
            refractory_period=int(refractory_period),
            excitability=1.0,  # Default excitability
            consecutive_fire_limit=0,  # Default no limit
            snooze_period=0,  # Default no extended refractory
            cortical_area=int(cortical_idx),
            x=int(position[0]),
            y=int(position[1]),
            z=int(position[2])
        )
        
        # ARCHITECTURE: All neuron data now in Rust NPU (no Python tracking needed)
        
        # Update state manager with new neuron count
        self._update_neuron_count_only()
        
        return neuron_id

    def get_neuron_count(self) -> int:
        """Return total neuron count (regular + memory) from NPU.

        Returns:
            Total number of neurons managed by NPU (0 if unavailable)
        """
        if hasattr(self, "_npu_interface") and self._npu_interface:
            try:
                # Use NPU interface method directly (authoritative count from Rust NPU)
                return self._npu_interface.get_neuron_count()
            except Exception as e:
                self.logger.warning(f"Failed to get neuron count from NPU: {e}")
                return 0
        return 0