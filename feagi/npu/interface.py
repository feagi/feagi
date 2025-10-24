"""
NPU Interface - Rust NPU Delegation Layer

This interface delegates ALL neuron and synapse operations to the Rust NPU,
eliminating duplicate Python data structures. It acts as a thin adapter between
Python code and the Rust NPU, maintaining API compatibility while ensuring
the Rust NPU is the single source of truth.

ARCHITECTURE:
- Rust NPU: Owns the ONLY neuron_array and synapse_array (in Rust memory)
- NPUInterface: Python API that DELEGATES to Rust NPU (proxy/adapter pattern)
- ConnectomeManager: Uses NPUInterface to update neurons/synapses in Rust NPU
"""

from typing import Dict, List, Tuple, Optional, Any, Union
from enum import Enum
from dataclasses import dataclass

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class OperationResult(Enum):
    """Result codes for NPU operations."""
    SUCCESS = "success"
    PARTIAL_SUCCESS = "partial_success"
    AREA_LOCKED = "area_locked"
    CAPACITY_EXCEEDED = "capacity_exceeded" 
    INVALID_INPUT = "invalid_input"
    BACKEND_ERROR = "backend_error"


@dataclass
class BatchOperationResult:
    """Result of a batch operation."""
    result: OperationResult
    successful_count: int
    failed_indices: List[int]
    error_message: Optional[str] = None
    data: Optional[Dict[str, Any]] = None
    
    @property
    def is_success(self) -> bool:
        """Check if the operation was fully successful."""
        return self.result == OperationResult.SUCCESS
    
    @property
    def has_partial_success(self) -> bool:
        """Check if the operation had partial success."""
        return self.successful_count > 0 and len(self.failed_indices) > 0


@dataclass
class SynapseCreationRequest:
    """Request for creating synapses."""
    source_neuron_ids: List[int]
    target_neuron_ids: List[int]
    weights: List[float]
    delays: Optional[List[int]] = None
    plasticity_types: Optional[List[int]] = None
    plasticity_coefficients: Optional[List[float]] = None


@dataclass
class NeuronCreationRequest:
    """Request for batch neuron creation."""
    cortical_idx: int
    positions: List[Tuple[int, int, int]]
    neuron_types: Optional[List[int]] = None
    initial_potentials: Optional[List[float]] = None
    thresholds: Optional[List[float]] = None
    leak_coefficients: Optional[List[float]] = None
    refractory_periods: Optional[List[int]] = None
    excitabilities: Optional[List[float]] = None
    resting_potentials: Optional[List[float]] = None
    consecutive_fire_limits: Optional[List[int]] = None
    snooze_periods: Optional[List[int]] = None
    mp_charge_accumulation: Optional[List[bool]] = None  # nx-mp_acc-b gene


@dataclass
class NeuronUpdateRequest:
    """Request for updating neuron properties."""
    neuron_ids: List[int]
    property_name: str
    values: List[Union[float, int]]


class NPUInterface:
    """NPU interface that delegates to Rust NPU (single source of truth).
    
    This class acts as a thin adapter layer between Python code and the Rust NPU.
    It maintains API compatibility while ensuring all neuron and synapse data
    lives ONLY in Rust memory.
    
    CRITICAL: No duplicate Python arrays! Rust NPU is the single source of truth.
    """
    
    def __init__(self, backend=None, max_neurons: int = 100_000, 
                 max_synapses: int = 500_000, max_memory_neurons: int = 50_000):
        """Initialize NPU interface with Rust NPU immediately.
        
        CORRECT FLOW (per architecture review):
        1. Calculate neuron/synapse capacity from genome
        2. Create Rust NPU with that capacity (SoA, GPU-compatible)
        3. Genome load directly fills Rust NPU arrays
        4. NO temporary Python structures!
        """
        from feagi.npu.rust_npu_integration import RustNPUIntegration, RUST_AVAILABLE
        
        if not RUST_AVAILABLE:
            raise RuntimeError(
                "🦀 [NPU-INTERFACE] CRITICAL: Rust NPU not available! "
                "FEAGI requires Rust NPU for production use."
            )
        
        self._connectome_manager = None  # Set by ConnectomeManager
        
        # State tracking
        self.max_neurons = max_neurons
        self.max_synapses = max_synapses
        self.max_memory_neurons = max_memory_neurons
        
        # Cortical area management for compatibility (metadata only - no neuron data!)
        self.cortical_areas: Dict[int, Dict[str, Any]] = {}
        
        # ❌ REMOVED: Python neuron tracking dictionaries (architectural violation)
        # ALL neuron data lives in Rust NPU - Python is management wrapper only!
        
        # CRITICAL: Create Rust NPU immediately with proper capacity (SoA, GPU-friendly)
        # Neuroembryogenesis will directly fill the pre-allocated Rust arrays
        logger.info("🦀 [NPU-INTERFACE] Creating Rust NPU with capacity: %d neurons, %d synapses", 
                   max_neurons, max_synapses)
        
        self._rust_npu_integration = RustNPUIntegration.create_empty(
            neuron_capacity=max_neurons,
            synapse_capacity=max_synapses,
            fire_ledger_window=20
        )
        
        logger.info("✅ [NPU-INTERFACE] Rust NPU created and ready for genome load")
        logger.info(f"   Neuron capacity: {max_neurons:,}, Synapse capacity: {max_synapses:,}")
        
        # Plasticity command queue (for future plasticity support)
        self._plasticity_queue_capacity: int = 0
        self._plasticity_queue: List[Dict[str, Any]] = []
        self._plasticity_lock = None
        try:
            import threading
            self._plasticity_lock = threading.RLock()
        except Exception:
            pass
    
    def set_rust_npu_integration(self, rust_npu_integration):
        """Set the Rust NPU integration reference.
        
        Called by BurstEngine after Rust NPU is initialized.
        """
        self._rust_npu_integration = rust_npu_integration
        logger.info("🦀 [NPU-INTERFACE] Rust NPU integration connected")
    
    def set_connectome_manager(self, connectome_manager):
        """Set the ConnectomeManager reference and update Rust NPU integration."""
        self._connectome_manager = connectome_manager
        if self._rust_npu_integration:
            self._rust_npu_integration.connectome_manager = connectome_manager
            logger.info("🦀 [NPU-INTERFACE] ConnectomeManager set on Rust NPU integration")
    
    @property
    def rust_npu(self):
        """Get Rust NPU handle (for direct access)."""
        if self._rust_npu_integration is None:
            raise RuntimeError("Rust NPU not initialized yet - call set_rust_npu_integration() first")
        return self._rust_npu_integration._rust_npu
    
    # ============================================================================
    # CLEAN API LAYER - Future Rust ConnectomeManager Interface
    # ============================================================================
    # These methods define the API contract that will be used when
    # ConnectomeManager is migrated to Rust. They delegate to Rust NPU.
    
    def get_neuron_count(self) -> int:
        """Get total number of neurons in the NPU."""
        return self.rust_npu.get_neuron_count()
    
    def get_synapse_count(self) -> int:
        """Get total number of synapses in the NPU."""
        return self.rust_npu.get_synapse_count()
    
    def get_neuron_cortical_idx(self, neuron_id: int) -> Optional[int]:
        """Get the cortical area index for a neuron from Rust NPU.
        
        Args:
            neuron_id: The neuron ID to query
            
        Returns:
            Cortical area index, or None if neuron doesn't exist
        """
        try:
            return self.rust_npu.get_neuron_cortical_area(neuron_id)
        except Exception as e:
            logger.debug(f"Neuron {neuron_id} not found in Rust NPU: {e}")
            return None
    
    def get_neuron_position(self, neuron_id: int) -> Optional[Tuple[int, int, int]]:
        """Get the 3D position of a neuron from Rust NPU.
        
        Args:
            neuron_id: The neuron ID to query
            
        Returns:
            (x, y, z) position tuple, or None if neuron doesn't exist
        """
        try:
            return self.rust_npu.get_neuron_coordinates(neuron_id)
        except Exception as e:
            logger.debug(f"Neuron {neuron_id} not found in Rust NPU: {e}")
            return None
    
    def neuron_exists(self, neuron_id: int) -> bool:
        """Check if a neuron ID exists in the NPU.
        
        Args:
            neuron_id: The neuron ID to check
            
        Returns:
            True if neuron exists, False otherwise
        """
        try:
            cortical_idx = self.rust_npu.get_neuron_cortical_area(neuron_id)
            return cortical_idx != 65535  # INVALID_CORTICAL_IDX
        except Exception:
            return False
    
    def get_neurons_in_cortical_area(self, cortical_idx: int) -> List[int]:
        """Get all neuron IDs in a specific cortical area.
        
        ARCHITECTURE: Queries Rust NPU directly (single source of truth).
        
        Args:
            cortical_idx: The cortical area index
            
        Returns:
            List of neuron IDs in that cortical area
        """
        logger.debug(f"🔍 [NPU-API] Looking for neurons in cortical_idx={cortical_idx}")
        result = self.rust_npu.get_neurons_in_cortical_area(cortical_idx)
        logger.debug(f"🔍 [NPU-API] Found {len(result)} neurons")
        return result
    
    def get_neurons_by_area(self, cortical_idx: int) -> List[int]:
        """API compatibility: alias for get_neurons_in_cortical_area."""
        return self.get_neurons_in_cortical_area(cortical_idx)
    
    # ✅ REMOVED: neuron_array proxy - Rust NPU is the ONLY source of truth
    # Use clean API methods instead:
    #   - get_neuron_count() for neuron count
    #   - max_neurons property for capacity
    #   - rust_npu.update_cortical_area_excitability() for property updates
    #   - rust_npu.delete_neuron() for neuron deletion
    
    @property
    def synapse_array(self):
        """DEPRECATED: Temporary compatibility property.
        
        Returns an object that provides ONLY synapse_count and max_synapses.
        All other synapse operations should use the clean API methods.
        This will be removed once ConnectomeManager is fully migrated to clean APIs.
        """
        logger.debug("[DEPRECATED] synapse_array property accessed - migrate to clean API methods")
        
        class _MinimalSynapseArrayProxy:
            def __init__(self, npu_interface):
                self._npu = npu_interface
            
            @property
            def synapse_count(self):
                if self._npu._rust_npu_integration and self._npu._rust_npu_integration._rust_npu_initialized:
                    return self._npu.rust_npu.get_synapse_count()
                return 0
            
            @property
            def count(self):
                return self.synapse_count
            
            @property
            def max_synapses(self):
                """Return the maximum synapse capacity."""
                return self._npu.max_synapses
            
            def get_outgoing_connections(self, source_neuron_id: int) -> List[Tuple[int, int]]:
                """Get all outgoing synapses from a source neuron.
                
                Args:
                    source_neuron_id: ID of the source neuron
                
                Returns:
                    List of tuples (target_neuron_id, weight)
                """
                if not self._npu._rust_npu_integration or not self._npu._rust_npu_integration._rust_npu_initialized:
                    return []
                
                # Read from Rust NPU (single source of truth)
                try:
                    return self._npu.rust_npu.get_outgoing_synapses(source_neuron_id)
                except Exception as e:
                    logger.error(f"🦀 [SYNAPSE-READ] Failed to get outgoing synapses for neuron {source_neuron_id}: {e}")
                    return []
            
            def has_synapse(self, source_neuron_id: int, target_neuron_id: int) -> bool:
                """Check if a synapse exists between two neurons.
                
                Args:
                    source_neuron_id: ID of the source neuron
                    target_neuron_id: ID of the target neuron
                
                Returns:
                    True if synapse exists, False otherwise
                """
                if not self._npu._rust_npu_integration or not self._npu._rust_npu_integration._rust_npu_initialized:
                    return False
                
                # Read from Rust NPU (single source of truth)
                try:
                    outgoing = self._npu.rust_npu.get_outgoing_synapses(source_neuron_id)
                    # outgoing is a list of (target_id, weight) tuples
                    return any(target_id == target_neuron_id for target_id, _ in outgoing)
                except Exception as e:
                    logger.error(f"🦀 [SYNAPSE-CHECK] Failed to check synapse {source_neuron_id} -> {target_neuron_id}: {e}")
                    return False
            
            def delete_synapse(self, source_neuron_id: int, target_neuron_id: int) -> bool:
                """Delete a synapse between two neurons via Rust NPU.
                
                This delegates to Rust NPU's remove_synapse() which performs soft deletion
                by marking the synapse as invalid in the valid_mask. The synapse is
                immediately invisible to all lookups and burst processing.
                
                Args:
                    source_neuron_id: ID of the source neuron
                    target_neuron_id: ID of the target neuron
                
                Returns:
                    True if synapse was deleted, False if not found or NPU not initialized
                """
                if not self._npu._rust_npu_integration or not self._npu._rust_npu_integration._rust_npu_initialized:
                    logger.warning(
                        f"🦀 [SYNAPSE-DELETE] Rust NPU not initialized - cannot delete synapse: "
                        f"{source_neuron_id} -> {target_neuron_id}"
                    )
                    return False
                
                try:
                    # Call Rust NPU remove_synapse method (soft deletion via valid_mask)
                    success = self._npu.rust_npu.remove_synapse(source_neuron_id, target_neuron_id)
                    
                    if success:
                        logger.debug(
                            f"🦀 [SYNAPSE-DELETE] ✅ Deleted synapse: {source_neuron_id} -> {target_neuron_id}"
                        )
                    else:
                        logger.debug(
                            f"🦀 [SYNAPSE-DELETE] ⚠️  Synapse not found: {source_neuron_id} -> {target_neuron_id}"
                        )
                    
                    return success
                except Exception as e:
                    logger.error(
                        f"🦀 [SYNAPSE-DELETE] ❌ Error deleting synapse {source_neuron_id} -> {target_neuron_id}: {e}"
                    )
                    return False
            
            def remove_synapses_from_sources(self, source_neuron_ids: List[int]) -> int:
                """SIMD-optimized batch removal: delete all synapses from specified sources.
                
                This method is 50-100x faster than looping through individual deletions.
                Optimized for cortical mapping removal where you want to delete all
                connections from a set of neurons.
                
                Args:
                    source_neuron_ids: List of source neuron IDs
                
                Returns:
                    Number of synapses deleted
                """
                if not self._npu._rust_npu_integration or not self._npu._rust_npu_integration._rust_npu_initialized:
                    logger.warning("🦀 [BATCH-DELETE] Rust NPU not initialized - cannot delete synapses")
                    return 0
                
                try:
                    deleted_count = self._npu.rust_npu.remove_synapses_from_sources(source_neuron_ids)
                    return deleted_count
                except Exception as e:
                    logger.error(f"🦀 [BATCH-DELETE] Error in batch deletion: {e}")
                    return 0
            
            def remove_synapses_between(self, source_neuron_ids: List[int], target_neuron_ids: List[int]) -> int:
                """SIMD-optimized batch removal: delete synapses between source and target sets.
                
                Uses bit-vector filtering for O(1) target membership testing.
                Optimal for both few→many (e.g., 1 → 16K) and many→many deletion patterns.
                
                Performance: 20-100x faster than nested loop deletions
                
                Args:
                    source_neuron_ids: List of source neuron IDs
                    target_neuron_ids: List of target neuron IDs
                
                Returns:
                    Number of synapses deleted
                """
                if not self._npu._rust_npu_integration or not self._npu._rust_npu_integration._rust_npu_initialized:
                    logger.warning("🦀 [BATCH-DELETE] Rust NPU not initialized - cannot delete synapses")
                    return 0
                
                try:
                    deleted_count = self._npu.rust_npu.remove_synapses_between(
                        source_neuron_ids, target_neuron_ids
                    )
                    return deleted_count
                except Exception as e:
                    logger.error(f"🦀 [BATCH-DELETE] Error in batch deletion: {e}")
                    return 0
        
        return _MinimalSynapseArrayProxy(self)
    
    def _infer_dimensions(self, positions: List[Tuple[int, int, int]], total_count: int) -> Tuple[int, int, int]:
        """Infer cortical area dimensions from position list.
        
        Temporary helper until neuroembryogenesis passes dimensions directly.
        """
        if not positions:
            return (1, 1, 1)
        
        max_x = max(p[0] for p in positions) + 1
        max_y = max(p[1] for p in positions) + 1
        max_z = max(p[2] for p in positions) + 1
        
        return (max_x, max_y, max_z)
    
    def create_neurons_batch(self, request: NeuronCreationRequest) -> BatchOperationResult:
        """Create neurons by directly calling Rust NPU."""
        try:
            if self._rust_npu_integration is None:
                raise RuntimeError(
                    "🦀 [NPU-INTERFACE] CRITICAL: _rust_npu_integration is None!"
                )
            
            if not self._rust_npu_integration._rust_npu_initialized:
                raise RuntimeError(
                    "🦀 [NPU-INTERFACE] CRITICAL: Rust NPU not initialized (_rust_npu_initialized=False)!"
                )
            
            # Validate required parameters
            if request.thresholds is None:
                raise ValueError("thresholds required")
            if request.leak_coefficients is None:
                raise ValueError("leak_coefficients required")
            if request.resting_potentials is None:
                raise ValueError("resting_potentials required")
            
            # Extract neuron count and prepare arrays
            # ✅ CORRECT ARCHITECTURE: Rust generates ALL coordinates and properties
            # Python passes ONLY scalar parameters - NO arrays, NO loops!
            # This eliminates the 4-second PyO3 array conversion bottleneck
            
            # Calculate dimensions from positions (Python still orchestrates, but doesn't build arrays)
            n = len(request.positions)
            
            # For now, assume positions represent width×height×depth grid
            # Extract defaults from request (scalars only!)
            default_threshold = float(request.thresholds[0]) if request.thresholds else 1.0
            default_leak = float(request.leak_coefficients[0]) if request.leak_coefficients else 0.0
            default_resting = float(request.resting_potentials[0]) if request.resting_potentials else 0.0
            default_type = int(request.neuron_types[0]) if request.neuron_types else 0
            default_refractory = int(request.refractory_periods[0]) if request.refractory_periods else 0
            default_excitability = float(request.excitabilities[0]) if request.excitabilities else 1.0
            default_fire_limit = int(request.consecutive_fire_limits[0]) if request.consecutive_fire_limits else 0
            default_snooze = int(request.snooze_periods[0]) if request.snooze_periods else 0
            default_mp_acc = bool(request.mp_charge_accumulation[0]) if request.mp_charge_accumulation else True
            
            # Calculate grid dimensions (assumes positions are in order)
            # This is temporary - ideally neuroembryogenesis should pass dimensions directly
            width, height, depth = self._infer_dimensions(request.positions, n)
            neurons_per_voxel = 1  # Default, will be configurable
            
            logger.debug(f"🦀 [NPU-INTERFACE] Creating {n} neurons for cortical area {request.cortical_idx} "
                        f"(dims: {width}×{height}×{depth})")
            
            import time
            start_time = time.perf_counter()
            
            # ✅ SINGLE RUST CALL with scalars only - NO array passing!
            success_count = self.rust_npu.create_cortical_area_neurons(
                cortical_idx=int(request.cortical_idx),
                width=width,
                height=height,
                depth=depth,
                neurons_per_voxel=neurons_per_voxel,
                default_threshold=default_threshold,
                default_leak_coefficient=default_leak,
                default_resting_potential=default_resting,
                default_neuron_type=default_type,
                default_refractory_period=default_refractory,
                default_excitability=default_excitability,
                default_consecutive_fire_limit=default_fire_limit,
                default_snooze_period=default_snooze,
                default_mp_charge_accumulation=default_mp_acc,
            )
            
            rust_call_time = time.perf_counter() - start_time
            
            logger.debug("🦀 [NPU-INTERFACE] Created %d neurons in %.3fs (Rust generated all coordinates, total: %d)", 
                        success_count, rust_call_time,
                        self.rust_npu.get_neuron_count())
            
            return BatchOperationResult(
                result=OperationResult.SUCCESS,
                successful_count=success_count,
                failed_indices=[]
            )
            
        except Exception as e:
            logger.error(f"[NPU-INTERFACE] Batch neuron creation failed: {e}")
            logger.exception("Full traceback:")
            return BatchOperationResult(
                result=OperationResult.BACKEND_ERROR,
                successful_count=0,
                failed_indices=list(range(len(request.positions))),
                error_message=str(e)
            )

    def create_synapses_batch(self, request: SynapseCreationRequest) -> BatchOperationResult:
        """Create synapses using SIMD-optimized batch method in Rust NPU.
        
        This method is 50-100x faster than the old Python loop approach because:
        - Single Python→Rust FFI call (vs N calls)
        - Contiguous SoA memory writes in Rust
        - Batch source_index updates
        - Cache-friendly sequential access patterns
        """
        try:
            if self._rust_npu_integration is None:
                return BatchOperationResult(
                    result=OperationResult.BACKEND_ERROR,
                    successful_count=0,
                    failed_indices=list(range(len(request.source_neuron_ids)))
                )
            
            # Validate neurons exist (Python-side pre-check to provide better error info)
            valid_indices = []
            failed_indices = []
            
            for i in range(len(request.source_neuron_ids)):
                source = int(request.source_neuron_ids[i])
                target = int(request.target_neuron_ids[i])
                
                # ARCHITECTURE: Check neuron existence via Rust NPU
                if not self.neuron_exists(source) or not self.neuron_exists(target):
                    failed_indices.append(i)
                else:
                    valid_indices.append(i)
            
            if not valid_indices:
                logger.warning("[BATCH-CREATE] No valid synapses to create after validation")
                return BatchOperationResult(
                    result=OperationResult.BACKEND_ERROR,
                    successful_count=0,
                    failed_indices=failed_indices
                )
            
            # Prepare arrays for valid synapses only
            sources = [int(request.source_neuron_ids[i]) for i in valid_indices]
            targets = [int(request.target_neuron_ids[i]) for i in valid_indices]
            weights = [int(max(0, min(255, request.weights[i]))) for i in valid_indices]
            conductances = weights.copy()  # Same as weights for now
            synapse_types = [0] * len(valid_indices)  # 0 = excitatory
            
            # ✅ SIMD-OPTIMIZED BATCH CREATION: Single Rust call for all synapses
            successful_count, rust_failed_indices = self.rust_npu.add_synapses_batch(
                sources, targets, weights, conductances, synapse_types
            )
            
            # Map Rust-side failures back to original indices
            for rust_idx in rust_failed_indices:
                original_idx = valid_indices[rust_idx]
                failed_indices.append(original_idx)
            
            # Rebuild synapse indexes after batch creation
            if successful_count > 0:
                try:
                    self.rust_npu.rebuild_indexes()
                except Exception as e:
                    logger.error(f"Failed to rebuild synapse indexes: {e}")
            
            return BatchOperationResult(
                result=OperationResult.SUCCESS if len(failed_indices) == 0 else OperationResult.PARTIAL_SUCCESS,
                successful_count=successful_count,
                failed_indices=sorted(failed_indices)
            )
            
        except Exception as e:
            logger.error(f"Batch synapse creation failed: {e}")
            return BatchOperationResult(
                result=OperationResult.BACKEND_ERROR,
                successful_count=0,
                failed_indices=list(range(len(request.source_neuron_ids))),
                error_message=str(e)
            )
    
    def get_property(self, neuron_id: int, property_name: str) -> Any:
        """Get neuron property from Rust NPU."""
        if self._rust_npu_integration is None:
            logger.warning("[NPU-INTERFACE] Rust NPU not ready - cannot get property")
            return None
        
        # Use get_neuron_state to retrieve properties
        try:
            state = self.rust_npu.get_neuron_state(neuron_id)
            if state is None:
                return None
            
            # Unpack state: (cfc, cfc_limit, snooze_period, potential, threshold, refrac_countdown)
            # Note: snooze_countdown removed - now unified in refractory_countdown
            cfc, cfc_limit, snooze_period, potential, threshold, refrac_countdown = state
            
            property_map = {
                'membrane_potential': potential,
                'threshold': threshold,
                'refractory_countdown': refrac_countdown,
                'consecutive_fire_count': cfc,
                'consecutive_fire_limit': cfc_limit,
                'snooze_period': snooze_period,
                # Note: snooze_countdown no longer available as separate field (unified in refractory_countdown)
            }
            
            return property_map.get(property_name)
            
        except Exception as e:
            logger.error(f"[NPU-INTERFACE] Failed to get property {property_name} for neuron {neuron_id}: {e}")
            return None
    
    def set_neuron_property(self, neuron_id: int, property_name: str, value: Any):
        """Set neuron property in Rust NPU.
        
        Note: Most properties can only be set during neuron creation.
        This is a compatibility method - actual updates should go through
        parameter_updater.py which triggers Rust NPU reinitialization.
        """
        logger.warning(f"[NPU-INTERFACE] set_neuron_property called for {property_name} - this should go through parameter_updater for Rust NPU reload")
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get NPU statistics from Rust NPU."""
        if self._rust_npu_integration is None:
            return {
                    'neuron_count': 0,
                    'synapse_count': 0,
                    'utilization': {'neurons': 0.0, 'synapses': 0.0}
                }
        
        try:
            return {
                'neuron_count': self.rust_npu.get_neuron_count(),
                'synapse_count': self.rust_npu.get_synapse_count(),
                'utilization': {
                    'neurons': self.rust_npu.get_neuron_count() / self.max_neurons,
                    'synapses': self.rust_npu.get_synapse_count() / self.max_synapses
                }
            }
        except Exception as e:
            logger.error(f"[NPU-INTERFACE] Failed to get statistics: {e}")
            return {
                'neuron_count': 0,
                'synapse_count': 0,
                'utilization': {'neurons': 0.0, 'synapses': 0.0}
            }
    
    def configure_plasticity_queue(self, capacity: int) -> None:
        """Configure bounded plasticity command queue capacity.

        Note: Plasticity is handled by Rust in the future, but this method
        exists for compatibility with PlasticityService initialization.
        """
        cap = int(capacity)
        if cap < 0:
            raise ValueError("plasticity queue capacity must be >= 0")
        
        if self._plasticity_lock:
            with self._plasticity_lock:
                self._plasticity_queue_capacity = cap
                # Trim if needed
                if len(self._plasticity_queue) > cap:
                    self._plasticity_queue = self._plasticity_queue[:cap]
        else:
            self._plasticity_queue_capacity = cap
            if len(self._plasticity_queue) > cap:
                self._plasticity_queue = self._plasticity_queue[:cap]
        
        logger.debug(f"[NPU-INTERFACE] Plasticity queue capacity set to {cap}")
    
    def create_cortical_area(self, cortical_idx: int, dimensions: Tuple[int, int, int], 
                            area_type: str = "regular", cortical_id: str = "") -> bool:
        """Register a cortical area in NPUInterface tracking.
        
        This does NOT create neurons - it just registers the area for tracking.
        Neurons are created later via create_neurons_batch().
        
        Args:
            cortical_idx: Cortical area index
            dimensions: (x, y, z) dimensions
            area_type: Type of area (regular, core, etc.)
            cortical_id: String ID of the cortical area
            
        Returns:
            True if successful
        """
        self.cortical_areas[cortical_idx] = {
            'dimensions': dimensions,
            'area_type': area_type,
            'cortical_id': cortical_id
        }
        logger.debug(f"[NPU-INTERFACE] Registered cortical area {cortical_id} (idx={cortical_idx})")
        return True
    
    def get_neurons_by_area(self, cortical_idx: int) -> List[int]:
        """Get list of neuron IDs in a cortical area.
        
        ARCHITECTURE: Delegates to get_neurons_in_cortical_area (Rust NPU).
        
        Returns:
            List of neuron IDs
        """
        return self.get_neurons_in_cortical_area(cortical_idx)
    
    def get_cortical_idx_by_id(self, cortical_id: str) -> Optional[int]:
        """Get cortical index from cortical ID string.
        
        Args:
            cortical_id: String ID of cortical area (e.g., "_power", "iic000")
            
        Returns:
            Cortical index, or None if not found
        """
        for idx, area_data in self.cortical_areas.items():
            if area_data.get('cortical_id') == cortical_id:
                return idx
        return None
    
    def inject_sensory_xyzp(self, cortical_id: str, xyzp_data: List[Tuple[Tuple[int, int, int], float]]):
        """
        Inject sensory data in XYZP format into NPU.
        
        Args:
            cortical_id: String ID of cortical area (e.g., "iic400")
            xyzp_data: List of ((x, y, z), potential) tuples
        """
        # Get cortical index from ID
        cortical_idx = self.get_cortical_idx_by_id(cortical_id)
        if cortical_idx is None:
            logger.warning(f"[NPU-INTERFACE] Cannot inject to unknown cortical area: {cortical_id}")
            return
        
        # Map XYZ coordinates to neuron IDs for this cortical area
        # ARCHITECTURE: Query Rust NPU for neurons in this area, then check coordinates
        neuron_potentials = []
        neurons_in_area = self.get_neurons_in_cortical_area(cortical_idx)
        
        for (x, y, z), potential in xyzp_data:
            # Find neuron at this position in this cortical area
            for neuron_id in neurons_in_area:
                pos = self.get_neuron_position(neuron_id)
                if pos == (x, y, z):
                    neuron_potentials.append((neuron_id, potential))
                    break
        
        if not neuron_potentials:
            logger.debug(f"[NPU-INTERFACE] No neurons found for {len(xyzp_data)} XYZP coordinates in {cortical_id}")
            return
        
        # Inject into Rust NPU
        if self.rust_npu:
            try:
                # Convert to format expected by Rust: Vec<(NeuronId, potential)>
                self.rust_npu.inject_sensory_with_potentials(neuron_potentials)
                logger.debug(f"[NPU-INTERFACE] ✓ Injected {len(neuron_potentials)} neurons into {cortical_id}")
            except Exception as e:
                logger.error(f"[NPU-INTERFACE] Failed to inject sensory data: {e}", exc_info=True)
        else:
            logger.warning("[NPU-INTERFACE] No Rust NPU available for injection")
    
    # COMPATIBILITY METHODS (for legacy code that expects these)