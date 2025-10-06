"""
Clean Burst Engine for FEAGI NPU

Complete rewrite with clear separation of concerns and proper data flow:
FCL (candidates) → Fire Queue (firing) → Fire Ledger (history)
"""

from typing import Dict, List, Optional, Any, Union, Tuple
import numpy as np
from feagi.utils.logger import setup_logger
from feagi.core.state_manager import FeagiStateManager, ServiceState

from .fire_candidate_list import FireCandidateList, FCLCandidate
from .fire_queue import FireQueue, FiringNeuron
from .fire_ledger import FireLedgerInterface
from .coordinate_converter import CoordinateConverter
from .fq_sampler import FQSampler
from .fcl_injector import FCLInjector
from .simd_neural_ops import simd_batch_neural_update

# Rust burst engine integration
try:
    import feagi_rust
    RUST_AVAILABLE = True
except ImportError:
    RUST_AVAILABLE = False

logger = setup_logger(__name__)


class BurstEngine:
    """Clean burst engine with proper separation of concerns."""
    
    # Lock-free singleton pattern for RTOS compliance
    _instance = None
    
    def __new__(cls, connectome_manager=None, state_manager=None, fire_ledger_window_size: int = 20):
        """Lock-free singleton pattern implementation."""
        if cls._instance is None:
            cls._instance = super(BurstEngine, cls).__new__(cls)
            cls._instance._initialized = False
            pass  # BurstEngine singleton created
        return cls._instance

    @classmethod
    def get_instance(cls, connectome_manager=None, state_manager=None, fire_ledger_window_size: int = 20):
        """Get or create the singleton instance."""
        if cls._instance is None:
            cls._instance = cls(connectome_manager, state_manager, fire_ledger_window_size)
        return cls._instance
    
    @classmethod
    def reset_instance(cls):
        """Reset the singleton instance (for testing/cleanup)."""
        cls._instance = None
    
    def __init__(self, connectome_manager=None, state_manager=None, fire_ledger_window_size: int = 20):
        """Initialize clean burst engine."""
        # Prevent double initialization for singleton
        if hasattr(self, '_initialized') and self._initialized:
            return

        self.connectome_manager = connectome_manager
        
        # Initialize logger
        self.logger = setup_logger(__name__)
        
        # STATE MANAGER INTEGRATION - Cache instance for performance 
        # Always use singleton instance as single source of truth
        self.state_manager = FeagiStateManager.instance()
        # BurstEngine using FeagiStateManager singleton
        
        # Set initial burst engine state to INITIALIZING
        try:
            # Prefer direct state manager call to avoid attribute resolution issues during import
            if self.state_manager:
                self.state_manager.set_burst_engine_state(ServiceState.INITIALIZING.value)
            else:
                # Fallback path should never happen since we cache the singleton above
                pass
        except Exception:
            # Do not block initialization on state publish
            pass
        
        # Core NPU components
        self.fire_ledger = FireLedgerInterface(fire_ledger_window_size)
        self.coordinate_converter = CoordinateConverter(connectome_manager) if connectome_manager else None
        self.fcl_injector = FCLInjector(self.coordinate_converter) if self.coordinate_converter else None
        
        # Per-area excitability cache for performance optimization (like old NPU)
        self._area_excitability_cache: Dict[int, float] = {}
        self._excitability_cache_dirty = True
        self._rng = np.random.default_rng()  # RNG for excitability (when needed)
        
        # Injection service for automatic power and special area injection
        self.injection_service = None
        self.enable_injection = True  # Enable automatic injection by default
        
        # FQ Sampler (initialized after burst engine is ready)
        self.fq_sampler: Optional[FQSampler] = None
        
        # Registry of external FQ samplers (for process manager integration)
        # Pre-allocated for RTOS compliance (max 10 FQ samplers)
        self._max_fq_samplers = 10
        self.registered_fq_samplers: List[Any] = [None] * self._max_fq_samplers
        self._fq_sampler_count = 0
        
        # Initialize frequency from state manager (single source of truth)
        try:
            state_freq = self.state_manager.get_burst_frequency() if self.state_manager else None
            if state_freq and state_freq > 0:
                self.desired_frequency = float(state_freq)
            else:
                # Use safe default and let state manager be updated by external controller
                self.desired_frequency = 10.0  # @architecture:acceptable - emergency fallback
        except Exception:
            self.desired_frequency = 10.0  # @architecture:acceptable - emergency fallback
        self.target_frequency = self.desired_frequency
        
        # Running state tracking (with debug integration)
        self._running_state = False
        
        # Genome integration tracking
        self.genome_loaded = False
        
        # Instance ID for debugging and tracking (RTOS-safe)
        self._instance_id = "burst_engine_%d" % id(self)
        
        self.current_timestep = 0
        self.burst_count = 0
        self.previous_fire_queue: Optional[FireQueue] = None
        
        # Performance monitoring for actual vs desired frequency
        self._last_burst_time = None
        self._burst_times = []  # Track last N burst times for moving average
        self._burst_times_window = 10  # Use 10 bursts for moving average
        self._performance_log_interval = 100  # Log performance every N bursts
        self._last_performance_log = 0

        # Compatibility adapter for legacy MemoryProcessor access in ConnectomeManager
        try:
            self.memory_processor = _MemoryProcessorAdapter(self)
        except Exception:
            self.memory_processor = None

        # Initialize injection service for power areas and special neurons
        try:
            import inspect
            src_path = inspect.getsourcefile(self.__class__)
            logger.info("[DEBUG] BurstEngine class loaded from: %s", src_path)
        except Exception:
            pass
        if hasattr(self, "_initialize_injection_service"):
            self._initialize_injection_service()
        else:
            logger.warning("[INJECTION] _initialize_injection_service not found on BurstEngine; skipping injection init")
        
        # Initialize Rust burst engine for high-performance synaptic propagation
        self._rust_engine = None
        self._rust_engine_initialized = False
        if RUST_AVAILABLE:
            logger.info("🦀 [RUST] Rust burst engine available - will initialize on first use")
        else:
            logger.warning("🦀 [RUST] Rust burst engine NOT available - using Python implementation")
        
        # Mark as initialized but keep in INITIALIZING state until burst processing works
        self._initialized = True
        # DON'T set to READY yet - wait until burst processing actually works

        logger.info("BurstEngine initialized with singleton pattern and state manager integration")
    
    def _initialize_rust_engine(self) -> bool:
        """Initialize the Rust synaptic propagation engine with connectome data.
        
        This is called lazily on first use to avoid overhead during initialization.
        
        Returns:
            bool: True if initialization successful, False otherwise
        """
        if self._rust_engine_initialized:
            return True
            
        if not RUST_AVAILABLE:
            logger.warning("🦀 [RUST] Cannot initialize: Rust module not available")
            return False
            
        if not self.connectome_manager:
            logger.warning("🦀 [RUST] Cannot initialize: No connectome manager")
            return False
            
        try:
            import time
            init_start = time.perf_counter()
            
            # Get NPU interface for synapse data access
            npu_interface = getattr(self.connectome_manager, '_npu_interface', None)
            if not npu_interface:
                logger.warning("🦀 [RUST] Cannot initialize: No NPU interface")
                return False
                
            synapse_array = getattr(npu_interface, 'synapse_array', None)
            neuron_array = getattr(npu_interface, 'neuron_array', None)
            if not synapse_array or not neuron_array:
                logger.warning("🦀 [RUST] Cannot initialize: No synapse/neuron arrays")
                return False
            
            # Extract synapse data as numpy arrays
            n_synapses = len(synapse_array.source_neuron_ids)
            
            source_neurons = synapse_array.source_neuron_ids.astype(np.uint32)
            target_neurons = synapse_array.target_neuron_ids.astype(np.uint32)
            weights = synapse_array.weights.astype(np.uint8)
            
            # Get conductances (with fallback to max)
            conductances_attr = getattr(synapse_array, 'conductances', None)
            if conductances_attr is not None:
                conductances = conductances_attr.astype(np.uint8)
            else:
                conductances = np.full(n_synapses, 255, dtype=np.uint8)
            
            # Get synapse types (with fallback to excitatory)
            types_attr = getattr(synapse_array, 'types', None)
            if types_attr is not None:
                types = types_attr.astype(np.uint8)
            else:
                types = np.zeros(n_synapses, dtype=np.uint8)  # 0 = excitatory
            
            # Get valid mask
            valid_mask_attr = getattr(synapse_array, 'valid_mask', None)
            if valid_mask_attr is not None:
                valid_mask = valid_mask_attr.astype(bool)
            else:
                valid_mask = np.ones(n_synapses, dtype=bool)
            
            # Create Rust engine
            self._rust_engine = feagi_rust.SynapticPropagationEngine()
            
            # Build synapse index
            self._rust_engine.build_index(
                source_neurons,
                target_neurons,
                weights,
                conductances,
                types,
                valid_mask
            )
            
            # Set neuron-to-cortical-area mapping
            neuron_to_area = getattr(npu_interface, 'neuron_to_area', {})
            n_neurons_mapped = len(neuron_to_area)
            if neuron_to_area:
                self._rust_engine.set_neuron_mapping(neuron_to_area)
            
            # Get some stats about the synapse index
            source_neuron_index = getattr(synapse_array, 'source_neuron_index', {})
            n_sources_with_synapses = len(source_neuron_index)
            
            init_time = (time.perf_counter() - init_start) * 1000
            
            self._rust_engine_initialized = True
            logger.info(
                "🦀 [RUST] Initialized synaptic propagation engine:\n"
                "   - Total synapses: %d\n"
                "   - Valid synapses: %d\n"
                "   - Source neurons with outgoing synapses: %d\n"
                "   - Neurons mapped to areas: %d\n"
                "   - Initialization time: %.2f ms",
                n_synapses, np.sum(valid_mask), n_sources_with_synapses, n_neurons_mapped, init_time
            )
            
            # DIAGNOSTIC: Check first few neurons in the index
            logger.info("🦀 [RUST-DIAGNOSTIC] First 10 source neurons with synapses: %s", list(source_neuron_index.keys())[:10])
            
            # DIAGNOSTIC: Check power neurons (ID 1 and 2)
            for power_id in [1, 2]:
                if power_id in source_neuron_index:
                    power_syn_indices = source_neuron_index[power_id]
                    # Check how many are actually valid
                    if valid_mask is not None:
                        valid_power_syns = sum(1 for idx in power_syn_indices if valid_mask[idx])
                    else:
                        valid_power_syns = len(power_syn_indices)
                    logger.info(
                        "🦀 [RUST-DIAGNOSTIC] Power neuron (ID=%d) has %d total synapses, %d valid",
                        power_id, len(power_syn_indices), valid_power_syns
                    )
                else:
                    logger.warning("🦀 [RUST-DIAGNOSTIC] Power neuron (ID=%d) has NO outgoing synapses in index!", power_id)
            
            return True
            
        except Exception as e:
            logger.error("🦀 [RUST] Initialization failed: %s", str(e), exc_info=True)
            self._rust_engine = None
            self._rust_engine_initialized = False
            return False
    
    def reinitialize_rust_engine(self) -> bool:
        """Force re-initialization of Rust engine (e.g., after connectome changes).
        
        This should be called after genome loading or synapse modifications.
        """
        logger.info("🦀 [RUST] Force re-initialization requested")
        self._rust_engine = None
        self._rust_engine_initialized = False
        return self._initialize_rust_engine()
    
    def process_burst(self) -> List[int]:
        """Execute complete burst processing with clean 5-phase workflow.
        
        RUST-COMPATIBLE: Deterministic processing with well-defined data flow.
        Uses SoA format internally for SIMD optimization.
        
        Returns:
            List[int]: Neuron IDs that fired in current timestep
        """
        return self._process_burst()
    
    def _process_burst(self) -> List[int]:
        """Internal implementation of burst processing."""
        # Process burst - no excessive logging
        
        # ALWAYS log burst processing to verify it's running
        logger.info(f"[BURST-ENGINE] Processing burst #{self.burst_count}")
        
        self.current_timestep = self.burst_count
        
        # NPU Debug logging (enabled with --debug-npu)
        debug_enabled = self.state_manager and self.state_manager.is_debug_npu_enabled()
        
        # Performance timing for bottleneck identification
        import time
        phase_times = {}
        burst_start = time.perf_counter()
        
        # Only log critical errors and major state changes
        
        # Phase 1: Collect candidates using FCL Injector
        phase1_start = time.perf_counter()
        fcl = FireCandidateList()
        
        self._inject_all_candidates(fcl)
        phase_times['phase1_injection'] = (time.perf_counter() - phase1_start) * 1000  # ms
        
                # CRITICAL: Process any accumulated external activations via FCL injector (sensory/IPU routes)
        if self.fcl_injector and hasattr(self, '_pending_external_activations'):
            try:
                import sys
                debug_mem = '--debug-mem' in sys.argv
                
                pending_activations = getattr(self, '_pending_external_activations', {})
                
                if debug_mem and pending_activations:
                    memory_areas = [area for area in pending_activations.keys() if 'memory_area' in area]
                    if memory_areas:
                        total_memory_neurons = sum(len(activations) for area, activations in pending_activations.items() if 'memory_area' in area)
                        print(f"[DEBUG-MEM] Processing {total_memory_neurons} memory neuron activations from {len(memory_areas)} memory areas")
                if pending_activations:
                    external_total = 0
                    # Create a safe copy to avoid mutation during iteration
                    pending_copy = dict(pending_activations)
                    for area_id, area_data in pending_copy.items():
                        # Coordinate dict format
                        if isinstance(area_data, dict) and 'coordinates_x' in area_data:
                            coords_x = np.asarray(area_data.get('coordinates_x', np.array([])))
                            coords_y = np.asarray(area_data.get('coordinates_y', np.array([])))
                            coords_z = np.asarray(area_data.get('coordinates_z', np.array([])))
                            potentials = np.asarray(area_data.get('membrane_potentials', np.array([])), dtype=np.float32)
                            # Guard: ensure arrays have the same length
                            if not (len(coords_x) == len(coords_y) == len(coords_z) == len(potentials)):
                                logger.error("BurstEngine: Skipping malformed external activations for area %s due to length mismatch: x=%d y=%d z=%d p=%d", area_id, len(coords_x), len(coords_y), len(coords_z), len(potentials))
                                continue
                            if len(coords_x) > 0:
                                injected = self.fcl_injector.inject_sensory_data(
                                    fcl=fcl,
                                    cortical_id=area_id,
                                    x_coords=coords_x,
                                    y_coords=coords_y,
                                    z_coords=coords_z,
                                    potentials=potentials,
                                )
                                external_total += injected
                        # Neuron ID list format
                        elif isinstance(area_data, (list, np.ndarray)) and len(area_data) > 0:
                            neuron_ids = np.array(area_data, dtype=np.uint32)
                            npu_iface = getattr(self.connectome_manager, '_npu_interface', None)
                            potentials_list = []
                            valid_neuron_ids = []
                            
                            # Check if this is a memory area (either by name pattern or by checking if neuron IDs are in memory range)
                            is_memory_area = ('memory_area' in area_id or 
                                            any(int(nid) >= 50000000 for nid in neuron_ids))
                            
                            if debug_mem and is_memory_area:
                                print(f"[DEBUG-MEM] Processing area '{area_id}' with {len(neuron_ids)} neuron IDs: {neuron_ids}")
                            
                            if npu_iface and hasattr(npu_iface, 'neuron_array') and hasattr(npu_iface.neuron_array, 'neuron_id_to_index'):
                                na = npu_iface.neuron_array
                                thresholds = getattr(na, 'thresholds', None)
                                id_to_idx = getattr(na, 'neuron_id_to_index', {})
                                
                                if debug_mem and is_memory_area:
                                    print(f"[DEBUG-MEM] Regular neuron array has {len(id_to_idx)} neurons registered")
                                
                                for nid in neuron_ids:
                                    nid_int = int(nid)
                                    if nid_int in id_to_idx and thresholds is not None:
                                        idx = id_to_idx[nid_int]
                                        if idx < len(thresholds):
                                            valid_neuron_ids.append(nid_int)
                                            potentials_list.append(float(thresholds[idx]))
                                            if debug_mem and is_memory_area:
                                                print(f"[DEBUG-MEM]   ✅ Found neuron {nid_int} in regular array at idx {idx}, threshold: {thresholds[idx]}")
                                    else:
                                        if debug_mem and is_memory_area:
                                            print(f"[DEBUG-MEM]   ❌ Memory neuron {nid_int} NOT found in regular neuron array")
                                            
                            # Check memory neuron array for memory neurons
                            if npu_iface and hasattr(npu_iface, 'memory_neuron_array') and is_memory_area:
                                mna = npu_iface.memory_neuron_array
                                if debug_mem:
                                    print(f"[DEBUG-MEM] Checking memory neuron array for memory neurons...")
                                
                                for nid in neuron_ids:
                                    nid_int = int(nid)
                                    # Memory neurons should be injected with above-threshold potential
                                    # Use memory neuron ID range from configuration
                                    memory_id_threshold = 50000000  # @architecture:acceptable - memory neuron ID range constant
                                    if nid_int >= memory_id_threshold:
                                        valid_neuron_ids.append(nid_int)
                                        # Use configuration-based potential
                                        try:
                                            from feagi.config.toml_loader import load_feagi_config
                                            config = load_feagi_config()
                                            memory_cfg = config.get('plasticity', {}).get('memory', {})
                                            firing_threshold = memory_cfg.get('firing_threshold', 1.0)
                                            injection_potential = firing_threshold + 0.5
                                        except Exception:
                                            # @architecture:acceptable - emergency fallback for memory neuron potential
                                            injection_potential = 1.5
                                        
                                        potentials_list.append(injection_potential)
                                        if debug_mem:
                                            print(f"[DEBUG-MEM]   ✅ Memory neuron {nid_int} added to FCL with potential {injection_potential}")
                                            
                            if valid_neuron_ids:
                                cortical_idx = self.connectome_manager.get_cortical_idx_for_id(area_id)
                                if cortical_idx is not None:
                                    excitatory_mask = np.ones(len(valid_neuron_ids), dtype=bool)
                                    injected = fcl.add_candidates_soa(
                                        cortical_idx=int(cortical_idx),
                                        neuron_ids=np.array(valid_neuron_ids, dtype=np.uint32),
                                        potential_deltas=np.array(potentials_list, dtype=np.float32),
                                        excitatory_mask=excitatory_mask,
                                    )
                                    external_total += injected
                                    
                                    if debug_mem and is_memory_area:
                                        print(f"[DEBUG-MEM] ✅ Injected {injected} memory neurons into FCL for cortical area {cortical_idx}")
                                        # Immediately verify the injection worked
                                        try:
                                            total_after_injection = fcl.get_total_candidate_count()
                                            print(f"[DEBUG-MEM] FCL candidate count after injection: {total_after_injection}")
                                        except:
                                            pass
                                else:
                                    if debug_mem and is_memory_area:
                                        print(f"[DEBUG-MEM] ❌ Could not find cortical_idx for area_id '{area_id}'")
                                        print(f"[DEBUG-MEM]   Available cortical areas: {list(self.connectome_manager.cortical_areas.keys())[:10]}...")
                            else:
                                if debug_mem and is_memory_area:
                                    print(f"[DEBUG-MEM] ❌ No valid memory neurons found for FCL injection")
                    # Clear processed activations deterministically
                    self._pending_external_activations.clear()
                    if external_total > 0:
                        logger.info("BurstEngine: injected %d sensory/IPU candidates via FCLInjector", external_total)
            except Exception as ext_e:
                logger.error("BurstEngine: Error processing accumulated external activations: %s", str(ext_e))

        fcl_candidate_count = fcl.get_total_candidate_count()
        
        # Phase 2: Process neural dynamics - convert FCL candidates to actual firing neurons
        fire_queue = FireQueue()
        
        # ALWAYS log before neural dynamics
        logger.info(f"[BURST-ENGINE] About to process neural dynamics with FCL count: {fcl_candidate_count}")
        
        import sys
        debug_mem = '--debug-mem' in sys.argv
        if debug_mem:
            print(f"[DEBUG-MEM] FCL has {fcl_candidate_count} candidates before neural dynamics")
            
            # Check if memory neuron 50000000 is in the FCL
            try:
                memory_neuron_found_in_fcl = False
                
                # Method 1: Check specific area (25) where memory neuron should be
                if hasattr(fcl, 'get_candidates_by_area'):
                    try:
                        print(f"[DEBUG-MEM] FCL checking: Method 1 - get_candidates_by_area(25)")
                        # Check area 25 specifically (memory area)
                        candidates = fcl.get_candidates_by_area(25)
                        print(f"[DEBUG-MEM] FCL Method 1: candidates = {candidates}")
                        if candidates:
                            # Handle list of FCLCandidate objects
                            if isinstance(candidates, list):
                                neuron_ids = [candidate.neuron_id for candidate in candidates if hasattr(candidate, 'neuron_id')]
                                print(f"[DEBUG-MEM] FCL Method 1: extracted neuron_ids from FCLCandidate list = {neuron_ids}")
                                if 50000000 in neuron_ids:
                                    memory_neuron_found_in_fcl = True
                                    idx = neuron_ids.index(50000000)
                                    potential = candidates[idx].membrane_potential_delta if hasattr(candidates[idx], 'membrane_potential_delta') else 'unknown'
                                    print(f"[DEBUG-MEM] ✅ Memory neuron 50000000 found in FCL area 25 with potential {potential}")
                                else:
                                    print(f"[DEBUG-MEM] FCL Method 1: Memory neuron 50000000 NOT in extracted neuron_ids")
                            # Handle structure with neuron_ids attribute
                            elif hasattr(candidates, 'neuron_ids'):
                                neuron_ids = candidates.neuron_ids
                                print(f"[DEBUG-MEM] FCL Method 1: neuron_ids = {list(neuron_ids) if hasattr(neuron_ids, '__iter__') else neuron_ids}")
                                if 50000000 in neuron_ids:
                                    memory_neuron_found_in_fcl = True
                                    idx = list(neuron_ids).index(50000000)
                                    potential = candidates.potential_deltas[idx] if hasattr(candidates, 'potential_deltas') else 'unknown'
                                    print(f"[DEBUG-MEM] ✅ Memory neuron 50000000 found in FCL area 25 with potential {potential}")
                                else:
                                    print(f"[DEBUG-MEM] FCL Method 1: Memory neuron 50000000 NOT in neuron_ids")
                            else:
                                print(f"[DEBUG-MEM] FCL Method 1: Candidates structure not recognized: {type(candidates)}")
                        else:
                            print(f"[DEBUG-MEM] FCL Method 1: No candidates returned")
                    except Exception as e:
                        print(f"[DEBUG-MEM] Error checking FCL area 25: {e}")
                        import traceback
                        traceback.print_exc()
                else:
                    print(f"[DEBUG-MEM] FCL Method 1: FCL has no get_candidates_by_area method")
                
                # Method 2: Use candidates_by_area property
                if not memory_neuron_found_in_fcl and hasattr(fcl, 'candidates_by_area'):
                    try:
                        candidates_by_area = fcl.candidates_by_area
                        for area_idx, candidates in candidates_by_area.items():
                            if hasattr(candidates, 'neuron_ids'):
                                neuron_ids = candidates.neuron_ids
                                if 50000000 in neuron_ids:
                                    memory_neuron_found_in_fcl = True
                                    idx = list(neuron_ids).index(50000000)
                                    potential = candidates.potential_deltas[idx] if hasattr(candidates, 'potential_deltas') else 'unknown'
                                    print(f"[DEBUG-MEM] ✅ Memory neuron 50000000 found in FCL area {area_idx} with potential {potential} (property)")
                                    break
                    except Exception as e:
                        print(f"[DEBUG-MEM] Error using candidates_by_area property: {e}")
                
                # Method 3: Use get_all_candidates
                if not memory_neuron_found_in_fcl and hasattr(fcl, 'get_all_candidates'):
                    try:
                        all_candidates = fcl.get_all_candidates()
                        if hasattr(all_candidates, 'neuron_ids'):
                            if 50000000 in all_candidates.neuron_ids:
                                memory_neuron_found_in_fcl = True
                                print(f"[DEBUG-MEM] ✅ Memory neuron 50000000 found in FCL all candidates")
                    except Exception as e:
                        print(f"[DEBUG-MEM] Error using get_all_candidates: {e}")
                
                if not memory_neuron_found_in_fcl:
                    print(f"[DEBUG-MEM] ❌ Memory neuron 50000000 NOT found in FCL candidates")
                    # Try to get more detailed info
                    try:
                        if hasattr(fcl, 'get_candidate_count_by_area'):
                            area_counts = fcl.get_candidate_count_by_area()
                            print(f"[DEBUG-MEM] FCL candidate counts by area: {dict(list(area_counts.items())[:5])}...")
                    except:
                        pass
                    
            except Exception as e:
                print(f"[DEBUG-MEM] Could not check FCL contents: {e}")
        
        # CRITICAL: Apply plasticity ops BEFORE neural dynamics to inject memory neurons into FCL
        try:
            import sys
            debug_mem = '--debug-mem' in sys.argv
            
            npu_interface = getattr(self.connectome_manager, '_npu_interface', None)
            if npu_interface and hasattr(npu_interface, 'apply_plasticity_ops'):
                # Get plasticity config from TOML configuration
                try:
                    from feagi.config.toml_loader import load_feagi_config
                    config = load_feagi_config()
                    p_cfg = config.get('plasticity', {})
                    budget = p_cfg.get('max_ops_per_burst', 100)  # Default budget
                    
                    if budget > 0:
                        applied_ops = npu_interface.apply_plasticity_ops(max_ops=budget)
                        if debug_mem and applied_ops > 0:
                            print(f"[DEBUG-MEM] BurstEngine applied {applied_ops} plasticity operations BEFORE neural dynamics")
                        elif debug_mem:
                            print(f"[DEBUG-MEM] BurstEngine: No plasticity operations to apply (queue empty)")
                    elif debug_mem:
                        print(f"[DEBUG-MEM] BurstEngine: Plasticity budget is 0")
                        
                except Exception as config_error:
                    # Fallback with default budget
                    if debug_mem:
                        print(f"[DEBUG-MEM] BurstEngine: Config error ({config_error}), using default budget")
                    applied_ops = npu_interface.apply_plasticity_ops(max_ops=100)
                    if debug_mem and applied_ops > 0:
                        print(f"[DEBUG-MEM] BurstEngine applied {applied_ops} plasticity operations BEFORE neural dynamics (fallback)")
            elif debug_mem:
                print(f"[DEBUG-MEM] BurstEngine: NPU interface not available for plasticity operations")
        except Exception as e:
            if debug_mem:
                print(f"[DEBUG-MEM] ❌ BurstEngine plasticity error: {e}")
            pass

        # CRITICAL: Apply neural dynamics processing AFTER plasticity operations
        phase2_start = time.perf_counter()
        fired_neurons = self._process_neural_dynamics(fcl)
        phase_times['phase2_dynamics'] = (time.perf_counter() - phase2_start) * 1000  # ms
        
        # ALWAYS log after neural dynamics
        fired_count = len(fired_neurons) if fired_neurons else 0
        logger.info(f"[BURST-ENGINE] Neural dynamics completed: {fired_count} neurons fired")
        
        if debug_mem:
            print(f"[DEBUG-MEM] Neural dynamics result: {fired_count} neurons fired")
            if fired_neurons:
                memory_fired = [nid for nid in fired_neurons if nid >= 50000000]
                if memory_fired:
                    print(f"[DEBUG-MEM] 🔥 Memory neurons fired: {memory_fired}")
                else:
                    print(f"[DEBUG-MEM] No memory neurons fired (all fired neurons: {fired_neurons[:10]}...)")
                    # Check if memory neuron 50000000 was processed
                    if 50000000 not in fired_neurons:
                        print(f"[DEBUG-MEM] ❌ Memory neuron 50000000 did not fire - investigating neural dynamics processing")
            else:
                print(f"[DEBUG-MEM] No neurons fired at all")
        
        # Phase 2: Process neural dynamics - convert FCL candidates to actual firing neurons
        try:
            fcl_candidate_count = fcl.get_total_candidate_count()
            
            # Neural dynamics processing completed - fired_neurons contains results
            total_fired = len(fired_neurons)
            
            if fired_neurons:
                # Convert fired neuron IDs to FiringNeuron objects
                firing_neurons = self._create_firing_neurons(fired_neurons)
                # FiringNeuron objects created
                
                fire_queue.add_fired_neurons(firing_neurons, self.current_timestep)
                
                # Log significant firing activity
                if total_fired > 0:
                    logger.debug("Burst #%d: %d neurons fired", self.burst_count, total_fired)
                    
        except Exception as e:
            # CRITICAL: Set to ERROR state if burst processing fails
            self._set_burst_engine_state(ServiceState.ERROR)
            logger.error("BURST-ENGINE: Critical error in burst processing - marking as ERROR state")
            
            logger.error("Burst processing error: %s", str(e))
            # Continue with empty fire queue to maintain burst cycle
        
        # Phase 3: Archive to fire ledger
        phase3_start = time.perf_counter()
        
        if not fire_queue.is_empty():
            neurons_by_area = {}
            for area_idx, neurons in fire_queue.firing_neurons_by_area.items():
                neurons_by_area[area_idx] = neurons
            
            # Archive neurons to fire ledger
            self.fire_ledger.archive_timestep(self.current_timestep, neurons_by_area)

            # Notify plasticity service after archival (read-only compute)
            try:
                import sys
                debug_mem = '--debug-mem' in sys.argv
                
                svc = getattr(self, '_plasticity_service', None)
                if svc is not None:
                    if debug_mem:
                        print(f"[DEBUG-MEM] BurstEngine notifying PlasticityService at timestep {self.current_timestep}")
                    svc.notify_burst(self.current_timestep)
                elif debug_mem:
                    print(f"[DEBUG-MEM] ❌ No PlasticityService attached to BurstEngine!")
            except Exception as e:
                if debug_mem:
                    print(f"[DEBUG-MEM] ❌ Exception notifying PlasticityService: {e}")
                pass
            
            # Update state manager with activity counters
            neuron_count = sum(len(neurons) for neurons in neurons_by_area.values())
            
            try:
                if self.state_manager:
                    self.state_manager.increment_cumulative_activity(neuron_count)
            except Exception:
                # Don't let state manager issues break burst processing
                logger.warning("Failed to update activity counters in state manager")
        # Fire queue was empty - no archiving needed
        phase_times['phase3_archival'] = (time.perf_counter() - phase3_start) * 1000  # ms
        
        # Phase 4: FQ Sampler access ready (no action needed)
        # Phase 5: Cleanup and prepare for next burst
        phase5_start = time.perf_counter()
            
        self.previous_fire_queue = fire_queue.copy_for_propagation()

        # Plasticity operations now applied BEFORE neural dynamics (moved earlier in burst processing)

        fcl.clear()
        self.burst_count += 1
        phase_times['phase5_cleanup'] = (time.perf_counter() - phase5_start) * 1000  # ms
        
        # Calculate total burst time
        phase_times['total'] = (time.perf_counter() - burst_start) * 1000  # ms
        
        # Get fired neuron IDs for logging and return
        fired_ids = fire_queue.get_all_neuron_ids()
        
        # Log performance breakdown every 100 bursts to identify bottlenecks
        if self.burst_count % 100 == 0:
            logger.warning(
                "⏱️ [BURST-ENGINE] Performance Breakdown (Burst #%d, %d neurons fired):\n"
                "   Phase 1 (Injection):  %6.2f ms (%5.1f%%)\n"
                "   Phase 2 (Dynamics):   %6.2f ms (%5.1f%%)\n"
                "   Phase 3 (Archival):   %6.2f ms (%5.1f%%)\n"
                "   Phase 5 (Cleanup):    %6.2f ms (%5.1f%%)\n"
                "   ═══════════════════════════════════\n"
                "   TOTAL:                %6.2f ms",
                self.burst_count,
                len(fired_ids),
                phase_times['phase1_injection'], 
                (phase_times['phase1_injection'] / phase_times['total'] * 100),
                phase_times['phase2_dynamics'], 
                (phase_times['phase2_dynamics'] / phase_times['total'] * 100),
                phase_times['phase3_archival'], 
                (phase_times['phase3_archival'] / phase_times['total'] * 100),
                phase_times['phase5_cleanup'], 
                (phase_times['phase5_cleanup'] / phase_times['total'] * 100),
                phase_times['total']
            )
        
        # Return fired neuron IDs for external systems
        
        # Set to READY after successful burst processing (if not already READY)
        # This handles cases where the first burst failed but subsequent ones succeed
        current_state = None
        if self.state_manager:
            try:
                current_state = self.state_manager.get_burst_engine_state()
            except Exception:
                pass
        
        if current_state != ServiceState.READY:
            self._set_burst_engine_state(ServiceState.READY)
            if self.burst_count == 1:
                logger.info("Burst engine initialized and ready")
            else:
                logger.debug("Burst engine recovered and ready (burst #%d)", self.burst_count)
            
            # Note: Sensory stream monitors burst engine state and will start automatically
            
        return fired_ids
    
    def get_current_fire_queue(self) -> Optional[FireQueue]:
        """Get current fire queue for FQ Sampler access."""
        return self.previous_fire_queue
    
    def get_fire_ledger(self) -> FireLedgerInterface:
        """Get fire ledger interface."""
        return self.fire_ledger
    
    def initialize_fq_sampler(self, sample_frequency_hz: float = 10.0, sampling_mode: str = "visualization") -> FQSampler:
        """Initialize FQ Sampler with this burst engine as provider."""
        self.fq_sampler = FQSampler(
            fire_queue_provider=self,  # BurstEngine provides get_current_fire_queue()
            sample_frequency_hz=sample_frequency_hz,
            sampling_mode=sampling_mode
        )
        logger.debug("FQ Sampler initialized: %s @ %dHz", sampling_mode, sample_frequency_hz)
        return self.fq_sampler
    
    def get_fq_sampler(self) -> Optional[FQSampler]:
        """Get FQ Sampler instance."""
        return self.fq_sampler
    
    def register_fq_sampler(self, fq_sampler: Any):
        """Register an external FQ sampler with this burst engine.
        
        This method allows the process manager to register FQ samplers 
        that need access to the burst engine's fire queue data.
        
        Args:
            fq_sampler: FQ sampler instance (FQSampler or UnifiedFQSampler)
        """
        # Check if sampler already registered
        sampler_already_registered = False
        for i in range(self._fq_sampler_count):
            if self.registered_fq_samplers[i] == fq_sampler:
                sampler_already_registered = True
                break
        
        if not sampler_already_registered:
            if self._fq_sampler_count < self._max_fq_samplers:
                self.registered_fq_samplers[self._fq_sampler_count] = fq_sampler
                self._fq_sampler_count += 1
                sampler_id = getattr(fq_sampler, 'instance_id', 'unknown')
                logger.debug("FQ sampler [%s] registered with BurstEngine", sampler_id)
            else:
                logger.error("Cannot register FQ sampler: maximum limit reached (%d)", self._max_fq_samplers)
        else:
            logger.warning("FQ sampler already registered: %s", getattr(fq_sampler, 'instance_id', 'unknown'))
    
    def unregister_fq_sampler(self, fq_sampler: Any):
        """Unregister an external FQ sampler from this burst engine.
        
        Args:
            fq_sampler: FQ sampler instance to unregister
        """
        # Find and remove sampler using index-based approach
        found_index = -1
        for i in range(self._fq_sampler_count):
            if self.registered_fq_samplers[i] == fq_sampler:
                found_index = i
                break
        
        if found_index >= 0:
            # Shift remaining elements left to fill gap
            for i in range(found_index, self._fq_sampler_count - 1):
                self.registered_fq_samplers[i] = self.registered_fq_samplers[i + 1]
            
            # Clear last element and decrement count
            self.registered_fq_samplers[self._fq_sampler_count - 1] = None
            self._fq_sampler_count -= 1
            
            sampler_id = getattr(fq_sampler, 'instance_id', 'unknown')
            logger.info("FQ sampler [%s] unregistered from BurstEngine", sampler_id)
        else:
            logger.warning("Attempted to unregister FQ sampler that wasn't registered: %s", getattr(fq_sampler, 'instance_id', 'unknown'))
    
    def get_registered_fq_samplers(self) -> List[Any]:
        """Get list of all registered FQ samplers."""
        # Return only active samplers (RTOS-safe: no dynamic allocation)
        return self.registered_fq_samplers[:self._fq_sampler_count]
    
    # ==============================================================
    # STATE MANAGER INTEGRATION METHODS
    # ==============================================================
    
    @property
    def _running(self):
        """Get the running state with debug tracking (state manager integration)."""
        return getattr(self, "_running_state", False)

    @_running.setter
    def _running(self, value):
        """Setter for _running with debug logging and state manager integration."""
        old_value = getattr(self, "_running_state", None)
        self._running_state = value

        # Update state manager when running state changes
        if old_value != value:
            if value:
                # When starting to run, keep in INITIALIZING state until first burst completes
                # DON'T set to READY here - wait for successful burst processing
                logger.debug("BurstEngine: Started running but waiting for first successful burst to mark as READY")
            else:
                # When stopping, set state to STOPPED
                self._set_burst_engine_state(ServiceState.STOPPED)
        
        # Burst engine running state updated
    
    def _set_burst_engine_state(self, state: ServiceState):
        """Set burst engine state in the state manager."""
        try:
            if self.state_manager:
                logger.debug(f"Setting burst engine state to {state.name} ({state.value})")
                result = self.state_manager.set_burst_engine_state(state.value)
                logger.debug(f"State manager set result: {result}")
                
                # Check if the result indicates an error
                if hasattr(result, 'is_err') and result.is_err:
                    error_msg = result.unwrap_err() if hasattr(result, 'unwrap_err') else "Unknown error"
                    logger.error(f"State manager returned error: {error_msg}")
                    raise Exception(f"State manager error: {error_msg}")
                
                # Verify the state was set
                current_state = self.state_manager.get_burst_engine_state()
                logger.debug(f"Verified state after set: {current_state}")
                if current_state != state.value:
                    logger.error(f"State manager failed to update: expected {state.value}, got {current_state}")
                    raise Exception(f"State verification failed: expected {state.value}, got {current_state}")
            else:
                logger.error("No state manager available for burst engine state update")
                raise Exception("No state manager available")
        except Exception as e:
            logger.error(f"Failed to update burst engine state: {e}")
            import traceback
            logger.error(f"Full traceback: {traceback.format_exc()}")
            raise  # Re-raise to propagate the error
    
    def _notify_sensory_stream_ready(self):
        """Notify the sensory stream that the burst engine is ready."""
        try:
            # Get the ZMQ server instance from the process manager
            from feagi.process_manager import ProcessManager
            process_manager = ProcessManager.get_instance()
            if process_manager and hasattr(process_manager, '_zmq_server'):
                zmq_server = process_manager._zmq_server
                if zmq_server and hasattr(zmq_server, '_sensory') and zmq_server._sensory:
                    # Schedule the sensory stream start in the ZMQ server's event loop
                    import asyncio
                    if zmq_server._loop and zmq_server._loop.is_running():
                        asyncio.run_coroutine_threadsafe(
                            zmq_server._sensory.start_when_burst_engine_ready(),
                            zmq_server._loop
                        )
                        logger.info("🔔 Notified sensory stream that burst engine is ready")
                    else:
                        logger.debug("ZMQ server event loop not available for sensory stream notification")
                else:
                    logger.debug("Sensory stream not available for notification")
            else:
                logger.debug("Process manager or ZMQ server not available for sensory stream notification")
        except Exception as e:
            logger.debug(f"Failed to notify sensory stream: {e}")
    
    def _initialize_frequency_from_state_manager(self):
        """Initialize burst frequency from state manager (single source of truth).
        
        Uses the same robust fallback pattern as the old BurstEngine for compatibility.
        """
        # Configuration fallback frequency (mimicking old BurstEngine behavior)
        config_frequency = 10.0  # Default fallback frequency
        
        try:
            # STATE MANAGER is the SINGLE SOURCE OF TRUTH for burst frequency
            # Get frequency from state manager (authoritative source)
            state_frequency = self.state_manager.get_burst_frequency()
            if state_frequency and state_frequency > 0:
                self.desired_frequency = float(state_frequency)
                logger.info("[BURST ENGINE] Using state manager frequency: %dHz", state_frequency)
            else:
                # Emergency fallback: use config and update state manager
                self.desired_frequency = config_frequency
                self.state_manager.set_burst_frequency(config_frequency)
                logger.warning(
                    "[BURST ENGINE] State manager frequency invalid (%sHz) - using config fallback: %dHz and updating state manager", 
                    state_frequency, config_frequency
                )
        except Exception:
            # Emergency fallback: use config frequency and try to update state manager
            self.desired_frequency = config_frequency
            try:
                if self.state_manager:
                    self.state_manager.set_burst_frequency(config_frequency)
                logger.warning("[BURST ENGINE] Failed to get frequency from state manager - using config fallback: %dHz", config_frequency)
            except Exception:
                # Completely fallback - just use the frequency without updating state manager
                logger.error("[BURST ENGINE] Could not initialize frequency in state manager - using local fallback: %dHz", config_frequency)
        
        # Ensure frequency is never zero to avoid division by zero (old BurstEngine safety)
        if self.desired_frequency <= 0:
            self.desired_frequency = config_frequency
            logger.warning("[BURST ENGINE] Frequency was zero - using safety fallback: %dHz", config_frequency)
        
        # Set target_frequency for backward compatibility
        self.target_frequency = self.desired_frequency
    
    def update_with_genome(self, connectome_manager) -> bool:
        """Update burst engine with genome data and connectome manager.
        
        This method is called after genome loading to ensure the burst engine
        has access to the connectome manager and can process neural dynamics.
        """
        try:
            logger.info("🧬 [GENOME-UPDATE] Updating burst engine with new genome...")
            
            if connectome_manager:
                self.connectome_manager = connectome_manager
                logger.info(f"🧬 [GENOME-UPDATE] Connectome manager updated: {type(connectome_manager)}")
                
                # Reinitialize components that depend on connectome manager
                from feagi.npu.coordinate_converter import CoordinateConverter
                from feagi.npu.fcl_injector import FCLInjector
                
                self.coordinate_converter = CoordinateConverter(connectome_manager)
                self.fcl_injector = FCLInjector(self.coordinate_converter)
                logger.info("🧬 [GENOME-UPDATE] Coordinate converter and FCL injector reinitialized")
                
                # Initialize injection service if enabled
                if self.enable_injection:
                    try:
                        self.injection_service = PowerInjectionService(connectome_manager)
                        logger.info("🧬 [GENOME-UPDATE] Power injection service initialized")
                    except Exception as injection_error:
                        logger.warning(f"🧬 [GENOME-UPDATE] Failed to initialize injection service: {injection_error}")
                
                logger.info("🧬 [GENOME-UPDATE] ✅ Burst engine updated successfully with genome")
                return True
            else:
                logger.warning("🧬 [GENOME-UPDATE] ⚠️ No connectome manager provided")
                return False
                
        except Exception as e:
            logger.error(f"🧬 [GENOME-UPDATE] ❌ Failed to update burst engine with genome: {e}")
            import traceback
            logger.error(f"🧬 [GENOME-UPDATE] Full traceback: {traceback.format_exc()}")
            return False

    def update_frequency(self, frequency_hz: float) -> bool:
        """Update burst engine frequency and sync with state manager."""
        try:
            logger.info("🔄 [API-UPDATE] update_frequency() called with %.2fHz (current: %.2fHz)", 
                       frequency_hz, self.desired_frequency)
            
            if frequency_hz <= 0 or frequency_hz > 10000:
                logger.error("Invalid frequency %dHz (must be 0 < freq <= 10000)", frequency_hz)
                return False
            
            old_frequency = self.desired_frequency
            self.desired_frequency = frequency_hz
            
            # Update state manager
            if self.state_manager:
                self.state_manager.set_burst_frequency(frequency_hz)
                logger.info("✅ [API-UPDATE] Frequency updated: %.2fHz → %.2fHz (dynamic timing will pick up immediately)", 
                           old_frequency, frequency_hz)
            else:
                logger.warning("⚠️  [API-UPDATE] No state manager available for frequency update")
            
            return True
        except Exception as e:
            logger.error("❌ [API-UPDATE] Failed to update burst engine frequency: %s", str(e))
            return False
    
    def start(self) -> bool:
        """Start the burst engine with proper state manager integration."""
        try:
            if self._running:
                logger.warning("BurstEngine: Already running")
                return True
            
            logger.info("🔧 [START-DEBUG] Checking connectome manager availability...")
            if self.connectome_manager:
                logger.info(f"🔧 [START-DEBUG] Connectome manager available: {type(self.connectome_manager)}")
            else:
                logger.warning("🔧 [START-DEBUG] ⚠️ No connectome manager available - burst engine will run with limited functionality")
            
            logger.info("🔧 [START-DEBUG] Setting burst engine state to INITIALIZING...")
            self._set_burst_engine_state(ServiceState.INITIALIZING)
            
            self._running = True
            
            logger.info("🔧 [START-DEBUG] Setting burst engine state to READY...")
            self._set_burst_engine_state(ServiceState.READY)
            
            logger.info("BurstEngine: Started successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to start BurstEngine: {e}")
            import traceback
            logger.error(f"Full traceback: {traceback.format_exc()}")
            try:
                self._set_burst_engine_state(ServiceState.ERROR)
            except Exception as state_error:
                logger.error(f"Failed to set ERROR state: {state_error}")
            return False
    
    def stop(self) -> bool:
        """Stop the burst engine with proper state manager integration."""
        try:
            if not self._running:
                logger.warning("BurstEngine: Already stopped")
                return True
            
            self._running = False
            self._set_burst_engine_state(ServiceState.STOPPED)
            logger.info("BurstEngine: Stopped successfully")
            return True
        except Exception:
            logger.error("Failed to stop BurstEngine")
            self._set_burst_engine_state(ServiceState.ERROR)
            return False
    
    def is_running(self) -> bool:
        """Check if burst engine is currently running."""
        return self._running
    
    def run(self) -> None:
        """Start the burst engine main processing loop.
        
        This method runs the continuous burst processing loop in the current thread.
        It's designed to be called from a background thread by the brain service.
        The loop continues until stop() is called or an exit condition is met.
        """
        try:
            logger.info("🚀 [RUN-DEBUG] Burst engine run() method called - starting main processing loop")
            
            # Add detailed debug logging for startup
            logger.info("🔧 [RUN-DEBUG] About to call self.start()...")
            try:
                self.start()
                logger.info(f"🔧 [RUN-DEBUG] self.start() completed successfully - _running: {self._running}")
            except Exception as start_error:
                logger.error(f"🚨 [RUN-DEBUG] self.start() failed with error: {start_error}")
                import traceback
                logger.error(f"🚨 [RUN-DEBUG] Full traceback: {traceback.format_exc()}")
                raise  # Re-raise to be caught by outer try-catch
            
            # Enter the main processing loop
            if self._running:
                logger.info("Main loop started at %dHz", self.desired_frequency) 
                
                # CRITICAL: Re-sync frequency from state manager before starting loop
                logger.info("🔄 [STARTUP-DEBUG] About to sync frequency from state manager...")
                try:
                    if self.state_manager:
                        logger.info("🔄 [STARTUP-DEBUG] State manager available, checking frequency...")
                        state_frequency = self.state_manager.get_burst_frequency()
                        logger.info("🔄 [STARTUP-DEBUG] State manager frequency: %.2fHz, Engine frequency: %.2fHz", 
                                   state_frequency or 0.0, self.desired_frequency)
                        if state_frequency and state_frequency > 0:
                            if abs(state_frequency - self.desired_frequency) > 0.001:
                                logger.warning("🚨 [FREQUENCY-SYNC] Frequency mismatch detected! Engine: %.2fHz, State Manager: %.2fHz", 
                                              self.desired_frequency, state_frequency)
                                self.desired_frequency = float(state_frequency)
                                logger.info("🔄 [FREQUENCY-SYNC] Updated engine frequency to match state manager: %.2fHz", 
                                           self.desired_frequency)
                except Exception as e:
                    logger.error("🚨 [FREQUENCY-SYNC] Failed to sync frequency from state manager: %s", str(e))
                
                logger.info("🔄 [FREQUENCY-DEBUG] Starting main loop with frequency: %.2fHz", self.desired_frequency)
                
                # Main burst processing loop
                import time
                while self._running:
                    try:
                        # Measure actual burst timing
                        burst_start_time = time.perf_counter()
                        
                        # Execute the actual burst processing (RTOS-safe: no timing calls)
                        try:
                            fired_neurons = self.process_burst()
                            burst_process_time = time.perf_counter() - burst_start_time
                            
                            # Track actual burst frequency
                            if self._last_burst_time is not None:
                                actual_burst_interval = burst_start_time - self._last_burst_time
                                self._burst_times.append(actual_burst_interval)
                                
                                # Keep only last N burst times for moving average
                                if len(self._burst_times) > self._burst_times_window:
                                    self._burst_times.pop(0)
                            
                            self._last_burst_time = burst_start_time
                            
                            # Performance logging every N bursts
                            if self.burst_count % self._performance_log_interval == 0 and len(self._burst_times) > 0:
                                # Calculate actual frequency from moving average
                                avg_interval = sum(self._burst_times) / len(self._burst_times)
                                actual_hz = 1.0 / avg_interval if avg_interval > 0 else 0.0
                                
                                # Calculate how much time burst processing took
                                processing_percent = (burst_process_time / avg_interval * 100.0) if avg_interval > 0 else 0.0
                                
                                # Log performance
                                if actual_hz < self.desired_frequency * 0.9:  # If actual < 90% of desired
                                    logger.warning(
                                        "⚠️ [BURST-ENGINE] PERFORMANCE BOTTLENECK: Burst #%d | "
                                        "Desired: %.2f Hz | Actual: %.2f Hz (%.1f%% of target) | "
                                        "Processing time: %.1f ms (%.1f%% of interval) | "
                                        "Neurons fired: %d",
                                        self.burst_count, self.desired_frequency, actual_hz,
                                        (actual_hz / self.desired_frequency * 100.0) if self.desired_frequency > 0 else 0.0,
                                        burst_process_time * 1000.0, processing_percent,
                                        len(fired_neurons)
                                    )
                                else:
                                    logger.info(
                                        "[BURST-ENGINE] Performance: Burst #%d | "
                                        "Desired: %.2f Hz | Actual: %.2f Hz | "
                                        "Processing: %.1f ms (%.1f%% of interval) | "
                                        "Neurons: %d",
                                        self.burst_count, self.desired_frequency, actual_hz,
                                        burst_process_time * 1000.0, processing_percent,
                                        len(fired_neurons)
                                    )
                                
                        except Exception:
                            logger.error("Error in burst processing #%d", self.burst_count)
                            # Continue processing even if one burst fails
                        
                        # Timing control: Add sleep to prevent runaway CPU usage
                        try:
                            # FIXED: Calculate interval dynamically instead of using cached variable
                            current_interval = 1.0 / self.desired_frequency if self.desired_frequency > 0 else 0.1
                            time.sleep(current_interval)
                        except Exception:
                            # Fallback if config unavailable
                            time.sleep(0.1)  # @architecture:acceptable - emergency fallback
                        
                        # Check for exit condition
                        if not self._running:
                            break
                            
                    except Exception:
                        logger.error("Error in burst engine run loop")
                        break
                        
                logger.info("[BURST-ENGINE] Main processing loop ended after %d bursts", self.burst_count)
            else:
                logger.error("[BURST-ENGINE] Failed to start - run loop exiting")
                
        except Exception:
            logger.error("Error in burst engine run() method")
            self._set_burst_engine_state(ServiceState.ERROR)
    
    def update_with_genome(self, connectome_manager=None) -> None:
        """Update the burst engine configuration when a new genome is loaded.
        
        RUST-COMPATIBLE: Reinitializes all memory structures with deterministic sizing
        based on genome neuron count and configuration mode (inference vs design).
        
        This method is called after a new genome is loaded into the connectome manager 
        to refresh the engine's understanding of the neural network. This ensures 
        compatibility with the genome loading process.
        
        Args:
            connectome_manager: Optional connectome manager to use. If not provided,
                               uses the existing one or attempts to get it from the genome service.
        """
        try:
            # Updating burst engine with new genome
            
            # Accept new connectome manager or use existing one
            if connectome_manager:
                self.connectome_manager = connectome_manager
                # ConnectomeManager updated
            elif not self.connectome_manager:
                # Try to get connectome manager from the global/service registry
                try:
                    # Get the connectome manager from the API service or other global reference
                    from feagi.api.core.services.core_api_service import CoreAPIService
                    core_service = CoreAPIService.get_instance()
                    if core_service and hasattr(core_service, 'connectome_manager'):
                        self.connectome_manager = core_service.connectome_manager
                        logger.debug("ConnectomeManager retrieved from CoreAPIService: %s", type(self.connectome_manager).__name__)
                    else:
                        logger.debug("CoreAPIService not available or has no connectome_manager")
                except Exception as e:
                    logger.debug("Failed to get connectome_manager from CoreAPIService: %s", str(e))
            
            logger.info("[CONFIG] Updating burst engine with new genome")
            
            # Sync with connectome manager's current state
            if self.connectome_manager:
                logger.debug("ConnectomeManager available - starting integration")
                
                # Check if connectome manager has neuron array data
                if hasattr(self.connectome_manager, "neuron_array"):
                    neuron_array = self.connectome_manager.neuron_array
                    if hasattr(neuron_array, "neuron_count"):
                        neuron_count = neuron_array.neuron_count
                        logger.debug("Synced with %d neurons from connectome", neuron_count)
                    else:
                        logger.debug("Neuron array present but no count available")
                else:
                    logger.debug("No neuron array data available yet")
                
                # CRITICAL: Re-initialize coordinate converter with connectome manager
                if not self.coordinate_converter:
                    from .coordinate_converter import CoordinateConverter
                    self.coordinate_converter = CoordinateConverter(self.connectome_manager)
                    logger.debug("CoordinateConverter created with ConnectomeManager")
                
                # CRITICAL: Re-initialize FCL injector with connectome data
                if not self.fcl_injector and self.coordinate_converter:
                    from .fcl_injector import FCLInjector
                    self.fcl_injector = FCLInjector(self.coordinate_converter)
                    logger.debug("FCLInjector created with CoordinateConverter")
                elif self.fcl_injector:
                    logger.debug("FCLInjector already exists")
                
                # CRITICAL: Re-initialize injection service for power neurons
                # Always re-initialize to ensure it has the latest fcl_injector
                if self.fcl_injector:
                    self._initialize_injection_service()
                    logger.debug("PowerInjectionService re-initialized with updated FCLInjector")
                elif self.injection_service and hasattr(self.injection_service, 'invalidate_cache'):
                    self.injection_service.invalidate_cache()
                    logger.debug("Power neuron cache invalidated after genome update")
                else:
                    logger.debug("Cannot initialize PowerInjectionService - FCLInjector not available")
                
                # Update coordinate converter with connectome dimensions if available
                if self.coordinate_converter and hasattr(self.connectome_manager, 'get_cortical_dimensions'):
                    logger.debug("CoordinateConverter will use updated cortical dimensions")
                
                # CRITICAL: Calculate and reallocate memory structures based on genome
                self._reinitialize_memory_structures_for_genome()
                
                # Mark that genome data has been integrated
                self.genome_loaded = True
                logger.debug("Genome integration marked complete")
                
                # RUST: Reinitialize Rust engine after genome loading
                if RUST_AVAILABLE:
                    logger.info("🦀 [RUST] Reinitializing Rust engine after genome load")
                    if self.reinitialize_rust_engine():
                        logger.info("🦀 [RUST] Rust engine reinitialized successfully with new genome data")
                    else:
                        logger.warning("🦀 [RUST] Rust engine reinitialization failed - will retry on first burst")
                
            else:
                logger.debug("No connectome manager available after integration attempt")
            
            logger.info("✅ Burst engine updated with new genome successfully")
            
        except Exception as e:
            logger.error("Error updating burst engine with genome: %s", str(e))
            import traceback
            logger.error("Traceback: %s", traceback.format_exc())
            # Don't raise - genome loading should not fail due to burst engine update issues
    
    def force_connectome_integration(self) -> bool:
        """Force integration with connectome manager for debugging purposes.
        
        This method attempts to manually connect the BurstEngine to the connectome
        manager using various discovery methods. Useful for debugging integration issues.
        
        Returns:
            bool: True if integration successful, False otherwise
        """
        try:
            logger.debug("force_connectome_integration() called")
            
            if self.connectome_manager:
                logger.debug("ConnectomeManager already available: %s", type(self.connectome_manager).__name__)
                return True
            
            # Try multiple methods to get connectome manager
            methods_tried = []
            
            # Method 1: Try CoreAPIService
            try:
                from feagi.api.core.services.core_api_service import CoreAPIService
                core_service = CoreAPIService.get_instance()
                if core_service and hasattr(core_service, 'connectome_manager') and core_service.connectome_manager:
                    self.connectome_manager = core_service.connectome_manager
                    logger.debug("ConnectomeManager found via CoreAPIService: %s", type(self.connectome_manager).__name__)
                    self.update_with_genome()
                    return True
                methods_tried.append("CoreAPIService (failed)")
            except Exception as e:
                methods_tried.append("CoreAPIService (error: %s)" % str(e))
            
            # Method 2: Try GenomeService
            try:
                from feagi.api.core.services.genome.genome_service import GenomeService
                # This is harder since GenomeService is not a singleton, but let's try
                methods_tried.append("GenomeService (no singleton pattern)")
            except Exception as e:
                methods_tried.append("GenomeService (error: %s)" % str(e))
            
            # Method 3: Try global registry if available
            try:
                # Check if there's a global connectome manager registry
                import feagi.bdu.connectome_manager as cm_module
                if hasattr(cm_module, '_global_instance'):
                    self.connectome_manager = cm_module._global_instance
                    logger.debug("ConnectomeManager found via global registry")
                    self.update_with_genome()
                    return True
                methods_tried.append("Global registry (not found)")
            except Exception as e:
                methods_tried.append("Global registry (error: %s)" % str(e))
            
            logger.debug("ConnectomeManager not found. Methods tried: %s", ", ".join(methods_tried))
            return False
            
        except Exception as e:
            logger.error("Error in force_connectome_integration: %s", str(e))
            return False
    
    def _inject_all_candidates(self, fcl: FireCandidateList):
        """Inject all candidates into FCL - power neurons, sensory data, and synaptic propagation."""
        
        # Performance sub-timing for Phase 1
        import time
        sub_times = {}
        
        # NPU Debug logging
        debug_enabled = self.state_manager and self.state_manager.is_debug_npu_enabled()
        periodic_debug = debug_enabled and (self.burst_count % 500 == 0)  # Every 50 bursts
        
        if periodic_debug:
            logger.debug("FCL injection starting...")
            logger.debug("Injection service available: %s", self.injection_service is not None)
            logger.debug("Injection enabled: %s", self.enable_injection)
            logger.debug("FCL injector available: %s", self.fcl_injector is not None)
        
        # 1. CRITICAL: Inject power neurons and special areas EVERY burst
        sub_start = time.perf_counter()
        if self.injection_service and self.enable_injection:
            if periodic_debug:
                logger.debug("Starting power neuron injection...")
                
            try:
                injected_count = self.injection_service.inject_power_neurons(fcl, self.burst_count)
                # Power injection completed
                    
            except Exception as e:
                if debug_enabled:
                    logger.error("Error in power neuron injection: %s", str(e))
                    if periodic_debug:  # Only show traceback periodically
                        import traceback
                        logger.error("Power injection traceback: %s", traceback.format_exc())
                else:
                    logger.error("Error in power neuron injection")
        else:
            if periodic_debug:
                if not self.injection_service:
                    logger.debug("No injection service available - power injection skipped")
                elif not self.enable_injection:
                    logger.debug("Injection disabled - power injection skipped")
        sub_times['power_injection'] = (time.perf_counter() - sub_start) * 1000  # ms
        
        # 2. Inject sensory data (if FCL injector available)
        sub_start = time.perf_counter()
        if self.fcl_injector:
            if periodic_debug:
                logger.debug("FCL injector available - checking for sensory/synaptic data...")
                
            # Synaptic propagation from previous timestep
            if self.previous_fire_queue:
                prev_neuron_count = len(self.previous_fire_queue.get_all_neuron_ids()) if self.previous_fire_queue else 0
                
                # Debug logging for NPU debug mode
                debug_enabled = (self.state_manager and self.state_manager.is_debug_npu_enabled())
                # Process synaptic propagation
                
                propagation_data = self._compute_synaptic_propagation()
                if propagation_data:
                    injected_count = self.fcl_injector.inject_synaptic_propagation(fcl, propagation_data)
                    total_targets = sum(len(targets) for targets in propagation_data.values())
                    logger.info("Synaptic propagation: %d candidates injected from %d fired neurons → %d target neurons", 
                               injected_count, prev_neuron_count, total_targets)

            # No previous fire queue - first burst or no synaptic propagation
            pass
        else:
            # FCL injector not available - connectome initialization issue
            pass
        sub_times['synaptic_propagation'] = (time.perf_counter() - sub_start) * 1000  # ms
        
        # FCL injection phase completed - log sub-timing every 100 bursts
        if self.burst_count % 100 == 0:
            total_injection = sum(sub_times.values())
            logger.warning(
                "⏱️ [PHASE-1 BREAKDOWN] Burst #%d:\n"
                "   Power Injection:       %6.2f ms (%5.1f%%)\n"
                "   Synaptic Propagation:  %6.2f ms (%5.1f%%)\n"
                "   ───────────────────────────────────\n"
                "   Phase 1 Total:         %6.2f ms",
                self.burst_count,
                sub_times.get('power_injection', 0),
                (sub_times.get('power_injection', 0) / total_injection * 100) if total_injection > 0 else 0,
                sub_times.get('synaptic_propagation', 0),
                (sub_times.get('synaptic_propagation', 0) / total_injection * 100) if total_injection > 0 else 0,
                total_injection
            )
    
    def _compute_synaptic_propagation(self) -> Dict[int, List[tuple]]:
        """Compute synaptic propagation data from previous fire queue.
        
        RUST IMPLEMENTATION: High-performance synaptic propagation using Rust.
        Replaces the Python implementation with 50-100x faster Rust code.
        
        Returns:
            Dict[cortical_idx, List[(target_neuron_id, synaptic_contribution)]]
        """
        import time
        
        if not self.previous_fire_queue or not self.connectome_manager:
            return {}
        
        # Get all fired neuron IDs from previous timestep
        fired_neuron_ids = self.previous_fire_queue.get_all_neuron_ids()
        if not fired_neuron_ids:
            return {}
        
        # Initialize Rust engine on first use (lazy initialization)
        if not self._rust_engine_initialized:
            if not self._initialize_rust_engine():
                logger.error("🦀 [RUST] Synaptic propagation failed: Rust engine not initialized")
                return {}
        
        # Call Rust engine for high-performance propagation
        try:
            prop_start = time.perf_counter()
            
            # Convert to numpy array for Rust
            fired_neurons_array = np.array(fired_neuron_ids, dtype=np.uint32)
            
            # Call Rust (THIS IS THE FAST PATH!)
            result = self._rust_engine.propagate(fired_neurons_array)
            
            prop_time = (time.perf_counter() - prop_start) * 1000
            
            # Count total targets across all areas
            total_targets = sum(len(targets) for targets in result.values())
            
            # Log performance every 100 bursts
            if self.burst_count % 100 == 0:
                logger.info(
                    "🦀 [RUST SYNAPTIC-PROPAGATION] Burst #%d: %d neurons fired → %d target injections across %d areas → %.2f ms",
                    self.burst_count,
                    len(fired_neuron_ids),
                    total_targets,
                    len(result),
                    prop_time
                )
            
            # DIAGNOSTIC: Log when we have fired neurons but no propagation targets
            if len(fired_neuron_ids) > 0 and total_targets == 0:
                logger.warning(
                    "🦀 [RUST-DIAGNOSTIC] Burst #%d: %d neurons fired but 0 targets! Fired: %s",
                    self.burst_count,
                    len(fired_neuron_ids),
                    fired_neuron_ids[:5]  # Show first 5 fired neurons
                )
            
            return result
            
        except Exception as e:
            logger.error("🦀 [RUST] Error in synaptic propagation: %s", str(e), exc_info=True)
            return {}
    
    def _get_neuron_firing_threshold(self, neuron_id: int) -> float:
        """Get the actual firing threshold for a specific neuron.
        
        RUST-COMPATIBLE: 100% deterministic lookup. NO FALLBACKS.
        Raises error if genome data is missing - FEAGI must be deterministic.
        
        Args:
            neuron_id: The neuron ID to get threshold for
            
        Returns:
            Actual firing threshold from neuron properties (from genome)
            
        Raises:
            ValueError: If neuron data is missing from genome/NPU interface
        """
        if not self.connectome_manager:
            raise ValueError(f"Cannot get threshold for neuron {neuron_id}: No connectome manager available")
            
        npu_interface = getattr(self.connectome_manager, '_npu_interface', None)
        if not npu_interface:
            raise ValueError(f"Cannot get threshold for neuron {neuron_id}: No NPU interface in connectome manager")
            
        neuron_array = getattr(npu_interface, 'neuron_array', None)
        if not neuron_array:
            raise ValueError(f"Cannot get threshold for neuron {neuron_id}: No neuron array in NPU interface")
            
        # Get neuron index from ID - MUST exist in genome
        neuron_id_to_index = getattr(neuron_array, 'neuron_id_to_index', None)
        if not neuron_id_to_index:
            raise ValueError(f"Cannot get threshold for neuron {neuron_id}: No neuron ID mapping in genome")
            
        if neuron_id not in neuron_id_to_index:
            raise ValueError(f"Neuron {neuron_id} not found in genome neuron array - missing from neuroembryogenesis")
            
        neuron_index = neuron_id_to_index[neuron_id]
        
        # Get threshold array - MUST exist in genome
        thresholds = getattr(neuron_array, 'thresholds', None)
        if thresholds is None:
            raise ValueError(f"No firing thresholds array in genome neuron data")
            
        if neuron_index >= len(thresholds):
            raise ValueError(f"Neuron {neuron_id} index {neuron_index} out of bounds in thresholds array")
            
        actual_threshold = float(thresholds[neuron_index])
        return actual_threshold
    
    def _create_firing_neurons(self, fired_neuron_ids: List[int]) -> List[FiringNeuron]:
        """Convert fired neuron IDs to FiringNeuron objects with properties."""
        # Use list comprehension to avoid index assignment errors
        firing_neurons = []
        
        try:
            for idx, neuron_id in enumerate(fired_neuron_ids):
                # Get actual neuron properties from genome - ZERO FALLBACKS, STRICT VALIDATION
                # All values MUST come from genome via neuroembryogenesis → connectome → NPU interface
                
                if not self.connectome_manager:
                    raise ValueError(f"Cannot create FiringNeuron for {neuron_id}: No connectome manager available")
                    
                npu_interface = getattr(self.connectome_manager, '_npu_interface', None)
                if not npu_interface:
                    raise ValueError("NPU interface not initialized; cannot build firing neurons deterministically")
                
                # Get cortical area - MUST exist in genome
                neuron_to_area = getattr(npu_interface, 'neuron_to_area', None)
                if not neuron_to_area:
                    raise ValueError(f"Cannot create FiringNeuron for {neuron_id}: No neuron-to-area mapping in genome")
                if neuron_id not in neuron_to_area:
                    raise ValueError(f"Neuron {neuron_id} not mapped to any cortical area in genome")
                cortical_idx = neuron_to_area[neuron_id]
                
                # Get coordinates from neuron array - STRICT VALIDATION, NO FALLBACKS
                neuron_array = getattr(npu_interface, 'neuron_array', None)
                if not neuron_array:
                    raise ValueError(f"Cannot create FiringNeuron for {neuron_id}: No neuron array in NPU interface")
                    
                neuron_id_to_index = getattr(neuron_array, 'neuron_id_to_index', None)
                if not neuron_id_to_index:
                    raise ValueError(f"Cannot create FiringNeuron for {neuron_id}: No neuron ID mapping in genome")
                    
                if neuron_id not in neuron_id_to_index:
                    raise ValueError(f"Neuron {neuron_id} not found in genome neuron array")
                    
                neuron_index = neuron_id_to_index[neuron_id]
                
                # Get coordinates - MUST exist in genome
                coordinates_x = getattr(neuron_array, 'coordinates_x', None)
                coordinates_y = getattr(neuron_array, 'coordinates_y', None) 
                coordinates_z = getattr(neuron_array, 'coordinates_z', None)
                
                if coordinates_x is None or coordinates_y is None or coordinates_z is None:
                    raise ValueError(f"Missing coordinate arrays in neuron array for neuron {neuron_id}")
                    
                if neuron_index >= len(coordinates_x) or neuron_index >= len(coordinates_y) or neuron_index >= len(coordinates_z):
                    raise ValueError(f"Neuron {neuron_id} index {neuron_index} out of bounds in coordinate arrays")
                
                x = int(coordinates_x[neuron_index])
                y = int(coordinates_y[neuron_index])  
                z = int(coordinates_z[neuron_index])
                coordinates = (x, y, z)
                
                # Get membrane potential - MUST exist in genome
                membrane_potentials = getattr(neuron_array, 'membrane_potentials', None)
                if membrane_potentials is None:
                    raise ValueError(f"No membrane potentials array in neuron array for neuron {neuron_id}")
                    
                if neuron_index >= len(membrane_potentials):
                    raise ValueError(f"Neuron {neuron_id} index {neuron_index} out of bounds in membrane potentials array")
                    
                membrane_potential = float(membrane_potentials[neuron_index])
                # Deterministic rule: pre_fire_potential is defined as the firing threshold for that neuron.
                pre_fire_potential = self._get_neuron_firing_threshold(neuron_id)
                
                # Get threshold - MUST exist in genome  
                threshold = self._get_neuron_firing_threshold(neuron_id)
                
                # Get consecutive fire count and refractory counter - ACTUAL VALUES from NPU
                consecutive_fire_count = 0
                refractory_counter = 0
                
                if hasattr(neuron_array, 'consecutive_fire_counts') and neuron_index < len(neuron_array.consecutive_fire_counts):
                    consecutive_fire_count = int(neuron_array.consecutive_fire_counts[neuron_index])
                
                if hasattr(neuron_array, 'refractory_counters') and neuron_index < len(neuron_array.refractory_counters):
                    refractory_counter = int(neuron_array.refractory_counters[neuron_index])
                
                # Create FiringNeuron with ACTUAL neural dynamics data
                firing_neuron = FiringNeuron(
                    neuron_id=neuron_id,
                    cortical_idx=cortical_idx,
                    membrane_potential=membrane_potential,
                    pre_fire_potential=pre_fire_potential,
                    coordinates=coordinates,
                    threshold=threshold,
                    consecutive_fire_count=consecutive_fire_count,  # ACTUAL value from NPU neural dynamics
                    refractory_counter=refractory_counter,          # ACTUAL value from NPU neural dynamics
                    timestamp=0.0  # RTOS-safe: no system time calls
                )
                
                # Use append instead of index assignment to avoid range errors
                firing_neurons.append(firing_neuron)
            
            # FiringNeuron objects created successfully
            
        except Exception as e:
            logger.error("Error creating firing neurons: %s", str(e))
            # Return empty list to prevent breaking burst cycle - CRITICAL FIX
            return []
            
        return firing_neurons
    
    def _reinitialize_memory_structures_for_genome(self) -> None:
        """Reinitialize all burst engine memory structures based on genome size and configuration.
        
        RUST-COMPATIBLE: Deterministic memory allocation with pre-calculated sizes.
        Implements inference vs design mode allocation strategy.
        """
        try:
            logger.info("🧠 GENOME-MEMORY: Starting burst engine memory reinitialization")
            
            # Load burst engine configuration from TOML
            burst_config = self._load_burst_engine_config()
            
            # Calculate memory requirements based on genome and mode
            memory_requirements = self._calculate_memory_requirements(burst_config)
            
            # Log allocation strategy
            logger.info(
                f"🧠 GENOME-MEMORY: Mode={burst_config['mode']}, "
                f"Genome neurons={memory_requirements['genome_neurons']}, "
                f"FCL capacity={memory_requirements['fcl_capacity']}, "
                f"FireQueue capacity={memory_requirements['fire_queue_capacity']}"
            )
            
            # Reinitialize data structures with calculated capacities
            self._reallocate_data_structures(memory_requirements)
            
            logger.info("✅ GENOME-MEMORY: Burst engine memory structures reinitialized successfully")
            
        except Exception as e:
            logger.error(f"❌ GENOME-MEMORY: Failed to reinitialize memory structures: {e}")
            # Don't raise - allow burst engine to continue with existing structures
    
    def _load_burst_engine_config(self) -> Dict[str, Any]:
        """Load burst engine configuration from TOML with defaults.
        
        Returns:
            Dictionary with burst engine configuration parameters
        """
        try:
            from feagi.config.toml_loader import load_feagi_config
            config = load_feagi_config()
            
            # Get burst_engine section with defaults
            burst_config = config.get('burst_engine', {})
            
            # Apply defaults for missing values
            defaults = {
                'mode': 'inference',
                'max_supported_neurons': 10000000,
                'design_mode_allocation_percent': 80,
                'fcl_capacity_multiplier': 1.5,
                'fire_queue_capacity_multiplier': 1.2,
                'memory_area_multiplier': 2.0,
                'enable_preallocation': True,
                'enable_capacity_warnings': True
            }
            
            for key, default_value in defaults.items():
                if key not in burst_config:
                    burst_config[key] = default_value
            
            return burst_config
            
        except Exception as e:
            logger.warning(f"Failed to load burst engine config, using defaults: {e}")
            # Return safe defaults
            return {
                'mode': 'inference',
                'max_supported_neurons': 1000000,  # Conservative default
                'design_mode_allocation_percent': 80,
                'fcl_capacity_multiplier': 1.5,
                'fire_queue_capacity_multiplier': 1.2,
                'memory_area_multiplier': 2.0,
                'enable_preallocation': True,
                'enable_capacity_warnings': True
            }
    
    def _calculate_memory_requirements(self, burst_config: Dict[str, Any]) -> Dict[str, int]:
        """Calculate memory requirements based on genome size and configuration mode.
        
        Args:
            burst_config: Burst engine configuration from TOML
            
        Returns:
            Dictionary with calculated memory capacities for each component
        """
        # Get genome neuron count from connectome manager
        genome_neurons = self._get_genome_neuron_count()
        
        if burst_config['mode'] == 'design':
            # Design mode: Use percentage of max supported neurons
            base_neurons = int(
                burst_config['max_supported_neurons'] * 
                (burst_config['design_mode_allocation_percent'] / 100.0)
            )
            logger.info(f"🎨 DESIGN MODE: Allocating for {base_neurons} neurons ({burst_config['design_mode_allocation_percent']}% of {burst_config['max_supported_neurons']})")
        else:
            # Inference mode: Use actual genome neuron count
            base_neurons = max(genome_neurons, 1000)  # Minimum 1000 neurons
            logger.info(f"⚡ INFERENCE MODE: Allocating for {base_neurons} neurons (genome actual)")
        
        # Calculate component capacities with safety multipliers
        fcl_capacity = int(base_neurons * burst_config['fcl_capacity_multiplier'])
        fire_queue_capacity = int(base_neurons * burst_config['fire_queue_capacity_multiplier'])
        memory_area_capacity = int(base_neurons * burst_config['memory_area_multiplier'])
        
        return {
            'genome_neurons': genome_neurons,
            'base_neurons': base_neurons,
            'fcl_capacity': fcl_capacity,
            'fire_queue_capacity': fire_queue_capacity,
            'memory_area_capacity': memory_area_capacity,
            'mode': burst_config['mode']
        }
    
    def _get_genome_neuron_count(self) -> int:
        """Get the actual neuron count from the loaded genome.
        
        Returns:
            Number of neurons in the currently loaded genome
        """
        if not self.connectome_manager:
            return 0
            
        # Try multiple methods to get neuron count
        neuron_count = 0
        
        # Method 1: From neuron array
        if hasattr(self.connectome_manager, 'neuron_array'):
            neuron_array = self.connectome_manager.neuron_array
            if hasattr(neuron_array, 'neuron_count'):
                neuron_count = max(neuron_count, neuron_array.neuron_count)
        
        # Method 2: From cortical areas
        if hasattr(self.connectome_manager, 'cortical_areas'):
            cortical_areas = self.connectome_manager.cortical_areas
            if cortical_areas:
                total_area_neurons = 0
                for area_id, area in cortical_areas.items():
                    if hasattr(area, 'neuron_count'):
                        total_area_neurons += area.neuron_count
                neuron_count = max(neuron_count, total_area_neurons)
        
        # Method 3: From brain statistics (if available)
        if hasattr(self.connectome_manager, 'get_brain_statistics'):
            try:
                stats = self.connectome_manager.get_brain_statistics()
                if stats and 'neuron_count' in stats:
                    neuron_count = max(neuron_count, stats['neuron_count'])
            except Exception:
                pass  # @architecture:acceptable - statistics lookup fallback
        
        logger.debug(f"Genome neuron count determined: {neuron_count}")
        return neuron_count
    
    def _reallocate_data_structures(self, memory_requirements: Dict[str, int]) -> None:
        """Reallocate all data structures with new memory requirements.
        
        RUST-COMPATIBLE: Pre-allocates all arrays with deterministic sizes.
        
        Args:
            memory_requirements: Dictionary with calculated capacities
        """
        # Note: In the current Python implementation, we don't need to explicitly
        # reallocate FCL and FireQueue since they use dynamic containers.
        # However, we log the intended capacities for monitoring and future Rust conversion.
        
        fcl_capacity = memory_requirements['fcl_capacity']
        fire_queue_capacity = memory_requirements['fire_queue_capacity']
        
        logger.info(f"📊 MEMORY ALLOCATION: FCL capacity set to {fcl_capacity:,} candidates")
        logger.info(f"📊 MEMORY ALLOCATION: FireQueue capacity set to {fire_queue_capacity:,} neurons")
        
        # In Rust conversion, this would be:
        # self.fcl_candidates = Vec::with_capacity(fcl_capacity)
        # self.fire_queue_neurons = Vec::with_capacity(fire_queue_capacity)
        
        # For now, we store the capacities for monitoring
        self._fcl_capacity = fcl_capacity
        self._fire_queue_capacity = fire_queue_capacity
        self._memory_requirements = memory_requirements
        
        logger.debug("Memory structure capacities configured for Rust conversion readiness")
    
    def _build_excitability_cache(self) -> None:
        """Build per-area excitability cache for optimal performance.
        
        This mirrors the old NPU implementation where excitability was cached
        per cortical area instead of per neuron for better performance.
        """
        if not self.connectome_manager:
            return
            
        self._area_excitability_cache.clear()
        
        # Build cache from cortical area properties
        for area_id, area in self.connectome_manager.cortical_areas.items():
            if hasattr(area, 'properties') and area.properties:
                excitability = area.properties.get('neuron_excitability', 1.0)
            else:
                excitability = 1.0  # Default excitability
                
            cortical_idx = area.cortical_idx
            self._area_excitability_cache[cortical_idx] = float(excitability)
            
        self._excitability_cache_dirty = False
        self.logger.debug(f"Built excitability cache for {len(self._area_excitability_cache)} areas")
    
    def _any_low_excitability_areas(self) -> bool:
        """Check if any cortical areas have excitability < 0.999.
        
        This determines whether we need RNG for probabilistic firing
        or can use the fast deterministic path.
        """
        if self._excitability_cache_dirty:
            self._build_excitability_cache()
            
        return any(ex < 0.999 for ex in self._area_excitability_cache.values())
    
    def _get_excitability_tuple(self, neuron_array, valid_range: int) -> tuple:
        """Create excitability tuple in the format expected by SIMD functions.
        
        Returns tuple format: (area_ex_map, cortical_idxs, any_low_flag)
        This matches the old NPU implementation for optimal performance.
        """
        if self._excitability_cache_dirty:
            self._build_excitability_cache()
            
        cortical_idxs = neuron_array.cortical_idxs[:valid_range]
        any_low_flag = self._any_low_excitability_areas()
        
        return (self._area_excitability_cache, cortical_idxs, any_low_flag)
    
    def invalidate_excitability_cache(self) -> None:
        """Invalidate the excitability cache to force rebuild on next access.
        
        Should be called when cortical area properties change.
        """
        self._excitability_cache_dirty = True
        self.logger.debug("Excitability cache invalidated - will rebuild on next access")
    
    def _process_neural_dynamics(self, fcl: FireCandidateList) -> List[int]:
        """Process neural dynamics with SIMD-optimized operations.
        
        Applies the complete neural processing pipeline:
        1. Apply FCL candidate potentials to membrane potentials
        2. Apply membrane decay with leak behavior  
        3. Update refractory counters
        4. Check firing with consecutive fire limits
        5. Update consecutive fire counts
        6. Reset fired neurons
        
        Args:
            fcl: Fire Candidate List with candidate neurons and potentials
            
        Returns:
            List of neuron IDs that fired after neural dynamics processing
            
        Note:
            RUST-COMPATIBLE: Uses vectorized SIMD operations throughout.
            RTOS-SAFE: Deterministic execution, pre-allocated arrays.
            GPU-READY: Optimal memory access patterns.
        """
        # ALWAYS log entry to neural dynamics
        fcl_count = fcl.get_total_candidate_count() if fcl else 0
        logger.info(f"[NEURAL-DYNAMICS] ENTRY: FCL has {fcl_count} candidates")
        if not self.connectome_manager:
            logger.warning("No connectome manager available for neural dynamics processing")
            return []
            
        npu_interface = getattr(self.connectome_manager, '_npu_interface', None)
        if not npu_interface:
            # Try alternative interface access or direct method call for compatibility
            if hasattr(self.connectome_manager, 'update_membrane_potentials'):
                # Call connectome manager directly for neural processing
                try:
                    fired_neurons = self.connectome_manager.update_membrane_potentials()
                    if fired_neurons:
                        logger.info(f"[NEURAL-DYNAMICS] Connectome direct: {len(fired_neurons)} neurons fired")
                    return fired_neurons or []
                except Exception as e:
                    logger.warning(f"Direct connectome call failed: {e}")
            
            logger.warning("No NPU interface available for neural dynamics processing")
            return []
            
        neuron_array = getattr(npu_interface, 'neuron_array', None)
        if not neuron_array:
            logger.warning("No neuron array available for neural dynamics processing")
            return []
        
        # Get valid neuron range for processing
        # FIX: use correct attribute name 'neuron_count' with safe fallback to 'count'
        try:
            _valid_count = getattr(neuron_array, "neuron_count", None)
            if _valid_count is None:
                _valid_count = getattr(neuron_array, "count", 0)
            _max_neurons = getattr(neuron_array, "max_neurons", int(_valid_count))
            valid_range = min(int(_valid_count), int(_max_neurons))
        except Exception:
            valid_range = 0
        if valid_range == 0:
            return []
        
        # Step 1: Apply FCL candidate potentials to membrane potentials
        self._apply_fcl_candidates_to_membrane_potentials(fcl, neuron_array, valid_range)
        
        # Step 2: Run complete SIMD neural dynamics processing 
        try:
            # Extract neural arrays for SIMD processing (slice to valid range)
            potentials = neuron_array.membrane_potentials[:valid_range]
            thresholds = neuron_array.thresholds[:valid_range]
            decay_rates = neuron_array.decay_rates[:valid_range]
            leak_coefficients = neuron_array.leak_coefficients[:valid_range]
            resting_potentials = neuron_array.resting_potentials[:valid_range]
            refractory_periods = neuron_array.refractory_periods[:valid_range]
            refractory_counters = neuron_array.refractory_counters[:valid_range]
            consecutive_fire_counts = neuron_array.consecutive_fire_counts[:valid_range]
            consecutive_fire_limits = neuron_array.consecutive_fire_limits[:valid_range]
            valid_mask = neuron_array.valid_mask[:valid_range]
            # CRITICAL FIX: Use optimized excitability system like old NPU
            # Get excitability tuple format for optimal performance
            excitability_tuple = self._get_excitability_tuple(neuron_array, valid_range)
            
            # Determine if we need RNG based on excitability values
            # Only use RNG when there are areas with excitability < 0.999
            needs_rng = self._any_low_excitability_areas()
            rng_for_excitability = self._rng if needs_rng else None
            
            self.logger.debug(f"Excitability processing: needs_rng={needs_rng}, areas_count={len(self._area_excitability_cache)}")
            
            # Debug firing threshold checks for sensory areas
            debug_enabled = (self.state_manager and self.state_manager.is_debug_npu_enabled())
            
            # ALWAYS log this to verify neural dynamics are running
            logger.info(f"[NEURAL-DYNAMICS] Processing {valid_range} neurons, debug_enabled={debug_enabled}")
            
            if debug_enabled:
                # Check ALL neurons for high thresholds (not just first 10)
                high_threshold_indices = []
                high_threshold_count = 0
                for i in range(valid_range):
                    if valid_mask[i] and thresholds[i] > 1000:  # High threshold neurons
                        high_threshold_count += 1
                        if len(high_threshold_indices) < 5:  # Show first 5 examples
                            high_threshold_indices.append(i)
                
                logger.info(f"[NEURAL-DEBUG] Found {high_threshold_count} neurons with threshold > 1000")
                
                if high_threshold_indices:
                    logger.info("[NEURAL-DEBUG] Pre-firing check - High threshold neurons (first 5):")
                    for i in high_threshold_indices:
                        logger.info(f"  Neuron[{i}]: potential={potentials[i]:.6f}, threshold={thresholds[i]:.1f}, refractory={refractory_counters[i]}")
                
                # Also check some random neurons to see typical threshold values
                import random
                sample_indices = random.sample(range(valid_range), min(5, valid_range))
                logger.info("[NEURAL-DEBUG] Random neuron sample:")
                for i in sample_indices:
                    if valid_mask[i]:
                        logger.info(f"  Neuron[{i}]: potential={potentials[i]:.6f}, threshold={thresholds[i]:.1f}")

            # Debug: Check potentials BEFORE SIMD processing
            if debug_enabled:
                high_pot_indices = np.where((valid_mask) & (potentials > 1000))[0]
                if len(high_pot_indices) > 0:
                    logger.error(f"[PRE-SIMD-BUG] {len(high_pot_indices)} neurons have inflated potentials BEFORE SIMD!")
                    for i in high_pot_indices[:3]:
                        logger.error(f"  PRE-SIMD Neuron[{i}]: potential={potentials[i]:.6f}, threshold={thresholds[i]:.1f}")

            # CRITICAL: Handle memory neurons separately - they fire unconditionally when in FCL
            import sys
            debug_mem = '--debug-mem' in sys.argv
            
            # STEP 1: First collect ALL memory neurons in FCL (regardless of valid_range)
            memory_neurons_detected = []
            
            # Identify memory neurons in FCL (neuron IDs >= 50000000)
            if debug_mem:
                print(f"[DEBUG-MEM] Neural dynamics: Starting memory neuron detection in FCL")
                print(f"[DEBUG-MEM] Neural dynamics: FCL object = {fcl}")
                print(f"[DEBUG-MEM] Neural dynamics: FCL has get_all_candidates = {hasattr(fcl, 'get_all_candidates') if fcl else 'FCL is None'}")
            
            if fcl and hasattr(fcl, 'get_all_candidates'):
                try:
                    if debug_mem:
                        print(f"[DEBUG-MEM] Neural dynamics: Calling fcl.get_all_candidates()")
                    all_candidates = fcl.get_all_candidates()
                    if debug_mem:
                        print(f"[DEBUG-MEM] Neural dynamics: all_candidates = {all_candidates}")
                        print(f"[DEBUG-MEM] Neural dynamics: all_candidates has neuron_ids = {hasattr(all_candidates, 'neuron_ids') if all_candidates else 'all_candidates is None'}")
                    
                    # Handle dictionary format: {area_idx: [FCLCandidate, ...]}
                    if isinstance(all_candidates, dict):
                        if debug_mem:
                            print(f"[DEBUG-MEM] Neural dynamics: Processing dictionary format with {len(all_candidates)} areas")
                        for area_idx, candidates_list in all_candidates.items():
                            if debug_mem:
                                print(f"[DEBUG-MEM] Neural dynamics: Processing area {area_idx} with {len(candidates_list)} candidates")
                            for candidate in candidates_list:
                                if hasattr(candidate, 'neuron_id'):
                                    neuron_id = candidate.neuron_id
                                    if neuron_id >= 50000000:  # Memory neuron
                                        if debug_mem:
                                            print(f"[DEBUG-MEM] Neural dynamics: Found memory neuron {neuron_id} in FCL")
                                        memory_neurons_detected.append(neuron_id)
                    # Handle legacy format with neuron_ids attribute
                    elif hasattr(all_candidates, 'neuron_ids'):
                        neuron_ids = all_candidates.neuron_ids
                        if debug_mem:
                            print(f"[DEBUG-MEM] Neural dynamics: Found neuron_ids = {list(neuron_ids) if hasattr(neuron_ids, '__iter__') else neuron_ids}")
                        for neuron_id in neuron_ids:
                            if neuron_id >= 50000000:  # Memory neuron
                                if debug_mem:
                                    print(f"[DEBUG-MEM] Neural dynamics: Found memory neuron {neuron_id} in FCL")
                                memory_neurons_detected.append(neuron_id)
                    else:
                        if debug_mem:
                            print(f"[DEBUG-MEM] Neural dynamics: all_candidates format not recognized: {type(all_candidates)}")
                except Exception as e:
                    if debug_mem:
                        print(f"[DEBUG-MEM] Neural dynamics: Error identifying memory neurons in FCL: {e}")
                        import traceback
                        traceback.print_exc()
            else:
                if debug_mem:
                    print(f"[DEBUG-MEM] Neural dynamics: FCL doesn't have get_all_candidates method")
            
            # STEP 2: Update valid_range if memory neurons were detected
            if len(memory_neurons_detected) > 0:
                if debug_mem:
                    print(f"[DEBUG-MEM] Neural dynamics: Found {len(memory_neurons_detected)} memory neurons in FCL: {memory_neurons_detected}")
                
                try:
                    # Update valid_range to include newly registered memory neurons
                    _updated_count = getattr(neuron_array, "neuron_count", None)
                    if _updated_count is None:
                        _updated_count = getattr(neuron_array, "count", 0)
                    _max_neurons = getattr(neuron_array, "max_neurons", int(_updated_count))
                    updated_valid_range = min(int(_updated_count), int(_max_neurons))
                    
                    if debug_mem:
                        print(f"[DEBUG-MEM] Neural dynamics: Updating valid_range from {valid_range} to {updated_valid_range}")
                    
                    # Update valid_range for neural processing
                    if updated_valid_range > valid_range:
                        valid_range = updated_valid_range
                        if debug_mem:
                            print(f"[DEBUG-MEM] Neural dynamics: ✅ Updated valid_range to {valid_range} to include memory neurons")
                    else:
                        if debug_mem:
                            print(f"[DEBUG-MEM] Neural dynamics: No valid_range update needed (current: {valid_range}, new: {updated_valid_range})")
                            
                except Exception as e:
                    if debug_mem:
                        print(f"[DEBUG-MEM] Neural dynamics: Error updating valid_range: {e}")
            
            # STEP 3: Create memory neuron mask with updated valid_range
            memory_neuron_mask = np.zeros(valid_range, dtype=bool)
            memory_neurons_in_fcl = []
            
            # STEP 4: Now process memory neurons with the correct valid_range
            for neuron_id in memory_neurons_detected:
                idx = neuron_array.neuron_id_to_index.get(neuron_id)
                if debug_mem:
                    print(f"[DEBUG-MEM] Neural dynamics: Processing memory neuron {neuron_id} -> array index {idx}")
                    print(f"[DEBUG-MEM] Neural dynamics: valid_range = {valid_range}")
                    print(f"[DEBUG-MEM] Neural dynamics: idx is not None = {idx is not None}")
                    print(f"[DEBUG-MEM] Neural dynamics: idx < valid_range = {idx < valid_range if idx is not None else 'N/A'}")
                if idx is not None and idx < valid_range:
                    memory_neuron_mask[idx] = True
                    memory_neurons_in_fcl.append(neuron_id)
                    if debug_mem:
                        print(f"[DEBUG-MEM] Neural dynamics: ✅ Added memory neuron {neuron_id} to mask at index {idx}")
                else:
                    if debug_mem:
                        if idx is None:
                            print(f"[DEBUG-MEM] Neural dynamics: ❌ Memory neuron {neuron_id} not found in neuron_id_to_index")
                        else:
                            print(f"[DEBUG-MEM] Neural dynamics: ❌ Memory neuron {neuron_id} index {idx} >= valid_range {valid_range}")
            
            if debug_mem and len(memory_neurons_in_fcl) > 0:
                print(f"[DEBUG-MEM] Neural dynamics: Successfully processed {len(memory_neurons_in_fcl)} memory neurons for unconditional firing")
            
            # SIMD-optimized neural processing pipeline with proper excitability
            firing_mask, num_fired = simd_batch_neural_update(
                potentials=potentials,
                thresholds=thresholds, 
                decay_rates=decay_rates,
                leak_coefficients=leak_coefficients,
                resting_potentials=resting_potentials,
                refractory_periods=refractory_periods,
                refractory_counters=refractory_counters,
                consecutive_fire_counts=consecutive_fire_counts,
                consecutive_fire_limits=consecutive_fire_limits,
                valid_mask=valid_mask,
                excitability=excitability_tuple,  # Use optimized tuple format
                rng=rng_for_excitability  # RNG only when needed for probabilistic firing
            )
            
            # CRITICAL: Force memory neurons to fire unconditionally
            if np.any(memory_neuron_mask):
                memory_fired_count = np.sum(memory_neuron_mask)
                firing_mask = firing_mask | memory_neuron_mask  # Force memory neurons to fire
                num_fired = np.sum(firing_mask)  # Recalculate total fired count
                
                if debug_mem:
                    print(f"[DEBUG-MEM] Neural dynamics: Forced {memory_fired_count} memory neurons to fire unconditionally")
                    for i, is_memory in enumerate(memory_neuron_mask):
                        if is_memory:
                            neuron_id = neuron_array.index_to_neuron_id.get(i)
                            print(f"[DEBUG-MEM] 🔥 Memory neuron {neuron_id} forced to fire (idx={i})")
            
            # Debug post-firing results for high threshold neurons
            if debug_enabled:
                # Count high threshold neurons that fired
                fired_high_threshold = []
                for i in range(valid_range):
                    if valid_mask[i] and thresholds[i] >= 1000 and firing_mask[i]:
                        fired_high_threshold.append(i)
                        if len(fired_high_threshold) <= 10:  # Show first 10 examples
                            logger.error(f"[NEURAL-DEBUG] ❌ HIGH THRESHOLD NEURON FIRED: Neuron[{i}] potential={potentials[i]:.6f}, threshold={thresholds[i]:.1f}")
                
                if fired_high_threshold:
                    logger.error(f"[NEURAL-DEBUG] ❌ TOTAL HIGH THRESHOLD NEURONS FIRED: {len(fired_high_threshold)} (BUG!)")
                else:
                    logger.info("[NEURAL-DEBUG] ✅ All high threshold neurons correctly blocked from firing")
                
                # Also show which areas are firing
                if num_fired > 0:
                    firing_by_area = {}
                    for i in range(valid_range):
                        if valid_mask[i] and firing_mask[i]:
                            threshold = thresholds[i]
                            potential = potentials[i]
                            area_key = f"thresh_{threshold:.1f}"
                            if area_key not in firing_by_area:
                                firing_by_area[area_key] = 0
                            firing_by_area[area_key] += 1
                    
                    logger.info(f"[NEURAL-DEBUG] Firing summary by threshold: {firing_by_area}")
            
            # Step 3: Convert firing mask to neuron IDs
            if num_fired == 0:
                return []
                
            firing_indices = np.where(firing_mask)[0]
            fired_neuron_ids = []
            
            for idx in firing_indices:
                # Convert array index to neuron ID
                neuron_id = neuron_array.index_to_neuron_id.get(int(idx))
                if neuron_id is not None:
                    fired_neuron_ids.append(neuron_id)
            
            # Log neural dynamics results
            if fired_neuron_ids:
                debug_enabled = (self.state_manager and self.state_manager.is_debug_npu_enabled())
                if debug_enabled:
                    logger.debug("Neural dynamics: %d neurons fired with consecutive fire limits enforced", len(fired_neuron_ids))
                
            return fired_neuron_ids
            
        except Exception as e:
            logger.error("Error in neural dynamics processing: %s", str(e))
            return []
    
    def _apply_fcl_candidates_to_membrane_potentials(self, fcl: FireCandidateList, 
                                                    neuron_array, valid_range: int) -> None:
        """Apply FCL candidate potentials to neuron membrane potentials.
        
        Args:
            fcl: Fire Candidate List with candidates and potentials
            neuron_array: Neuron array to update
            valid_range: Valid neuron range for processing
            
        Note:
            RUST-COMPATIBLE: Uses vectorized operations for batch updates.
        """
        try:
            # Process each cortical area's candidates
            for cortical_idx, candidates in fcl.candidates_by_area.items():
                if not candidates:
                    continue
                
                # Extract neuron IDs and potentials from candidates
                candidate_neuron_ids = []
                potential_deltas = []
                
                for candidate in candidates:
                    candidate_neuron_ids.append(candidate.neuron_id)
                    potential_deltas.append(candidate.membrane_potential_delta)
                
                # Convert to numpy arrays for vectorized processing
                neuron_ids = np.array(candidate_neuron_ids, dtype=np.int32)
                deltas = np.array(potential_deltas, dtype=np.float32)
                
                # Apply potentials to membrane potentials (vectorized)
                matched = 0
                skipped_unmapped = 0
                out_of_range = 0
                thresholds_arr = getattr(neuron_array, 'thresholds', None)
                for i, neuron_id in enumerate(neuron_ids):
                    # Get neuron index from ID mapping
                    if neuron_id in neuron_array.neuron_id_to_index:
                        idx = neuron_array.neuron_id_to_index[neuron_id]
                        
                        # Bounds check
                        if 0 <= idx < valid_range:
                            # DEBUG: Log potential accumulation bug
                            old_potential = neuron_array.membrane_potentials[idx]
                            delta = deltas[i]
                            
                            # Check mp_charge_accumulation setting for this cortical area
                            cortical_id = None
                            mp_accumulation = True  # Default to accumulation
                            try:
                                # Get cortical_id from cortical_idx
                                if hasattr(self.connectome_manager, 'get_cortical_id_for_idx'):
                                    cortical_id = self.connectome_manager.get_cortical_id_for_idx(cortical_idx)
                                    if cortical_id and hasattr(self.connectome_manager, 'cortical_areas'):
                                        area = self.connectome_manager.cortical_areas.get(cortical_id)
                                        if area and hasattr(area, 'properties'):
                                            mp_accumulation = area.properties.get('mp_acc', True)
                            except Exception:
                                pass  # Use default accumulation behavior
                            
                            # Apply potential based on accumulation setting with clamp to threshold
                            if mp_accumulation:
                                # ACCUMULATE: Add delta to current potential
                                new_val = float(old_potential + delta)
                                # DEBUG: Log accumulation behavior
                                if matched < 3:  # Only log first few for performance
                                    logger.info(f"[ACCUMULATION-DEBUG] Neuron[{idx}] ACCUMULATE: {old_potential:.6f} + {delta:.6f} = {new_val:.6f}")
                            else:
                                # REPLACE: Set potential to delta value
                                new_val = float(delta)
                                # DEBUG: Log replacement behavior
                                if matched < 3:  # Only log first few for performance
                                    logger.info(f"[ACCUMULATION-DEBUG] Neuron[{idx}] REPLACE: {old_potential:.6f} -> {new_val:.6f}")

                            # Clamp pre-SIMD membrane potential to neuron threshold to avoid inflation
                            if thresholds_arr is not None and 0 <= idx < len(thresholds_arr):
                                thr_val = float(thresholds_arr[idx])
                                if new_val > thr_val:
                                    new_val = thr_val

                            neuron_array.membrane_potentials[idx] = new_val

                            new_potential = new_val
                            
                            # Log if potential becomes inflated
                            if new_potential > 1000 and matched < 3:
                                logger.error(f"[FCL-BUG] Neuron[{idx}] potential inflated: {old_potential:.6f} + {delta:.6f} = {new_potential:.6f}")
                            
                            matched += 1
                        else:
                            out_of_range += 1
                    else:
                        skipped_unmapped += 1

                # Optional debug logging (gated by --debug-npu)
                try:
                    from feagi.core.state_manager import FeagiStateManager
                    if FeagiStateManager.instance().is_debug_npu_enabled():
                        total = int(len(neuron_ids))
                        logger.info(
                            f"[SENSORY-DEBUG] Applied FCL candidates: area_idx={cortical_idx} matched={matched}/{total} "
                            f"unmapped={skipped_unmapped} out_of_range={out_of_range}"
                        )
                except Exception:
                    pass
                        
        except Exception as e:
            logger.error("Error applying FCL candidates to membrane potentials: %s", str(e))
    
    def update_consecutive_fire_limits(self, cortical_id: str, limit: int) -> bool:
        """Update consecutive fire limits for all neurons in a cortical area.
        
        This method is called when the genome parameter 'neuron_consecutive_fire_count'
        is updated via the API to ensure the NPU neural dynamics use the correct limits.
        
        Args:
            cortical_id: Cortical area ID (e.g., 'c__bac')
            limit: New consecutive fire limit
            
        Returns:
            True if update successful, False otherwise
        """
        try:
            if not self.connectome_manager:
                logger.warning("Cannot update consecutive fire limits: No connectome manager")
                return False
                
            npu_interface = getattr(self.connectome_manager, '_npu_interface', None)
            if not npu_interface:
                logger.warning("Cannot update consecutive fire limits: No NPU interface")
                return False
                
            neuron_array = getattr(npu_interface, 'neuron_array', None)
            if not neuron_array:
                logger.warning("Cannot update consecutive fire limits: No neuron array")
                return False
            
            # Get cortical index from cortical ID
            cortical_idx = None
            if hasattr(self.connectome_manager, 'get_cortical_idx_for_id'):
                cortical_idx = self.connectome_manager.get_cortical_idx_for_id(cortical_id)
            
            if cortical_idx is None:
                logger.warning("Cannot update consecutive fire limits: Cortical area %s not found", cortical_id)
                return False
            
            # Update consecutive fire limits for all neurons in this cortical area
            updated_count = neuron_array.update_consecutive_fire_limits_by_cortical_area(cortical_idx, limit)
            
            if updated_count > 0:
                logger.info("Updated consecutive fire limits to %d for %d neurons in cortical area %s", 
                           limit, updated_count, cortical_id)
                return True
            else:
                logger.warning("No neurons found in cortical area %s to update consecutive fire limits", cortical_id)
                return False
                
        except Exception as e:
            logger.error("Error updating consecutive fire limits for cortical area %s: %s", cortical_id, str(e))
            return False
    
    def _initialize_injection_service(self):
        """Initialize injection service for power areas and special neuron injection."""
        try:
            if not self.connectome_manager:
                logger.info("No connectome manager - injection service disabled")
                return
            
            # Create simple power injection service
            self.injection_service = PowerInjectionService(
                connectome_manager=self.connectome_manager,
                fcl_injector=self.fcl_injector
            )
            
            logger.info("Power injection service initialized for automatic burst injection")
            
        except Exception:
            logger.error("Failed to initialize injection service")
            self.injection_service = None
    
    # ==============================================================
    # CLASS METHODS FOR TEST COMPATIBILITY
    # ==============================================================
    
    @classmethod
    def run_with_fire_queue(cls, fire_queue: Any = None, max_iterations: int = 1) -> Dict[str, Any]:
        """Run burst engine with fire queue (class method for compatibility)."""
        instance = cls.get_instance()
        
        results = {
            'iterations': 0,
            'fired_neurons': [],
            'performance_metrics': {}
        }
        
        for i in range(max_iterations):
            try:
                # Process a single burst
                fired_neurons = instance.process_burst()
                results['fired_neurons'].extend(fired_neurons or [])
                results['iterations'] += 1
                
                # If no neurons fired, break early
                if not fired_neurons:
                    break
                    
            except Exception as e:
                logger.warning(f"Burst processing error: {e}")
                break
        
        results['performance_metrics'] = {
            'total_neurons_fired': len(results['fired_neurons']),
            'iterations_completed': results['iterations']
        }
        
        return results


class _MemoryProcessorAdapter:
    """Adapter that exposes minimal interface expected by ConnectomeManager.

    This bridges old MemoryProcessor calls to the new ledger-backed pipeline
    without introducing fallbacks. All methods are deterministic and bounded.
    """

    def __init__(self, burst_engine: BurstEngine):
        self._be = burst_engine
        self.active_memory_areas = set()

    def register_memory_area(
        self,
        cortical_id: str,
        temporal_depth: int,
        initial_lifespan: int,
        lifespan_growth_rate: float,
        longterm_threshold: int,
        upstream_areas: set,
    ) -> bool:
        try:
            # In new design, use fire ledger memory area config
            cm = getattr(self._be, 'connectome_manager', None)
            if not cm or not hasattr(cm, '_npu_interface'):
                return False
            npu_if = cm._npu_interface
            # Map cortical_id to index via NPUInterface
            idx = npu_if.get_cortical_idx_by_id(cortical_id) if hasattr(npu_if, 'get_cortical_idx_by_id') else None
            if idx is None:
                return False
            self._be.fire_ledger.configure_memory_area(idx, max(temporal_depth, 1), [])
            self.active_memory_areas.add(cortical_id)
            return True
        except Exception:
            return False

    def update_memory_area_upstream(self, cortical_id: str, upstream_areas: set) -> None:
        # New ledger stores upstream list; no-op here until extended API added
        _ = (cortical_id, upstream_areas)
        return


class PowerInjectionService:
    """Simple power injection service for constant brain activity."""
    
    def __init__(self, connectome_manager, fcl_injector):
        """Initialize power injection service."""
        self.connectome_manager = connectome_manager
        self.fcl_injector = fcl_injector
        self._power_neurons_cache = None
        self._cache_valid = False
        
        logger.info("PowerInjectionService created")
        
        # Invalidate cache to ensure power neurons get refractory_periods = 0 on first detection
        self.invalidate_cache()
    
    def _get_neuron_firing_threshold(self, neuron_id: int) -> float:
        """Get the actual firing threshold for a specific neuron.
        
        RUST-COMPATIBLE: 100% deterministic lookup. NO FALLBACKS.
        Raises error if genome data is missing - FEAGI must be deterministic.
        
        Args:
            neuron_id: The neuron ID to get threshold for
            
        Returns:
            Actual firing threshold from neuron properties (from genome)
            
        Raises:
            ValueError: If neuron data is missing from genome/NPU interface
        """
        if not self.connectome_manager:
            raise ValueError(f"Cannot get threshold for neuron {neuron_id}: No connectome manager available")
            
        npu_interface = getattr(self.connectome_manager, '_npu_interface', None)
        if not npu_interface:
            raise ValueError(f"Cannot get threshold for neuron {neuron_id}: No NPU interface in connectome manager")
            
        neuron_array = getattr(npu_interface, 'neuron_array', None)
        if not neuron_array:
            raise ValueError(f"Cannot get threshold for neuron {neuron_id}: No neuron array in NPU interface")
            
        # Get neuron index from ID - MUST exist in genome
        neuron_id_to_index = getattr(neuron_array, 'neuron_id_to_index', None)
        if not neuron_id_to_index:
            raise ValueError(f"Cannot get threshold for neuron {neuron_id}: No neuron ID mapping in genome")
            
        if neuron_id not in neuron_id_to_index:
            raise ValueError(f"Neuron {neuron_id} not found in genome neuron array - missing from neuroembryogenesis")
            
        neuron_index = neuron_id_to_index[neuron_id]
        
        # Get threshold array - MUST exist in genome
        thresholds = getattr(neuron_array, 'thresholds', None)
        if thresholds is None:
            raise ValueError(f"No firing thresholds array in genome neuron data")
            
        if neuron_index >= len(thresholds):
            raise ValueError(f"Neuron {neuron_id} index {neuron_index} out of bounds in thresholds array")
            
        actual_threshold = float(thresholds[neuron_index])
        return actual_threshold
    
    def inject_power_neurons(self, fcl: FireCandidateList, current_timestep: int) -> int:
        """Inject power neurons into FCL every burst for constant brain activity."""
        
        # NPU Debug logging for power injection - use periodic logging to prevent spam
        debug_enabled = (hasattr(self.connectome_manager, 'state_manager') and 
                        self.connectome_manager.state_manager and 
                        self.connectome_manager.state_manager.is_debug_npu_enabled())
        periodic_debug = debug_enabled and (current_timestep % 500 == 0)  # Every 50 bursts
        
        if periodic_debug:
            logger.debug("PowerInjectionService: Starting power neuron injection...")
            logger.debug("PowerInjectionService: Cache valid: %s", self._cache_valid)
            logger.debug("PowerInjectionService: Instance ID: %s", id(self))
            logger.debug("PowerInjectionService: Method entry - about to process power and external activations")
        
        try:
            # Get power neurons (cached for performance)
            power_neurons = self._get_power_neurons()
            
            if periodic_debug:
                logger.debug("PowerInjectionService: Retrieved %d power neurons from cache/detection", len(power_neurons))
            
            # Process power neurons if available
            injected_count = 0
            if not power_neurons:
                # Only log occasionally to avoid spam
                if current_timestep % 1000 == 0:
                    if periodic_debug:
                        logger.debug("PowerInjectionService: No power neurons available at timestep %d", current_timestep)
                    else:
                        logger.debug("No power neurons found at timestep %d", current_timestep)
                # Don't return early - continue to process external activations
            
            # Add power neurons to FCL using SoA format
            if power_neurons:
                if periodic_debug:
                    logger.debug("PowerInjectionService: Converting %d power neurons to SoA format", len(power_neurons))
                
                # Convert to numpy arrays for SoA format - NO HARDCODED VALUES
                neuron_ids = np.array(power_neurons, dtype=np.uint32)
                
                # Compute delta = max(0, threshold - current_potential) per neuron (deterministic, non-accumulating)
                potential_deltas = []
                npu_interface = self.connectome_manager._npu_interface
                neuron_array = npu_interface.neuron_array if npu_interface else None
                
                if neuron_array:
                    thresholds_arr = getattr(neuron_array, 'thresholds', None)
                    current_pots_arr = getattr(neuron_array, 'membrane_potentials', None)
                    if thresholds_arr is None or current_pots_arr is None:
                        raise ValueError("Cannot inject power neurons: Missing thresholds or membrane_potentials in neuron array")
                    for power_neuron_id in power_neurons:
                        if power_neuron_id not in neuron_array.neuron_id_to_index:
                            raise ValueError(f"Power neuron {power_neuron_id} not found in neuron array")
                        idx = neuron_array.neuron_id_to_index[power_neuron_id]
                        if idx >= len(thresholds_arr) or idx >= len(current_pots_arr):
                            raise ValueError(f"Power neuron index {idx} out of bounds for thresholds or membrane_potentials")
                        threshold = float(thresholds_arr[idx])
                        current_potential = float(current_pots_arr[idx])
                        delta = threshold - current_potential
                        if delta < 0.0:
                            delta = 0.0
                        potential_deltas.append(delta)
                    potential_deltas = np.array(potential_deltas, dtype=np.float32)
                else:
                    raise ValueError("Cannot inject power neurons: No neuron array available from genome")
                
                excitatory_mask = np.ones(len(power_neurons), dtype=bool)  # All excitatory
                
                if periodic_debug:
                    logger.debug("PowerInjectionService: SoA arrays created - neuron_ids: %s, potentials: %s", 
                              neuron_ids.shape, potential_deltas.shape)
                
                # Add all power neurons to cortical area 1 (reserved _power area)
                injected_count = fcl.add_candidates_soa(
                    cortical_idx=1,  # Reserved power area index
                    neuron_ids=neuron_ids,
                    potential_deltas=potential_deltas,
                    excitatory_mask=excitatory_mask
                )
                
                if periodic_debug:
                    logger.debug("PowerInjectionService: FCL.add_candidates_soa returned: %d", injected_count)
            else:
                # No power neurons available
                if periodic_debug:
                    logger.debug("PowerInjectionService: No power neurons to inject")
            
            # Log occasionally (every 100 bursts) to avoid spam
            if current_timestep % 100 == 0:
                if periodic_debug:
                    logger.debug("PowerInjectionService: Periodic status - %d neurons injected at burst %d", injected_count, current_timestep)
                else:
                    logger.info("Power injection: %d neurons injected at burst %d", injected_count, current_timestep)
            
            # Return count of power neurons injected this burst
            if periodic_debug:
                logger.debug("PowerInjectionService: Power injection complete - total=%d", injected_count)
            return injected_count
            
        except Exception as e:
            if debug_enabled:
                logger.error("PowerInjectionService: Exception in inject_power_neurons(): %s", str(e))
                if periodic_debug:  # Only show traceback periodically
                    import traceback
                    logger.error("PowerInjectionService: Traceback: %s", traceback.format_exc())
            else:
                logger.error("Error injecting power neurons: %s", str(e))
            return 0
    
    def inject_external_activations(self, activations: Dict, current_timestep: int, source: str = "external") -> int:
        """Buffer external activations deterministically for BurstEngine to inject via FCLInjector."""
        debug_enabled = (hasattr(self.connectome_manager, 'state_manager') and 
                        self.connectome_manager.state_manager and 
                        self.connectome_manager.state_manager.is_debug_npu_enabled())
        periodic_debug = debug_enabled and (current_timestep % 500 == 0)  # Every 50 bursts
        
        
        try:
            # Deterministic buffer for BurstEngine to process; do not merge arrays here
            if not hasattr(self.connectome_manager, 'burst_engine'):
                # Fallback: store locally for BurstEngine to read via reference
                if not hasattr(self, '_pending_external_activations'):
                    self._pending_external_activations = {}
                for area_id, area_data in activations.items():
                    self._pending_external_activations[area_id] = area_data
                return sum(
                    len(v.get('coordinates_x', [])) if isinstance(v, dict) else (len(v) if hasattr(v, '__len__') else 0)
                    for v in activations.values()
                )
            else:
                be = self.connectome_manager.burst_engine
                if not hasattr(be, '_pending_external_activations'):
                    be._pending_external_activations = {}
                for area_id, area_data in activations.items():
                    be._pending_external_activations[area_id] = area_data
                return sum(
                    len(v.get('coordinates_x', [])) if isinstance(v, dict) else (len(v) if hasattr(v, '__len__') else 0)
                    for v in activations.values()
                )
            
        except Exception as e:
            if debug_enabled:
                logger.error("PowerInjectionService: Error in external activation injection: %s", str(e))
            else:
                logger.error("Error injecting external activations: %s", str(e))
            return 0
    
    def _get_power_neurons(self) -> List[int]:
        """Get neurons from power areas (cached for performance)."""
        if self._cache_valid and self._power_neurons_cache is not None:
            return self._power_neurons_cache
        
        power_neurons = []
        
        # Debug power neuron detection
        debug_enabled = (hasattr(self.connectome_manager, 'state_manager') and 
                        self.connectome_manager.state_manager and 
                        self.connectome_manager.state_manager.is_debug_npu_enabled())
        
        if debug_enabled:
            logger.debug("PowerInjectionService: Starting power neuron detection...")
        
        try:
            # Check if connectome manager has NPU interface
            if hasattr(self.connectome_manager, '_npu_interface') and self.connectome_manager._npu_interface:
                if debug_enabled:
                    logger.debug("PowerInjectionService: Found NPU interface")
            else:
                if debug_enabled:
                    logger.debug("PowerInjectionService: No NPU interface found on connectome_manager")
                    logger.debug("PowerInjectionService: ConnectomeManager attributes: %s", 
                                  [attr for attr in dir(self.connectome_manager) if not attr.startswith('_')])
                return power_neurons  # Return empty list
            
            if hasattr(self.connectome_manager, '_npu_interface') and self.connectome_manager._npu_interface:
                npu_interface = self.connectome_manager._npu_interface
                # Only use reserved power area at cortical_idx=1
                if hasattr(npu_interface, 'cortical_areas') and 1 in npu_interface.cortical_areas:
                    area_data = npu_interface.cortical_areas[1]
                    cortical_id = area_data.get('cortical_id', '')
                    neurons = npu_interface.get_neurons_by_area(1)
                    if neurons:
                        # SINGLE KNOWN POWER NEURON: choose deterministically (minimum ID)
                        single_power_neuron = int(min(neurons))
                        power_neurons = [single_power_neuron]
                        if debug_enabled:
                            logger.debug(
                                "PowerInjectionService: Using single power neuron %d from cortical_idx=1 (cortical_id='%s')",
                                single_power_neuron,
                                cortical_id,
                            )
                        else:
                            logger.info(
                                "Using single power neuron %d from reserved power area at cortical_idx=1 ('%s')",
                                single_power_neuron,
                                cortical_id,
                            )
                    else:
                        if debug_enabled:
                            logger.debug("PowerInjectionService: Reserved power area at cortical_idx=1 has no neurons")
                else:
                    if debug_enabled:
                        logger.debug("PowerInjectionService: Reserved power area at cortical_idx=1 not found")
            
            # CRITICAL: Set refractory period to 0 for the selected power neuron (so it can fire every burst)
            if power_neurons and hasattr(self.connectome_manager, '_npu_interface'):
                npu_interface = self.connectome_manager._npu_interface
                neuron_array = getattr(npu_interface, 'neuron_array', None)
                
                if neuron_array and hasattr(neuron_array, 'refractory_periods'):
                    power_neuron_id = power_neurons[0]
                    if power_neuron_id in neuron_array.neuron_id_to_index:
                        idx = neuron_array.neuron_id_to_index[power_neuron_id]
                        if idx < len(neuron_array.refractory_periods):
                            neuron_array.refractory_periods[idx] = 0
                            logger.info("Set refractory period to 0 for power neuron %d (enables every-burst firing)", power_neuron_id)
                        elif debug_enabled:
                            logger.debug("PowerInjectionService: Power neuron index %d out of bounds for refractory update", idx)
                    elif debug_enabled:
                        logger.debug("PowerInjectionService: Power neuron %d not found in neuron_id_to_index", power_neuron_id)
            
            # Cache the result
            self._power_neurons_cache = power_neurons
            self._cache_valid = True
            
            if debug_enabled:
                logger.debug("PowerInjectionService: Power detection complete - %d total neurons found", len(power_neurons))
                if not power_neurons:
                    logger.debug("PowerInjectionService: No power areas detected - brain will rely entirely on external stimulation")
                    logger.debug("PowerInjectionService: Consider adding '_power', 'xxx_pwr', or 'xxx_power' cortical area to genome")
            
            if power_neurons:
                logger.info("Power neuron detection complete: %d total power neurons cached", len(power_neurons))
            else:
                logger.warning("No power areas detected - consider adding '_power' area to genome for constant brain activity")
                
        except Exception:
            logger.error("Error detecting power neurons")
            power_neurons = []
            
        return power_neurons
    
    def invalidate_cache(self):
        """Invalidate power neuron cache (call when genome changes)."""
        self._cache_valid = False
        self._power_neurons_cache = None
        
        # When cache is invalidated, power neurons will be re-detected and 
        # their refractory periods will be set to 0 on next access
    
    # ==============================================================
    # CLASS METHODS FOR TEST COMPATIBILITY
    # ==============================================================
    
    @classmethod
    def run_with_fire_queue(cls, fire_queue: Any = None, max_iterations: int = 1) -> Dict[str, Any]:
        """Run burst engine with fire queue (class method for compatibility)."""
        instance = cls.get_instance()
        
        results = {
            'iterations': 0,
            'fired_neurons': [],
            'performance_metrics': {}
        }
        
        for i in range(max_iterations):
            try:
                # Process a single burst
                fired_neurons = instance.process_burst()
                results['fired_neurons'].extend(fired_neurons or [])
                results['iterations'] += 1
                
                # If no neurons fired, break early
                if not fired_neurons:
                    break
                    
            except Exception as e:
                logger.warning(f"Burst processing error: {e}")
                break
        
        results['performance_metrics'] = {
            'total_neurons_fired': len(results['fired_neurons']),
            'iterations_completed': results['iterations']
        }
        
        return results