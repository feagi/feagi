"""
Rust NPU Integration Module

This module provides the integration layer between Python (burst_engine.py) and the Rust NPU.

PRODUCTION-READY: Direct integration with no fallbacks.
"""

from typing import List, Dict, Optional
import time
from feagi.utils.logger import setup_logger

try:
    import feagi_rust
    RUST_AVAILABLE = True
except ImportError:
    RUST_AVAILABLE = False

logger = setup_logger(__name__)


class RustNPUIntegration:
    """Integration layer for Rust NPU.
    
    This class handles:
    - Rust NPU initialization
    - Connectome data loading
    - Burst processing
    - Performance monitoring
    
    CRITICAL: This is the ONLY burst processing path in production.
    NO FALLBACKS - fail fast with clear errors if anything goes wrong.
    """
    
    def __init__(self, connectome_manager, fire_ledger_window: int = 20):
        """Initialize Rust NPU integration.
        
        Args:
            connectome_manager: ConnectomeManager instance with neuron/synapse data
            fire_ledger_window: Number of historical bursts to keep
        
        Raises:
            RuntimeError: If Rust NPU is not available (production requirement)
        """
        if not RUST_AVAILABLE:
            raise RuntimeError(
                "🦀 [RUST-NPU] CRITICAL: Rust NPU not available! "
                "FEAGI requires the Rust NPU for production use. "
                "Please build the Rust components:\n"
                "  cd feagi_core/feagi-rust\n"
                "  cargo build --release\n"
                "  cp target/release/libfeagi_rust.* ../feagi/npu/"
            )
        
        self.connectome_manager = connectome_manager
        self.fire_ledger_window = fire_ledger_window
        
        self._rust_npu: Optional[feagi_rust.RustNPU] = None
        self._rust_npu_initialized = False
        
        # Performance tracking
        self._burst_times = []
        self._burst_times_window = 10
        self._last_performance_log = 0
        self._performance_log_interval = 100
        
        logger.info("🦀 [RUST-NPU] Integration layer initialized - ready to load connectome")
    
    def initialize(self) -> None:
        """Initialize the Rust NPU with connectome data.
        
        CRITICAL: This must succeed or we fail fast.
        
        Raises:
            RuntimeError: If initialization fails
        """
        if self._rust_npu_initialized:
            logger.info("🦀 [RUST-NPU] Already initialized - skipping")
            return
        
        if not self.connectome_manager:
            raise RuntimeError(
                "🦀 [RUST-NPU] CRITICAL: Cannot initialize without connectome_manager"
            )
        
        logger.info("🦀 [RUST-NPU] Starting initialization...")
        init_start = time.perf_counter()
        
        try:
            # Get neuron and synapse counts
            neuron_count = self._get_neuron_count()
            synapse_count = self._get_synapse_count()
            
            # Create Rust NPU with headroom for dynamic additions
            neuron_capacity = max(neuron_count * 2, 10000)
            synapse_capacity = max(synapse_count * 2, 100000)
            
            logger.info("🦀 [RUST-NPU] Creating NPU: %d neurons (capacity: %d), %d synapses (capacity: %d)",
                        neuron_count, neuron_capacity, synapse_count, synapse_capacity)
            
            self._rust_npu = feagi_rust.RustNPU(
                neuron_capacity=neuron_capacity,
                synapse_capacity=synapse_capacity,
                fire_ledger_window=self.fire_ledger_window
            )
            
            # Load connectome
            self._load_connectome()
            
            self._rust_npu_initialized = True
            
            init_time = (time.perf_counter() - init_start) * 1000
            logger.info("🦀 [RUST-NPU] ✅ Initialization complete in %.2f ms: %d neurons, %d synapses",
                        init_time,
                        self._rust_npu.get_neuron_count(),
                        self._rust_npu.get_synapse_count())
            
        except Exception as e:
            logger.error("🦀 [RUST-NPU] CRITICAL: Initialization failed: %s", str(e), exc_info=True)
            raise RuntimeError(f"Rust NPU initialization failed: {e}") from e
    
    def _get_neuron_count(self) -> int:
        """Get neuron count from connectome_manager."""
        if hasattr(self.connectome_manager, 'get_neuron_count'):
            return self.connectome_manager.get_neuron_count()
        elif hasattr(self.connectome_manager, 'neuron_array'):
            neuron_array = self.connectome_manager.neuron_array
            if hasattr(neuron_array, 'neuron_count'):
                return neuron_array.neuron_count
        # Safe default
        return 10000
    
    def _get_synapse_count(self) -> int:
        """Get synapse count from connectome_manager."""
        if hasattr(self.connectome_manager, 'get_synapse_count'):
            return self.connectome_manager.get_synapse_count()
        elif hasattr(self.connectome_manager, '_npu_interface'):
            npu_interface = self.connectome_manager._npu_interface
            if hasattr(npu_interface, 'synapse_array'):
                synapse_array = npu_interface.synapse_array
                if hasattr(synapse_array, '__len__'):
                    return len(synapse_array.source_neuron_ids)
        # Safe default
        return 100000
    
    def _load_connectome(self) -> None:
        """Load all neurons and synapses from connectome_manager into Rust NPU.
        
        This is the CRITICAL data loading path.
        """
        load_start = time.perf_counter()
        
        # Load neurons
        neurons_loaded = self._load_neurons()
        logger.info("🦀 [RUST-NPU] Loaded %d neurons", neurons_loaded)
        
        # Load synapses
        synapses_loaded = self._load_synapses()
        logger.info("🦀 [RUST-NPU] Loaded %d synapses", synapses_loaded)
        
        # Rebuild indexes (CRITICAL for performance!)
        logger.info("🦀 [RUST-NPU] Rebuilding synapse indexes for %d synapses...", synapses_loaded)
        self._rust_npu.rebuild_indexes()
        logger.info("🦀 [RUST-NPU] ✅ Indexes rebuilt - synapse lookup optimized")
        
        # Set neuron→cortical_area mapping
        mapping_count = self._set_neuron_mapping()
        logger.info("🦀 [RUST-NPU] Neuron mapping set (%d neurons mapped)", mapping_count)
        
        load_time = (time.perf_counter() - load_start) * 1000
        logger.info("🦀 [RUST-NPU] Connectome load complete in %.2f ms", load_time)
    
    def _load_neurons(self) -> int:
        """Load neurons from connectome_manager."""
        count = 0
        
        # Try NPU interface first
        if hasattr(self.connectome_manager, '_npu_interface'):
            npu_interface = self.connectome_manager._npu_interface
            if hasattr(npu_interface, 'neuron_array'):
                neuron_array = npu_interface.neuron_array
                count = self._load_neurons_from_array(neuron_array)
        
        return count
    
    def _load_neurons_from_array(self, neuron_array) -> int:
        """Load neurons from neuron array with ALL properties (matches Python exactly)."""
        count = 0
        neuron_count = getattr(neuron_array, 'neuron_count', 0)
        
        # CRITICAL DEBUG: Track neurons per cortical area during load
        neurons_per_area = {}
        
        # Debug: Check excitability values
        logger.error("="*80)
        logger.error("🔥 [EXCITABILITY-DEBUG] RELOADING NEURONS INTO RUST NPU - CHECKING EXCITABILITY!")
        logger.error("="*80)
        if hasattr(neuron_array, 'excitabilities'):
            unique_excitabilities = sorted(set(neuron_array.excitabilities[:neuron_count]))
            logger.error("🔥 [EXCITABILITY-DEBUG] Loading %d neurons, unique excitability values: %s", 
                          neuron_count, unique_excitabilities[:20])  # Show up to 20 unique values
        else:
            logger.error("🔥 [EXCITABILITY-DEBUG] WARNING: neuron_array has NO 'excitabilities' attribute!")
        
        logger.error("🔥 [EXCITABILITY-DEBUG] Starting neuron load: neuron_count=%d", neuron_count)
        
        for i in range(neuron_count):
            try:
                cortical_area_idx = int(neuron_array.cortical_areas[i]) if hasattr(neuron_array, 'cortical_areas') else 0
                excitability = float(neuron_array.excitabilities[i]) if hasattr(neuron_array, 'excitabilities') else 1.0
                
                # Track per-area statistics
                if cortical_area_idx not in neurons_per_area:
                    neurons_per_area[cortical_area_idx] = {'count': 0, 'excitabilities': set()}
                neurons_per_area[cortical_area_idx]['count'] += 1
                neurons_per_area[cortical_area_idx]['excitabilities'].add(excitability)
                
                neuron_id = self._rust_npu.add_neuron(
                    threshold=float(neuron_array.thresholds[i]) if hasattr(neuron_array, 'thresholds') else 1.0,
                    leak_coefficient=float(neuron_array.leak_coefficients[i]) if hasattr(neuron_array, 'leak_coefficients') else 0.0,
                    resting_potential=float(neuron_array.resting_potentials[i]) if hasattr(neuron_array, 'resting_potentials') else 0.0,
                    neuron_type=int(neuron_array.neuron_types[i]) if hasattr(neuron_array, 'neuron_types') else 0,
                    refractory_period=int(neuron_array.refractory_periods[i]) if hasattr(neuron_array, 'refractory_periods') else 0,
                    excitability=excitability,
                    consecutive_fire_limit=int(neuron_array.consecutive_fire_limits[i]) if hasattr(neuron_array, 'consecutive_fire_limits') else 0,
                    snooze_period=int(neuron_array.snooze_periods[i]) if hasattr(neuron_array, 'snooze_periods') else 0,
                    cortical_area=cortical_area_idx,
                    x=int(neuron_array.coordinates[i * 3]) if hasattr(neuron_array, 'coordinates') else 0,
                    y=int(neuron_array.coordinates[i * 3 + 1]) if hasattr(neuron_array, 'coordinates') else 0,
                    z=int(neuron_array.coordinates[i * 3 + 2]) if hasattr(neuron_array, 'coordinates') else 0
                )
                
                # Update NPUInterface tracking (for diagnostics and compatibility)
                if hasattr(self.connectome_manager, '_npu_interface'):
                    npu_interface = self.connectome_manager._npu_interface
                    npu_interface.neuron_to_area[neuron_id] = cortical_area_idx
                    npu_interface.neuron_id_to_index[neuron_id] = neuron_id
                    npu_interface.index_to_neuron_id[neuron_id] = neuron_id
                
                count += 1
            except Exception as e:
                logger.warning("🦀 [RUST-NPU] Failed to load neuron %d: %s", i, str(e))
        
        # Log per-area summary
        logger.error("="*80)
        logger.error("🔥 [EXCITABILITY-DEBUG] Loaded %d neurons across %d cortical areas", count, len(neurons_per_area))
        logger.error("="*80)
        for area_idx in sorted(neurons_per_area.keys()):  # Log ALL areas
            area_info = neurons_per_area[area_idx]
            logger.error("🔥 [EXCITABILITY-DEBUG]   Area %d: %d neurons, excitabilities=%s", 
                          area_idx, area_info['count'], sorted(area_info['excitabilities']))
        logger.error("="*80)
        
        return count
    
    def _load_synapses(self) -> int:
        """Load synapses from connectome_manager."""
        count = 0
        
        logger.info("🦀 [RUST-NPU] [DIAG] Checking for synapses in connectome_manager...")
        
        # Try NPU interface first
        if hasattr(self.connectome_manager, '_npu_interface'):
            npu_interface = self.connectome_manager._npu_interface
            logger.info("🦀 [RUST-NPU] [DIAG] Found _npu_interface")
            
            if hasattr(npu_interface, 'synapse_array'):
                synapse_array = npu_interface.synapse_array
                logger.info("🦀 [RUST-NPU] [DIAG] Found synapse_array in _npu_interface")
                count = self._load_synapses_from_array(synapse_array)
            else:
                logger.warning("🦀 [RUST-NPU] [DIAG] _npu_interface has no synapse_array attribute!")
                logger.warning("🦀 [RUST-NPU] [DIAG] _npu_interface attributes: %s", dir(npu_interface))
        else:
            logger.warning("🦀 [RUST-NPU] [DIAG] connectome_manager has no _npu_interface attribute!")
            logger.warning("🦀 [RUST-NPU] [DIAG] connectome_manager attributes: %s", 
                          [attr for attr in dir(self.connectome_manager) if not attr.startswith('__')])
        
        return count
    
    def _load_synapses_from_array(self, synapse_array) -> int:
        """Load synapses from synapse array."""
        count = 0
        power_synapses = 0
        synapse_count = len(synapse_array.source_neuron_ids) if hasattr(synapse_array, 'source_neuron_ids') else 0
        
        logger.info("🦀 [RUST-NPU] Loading %d synapses from array...", synapse_count)
        
        # Track first few synapses for debugging
        first_synapses = []
        
        for i in range(synapse_count):
            try:
                # Check if synapse is valid
                if hasattr(synapse_array, 'valid_mask') and not synapse_array.valid_mask[i]:
                    continue
                
                source = int(synapse_array.source_neuron_ids[i])
                target = int(synapse_array.target_neuron_ids[i])
                weight = int(synapse_array.weights[i]) if hasattr(synapse_array, 'weights') else 128
                conductance = int(synapse_array.conductances[i]) if hasattr(synapse_array, 'conductances') else 255
                synapse_type = int(synapse_array.types[i]) if hasattr(synapse_array, 'types') else 0
                
                self._rust_npu.add_synapse(
                    source=source,
                    target=target,
                    weight=weight,
                    conductance=conductance,
                    synapse_type=synapse_type
                )
                count += 1
                
                # Track power neuron synapses (neuron ID 2 is typically power)
                if source == 2:
                    power_synapses += 1
                    if power_synapses <= 5:  # Log first 5 power synapses
                        logger.info("🦀 [RUST-NPU] Power synapse #%d: source=%d → target=%d, weight=%d, conductance=%d, type=%d",
                                    power_synapses, source, target, weight, conductance, synapse_type)
                
                # Track first 3 synapses for debugging
                if count <= 3:
                    first_synapses.append(f"source={source}→target={target}")
                    
            except Exception as e:
                logger.warning("🦀 [RUST-NPU] Failed to load synapse %d: %s", i, str(e))
        
        logger.info("🦀 [RUST-NPU] Loaded %d synapses (%d from power neuron)", count, power_synapses)
        if first_synapses:
            logger.info("🦀 [RUST-NPU] First synapses: %s", ", ".join(first_synapses))
        
        return count
    
    def _set_neuron_mapping(self) -> int:
        """Set neuron→cortical_area mapping."""
        mapping = {}
        
        if hasattr(self.connectome_manager, '_npu_interface'):
            npu_interface = self.connectome_manager._npu_interface
            if hasattr(npu_interface, 'neuron_array'):
                neuron_array = npu_interface.neuron_array
                neuron_count = getattr(neuron_array, 'neuron_count', 0)
                
                for i in range(neuron_count):
                    cortical_area = int(neuron_array.cortical_areas[i]) if hasattr(neuron_array, 'cortical_areas') else 0
                    mapping[i] = cortical_area
        
        if mapping:
            self._rust_npu.set_neuron_mapping(mapping)
        
        return len(mapping)
    
    def process_burst(self, power_neurons: List[int]) -> Dict:
        """Process a single burst using Rust NPU.
        
        Args:
            power_neurons: List of neuron IDs to inject power into
        
        Returns:
            Dict with keys: fired_neurons, burst, neuron_count, etc.
        
        Raises:
            RuntimeError: If burst processing fails
        """
        # Ensure initialized
        if not self._rust_npu_initialized:
            self.initialize()
        
        logger.warning("🦀 [POWER-DEBUG] process_burst called with %d power neurons: %s", 
                      len(power_neurons),
                      power_neurons[:10] if len(power_neurons) > 10 else power_neurons)
        
        # Call Rust NPU (THIS IS THE FAST PATH - ALL IN RUST!)
        burst_start = time.perf_counter()
        
        try:
            result = self._rust_npu.process_burst(power_neurons=power_neurons)
            logger.warning("🦀 [POWER-DEBUG] Rust NPU process_burst returned: power_injections=%d, fired_neurons=%d", 
                          result.power_injections, len(result.fired_neurons))
            burst_time = (time.perf_counter() - burst_start) * 1000
            
            # Track performance
            self._track_performance(burst_start, result, burst_time)
            
            # Log performance periodically
            if result.burst % self._performance_log_interval == 0:
                self._log_performance(result, burst_time)
            
            # DIAGNOSTIC: Check snooze state of neuron 16439 (target of power synapse in iic000)
            self._diagnose_neuron_state(16439)
            
            # Return as dict for easier integration
            return {
                'fired_neurons': result.fired_neurons,
                'burst': result.burst,
                'neuron_count': result.neuron_count,
                'power_injections': result.power_injections,
                'synaptic_injections': result.synaptic_injections,
                'neurons_processed': result.neurons_processed,
                'neurons_in_refractory': result.neurons_in_refractory,
                'processing_time_ms': burst_time
            }
            
        except Exception as e:
            logger.error("🦀 [RUST-NPU] CRITICAL: Burst processing failed: %s", str(e), exc_info=True)
            raise RuntimeError(f"Rust NPU burst processing failed: {e}") from e
    
    def _diagnose_neuron_state(self, neuron_id: int) -> None:
        """Diagnostic helper to check neuron state (snooze, CFC, potential, etc.).
        
        ✅ Reads from Rust NPU (single source of truth), NOT deprecated Python arrays.
        """
        try:
            # Get neuron state directly from Rust NPU (live data!)
            state = self._rust_npu.get_neuron_state(neuron_id)
            if state is None:
                logger.warning("🔍 [NEURON-DEBUG] Neuron %d not found in Rust NPU", neuron_id)
                return
            
            # Unpack state: (cfc, cfc_limit, snooze_countdown, snooze_period, potential, threshold, refrac_countdown)
            cfc, cfc_limit, snooze_countdown, snooze_period, potential, threshold, refrac_countdown = state
            
            # Get cortical_idx from neuron_to_area mapping (no Python array needed!)
            cortical_idx = -1
            if hasattr(self.connectome_manager, '_npu_interface'):
                npu_interface = self.connectome_manager._npu_interface
                cortical_idx = npu_interface.neuron_to_area.get(neuron_id, -1)
            
            logger.error("🔍 [RUST-NEURON-STATE] Neuron %d (cortical_idx=%d) from RUST NPU:", neuron_id, cortical_idx)
            logger.error("🔍   potential=%.3f, threshold=%.3f, above_threshold=%s", 
                        potential, threshold, potential >= threshold)
            logger.error("🔍   refractory_countdown=%d", refrac_countdown)
            logger.error("🔍   consecutive_fire_count=%d/%d", cfc, cfc_limit)
            logger.error("🔍   snooze_countdown=%d (period=%d)", snooze_countdown, snooze_period)
            logger.error("🔍   BLOCKED=%s (refrac=%s, snooze=%s, cfc_limit=%s)", 
                        refrac_countdown > 0 or snooze_countdown > 0 or (cfc_limit > 0 and cfc >= cfc_limit),
                        refrac_countdown > 0, snooze_countdown > 0, 
                        cfc_limit > 0 and cfc >= cfc_limit)
            
        except Exception as e:
            logger.error("🔍 [NEURON-DEBUG] Failed to diagnose neuron %d: %s", neuron_id, str(e))
            logger.exception("Full stack trace:")
    
    def _track_performance(self, burst_start_time, result, burst_time_ms):
        """Track burst performance metrics."""
        current_time = time.perf_counter()
        
        if len(self._burst_times) > 0:
            interval = current_time - self._burst_times[-1] if self._burst_times else 0
            self._burst_times.append(interval)
            
            # Keep only last N burst times
            if len(self._burst_times) > self._burst_times_window:
                self._burst_times.pop(0)
    
    def _log_performance(self, result, burst_time_ms):
        """Log performance metrics."""
        if len(self._burst_times) > 0:
            avg_interval = sum(self._burst_times) / len(self._burst_times)
            actual_hz = 1.0 / avg_interval if avg_interval > 0 else 0.0
            
            logger.info(
                "🦀 [RUST-NPU] Burst #%d: %.2f Hz | %.2f ms | %d neurons fired | "
                "power: %d, synaptic: %d, refractory: %d",
                result.burst, actual_hz, burst_time_ms, result.neuron_count,
                result.power_injections, result.synaptic_injections,
                result.neurons_in_refractory
            )
    
    def get_burst_count(self) -> int:
        """Get current burst count."""
        if self._rust_npu:
            return self._rust_npu.get_burst_count()
        return 0
    
    def get_neuron_count(self) -> int:
        """Get neuron count."""
        if self._rust_npu:
            return self._rust_npu.get_neuron_count()
        return 0
    
    def get_synapse_count(self) -> int:
        """Get synapse count."""
        if self._rust_npu:
            return self._rust_npu.get_synapse_count()
        return 0
