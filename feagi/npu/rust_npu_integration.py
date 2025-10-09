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
    
    def __init__(self, connectome_manager, fire_ledger_window: int = 20, 
                 neuron_capacity: Optional[int] = None, synapse_capacity: Optional[int] = None):
        """Initialize Rust NPU integration.
        
        Args:
            connectome_manager: ConnectomeManager instance (can be None initially)
            fire_ledger_window: Number of historical bursts to keep
            neuron_capacity: Optional neuron capacity (for pre-allocation)
            synapse_capacity: Optional synapse capacity (for pre-allocation)
        
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
        
        # If capacities provided, create Rust NPU immediately
        if neuron_capacity is not None and synapse_capacity is not None:
            logger.info("🦀 [RUST-NPU] Creating empty Rust NPU with capacity: %d neurons, %d synapses",
                       neuron_capacity, synapse_capacity)
            self._rust_npu = feagi_rust.RustNPU(
                neuron_capacity=neuron_capacity,
                synapse_capacity=synapse_capacity,
                fire_ledger_window=fire_ledger_window
            )
            self._rust_npu_initialized = True
            logger.info("🦀 [RUST-NPU] ✅ Empty Rust NPU created - ready for neuron creation")
        else:
            logger.info("🦀 [RUST-NPU] Integration layer initialized - will load connectome later")
    
    @classmethod
    def create_empty(cls, neuron_capacity: int, synapse_capacity: int, fire_ledger_window: int = 20):
        """Create an empty Rust NPU without connectome data.
        
        This is used during initialization to pre-allocate Rust arrays before genome load.
        
        Args:
            neuron_capacity: Maximum number of neurons
            synapse_capacity: Maximum number of synapses
            fire_ledger_window: Number of historical bursts to keep
        
        Returns:
            RustNPUIntegration instance with empty Rust NPU
        """
        return cls(
            connectome_manager=None,
            fire_ledger_window=fire_ledger_window,
            neuron_capacity=neuron_capacity,
            synapse_capacity=synapse_capacity
        )
    
    def process_burst(self, power_neurons: List[int]) -> Dict:
        """Process a single burst using Rust NPU.
        
        Args:
            power_neurons: List of neuron IDs to inject power into
        
        Returns:
            Dict with keys: fired_neurons, burst, neuron_count, etc.
        
        Raises:
            RuntimeError: If burst processing fails
        """
        # Ensure initialized (should always be true with new architecture)
        if not self._rust_npu_initialized:
            raise RuntimeError("🦀 [RUST-NPU] CRITICAL: Rust NPU not initialized! This should never happen.")
        
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
            
            # Unpack state: (cfc, cfc_limit, snooze_period, potential, threshold, refrac_countdown)
            # Note: snooze_countdown removed - now unified in refractory_countdown
            cfc, cfc_limit, snooze_period, potential, threshold, refrac_countdown = state
            
            # Get cortical_idx from neuron_to_area mapping (no Python array needed!)
            cortical_idx = -1
            if hasattr(self.connectome_manager, '_npu_interface'):
                npu_interface = self.connectome_manager._npu_interface
                cortical_idx = npu_interface.neuron_to_area.get(neuron_id, -1)
            
            logger.info("🔍 [RUST-NEURON-STATE] Neuron %d (cortical_idx=%d) from RUST NPU:", neuron_id, cortical_idx)
            logger.info("🔍   potential=%.3f, threshold=%.3f, above_threshold=%s", 
                        potential, threshold, potential >= threshold)
            logger.info("🔍   refractory_countdown=%d (unified, includes extended period)", refrac_countdown)
            logger.info("🔍   consecutive_fire_count=%d/%d", cfc, cfc_limit)
            logger.info("🔍   snooze_period=%d (gene value, applied when cfc_limit hit)", snooze_period)
            logger.info("🔍   BLOCKED=%s (refrac=%s, cfc_limit=%s)", 
                        refrac_countdown > 0 or (cfc_limit > 0 and cfc >= cfc_limit),
                        refrac_countdown > 0, 
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
