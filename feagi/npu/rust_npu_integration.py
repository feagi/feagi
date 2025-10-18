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
    
    def process_burst(self) -> Dict:
        """Process a single burst using Rust NPU.
        
        🔋 Power neurons auto-discovered from neuron array (cortical_area = 1)
        
        Returns:
            Dict with keys: fired_neurons, burst, neuron_count, etc.
        
        Raises:
            RuntimeError: If burst processing fails
        """
        # Ensure initialized (should always be true with new architecture)
        if not self._rust_npu_initialized:
            raise RuntimeError("🦀 [RUST-NPU] CRITICAL: Rust NPU not initialized! This should never happen.")
        
        # Call Rust NPU (THIS IS THE FAST PATH - ALL IN RUST!)
        burst_start = time.perf_counter()
        
        try:
            result = self._rust_npu.process_burst()
            logger.debug("🦀 Rust NPU process_burst returned: power_injections=%d, fired_neurons=%d", 
                          result.power_injections, len(result.fired_neurons))
            burst_time = (time.perf_counter() - burst_start) * 1000
            
            # Track performance
            self._track_performance(burst_start, result, burst_time)
            
            # Log performance periodically
            if result.burst % self._performance_log_interval == 0:
                self._log_performance(result, burst_time)
            
            # DIAGNOSTIC: Track firing intervals to validate refractory periods
            # (Only log for non-power neurons to reduce spam)
            if not hasattr(self, '_last_fire_burst'):
                self._last_fire_burst = {}
            
            for neuron_id in result.fired_neurons:
                if neuron_id == 1:  # Skip power neuron
                    continue
                    
                current_burst = result.burst
                self._last_fire_burst[neuron_id] = current_burst
            
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
    
    # ═══════════════════════════════════════════════════════════
    # FQ SAMPLER API (Entry Point #2: Motor/Visualization Output)
    # ═══════════════════════════════════════════════════════════
    
    def sample_fire_queue(self) -> Optional[Dict[int, Dict[str, List]]]:
        """Sample the current Fire Queue for visualization/motor output.
        
        ⚠️ CHANGED: Now uses get_latest_fire_queue_sample() internally to avoid deduplication issues.
        
        This is Entry Point #2: The thin Python API for external systems.
        All sampling logic is in Rust for maximum performance.
        
        Returns:
            Dict mapping cortical_idx to area data:
            {
                cortical_idx: {
                    "neuron_ids": [id1, id2, ...],
                    "coordinates_x": [x1, x2, ...],
                    "coordinates_y": [y1, y2, ...],
                    "coordinates_z": [z1, z2, ...],
                    "membrane_potentials": [p1, p2, ...]
                }
            }
            
            Returns None if no bursts have been processed yet.
        
        Example:
            sample = rust_npu_integration.sample_fire_queue()
            if sample:
                for cortical_idx, area_data in sample.items():
                    neuron_ids = area_data["neuron_ids"]
                    coords_x = area_data["coordinates_x"]
                    # Process firing neurons...
        """
        if not self._rust_npu:
            return None
        
        # 🦀 RUST: Use get_latest_fire_queue_sample() to avoid deduplication
        # The burst loop already calls sample() in Phase 5, so we just read the cached result
        rust_sample = self._rust_npu.get_latest_fire_queue_sample()
        if rust_sample is None:
            return None
        
        # Convert from Rust tuple format to Python dict format
        # Rust returns: Dict[int, Tuple[neuron_ids, coords_x, coords_y, coords_z, potentials]]
        # Python expects: Dict[int, Dict[str, List]]
        # PERFORMANCE: Rust tuples are already list-like, avoid list() copy for large neuron counts
        result = {}
        for cortical_idx, (neuron_ids, coords_x, coords_y, coords_z, potentials) in rust_sample.items():
            result[cortical_idx] = {
                "neuron_ids": neuron_ids,  # No list() copy - Rust Vec is already Python list-like
                "coordinates_x": coords_x,
                "coordinates_y": coords_y,
                "coordinates_z": coords_z,
                "membrane_potentials": potentials
            }
        
        return result
    
    def set_fq_sampler_frequency(self, frequency_hz: float):
        """Set FQ Sampler frequency (Hz)."""
        if self._rust_npu:
            self._rust_npu.set_fq_sampler_frequency(frequency_hz)
    
    def get_fq_sampler_frequency(self) -> float:
        """Get FQ Sampler frequency (Hz)."""
        if self._rust_npu:
            return self._rust_npu.get_fq_sampler_frequency()
        return 10.0
    
    def set_visualization_subscribers(self, has_subscribers: bool):
        """Set visualization subscriber state."""
        if self._rust_npu:
            self._rust_npu.set_visualization_subscribers(has_subscribers)
    
    def set_motor_subscribers(self, has_subscribers: bool):
        """Set motor subscriber state."""
        if self._rust_npu:
            self._rust_npu.set_motor_subscribers(has_subscribers)
    
    def get_current_fire_queue(self):
        """Get current Fire Queue directly (bypasses FQ Sampler rate limiting).
        
        Used by FCL endpoint to get real-time firing data without sampling delays.
        Unlike sample_fire_queue(), this always returns the current Fire Queue.
        
        Returns:
            Dict[int, Dict[str, List]] or None
            {
                cortical_idx: {
                    "neuron_ids": [int, ...],
                    "coordinates_x": [int, ...],
                    "coordinates_y": [int, ...],
                    "coordinates_z": [int, ...],
                    "membrane_potentials": [float, ...]
                }
            }
        """
        if not self._rust_npu:
            return None
        
        # Call Rust to get current Fire Queue (no rate limiting)
        rust_data = self._rust_npu.get_current_fire_queue()
        if not rust_data:
            return {}
        
        # Convert from Rust tuple format to Python dict format
        # PERFORMANCE: Avoid list() copy for large neuron counts
        result = {}
        for cortical_idx, (neuron_ids, coords_x, coords_y, coords_z, potentials) in rust_data.items():
            result[cortical_idx] = {
                "neuron_ids": neuron_ids,
                "coordinates_x": coords_x,
                "coordinates_y": coords_y,
                "coordinates_z": coords_z,
                "membrane_potentials": potentials
            }
        
        return result
    
    # ═══════════════════════════════════════════════════════════
    # FIRE LEDGER API (Entry Point #3: Historical Debugging)
    # ═══════════════════════════════════════════════════════════
    
    def get_fire_ledger_history(self, cortical_idx: int, num_timesteps: int) -> List[Dict]:
        """Get Fire Ledger history for a cortical area.
        
        Args:
            cortical_idx: Cortical area ID
            num_timesteps: Number of recent timesteps to retrieve
        
        Returns:
            List of timesteps, each containing:
            {
                "timestep": int,
                "neuron_ids": [int, ...],
                "coordinates_x": [int, ...],
                "coordinates_y": [int, ...],
                "coordinates_z": [int, ...],
                "potentials": [float, ...]
            }
        """
        if not self._rust_npu:
            return []
        
        return self._rust_npu.get_fire_ledger_history(cortical_idx, num_timesteps)
    
    def get_fire_ledger_window_size(self, cortical_idx: int) -> int:
        """Get Fire Ledger window size for a cortical area."""
        if not self._rust_npu:
            return 0
        
        return self._rust_npu.get_fire_ledger_window_size(cortical_idx)
    
    def configure_fire_ledger_window(self, cortical_idx: int, window_size: int):
        """Configure Fire Ledger window size for a cortical area."""
        if self._rust_npu:
            self._rust_npu.configure_fire_ledger_window(cortical_idx, window_size)
    
    def get_all_fire_ledger_configs(self) -> Dict[int, int]:
        """Get all Fire Ledger window configurations.
        
        Returns:
            Dict mapping cortical_idx -> window_size
        """
        if not self._rust_npu:
            return {}
        
        return self._rust_npu.get_all_fire_ledger_configs()
    
    def get_neurons_at_coordinates_batch(
        self, 
        cortical_idx: int,
        coords_x: List[int],
        coords_y: List[int],
        coords_z: List[int]
    ) -> List[Optional[int]]:
        """BATCH: Get neuron IDs for multiple coordinates (high-performance sensory injection).
        
        This is 10-100x faster than calling individual lookups in a Python loop because
        it eliminates FFI overhead and enables Rust-side vectorization.
        
        Args:
            cortical_idx: Cortical area index
            coords_x: List of X coordinates
            coords_y: List of Y coordinates  
            coords_z: List of Z coordinates
            
        Returns:
            List of neuron IDs (None if no neuron at that coordinate)
            Length matches input coordinate lists
            
        Example:
            >>> neuron_ids = rust_npu.get_neurons_at_coordinates_batch(
            ...     cortical_idx=5,
            ...     coords_x=[0, 1, 2],
            ...     coords_y=[0, 0, 0],
            ...     coords_z=[0, 0, 0]
            ... )
            >>> # neuron_ids = [1001, None, 1003]  # None = no neuron at (1,0,0)
        """
        if not self._rust_npu:
            return [None] * len(coords_x)
        
        return self._rust_npu.get_neurons_at_coordinates_batch(
            cortical_idx, coords_x, coords_y, coords_z
        )
    
    # ═══════════════════════════════════════════════════════════
    # SYNAPSE MANAGEMENT API (For Dynamic Synapse Updates)
    # ═══════════════════════════════════════════════════════════
    
    def rebuild_synapse_index(self):
        """Rebuild the synapse index after synapses have been added/removed.
        
        This MUST be called after synapses are created via neuroembryogenesis or
        other mechanisms so that the Rust PropagationEngine can use them.
        
        Without this, newly created synapses won't participate in burst processing.
        """
        if self._rust_npu:
            self._rust_npu.rebuild_indexes()
            logger.info("🦀 [RUST-NPU] Synapse index rebuilt successfully")
        else:
            logger.warning("🦀 [RUST-NPU] Cannot rebuild synapse index - NPU not initialized")
