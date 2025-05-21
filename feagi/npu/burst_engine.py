import time
import signal
import threading
from typing import Dict, List, Optional, Set, Any, Union
from feagi.core.state_manager import FeagiStateManager, ServiceState
from feagi.utils.logger import setup_logger
logger = setup_logger()



"""
Burst Engine Implementation for FEAGI.

The BurstEngine is FEAGI's primary neural simulation component. It drives the dynamics 
of neuron firing, manages membrane potentials, and coordinates the Fire Candidate List (FCL).

Key features:
- Standby Mode: Initializes without requiring a genome
- RTOS-Friendly: Designed for real-time operating systems with predictable timing  
- State-Driven: Uses explicit state transitions with consistent logging
- Dependency Injected: No global state, all dependencies passed explicitly

Usage:
    # Create and initialize
    engine = BurstEngine(connectome_manager)
    
    # Start the engine
    engine.run()
    
    # When genome is loaded
    engine.update_with_genome()
    
    # Graceful shutdown
    engine.stop()
"""

class BurstEngine:
    """
    RTOS/Rust-friendly burst engine for FEAGI neural simulation.
    - No dynamic allocation in the main loop
    - All configuration and memory allocation happens before entering the loop
    - Main loop is a single, clear sequence of steps
    - Supports graceful shutdown
    - New: Initializes in standby mode without requiring a genome
    """
    def __init__(self, connectome_manager: Any, fcl_manager: Optional[Any] = None, config: Optional[Dict[str, Any]] = None) -> None:
        """
        Initialize the Burst Engine.
        
        Args:
            connectome_manager: The connectome manager
            fcl_manager: FCL manager (optional)
            config: Configuration parameters (optional)
        """
        self.connectome_manager = connectome_manager
        self.fcl_manager = fcl_manager or connectome_manager.fcl_manager
        self.config = config or {}
        self.genome_loaded = False
        self._running = False
        self.burst_count = 0
        self.last_burst_time = 0.0
        
        # Initialize in a valid but inactive state
        # Will become fully operational when a genome is loaded
        logger.info("Burst Engine initialized in standby mode", emoji1="⚡️")
        
        self.state_manager = FeagiStateManager.instance()
        
        # Support both parameter names for backward compatibility
        self.desired_frequency = self.config.get('desired_frequency_hz', 
                                              self.config.get('target_frequency', 100.0))
        self.target_frequency = self.desired_frequency  # For backward compatibility
        self.burst_interval = 1.0 / self.desired_frequency
        
        # Use cortical_areas instead of _areas - fix the attribute name
        self.cortical_areas = list(self.connectome_manager.cortical_areas.values()) if hasattr(self.connectome_manager, 'cortical_areas') else []
        self.shed_areas = set(area.id for area in self.cortical_areas if area.properties.get('__shed', False))

    def _process_burst(self) -> List[int]:
        """
        Process a single burst cycle using the standard method.
        
        This method updates membrane potentials and processes neuron firing.
        It's used as a fallback when optimized implementations are not available.
        
        Returns:
            List of neuron IDs that fired in this burst
        """
        # Update membrane potentials and get fired neurons
        fired_neurons = self.connectome_manager.update_membrane_potentials()
        
        # Any additional processing can be added here
        # For example, you might want to track fired neurons in specific areas
        
        return fired_neurons

    def run(self) -> None:
        """
        Run the burst engine main loop.
        
        This function begins the burst execution loop, processing neuron firings
        based on the target burst frequency.
        """
        self._running = True
        self.state_manager.set_burst_engine_state(ServiceState.READY)
        def handle_signal(signum: int, frame: Any) -> None:
            logger.info(f"\nReceived signal {signum}, shutting down BurstEngine gracefully...")
            self.stop()
        # Register signal handlers for graceful shutdown only in main thread
        if threading.current_thread() is threading.main_thread():
            signal.signal(signal.SIGINT, handle_signal)
            signal.signal(signal.SIGTERM, handle_signal)
        while self._running:
            start = time.perf_counter()
            # 1. Process neuron firing (update membrane potentials and FCL)
            fired_neurons = self.connectome_manager.update_membrane_potentials()
            # 2. Measure actual frequency
            end = time.perf_counter()
            elapsed = end - start
            actual_freq = 1.0 / elapsed if elapsed > 0 else 0
            self.state_manager.set_burst_frequency(actual_freq)
            # 3. Load shedding if needed
            if actual_freq < self.desired_frequency:
                for area_id in self.shed_areas:
                    # Clear FCL for this area for the current burst
                    self.fcl_manager.area_fcl_history[area_id][self.fcl_manager.current_window_index].clear()
            # 4. Sleep for the remainder of the interval
            if elapsed < self.burst_interval:
                time.sleep(self.burst_interval - elapsed)
        logger.info("BurstEngine stopped.")
        self.state_manager.set_burst_engine_state(ServiceState.UNAVAILABLE)

    def stop(self) -> None:
        """Stop the burst engine."""
        self._running = False

    def run_test(self) -> List[int]:
        """
        Run a single burst cycle for testing purposes.
        
        This method executes one iteration of the burst loop without setting up
        signal handlers or entering an infinite loop, making it suitable for testing.
        
        Returns:
            The list of fired neurons from this burst cycle
        """
        start = time.perf_counter()
        
        # Process neuron firing
        fired_neurons = self.connectome_manager.update_membrane_potentials()
        
        # Measure actual frequency
        end = time.perf_counter()
        elapsed = end - start
        actual_freq = 1.0 / elapsed if elapsed > 0 else 0
        self.state_manager.set_burst_frequency(actual_freq)
        
        # Load shedding if needed
        if actual_freq < self.desired_frequency:
            for area_id in self.shed_areas:
                # Clear FCL for this area for the current burst
                self.fcl_manager.area_fcl_history[area_id][self.fcl_manager.current_window_index].clear()
        
        return fired_neurons

    def update_with_genome(self) -> None:
        """Called when a genome is loaded to update burst engine state"""
        self.genome_loaded = True
        # Update cortical areas list and shed areas set
        if hasattr(self.connectome_manager, 'cortical_areas') and self.connectome_manager.cortical_areas:
            self.cortical_areas = list(self.connectome_manager.cortical_areas.values())
            self.shed_areas = set(area.id for area in self.cortical_areas if area.properties.get('__shed', False))
        logger.info("Burst Engine updated with genome information", emoji1="⚡ ")

    def run_with_fire_queue(self, mpf: bool = True, puf: bool = False, max_consecutive_fires: int = 10) -> bool:
        """
        Run the burst engine using the fire queue process.
        
        This method uses the enhanced fire queue process with PSP calculation as 
        described in the architecture documentation.
        
        Args:
            mpf: Membrane Potential Driven PSP Flag
            puf: PSP Uniformity Flag
            max_consecutive_fires: Maximum consecutive fire count before inhibiting firing
            
        Returns:
            True if completed successfully, False otherwise
        """
        if self.state_manager.get_burst_engine_state() != ServiceState.READY:
            logger.warning("Burst engine is not ready, cannot start burst execution")
            return False
            
        # Update state - use READY state to indicate it's running (later could be changed to SYNCING or similar)
        self.state_manager.set_burst_engine_state(ServiceState.READY)
        logger.info("Burst engine starting with fire queue process", emoji1="🚀 ")
        
        # Set running flag
        self._running = True
        
        # Try to use optimized structures if available
        try:
            from feagi.npu.optimized_integration import step_simulation_with_fire_queue
            optimized_available = True
        except ImportError:
            optimized_available = False
        
        # Main loop
        while self._running:
            start_time = time.perf_counter()
            
            # Process bursts using fire queue
            if optimized_available:
                # Get the core from connectome manager
                core = self.connectome_manager.get_optimized_core()
                if core:
                    # Use optimized implementation
                    step_simulation_with_fire_queue(core, mpf, puf, max_consecutive_fires)
                else:
                    # Fall back to standard process
                    self._process_burst()
            else:
                # Fall back to standard process
                self._process_burst()
                
            # Calculate time taken for this burst
            end_time = time.perf_counter()
            elapsed = end_time - start_time
            self.last_burst_time = elapsed
            
            # Calculate actual frequency
            actual_freq = 1.0 / elapsed if elapsed > 0 else 0
            self.state_manager.set_burst_frequency(actual_freq)
            
            # Log performance every 100 bursts
            if self.burst_count % 100 == 0:
                logger.info(f"Processed {self.burst_count} bursts. "
                           f"Target: {self.desired_frequency:.1f}Hz, "
                           f"Actual: {actual_freq:.1f}Hz",
                           emoji1="⚡ ")
            
            # Increment burst count
            self.burst_count += 1
            
            # Sleep if needed to maintain target frequency
            if self.desired_frequency > 0 and elapsed < self.burst_interval:
                time.sleep(self.burst_interval - elapsed)
                
        # Update state when stopped
        self.state_manager.set_burst_engine_state(ServiceState.READY)
        logger.info("Burst engine stopped", emoji1="🛑 ")
        return True

# --- FCLSampler Implementation ---

class FCLSampler:
    """
    FCLSampler: Samples the latest FCL at a configurable frequency and forwards it to consumers (e.g., visualization, motor output).
    - Now supports per-area sample rates using the 'fcl_sample_rate' property in each cortical area's properties dict.
    - RTOS/Rust-friendly: runs as a periodic task/thread, no dynamic allocation in the main loop
    - Supports graceful shutdown
    """
    def __init__(self, fcl_manager: Any, sample_frequency_hz: float, output_queue: Any, connectome_manager: Optional[Any] = None) -> None:
        """
        Initialize the FCL Sampler.
        
        Args:
            fcl_manager: The FCL manager to sample from
            sample_frequency_hz: Frequency to sample FCLs at (global default)
            output_queue: Queue to output sampled FCLs to
            connectome_manager: Optional connectome manager for per-area sampling rates
        """
        self.fcl_manager = fcl_manager
        self.sample_frequency = sample_frequency_hz  # Global default
        self.sample_interval = 1.0 / sample_frequency_hz
        self.output_queue = output_queue
        self.connectome_manager = connectome_manager  # Needed for per-area properties
        self.running = False
        self._last_sample_time_per_area = {}  # area_id -> last sample time
        self._max_retries = 3  # Maximum number of retries for transient errors
        self._retry_delay = 0.01  # Delay between retries in seconds

    def run(self) -> None:
        """
        Run the FCL sampler main loop.
        
        This method continuously samples FCLs according to the configured
        sample rate and outputs them to the queue.
        """
        self.running = True
        while self.running:
            start = time.perf_counter()
            now = start
            # If connectome_manager is provided, support per-area sample rates
            if self.connectome_manager is not None:
                for area in self.connectome_manager._areas.values():
                    area_id = area.id
                    # Get per-area sample rate if set, else use global
                    rate = area.properties.get('fcl_sample_rate', self.sample_frequency)
                    interval = 1.0 / rate if rate > 0 else self.sample_interval
                    last_time = self._last_sample_time_per_area.get(area_id, 0)
                    if now - last_time >= interval:
                        # Sample this area's FCL with retry mechanism
                        retry_count = 0
                        while retry_count < self._max_retries:
                            try:
                                area_fcl = self.fcl_manager.get_area_fcl(area_id)
                                # Put (area_id, area_fcl) in the output queue (non-blocking, drop if full)
                                try:
                                    self.output_queue.put_nowait((area_id, area_fcl))
                                    break  # Success, exit retry loop
                                except Exception as e:
                                    # Queue is likely full, log and continue
                                    if retry_count == self._max_retries - 1:  # Only log on last retry
                                        logger.warning(f"Output queue full, skipping FCL sample for area {area_id}")
                            except Exception as e:
                                # Log error but continue with other areas
                                if retry_count == self._max_retries - 1:  # Only log on last retry
                                    logger.error(f"FCLSampler error (area {area_id}): {e}")
                                # Wait before retrying
                                time.sleep(self._retry_delay)
                            retry_count += 1
                        
                        # Update last sample time even if sampling failed
                        self._last_sample_time_per_area[area_id] = now
            else:
                # Global sampling (legacy behavior)
                retry_count = 0
                while retry_count < self._max_retries:
                    try:
                        fcl_snapshot = self.fcl_manager.get_global_fcl()
                        try:
                            self.output_queue.put_nowait(fcl_snapshot)
                            break  # Success, exit retry loop
                        except Exception as e:
                            # Queue is likely full, log and continue
                            if retry_count == self._max_retries - 1:  # Only log on last retry
                                logger.warning("Output queue full, skipping global FCL sample")
                    except Exception as e:
                        # Log error but continue
                        if retry_count == self._max_retries - 1:  # Only log on last retry
                            logger.error(f"FCLSampler error: {e}")
                        # Wait before retrying
                        time.sleep(self._retry_delay)
                    retry_count += 1
                    
            # Sleep for the remainder of the global sample interval
            elapsed = time.perf_counter() - start
            if elapsed < self.sample_interval:
                time.sleep(self.sample_interval - elapsed)
        logger.info("FCLSampler stopped.")

    def stop(self) -> None:
        """Stop the FCL sampler."""
        self.running = False

    def update_area_sample_rate(self, area_id: int, rate: float) -> None:
        """
        Update the sample rate for a specific area at runtime (live reconfiguration).
        This updates the last sample time and ensures the new rate is used immediately.
        
        Args:
            area_id: ID of the cortical area to update
            rate: New sample rate in Hz
        """
        if self.connectome_manager is not None:
            area = self.connectome_manager._areas.get(area_id)
            if area is not None:
                area.properties['fcl_sample_rate'] = rate
                # Optionally reset last sample time to force immediate sample
                self._last_sample_time_per_area[area_id] = 0 