import time
import signal
import threading
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
    def __init__(self, connectome_manager, fcl_manager, config=None):
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
        
        # Initialize in a valid but inactive state
        # Will become fully operational when a genome is loaded
        logger.info("Burst Engine initialized in standby mode", emoji1="⚡️")
        
        self.state_manager = FeagiStateManager.instance()
        
        # Support both parameter names for backward compatibility
        self.desired_frequency = self.config.get('desired_frequency_hz', 
                                              self.config.get('target_frequency', 100.0))
        self.target_frequency = self.desired_frequency  # For backward compatibility
        self.burst_interval = 1.0 / self.desired_frequency
        
        # Use _areas instead of cortical_areas - fix the attribute name
        self.cortical_areas = list(self.connectome_manager._areas.values()) if hasattr(self.connectome_manager, '_areas') else []
        self.shed_areas = set(area.id for area in self.cortical_areas if area.properties.get('__shed', False))

    def run(self):
        self._running = True
        self.state_manager.set_burst_engine_state(ServiceState.READY)
        def handle_signal(signum, frame):
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

    def stop(self):
        self._running = False

    def run_test(self):
        # This method is added for testing purposes
        # It should be implemented to run the burst loop in a test environment
        pass 

    def update_with_genome(self):
        """Called when a genome is loaded to update burst engine state"""
        self.genome_loaded = True
        # Use is_ready instead of is_active to check connectome status
        if hasattr(self.connectome_manager, 'is_ready') and self.connectome_manager.is_ready():
            self.cortical_areas = list(self.connectome_manager._areas.values())
            self.shed_areas = set(area.id for area in self.cortical_areas if area.properties.get('__shed', False))
        logger.info("Burst Engine updated with genome information", emoji1="⚡ ")

# --- FCLSampler Implementation ---

class FCLSampler:
    """
    FCLSampler: Samples the latest FCL at a configurable frequency and forwards it to consumers (e.g., visualization, motor output).
    - Now supports per-area sample rates using the 'fcl_sample_rate' property in each cortical area's properties dict.
    - RTOS/Rust-friendly: runs as a periodic task/thread, no dynamic allocation in the main loop
    - Supports graceful shutdown
    """
    def __init__(self, fcl_manager, sample_frequency_hz, output_queue, connectome_manager=None):
        self.fcl_manager = fcl_manager
        self.sample_frequency = sample_frequency_hz  # Global default
        self.sample_interval = 1.0 / sample_frequency_hz
        self.output_queue = output_queue
        self.connectome_manager = connectome_manager  # Needed for per-area properties
        self.running = False
        self._last_sample_time_per_area = {}  # area_id -> last sample time

    def run(self):
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
                        # Sample this area's FCL
                        try:
                            area_fcl = self.fcl_manager.get_area_fcl(area_id)
                            # Put (area_id, area_fcl) in the output queue (non-blocking, drop if full)
                            try:
                                self.output_queue.put_nowait((area_id, area_fcl))
                            except Exception:
                                pass  # Optionally log dropped samples
                        except Exception as e:
                            logger.error(f"FCLSampler error (area {area_id}): {e}")
                        self._last_sample_time_per_area[area_id] = now
            else:
                # Global sampling (legacy behavior)
                try:
                    fcl_snapshot = self.fcl_manager.get_global_fcl()
                    try:
                        self.output_queue.put_nowait(fcl_snapshot)
                    except Exception:
                        pass
                except Exception as e:
                    logger.error(f"FCLSampler error: {e}")
            # Sleep for the remainder of the global sample interval
            elapsed = time.perf_counter() - start
            if elapsed < self.sample_interval:
                time.sleep(self.sample_interval - elapsed)
        logger.info("FCLSampler stopped.")

    def stop(self):
        self.running = False

    def update_area_sample_rate(self, area_id, rate):
        """
        Update the sample rate for a specific area at runtime (live reconfiguration).
        This updates the last sample time and ensures the new rate is used immediately.
        """
        if self.connectome_manager is not None:
            area = self.connectome_manager._areas.get(area_id)
            if area is not None:
                area.properties['fcl_sample_rate'] = rate
                # Optionally reset last sample time to force immediate sample
                self._last_sample_time_per_area[area_id] = 0 