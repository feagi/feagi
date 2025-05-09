import time
import signal
from feagi.core.state_manager import FeagiStateManager

class BurstEngine:
    """
    RTOS/Rust-friendly burst engine for FEAGI neural simulation.
    - No dynamic allocation in the main loop
    - All configuration and memory allocation happens before entering the loop
    - Main loop is a single, clear sequence of steps
    - Supports graceful shutdown
    """
    def __init__(self, connectome_manager, desired_frequency_hz):
        self.connectome = connectome_manager
        self.desired_frequency = desired_frequency_hz
        self.state_manager = FeagiStateManager.instance()
        self.burst_interval = 1.0 / desired_frequency_hz
        # Pre-allocate cortical area list and shed area set
        self.cortical_areas = list(self.connectome.cortical_areas.values())
        self.shed_areas = set(area.id for area in self.cortical_areas if area.properties.get('__shed', False))
        self.running = False

    def run(self):
        self.running = True
        def handle_signal(signum, frame):
            print(f"\nReceived signal {signum}, shutting down BurstEngine gracefully...")
            self.stop()
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, handle_signal)
        signal.signal(signal.SIGTERM, handle_signal)
        while self.running:
            start = time.perf_counter()
            # 1. Process neuron firing (update membrane potentials and FCL)
            fired_neurons = self.connectome.update_membrane_potentials()
            # 2. Measure actual frequency
            end = time.perf_counter()
            elapsed = end - start
            actual_freq = 1.0 / elapsed if elapsed > 0 else 0
            self.state_manager.set_burst_frequency(actual_freq)
            # 3. Load shedding if needed
            if actual_freq < self.desired_frequency:
                for area_id in self.shed_areas:
                    # Clear FCL for this area for the current burst
                    self.connectome.fcl_manager.area_fcl_history[area_id][self.connectome.fcl_manager.current_window_index].clear()
            # 4. Sleep for the remainder of the interval
            if elapsed < self.burst_interval:
                time.sleep(self.burst_interval - elapsed)
        print("BurstEngine stopped.")

    def stop(self):
        self.running = False

    def run_test(self):
        # This method is added for testing purposes
        # It should be implemented to run the burst loop in a test environment
        pass 