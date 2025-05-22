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
                for cortical_id in self.shed_areas:
                    # Clear FCL for this area for the current burst
                    self.fcl_manager.area_fcl_history[cortical_id][self.fcl_manager.current_window_index].clear()
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
            for cortical_id in self.shed_areas:
                # Clear FCL for this area for the current burst
                self.fcl_manager.area_fcl_history[cortical_id][self.fcl_manager.current_window_index].clear()
        
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
    - Only samples when there are consumers (visualization clients or motor outputs)
    - Uses best-effort delivery - silently drops samples when output queue is full to prioritize real-time performance
    """
    def __init__(self, fcl_manager: Any, sample_frequency_hz: float, output_queue: Any, connectome_manager: Optional[Any] = None) -> None:
        """
        Initialize the FCL Sampler.
        
        Args:
            fcl_manager: FCL manager instance to sample from
            sample_frequency_hz: Sampling frequency in Hz
            output_queue: Queue to put sampled FCL data into
            connectome_manager: Optional connectome manager to get cortical area properties
        """
        self.fcl_manager = fcl_manager
        self.sample_frequency = sample_frequency_hz  # Global default
        self.sample_interval = 1.0 / sample_frequency_hz
        self.output_queue = output_queue
        self.connectome_manager = connectome_manager  # Needed for per-area properties
        
        self.running = False
        self._last_sample_time_per_area = {}  # cortical_id -> last sample time
        self._max_retries = 2  # Maximum number of retries for transient errors
        self._retry_delay = 0.01  # Delay between retries in seconds
        
        # Visualization clients tracking
        self._has_visualization_subscribers = False
        self._has_motor_subscribers = False
        
    def set_visualization_subscribers(self, has_subscribers: bool) -> None:
        """
        Set whether there are visualization subscribers.
        
        Args:
            has_subscribers: True if there are visualization subscribers, False otherwise
        """
        if has_subscribers != self._has_visualization_subscribers:
            logger.info(f"FCLSampler visualization subscribers changed: {has_subscribers}")
            self._has_visualization_subscribers = has_subscribers
            
            # Check if test visualization mode is enabled
            test_viz_mode = False
            try:
                from feagi.core.state_manager import FeagiStateManager
                state_manager = FeagiStateManager.instance()
                test_viz_mode = state_manager.get_test_visualization_mode()
                if test_viz_mode and has_subscribers:
                    logger.debug("TEST VISUALIZATION MODE IS ACTIVE - Will log raw neuron data")
            except Exception as e:
                pass

    def run(self) -> None:
        """
        Run the FCL sampler main loop.
        
        This method continuously samples FCLs according to the configured
        sample rate and outputs them to the queue using a best-effort approach.
        When the output queue is full, new samples are silently dropped to maintain
        real-time performance.
        """
        self.running = True
        while self.running:
            start = time.perf_counter()
            now = start
            
            # For test visualization mode - collect all data in one dictionary
            combined_neuron_data = {}
            in_test_viz_mode = False
            try:
                from feagi.core.state_manager import FeagiStateManager
                state_manager = FeagiStateManager.instance()
                in_test_viz_mode = state_manager.get_test_visualization_mode()
            except Exception:
                pass
            
            # Skip sampling if no subscribers
            if not self._has_visualization_subscribers and not self._has_motor_subscribers:
                # Sleep for the sample interval and check again
                time.sleep(self.sample_interval)
                continue
                
            # If connectome_manager is provided, support per-area sample rates
            if self.connectome_manager is not None:
                for area in self.connectome_manager.cortical_areas.values():
                    cortical_id = area.id
                    # Get per-area sample rate if set, else use global
                    rate = area.properties.get('fcl_sample_rate', self.sample_frequency)
                    interval = 1.0 / rate if rate > 0 else self.sample_interval
                    last_time = self._last_sample_time_per_area.get(cortical_id, 0)
                    if now - last_time >= interval:
                        # Sample this area's FCL with retry mechanism
                        retry_count = 0
                        while retry_count < self._max_retries:
                            try:
                                area_fcl = self.fcl_manager.get_cortical_fcl(cortical_id)
                                # Put (cortical_id, area_fcl) in the output queue (non-blocking, drop if full)
                                try:
                                    self.output_queue.put_nowait((cortical_id, area_fcl))
                                    
                                    # Check if in test visualization mode and log the raw data
                                    try:
                                        from feagi.core.state_manager import FeagiStateManager
                                        state_manager = FeagiStateManager.instance()
                                        if state_manager.get_test_visualization_mode():
                                            # Format the data for logging
                                            if area_fcl:
                                                # Convert the bitmap to a list format
                                                neuron_ids = list(area_fcl)
                                                x_values = []
                                                y_values = []
                                                z_values = []
                                                potentials = []
                                                
                                                # Get the area object to access neuron positions
                                                area_obj = self.connectome_manager.cortical_areas.get(cortical_id)
                                                if area_obj:
                                                    for neuron_id in neuron_ids:
                                                        # Try to get neuron position
                                                        try:
                                                            # If get_neuron_by_id exists, use it
                                                            if hasattr(area_obj, 'get_neuron_by_id'):
                                                                neuron = area_obj.get_neuron_by_id(neuron_id)
                                                                if neuron and hasattr(neuron, 'position'):
                                                                    x, y, z = neuron.position
                                                                else:
                                                                    # Estimate position from ID
                                                                    dimensions = area_obj.properties.get('dimensions', {'x': 10, 'y': 10, 'z': 1})
                                                                    x = neuron_id % dimensions['x']
                                                                    y = (neuron_id // dimensions['x']) % dimensions['y']
                                                                    z = (neuron_id // (dimensions['x'] * dimensions['y'])) % dimensions['z']
                                                            else:
                                                                # Estimate position from ID
                                                                dimensions = area_obj.properties.get('dimensions', {'x': 10, 'y': 10, 'z': 1})
                                                                x = neuron_id % dimensions['x']
                                                                y = (neuron_id // dimensions['x']) % dimensions['y']
                                                                z = (neuron_id // (dimensions['x'] * dimensions['y'])) % dimensions['z']
                                                        except Exception as e:
                                                            # Fallback to default coordinates
                                                            x, y, z = 0, 0, 0
                                                        
                                                        # Add coordinates and default potential for this neuron
                                                        x_values.append(x)
                                                        y_values.append(y)
                                                        z_values.append(z)
                                                        potentials.append(1.0)  # Default potential for firing neurons
                                                
                                                # Format cortical ID (6 characters) and add to combined data
                                                cort_id_6 = cortical_id[:6].ljust(6)
                                                combined_neuron_data[cort_id_6] = [x_values, y_values, z_values, potentials]
                                    except Exception as e:
                                        # Don't let errors in test mode logging affect normal operation
                                        print(f"Error in test visualization logging: {e}")
                                        import traceback
                                        print(traceback.format_exc())
                                    
                                    break  # Success, exit retry loop
                                except Exception as e:
                                    # Queue is full - silently drop instead of logging warnings
                                    # This implements a conflating behavior where we prioritize new data
                                    break  # Skip retries when queue is full
                            except Exception as e:
                                # Log error but continue with other areas
                                if retry_count == self._max_retries - 1:  # Only log on last retry
                                    logger.error(f"FCLSampler error (area {cortical_id}): {e}")
                                # Wait before retrying
                                time.sleep(self._retry_delay)
                            retry_count += 1
                        
                        # Update last sample time even if sampling failed
                        self._last_sample_time_per_area[cortical_id] = now
            else:
                # Global sampling (legacy behavior)
                retry_count = 0
                while retry_count < self._max_retries:
                    try:
                        fcl_snapshot = self.fcl_manager.get_global_fcl()
                        try:
                            self.output_queue.put_nowait(fcl_snapshot)
                            
                            # Check if in test visualization mode
                            try:
                                from feagi.core.state_manager import FeagiStateManager
                                state_manager = FeagiStateManager.instance()
                                if state_manager.get_test_visualization_mode():
                                    # If it's a dictionary, add it to the combined data
                                    if isinstance(fcl_snapshot, dict):
                                        # Add each area's data to the combined dictionary
                                        for cortical_id, fcl_data in fcl_snapshot.items():
                                            cort_id_6 = cortical_id[:6].ljust(6)
                                            # Process FCL data based on its structure
                                            # (This is an example - actual format may vary)
                                            x_values, y_values, z_values, potentials = [], [], [], []
                                            if isinstance(fcl_data, set):
                                                # It's just a set of neuron IDs
                                                for neuron_id in fcl_data:
                                                    # Simplified coordinates based on neuron ID
                                                    x_values.append(neuron_id % 10)
                                                    y_values.append((neuron_id // 10) % 10)
                                                    z_values.append(0)
                                                    potentials.append(1.0)
                                            combined_neuron_data[cort_id_6] = [x_values, y_values, z_values, potentials]
                                    # For debugging, still log a summary of the snapshot
                                    print(f"\n=== GLOBAL FCL SNAPSHOT: {type(fcl_snapshot)} ===")
                                    if isinstance(fcl_snapshot, dict):
                                        print(f"Contains data for {len(fcl_snapshot)} areas")
                                    elif isinstance(fcl_snapshot, bytes):
                                        # If it's bytes, print first 50 bytes as hex
                                        hex_dump = ' '.join([f'{b:02x}' for b in fcl_snapshot[:50]])
                                        print(f"First 50 bytes: {hex_dump}")
                                    print("================================\n")
                            except Exception as e:
                                # Don't let test mode logging failures affect normal operation
                                print(f"Error in global test visualization logging: {e}")
                            
                            break  # Success, exit retry loop
                        except Exception as e:
                            # Log error but continue
                            if retry_count == self._max_retries - 1:  # Only log on last retry
                                logger.error(f"FCLSampler error: {e}")
                            # Wait before retrying
                            time.sleep(self._retry_delay)
                        retry_count += 1
                    except Exception as e:
                        # Log error but continue
                        if retry_count == self._max_retries - 1:  # Only log on last retry
                            logger.error(f"FCLSampler error: {e}")
                        # Wait before retrying
                        time.sleep(self._retry_delay)
                    retry_count += 1
                    
            # Sleep for the remainder of the global sample interval
            elapsed = time.perf_counter() - start
            
            # Print combined neuron data if in test visualization mode
            if in_test_viz_mode and combined_neuron_data:
                # Log the combined data
                import json
                combined_data_str = json.dumps(combined_neuron_data, separators=(',', ':'))
                logger.debug(f"COMBINED NEURON DATA ({len(combined_neuron_data)} areas):")
                logger.debug(combined_data_str)
                
                # Also print to stdout directly for maximum visibility
                print(f"\n=== COMBINED NEURON DATA FOR {len(combined_neuron_data)} AREAS ===")
                print(combined_data_str)
                print("===========================================================\n")
            
            if elapsed < self.sample_interval:
                time.sleep(self.sample_interval - elapsed)
        logger.info("FCLSampler stopped.")

    def stop(self) -> None:
        """Stop the FCL sampler."""
        self.running = False
        
    def update_area_sample_rate(self, cortical_id, rate):
        """Set the sampling rate for a specific cortical area."""
        # Store the last sample time to avoid immediate sampling
        if cortical_id not in self._last_sample_time_per_area:
            self._last_sample_time_per_area[cortical_id] = time.perf_counter()
        # Rate will be picked up from cortical area properties
        
    def set_motor_subscribers(self, has_subscribers: bool) -> None:
        """
        Update whether there are motor subscribers.
        
        Args:
            has_subscribers: Whether there are motor subscribers
        """
        if has_subscribers != self._has_motor_subscribers:
            logger.info(f"FCLSampler motor subscribers changed: {has_subscribers}")
            self._has_motor_subscribers = has_subscribers 

class FQSampler:
    """
    Fire Queue Sampler for visualization.
    
    Samples neuron firing data directly from the fire queue, providing richer
    information including membrane potentials, thresholds, and coordinates.
    This replaces FCLSampler which only provided neuron IDs.
    """
    
    def __init__(self, fire_queue_provider: Any, sample_frequency_hz: float, 
                 output_queue: Any, connectome_manager: Optional[Any] = None) -> None:
        """
        Initialize FQ sampler.
        
        Args:
            fire_queue_provider: Object that provides access to fire queue data
            sample_frequency_hz: Sampling rate in Hz
            output_queue: Queue to put sampled data into
            connectome_manager: Optional connectome manager for area-specific sampling
        """
        self.fire_queue_provider = fire_queue_provider
        self.sample_frequency = sample_frequency_hz
        self.sample_interval = 1.0 / sample_frequency_hz if sample_frequency_hz > 0 else 0.1
        self.output_queue = output_queue
        self.connectome_manager = connectome_manager
        self.running = False
        
        # Per-area sampling support
        self._last_sample_time_per_area = {}
        
        # Subscriber tracking
        self._has_visualization_subscribers = False
        self._has_motor_subscribers = False
        
        # Error handling
        self._max_retries = 3
        self._retry_delay = 0.001
        
        logger.info(f"FQSampler initialized with {sample_frequency_hz}Hz sampling")

    def set_visualization_subscribers(self, has_subscribers: bool) -> None:
        """
        Update whether there are visualization subscribers.
        
        Args:
            has_subscribers: Whether there are visualization subscribers
        """
        if has_subscribers != self._has_visualization_subscribers:
            logger.info(f"FQSampler visualization subscribers changed: {has_subscribers}")
            self._has_visualization_subscribers = has_subscribers

    def set_motor_subscribers(self, has_subscribers: bool) -> None:
        """
        Update whether there are motor subscribers.
        
        Args:
            has_subscribers: Whether there are motor subscribers
        """
        if has_subscribers != self._has_motor_subscribers:
            logger.info(f"FQSampler motor subscribers changed: {has_subscribers}")
            self._has_motor_subscribers = has_subscribers

    def run(self) -> None:
        """Main sampling loop."""
        logger.info("FQSampler started")
        self.running = True
        
        while self.running:
            start = time.perf_counter()
            now = time.perf_counter()
            
            # Skip sampling if no subscribers
            if not self._has_visualization_subscribers and not self._has_motor_subscribers:
                time.sleep(self.sample_interval)
                continue
                
            # Sample fire queue data
            if self.connectome_manager is not None:
                # Per-area sampling
                for area in self.connectome_manager.cortical_areas.values():
                    cortical_id = area.id
                    # Get per-area sample rate if set, else use global
                    rate = area.properties.get('fq_sample_rate', self.sample_frequency)
                    
                    # Skip sampling if rate is zero
                    if rate <= 0:
                        continue
                        
                    interval = 1.0 / rate
                    last_time = self._last_sample_time_per_area.get(cortical_id, 0)
                    
                    if now - last_time >= interval:
                        self._sample_area_fire_queue(cortical_id)
                        self._last_sample_time_per_area[cortical_id] = now
            else:
                # Global sampling
                self._sample_global_fire_queue()
                    
            # Sleep for the remainder of the sample interval
            elapsed = time.perf_counter() - start
            if elapsed < self.sample_interval:
                time.sleep(self.sample_interval - elapsed)
                
        logger.info("FQSampler stopped.")

    def _sample_area_fire_queue(self, cortical_id: str) -> None:
        """Sample fire queue data for a specific cortical area."""
        retry_count = 0
        while retry_count < self._max_retries:
            try:
                # Get fire queue data for this area
                area_fire_data = self._get_area_fire_queue_data(cortical_id)
                
                if area_fire_data:
                    # Put (cortical_id, fire_data) in the output queue
                    try:
                        self.output_queue.put_nowait((cortical_id, area_fire_data))
                        break  # Success
                    except Exception:
                        # Queue full - drop data (conflating behavior)
                        break
                else:
                    # No fire data for this area
                    break
                    
            except Exception as e:
                if retry_count == self._max_retries - 1:
                    logger.error(f"FQSampler error (area {cortical_id}): {e}")
                time.sleep(self._retry_delay)
            retry_count += 1

    def _sample_global_fire_queue(self) -> None:
        """Sample global fire queue data."""
        retry_count = 0
        while retry_count < self._max_retries:
            try:
                # Get global fire queue data
                fire_data = self._get_global_fire_queue_data()
                
                if fire_data:
                    try:
                        self.output_queue.put_nowait(fire_data)
                        break  # Success
                    except Exception:
                        # Queue full - drop data
                        break
                else:
                    break
                    
            except Exception as e:
                if retry_count == self._max_retries - 1:
                    logger.error(f"FQSampler error: {e}")
                time.sleep(self._retry_delay)
            retry_count += 1

    def _get_area_fire_queue_data(self, cortical_id: str) -> Optional[Dict[str, Any]]:
        """
        Get fire queue data for a specific cortical area.
        
        Returns:
            Dictionary with fire queue data: {
                'neuron_ids': List[int],
                'membrane_potentials': List[float], 
                'thresholds': List[float],
                'consecutive_fire_counts': List[int],
                'refractory_counters': List[int],
                'coordinates': List[Tuple[int, int, int]]  # (x, y, z) positions
            }
        """
        try:
            # Get fire queue from provider
            if hasattr(self.fire_queue_provider, 'get_area_fire_queue'):
                fire_queue = self.fire_queue_provider.get_area_fire_queue(cortical_id)
            elif hasattr(self.fire_queue_provider, 'get_fire_queue'):
                # Filter global fire queue for this area
                global_fire_queue = self.fire_queue_provider.get_fire_queue()
                fire_queue = self._filter_fire_queue_by_area(global_fire_queue, cortical_id)
            else:
                return None
                
            if not fire_queue or not fire_queue.get('neuron_ids'):
                return None
                
            # Add coordinate information
            coordinates = self._get_neuron_coordinates(cortical_id, fire_queue['neuron_ids'])
            fire_queue['coordinates'] = coordinates
            
            return fire_queue
            
        except Exception as e:
            logger.error(f"Error getting area fire queue data for {cortical_id}: {e}")
            return None

    def _get_global_fire_queue_data(self) -> Optional[Dict[str, Any]]:
        """Get global fire queue data."""
        try:
            if hasattr(self.fire_queue_provider, 'get_fire_queue'):
                fire_queue = self.fire_queue_provider.get_fire_queue()
                
                if not fire_queue or not fire_queue.get('neuron_ids'):
                    return None
                    
                # Add coordinate information
                coordinates = self._get_global_neuron_coordinates(fire_queue['neuron_ids'])
                fire_queue['coordinates'] = coordinates
                
                return fire_queue
            return None
            
        except Exception as e:
            logger.error(f"Error getting global fire queue data: {e}")
            return None

    def _filter_fire_queue_by_area(self, fire_queue: Dict[str, Any], cortical_id: str) -> Dict[str, Any]:
        """Filter fire queue data to only include neurons from specified area."""
        if not fire_queue or not self.connectome_manager:
            return fire_queue
            
        try:
            area = self.connectome_manager.cortical_areas.get(cortical_id)
            if not area:
                return {'neuron_ids': [], 'membrane_potentials': [], 'thresholds': [], 
                       'consecutive_fire_counts': [], 'refractory_counters': []}
                
            # Get neuron ID range for this area
            area_neuron_ids = set(area.get_neuron_ids()) if hasattr(area, 'get_neuron_ids') else set()
            
            # Filter fire queue data
            filtered_data = {
                'neuron_ids': [],
                'membrane_potentials': [],
                'thresholds': [],
                'consecutive_fire_counts': [],
                'refractory_counters': []
            }
            
            for i, neuron_id in enumerate(fire_queue.get('neuron_ids', [])):
                if neuron_id in area_neuron_ids:
                    filtered_data['neuron_ids'].append(neuron_id)
                    filtered_data['membrane_potentials'].append(fire_queue['membrane_potentials'][i])
                    filtered_data['thresholds'].append(fire_queue['thresholds'][i])
                    filtered_data['consecutive_fire_counts'].append(fire_queue['consecutive_fire_counts'][i])
                    filtered_data['refractory_counters'].append(fire_queue['refractory_counters'][i])
                    
            return filtered_data
            
        except Exception as e:
            logger.error(f"Error filtering fire queue by area {cortical_id}: {e}")
            return fire_queue

    def _get_neuron_coordinates(self, cortical_id: str, neuron_ids: List[int]) -> List[tuple]:
        """Get 3D coordinates for neurons in a specific cortical area."""
        coordinates = []
        
        try:
            if self.connectome_manager:
                area = self.connectome_manager.cortical_areas.get(cortical_id)
                if area:
                    for neuron_id in neuron_ids:
                        try:
                            # Try to get actual neuron position
                            if hasattr(area, 'get_neuron_by_id'):
                                neuron = area.get_neuron_by_id(neuron_id)
                                if neuron and hasattr(neuron, 'position'):
                                    coordinates.append(neuron.position)
                                    continue
                            
                            # Fallback: estimate position from area dimensions
                            dimensions = area.properties.get('dimensions', {'x': 10, 'y': 10, 'z': 1})
                            x = neuron_id % dimensions['x']
                            y = (neuron_id // dimensions['x']) % dimensions['y']
                            z = (neuron_id // (dimensions['x'] * dimensions['y'])) % dimensions['z']
                            coordinates.append((x, y, z))
                            
                        except Exception:
                            # Default coordinates
                            coordinates.append((0, 0, 0))
                else:
                    # No area found, use default coordinates
                    coordinates = [(0, 0, 0) for _ in neuron_ids]
            else:
                # No connectome manager, estimate coordinates
                coordinates = [(nid % 100, (nid // 100) % 100, nid // 10000) for nid in neuron_ids]
                
        except Exception as e:
            logger.error(f"Error getting neuron coordinates: {e}")
            coordinates = [(0, 0, 0) for _ in neuron_ids]
            
        return coordinates

    def _get_global_neuron_coordinates(self, neuron_ids: List[int]) -> List[tuple]:
        """Get 3D coordinates for neurons globally."""
        coordinates = []
        
        try:
            if self.connectome_manager:
                # Map neuron IDs to their areas and get coordinates
                for neuron_id in neuron_ids:
                    found = False
                    for cortical_id, area in self.connectome_manager.cortical_areas.items():
                        try:
                            if hasattr(area, 'contains_neuron') and area.contains_neuron(neuron_id):
                                coord = self._get_neuron_coordinates(cortical_id, [neuron_id])
                                coordinates.append(coord[0] if coord else (0, 0, 0))
                                found = True
                                break
                        except Exception:
                            continue
                    
                    if not found:
                        # Fallback coordinate calculation
                        x = neuron_id % 100
                        y = (neuron_id // 100) % 100
                        z = neuron_id // 10000
                        coordinates.append((x, y, z))
            else:
                # No connectome manager, use simple mapping
                coordinates = [(nid % 100, (nid // 100) % 100, nid // 10000) for nid in neuron_ids]
                
        except Exception as e:
            logger.error(f"Error getting global neuron coordinates: {e}")
            coordinates = [(0, 0, 0) for _ in neuron_ids]
            
        return coordinates

    def stop(self) -> None:
        """Stop the FQ sampler."""
        self.running = False
        
    def update_area_sample_rate(self, cortical_id: str, rate: float) -> None:
        """Set the sampling rate for a specific cortical area."""
        if cortical_id not in self._last_sample_time_per_area:
            self._last_sample_time_per_area[cortical_id] = time.perf_counter() 