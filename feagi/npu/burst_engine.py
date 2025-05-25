import time
# RTOS-COMPATIBLE: Removed signal and threading imports - not available in RTOS
# import signal  # REMOVED: Not compatible with RTOS
# import threading  # REMOVED: Not compatible with RTOS - use RTOS task primitives instead
import os  # Add for environment variable checking
from typing import Dict, List, Optional, Set, Any, Union
from feagi.core.state_manager import FeagiStateManager, ServiceState
from feagi.utils.logger import setup_logger

# New imports for power area injection
from feagi.npu.special_area_handler import SpecialAreaHandler
from feagi.npu.fcl_injection_service import FCLInjectionService

logger = setup_logger()

# RTOS-COMPATIBLE: Use deterministic pseudo-random for instance IDs
def _generate_instance_id() -> int:
    """Generate a deterministic instance ID without using random module."""
    import time
    # Use time-based deterministic ID generation instead of random
    return int((time.perf_counter() * 1000000) % 10000) + 1000



"""
Burst Engine Implementation for FEAGI.

The BurstEngine is FEAGI's primary neural simulation component. It drives the dynamics 
of neuron firing, manages membrane potentials, and coordinates the Fire Candidate List (FCL).

Key features:
- Standby Mode: Initializes without requiring a genome
- RTOS-Friendly: Designed for real-time operating systems with predictable timing  
- State-Driven: Uses explicit state transitions with consistent logging
- Dependency Injected: No global state, all dependencies passed explicitly
- Power Area Support: Handles special cortical areas like "___pwr" with automatic injection
- Singleton Pattern: Only one instance can exist at any time

Usage:
    # Create and initialize (will return existing instance if one exists)
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
    - New: Supports special area handling including power area injection
    - Singleton: Only one instance can exist at any time
    """
    
    _instance = None
    _instance_id = None
    _lock = None
    
    def __new__(cls, connectome_manager: Any, fcl_manager: Optional[Any] = None, config: Optional[Dict[str, Any]] = None):
        """
        Singleton pattern implementation to ensure only one BurstEngine instance exists.
        """
        if cls._instance is None:
            cls._instance = super(BurstEngine, cls).__new__(cls)
            cls._instance_id = _generate_instance_id()
            print(f"🔥 BURST ENGINE: Creating NEW singleton instance {cls._instance_id}")
            logger.info(f"🔥 BURST ENGINE: Created new singleton instance {cls._instance_id}")
        else:
            print(f"🔥 BURST ENGINE: Returning EXISTING singleton instance {cls._instance_id}")
            logger.info(f"🔥 BURST ENGINE: Returning existing singleton instance {cls._instance_id}")
        return cls._instance
    
    @property
    def _running(self):
        """Get the running state with debug tracking."""
        return getattr(self, '_running_state', False)
    
    @_running.setter
    def _running(self, value):
        """Set the running state with debug tracking."""
        import traceback
        old_value = getattr(self, '_running_state', False)
        self._running_state = value
        
        if os.environ.get('FEAGI_DEBUG_NPU') == '1' and old_value != value:
            print(f"🔥 BURST ENGINE: Instance {self._instance_id} _running changed: {old_value} → {value}")
            print(f"🔥 BURST ENGINE: Stack trace:")
            for line in traceback.format_stack()[-3:-1]:  # Show last 2 stack frames
                print(f"    {line.strip()}")
            print()

    def __init__(self, connectome_manager: Any, fcl_manager: Optional[Any] = None, config: Optional[Dict[str, Any]] = None) -> None:
        """
        Initialize the Burst Engine.
        
        Args:
            connectome_manager: The connectome manager
            fcl_manager: FCL manager (optional)
            config: Configuration parameters (optional)
        """
        # Prevent re-initialization if already initialized
        if hasattr(self, '_initialized') and self._initialized:
            print(f"🔥 BURST ENGINE: Instance {self._instance_id} already initialized, skipping")
            logger.info(f"🔥 BURST ENGINE: Instance {self._instance_id} already initialized, skipping")
            return
            
        print(f"🔥 BURST ENGINE: Initializing singleton instance {self._instance_id}")
        logger.info(f"🔥 BURST ENGINE: Initializing singleton instance {self._instance_id}")
        
        self.connectome_manager = connectome_manager
        self.fcl_manager = fcl_manager or connectome_manager.fcl_manager
        self.config = config or {}
        self.genome_loaded = False
        self._running = False  # This will now trigger the setter with debug logging
        self.burst_count = 0
        self.last_burst_time = 0.0
        
        # Initialize special area handler and injection service
        self.special_area_handler: Optional[SpecialAreaHandler] = None
        self.fcl_injection_service: Optional[FCLInjectionService] = None
        
        # Power area injection configuration
        self.enable_power_injection = self.config.get('enable_power_injection', True)
        self.power_injection_timing = self.config.get('power_injection_timing', 'pre_burst')
        
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
        
        # Initialize special area handling if a genome is already loaded
        if self.cortical_areas:
            self._initialize_special_area_services()
        
        # Initialize frequency measurement system
        self._burst_timing_buffer = []  # Circular buffer for burst durations
        self._processing_timing_buffer = []  # Circular buffer for pure processing durations
        self._timing_buffer_size = 100  # Keep last 100 burst measurements
        self._last_frequency_update = 0.0
        self._frequency_measurement_enabled = False  # Only enable when requested via API
        
        # Mark as initialized
        self._initialized = True
        print(f"🔥 BURST ENGINE: Instance {self._instance_id} initialization complete")
        logger.info(f"🔥 BURST ENGINE: Instance {self._instance_id} initialization complete")
    
    @classmethod
    def get_instance(cls) -> Optional['BurstEngine']:
        """Get the current singleton instance if it exists."""
        return cls._instance
    
    @classmethod
    def reset_singleton(cls):
        """Reset the singleton instance. USE WITH EXTREME CAUTION - for testing only."""
        cls._instance = None
        cls._instance_id = None

    def _initialize_special_area_services(self) -> None:
        """
        Initialize special area handler and injection services.
        
        This is called when a genome is loaded or when the burst engine is created
        with an already-loaded connectome.
        """
        if not self.enable_power_injection:
            logger.info("Power injection disabled by configuration", emoji1="⚡")
            return
        
        try:
            # Initialize special area handler
            self.special_area_handler = SpecialAreaHandler(
                connectome_manager=self.connectome_manager,
                config=self.config.get('special_area_config', {})
            )
            
            # Detect special areas
            self.special_area_handler.detect_special_areas()
            
            # Initialize FCL injection service if power areas detected
            power_areas = self.special_area_handler.get_power_areas()
            if power_areas:
                self.fcl_injection_service = FCLInjectionService(
                    fcl_manager=self.fcl_manager,
                    special_area_handler=self.special_area_handler,
                    config=self.config.get('fcl_injection_config', {})
                )
                
                logger.info(f"Initialized power injection for {len(power_areas)} power areas", emoji1="💉")
                
                # Log power area preview
                preview = self.fcl_injection_service.get_power_injection_preview()
                logger.debug(f"Power injection preview: {preview}")
            else:
                logger.info("No power areas detected, injection service not initialized", emoji1="⚡")
                
        except Exception as e:
            logger.error(f"Error initializing special area services: {e}")
            self.special_area_handler = None
            self.fcl_injection_service = None

    def _process_burst(self) -> List[int]:
        """
        Process a single burst cycle using the standard method.
        
        This method updates membrane potentials and processes neuron firing.
        It's used as a fallback when optimized implementations are not available.
        
        Returns:
            List of neuron IDs that fired in this burst
        """
        # Debug logging if --debug-npu is enabled
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            print(f"🔥 BURST ENGINE _process_burst called! Instance {self._instance_id}, Burst count: {self.burst_count}")
            
            # Check injection service availability
            if self.fcl_injection_service:
                print(f"🔥 BURST ENGINE: Injection service AVAILABLE")
            else:
                print(f"🔥 BURST ENGINE: NO INJECTION SERVICE!")
        
        # 1. Pre-burst power injection
        if self.fcl_injection_service and self.power_injection_timing == 'pre_burst':
            if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                print(f"🔥 BURST ENGINE: Calling pre-burst injection")
            self.fcl_injection_service.inject_pre_burst(self.burst_count)
        
        # 2. Update membrane potentials and get fired neurons
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            # FCL manager uses sliding window with current timestep always 0
            current_timestep = 0  # Fixed: always use 0 for current timestep
            print(f"🔥 BURST ENGINE: About to call update_membrane_potentials with timestep {current_timestep}")
        
        fired_neurons = self.connectome_manager.update_membrane_potentials()
        
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            fired_count = len(fired_neurons) if fired_neurons else 0
            print(f"🔥 BURST ENGINE: Got {fired_count} firing neurons")
        
        # 3. During-burst injection (for modulators)
        if self.fcl_injection_service and self.power_injection_timing == 'during_burst':
            if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                print(f"🔥 BURST ENGINE: Calling during-burst injection")
            self.fcl_injection_service.inject_during_burst(self.burst_count)
        
        # 4. Post-burst injection  
        if self.fcl_injection_service and self.power_injection_timing == 'post_burst':
            if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                print(f"🔥 BURST ENGINE: Calling post-burst injection")
            self.fcl_injection_service.inject_post_burst(self.burst_count)
        
        # 5. Debug fire queue output if --debug-npu flag is enabled
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            self._debug_fire_queue_output()
        
        return fired_neurons

    def _process_burst_with_power_injection(self, current_timestep: int) -> List[int]:
        """
        Enhanced burst processing with power area injection.
        
        Args:
            current_timestep: Current simulation timestep (should be 0 for current)
            
        Returns:
            List of neuron IDs that fired in this burst
        """
        # Debug logging if --debug-npu is enabled
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            print(f"🔥 BURST ENGINE _process_burst_with_power_injection called! Instance {self._instance_id}, Timestep: {current_timestep}")
            
            # Check injection service availability
            if self.fcl_injection_service:
                print(f"🔥 BURST ENGINE: Enhanced injection service AVAILABLE")
            else:
                print(f"🔥 BURST ENGINE: NO ENHANCED INJECTION SERVICE!")
        
        # 1. Pre-burst power injection (inject power area neurons)
        if self.fcl_injection_service:
            if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                print(f"🔥 BURST ENGINE: Calling enhanced pre-burst injection")
            injected_pre = self.fcl_injection_service.inject_pre_burst(current_timestep)
            if injected_pre > 0:
                logger.debug(f"Pre-burst injection: {injected_pre} neurons")
                if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                    print(f"🔥 BURST ENGINE: Pre-burst injected {injected_pre} neurons")
        
        # 2. Standard burst processing (membrane potential updates, regular firing)
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            print(f"🔥 BURST ENGINE: About to call enhanced update_membrane_potentials with timestep {current_timestep}")
        
        fired_neurons = self.connectome_manager.update_membrane_potentials()
        
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            fired_count = len(fired_neurons) if fired_neurons else 0
            print(f"🔥 BURST ENGINE: Enhanced processing got {fired_count} firing neurons")
        
        # 3. During-burst injection (for modulator areas)
        if self.fcl_injection_service:
            if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                print(f"🔥 BURST ENGINE: Calling enhanced during-burst injection")
            injected_during = self.fcl_injection_service.inject_during_burst(current_timestep)
            if injected_during > 0:
                logger.debug(f"During-burst injection: {injected_during} neurons")
                if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                    print(f"🔥 BURST ENGINE: During-burst injected {injected_during} neurons")
        
        # 4. Post-burst injection (for cleanup or special processing)
        if self.fcl_injection_service:
            if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                print(f"🔥 BURST ENGINE: Calling enhanced post-burst injection")
            injected_post = self.fcl_injection_service.inject_post_burst(current_timestep)
            if injected_post > 0:
                logger.debug(f"Post-burst injection: {injected_post} neurons")
                if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                    print(f"🔥 BURST ENGINE: Post-burst injected {injected_post} neurons")
        
        # 5. Debug fire queue output if --debug-npu flag is enabled
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            self._debug_fire_queue_output()
        
        return fired_neurons

    def run(self) -> None:
        """
        Run the burst engine main loop.
        
        This function begins the burst execution loop, processing neuron firings
        based on the target burst frequency.
        
        RTOS-COMPATIBLE: This version uses basic loop control without signal handling.
        In RTOS environment, replace this with RTOS task control and event handling.
        """
        import traceback
        
        # Debug logging for run method entry
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            print(f"🔥 BURST ENGINE: run() method called for instance {self._instance_id}")
            print(f"🔥 BURST ENGINE: Current _running state: {self._running}")
            print(f"🔥 BURST ENGINE: Stack trace:")
            for line in traceback.format_stack()[-5:-1]:  # Show last 4 stack frames
                print(f"    {line.strip()}")
            print()
        
        self._running = True
        self.state_manager.set_burst_engine_state(ServiceState.READY)
        
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            print(f"🔥 BURST ENGINE: Set _running=True, entering main loop")
            print(f"🔥 BURST ENGINE: Target frequency: {self.desired_frequency}Hz, interval: {self.burst_interval}s")
        
        # RTOS-COMPATIBLE: Removed signal handling - not available in RTOS
        # In RTOS environment, use task control and events instead of signal handlers
        # def handle_signal(signum: int, frame: Any) -> None:
        #     logger.info(f"\nReceived signal {signum}, shutting down BurstEngine gracefully...")
        #     self.stop()
        # # Register signal handlers for graceful shutdown only in main thread
        # if threading.current_thread() is threading.main_thread():
        #     signal.signal(signal.SIGINT, handle_signal)
        #     signal.signal(signal.SIGTERM, handle_signal)
            
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            print(f"🔥 BURST ENGINE: About to enter main while loop")
            
        try:
            while self._running:
                cycle_start = time.perf_counter()
                
                # Debug logging if --debug-npu is enabled
                if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                    print(f"🔥 BURST ENGINE: Starting burst {self.burst_count + 1} in main loop")
                
                # Measure pure processing time
                processing_start = time.perf_counter()
                
                # Choose processing method based on power injection availability
                if self.fcl_injection_service:
                    # Enhanced processing with power injection - always use timestep 0 for current
                    if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                        print(f"🔥 BURST ENGINE: Using ENHANCED processing with power injection")
                    fired_neurons = self._process_burst_with_power_injection(0)  # Fixed: use 0 for current timestep
                else:
                    # Standard processing
                    if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                        print(f"🔥 BURST ENGINE: Using STANDARD processing (no injection service)")
                    fired_neurons = self._process_burst()
                
                # End of pure processing time measurement
                processing_end = time.perf_counter()
                processing_elapsed = processing_end - processing_start
                
                # Calculate potential frequency (pure processing speed without delays)
                potential_freq = 1.0 / processing_elapsed if processing_elapsed > 0 else 0
                
                # Record pure processing timing data if frequency measurement is enabled
                self._record_processing_timing(processing_elapsed)
                
                # 2. Load shedding if needed (based on potential frequency vs target)
                if potential_freq < self.desired_frequency:
                    for cortical_id in self.shed_areas:
                        # Clear FCL for this area for the current burst
                        if hasattr(self.fcl_manager, 'area_fcl_history'):
                            if cortical_id in self.fcl_manager.area_fcl_history:
                                self.fcl_manager.area_fcl_history[cortical_id][self.fcl_manager.current_window_index].clear()
                        elif hasattr(self.fcl_manager, 'cortical_fcl_history'):
                            if cortical_id in self.fcl_manager.cortical_fcl_history:
                                self.fcl_manager.cortical_fcl_history[cortical_id][self.fcl_manager.current_window_index].clear()
                
                # 3. Calculate full cycle time and actual frequency
                cycle_end = time.perf_counter()
                cycle_elapsed = cycle_end - cycle_start
                
                # 4. RTOS-COMPATIBLE: Replace time.sleep with timing wheel or task delay
                # In RTOS environment, replace time.sleep() with deterministic timing mechanism
                if cycle_elapsed < self.burst_interval:
                    # RTOS-COMPATIBLE: Replace time.sleep with busy-wait for deterministic timing
                    target_end_time = cycle_start + self.burst_interval
                    while time.perf_counter() < target_end_time:
                        pass  # Busy-wait - deterministic but CPU intensive
                
                # 5. Calculate actual frequency (including delays)
                final_cycle_time = time.perf_counter() - cycle_start
                actual_freq = 1.0 / final_cycle_time if final_cycle_time > 0 else 0
                
                # Update state manager with actual frequency (maintains compatibility)
                self.state_manager.set_burst_frequency(actual_freq)
                
                # Record full cycle timing data if frequency measurement is enabled
                self._record_burst_timing(final_cycle_time)
                
                # Debug timing information
                if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                    print(f"🔥 BURST ENGINE: Burst {self.burst_count + 1} - Processing: {processing_elapsed*1000:.2f}ms, "
                          f"Full cycle: {final_cycle_time*1000:.2f}ms, Potential: {potential_freq:.1f}Hz, Actual: {actual_freq:.1f}Hz")
                
                # Increment burst count
                self.burst_count += 1
                
        except Exception as e:
            # Handle crashes in the main loop by resetting _running flag
            if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                print(f"🔥 BURST ENGINE: EXCEPTION in main loop: {e}")
                print(f"🔥 BURST ENGINE: Stack trace:")
                traceback.print_exc()
            
            logger.error(f"BurstEngine main loop crashed: {e}")
            self._running = False  # Reset the running flag
            self.state_manager.set_burst_engine_state(ServiceState.ERROR)
            return
                
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            print(f"🔥 BURST ENGINE: Main loop exited normally, _running={self._running}")
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
        cycle_start = time.perf_counter()
        
        # Measure pure processing time
        processing_start = time.perf_counter()
        
        # Choose processing method based on power injection availability
        if self.fcl_injection_service:
            # Enhanced processing with power injection
            fired_neurons = self._process_burst_with_power_injection(self.burst_count)
        else:
            # Standard processing
            fired_neurons = self._process_burst()
        
        # End of pure processing time measurement
        processing_end = time.perf_counter()
        processing_elapsed = processing_end - processing_start
        
        # Calculate potential frequency (pure processing speed)
        potential_freq = 1.0 / processing_elapsed if processing_elapsed > 0 else 0
        
        # Record pure processing timing data if frequency measurement is enabled
        self._record_processing_timing(processing_elapsed)
        
        # Calculate full cycle time and actual frequency
        cycle_end = time.perf_counter()
        cycle_elapsed = cycle_end - cycle_start
        actual_freq = 1.0 / cycle_elapsed if cycle_elapsed > 0 else 0
        self.state_manager.set_burst_frequency(actual_freq)
        
        # Record full cycle timing data if frequency measurement is enabled
        self._record_burst_timing(cycle_elapsed)
        
        # Load shedding if needed (based on potential frequency)
        if potential_freq < self.desired_frequency:
            for cortical_id in self.shed_areas:
                # Clear FCL for this area for the current burst
                if hasattr(self.fcl_manager, 'area_fcl_history'):
                    if cortical_id in self.fcl_manager.area_fcl_history:
                        self.fcl_manager.area_fcl_history[cortical_id][self.fcl_manager.current_window_index].clear()
                elif hasattr(self.fcl_manager, 'cortical_fcl_history'):
                    if cortical_id in self.fcl_manager.cortical_fcl_history:
                        self.fcl_manager.cortical_fcl_history[cortical_id][self.fcl_manager.current_window_index].clear()
        
        # Increment burst count for testing
        self.burst_count += 1
        
        # Debug fire queue output if --debug-npu flag is enabled
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            self._debug_fire_queue_output()
        
        return fired_neurons

    def update_with_genome(self) -> None:
        """Called when a genome is loaded to update burst engine state"""
        print(f"🔥 BURST ENGINE: Instance {self._instance_id} updating with genome")
        
        self.genome_loaded = True
        # Update cortical areas list and shed areas set
        if hasattr(self.connectome_manager, 'cortical_areas') and self.connectome_manager.cortical_areas:
            self.cortical_areas = list(self.connectome_manager.cortical_areas.values())
            self.shed_areas = set(area.id for area in self.cortical_areas if area.properties.get('__shed', False))
            
            # CRITICAL: Re-initialize special area services with new genome
            print(f"🔥 BURST ENGINE: Instance {self._instance_id} re-initializing special area services for new genome")
            self._initialize_special_area_services()
            
        logger.info("Burst Engine updated with genome information", emoji1="⚡ ")

    def refresh_special_areas(self) -> None:
        """
        Refresh special area detection and injection batches.
        
        This should be called when cortical areas are added, removed, or modified.
        """
        if self.special_area_handler:
            self.special_area_handler.refresh_all_caches()
            
        if self.fcl_injection_service:
            self.fcl_injection_service.refresh_injection_batches()
            
        logger.info("Refreshed special area services", emoji1="🔄")

    def get_power_injection_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about power area injection.
        
        Returns:
            Dictionary with injection and special area statistics
        """
        stats = {}
        
        if self.special_area_handler:
            stats['special_areas'] = self.special_area_handler.get_statistics()
        
        if self.fcl_injection_service:
            stats['injection'] = self.fcl_injection_service.get_statistics()
            
        stats['power_injection_enabled'] = self.enable_power_injection
        stats['power_injection_timing'] = self.power_injection_timing
        
        return stats

    def set_power_injection_enabled(self, cortical_id: str, enabled: bool) -> bool:
        """
        Enable or disable power injection for a specific cortical area.
        
        Args:
            cortical_id: The cortical area ID
            enabled: Whether to enable or disable injection
            
        Returns:
            True if the setting was applied, False if area not found
        """
        if self.fcl_injection_service:
            return self.fcl_injection_service.set_injection_enabled(cortical_id, enabled)
        return False

    def run_with_fire_queue(self, mpf: bool = True, puf: bool = False, max_consecutive_fires: int = 10) -> bool:
        """
        Run the burst engine using the fire queue process with power injection support.
        
        This method uses the enhanced fire queue process with PSP calculation as 
        described in the architecture documentation. Now includes power area injection.
        
        Args:
            mpf: Membrane Potential Driven PSP Flag
            puf: PSP Uniformity Flag
            max_consecutive_fires: Maximum consecutive fire count before inhibiting firing
            
        Returns:
            True if completed successfully, False otherwise
        """
        import traceback
        
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            print(f"🔥 BURST ENGINE: run_with_fire_queue() called for instance {self._instance_id}")
            print(f"🔥 BURST ENGINE: Stack trace:")
            for line in traceback.format_stack()[-5:-1]:  # Show last 4 stack frames
                print(f"    {line.strip()}")
            print()
        
        if self.state_manager.get_burst_engine_state() != ServiceState.READY:
            if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                print(f"🔥 BURST ENGINE: run_with_fire_queue() - engine not ready, returning False")
            logger.warning("Burst engine is not ready, cannot start burst execution")
            return False
            
        # Update state - use READY state to indicate it's running (later could be changed to SYNCING or similar)
        self.state_manager.set_burst_engine_state(ServiceState.READY)
        logger.info("Burst engine starting with fire queue process", emoji1="🚀 ")
        
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            print(f"🔥 BURST ENGINE: run_with_fire_queue() - about to set _running = True")
        
        # Set running flag
        self._running = True
        
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            print(f"🔥 BURST ENGINE: run_with_fire_queue() - _running set to True, entering main loop")
        
        # Try to use optimized structures if available
        try:
            from feagi.npu.optimized_integration import step_simulation_with_fire_queue
            optimized_available = True
        except ImportError:
            optimized_available = False
        
        # Main loop with exception handling
        try:
            while self._running:
                cycle_start_time = time.perf_counter()
                
                if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                    print(f"🔥 BURST ENGINE: run_with_fire_queue() - main loop iteration, burst {self.burst_count}")
                
                # Measure pure processing time
                processing_start_time = time.perf_counter()
                
                # 1. Pre-burst power injection - use timestep 0 for current
                if self.fcl_injection_service:
                    injected_pre = self.fcl_injection_service.inject_pre_burst(0)  # Fixed: use 0 for current timestep
                    if injected_pre > 0:
                        logger.debug(f"Pre-burst injection: {injected_pre} neurons")
                
                # 2. Process bursts using fire queue
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
                
                # 3. During/post-burst injection - use timestep 0 for current
                if self.fcl_injection_service:
                    injected_during = self.fcl_injection_service.inject_during_burst(0)  # Fixed: use 0 for current timestep
                    injected_post = self.fcl_injection_service.inject_post_burst(0)  # Fixed: use 0 for current timestep
                    if injected_during > 0:
                        logger.debug(f"During-burst injection: {injected_during} neurons")
                    if injected_post > 0:
                        logger.debug(f"Post-burst injection: {injected_post} neurons")
                
                # End of pure processing time measurement
                processing_end_time = time.perf_counter()
                processing_elapsed = processing_end_time - processing_start_time
                
                # Calculate potential frequency (pure processing speed without delays)
                potential_freq = 1.0 / processing_elapsed if processing_elapsed > 0 else 0
                
                # Record pure processing timing data if frequency measurement is enabled
                self._record_processing_timing(processing_elapsed)
                    
                # Calculate time taken for this burst cycle so far
                cycle_elapsed = time.perf_counter() - cycle_start_time
                self.last_burst_time = cycle_elapsed
                
                # Sleep if needed to maintain target frequency
                if self.desired_frequency > 0 and cycle_elapsed < self.burst_interval:
                    # RTOS-COMPATIBLE: Replace time.sleep with busy-wait for deterministic timing
                    target_end_time = cycle_start_time + self.burst_interval
                    while time.perf_counter() < target_end_time:
                        pass  # Busy-wait - deterministic but CPU intensive
                
                # Calculate actual frequency (including delays)
                final_cycle_time = time.perf_counter() - cycle_start_time
                actual_freq = 1.0 / final_cycle_time if final_cycle_time > 0 else 0
                self.state_manager.set_burst_frequency(actual_freq)
                
                # Record timing data if frequency measurement is enabled
                self._record_burst_timing(final_cycle_time)
                
                # Log performance every 100 bursts
                if self.burst_count % 100 == 0:
                    logger.info(f"Processed {self.burst_count} bursts. "
                               f"Target: {self.desired_frequency:.1f}Hz, "
                               f"Actual: {actual_freq:.1f}Hz, "
                               f"Potential: {potential_freq:.1f}Hz",
                               emoji1="⚡ ")
                
                # Increment burst count
                self.burst_count += 1
                
        except Exception as e:
            # Handle crashes in the fire queue main loop
            if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                print(f"🔥 BURST ENGINE: EXCEPTION in run_with_fire_queue main loop: {e}")
                print(f"🔥 BURST ENGINE: Stack trace:")
                traceback.print_exc()
            
            logger.error(f"BurstEngine fire queue main loop crashed: {e}")
            self._running = False  # Reset the running flag
            self.state_manager.set_burst_engine_state(ServiceState.ERROR)
            return False
        
        # Update state when stopped
        self.state_manager.set_burst_engine_state(ServiceState.READY)
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            print(f"🔥 BURST ENGINE: run_with_fire_queue() - exiting normally, loop finished")
        logger.info("Burst engine stopped", emoji1="🛑 ")
        return True

    def _debug_fire_queue_output(self) -> None:
        """
        Debug fire queue output for NPU debugging.
        
        When --debug-npu flag is enabled, this method displays detailed information
        about the current fire queue contents including:
        - Global FCL summary
        - Per-cortical area breakdown
        - Neuron firing statistics
        """
        try:
            print(f"\n🔥 ===== NPU DEBUG - BURST {self.burst_count} =====")
            
            # Get global FCL
            global_fcl = self.fcl_manager.get_global_fcl()
            total_firing = len(global_fcl)
            
            print(f"📊 Global Fire Summary:")
            print(f"   Total firing neurons: {total_firing}")
            print(f"   Burst frequency: {1.0/self.burst_interval:.1f}Hz target")
            
            if total_firing > 0:
                # Get firing neurons by cortical area
                fcl_by_cortical = self.fcl_manager.get_fcl_by_cortical()
                
                print(f"🧠 Per-Area Breakdown ({len(fcl_by_cortical)} active areas):")
                
                # Sort areas by number of firing neurons for consistent output
                sorted_areas = sorted(fcl_by_cortical.items(), key=lambda x: len(x[1]), reverse=True)
                
                for cortical_id, area_fcl in sorted_areas:
                    area_count = len(area_fcl)
                    percentage = (area_count / total_firing) * 100 if total_firing > 0 else 0
                    
                    # Display first few neurons for small lists, summarize for large ones
                    if area_count <= 10:
                        neuron_list = sorted(list(area_fcl))
                        print(f"   {cortical_id}: {area_count} neurons ({percentage:.1f}%) - {neuron_list}")
                    else:
                        neuron_sample = sorted(list(area_fcl))[:5]
                        print(f"   {cortical_id}: {area_count} neurons ({percentage:.1f}%) - {neuron_sample}... (+{area_count-5} more)")
                
                # Show power area injection info if available
                if self.fcl_injection_service:
                    stats = self.get_power_injection_statistics()
                    if 'injection' in stats and stats['injection'].get('total_injections', 0) > 0:
                        power_neurons = stats['special_areas'].get('total_power_neurons', 0)
                        print(f"⚡ Power Injection: {power_neurons} neurons from {stats['special_areas'].get('power_areas_count', 0)} power areas")
            else:
                print("   No neurons firing this burst")
                
            # Show recent firing statistics if available
            if hasattr(self.fcl_manager, 'get_firing_statistics'):
                firing_stats = self.fcl_manager.get_firing_statistics()
                if firing_stats:
                    print(f"📈 Recent Activity:")
                    print(f"   Average firing rate: {firing_stats.get('average_firing_rate', 0):.1f} neurons/burst")
                    print(f"   Peak firing: {firing_stats.get('peak_firing', 0)} neurons")
            
            print(f"🔥 ========================================\n")
            
        except Exception as e:
            print(f"🔥 NPU DEBUG ERROR: Failed to display fire queue - {e}")
            logger.error(f"NPU debug output error: {e}")  

    def measure_actual_frequency(self, duration_seconds: float = 5.0, sample_count: int = 100) -> dict:
        """
        Measure both actual and potential burst frequencies over a specified period.
        
        This is an expensive operation that should only be called on-demand for monitoring
        or debugging purposes. It collects detailed timing data during burst processing.
        
        Args:
            duration_seconds: How long to collect timing data (default 5 seconds)
            sample_count: Number of burst samples to collect (default 100)
            
        Returns:
            Dictionary with measurement results including both actual and potential frequencies
        """
        import time
        import statistics
        
        if not self._running:
            raise RuntimeError("Cannot measure frequency - burst engine is not running")
        
        # Only log detailed frequency measurement start when debugging NPU
        if os.environ.get('FEAGI_DEBUG_NPU') == '1':
            logger.info(f"Starting frequency measurement for {duration_seconds}s", emoji1="🔬")
        
        # Enable frequency measurement mode
        old_measurement_enabled = getattr(self, '_frequency_measurement_enabled', False)
        self._frequency_measurement_enabled = True
        
        # Clear any existing timing buffers
        self._burst_timing_buffer.clear()
        self._processing_timing_buffer.clear()
        
        measurement_start = time.perf_counter()
        measurement_end = measurement_start + duration_seconds
        burst_count_start = self.burst_count
        
        try:
            # Wait for measurements to be collected
            # The timing data will be collected automatically in the main burst loop
            while (time.perf_counter() < measurement_end and 
                   len(self._burst_timing_buffer) < sample_count):
                # RTOS-COMPATIBLE: Replace time.sleep with CPU-friendly yield
                # In RTOS environment, use task yield or timer wait
                import time
                last_check = time.perf_counter()
                while time.perf_counter() - last_check < 0.01:
                    pass  # Busy-wait for 10ms equivalent
                
                # Safety check - ensure burst engine is still running
                if not self._running:
                    raise RuntimeError("Burst engine stopped during measurement")
            
            measurement_actual_duration = time.perf_counter() - measurement_start
            burst_count_end = self.burst_count
            total_bursts_measured = burst_count_end - burst_count_start
            
            # Calculate frequency metrics from collected timing data
            if not self._burst_timing_buffer or not self._processing_timing_buffer:
                raise RuntimeError("No timing data collected during measurement period")
            
            # Full cycle timing statistics (includes delays) - for actual frequency
            full_cycle_data_ms = [t * 1000 for t in self._burst_timing_buffer]
            min_cycle_time_ms = min(full_cycle_data_ms)
            max_cycle_time_ms = max(full_cycle_data_ms)
            avg_cycle_time_ms = statistics.mean(full_cycle_data_ms)
            cycle_std_dev_ms = statistics.stdev(full_cycle_data_ms) if len(full_cycle_data_ms) > 1 else 0.0
            
            # Processing timing statistics (pure processing) - for potential frequency
            processing_data_ms = [t * 1000 for t in self._processing_timing_buffer]
            min_processing_time_ms = min(processing_data_ms)
            max_processing_time_ms = max(processing_data_ms)
            avg_processing_time_ms = statistics.mean(processing_data_ms)
            processing_std_dev_ms = statistics.stdev(processing_data_ms) if len(processing_data_ms) > 1 else 0.0
            
            # Frequency calculations
            avg_cycle_time_seconds = avg_cycle_time_ms / 1000.0
            avg_processing_time_seconds = avg_processing_time_ms / 1000.0
            
            actual_frequency_hz = 1.0 / avg_cycle_time_seconds if avg_cycle_time_seconds > 0 else 0.0
            potential_frequency_hz = 1.0 / avg_processing_time_seconds if avg_processing_time_seconds > 0 else 0.0
            
            # Alternative frequency calculation based on total time
            cycles_per_second = total_bursts_measured / measurement_actual_duration if measurement_actual_duration > 0 else 0.0
            
            measurement_result = {
                "actual_frequency_hz": actual_frequency_hz,
                "potential_frequency_hz": potential_frequency_hz,
                "alternative_frequency_hz": cycles_per_second,  # Alternative calculation method
                "target_frequency_hz": self.desired_frequency,
                "measurement_duration_s": measurement_actual_duration,
                "sample_count": len(self._burst_timing_buffer),
                "total_bursts_measured": total_bursts_measured,
                
                # Full cycle timing stats (actual frequency)
                "min_cycle_time_ms": min_cycle_time_ms,
                "max_cycle_time_ms": max_cycle_time_ms,
                "avg_cycle_time_ms": avg_cycle_time_ms,
                "cycle_std_dev_ms": cycle_std_dev_ms,
                
                # Processing timing stats (potential frequency)
                "min_processing_time_ms": min_processing_time_ms,
                "max_processing_time_ms": max_processing_time_ms,
                "avg_processing_time_ms": avg_processing_time_ms,
                "processing_std_dev_ms": processing_std_dev_ms,
                
                # Performance analysis
                "actual_performance_ratio": actual_frequency_hz / self.desired_frequency if self.desired_frequency > 0 else 0.0,
                "potential_performance_ratio": potential_frequency_hz / self.desired_frequency if self.desired_frequency > 0 else 0.0,
                "efficiency_ratio": actual_frequency_hz / potential_frequency_hz if potential_frequency_hz > 0 else 0.0,
                "headroom_hz": potential_frequency_hz - self.desired_frequency,
                
                # Debug data
                "cycle_timing_data_ms": full_cycle_data_ms[-20:],  # Include last 20 samples for debugging
                "processing_timing_data_ms": processing_data_ms[-20:]  # Include last 20 samples for debugging
            }
            
            # Only log detailed completion when debugging NPU
            if os.environ.get('FEAGI_DEBUG_NPU') == '1':
                logger.info(f"Frequency measurement complete - Actual: {actual_frequency_hz:.1f}Hz, Potential: {potential_frequency_hz:.1f}Hz (target: {self.desired_frequency:.1f}Hz)", emoji1="📊")
            
            return measurement_result
            
        finally:
            # Restore previous measurement state
            self._frequency_measurement_enabled = old_measurement_enabled
            
            # Clear timing buffers to free memory
            if not self._frequency_measurement_enabled:
                self._burst_timing_buffer.clear()
                self._processing_timing_buffer.clear()

    def _record_burst_timing(self, burst_duration_seconds: float) -> None:
        """
        Record burst timing data if frequency measurement is enabled.
        
        Args:
            burst_duration_seconds: Duration of the burst in seconds
        """
        if not getattr(self, '_frequency_measurement_enabled', False):
            return
            
        # Add to circular buffer
        self._burst_timing_buffer.append(burst_duration_seconds)
        
        # Maintain buffer size
        if len(self._burst_timing_buffer) > self._timing_buffer_size:
            self._burst_timing_buffer.pop(0)  # Remove oldest entry

    def _record_processing_timing(self, processing_duration_seconds: float) -> None:
        """
        Record processing timing data if frequency measurement is enabled.
        
        Args:
            processing_duration_seconds: Duration of the processing in seconds
        """
        if not getattr(self, '_frequency_measurement_enabled', False):
            return
            
        # Add to circular buffer
        self._processing_timing_buffer.append(processing_duration_seconds)
        
        # Maintain buffer size
        if len(self._processing_timing_buffer) > self._timing_buffer_size:
            self._processing_timing_buffer.pop(0)  # Remove oldest entry

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
                # RTOS-COMPATIBLE: Replace time.sleep with deterministic timing
                target_time = start + self.sample_interval
                while time.perf_counter() < target_time:
                    pass  # Busy-wait for sample interval
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
                                # RTOS-COMPATIBLE: Replace time.sleep with deterministic delay
                                delay_start = time.perf_counter()
                                while time.perf_counter() - delay_start < self._retry_delay:
                                    pass  # Busy-wait for retry delay
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
                            # RTOS-COMPATIBLE: Replace time.sleep with deterministic delay
                            delay_start = time.perf_counter()
                            while time.perf_counter() - delay_start < self._retry_delay:
                                pass  # Busy-wait for retry delay
                            retry_count += 1
                    except Exception as e:
                        # Log error but continue
                        if retry_count == self._max_retries - 1:  # Only log on last retry
                            logger.error(f"FCLSampler error: {e}")
                        # Wait before retrying
                        # RTOS-COMPATIBLE: Replace time.sleep with deterministic delay
                        delay_start = time.perf_counter()
                        while time.perf_counter() - delay_start < self._retry_delay:
                            pass  # Busy-wait for retry delay
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
                # RTOS-COMPATIBLE: Replace time.sleep with deterministic timing
                target_end_time = start + self.sample_interval
                while time.perf_counter() < target_end_time:
                    pass  # Busy-wait for remainder of sample interval
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
                # RTOS-COMPATIBLE: Replace time.sleep with deterministic timing
                target_time = start + self.sample_interval
                while time.perf_counter() < target_time:
                    pass  # Busy-wait for sample interval
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
                # RTOS-COMPATIBLE: Replace time.sleep with deterministic timing
                target_end_time = start + self.sample_interval
                while time.perf_counter() < target_end_time:
                    pass  # Busy-wait for remainder of sample interval
                
        logger.info("FQSampler stopped.")

    def _sample_area_fire_queue(self, cortical_id: str) -> None:
        """Sample fire queue data for a specific cortical area."""
        retry_count = 0
        while retry_count < self._max_retries:
            try:
                # Get fire queue data for this area
                area_fire_data = self._get_area_fire_queue_data(cortical_id)
                
                if area_fire_data:
                    try:
                        self.output_queue.put((cortical_id, area_fire_data))
                        break  # Success
                    except Exception as e:
                        logger.error(f"Error putting area data in queue: {e}")
                else:
                    break
                    
            except Exception as e:
                if retry_count == self._max_retries - 1:
                    logger.error(f"FQSampler error (area {cortical_id}): {e}")
                # Wait before retrying
                # RTOS-COMPATIBLE: Replace time.sleep with deterministic delay
                delay_start = time.perf_counter()
                while time.perf_counter() - delay_start < self._retry_delay:
                    pass  # Busy-wait for retry delay
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
                    except Exception as e:
                        # Queue full - drop data
                        logger.warning(f"FQSampler queue full, dropping global data: {e}")
                        break
                else:
                    break
                    
            except Exception as e:
                if retry_count == self._max_retries - 1:
                    logger.error(f"FQSampler error: {e}")
                # Wait before retrying
                # RTOS-COMPATIBLE: Replace time.sleep with deterministic delay
                delay_start = time.perf_counter()
                while time.perf_counter() - delay_start < self._retry_delay:
                    pass  # Busy-wait for retry delay
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