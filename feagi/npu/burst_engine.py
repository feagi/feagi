"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import time
import logging
# RTOS-COMPATIBLE: Removed signal and threading imports - not available in RTOS
# import signal  # REMOVED: Not compatible with RTOS
# import threading  # REMOVED: Not compatible with RTOS - use RTOS task primitives instead
# WGPU-COMPATIBLE: Remove os import to eliminate environment variable dependencies
# import os  # REMOVED: Environment variables not available in WGPU contexts
from typing import Dict, List, Optional, Set, Any, Union
from feagi.core.state_manager import FeagiStateManager, ServiceState
from feagi.utils.logger import setup_logger

# New imports for power area injection
from feagi.npu.special_area_handler import SpecialAreaHandler
from feagi.npu.fcl_injection_service import FCLInjectionService

# SIMD and performance imports
try:
    from ..utils.simd_detection import get_simd_detector, get_backend_selector, get_simd_config
    from ..utils.simd_profiler import get_profiler, profile_simd_operation
    from .optimized_membrane_operations import SIMDMembraneProcessor
    SIMD_AVAILABLE = True
except ImportError:
    SIMD_AVAILABLE = False

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
            # WGPU-COMPATIBLE: Use logger instead of print for debug output
            logger.info(f"🔥 BURST ENGINE: Creating NEW singleton instance {cls._instance_id}")
        else:
            # WGPU-COMPATIBLE: Use logger instead of print for debug output
            logger.info(f"🔥 BURST ENGINE: Returning EXISTING singleton instance {cls._instance_id}")
        return cls._instance
    
    @property
    def _running(self):
        """Get the running state with debug tracking."""
        return getattr(self, '_running_state', False)
    
    @_running.setter
    def _running(self, value):
        """Setter for _running with debug logging."""
        old_value = getattr(self, '_running_state', None)
        self._running_state = value
        
        # WGPU-COMPATIBLE: Check debug_npu config instead of environment variable
        if hasattr(self, 'debug_npu') and self.debug_npu and old_value != value:
            # WGPU-COMPATIBLE: Use logger instead of print for debug output
            logger.debug(f"🔥 BURST ENGINE: Instance {self._instance_id} _running changed: {old_value} → {value}")
            import traceback
            logger.debug(f"🔥 BURST ENGINE: Stack trace:")
            for line in traceback.format_stack():
                logger.debug(f"    {line.strip()}")

    def __init__(self, connectome_manager: Any, fcl_manager: Optional[Any] = None, config: Optional[Dict[str, Any]] = None) -> None:
        """
        Initialize the Burst Engine.
        
        Args:
            connectome_manager: The connectome manager
            fcl_manager: FCL manager (optional)
            config: Configuration parameters (optional)
                   - debug_npu: Enable debug logging (replaces FEAGI_DEBUG_NPU env var)
                   - enable_power_injection: Enable power area injection
                   - power_injection_timing: When to inject power neurons
                   - desired_frequency_hz: Target frequency in Hz
        """
        # Prevent re-initialization if already initialized
        if hasattr(self, '_initialized') and self._initialized:
            # WGPU-COMPATIBLE: Use logger instead of print for debug output
            logger.info(f"🔥 BURST ENGINE: Instance {self._instance_id} already initialized, skipping")
            return
            
        # WGPU-COMPATIBLE: Use logger instead of print for debug output
        logger.info(f"🔥 BURST ENGINE: Initializing singleton instance {self._instance_id}")
        
        # Initialize logger for this instance
        self.logger = logging.getLogger(__name__ + f".BurstEngine.{self._instance_id}")
        
        self.connectome_manager = connectome_manager
        self.fcl_manager = fcl_manager or connectome_manager.fcl_manager
        self.config = config or {}
        
        # WGPU-COMPATIBLE: Check debug_npu from config only (no environment variables)
        self.debug_npu = self.config.get('debug_npu', False)
        
        # Log debug NPU status when enabled
        if self.debug_npu:
            logger.info("🔥 NPU debug mode enabled - will show detailed fire queue contents during bursts")
        
        self.genome_loaded = False
        self._running = False  # This will now trigger the setter with debug logging
        self.burst_count = 0
        self.last_burst_time = 0.0
        
        # Initialize special area handler and injection service
        self.special_area_handler: Optional[SpecialAreaHandler] = None
        self.fcl_injection_service: Optional[FCLInjectionService] = None
        
        # FQ Sampler registry for debugging motor and visualization streams
        self._fq_samplers: List[Any] = []  # List of registered FQ samplers
        
        # Power area injection configuration
        self.enable_power_injection = self.config.get('enable_power_injection', True)
        self.power_injection_timing = self.config.get('power_injection_timing', 'pre_burst')
        
        # Initialize in a valid but inactive state
        # Will become fully operational when a genome is loaded
        logger.info("Burst Engine initialized in standby mode", emoji1="⚡️")
        
        self.state_manager = FeagiStateManager.instance()
        
        # Support both parameter names for backward compatibility
        self.desired_frequency = self.config.get('desired_frequency_hz', 
                                              self.config.get('target_frequency', 1.0))
        self.target_frequency = self.desired_frequency  # For backward compatibility
        self.burst_interval = 1.0 / self.desired_frequency
        
        # Use cortical_areas instead of _areas - fix the attribute name
        self.cortical_areas = list(self.connectome_manager.cortical_areas_by_id.values()) if hasattr(self.connectome_manager, 'cortical_areas_by_id') else []
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
        
        # Initialize SIMD support
        self._init_simd_support()
        
        # Core processing components
        self.scheduler = None
        self.fire_queue_provider = None
        
        # Performance monitoring
        self.total_neurons_processed = 0
        self.last_performance_report = time.time()
        
        # SIMD-optimized membrane processor
        if SIMD_AVAILABLE:
            self.membrane_processor = None  # Initialized when capacity is known
        
        # Runtime configuration
        self.use_simd_profiling = self.config.get('simd_profiling', False)
        self.performance_monitoring = self.config.get('performance_monitoring', True)
        
        # Mark as initialized
        self._initialized = True
        # WGPU-COMPATIBLE: Use logger instead of print for debug output
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
        # WGPU-COMPATIBLE: Use logger instead of print for debug output
        if self.debug_npu:
            logger.debug(f"🔥 BURST ENGINE _process_burst called! Instance {self._instance_id}, Burst count: {self.burst_count}")
            
            # Check injection service availability
            if self.fcl_injection_service:
                logger.debug(f"🔥 BURST ENGINE: Injection service AVAILABLE")
            else:
                logger.debug(f"🔥 BURST ENGINE: NO INJECTION SERVICE!")
        
        # 1. Pre-burst power injection
        if self.fcl_injection_service and self.power_injection_timing == 'pre_burst':
            if self.debug_npu:
                logger.debug(f"🔥 BURST ENGINE: Calling pre-burst injection")
            self.fcl_injection_service.inject_pre_burst(self.burst_count)
        
        # 2. Update membrane potentials and get fired neurons
        if self.debug_npu:
            # FCL manager uses sliding window with current timestep always 0
            current_timestep = 0  # Fixed: always use 0 for current timestep
            logger.debug(f"🔥 BURST ENGINE: About to call update_membrane_potentials with timestep {current_timestep}")
        
        fired_neurons = self.connectome_manager.update_membrane_potentials()
        
        if self.debug_npu:
            fired_count = len(fired_neurons) if fired_neurons else 0
            logger.debug(f"🔥 BURST ENGINE: Got {fired_count} firing neurons")
        
        # 3. During-burst injection (for modulators)
        if self.fcl_injection_service and self.power_injection_timing == 'during_burst':
            if self.debug_npu:
                logger.debug(f"🔥 BURST ENGINE: Calling during-burst injection")
            self.fcl_injection_service.inject_during_burst(self.burst_count)
        
        # 4. Post-burst injection  
        if self.fcl_injection_service and self.power_injection_timing == 'post_burst':
            if self.debug_npu:
                logger.debug(f"🔥 BURST ENGINE: Calling post-burst injection")
            self.fcl_injection_service.inject_post_burst(self.burst_count)
        
        # 5. Debug fire queue output if --debug-npu flag is enabled
        if self.debug_npu:
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
        if self.debug_npu:
            logger.debug(f"🔥 BURST ENGINE _process_burst_with_power_injection called! Instance {self._instance_id}, Timestep: {current_timestep}")
            
            # Check injection service availability
            if self.fcl_injection_service:
                logger.debug(f"🔥 BURST ENGINE: Enhanced injection service AVAILABLE")
            else:
                logger.debug(f"🔥 BURST ENGINE: NO ENHANCED INJECTION SERVICE!")
        
        # 1. Pre-burst power injection (inject power area neurons)
        if self.fcl_injection_service:
            if self.debug_npu:
                logger.debug(f"🔥 BURST ENGINE: Calling enhanced pre-burst injection")
            injected_pre = self.fcl_injection_service.inject_pre_burst(current_timestep)
            if injected_pre > 0:
                logger.debug(f"Pre-burst injection: {injected_pre} neurons")
                if self.debug_npu:
                    logger.debug(f"🔥 BURST ENGINE: Pre-burst injected {injected_pre} neurons")
        
        # 2. Standard burst processing (membrane potential updates, regular firing)
        if self.debug_npu:
            logger.debug(f"🔥 BURST ENGINE: About to call enhanced update_membrane_potentials with timestep {current_timestep}")
        
        fired_neurons = self.connectome_manager.update_membrane_potentials()
        
        if self.debug_npu:
            fired_count = len(fired_neurons) if fired_neurons else 0
            logger.debug(f"🔥 BURST ENGINE: Enhanced processing got {fired_count} firing neurons")
        
        # 3. During-burst injection (for modulator areas)
        if self.fcl_injection_service:
            if self.debug_npu:
                logger.debug(f"🔥 BURST ENGINE: Calling enhanced during-burst injection")
            injected_during = self.fcl_injection_service.inject_during_burst(current_timestep)
            if injected_during > 0:
                logger.debug(f"During-burst injection: {injected_during} neurons")
                if self.debug_npu:
                    logger.debug(f"🔥 BURST ENGINE: During-burst injected {injected_during} neurons")
        
        # 4. Post-burst injection (for cleanup or special processing)
        if self.fcl_injection_service:
            if self.debug_npu:
                logger.debug(f"🔥 BURST ENGINE: Calling enhanced post-burst injection")
            injected_post = self.fcl_injection_service.inject_post_burst(current_timestep)
            if injected_post > 0:
                logger.debug(f"Post-burst injection: {injected_post} neurons")
                if self.debug_npu:
                    logger.debug(f"🔥 BURST ENGINE: Post-burst injected {injected_post} neurons")
        
        # 5. Debug fire queue output if --debug-npu flag is enabled
        if self.debug_npu:
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
        
        # ALWAYS log key run method events for debugging
        logger.info(f"🔥 BURST ENGINE: run() method called for instance {self._instance_id}")
        logger.info(f"🔥 BURST ENGINE: Target frequency: {self.desired_frequency}Hz, interval: {self.burst_interval}s")
        
        # Debug logging for run method entry
        if self.debug_npu:
            logger.debug(f"🔥 BURST ENGINE: Current _running state: {self._running}")
            logger.debug(f"🔥 BURST ENGINE: Stack trace:")
            for line in traceback.format_stack()[-5:-1]:  # Show last 4 stack frames
                logger.debug(f"    {line.strip()}")
            logger.debug("")
        
        self._running = True
        self.state_manager.set_burst_engine_state(ServiceState.READY)
        logger.info(f"🔥 BURST ENGINE: Set _running=True, about to enter main loop")
        
        # RTOS-COMPATIBLE: Removed signal handling - not available in RTOS
        # In RTOS environment, use task control and events instead of signal handlers
            
        try:
            logger.info(f"🔥 BURST ENGINE: Entering main while loop")
            loop_count = 0
            while self._running:
                loop_count += 1
                cycle_start = time.perf_counter()
                
                # Log first few loops and every 10th loop for debugging
                if loop_count <= 3 or loop_count % 10 == 0:
                    logger.info(f"🔥 BURST ENGINE: Starting burst {self.burst_count + 1} (loop {loop_count})")
                
                # Debug logging if --debug-npu is enabled
                if self.debug_npu:
                    logger.debug(f"🔥 BURST ENGINE: Starting burst {self.burst_count + 1} in main loop")
                
                # Measure pure processing time
                processing_start = time.perf_counter()
                
                # Choose processing method based on power injection availability
                if self.fcl_injection_service:
                    # Enhanced processing with power injection - always use timestep 0 for current
                    if self.debug_npu:
                        logger.debug(f"🔥 BURST ENGINE: Using ENHANCED processing with power injection")
                    fired_neurons = self._process_burst_with_power_injection(0)  # Fixed: use 0 for current timestep
                else:
                    # Standard processing
                    if self.debug_npu:
                        logger.debug(f"🔥 BURST ENGINE: Using STANDARD processing (no injection service)")
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
                
                # Debug timing information - log first few and every 10th
                if loop_count <= 3 or loop_count % 10 == 0:
                    logger.info(f"🔥 BURST ENGINE: Burst {self.burst_count + 1} - Processing: {processing_elapsed*1000:.2f}ms, "
                                  f"Full cycle: {final_cycle_time*1000:.2f}ms, Target: {self.desired_frequency:.1f}Hz, Actual: {actual_freq:.1f}Hz")
                elif self.debug_npu:
                    logger.debug(f"🔥 BURST ENGINE: Burst {self.burst_count + 1} - Processing: {processing_elapsed*1000:.2f}ms, "
                                  f"Full cycle: {final_cycle_time*1000:.2f}ms, Potential: {potential_freq:.1f}Hz, Actual: {actual_freq:.1f}Hz")
                
                # Increment burst count
                self.burst_count += 1
                
        except Exception as e:
            # Handle crashes in the main loop by resetting _running flag
            logger.error(f"🔥 BURST ENGINE: EXCEPTION in main loop: {e}")
            logger.error(f"🔥 BURST ENGINE: Exception traceback:")
            logger.error(traceback.format_exc())
            
            logger.error(f"BurstEngine main loop crashed: {e}")
            self._running = False  # Reset the running flag
            self.state_manager.set_burst_engine_state(ServiceState.ERROR)
            return
                
        logger.info(f"🔥 BURST ENGINE: Main loop exited normally, _running={self._running}")
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
        if self.debug_npu:
            self._debug_fire_queue_output()
        
        return fired_neurons

    def update_with_genome(self) -> None:
        """
        Update burst engine with new genome configuration.
        
        This method should be called whenever the genome is loaded or reloaded.
        It reinitializes the special area handler and injection service for the new genome.
        """
        # WGPU-COMPATIBLE: Use config-based debug instead of environment variable
        if self.debug_npu:
            logger.debug(f"🔥 BURST ENGINE: Instance {self._instance_id} updating with genome")
        
        self.genome_loaded = True
        self.burst_count = 0  # Reset burst count for new genome
        
        # Use cortical_areas instead of _areas - fix the attribute name
        self.cortical_areas = list(self.connectome_manager.cortical_areas_by_id.values()) if hasattr(self.connectome_manager, 'cortical_areas_by_id') else []
        self.shed_areas = set(area.id for area in self.cortical_areas if area.properties.get('__shed', False))
        
        # WGPU-COMPATIBLE: Use config-based debug instead of environment variable
        if self.debug_npu:
            logger.debug(f"🔥 BURST ENGINE: Instance {self._instance_id} re-initializing special area services for new genome")
        
        # Reinitialize special area services for the new genome
        self._initialize_special_area_services()

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
        
        if self.debug_npu:
            logger.debug(f"🔥 BURST ENGINE: run_with_fire_queue() called for instance {self._instance_id}")
            logger.debug(f"🔥 BURST ENGINE: Stack trace:")
            for line in traceback.format_stack()[-5:-1]:  # Show last 4 stack frames
                logger.debug(f"    {line.strip()}")
            logger.debug("")
        
        if self.state_manager.get_burst_engine_state() != ServiceState.READY:
            if self.debug_npu:
                logger.warning(f"🔥 BURST ENGINE: run_with_fire_queue() - engine not ready, returning False")
            logger.warning("Burst engine is not ready, cannot start burst execution")
            return False
            
        # Update state - use READY state to indicate it's running (later could be changed to SYNCING or similar)
        self.state_manager.set_burst_engine_state(ServiceState.READY)
        logger.info("Burst engine starting with fire queue process", emoji1="🚀 ")
        
        if self.debug_npu:
            logger.debug(f"🔥 BURST ENGINE: run_with_fire_queue() - about to set _running = True")
        
        # Set running flag
        self._running = True
        
        if self.debug_npu:
            logger.debug(f"🔥 BURST ENGINE: run_with_fire_queue() - _running set to True, entering main loop")
        
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
                
                if self.debug_npu:
                    logger.debug(f"🔥 BURST ENGINE: run_with_fire_queue() - main loop iteration, burst {self.burst_count}")
                
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
            if self.debug_npu:
                logger.error(f"🔥 BURST ENGINE: EXCEPTION in run_with_fire_queue main loop: {e}")
                logger.error(f"🔥 BURST ENGINE: Stack trace:")
                traceback.print_exc()
            
            logger.error(f"BurstEngine fire queue main loop crashed: {e}")
            self._running = False  # Reset the running flag
            self.state_manager.set_burst_engine_state(ServiceState.ERROR)
            return False
        
        # Update state when stopped
        self.state_manager.set_burst_engine_state(ServiceState.READY)
        if self.debug_npu:
            logger.debug(f"🔥 BURST ENGINE: run_with_fire_queue() - exiting normally, loop finished")
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
        - FQ Sampler status and data (for motor/visualization debugging)
        - FCL Sampler status and data (legacy)
        """
        try:
            logger.info(f"\n🔥 ===== NPU DEBUG - BURST {self.burst_count} =====")
            
            # Get global FCL
            global_fcl = self.fcl_manager.get_global_fcl()
            total_firing = len(global_fcl)
            
            logger.info(f"📊 Global Fire Summary:")
            logger.info(f"   Total firing neurons: {total_firing}")
            logger.info(f"   Burst frequency: {1.0/self.burst_interval:.1f}Hz target")
            
            if total_firing > 0:
                # Get firing neurons by cortical area
                fcl_by_cortical = self.fcl_manager.get_fcl_by_cortical()
                
                logger.info(f"🧠 Per-Area Breakdown ({len(fcl_by_cortical)} active areas):")
                
                # Sort areas by number of firing neurons for consistent output
                sorted_areas = sorted(fcl_by_cortical.items(), key=lambda x: len(x[1]), reverse=True)
                
                for cortical_id, area_fcl in sorted_areas:
                    area_count = len(area_fcl)
                    percentage = (area_count / total_firing) * 100 if total_firing > 0 else 0
                    
                    # Display first few neurons for small lists, summarize for large ones
                    if area_count <= 10:
                        neuron_list = sorted(list(area_fcl))
                        logger.info(f"   {cortical_id}: {area_count} neurons ({percentage:.1f}%) - {neuron_list}")
                    else:
                        neuron_sample = sorted(list(area_fcl))[:5]
                        logger.info(f"   {cortical_id}: {area_count} neurons ({percentage:.1f}%) - {neuron_sample}... (+{area_count-5} more)")
                
                # Show power area injection info if available
                if self.fcl_injection_service:
                    stats = self.get_power_injection_statistics()
                    if 'injection' in stats and stats['injection'].get('total_injections', 0) > 0:
                        power_neurons = stats['special_areas'].get('total_power_neurons', 0)
                        logger.info(f"⚡ Power Injection: {power_neurons} neurons from {stats['special_areas'].get('power_areas_count', 0)} power areas")
            else:
                logger.info("   No neurons firing this burst")
                
            # Show recent firing statistics if available
            if hasattr(self.fcl_manager, 'get_firing_statistics'):
                firing_stats = self.fcl_manager.get_firing_statistics()
                if firing_stats:
                    logger.info(f"📈 Recent Activity:")
                    logger.info(f"   Average firing rate: {firing_stats.get('average_firing_rate', 0):.1f} neurons/burst")
                    logger.info(f"   Peak firing: {firing_stats.get('peak_firing', 0)} neurons")
            
            # === NEW: FQ SAMPLER DEBUG INFORMATION ===
            logger.info(f"🎯 Sampler Debug Information:")
            
            # FQ Samplers (for motor and visualization)
            if self._fq_samplers:
                logger.info(f"   📺 FQ Samplers Active: {len(self._fq_samplers)}")
                for i, fq_sampler in enumerate(self._fq_samplers):
                    try:
                        sampler_name = f"FQSampler-{i+1}"
                        running_status = "RUNNING" if getattr(fq_sampler, 'running', False) else "STOPPED"
                        sample_freq = getattr(fq_sampler, 'sample_frequency', 0)
                        
                        # Get subscriber status
                        viz_subs = getattr(fq_sampler, '_has_visualization_subscribers', False)
                        motor_subs = getattr(fq_sampler, '_has_motor_subscribers', False)
                        
                        logger.info(f"      {sampler_name}: {running_status} @ {sample_freq:.1f}Hz")
                        logger.info(f"         📺 Viz subscribers: {'YES' if viz_subs else 'NO'}")
                        logger.info(f"         🚗 Motor subscribers: {'YES' if motor_subs else 'NO'}")
                        
                        # Try to get sample data for current burst
                        if hasattr(fq_sampler, '_get_global_fire_queue_data'):
                            sample_data = fq_sampler._get_global_fire_queue_data()
                            if sample_data and sample_data.get('neuron_ids'):
                                sample_count = len(sample_data['neuron_ids'])
                                logger.info(f"         📊 Sample data: {sample_count} neurons")
                                
                                # Show membrane potential range if available
                                if sample_data.get('membrane_potentials'):
                                    potentials = sample_data['membrane_potentials']
                                    min_pot = min(potentials)
                                    max_pot = max(potentials)
                                    avg_pot = sum(potentials) / len(potentials)
                                    logger.info(f"         🧠 Membrane potentials: {min_pot:.2f} - {max_pot:.2f} (avg: {avg_pot:.2f})")
                                    
                                # Show coordinate range if available
                                if sample_data.get('coordinates'):
                                    coords = sample_data['coordinates']
                                    if coords:
                                        x_coords = [c[0] for c in coords]
                                        y_coords = [c[1] for c in coords]
                                        z_coords = [c[2] for c in coords]
                                        logger.info(f"         📍 Coordinate ranges: X:{min(x_coords)}-{max(x_coords)} Y:{min(y_coords)}-{max(y_coords)} Z:{min(z_coords)}-{max(z_coords)}")
                            else:
                                logger.info(f"         📊 Sample data: No neurons firing")
                        
                        # Check queue status
                        if hasattr(fq_sampler, 'output_queue'):
                            try:
                                queue_size = fq_sampler.output_queue.qsize()
                                logger.info(f"         📤 Output queue: {queue_size} items")
                            except:
                                logger.info(f"         📤 Output queue: Status unknown")
                                
                    except Exception as sampler_error:
                        logger.info(f"      FQSampler-{i+1}: ERROR - {sampler_error}")
            else:
                logger.info(f"   📺 FQ Samplers: NONE REGISTERED")
            
            # === MOTOR/VISUALIZATION STREAM DEBUGGING ===
            if self._fq_samplers:
                logger.info(f"🚗 Motor & Visualization Stream Debug:")
                
                # Check if any samplers are active for motor output
                motor_active_count = 0
                viz_active_count = 0
                
                for sampler in self._fq_samplers:
                    if getattr(sampler, '_has_motor_subscribers', False):
                        motor_active_count += 1
                    if getattr(sampler, '_has_visualization_subscribers', False):
                        viz_active_count += 1
                
                logger.info(f"   🚗 Motor stream: {motor_active_count} active samplers")
                logger.info(f"   📺 Visualization stream: {viz_active_count} active samplers")
                
                if motor_active_count == 0 and viz_active_count == 0:
                    logger.info(f"   ⚠️  WARNING: No active subscribers - streams may be inactive!")
                
                # Sample recent data from each active FQ sampler
                for i, fq_sampler in enumerate(self._fq_samplers):
                    if getattr(fq_sampler, 'running', False) and (
                        getattr(fq_sampler, '_has_visualization_subscribers', False) or 
                        getattr(fq_sampler, '_has_motor_subscribers', False)
                    ):
                        logger.info(f"   📊 FQSampler-{i+1} Recent Sample:")
                        
                        # Try to get per-area samples for key areas
                        if hasattr(fq_sampler, 'connectome_manager') and fq_sampler.connectome_manager:
                            try:
                                # Sample a few key areas (motor and sensory)
                                sample_areas = ['motor_', 'vision', 'sensor', 'output', 'input']  # Common prefixes
                                sampled_any = False
                                
                                for area_id in list(fcl_by_cortical.keys())[:5]:  # Sample first 5 active areas
                                    area_sample = fq_sampler._get_area_fire_queue_data(area_id)
                                    if area_sample and area_sample.get('neuron_ids'):
                                        neuron_count = len(area_sample['neuron_ids'])
                                        logger.info(f"      {area_id}: {neuron_count} active neurons")
                                        sampled_any = True
                                
                                if not sampled_any:
                                    logger.info(f"      No area samples available")
                                    
                            except Exception as sample_error:
                                logger.info(f"      Sample error: {sample_error}")
            
            logger.info(f"🔥 ========================================\n")
            
        except Exception as e:
            logger.error(f"🔥 NPU DEBUG ERROR: Failed to display fire queue - {e}")
            logger.error(f"NPU debug output error: {e}")
            # Include stack trace for debugging
            import traceback
            logger.error(f"🔥 NPU DEBUG ERROR stack trace:")
            logger.error(traceback.format_exc())

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
        if self.debug_npu:
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
            if self.debug_npu:
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

    def register_fq_sampler(self, fq_sampler: Any) -> None:
        """
        Register an FQ sampler for debugging and monitoring.
        
        Args:
            fq_sampler: FQSampler instance to register
        """
        if fq_sampler not in self._fq_samplers:
            self._fq_samplers.append(fq_sampler)
            if self.debug_npu:
                logger.info(f"🔥 NPU DEBUG: Registered FQ sampler - Total FQ samplers: {len(self._fq_samplers)}")

    def unregister_fq_sampler(self, fq_sampler: Any) -> None:
        """
        Unregister an FQ sampler.
        
        Args:
            fq_sampler: FQSampler instance to unregister
        """
        if fq_sampler in self._fq_samplers:
            self._fq_samplers.remove(fq_sampler)
            if self.debug_npu:
                logger.info(f"🔥 NPU DEBUG: Unregistered FQ sampler - Total FQ samplers: {len(self._fq_samplers)}")

    def _init_simd_support(self):
        """Initialize SIMD detection and configuration."""
        if SIMD_AVAILABLE:
            try:
                self.simd_detector = get_simd_detector()
                self.backend_selector = get_backend_selector()
                self.simd_config = get_simd_config()
                self.simd_profiler = get_profiler()
                
                # Log SIMD capabilities
                caps = self.simd_detector.capabilities
                self.logger.info(f"🧮 SIMD Backend: {self.simd_config['recommended_backend']}")
                self.logger.info(f"🎯 Vector Width: {caps.vector_width}, Cache Line: {caps.cache_line_size}B")
                
            except Exception as e:
                self.logger.warning(f"Failed to initialize SIMD support: {e}")
                self.simd_detector = None
                self.backend_selector = None
                self.simd_config = {}
        else:
            self.simd_detector = None
            self.backend_selector = None
            self.simd_config = {}

    def initialize_membrane_processor(self, capacity: int):
        """Initialize SIMD-optimized membrane processor with given capacity."""
        if SIMD_AVAILABLE and self.membrane_processor is None:
            try:
                self.membrane_processor = SIMDMembraneProcessor(
                    capacity=capacity,
                    use_profiling=self.use_simd_profiling
                )
                self.logger.info(f"🧠 SIMD membrane processor initialized for {capacity} neurons")
            except Exception as e:
                self.logger.warning(f"Failed to initialize SIMD membrane processor: {e}")
                self.membrane_processor = None

    def burst(self, burst_size: Optional[int] = None, use_gpu: bool = False) -> Dict[str, Any]:
        """
        Execute a neural processing burst with SIMD optimization.
        
        Args:
            burst_size: Number of neurons to process (None for auto-sizing)
            use_gpu: Whether to use GPU acceleration
            
        Returns:
            Burst execution results and performance metrics
        """
        
        burst_start = time.perf_counter()
        burst_results = {
            "neurons_processed": 0,
            "neurons_fired": 0,
            "processing_time": 0.0,
            "simd_efficiency": 0.0,
            "backend_used": "unknown"
        }
        
        try:
            # SIMD profiling session
            if SIMD_AVAILABLE and self.use_simd_profiling:
                with self.simd_profiler.profile_session(f"burst_{self.burst_count}"):
                    burst_results = self._execute_burst_impl(burst_size, use_gpu, burst_start)
            else:
                burst_results = self._execute_burst_impl(burst_size, use_gpu, burst_start)
                
        except Exception as e:
            self.logger.error(f"Burst execution failed: {e}", exc_info=True)
            burst_results["error"] = str(e)
        
        # Update performance statistics
        self.burst_count += 1
        self.total_neurons_processed += burst_results.get("neurons_processed", 0)
        
        # Periodic performance reporting
        if self.performance_monitoring and time.time() - self.last_performance_report > 10.0:
            self._report_performance()
            self.last_performance_report = time.time()
        
        return burst_results
    
    def _execute_burst_impl(self, burst_size: Optional[int], use_gpu: bool, burst_start: float) -> Dict[str, Any]:
        """Core burst execution implementation with SIMD optimization."""
        
        results = {
            "neurons_processed": 0,
            "neurons_fired": 0,
            "processing_time": 0.0,
            "simd_efficiency": 0.0,
            "backend_used": "scalar"
        }
        
        # Determine optimal burst size
        if burst_size is None:
            if SIMD_AVAILABLE and self.backend_selector:
                burst_size = self.backend_selector.get_chunk_size(10000)  # Default processing size
            else:
                burst_size = 1000  # Conservative default
        
        # Initialize membrane processor if needed
        if SIMD_AVAILABLE and self.membrane_processor is None:
            self.initialize_membrane_processor(burst_size * 2)  # Some headroom
        
        # Get fire candidates with SIMD optimization
        if SIMD_AVAILABLE and self.use_simd_profiling:
            with profile_simd_operation("fire_candidate_detection", burst_size):
                fire_candidates = self._get_fire_candidates_simd()
        else:
            fire_candidates = self._get_fire_candidates_simd() if SIMD_AVAILABLE else []
        
        # Process membrane potential updates with SIMD
        if fire_candidates and SIMD_AVAILABLE and self.membrane_processor:
            with profile_simd_operation("membrane_processing", len(fire_candidates)):
                fired_neurons = self._process_membrane_updates_simd(fire_candidates)
        else:
            fired_neurons = fire_candidates  # Fallback
        
        # Update results
        results["neurons_processed"] = burst_size
        results["neurons_fired"] = len(fired_neurons)
        results["processing_time"] = time.perf_counter() - burst_start
        
        if SIMD_AVAILABLE:
            results["backend_used"] = self.simd_config.get("recommended_backend", "scalar")
            
            # Get SIMD efficiency from profiler
            if self.use_simd_profiling and hasattr(self.simd_profiler, 'current_session'):
                session = self.simd_profiler.current_session
                if session and session.operations:
                    avg_efficiency = np.mean([
                        op.simd_efficiency for op in session.operations.values()
                    ])
                    results["simd_efficiency"] = avg_efficiency
        
        return results
    
    def _get_fire_candidates_simd(self) -> List[int]:
        """Get fire candidates using SIMD-optimized detection."""
        
        # This would integrate with the actual connectome/GNA
        # For now, return empty list as placeholder
        if hasattr(self, 'gna') and hasattr(self.gna, 'simd_optimized_find_fire_candidates'):
            return self.gna.simd_optimized_find_fire_candidates(self.burst_count)
        
        return []
    
    def _process_membrane_updates_simd(self, candidates: List[int]) -> List[int]:
        """Process membrane potential updates using SIMD optimization."""
        
        if not candidates or not self.membrane_processor:
            return candidates
        
        try:
            # Convert to arrays for SIMD processing
            candidate_indices = np.array(candidates, dtype=np.int32)
            input_currents = np.ones(len(candidates), dtype=np.float32)  # Placeholder currents
            
            # Use SIMD-optimized membrane update
            fired_neurons = self.membrane_processor.vectorized_membrane_update(
                candidate_indices, input_currents
            )
            
            return fired_neurons.tolist()
            
        except Exception as e:
            self.logger.warning(f"SIMD membrane processing failed, using fallback: {e}")
            return candidates
    
    def _report_performance(self):
        """Report SIMD performance statistics."""
        
        if not SIMD_AVAILABLE:
            return
        
        try:
            # Get performance stats
            if self.membrane_processor:
                stats = self.membrane_processor.get_performance_stats()
                self.logger.info(
                    f"🚀 SIMD Performance: {stats['avg_neurons_per_update']:.0f} neurons/update, "
                    f"backend: {stats['simd_backend']}, vector_width: {stats['vector_width']}"
                )
            
            # Get profiler report if available
            if self.use_simd_profiling and hasattr(self.simd_profiler, 'sessions'):
                if self.simd_profiler.sessions:
                    report = self.simd_profiler.get_performance_report()
                    top_ops = report.get("top_operations", [])
                    if top_ops:
                        self.logger.info(f"🔥 Top SIMD operation: {top_ops[0]['name']} "
                                       f"({top_ops[0]['simd_efficiency']:.2f} efficiency)")
                        
        except Exception as e:
            self.logger.debug(f"Performance reporting failed: {e}")

class FQSampler:
    """
    Fire Queue Sampler for FEAGI NPU.
    
    This class implements differentiated sampling behavior based on subscriber types:
    - Visualization subscribers: Sample all cortical areas at configured rate
    - Motor subscribers: Sample only OPU cortical areas at burst frequency
    
    The sampler is RTOS-compatible and optimized for real-time performance.
    """
    
    def __init__(self, fire_queue_provider: Any, sample_frequency_hz: float, 
                 output_queue: Any, connectome_manager: Optional[Any] = None) -> None:
        """
        Initialize the FQ sampler.
        
        Args:
            fire_queue_provider: Object providing fire queue data
            sample_frequency_hz: Base sampling frequency for visualization
            output_queue: Queue for output data
            connectome_manager: Optional connectome manager for area info
        """
        self.fire_queue_provider = fire_queue_provider
        self.sample_frequency = sample_frequency_hz
        self.sample_interval = 1.0 / sample_frequency_hz if sample_frequency_hz > 0 else 0.1
        self.output_queue = output_queue
        self.connectome_manager = connectome_manager
        self.running = False
        
        # Subscriber tracking
        self._has_visualization_subscribers = False
        self._has_motor_subscribers = False
        
        # Per-area sample timing for visualization (respects configured rates)
        self._last_sample_time_per_area: Dict[str, float] = {}
        
        # Motor sampling tracking (every burst for OPU areas)
        self._last_motor_sample_time: float = 0.0
        self._motor_sample_interval = 1.0  # Default 1Hz for motor, will be updated based on burst frequency
        
        # Error handling
        self._max_retries = 3
        self._retry_delay = 0.001  # 1ms
        
        # Get burst frequency from provider if available (for motor sampling)
        self._update_motor_sample_rate()
        
        # Auto-register with BurstEngine if available
        try:
            burst_engine = BurstEngine.get_instance()
            if burst_engine:
                burst_engine.register_fq_sampler(self)
                logger.info("FQSampler auto-registered with BurstEngine")
        except Exception as e:
            if "No BurstEngine instance exists" not in str(e):
                logger.warning(f"Failed to auto-register FQSampler with BurstEngine: {e}")
        
        logger.info(f"FQSampler initialized with {sample_frequency_hz}Hz sampling")

    def _update_motor_sample_rate(self):
        """Update motor sampling rate based on burst frequency."""
        try:
            # Try to get burst frequency from fire queue provider
            if hasattr(self.fire_queue_provider, 'get_burst_frequency'):
                burst_freq = self.fire_queue_provider.get_burst_frequency()
                if burst_freq and burst_freq > 0:
                    self._motor_sample_interval = 1.0 / burst_freq
                    logger.info(f"Motor sampling set to burst frequency: {burst_freq}Hz")
                    return
            
            # Try to get from BurstEngine
            try:
                burst_engine = BurstEngine.get_instance()
                if burst_engine and hasattr(burst_engine, '_configuration'):
                    burst_freq = burst_engine._configuration.get('burst_frequency', 1)
                    self._motor_sample_interval = 1.0 / burst_freq
                    logger.info(f"Motor sampling set to burst frequency from BurstEngine: {burst_freq}Hz")
                    return
            except Exception:
                pass
                
            # Default fallback
            logger.info("Using default motor sampling rate: 1Hz")
            
        except Exception as e:
            logger.warning(f"Error updating motor sample rate: {e}")

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
            # Update motor sample rate when motor subscribers change
            if has_subscribers:
                self._update_motor_sample_rate()

    def run(self) -> None:
        """Main sampling loop with differentiated behavior for visualization and motor."""
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
            
            # Process visualization sampling (all areas at configured rates)
            if self._has_visualization_subscribers:
                self._process_visualization_sampling(now)
            
            # Process motor sampling (OPU areas at burst frequency)
            if self._has_motor_subscribers:
                self._process_motor_sampling(now)
                    
            # Sleep for the remainder of the sample interval
            elapsed = time.perf_counter() - start
            sleep_time = min(self.sample_interval, self._motor_sample_interval)
            if elapsed < sleep_time:
                # RTOS-COMPATIBLE: Replace time.sleep with deterministic timing
                target_end_time = start + sleep_time
                while time.perf_counter() < target_end_time:
                    pass  # Busy-wait for remainder of sample interval
                
        logger.info("FQSampler stopped.")

    def _process_visualization_sampling(self, now: float) -> None:
        """Process sampling for visualization subscribers (all areas at configured rates)."""
        try:
            if self.connectome_manager is not None:
                # Per-area sampling for visualization
                areas_processed = 0
                areas_with_data = 0
                
                for area in self.connectome_manager.cortical_areas_by_id.values():
                    cortical_id = area.id
                    areas_processed += 1
                    
                    # Get per-area sample rate if set, else use global
                    rate = area.properties.get('fq_sample_rate', self.sample_frequency)
                    
                    # Skip sampling if rate is zero
                    if rate <= 0:
                        continue
                        
                    interval = 1.0 / rate
                    last_time = self._last_sample_time_per_area.get(cortical_id, 0)
                    
                    if now - last_time >= interval:
                        # Debug: Log what we're trying to sample
                        logger.debug(f"🔬 FQ Sampler: Attempting to sample area {cortical_id} (type: {area.area_type})")
                        
                        # Try to get area fire queue data first for debugging
                        test_data = self._get_area_fire_queue_data(cortical_id)
                        if test_data and test_data.get('neuron_ids'):
                            neuron_count = len(test_data['neuron_ids'])
                            logger.debug(f"🎯 FQ Sampler: Area {cortical_id} has {neuron_count} firing neurons")
                            areas_with_data += 1
                        else:
                            logger.debug(f"❌ FQ Sampler: Area {cortical_id} has no fire queue data")
                           
                        self._sample_area_fire_queue(cortical_id, target='visualization')
                        self._last_sample_time_per_area[cortical_id] = now
                        
                # Log summary every 100 samples
                if hasattr(self, '_debug_sample_count'):
                    self._debug_sample_count += 1
                else:
                    self._debug_sample_count = 1
                    
                if self._debug_sample_count % 100 == 0:
                    logger.info(f"🔬 FQ Sampler Debug: Processed {areas_processed} areas, {areas_with_data} had data")
            else:
                # Global sampling for visualization
                logger.debug("🌍 FQ Sampler: Using global sampling (no connectome manager)")
                self._sample_global_fire_queue(target='visualization')
                
        except Exception as e:
            logger.error(f"Error in visualization sampling: {e}")
            import traceback
            traceback.print_exc()

    def _process_motor_sampling(self, now: float) -> None:
        """Process sampling for motor subscribers (OPU areas at burst frequency)."""
        try:
            # Check if it's time for motor sampling
            if now - self._last_motor_sample_time < self._motor_sample_interval:
                return
                
            self._last_motor_sample_time = now
            
            if self.connectome_manager is not None:
                # Sample only OPU (Output Processing Unit) areas for motor
                opu_areas = self._get_opu_cortical_areas()
                
                for cortical_id in opu_areas:
                    self._sample_area_fire_queue(cortical_id, target='motor')
            else:
                # Global sampling for motor (will be filtered by motor stream)
                self._sample_global_fire_queue(target='motor')
                
        except Exception as e:
            logger.error(f"Error in motor sampling: {e}")

    def _get_opu_cortical_areas(self) -> List[str]:
        """Get list of OPU (Output Processing Unit) cortical area IDs."""
        opu_areas = []
        
        try:
            if not self.connectome_manager:
                return opu_areas
                
            for area in self.connectome_manager.cortical_areas_by_id.values():
                # Check if area is of type OPU
                area_type = area.properties.get('cortical_type', '').upper()
                
                # Multiple ways to identify OPU areas
                is_opu = (
                    area_type == 'OPU' or
                    area_type == 'OUTPUT' or
                    area_type == 'MOTOR' or
                    'OPU' in area_type or
                    'OUTPUT' in area_type or
                    'MOTOR' in area_type or
                    area.id.startswith('opu_') or
                    area.id.startswith('motor_') or
                    area.id.startswith('output_')
                )
                
                if is_opu:
                    opu_areas.append(area.id)
                    logger.debug(f"Found OPU area for motor sampling: {area.id} (type: {area_type})")
                    
        except Exception as e:
            logger.error(f"Error identifying OPU areas: {e}")
            
        if not opu_areas:
            logger.warning("No OPU areas found for motor sampling. Motor subscribers may not receive data.")
            
        return opu_areas

    def _sample_area_fire_queue(self, cortical_id: str, target: str = 'visualization') -> None:
        """Sample fire queue data for a specific cortical area."""
        retry_count = 0
        
        while retry_count < self._max_retries:
            try:
                # Get fire queue data for this area
                area_fire_data = self._get_area_fire_queue_data(cortical_id)
                
                if area_fire_data:
                    # Only log when we actually have data to process
                    neuron_count = len(area_fire_data.get('neuron_ids', []))
                    
                    try:
                        # For backward compatibility, also support tuple format
                        if target == 'visualization':
                            # THE CRITICAL PUT OPERATION
                            self.output_queue.put((cortical_id, area_fire_data))
                            logger.debug(f"📤 Queued {cortical_id}: {neuron_count} neurons for visualization")
                        else:
                            # For motor, use tagged format
                            data_package = {
                                'cortical_id': cortical_id,
                                'fire_queue_data': area_fire_data,
                                'target': target,
                                'timestamp': time.time()
                            }
                            self.output_queue.put(data_package)
                            logger.debug(f"📤 Queued {cortical_id}: {neuron_count} neurons for {target}")
                            
                        break  # Success
                        
                    except Exception as put_error:
                        logger.error(f"❌ Error queuing {cortical_id} data: {put_error}")
                        break  # Don't retry on queue errors
                        
                else:
                    # Only log occasionally for areas with no data to avoid spam
                    if retry_count == 0:  # Only log on first attempt
                        logger.debug(f"📭 No data for {cortical_id}")
                    break
                    
            except Exception as general_error:
                logger.error(f"❌ Error sampling {cortical_id}: {general_error}")
                if retry_count == self._max_retries - 1:
                    logger.error(f"❌ Max retries reached for {cortical_id}")
                    
                # Wait before retrying
                delay_start = time.perf_counter()
                while time.perf_counter() - delay_start < self._retry_delay:
                    pass  # Busy-wait for retry delay
                retry_count += 1

    def _sample_global_fire_queue(self, target: str = 'visualization') -> None:
        """Sample global fire queue data."""
        retry_count = 0
        
        while retry_count < self._max_retries:
            try:
                # Get global fire queue data
                fire_data = self._get_global_fire_queue_data()
                
                if fire_data:
                    neuron_count = len(fire_data.get('neuron_ids', []))
                    
                    try:
                        # Tag the data with target type for proper routing
                        if target == 'motor':
                            data_package = {
                                'fire_queue_data': fire_data,
                                'target': target,
                                'timestamp': time.time()
                            }
                            self.output_queue.put_nowait(data_package)
                            logger.debug(f"📤 Queued global motor data: {neuron_count} neurons")
                        else:
                            # For visualization, use existing format
                            self.output_queue.put_nowait(fire_data)
                            logger.debug(f"📤 Queued global visualization data: {neuron_count} neurons")
                            
                        break  # Success
                        
                    except Exception as put_error:
                        logger.error(f"❌ Error queuing global data: {put_error}")
                        break
                        
                else:
                    logger.debug(f"📭 No global fire queue data")
                    break
                    
            except Exception as general_error:
                logger.error(f"❌ Error in global sampling: {general_error}")
                if retry_count == self._max_retries - 1:
                    logger.error(f"❌ Max retries reached for global sampling")
                    
                # Wait before retrying
                delay_start = time.perf_counter()
                while time.perf_counter() - delay_start < self._retry_delay:
                    pass
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
            logger.debug(f"🔍 Getting fire queue data for area: {cortical_id}")
            
            # Get fire queue from provider
            if hasattr(self.fire_queue_provider, 'get_area_fire_queue'):
                logger.debug(f"🔍 Using provider.get_area_fire_queue for {cortical_id}")
                fire_queue = self.fire_queue_provider.get_area_fire_queue(cortical_id)
                logger.debug(f"🔍 Provider returned: {fire_queue is not None} for {cortical_id}")
                
                if fire_queue:
                    neuron_ids = fire_queue.get('neuron_ids', [])
                    logger.debug(f"🔍 Fire queue has {len(neuron_ids)} neuron_ids for {cortical_id}")
               
            elif hasattr(self.fire_queue_provider, 'get_fire_queue'):
                logger.debug(f"🔍 Using global fire queue and filtering for {cortical_id}")
                # Filter global fire queue for this area
                global_fire_queue = self.fire_queue_provider.get_fire_queue()
                logger.debug(f"🔍 Global fire queue: {global_fire_queue is not None}")
                
                if global_fire_queue:
                    global_neuron_count = len(global_fire_queue.get('neuron_ids', []))
                    logger.debug(f"🔍 Global fire queue has {global_neuron_count} neurons total")
                    
                fire_queue = self._filter_fire_queue_by_area(global_fire_queue, cortical_id)
                
                if fire_queue:
                    filtered_neuron_count = len(fire_queue.get('neuron_ids', []))
                    logger.debug(f"🔍 Filtered fire queue has {filtered_neuron_count} neurons for {cortical_id}")
            else:
                logger.warning(f"🔍 Fire queue provider has no suitable methods for {cortical_id}")
                return None
                
            if not fire_queue:
                logger.debug(f"❌ No fire queue returned for {cortical_id}")
                return None
                
            if not fire_queue.get('neuron_ids'):
                logger.debug(f"❌ Fire queue has no neuron_ids for {cortical_id}")
                return None
                
            # Add coordinate information
            coordinates = self._get_neuron_coordinates(cortical_id, fire_queue['neuron_ids'])
            fire_queue['coordinates'] = coordinates
            
            logger.debug(f"✅ Returning fire queue data for {cortical_id} with {len(fire_queue['neuron_ids'])} neurons")
            return fire_queue
            
        except Exception as e:
            logger.error(f"Error getting area fire queue data for {cortical_id}: {e}")
            import traceback
            traceback.print_exc()
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
            area = self.connectome_manager.cortical_areas_by_id.get(cortical_id)
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
        
        # Add detailed debugging for coordinate issues
        logger.info(f"🔍 COORDINATE DEBUG: Getting coordinates for {len(neuron_ids)} neurons in area {cortical_id}")
        logger.info(f"   🔢 Neuron IDs being processed: {neuron_ids[:10]}{'...' if len(neuron_ids) > 10 else ''}")
        
        try:
            if self.connectome_manager:
                area = self.connectome_manager.cortical_areas_by_id.get(cortical_id)
                if area:
                    logger.info(f"   ✅ Found area '{area.name}' with {area.neuron_count} total neurons")
                    
                    # Check how many neurons in the area have positions
                    total_neurons_with_positions = len(area._position_map) if hasattr(area, '_position_map') else 0
                    logger.info(f"   📍 Area has {total_neurons_with_positions} neurons with stored positions")
                    
                    # Show sample of neurons with positions for debugging
                    if hasattr(area, '_position_map') and area._position_map:
                        sample_stored_neurons = list(area._position_map.keys())[:5]
                        logger.info(f"   📝 Sample neurons with positions: {sample_stored_neurons}")
                    
                    found_positions = 0
                    fallback_positions = 0
                    default_positions = 0
                    
                    for neuron_id in neuron_ids:
                        try:
                            # Try to get actual neuron position from area
                            position = area.get_neuron_position(neuron_id)
                            if position is not None:
                                coordinates.append(position)
                                found_positions += 1
                                continue
                            
                            # Log which neurons don't have stored positions
                            logger.debug(f"   ❌ Neuron {neuron_id} not found in area position map")
                            
                            # If neuron not in this area's position map, try fallback calculation
                            # Use area dimensions (tuple format: width, height, depth)
                            width, height, depth = area.dimensions
                            
                            # Calculate position within area based on neuron_id
                            x = neuron_id % width
                            y = (neuron_id // width) % height
                            z = (neuron_id // (width * height)) % depth
                            coordinates.append((x, y, z))
                            fallback_positions += 1
                            
                        except Exception as e:
                            logger.debug(f"Error getting position for neuron {neuron_id} in {cortical_id}: {e}")
                            # Default coordinates
                            coordinates.append((0, 0, 0))
                            default_positions += 1
                    
                    # Log summary of coordinate lookup results
                    logger.info(f"   📊 Coordinate lookup results for {cortical_id}:")
                    logger.info(f"      ✅ Found stored positions: {found_positions}")
                    logger.info(f"      🔄 Fallback calculations: {fallback_positions}")
                    logger.info(f"      ⚠️  Default (0,0,0) coords: {default_positions}")
                    
                    if found_positions == 0 and len(neuron_ids) > 0:
                        logger.warning(f"   🚨 CRITICAL: NO stored positions found for ANY neurons in {cortical_id}!")
                        logger.warning(f"      This indicates neurons in FCL were not properly created during embryogenesis")
                        logger.warning(f"      or there's a mismatch between FCL neuron IDs and stored neuron IDs")
                        
                        # Check if these neurons exist at all in the connectome
                        if hasattr(self.connectome_manager, 'neurons'):
                            existing_neurons = [nid for nid in neuron_ids if nid in self.connectome_manager.neurons]
                            logger.warning(f"      🔍 {len(existing_neurons)}/{len(neuron_ids)} neurons exist in global connectome")
                            if existing_neurons:
                                logger.warning(f"         Sample existing: {existing_neurons[:5]}")
                            
                else:
                    logger.warning(f"No area found for cortical_id {cortical_id}, using default coordinates")
                    # No area found, use default coordinates
                    coordinates = [(0, 0, 0) for _ in neuron_ids]
            else:
                logger.warning("No connectome manager, estimating coordinates")
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
                    for cortical_id, area in self.connectome_manager.cortical_areas_by_id.items():
                        try:
                            # Check if this neuron is in this area by checking the position map
                            position = area.get_neuron_position(neuron_id)
                            if position is not None:
                                coordinates.append(position)
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
        
        # Auto-unregister from burst engine if it was registered
        if hasattr(self.fire_queue_provider, 'unregister_fq_sampler'):
            try:
                self.fire_queue_provider.unregister_fq_sampler(self)
                logger.info(f"🔥 NPU DEBUG: Unregistered FQ sampler - Total FQ samplers: {len(self._fq_samplers)}")
            except Exception as e:
                logger.warning(f"Failed to auto-unregister FQSampler from BurstEngine: {e}")
        
    def update_area_sample_rate(self, cortical_id: str, rate: float) -> None:
        """Set the sampling rate for a specific cortical area."""
        if cortical_id not in self._last_sample_time_per_area:
            self._last_sample_time_per_area[cortical_id] = time.perf_counter() 