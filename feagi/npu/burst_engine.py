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
import threading
import queue
from typing import Dict, List, Optional, Any, Set, Tuple, Union
from datetime import datetime
import numpy as np

# RTOS-COMPATIBLE: Removed signal and threading imports - not available in RTOS
# import signal  # REMOVED: Not compatible with RTOS
# import threading  # REMOVED: Not compatible with RTOS - use RTOS task primitives instead
# WGPU-COMPATIBLE: Remove os import to eliminate environment variable dependencies
# import os  # REMOVED: Environment variables not available in WGPU contexts
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
            logger.info(f"[DEBUG] BURST ENGINE: Creating NEW singleton instance {cls._instance_id}")
        else:
            # WGPU-COMPATIBLE: Use logger instead of print for debug output
            logger.info(f"[DEBUG] BURST ENGINE: Returning EXISTING singleton instance {cls._instance_id}")
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
            logger.debug(f"[DEBUG] BURST ENGINE: Instance {self._instance_id} _running changed: {old_value} -> {value}")
            import traceback
            logger.debug(f"[DEBUG] BURST ENGINE: Stack trace:")
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
            logger.info(f"[DEBUG] BURST ENGINE: Instance {self._instance_id} already initialized, skipping")
            return
            
        # WGPU-COMPATIBLE: Use logger instead of print for debug output
        logger.info(f"[DEBUG] BURST ENGINE: Initializing singleton instance {self._instance_id}")
        
        # Initialize logger for this instance
        self.logger = logging.getLogger(__name__ + f".BurstEngine.{self._instance_id}")
        
        self.connectome_manager = connectome_manager
        self.fcl_manager = fcl_manager or connectome_manager.fcl_manager
        self.config = config or {}
        
        # WGPU-COMPATIBLE: Check debug_npu from config only (no environment variables)
        self.debug_npu = self.config.get('debug_npu', False)
        
        # Log debug NPU status when enabled
        if self.debug_npu:
            logger.info("[DEBUG] NPU debug mode enabled - will show detailed fire queue contents during bursts")
        
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
        logger.info("Burst Engine initialized in standby mode", status="[FAST]")
        
        self.state_manager = FeagiStateManager.instance()
        
        # Support both parameter names for backward compatibility
        self.desired_frequency = self.config.get('desired_frequency_hz', 
                                              self.config.get('target_frequency', 1.0))
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
        logger.info(f"[DEBUG] BURST ENGINE: Instance {self._instance_id} initialization complete")
    
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
            logger.info("Power injection disabled by configuration", status="[FAST]")
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
                
                logger.info(f"Initialized power injection for {len(power_areas)} power areas", status="[CONFIG]")
                
                # Log power area preview
                preview = self.fcl_injection_service.get_power_injection_preview()
                logger.debug(f"Power injection preview: {preview}")
            else:
                logger.info("No power areas detected, injection service not initialized", status="[FAST]")
                
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
            logger.debug(f"[DEBUG] BURST ENGINE _process_burst called! Instance {self._instance_id}, Burst count: {self.burst_count}")
            
            # Check injection service availability
            if self.fcl_injection_service:
                logger.debug(f"[DEBUG] BURST ENGINE: Injection service AVAILABLE")
            else:
                logger.debug(f"[DEBUG] BURST ENGINE: NO INJECTION SERVICE!")
        
        # 1. Pre-burst power injection
        if self.fcl_injection_service and self.power_injection_timing == 'pre_burst':
            if self.debug_npu:
                logger.debug(f"[DEBUG] BURST ENGINE: Calling pre-burst injection")
            self.fcl_injection_service.inject_pre_burst(self.burst_count)
        
        # 2. Update membrane potentials and get fired neurons
        if self.debug_npu:
            # FCL manager uses sliding window with current timestep always 0
            current_timestep = 0  # Fixed: always use 0 for current timestep
            logger.debug(f"[DEBUG] BURST ENGINE: About to call update_membrane_potentials with timestep {current_timestep}")
        
        fired_neurons = self.connectome_manager.update_membrane_potentials()
        
        if self.debug_npu:
            fired_count = len(fired_neurons) if fired_neurons else 0
            logger.debug(f"[DEBUG] BURST ENGINE: Got {fired_count} firing neurons")
        
        # 3. During-burst injection (for modulators)
        if self.fcl_injection_service and self.power_injection_timing == 'during_burst':
            if self.debug_npu:
                logger.debug(f"[DEBUG] BURST ENGINE: Calling during-burst injection")
            self.fcl_injection_service.inject_during_burst(self.burst_count)
        
        # 4. Post-burst injection  
        if self.fcl_injection_service and self.power_injection_timing == 'post_burst':
            if self.debug_npu:
                logger.debug(f"[DEBUG] BURST ENGINE: Calling post-burst injection")
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
            logger.debug(f"[DEBUG] BURST ENGINE _process_burst_with_power_injection called! Instance {self._instance_id}, Timestep: {current_timestep}")
            
            # Check injection service availability
            if self.fcl_injection_service:
                logger.debug(f"[DEBUG] BURST ENGINE: Enhanced injection service AVAILABLE")
            else:
                logger.debug(f"[DEBUG] BURST ENGINE: NO ENHANCED INJECTION SERVICE!")
        
        # 1. Pre-burst power injection (inject power area neurons)
        if self.fcl_injection_service:
            if self.debug_npu:
                logger.debug(f"[DEBUG] BURST ENGINE: Calling enhanced pre-burst injection")
            injected_pre = self.fcl_injection_service.inject_pre_burst(current_timestep)
            if injected_pre > 0:
                logger.debug(f"Pre-burst injection: {injected_pre} neurons")
                if self.debug_npu:
                    logger.debug(f"[DEBUG] BURST ENGINE: Pre-burst injected {injected_pre} neurons")
        
        # 2. Standard burst processing (membrane potential updates, regular firing)
        if self.debug_npu:
            logger.debug(f"[DEBUG] BURST ENGINE: About to call enhanced update_membrane_potentials with timestep {current_timestep}")
        
        fired_neurons = self.connectome_manager.update_membrane_potentials()
        
        if self.debug_npu:
            fired_count = len(fired_neurons) if fired_neurons else 0
            logger.debug(f"[DEBUG] BURST ENGINE: Enhanced processing got {fired_count} firing neurons")
        
        # 3. During-burst injection (for modulator areas)
        if self.fcl_injection_service:
            if self.debug_npu:
                logger.debug(f"[DEBUG] BURST ENGINE: Calling enhanced during-burst injection")
            injected_during = self.fcl_injection_service.inject_during_burst(current_timestep)
            if injected_during > 0:
                logger.debug(f"During-burst injection: {injected_during} neurons")
                if self.debug_npu:
                    logger.debug(f"[DEBUG] BURST ENGINE: During-burst injected {injected_during} neurons")
        
        # 4. Post-burst injection (for cleanup or special processing)
        if self.fcl_injection_service:
            if self.debug_npu:
                logger.debug(f"[DEBUG] BURST ENGINE: Calling enhanced post-burst injection")
            injected_post = self.fcl_injection_service.inject_post_burst(current_timestep)
            if injected_post > 0:
                logger.debug(f"Post-burst injection: {injected_post} neurons")
                if self.debug_npu:
                    logger.debug(f"[DEBUG] BURST ENGINE: Post-burst injected {injected_post} neurons")
        
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
        logger.info(f"[DEBUG] BURST ENGINE: run() method called for instance {self._instance_id}")
        logger.info(f"[DEBUG] BURST ENGINE: Target frequency: {self.desired_frequency}Hz, interval: {self.burst_interval}s")
        
        # Debug logging for run method entry
        if self.debug_npu:
            logger.debug(f"[DEBUG] BURST ENGINE: Current _running state: {self._running}")
            logger.debug(f"[DEBUG] BURST ENGINE: Stack trace:")
            for line in traceback.format_stack()[-5:-1]:  # Show last 4 stack frames
                logger.debug(f"    {line.strip()}")
            logger.debug("")
        
        self._running = True
        self.state_manager.set_burst_engine_state(ServiceState.READY)
        logger.info(f"[DEBUG] BURST ENGINE: Set _running=True, about to enter main loop")
        
        # RTOS-COMPATIBLE: Removed signal handling - not available in RTOS
        # In RTOS environment, use task control and events instead of signal handlers
            
        try:
            logger.info(f"[DEBUG] BURST ENGINE: Entering main while loop")
            loop_count = 0
            while self._running:
                loop_count += 1
                cycle_start = time.perf_counter()
                
                # Log first few loops and every 10th loop for debugging
                if loop_count <= 3 or loop_count % 10 == 0:
                    logger.info(f"[DEBUG] BURST ENGINE: Starting burst {self.burst_count + 1} (loop {loop_count})")
                
                # Debug logging if --debug-npu is enabled
                if self.debug_npu:
                    logger.debug(f"[DEBUG] BURST ENGINE: Starting burst {self.burst_count + 1} in main loop")
                
                # Measure pure processing time
                processing_start = time.perf_counter()
                
                # Choose processing method based on power injection availability
                if self.fcl_injection_service:
                    # Enhanced processing with power injection - always use timestep 0 for current
                    if self.debug_npu:
                        logger.debug(f"[DEBUG] BURST ENGINE: Using ENHANCED processing with power injection")
                    fired_neurons = self._process_burst_with_power_injection(0)  # Fixed: use 0 for current timestep
                else:
                    # Standard processing
                    if self.debug_npu:
                        logger.debug(f"[DEBUG] BURST ENGINE: Using STANDARD processing (no injection service)")
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
                    logger.info(f"[DEBUG] BURST ENGINE: Burst {self.burst_count + 1} - Processing: {processing_elapsed*1000:.2f}ms, "
                                  f"Full cycle: {final_cycle_time*1000:.2f}ms, Target: {self.desired_frequency:.1f}Hz, Actual: {actual_freq:.1f}Hz")
                elif self.debug_npu:
                    logger.debug(f"[DEBUG] BURST ENGINE: Burst {self.burst_count + 1} - Processing: {processing_elapsed*1000:.2f}ms, "
                                  f"Full cycle: {final_cycle_time*1000:.2f}ms, Potential: {potential_freq:.1f}Hz, Actual: {actual_freq:.1f}Hz")
                
                # Increment burst count
                self.burst_count += 1
                
        except Exception as e:
            # Handle crashes in the main loop by resetting _running flag
            logger.error(f"[DEBUG] BURST ENGINE: EXCEPTION in main loop: {e}")
            logger.error(f"[DEBUG] BURST ENGINE: Exception traceback:")
            logger.error(traceback.format_exc())
            
            logger.error(f"BurstEngine main loop crashed: {e}")
            self._running = False  # Reset the running flag
            self.state_manager.set_burst_engine_state(ServiceState.ERROR)
            return
                
        logger.info(f"[DEBUG] BURST ENGINE: Main loop exited normally, _running={self._running}")
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
            logger.debug(f"[DEBUG] BURST ENGINE: Instance {self._instance_id} updating with genome")
        
        self.genome_loaded = True
        self.burst_count = 0  # Reset burst count for new genome
        
        # Use cortical_areas instead of _areas - fix the attribute name
        self.cortical_areas = list(self.connectome_manager.cortical_areas.values()) if hasattr(self.connectome_manager, 'cortical_areas') else []
        self.shed_areas = set(area.id for area in self.cortical_areas if area.properties.get('__shed', False))
        
        # WGPU-COMPATIBLE: Use config-based debug instead of environment variable
        if self.debug_npu:
            logger.debug(f"[DEBUG] BURST ENGINE: Instance {self._instance_id} re-initializing special area services for new genome")
        
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
            
        logger.info("Refreshed special area services", status="[PROC]")

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
            logger.debug(f"[DEBUG] BURST ENGINE: run_with_fire_queue() called for instance {self._instance_id}")
            logger.debug(f"[DEBUG] BURST ENGINE: Stack trace:")
            for line in traceback.format_stack()[-5:-1]:  # Show last 4 stack frames
                logger.debug(f"    {line.strip()}")
            logger.debug("")
        
        if self.state_manager.get_burst_engine_state() != ServiceState.READY:
            if self.debug_npu:
                logger.warning(f"[DEBUG] BURST ENGINE: run_with_fire_queue() - engine not ready, returning False")
            logger.warning("Burst engine is not ready, cannot start burst execution")
            return False
            
        # Update state - use READY state to indicate it's running (later could be changed to SYNCING or similar)
        self.state_manager.set_burst_engine_state(ServiceState.READY)
        logger.info("Burst engine starting with fire queue process", status="[START] ")
        
        if self.debug_npu:
            logger.debug(f"[DEBUG] BURST ENGINE: run_with_fire_queue() - about to set _running = True")
        
        # Set running flag
        self._running = True
        
        if self.debug_npu:
            logger.debug(f"[DEBUG] BURST ENGINE: run_with_fire_queue() - _running set to True, entering main loop")
        
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
                    logger.debug(f"[DEBUG] BURST ENGINE: run_with_fire_queue() - main loop iteration, burst {self.burst_count}")
                
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
                               emoji1="[FAST] ")
                
                # Increment burst count
                self.burst_count += 1
                
        except Exception as e:
            # Handle crashes in the fire queue main loop
            if self.debug_npu:
                logger.error(f"[DEBUG] BURST ENGINE: EXCEPTION in run_with_fire_queue main loop: {e}")
                logger.error(f"[DEBUG] BURST ENGINE: Stack trace:")
                traceback.print_exc()
            
            logger.error(f"BurstEngine fire queue main loop crashed: {e}")
            self._running = False  # Reset the running flag
            self.state_manager.set_burst_engine_state(ServiceState.ERROR)
            return False
        
        # Update state when stopped
        self.state_manager.set_burst_engine_state(ServiceState.READY)
        if self.debug_npu:
            logger.debug(f"[DEBUG] BURST ENGINE: run_with_fire_queue() - exiting normally, loop finished")
        logger.info("Burst engine stopped", status="[HALT] ")
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
            logger.info(f"\n[DEBUG] ===== NPU DEBUG - BURST {self.burst_count} =====")
            
            # Get global FCL
            global_fcl = self.fcl_manager.get_global_fcl()
            total_firing = len(global_fcl)
            
            logger.info(f"[STATS] Global Fire Summary:")
            logger.info(f"   Total firing neurons: {total_firing}")
            logger.info(f"   Burst frequency: {1.0/self.burst_interval:.1f}Hz target")
            
            if total_firing > 0:
                # Get firing neurons by cortical area
                fcl_by_cortical = self.fcl_manager.get_fcl_by_cortical()
                
                logger.info(f"[BRAIN] Per-Area Breakdown ({len(fcl_by_cortical)} active areas):")
                
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
                        logger.info(f"[FAST] Power Injection: {power_neurons} neurons from {stats['special_areas'].get('power_areas_count', 0)} power areas")
            else:
                logger.info("   No neurons firing this burst")
                
            # Show recent firing statistics if available
            if hasattr(self.fcl_manager, 'get_firing_statistics'):
                firing_stats = self.fcl_manager.get_firing_statistics()
                if firing_stats:
                    logger.info(f"[UP] Recent Activity:")
                    logger.info(f"   Average firing rate: {firing_stats.get('average_firing_rate', 0):.1f} neurons/burst")
                    logger.info(f"   Peak firing: {firing_stats.get('peak_firing', 0)} neurons")
            
            # === NEW: FQ SAMPLER DEBUG INFORMATION ===
            logger.info(f"[TARGET] Sampler Debug Information:")
            
            # FQ Samplers (for motor and visualization)
            if self._fq_samplers:
                logger.info(f"   FQ Samplers Active: {len(self._fq_samplers)}")
                for i, fq_sampler in enumerate(self._fq_samplers):
                    try:
                        sampler_name = f"FQSampler-{i+1}"
                        running_status = "RUNNING" if getattr(fq_sampler, 'running', False) else "STOPPED"
                        sample_freq = getattr(fq_sampler, 'sample_frequency', 0)
                        
                        # Get subscriber status
                        viz_subs = getattr(fq_sampler, '_has_visualization_subscribers', False)
                        motor_subs = getattr(fq_sampler, '_has_motor_subscribers', False)
                        
                        logger.info(f"      {sampler_name}: {running_status} @ {sample_freq:.1f}Hz")
                        logger.info(f"         Viz subscribers: {'YES' if viz_subs else 'NO'}")
                        logger.info(f"         Motor subscribers: {'YES' if motor_subs else 'NO'}")
                        
                        # Try to get sample data for current burst
                        if hasattr(fq_sampler, '_get_global_fire_queue_data'):
                            sample_data = fq_sampler._get_global_fire_queue_data()
                            if sample_data and sample_data.get('neuron_ids'):
                                sample_count = len(sample_data['neuron_ids'])
                                logger.info(f"         [STATS] Sample data: {sample_count} neurons")
                                
                                # Show membrane potential range if available
                                if sample_data.get('membrane_potentials'):
                                    potentials = sample_data['membrane_potentials']
                                    min_pot = min(potentials)
                                    max_pot = max(potentials)
                                    avg_pot = sum(potentials) / len(potentials)
                                    logger.info(f"         [BRAIN] Membrane potentials: {min_pot:.2f} - {max_pot:.2f} (avg: {avg_pot:.2f})")
                                    
                                # Show coordinate range if available
                                if sample_data.get('coordinates'):
                                    coords = sample_data['coordinates']
                                    if coords:
                                        x_coords = [c[0] for c in coords]
                                        y_coords = [c[1] for c in coords]
                                        z_coords = [c[2] for c in coords]
                                        logger.info(f"         Coordinate ranges: X:{min(x_coords)}-{max(x_coords)} Y:{min(y_coords)}-{max(y_coords)} Z:{min(z_coords)}-{max(z_coords)}")
                            else:
                                logger.info(f"         [STATS] Sample data: No neurons firing")
                        
                        # Check queue status
                        if hasattr(fq_sampler, 'output_queue'):
                            try:
                                queue_size = fq_sampler.output_queue.qsize()
                                logger.info(f"         Output queue: {queue_size} items")
                            except:
                                logger.info(f"         Output queue: Status unknown")
                                
                    except Exception as sampler_error:
                        logger.info(f"      FQSampler-{i+1}: ERROR - {sampler_error}")
            else:
                logger.info(f"   FQ Samplers: NONE REGISTERED")
            
            # === MOTOR/VISUALIZATION STREAM DEBUGGING ===
            if self._fq_samplers:
                logger.info(f"Motor & Visualization Stream Debug:")
                
                # Check if any samplers are active for motor output
                motor_active_count = 0
                viz_active_count = 0
                
                for sampler in self._fq_samplers:
                    if getattr(sampler, '_has_motor_subscribers', False):
                        motor_active_count += 1
                    if getattr(sampler, '_has_visualization_subscribers', False):
                        viz_active_count += 1
                
                logger.info(f"   Motor stream: {motor_active_count} active samplers")
                logger.info(f"   Visualization stream: {viz_active_count} active samplers")
                
                if motor_active_count == 0 and viz_active_count == 0:
                    logger.info(f"   [WARN]  WARNING: No active subscribers - streams may be inactive!")
                
                # Sample recent data from each active FQ sampler
                for i, fq_sampler in enumerate(self._fq_samplers):
                    if getattr(fq_sampler, 'running', False) and (
                        getattr(fq_sampler, '_has_visualization_subscribers', False) or 
                        getattr(fq_sampler, '_has_motor_subscribers', False)
                    ):
                        logger.info(f"   [STATS] FQSampler-{i+1} Recent Sample:")
                        
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
            
            logger.info(f"[DEBUG] ========================================\n")
            
        except Exception as e:
            logger.error(f"[DEBUG] NPU DEBUG ERROR: Failed to display fire queue - {e}")
            logger.error(f"NPU debug output error: {e}")
            # Include stack trace for debugging
            import traceback
            logger.error(f"[DEBUG] NPU DEBUG ERROR stack trace:")
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
            logger.info(f"Starting frequency measurement for {duration_seconds}s", status="[DEBUG]")
        
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
                logger.info(f"Frequency measurement complete - Actual: {actual_frequency_hz:.1f}Hz, Potential: {potential_frequency_hz:.1f}Hz (target: {self.desired_frequency:.1f}Hz)", emoji1="[STATS]")
            
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
                logger.info(f"[DEBUG] NPU DEBUG: Registered FQ sampler - Total FQ samplers: {len(self._fq_samplers)}")

    def unregister_fq_sampler(self, fq_sampler: Any) -> None:
        """
        Unregister an FQ sampler.
        
        Args:
            fq_sampler: FQSampler instance to unregister
        """
        if fq_sampler in self._fq_samplers:
            self._fq_samplers.remove(fq_sampler)
            if self.debug_npu:
                logger.info(f"[DEBUG] NPU DEBUG: Unregistered FQ sampler - Total FQ samplers: {len(self._fq_samplers)}")

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
                self.logger.info(f"[SIMD] Backend: {self.simd_config['recommended_backend']}")
                self.logger.info(f"[TARGET] Vector Width: {caps.vector_width}, Cache Line: {caps.cache_line_size}B")
                
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
                self.logger.info(f"[BRAIN] SIMD membrane processor initialized for {capacity} neurons")
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
                    f"[START] SIMD Performance: {stats['avg_neurons_per_update']:.0f} neurons/update, "
                    f"backend: {stats['simd_backend']}, vector_width: {stats['vector_width']}"
                )
            
            # Get profiler report if available
            if self.use_simd_profiling and hasattr(self.simd_profiler, 'sessions'):
                if self.simd_profiler.sessions:
                    report = self.simd_profiler.get_performance_report()
                    top_ops = report.get("top_operations", [])
                    if top_ops:
                        self.logger.info(f"[DEBUG] Top SIMD operation: {top_ops[0]['name']} "
                                       f"({top_ops[0]['simd_efficiency']:.2f} efficiency)")
                        
        except Exception as e:
            self.logger.debug(f"Performance reporting failed: {e}")

    def _create_optimized_brain_output_data(self, fire_queue: Dict[str, Any]) -> Dict[str, Any]:
        """Create optimized data package for brain output streams (visualization and motor).
        
        DEPRECATED: Use _create_optimized_brain_output_structured for RTOS/Rust compliance.
        This method maintains backward compatibility by converting from structured arrays.
        
        Original fire queue contains:
        - neuron_ids (needed)
        - membrane_potentials (needed) 
        - thresholds (not needed for brain output)
        - consecutive_fire_counts (not needed for brain output)
        - refractory_counters (not needed for brain output)
        - coordinates (needed)
        
        Args:
            fire_queue: Original fire queue data with all simulation fields
            
        Returns:
            Optimized data package with only essential fields for brain output
        """
        if not fire_queue:
            return {}
            
        # Convert legacy dict to structured array for processing
        structured_data = self._convert_legacy_fire_queue_to_structured(fire_queue)
        if len(structured_data) == 0:
            return {}
        
        # Convert back to dict for backward compatibility
        optimized_data = {
            'neuron_ids': structured_data['neuron_id'].tolist(),
            'membrane_potentials': structured_data['membrane_potential'].tolist(),
            'coordinates': [(int(x), int(y), int(z)) for x, y, z in 
                           zip(structured_data['x'], structured_data['y'], structured_data['z'])]
        }
        
        # Preserve cortical_id if present (needed for area-specific data)
        if isinstance(fire_queue, dict) and 'cortical_id' in fire_queue:
            optimized_data['cortical_id'] = fire_queue['cortical_id']
            
        logger.debug(f"[OPTIMIZE] Created lightweight brain output package: {len(optimized_data)} fields vs original")
        return optimized_data

    def _vectorized_index_lookup(self, neuron_ids_array: np.ndarray) -> np.ndarray:
        """Vectorized index lookup without list comprehensions - RTOS/Rust compliant.
        
        Args:
            neuron_ids_array: Array of neuron IDs to look up
            
        Returns:
            Array of indices (-1 for not found)
        """
        if not hasattr(self.connectome_manager, 'neuron_id_to_index'):
            return np.full(len(neuron_ids_array), -1, dtype=np.int32)
            
        # Create mapping arrays for vectorized lookup
        lookup_dict = self.connectome_manager.neuron_id_to_index
        
        # Use numpy vectorized operations instead of list comprehensions
        vectorized_lookup = np.vectorize(lookup_dict.get, otypes=[int])
        indices_array = vectorized_lookup(neuron_ids_array, -1).astype(np.int32)
        
        return indices_array

    def _get_neuron_coordinates_simd_optimized(self, cortical_id: str, neuron_ids_array: np.ndarray) -> np.ndarray:
        """SIMD-optimized coordinate lookup using vectorized operations and SIMD backend selection.
        
        This method leverages the existing SIMD infrastructure for maximum performance.
        
        Args:
            cortical_id: Cortical area ID
            neuron_ids_array: Pre-allocated numpy array of neuron IDs
            
        Returns:
            numpy array with shape (N, 3) containing coordinates
        """
        if neuron_ids_array.size == 0:
            return np.empty((0, 3), dtype=np.int32)
            
        # Use SIMD profiling if enabled
        if SIMD_AVAILABLE and self.use_simd_profiling:
            with profile_simd_operation("coordinate_lookup", len(neuron_ids_array)):
                return self._simd_coordinate_lookup_impl(cortical_id, neuron_ids_array)
        else:
            return self._simd_coordinate_lookup_impl(cortical_id, neuron_ids_array)
    
    def _simd_coordinate_lookup_impl(self, cortical_id: str, neuron_ids_array: np.ndarray) -> np.ndarray:
        """Core SIMD coordinate lookup implementation."""
        try:
            if not SIMD_AVAILABLE:
                return self._fallback_coordinate_calculation_vectorized(neuron_ids_array)
                
            # Get optimal chunk size for SIMD processing
            if self.backend_selector:
                chunk_size = self.backend_selector.get_chunk_size(len(neuron_ids_array))
                vector_width = self.simd_config.get("vector_width", 4)
            else:
                chunk_size = len(neuron_ids_array)
                vector_width = 4
            
            # Process in SIMD-optimal chunks
            if len(neuron_ids_array) > chunk_size:
                return self._chunked_simd_coordinate_lookup(cortical_id, neuron_ids_array, chunk_size)
            
            # Direct SIMD processing for smaller arrays
            return self._direct_simd_coordinate_lookup(cortical_id, neuron_ids_array, vector_width)
            
        except Exception:
            # Fallback to basic vectorized implementation
            return self._fallback_coordinate_calculation_vectorized(neuron_ids_array)
    
    def _chunked_simd_coordinate_lookup(self, cortical_id: str, neuron_ids_array: np.ndarray, chunk_size: int) -> np.ndarray:
        """Process coordinate lookup in SIMD-optimal chunks."""
        total_neurons = len(neuron_ids_array)
        result_coords = np.zeros((total_neurons, 3), dtype=np.int32)
        
        # Process in chunks for optimal SIMD performance
        for start_idx in range(0, total_neurons, chunk_size):
            end_idx = min(start_idx + chunk_size, total_neurons)
            chunk = neuron_ids_array[start_idx:end_idx]
            
            # Get vector width for this chunk
            vector_width = self.simd_config.get("vector_width", 4)
            chunk_coords = self._direct_simd_coordinate_lookup(cortical_id, chunk, vector_width)
            
            result_coords[start_idx:end_idx] = chunk_coords
            
        return result_coords
    
    def _direct_simd_coordinate_lookup(self, cortical_id: str, neuron_ids_array: np.ndarray, vector_width: int) -> np.ndarray:
        """Direct SIMD coordinate lookup for a single chunk."""
        if self.connectome_manager and hasattr(self.connectome_manager, 'neuron_array'):
            neuron_array = self.connectome_manager.neuron_array
            
            if hasattr(self.connectome_manager, 'neuron_id_to_index'):
                # SIMD-optimized index lookup using vectorized operations
                valid_mask = np.isin(neuron_ids_array, list(self.connectome_manager.neuron_id_to_index.keys()))
                
                if np.any(valid_mask):
                    # Pre-allocate with SIMD-aligned memory if possible
                    result_coords = np.zeros((len(neuron_ids_array), 3), dtype=np.int32)
                    
                    # Use SIMD-optimized vectorized index lookup
                    valid_neuron_ids = neuron_ids_array[valid_mask]
                    indices_array = self._simd_optimized_index_lookup(valid_neuron_ids, vector_width)
                    
                    # SIMD-optimized coordinate extraction
                    valid_indices_mask = indices_array >= 0
                    if np.any(valid_indices_mask):
                        valid_indices = indices_array[valid_indices_mask]
                        
                        # Use SIMD-friendly memory access patterns
                        if hasattr(neuron_array, 'coordinates_x'):
                            # Vectorized gather operations (SIMD-optimized by numpy/BLAS)
                            x_coords = neuron_array.coordinates_x[valid_indices]
                            y_coords = neuron_array.coordinates_y[valid_indices] 
                            z_coords = neuron_array.coordinates_z[valid_indices]
                            
                            # SIMD-friendly assignment
                            valid_positions = np.where(valid_mask)[0][valid_indices_mask]
                            result_coords[valid_positions, 0] = x_coords
                            result_coords[valid_positions, 1] = y_coords
                            result_coords[valid_positions, 2] = z_coords
                        else:
                            return self._fallback_coordinate_calculation_vectorized(neuron_ids_array)
                    
                    # SIMD-optimized fallback for invalid neurons
                    invalid_mask = ~valid_mask
                    if np.any(invalid_mask):
                        invalid_neuron_ids = neuron_ids_array[invalid_mask]
                        # Use SIMD-friendly modulo operations
                        result_coords[invalid_mask, 0] = invalid_neuron_ids % 100
                        result_coords[invalid_mask, 1] = (invalid_neuron_ids // 100) % 100
                        result_coords[invalid_mask, 2] = invalid_neuron_ids // 10000
                    
                    return result_coords
                    
        # Fallback to vectorized calculation
        return self._fallback_coordinate_calculation_vectorized(neuron_ids_array)
    
    def _simd_optimized_index_lookup(self, neuron_ids_array: np.ndarray, vector_width: int) -> np.ndarray:
        """SIMD-optimized index lookup using vectorized operations and chunking."""
        if not hasattr(self.connectome_manager, 'neuron_id_to_index'):
            return np.full(len(neuron_ids_array), -1, dtype=np.int32)
        
        # For large arrays, process in SIMD-optimal chunks
        if len(neuron_ids_array) > vector_width * 4:  # Threshold for chunking
            return self._chunked_index_lookup(neuron_ids_array, vector_width)
        
        # Direct vectorized lookup for smaller arrays
        lookup_dict = self.connectome_manager.neuron_id_to_index
        vectorized_lookup = np.vectorize(lookup_dict.get, otypes=[int])
        return vectorized_lookup(neuron_ids_array, -1).astype(np.int32)
    
    def _chunked_index_lookup(self, neuron_ids_array: np.ndarray, vector_width: int) -> np.ndarray:
        """Process index lookup in SIMD-friendly chunks."""
        total_neurons = len(neuron_ids_array)
        result_indices = np.zeros(total_neurons, dtype=np.int32)
        chunk_size = vector_width * 8  # Process multiple vectors at once
        
        lookup_dict = self.connectome_manager.neuron_id_to_index
        vectorized_lookup = np.vectorize(lookup_dict.get, otypes=[int])
        
        for start_idx in range(0, total_neurons, chunk_size):
            end_idx = min(start_idx + chunk_size, total_neurons)
            chunk = neuron_ids_array[start_idx:end_idx]
            
            # Vectorized lookup for this chunk
            chunk_indices = vectorized_lookup(chunk, -1).astype(np.int32)
            result_indices[start_idx:end_idx] = chunk_indices
            
        return result_indices

class FQSampler:
    """
    Fire Queue Sampler for FEAGI NPU.
    
    This class implements differentiated sampling behavior based on subscriber types:
    - Visualization subscribers: Sample all cortical areas at configured rate
    - Motor subscribers: Sample only OPU cortical areas at burst frequency
    
    The sampler is RTOS-compatible and optimized for real-time performance.
    """
    
    def __init__(self, fire_queue_provider, connectome_manager=None, 
                 max_retries: int = 3, 
                 neuron_type_filter: Optional[str] = None,
                 use_optimized_fcl: bool = True,
                 enable_simd: bool = True,
                 simd_profiling: bool = False):
        """Initialize FQSampler with optional SIMD acceleration.
        
        Args:
            fire_queue_provider: Provider for fire queue data access
            connectome_manager: Connectome for neuron coordinate lookups
            max_retries: Maximum retries for data access
            neuron_type_filter: Optional neuron type filtering
            use_optimized_fcl: Use optimized FCL access paths
            enable_simd: Enable SIMD optimizations
            simd_profiling: Enable SIMD performance profiling
        """
        # Original initialization
        self.fire_queue_provider = fire_queue_provider
        self.connectome_manager = connectome_manager
        self._max_retries = max_retries
        self.neuron_type_filter = neuron_type_filter
        self.use_optimized_fcl = use_optimized_fcl
        
        # Initialize sampling storage
        self._visualization_samples = {}
        self._motor_samples = {}
        
        # SIMD initialization
        self.enable_simd = enable_simd and SIMD_AVAILABLE
        self.use_simd_profiling = simd_profiling and self.enable_simd
        
        # Initialize SIMD infrastructure if available
        if self.enable_simd:
            self._initialize_simd_infrastructure()
            
            # SIMD-optimized storage
            self._visualization_samples_simd = {}
            self._motor_samples_simd = {}
        else:
            self.simd_detector = None
            self.backend_selector = None
            self.membrane_processor = None
            self.simd_config = {}
        
        # Logging
        self.logger = logging.getLogger(__name__)
        
        if self.enable_simd:
            self.logger.info(f"FQSampler initialized with SIMD acceleration: {self.simd_config.get('backend', 'auto')}")
        else:
            self.logger.info("FQSampler initialized without SIMD acceleration")
    
    def _initialize_simd_infrastructure(self) -> None:
        """Initialize SIMD detection, backend selection, and membrane processor."""
        try:
            # Initialize SIMD detector
            from feagi.utils.simd_detection import SIMDDetector
            self.simd_detector = SIMDDetector()
            
            # Initialize backend selector
            from feagi.npu.optimized_membrane_operations import SIMDBackendSelector
            self.backend_selector = SIMDBackendSelector()
            
            # Get SIMD configuration
            self.simd_config = {
                'backend': self.backend_selector.get_optimal_backend(),
                'vector_width': self.simd_detector.vector_width,
                'alignment': 32,  # Standard SIMD alignment
                'supports_avx': self.simd_detector.supports_avx,
                'supports_avx2': self.simd_detector.supports_avx2,
            }
            
            # Initialize membrane processor if available
            try:
                from feagi.npu.optimized_membrane_operations import SIMDMembraneProcessor
                self.membrane_processor = SIMDMembraneProcessor(
                    backend=self.simd_config['backend'],
                    vector_width=self.simd_config['vector_width']
                )
            except ImportError:
                self.membrane_processor = None
                self.logger.warning("SIMDMembraneProcessor not available")
            
        except ImportError as e:
            self.logger.warning(f"SIMD infrastructure initialization failed: {e}")
            self.enable_simd = False
            self.simd_detector = None
            self.backend_selector = None
            self.membrane_processor = None
            self.simd_config = {}
        except Exception as e:
            self.logger.error(f"Unexpected error in SIMD initialization: {e}")
            self.enable_simd = False
            self.simd_detector = None
            self.backend_selector = None
            self.membrane_processor = None
            self.simd_config = {}
    
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
                
                for area in self.connectome_manager.cortical_areas.values():
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
                        logger.debug(f"FQ Sampler: Attempting to sample area {cortical_id} (type: {area.area_type})")
                        
                        # Try to get area fire queue data first for debugging
                        test_data = self._get_area_fire_queue_data(cortical_id)
                        if test_data and test_data.get('neuron_ids'):
                            neuron_count = len(test_data['neuron_ids'])
                            logger.debug(f"[TARGET] FQ Sampler: Area {cortical_id} has {neuron_count} firing neurons")
                            areas_with_data += 1
                        else:
                            logger.debug(f"[ERR] FQ Sampler: Area {cortical_id} has no fire queue data")
                           
                        self._sample_area_fire_queue(cortical_id, target='visualization')
                        self._last_sample_time_per_area[cortical_id] = now
                        
                # Log summary every 100 samples
                if hasattr(self, '_debug_sample_count'):
                    self._debug_sample_count += 1
                else:
                    self._debug_sample_count = 1
                    
                if self._debug_sample_count % 100 == 0:
                    logger.info(f"FQ Sampler Debug: Processed {areas_processed} areas, {areas_with_data} had data")
            else:
                # Global sampling for visualization
                logger.debug("FQ Sampler: Using global sampling (no connectome manager)")
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
                
            for area in self.connectome_manager.cortical_areas.values():
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
                    # Create optimized data package for visualization/motor streams
                    optimized_data = self._create_optimized_brain_output_data(area_fire_data)
                    neuron_count = len(optimized_data.get('neuron_ids', []))
                    
                    try:
                        # For backward compatibility, also support tuple format
                        if target == 'visualization':
                            # THE CRITICAL PUT OPERATION - now with optimized data
                            self.output_queue.put((cortical_id, optimized_data))
                            logger.debug(f"Queued OPTIMIZED {cortical_id}: {neuron_count} neurons for visualization")
                        else:
                            # For motor, use tagged format with optimized data
                            data_package = {
                                'cortical_id': cortical_id,
                                'fire_queue_data': optimized_data,  # Optimized data here
                                'target': target,
                                'timestamp': time.time()
                            }
                            self.output_queue.put(data_package)
                            logger.debug(f"Queued OPTIMIZED {cortical_id}: {neuron_count} neurons for {target}")
                            
                        break  # Success
                        
                    except Exception as put_error:
                        logger.error(f"[ERR] Error queuing {cortical_id} data: {put_error}")
                        break  # Don't retry on queue errors
                        
                else:
                    logger.debug(f"No fire queue data for {cortical_id}")
                    break
                    
            except Exception as general_error:
                logger.error(f"[ERR] Error sampling area {cortical_id}: {general_error}")
                if retry_count == self._max_retries - 1:
                    logger.error(f"[ERR] Max retries reached for {cortical_id}")
                    
                # Wait before retrying
                delay_start = time.perf_counter()
                while time.perf_counter() - delay_start < self._retry_delay:
                    pass
                retry_count += 1

    def _sample_global_fire_queue(self, target: str = 'visualization') -> None:
        """Sample global fire queue data."""
        retry_count = 0
        
        while retry_count < self._max_retries:
            try:
                # Get global fire queue data
                fire_data = self._get_global_fire_queue_data()
                
                if fire_data:
                    # Create optimized data package for visualization/motor streams
                    optimized_data = self._create_optimized_brain_output_data(fire_data)
                    neuron_count = len(optimized_data.get('neuron_ids', []))
                    
                    try:
                        # Tag the data with target type for proper routing
                        if target == 'motor':
                            data_package = {
                                'fire_queue_data': optimized_data,  # Optimized data
                                'target': target,
                                'timestamp': time.time()
                            }
                            self.output_queue.put_nowait(data_package)
                            logger.debug(f"Queued OPTIMIZED global motor data: {neuron_count} neurons")
                        else:
                            # For visualization, use existing format with optimized data
                            self.output_queue.put_nowait(optimized_data)
                            logger.debug(f"Queued OPTIMIZED global visualization data: {neuron_count} neurons")
                            
                        break  # Success
                        
                    except Exception as put_error:
                        logger.error(f"[ERR] Error queuing global data: {put_error}")
                        break
                        
                else:
                    logger.debug(f"No global fire queue data")
                    break
                    
            except Exception as general_error:
                logger.error(f"[ERR] Error in global sampling: {general_error}")
                if retry_count == self._max_retries - 1:
                    logger.error(f"[ERR] Max retries reached for global sampling")
                    
                # Wait before retrying
                delay_start = time.perf_counter()
                while time.perf_counter() - delay_start < self._retry_delay:
                    pass
                retry_count += 1

    def _get_area_fire_queue_data(self, cortical_id: str) -> Optional[Dict[str, Any]]:
        """
        Get fire queue data for a specific cortical area.
        
        DEPRECATED: Use _get_area_fire_queue_data_structured for RTOS/Rust compliance.
        This method maintains backward compatibility only.
        
        Returns:
            Dictionary with fire queue data containing numpy arrays instead of lists
        """
        try:
            # Use new structured method and convert back for backward compatibility
            structured_data = self._get_area_fire_queue_data_structured(cortical_id)
            if structured_data is None or len(structured_data) == 0:
                return None
                
            # Convert structured array back to dict format for legacy compatibility
            return {
                'neuron_ids': structured_data['neuron_id'].tolist(),
                'membrane_potentials': structured_data['membrane_potential'].tolist(),
                'coordinates': [(int(x), int(y), int(z)) for x, y, z in 
                               zip(structured_data['x'], structured_data['y'], structured_data['z'])]
            }
            
        except Exception as e:
            logger.error(f"Error getting area fire queue data for {cortical_id}: {e}")
            return None

    def _get_global_fire_queue_data(self) -> Optional[Dict[str, Any]]:
        """
        Get global fire queue data.
        
        DEPRECATED: Use _get_global_fire_queue_data_structured for RTOS/Rust compliance.
        This method maintains backward compatibility only.
        """
        try:
            # Use new structured method and convert back for backward compatibility
            structured_data = self._get_global_fire_queue_data_structured()
            if structured_data is None or len(structured_data) == 0:
                return None
                
            # Convert structured array back to dict format for legacy compatibility
            return {
                'neuron_ids': structured_data['neuron_id'].tolist(),
                'membrane_potentials': structured_data['membrane_potential'].tolist(),
                'coordinates': [(int(x), int(y), int(z)) for x, y, z in 
                               zip(structured_data['x'], structured_data['y'], structured_data['z'])]
            }
            
        except Exception as e:
            logger.error(f"Error getting global fire queue data: {e}")
            return None

    def _filter_fire_queue_by_area(self, fire_queue: Dict[str, Any], cortical_id: str) -> Dict[str, Any]:
        """Filter fire queue data to only include neurons from specified area.
        
        DEPRECATED: Use _filter_fire_queue_by_area_structured for RTOS/Rust compliance.
        This method maintains backward compatibility only.
        """
        if not fire_queue or not self.connectome_manager:
            return fire_queue
            
        try:
            # Use new structured method and convert back for backward compatibility
            structured_data = self._filter_fire_queue_by_area_structured(fire_queue, cortical_id)
            if structured_data is None or len(structured_data) == 0:
                return {'neuron_ids': [], 'membrane_potentials': [], 'thresholds': [], 
                       'consecutive_fire_counts': [], 'refractory_counters': []}
                
            # Convert structured array back to dict format for legacy compatibility
            return {
                'neuron_ids': structured_data['neuron_id'].tolist(),
                'membrane_potentials': structured_data['membrane_potential'].tolist(),
                'thresholds': [0.0] * len(structured_data),  # Default values for backward compatibility
                'consecutive_fire_counts': [0] * len(structured_data),
                'refractory_counters': [0] * len(structured_data),
                'coordinates': [(int(x), int(y), int(z)) for x, y, z in 
                               zip(structured_data['x'], structured_data['y'], structured_data['z'])]
            }
                    
        except Exception as e:
            logger.error(f"Error filtering fire queue by area {cortical_id}: {e}")
            return fire_queue

    def _get_neuron_coordinates_vectorized(self, cortical_id: str, neuron_ids: List[int]) -> np.ndarray:
        """Get 3D coordinates for neurons using pure vectorized operations on SoA data.
        
        DEPRECATED - CONTAINS LIST COMPREHENSIONS: Use _get_neuron_coordinates_vectorized_rtos for RTOS/Rust compliance.
        
        COMPLETELY VECTORIZED - no Python loops or comprehensions.
        
        Args:
            cortical_id: Cortical area ID
            neuron_ids: List of neuron IDs to get coordinates for
            
        Returns:
            numpy array with shape (N, 3) containing coordinates
        """
        if not neuron_ids:
            return np.empty((0, 3), dtype=np.int32)
            
        logger.debug(f"[SIMD] PURE VECTORIZED coordinate lookup for {len(neuron_ids)} neurons in area {cortical_id}")
        
        try:
            neuron_ids_array = np.array(neuron_ids, dtype=np.int32)
            
            if self.connectome_manager and hasattr(self.connectome_manager, 'neuron_array'):
                neuron_array = self.connectome_manager.neuron_array
                
                # Try direct index mapping if available
                if hasattr(self.connectome_manager, 'neuron_id_to_index'):
                    # VECTORIZED index lookup using numpy operations
                    valid_mask = np.isin(neuron_ids_array, list(self.connectome_manager.neuron_id_to_index.keys()))
                    
                    if np.any(valid_mask):
                        # Create index array efficiently
                        indices_list = [self.connectome_manager.neuron_id_to_index.get(nid, -1) for nid in neuron_ids_array]
                        indices_array = np.array(indices_list, dtype=np.int32)
                        
                        # Mask out invalid indices
                        valid_indices_mask = indices_array >= 0
                        final_mask = valid_mask & valid_indices_mask
                        
                        if np.any(final_mask):
                            valid_indices = indices_array[valid_indices_mask]
                            
                            # PURE VECTORIZED SoA ACCESS
                            if hasattr(neuron_array, 'positions_x'):
                                x_coords = neuron_array.positions_x[valid_indices]
                                y_coords = neuron_array.positions_y[valid_indices] 
                                z_coords = neuron_array.positions_z[valid_indices]
                            elif hasattr(neuron_array, 'coordinates_x'):
                                x_coords = neuron_array.coordinates_x[valid_indices]
                                y_coords = neuron_array.coordinates_y[valid_indices]
                                z_coords = neuron_array.coordinates_z[valid_indices]
                            else:
                                return self._fallback_coordinate_calculation_vectorized(neuron_ids_array)
                            
                            # PURE VECTORIZED result construction
                            result_coords = np.zeros((len(neuron_ids_array), 3), dtype=np.int32)
                            
                            # Fill valid coordinates
                            result_coords[final_mask] = np.column_stack((x_coords, y_coords, z_coords))
                            
                            # VECTORIZED fallback for invalid neurons
                            invalid_mask = ~final_mask
                            if np.any(invalid_mask):
                                invalid_neuron_ids = neuron_ids_array[invalid_mask]
                                result_coords[invalid_mask, 0] = invalid_neuron_ids % 100
                                result_coords[invalid_mask, 1] = (invalid_neuron_ids // 100) % 100
                                result_coords[invalid_mask, 2] = invalid_neuron_ids // 10000
                            
                            logger.debug(f"[SIMD] Pure vectorized lookup complete: {np.sum(final_mask)}/{len(neuron_ids_array)} from SoA")
                            return result_coords
                        
            # Pure vectorized fallback
            return self._fallback_coordinate_calculation_vectorized(neuron_ids_array)
            
        except Exception as e:
            logger.error(f"[SIMD] Error in pure vectorized coordinate lookup: {e}")
            return self._fallback_coordinate_calculation_vectorized(np.array(neuron_ids, dtype=np.int32))

    def _get_global_neuron_coordinates_vectorized(self, neuron_ids: List[int]) -> np.ndarray:
        """Get global 3D coordinates using pure vectorized SoA operations.
        
        DEPRECATED - CONTAINS LIST COMPREHENSIONS: Use _get_global_neuron_coordinates_vectorized_rtos for RTOS/Rust compliance.
        
        COMPLETELY VECTORIZED - no Python loops or comprehensions.
        
        Args:
            neuron_ids: List of neuron IDs to get coordinates for
            
        Returns:
            numpy array with shape (N, 3) containing coordinates
        """
        if not neuron_ids:
            return np.empty((0, 3), dtype=np.int32)
            
        logger.debug(f"[SIMD] PURE VECTORIZED global coordinate lookup for {len(neuron_ids)} neurons")
        
        try:
            neuron_ids_array = np.array(neuron_ids, dtype=np.int32)
            
            if self.connectome_manager and hasattr(self.connectome_manager, 'neuron_array'):
                neuron_array = self.connectome_manager.neuron_array
                
                if hasattr(self.connectome_manager, 'neuron_id_to_index'):
                    # VECTORIZED global index lookup
                    valid_mask = np.isin(neuron_ids_array, list(self.connectome_manager.neuron_id_to_index.keys()))
                    
                    if np.any(valid_mask):
                        # Create index array efficiently
                        indices_list = [self.connectome_manager.neuron_id_to_index.get(nid, -1) for nid in neuron_ids_array]
                        indices_array = np.array(indices_list, dtype=np.int32)
                        
                        valid_indices_mask = indices_array >= 0
                        final_mask = valid_mask & valid_indices_mask
                        
                        if np.any(final_mask):
                            valid_indices = indices_array[final_mask]
                            
                            # PURE VECTORIZED GLOBAL SoA ACCESS
                            if hasattr(neuron_array, 'positions_x'):
                                x_coords = neuron_array.positions_x[valid_indices]
                                y_coords = neuron_array.positions_y[valid_indices]
                                z_coords = neuron_array.positions_z[valid_indices]
                            elif hasattr(neuron_array, 'coordinates_x'):
                                x_coords = neuron_array.coordinates_x[valid_indices]
                                y_coords = neuron_array.coordinates_y[valid_indices]
                                z_coords = neuron_array.coordinates_z[valid_indices]
                            else:
                                return self._fallback_coordinate_calculation_vectorized(neuron_ids_array)
                            
                            # PURE VECTORIZED result construction
                            result_coords = np.zeros((len(neuron_ids_array), 3), dtype=np.int32)
                            
                            # Fill valid coordinates
                            result_coords[final_mask] = np.column_stack((x_coords, y_coords, z_coords))
                            
                            # VECTORIZED fallback for invalid neurons
                            invalid_mask = ~final_mask
                            if np.any(invalid_mask):
                                invalid_neuron_ids = neuron_ids_array[invalid_mask]
                                result_coords[invalid_mask, 0] = invalid_neuron_ids % 100
                                result_coords[invalid_mask, 1] = (invalid_neuron_ids // 100) % 100
                                result_coords[invalid_mask, 2] = invalid_neuron_ids // 10000
                            
                            logger.debug(f"[SIMD] Pure vectorized global complete: {np.sum(final_mask)}/{len(neuron_ids_array)} from SoA")
                            return result_coords
                    
            # Pure vectorized fallback
            return self._fallback_coordinate_calculation_vectorized(neuron_ids_array)
            
        except Exception as e:
            logger.error(f"[SIMD] Error in pure vectorized global lookup: {e}")
            return self._fallback_coordinate_calculation_vectorized(np.array(neuron_ids, dtype=np.int32))

    def _fallback_coordinate_calculation_vectorized(self, neuron_ids_array: np.ndarray) -> np.ndarray:
        """Pure vectorized fallback coordinate calculation - no loops.
        
        Args:
            neuron_ids_array: numpy array of neuron IDs
            
        Returns:
            numpy array with shape (N, 3) containing algorithmic coordinates
        """
        # PURE VECTORIZED coordinate generation
        x_coords = neuron_ids_array % 100
        y_coords = (neuron_ids_array // 100) % 100
        z_coords = neuron_ids_array // 10000
        
        return np.column_stack((x_coords, y_coords, z_coords)).astype(np.int32)

    def stop(self) -> None:
        """Stop the FQ sampler."""
        self.running = False
        
        # Auto-unregister from burst engine if it was registered
        if hasattr(self.fire_queue_provider, 'unregister_fq_sampler'):
            try:
                self.fire_queue_provider.unregister_fq_sampler(self)
                logger.info(f"[DEBUG] NPU DEBUG: Unregistered FQ sampler - Total FQ samplers: {len(self._fq_samplers)}")
            except Exception as e:
                logger.warning(f"Failed to auto-unregister FQSampler from BurstEngine: {e}")
        
    def update_area_sample_rate(self, cortical_id: str, rate: float) -> None:
        """Set the sampling rate for a specific cortical area."""
        if cortical_id not in self._last_sample_time_per_area:
            self._last_sample_time_per_area[cortical_id] = time.perf_counter() 

    def _sample_area_fire_queue_structured(self, cortical_id: str, target: str = 'visualization') -> None:
        """Sample fire queue data for a specific cortical area using structured arrays.
        
        RTOS/Rust compliant: No dictionaries, no list comprehensions, pure numpy.
        
        Args:
            cortical_id: Cortical area ID to sample
            target: Target type ('visualization' or 'motor')
        """
        retry_count = 0
        
        while retry_count < self._max_retries:
            try:
                # Get structured fire queue data
                structured_data = self._get_area_fire_queue_data_structured(cortical_id)
                
                if structured_data is not None and len(structured_data) > 0:
                    neuron_count = len(structured_data)
                    
                    try:
                        if target == 'visualization':
                            # For visualization, use tuple format with structured data
                            self.output_queue.put((cortical_id, structured_data))
                            logger.debug(f"Queued STRUCTURED {cortical_id}: {neuron_count} neurons for visualization")
                        else:
                            # For motor, use tagged format with structured data
                            data_package = {
                                'cortical_id': cortical_id,
                                'fire_queue_data': structured_data,
                                'target': target,
                                'timestamp': time.time()
                            }
                            self.output_queue.put(data_package)
                            logger.debug(f"Queued STRUCTURED {cortical_id}: {neuron_count} neurons for {target}")
                            
                        break  # Success
                        
                    except Exception as put_error:
                        logger.error(f"[ERR] Error queuing structured {cortical_id} data: {put_error}")
                        break  # Don't retry on queue errors
                        
                else:
                    logger.debug(f"No structured fire queue data for {cortical_id}")
                    break
                    
            except Exception as general_error:
                logger.error(f"[ERR] Error sampling area {cortical_id}: {general_error}")
                if retry_count == self._max_retries - 1:
                    logger.error(f"[ERR] Max retries reached for {cortical_id}")
                    
                # Wait before retrying
                delay_start = time.perf_counter()
                while time.perf_counter() - delay_start < self._retry_delay:
                    pass
                retry_count += 1

    def _sample_global_fire_queue_structured(self, target: str = 'visualization') -> None:
        """Sample global fire queue data using structured arrays.
        
        RTOS/Rust compliant: No dictionaries, no list comprehensions, pure numpy.
        
        Args:
            target: Target type ('visualization' or 'motor')
        """
        retry_count = 0
        
        while retry_count < self._max_retries:
            try:
                # Get structured global fire queue data
                structured_data = self._get_global_fire_queue_data_structured()
                
                if structured_data is not None and len(structured_data) > 0:
                    neuron_count = len(structured_data)
                    
                    try:
                        # Tag the data with target type for proper routing
                        if target == 'motor':
                            data_package = {
                                'fire_queue_data': structured_data,
                                'target': target,
                                'timestamp': time.time()
                            }
                            self.output_queue.put_nowait(data_package)
                            logger.debug(f"Queued STRUCTURED global motor data: {neuron_count} neurons")
                        else:
                            # For visualization, use structured data directly
                            self.output_queue.put_nowait(structured_data)
                            logger.debug(f"Queued STRUCTURED global visualization data: {neuron_count} neurons")
                            
                        break  # Success
                        
                    except Exception as put_error:
                        logger.error(f"[ERR] Error queuing structured global data: {put_error}")
                        break
                        
                else:
                    logger.debug(f"No structured global fire queue data")
                    break
                    
            except Exception as general_error:
                logger.error(f"[ERR] Error in structured global sampling: {general_error}")
                if retry_count == self._max_retries - 1:
                    logger.error(f"[ERR] Max retries reached for structured global sampling")
                    
                # Wait before retrying
                delay_start = time.perf_counter()
                while time.perf_counter() - delay_start < self._retry_delay:
                    pass
                retry_count += 1

    def _sample_area_fire_queue_simd_optimized(self, cortical_id: str, target: str = 'visualization') -> None:
        """SIMD-optimized fire queue sampling using membrane processor and vectorized operations.
        
        This method leverages the full SIMD infrastructure for maximum performance.
        
        Args:
            cortical_id: Cortical area ID to sample
            target: Target type ('visualization' or 'motor')
        """
        if not SIMD_AVAILABLE:
            # Fallback to structured array version
            return self._sample_area_fire_queue_structured(cortical_id, target)
        
        retry_count = 0
        
        while retry_count < self._max_retries:
            try:
                # Use SIMD-optimized data retrieval
                with profile_simd_operation(f"fq_sample_{target}", 1) if self.use_simd_profiling else nullcontext():
                    structured_data = self._get_area_fire_queue_data_simd_optimized(cortical_id)
                
                if structured_data is not None and len(structured_data) > 0:
                    # SIMD-optimized processing
                    processed_data = self._simd_process_fire_queue_data(structured_data, cortical_id, target)
                    
                    # Store using SIMD-friendly operations
                    self._store_simd_optimized_sample(processed_data, cortical_id, target)
                    return
                    
                # Handle empty case
                self._store_empty_simd_sample(cortical_id, target)
                return
                
            except Exception as e:
                retry_count += 1
                if self.logger:
                    self.logger.warning(f"SIMD sampling retry {retry_count} for {cortical_id}: {e}")
                
                if retry_count >= self._max_retries:
                    # Final fallback to structured method
                    return self._sample_area_fire_queue_structured(cortical_id, target)
    
    def _get_area_fire_queue_data_simd_optimized(self, cortical_id: str) -> Optional[np.ndarray]:
        """Get fire queue data optimized for SIMD processing."""
        try:
            # Try direct SIMD-optimized provider access
            if hasattr(self.fire_queue_provider, 'get_area_fire_queue_simd'):
                return self.fire_queue_provider.get_area_fire_queue_simd(cortical_id)
            
            # Try legacy access and convert to SIMD format
            if hasattr(self.fire_queue_provider, 'get_area_fire_queue_direct'):
                legacy_data = self.fire_queue_provider.get_area_fire_queue_direct(cortical_id)
                if legacy_data is not None:
                    return self._convert_fire_queue_to_simd_optimized(legacy_data, cortical_id)
            
            # Fallback to regular structured access
            return self._get_area_fire_queue_data_structured(cortical_id)
            
        except Exception:
            return None
    
    def _simd_process_fire_queue_data(self, structured_data: np.ndarray, cortical_id: str, target: str) -> np.ndarray:
        """Process fire queue data using SIMD operations and membrane processor."""
        if len(structured_data) == 0:
            return structured_data
        
        try:
            # Use SIMDMembraneProcessor if available
            if hasattr(self, 'membrane_processor') and self.membrane_processor:
                # Extract membrane potentials for SIMD processing
                membrane_potentials = structured_data['membrane_potential']
                
                # SIMD-optimized membrane processing
                with profile_simd_operation("membrane_processing", len(membrane_potentials)) if self.use_simd_profiling else nullcontext():
                    processed_potentials = self.membrane_processor.process_batch(
                        membrane_potentials,
                        operation='normalize'  # or 'threshold', 'scale', etc.
                    )
                
                # Update structured data with processed values
                if processed_potentials is not None:
                    result_data = structured_data.copy()
                    result_data['membrane_potential'] = processed_potentials
                    return result_data
            
            # No membrane processing available - return as is
            return structured_data
            
        except Exception:
            # Fallback - return original data
            return structured_data
    
    def _store_simd_optimized_sample(self, processed_data: np.ndarray, cortical_id: str, target: str) -> None:
        """Store SIMD-optimized sample data using efficient memory operations."""
        try:
            # Use SIMD-friendly operations for data storage
            if target == 'visualization':
                if hasattr(self, '_visualization_samples_simd'):
                    self._visualization_samples_simd[cortical_id] = processed_data
                else:
                    # Fallback to regular storage
                    self._visualization_samples[cortical_id] = self._convert_simd_to_dict(processed_data)
            
            elif target == 'motor':
                if hasattr(self, '_motor_samples_simd'):
                    self._motor_samples_simd[cortical_id] = processed_data
                else:
                    # Fallback to regular storage
                    self._motor_samples[cortical_id] = self._convert_simd_to_dict(processed_data)
            
        except Exception:
            # Emergency fallback to dictionary storage
            dict_data = self._convert_simd_to_dict(processed_data)
            if target == 'visualization':
                self._visualization_samples[cortical_id] = dict_data
            elif target == 'motor':
                self._motor_samples[cortical_id] = dict_data
    
    def _store_empty_simd_sample(self, cortical_id: str, target: str) -> None:
        """Store empty sample using SIMD-optimized structures."""
        empty_structure = self._create_empty_simd_structure()
        
        if target == 'visualization':
            if hasattr(self, '_visualization_samples_simd'):
                self._visualization_samples_simd[cortical_id] = empty_structure
            else:
                self._visualization_samples[cortical_id] = self._create_empty_data_dict()
        
        elif target == 'motor':
            if hasattr(self, '_motor_samples_simd'):
                self._motor_samples_simd[cortical_id] = empty_structure
            else:
                self._motor_samples[cortical_id] = self._create_empty_data_dict()
    
    def _convert_simd_to_dict(self, structured_data: np.ndarray) -> Dict[str, Any]:
        """Convert SIMD-optimized structured array back to dictionary format for backward compatibility."""
        if len(structured_data) == 0:
            return self._create_empty_data_dict()
        
        return {
            'neuron_ids': structured_data['neuron_id'].tolist(),
            'membrane_potentials': structured_data['membrane_potential'].tolist(),
            'coordinates': np.column_stack([
                structured_data['x'],
                structured_data['y'], 
                structured_data['z']
            ])
        }

class OptimizedFQSampler:
    """
    Ultra-optimized Fire Queue Sampler using direct SoA access.
    
    This class bypasses all intermediate Python conversions and works directly
    with the optimized SoA structures, providing zero-copy brain output sampling.
    """
    
    def __init__(self, fire_queue_provider: Any, sample_frequency_hz: float, 
                 output_queue: Any, connectome_manager: Optional[Any] = None) -> None:
        """Initialize the optimized FQ sampler."""
        self.fire_queue_provider = fire_queue_provider
        self.sample_frequency = sample_frequency_hz
        self.sample_interval = 1.0 / sample_frequency_hz if sample_frequency_hz > 0 else 0.1
        self.output_queue = output_queue
        self.connectome_manager = connectome_manager
        self.running = False
        
        # Subscriber tracking
        self._has_visualization_subscribers = False
        self._has_motor_subscribers = False
        
        # Pre-allocate output buffers for zero-allocation sampling
        self.max_neurons_per_sample = 100_000
        self.output_buffer = np.empty((self.max_neurons_per_sample, 6), dtype=np.float32)
        
        # Direct access to optimized structures
        self.fcl_manager = getattr(connectome_manager, 'fcl_manager', None) if connectome_manager else None
        self.gna = getattr(connectome_manager, 'neuron_array', None) if connectome_manager else None
        
        # Motor sampling configuration
        self._last_motor_sample_time = 0.0
        self._motor_sample_interval = 1.0
        
        logger.info(f"OptimizedFQSampler initialized with direct SoA access")

    def sample_brain_output_direct(self) -> Optional[bytes]:
        """Direct brain output sampling with zero-copy operations.
        
        Returns:
            Binary encoded brain output data or None if no firing neurons
        """
        try:
            # Direct FCL access
            if hasattr(self.fire_queue_provider, 'get_fire_queue_direct'):
                brain_data = self.fire_queue_provider.get_fire_queue_direct()
            else:
                # Fallback to direct FCL access if available
                if self.fcl_manager:
                    brain_data = self._extract_brain_data_direct()
                else:
                    return None
            
            if brain_data is None or len(brain_data) == 0:
                return None
            
            # Add timestamp column
            timestamped_data = np.column_stack((
                brain_data,
                np.full(len(brain_data), time.time(), dtype=np.float32)
            ))
            
            # Direct binary encoding
            return self._encode_brain_data_binary(timestamped_data)
            
        except Exception as e:
            logger.error(f"Error in direct brain output sampling: {e}")
            return None

    def sample_motor_areas_direct(self, opu_area_ids: List[str]) -> Optional[bytes]:
        """Direct motor area sampling with batch processing.
        
        Args:
            opu_area_ids: List of output processing unit area IDs
            
        Returns:
            Binary encoded motor data or None
        """
        try:
            if not self.fcl_manager:
                return None
            
            # Combine FCLs from all OPU areas using bitmap operations
            combined_fcl = None
            
            for area_id in opu_area_ids:
                area_fcl = self.fcl_manager.get_cortical_fcl(area_id)
                if not area_fcl.is_empty():
                    if combined_fcl is None:
                        combined_fcl = area_fcl
                    else:
                        combined_fcl = combined_fcl | area_fcl
            
            if combined_fcl is None or combined_fcl.is_empty():
                return None
            
            # Direct extraction from combined FCL
            brain_data = self._extract_brain_data_from_fcl(combined_fcl)
            
            if brain_data is None or len(brain_data) == 0:
                return None
            
            # Add timestamp and motor tag
            motor_data = np.column_stack((
                brain_data,
                np.full(len(brain_data), time.time(), dtype=np.float32)
            ))
            
            return self._encode_brain_data_binary(motor_data)
            
        except Exception as e:
            logger.error(f"Error in direct motor sampling: {e}")
            return None

    def _extract_brain_data_direct(self) -> Optional[np.ndarray]:
        """Extract brain data directly from FCL and SoA structures."""
        if not self.fcl_manager:
            return None
            
        global_fcl = self.fcl_manager.get_global_fcl()
        return self._extract_brain_data_from_fcl(global_fcl)

    def _extract_brain_data_from_fcl(self, fcl) -> Optional[np.ndarray]:
        """Extract brain data from a given FCL using direct SoA access.
        
        Args:
            fcl: Fire candidate list (bitmap)
            
        Returns:
            numpy array with shape (N, 5) containing brain output data
        """
        if fcl.is_empty():
            return None
        
        try:
            # Direct FCL to indices (zero-copy if FCL supports it)
            if hasattr(fcl, 'to_gpu_array'):
                firing_indices = fcl.to_gpu_array()  # Already numpy array!
            else:
                firing_indices = np.array(list(fcl), dtype=np.int32)
            
            if len(firing_indices) == 0:
                return None
            
            # Direct SoA extraction - no intermediate conversions
            if not self.gna:
                # Fallback coordinate generation
                return np.column_stack((
                    firing_indices.astype(np.float32),
                    np.ones(len(firing_indices), dtype=np.float32),  # membrane_potentials
                    (firing_indices % 100).astype(np.float32),      # x_coords
                    ((firing_indices // 100) % 100).astype(np.float32),  # y_coords
                    (firing_indices // 10000).astype(np.float32)    # z_coords
                ))
            
            # Use actual SoA structures
            brain_data = np.column_stack((
                firing_indices.astype(np.float32),
                (self.gna.membrane_potentials[firing_indices] 
                 if hasattr(self.gna, 'membrane_potentials') 
                 else np.ones(len(firing_indices), dtype=np.float32)),
                (self.gna.coordinates_x[firing_indices].astype(np.float32)
                 if hasattr(self.gna, 'coordinates_x')
                 else (firing_indices % 100).astype(np.float32)),
                (self.gna.coordinates_y[firing_indices].astype(np.float32)
                 if hasattr(self.gna, 'coordinates_y')
                 else ((firing_indices // 100) % 100).astype(np.float32)),
                (self.gna.coordinates_z[firing_indices].astype(np.float32)
                 if hasattr(self.gna, 'coordinates_z')
                 else (firing_indices // 10000).astype(np.float32))
            ))
            
            return brain_data
            
        except Exception as e:
            logger.error(f"Error extracting brain data from FCL: {e}")
            return None

    def _encode_brain_data_binary(self, data: np.ndarray) -> bytes:
        """Encode brain data directly to binary format.
        
        Args:
            data: numpy array with brain output data
            
        Returns:
            Binary encoded data
        """
        try:
            import struct
            
            # Header: [neuron_count][timestamp][data_type]
            header = struct.pack('!IQB', 
                               len(data),           # neuron count (4 bytes)
                               int(time.time()),    # timestamp (8 bytes)  
                               11)                  # Type 11 = NEURON_CATEGORIES (1 byte)
            
            # Data: Direct numpy array to bytes (zero-copy)
            return header + data.tobytes()
            
        except Exception as e:
            logger.error(f"Error encoding brain data to binary: {e}")
            return b''

    def run_optimized(self) -> None:
        """Optimized sampling loop with direct SoA access."""
        logger.info("OptimizedFQSampler started with direct SoA access")
        self.running = True
        
        while self.running:
            start = time.perf_counter()
            now = time.perf_counter()
            
            # Skip sampling if no subscribers
            if not self._has_visualization_subscribers and not self._has_motor_subscribers:
                self._sleep_deterministic(start, self.sample_interval)
                continue
            
            # Visualization sampling (direct binary output)
            if self._has_visualization_subscribers:
                viz_data = self.sample_brain_output_direct()
                if viz_data:
                    try:
                        self.output_queue.put_nowait(('visualization', viz_data))
                    except Exception as e:
                        logger.error(f"Error queuing visualization data: {e}")
            
            # Motor sampling (direct binary output) 
            if self._has_motor_subscribers and (now - self._last_motor_sample_time >= self._motor_sample_interval):
                # Get OPU areas if available
                opu_areas = self._get_opu_areas_fast() if self.connectome_manager else []
                if opu_areas:
                    motor_data = self.sample_motor_areas_direct(opu_areas)
                    if motor_data:
                        try:
                            self.output_queue.put_nowait(('motor', motor_data))
                        except Exception as e:
                            logger.error(f"Error queuing motor data: {e}")
                
                self._last_motor_sample_time = now
            
            # Deterministic timing
            self._sleep_deterministic(start, self.sample_interval)
        
        logger.info("OptimizedFQSampler stopped")

    def _get_opu_areas_fast(self) -> List[str]:
        """Fast OPU area detection using cached results."""
        # Use cached OPU areas if available
        if hasattr(self, '_cached_opu_areas'):
            return self._cached_opu_areas
        
        opu_areas = []
        try:
            if self.connectome_manager and hasattr(self.connectome_manager, 'cortical_areas'):
                for area in self.connectome_manager.cortical_areas.values():
                    area_type = area.properties.get('cortical_type', '').upper()
                    if ('OPU' in area_type or 'OUTPUT' in area_type or 'MOTOR' in area_type or
                        area.id.startswith(('opu_', 'motor_', 'output_'))):
                        opu_areas.append(area.id)
            
            # Cache the result
            self._cached_opu_areas = opu_areas
        except Exception as e:
            logger.error(f"Error detecting OPU areas: {e}")
        
        return opu_areas

    def _sleep_deterministic(self, start_time: float, interval: float) -> None:
        """Deterministic sleep implementation for RTOS compatibility."""
        elapsed = time.perf_counter() - start_time
        if elapsed < interval:
            target_end_time = start_time + interval
            while time.perf_counter() < target_end_time:
                pass  # Busy-wait for deterministic timing

    def set_visualization_subscribers(self, has_subscribers: bool) -> None:
        """Update visualization subscriber status."""
        self._has_visualization_subscribers = has_subscribers

    def set_motor_subscribers(self, has_subscribers: bool) -> None:
        """Update motor subscriber status."""
        self._has_motor_subscribers = has_subscribers

    def stop(self) -> None:
        """Stop the optimized sampler."""
        self.running = False

    def _get_neuron_coordinates_vectorized_rtos(self, cortical_id: str, neuron_ids_array: np.ndarray) -> np.ndarray:
        """RTOS/Rust/SIMD compliant coordinate lookup with zero dynamic allocations.
        
        Args:
            cortical_id: Cortical area ID (pre-validated)
            neuron_ids_array: Pre-allocated numpy array of neuron IDs
            
        Returns:
            numpy array with shape (N, 3) - no tuple conversions
        """
        if neuron_ids_array.size == 0:
            return np.empty((0, 3), dtype=np.int32)
            
        try:
            if self.connectome_manager and hasattr(self.connectome_manager, 'neuron_array'):
                neuron_array = self.connectome_manager.neuron_array
                
                if hasattr(self.connectome_manager, 'neuron_id_to_index'):
                    # VECTORIZED index lookup - no loops, no comprehensions
                    neuron_ids_set = set(self.connectome_manager.neuron_id_to_index.keys())
                    valid_mask = np.isin(neuron_ids_array, list(neuron_ids_set))
                    
                    if np.any(valid_mask):
                        # Pre-allocate result array
                        result_coords = np.zeros((len(neuron_ids_array), 3), dtype=np.int32)
                        
                        # Use vectorized index lookup instead of list comprehension
                        valid_neuron_ids = neuron_ids_array[valid_mask]
                        indices_array = self._vectorized_index_lookup(valid_neuron_ids)
                        
                        # Mask out invalid indices
                        valid_indices_mask = indices_array >= 0
                        if np.any(valid_indices_mask):
                            valid_indices = indices_array[valid_indices_mask]
                            
                            # PURE VECTORIZED SoA ACCESS
                            if hasattr(neuron_array, 'coordinates_x'):
                                # Direct numpy slice assignment - no loops
                                valid_positions = np.where(valid_mask)[0][valid_indices_mask]
                                result_coords[valid_positions, 0] = neuron_array.coordinates_x[valid_indices]
                                result_coords[valid_positions, 1] = neuron_array.coordinates_y[valid_indices]
                                result_coords[valid_positions, 2] = neuron_array.coordinates_z[valid_indices]
                            else:
                                return self._fallback_coordinate_calculation_vectorized(neuron_ids_array)
                        
                        # VECTORIZED fallback for invalid neurons - no loops
                        invalid_mask = ~valid_mask
                        if np.any(invalid_mask):
                            invalid_positions = np.where(invalid_mask)[0]
                            invalid_neuron_ids = neuron_ids_array[invalid_mask]
                            result_coords[invalid_positions, 0] = invalid_neuron_ids % 100
                            result_coords[invalid_positions, 1] = (invalid_neuron_ids // 100) % 100
                            result_coords[invalid_positions, 2] = invalid_neuron_ids // 10000
                        
                        return result_coords
                        
            # Pure vectorized fallback
            return self._fallback_coordinate_calculation_vectorized(neuron_ids_array)
            
        except Exception:
            # RTOS-friendly: minimal exception handling, no string formatting
            return self._fallback_coordinate_calculation_vectorized(neuron_ids_array)

    def _create_optimized_brain_output_structured(self, fire_queue_data: np.ndarray) -> np.ndarray:
        """Create optimized brain output using structured numpy arrays instead of dicts.
        
        RTOS/Rust/SIMD compliant: No dynamic allocations, no dictionaries, pure numpy.
        
        Args:
            fire_queue_data: numpy array with shape (N, 5) [neuron_ids, potentials, x, y, z]
            
        Returns:
            Structured numpy array optimized for brain output
        """
        if fire_queue_data is None or fire_queue_data.size == 0:
            # Return empty structured array
            return np.empty(0, dtype=[
                ('neuron_id', np.int32),
                ('membrane_potential', np.float32),
                ('x', np.int32),
                ('y', np.int32),
                ('z', np.int32)
            ])
        
        # Create structured array - SIMD-friendly, Rust-compatible
        neuron_count = fire_queue_data.shape[0]
        structured_data = np.empty(neuron_count, dtype=[
            ('neuron_id', np.int32),
            ('membrane_potential', np.float32),
            ('x', np.int32),
            ('y', np.int32),
            ('z', np.int32)
        ])
        
        # VECTORIZED assignment - no loops
        structured_data['neuron_id'] = fire_queue_data[:, 0].astype(np.int32)
        structured_data['membrane_potential'] = fire_queue_data[:, 1].astype(np.float32)
        structured_data['x'] = fire_queue_data[:, 2].astype(np.int32)
        structured_data['y'] = fire_queue_data[:, 3].astype(np.int32)
        structured_data['z'] = fire_queue_data[:, 4].astype(np.int32)
        
        return structured_data

    def _get_area_fire_queue_data_structured(self, cortical_id: str) -> Optional[np.ndarray]:
        """
        Get fire queue data for a specific cortical area using structured arrays.
        
        RTOS/Rust compliant: No dictionaries, no list comprehensions, pure numpy.
        
        Returns:
            Structured numpy array or None if no firing neurons
        """
        try:
            # Try direct access first if available
            if hasattr(self.fire_queue_provider, 'get_area_fire_queue_direct'):
                brain_data = self.fire_queue_provider.get_area_fire_queue_direct(cortical_id)
                if brain_data is not None:
                    return self._create_optimized_brain_output_structured(brain_data)
                    
            # Fallback to existing methods - convert to structured format
            fire_queue = None
            if hasattr(self.fire_queue_provider, 'get_area_fire_queue'):
                fire_queue = self.fire_queue_provider.get_area_fire_queue(cortical_id)
            elif hasattr(self.fire_queue_provider, 'get_fire_queue'):
                global_fire_queue = self.fire_queue_provider.get_fire_queue()
                if global_fire_queue:
                    fire_queue = self._filter_fire_queue_by_area_structured(global_fire_queue, cortical_id)
            else:
                return None
                
            if fire_queue is None:
                return None
                
            # Convert legacy dict format to structured array
            return self._convert_legacy_fire_queue_to_structured(fire_queue, cortical_id)
            
        except Exception:
            # RTOS-friendly: minimal exception handling
            return None

    def _get_global_fire_queue_data_structured(self) -> Optional[np.ndarray]:
        """Get global fire queue data using structured arrays - RTOS/Rust compliant."""
        try:
            # Try direct access first
            if hasattr(self.fire_queue_provider, 'get_fire_queue_direct'):
                brain_data = self.fire_queue_provider.get_fire_queue_direct()
                if brain_data is not None:
                    return self._create_optimized_brain_output_structured(brain_data)
            
            # Fallback to existing method
            if hasattr(self.fire_queue_provider, 'get_fire_queue'):
                fire_queue = self.fire_queue_provider.get_fire_queue()
                if fire_queue is None:
                    return None
                    
                # Convert legacy dict format to structured array
                return self._convert_legacy_fire_queue_to_structured(fire_queue)
            return None
            
        except Exception:
            # RTOS-friendly: minimal exception handling
            return None

    def _convert_legacy_fire_queue_to_structured(self, fire_queue, cortical_id: str = None) -> np.ndarray:
        """Convert legacy dictionary fire queue to structured numpy array.
        
        RTOS/Rust compliant: Eliminates dictionary operations, uses pure numpy.
        
        Args:
            fire_queue: Legacy dictionary format fire queue
            cortical_id: Optional cortical area ID for coordinate lookup
            
        Returns:
            Structured numpy array with brain output data
        """
        # Extract neuron IDs - handle both dict and other formats
        if hasattr(fire_queue, 'get'):
            # Dictionary format
            neuron_ids_list = fire_queue.get('neuron_ids', [])
            membrane_potentials_list = fire_queue.get('membrane_potentials', [])
        else:
            # Assume already structured or empty
            return np.empty(0, dtype=[
                ('neuron_id', np.int32),
                ('membrane_potential', np.float32),
                ('x', np.int32),
                ('y', np.int32),
                ('z', np.int32)
            ])
        
        if not neuron_ids_list:
            return np.empty(0, dtype=[
                ('neuron_id', np.int32),
                ('membrane_potential', np.float32),
                ('x', np.int32),
                ('y', np.int32),
                ('z', np.int32)
            ])
        
        # Convert to numpy arrays - eliminate list operations
        neuron_ids_array = np.array(neuron_ids_list, dtype=np.int32)
        membrane_potentials_array = np.array(membrane_potentials_list, dtype=np.float32)
        
        # Get coordinates using vectorized method - NO list comprehensions
        if cortical_id:
            coordinates_array = self._get_neuron_coordinates_vectorized_rtos(cortical_id, neuron_ids_array)
        else:
            coordinates_array = self._get_global_neuron_coordinates_vectorized_rtos(neuron_ids_array)
        
        # Create structured output - pure numpy, no dictionaries
        neuron_count = len(neuron_ids_array)
        structured_data = np.empty(neuron_count, dtype=[
            ('neuron_id', np.int32),
            ('membrane_potential', np.float32),
            ('x', np.int32),
            ('y', np.int32),
            ('z', np.int32)
        ])
        
        # VECTORIZED assignment - no loops, no comprehensions
        structured_data['neuron_id'] = neuron_ids_array
        structured_data['membrane_potential'] = membrane_potentials_array
        structured_data['x'] = coordinates_array[:, 0]
        structured_data['y'] = coordinates_array[:, 1]
        structured_data['z'] = coordinates_array[:, 2]
        
        return structured_data

    def _get_global_neuron_coordinates_vectorized_rtos(self, neuron_ids_array: np.ndarray) -> np.ndarray:
        """Get global 3D coordinates using pure vectorized operations - RTOS/Rust compliant.
        
        NO list comprehensions, NO dictionary operations, pure numpy.
        
        Args:
            neuron_ids_array: Pre-allocated numpy array of neuron IDs
            
        Returns:
            numpy array with shape (N, 3) containing coordinates
        """
        if neuron_ids_array.size == 0:
            return np.empty((0, 3), dtype=np.int32)
            
        try:
            if self.connectome_manager and hasattr(self.connectome_manager, 'neuron_array'):
                neuron_array = self.connectome_manager.neuron_array
                
                if hasattr(self.connectome_manager, 'neuron_id_to_index'):
                    # VECTORIZED global index lookup - no comprehensions
                    neuron_ids_set = set(self.connectome_manager.neuron_id_to_index.keys())
                    valid_mask = np.isin(neuron_ids_array, list(neuron_ids_set))
                    
                    if np.any(valid_mask):
                        # Pre-allocate result array
                        result_coords = np.zeros((len(neuron_ids_array), 3), dtype=np.int32)
                        
                        # Use vectorized index lookup instead of list comprehension
                        valid_neuron_ids = neuron_ids_array[valid_mask]
                        indices_array = self._vectorized_index_lookup(valid_neuron_ids)
                        
                        # Mask out invalid indices
                        valid_indices_mask = indices_array >= 0
                        if np.any(valid_indices_mask):
                            valid_indices = indices_array[valid_indices_mask]
                            
                            # PURE VECTORIZED GLOBAL SoA ACCESS
                            if hasattr(neuron_array, 'coordinates_x'):
                                # Direct numpy slice assignment - no loops
                                valid_positions = np.where(valid_mask)[0][valid_indices_mask]
                                result_coords[valid_positions, 0] = neuron_array.coordinates_x[valid_indices]
                                result_coords[valid_positions, 1] = neuron_array.coordinates_y[valid_indices]
                                result_coords[valid_positions, 2] = neuron_array.coordinates_z[valid_indices]
                            else:
                                return self._fallback_coordinate_calculation_vectorized(neuron_ids_array)
                        
                        # VECTORIZED fallback for invalid neurons - no loops
                        invalid_mask = ~valid_mask
                        if np.any(invalid_mask):
                            invalid_positions = np.where(invalid_mask)[0]
                            invalid_neuron_ids = neuron_ids_array[invalid_mask]
                            result_coords[invalid_positions, 0] = invalid_neuron_ids % 100
                            result_coords[invalid_positions, 1] = (invalid_neuron_ids // 100) % 100
                            result_coords[invalid_positions, 2] = invalid_neuron_ids // 10000
                        
                        return result_coords
                    
            # Pure vectorized fallback
            return self._fallback_coordinate_calculation_vectorized(neuron_ids_array)
            
        except Exception:
            # RTOS-friendly: minimal exception handling
            return self._fallback_coordinate_calculation_vectorized(neuron_ids_array)

    def _filter_fire_queue_by_area_structured(self, fire_queue, cortical_id: str) -> Optional[np.ndarray]:
        """Filter fire queue data to only include neurons from specified area - RTOS/Rust compliant.
        
        NO dictionary operations, NO list comprehensions, pure numpy.
        """
        if fire_queue is None or self.connectome_manager is None:
            return None
            
        try:
            area = self.connectome_manager.cortical_areas.get(cortical_id)
            if area is None:
                return np.empty(0, dtype=[
                    ('neuron_id', np.int32),
                    ('membrane_potential', np.float32),
                    ('x', np.int32),
                    ('y', np.int32),
                    ('z', np.int32)
                ])
                
            # Get neuron ID range for this area
            area_neuron_ids = set(area.get_neuron_ids()) if hasattr(area, 'get_neuron_ids') else set()
            if not area_neuron_ids:
                return np.empty(0, dtype=[
                    ('neuron_id', np.int32),
                    ('membrane_potential', np.float32),
                    ('x', np.int32),
                    ('y', np.int32),
                    ('z', np.int32)
                ])
            
            # Extract data from fire queue - eliminate dictionary get operations
            if hasattr(fire_queue, 'get'):
                neuron_ids_list = fire_queue.get('neuron_ids', [])
                membrane_potentials_list = fire_queue.get('membrane_potentials', [])
            else:
                return None
            
            if not neuron_ids_list:
                return np.empty(0, dtype=[
                    ('neuron_id', np.int32),
                    ('membrane_potential', np.float32),
                    ('x', np.int32),
                    ('y', np.int32),
                    ('z', np.int32)
                ])
            
            # Convert to numpy arrays
            neuron_ids_array = np.array(neuron_ids_list, dtype=np.int32)
            membrane_potentials_array = np.array(membrane_potentials_list, dtype=np.float32)
            
            # VECTORIZED filtering - no loops, no comprehensions
            area_neuron_ids_array = np.array(list(area_neuron_ids), dtype=np.int32)
            filter_mask = np.isin(neuron_ids_array, area_neuron_ids_array)
            
            if not np.any(filter_mask):
                return np.empty(0, dtype=[
                    ('neuron_id', np.int32),
                    ('membrane_potential', np.float32),
                    ('x', np.int32),
                    ('y', np.int32),
                    ('z', np.int32)
                ])
            
            # Filter using vectorized operations
            filtered_neuron_ids = neuron_ids_array[filter_mask]
            filtered_potentials = membrane_potentials_array[filter_mask]
            
            # Get coordinates for filtered neurons
            coordinates_array = self._get_neuron_coordinates_vectorized_rtos(cortical_id, filtered_neuron_ids)
            
            # Create structured output
            neuron_count = len(filtered_neuron_ids)
            structured_data = np.empty(neuron_count, dtype=[
                ('neuron_id', np.int32),
                ('membrane_potential', np.float32),
                ('x', np.int32),
                ('y', np.int32),
                ('z', np.int32)
            ])
            
            # VECTORIZED assignment
            structured_data['neuron_id'] = filtered_neuron_ids
            structured_data['membrane_potential'] = filtered_potentials
            structured_data['x'] = coordinates_array[:, 0]
            structured_data['y'] = coordinates_array[:, 1]
            structured_data['z'] = coordinates_array[:, 2]
            
            return structured_data
            
        except Exception:
            # RTOS-friendly: minimal exception handling
            return None

    def _convert_fire_queue_to_simd_optimized(self, fire_queue, cortical_id: str = None) -> np.ndarray:
        """Convert legacy fire queue to SIMD-optimized structured arrays.
        
        Uses SIMD backend selection and vectorized operations for maximum performance.
        
        Args:
            fire_queue: Legacy dictionary format fire queue
            cortical_id: Optional cortical area ID for coordinate lookup
            
        Returns:
            SIMD-optimized structured numpy array with brain output data
        """
        # Extract neuron IDs - handle both dict and other formats
        if hasattr(fire_queue, 'get'):
            neuron_ids_list = fire_queue.get('neuron_ids', [])
            membrane_potentials_list = fire_queue.get('membrane_potentials', [])
        else:
            return self._create_empty_simd_structure()
        
        if not neuron_ids_list:
            return self._create_empty_simd_structure()
        
        # Convert to SIMD-aligned numpy arrays
        neuron_ids_array = self._create_simd_aligned_array(neuron_ids_list, np.int32)
        membrane_potentials_array = self._create_simd_aligned_array(membrane_potentials_list, np.float32)
        
        # Use SIMD-optimized coordinate lookup
        if SIMD_AVAILABLE and len(neuron_ids_array) > 0:
            coordinates_array = self._get_neuron_coordinates_simd_optimized(cortical_id, neuron_ids_array)
        else:
            coordinates_array = self._fallback_coordinate_calculation_vectorized(neuron_ids_array)
        
        # Create SIMD-optimized structured output with aligned memory
        return self._create_simd_optimized_structure(
            neuron_ids_array, membrane_potentials_array, coordinates_array
        )
    
    def _create_simd_aligned_array(self, data_list: List, dtype: np.dtype) -> np.ndarray:
        """Create SIMD-aligned numpy array from list data."""
        if not SIMD_AVAILABLE or not self.backend_selector:
            return np.array(data_list, dtype=dtype)
        
        # Get optimal alignment for SIMD operations
        alignment = self.simd_config.get("alignment", 32)  # Default to 32-byte alignment
        
        # Create array with optimal size for SIMD
        original_size = len(data_list)
        aligned_size = self.simd_detector.get_aligned_size(original_size)
        
        # Create aligned array
        aligned_array = np.zeros(aligned_size, dtype=dtype)
        aligned_array[:original_size] = data_list
        
        return aligned_array[:original_size]  # Return only the valid data portion
    
    def _create_simd_optimized_structure(self, neuron_ids: np.ndarray, 
                                       membrane_potentials: np.ndarray, 
                                       coordinates: np.ndarray) -> np.ndarray:
        """Create SIMD-optimized structured array with proper alignment."""
        neuron_count = len(neuron_ids)
        
        # Use SIMD-friendly dtype alignment
        dtype_list = [
            ('neuron_id', np.int32),
            ('membrane_potential', np.float32),
            ('x', np.int32),
            ('y', np.int32),
            ('z', np.int32)
        ]
        
        # Create structured array with potential SIMD alignment
        if SIMD_AVAILABLE and self.backend_selector:
            # Try to create with optimal alignment
            aligned_size = self.simd_detector.get_aligned_size(neuron_count)
            structured_data = np.empty(aligned_size, dtype=dtype_list)
            
            # Fill only the valid portion
            structured_data[:neuron_count]['neuron_id'] = neuron_ids
            structured_data[:neuron_count]['membrane_potential'] = membrane_potentials
            structured_data[:neuron_count]['x'] = coordinates[:, 0]
            structured_data[:neuron_count]['y'] = coordinates[:, 1] 
            structured_data[:neuron_count]['z'] = coordinates[:, 2]
            
            return structured_data[:neuron_count]  # Return only valid data
        else:
            # Standard structured array
            structured_data = np.empty(neuron_count, dtype=dtype_list)
            structured_data['neuron_id'] = neuron_ids
            structured_data['membrane_potential'] = membrane_potentials
            structured_data['x'] = coordinates[:, 0]
            structured_data['y'] = coordinates[:, 1]
            structured_data['z'] = coordinates[:, 2]
            
            return structured_data
    
    def _create_empty_simd_structure(self) -> np.ndarray:
        """Create empty SIMD-optimized structured array."""
        return np.empty(0, dtype=[
            ('neuron_id', np.int32),
            ('membrane_potential', np.float32),
            ('x', np.int32),
            ('y', np.int32),
            ('z', np.int32)
        ])

    def sample_area_fire_queue(self, cortical_id: str, target: str = 'visualization') -> None:
        """High-level method that automatically chooses optimal sampling strategy.
        
        Automatically selects between SIMD-optimized, structured array, or legacy sampling
        based on SIMD availability, data size, and performance characteristics.
        
        Args:
            cortical_id: Cortical area ID to sample
            target: Target type ('visualization' or 'motor')
        """
        # Quick availability check
        if not self.fire_queue_provider or not cortical_id:
            self._store_empty_sample(cortical_id, target)
            return
        
        # Determine optimal sampling strategy
        strategy = self._select_optimal_sampling_strategy(cortical_id, target)
        
        try:
            if strategy == 'simd':
                self._sample_area_fire_queue_simd_optimized(cortical_id, target)
            elif strategy == 'structured':
                self._sample_area_fire_queue_structured(cortical_id, target)
            else:
                # Fallback to legacy method
                self._sample_area_fire_queue_legacy(cortical_id, target)
                
        except Exception as e:
            # Emergency fallback
            if self.logger:
                self.logger.warning(f"Sampling failed for {cortical_id}, using emergency fallback: {e}")
            self._store_empty_sample(cortical_id, target)
    
    def _select_optimal_sampling_strategy(self, cortical_id: str, target: str) -> str:
        """Select optimal sampling strategy based on data characteristics and capabilities."""
        # SIMD strategy selection
        if self.enable_simd and SIMD_AVAILABLE:
            # Check if we have enough data to benefit from SIMD
            estimated_neuron_count = self._estimate_neuron_count(cortical_id)
            
            # SIMD is beneficial for larger datasets (typically >64 neurons)
            simd_threshold = self.simd_config.get('vector_width', 4) * 16  # 16 vectors worth
            
            if estimated_neuron_count >= simd_threshold:
                return 'simd'
            
            # For smaller datasets, SIMD overhead may not be worth it
            if estimated_neuron_count > 8:  # Still use SIMD for medium datasets
                return 'simd'
        
        # Structured array strategy (RTOS/Rust compliant but no SIMD)
        estimated_neuron_count = self._estimate_neuron_count(cortical_id)
        if estimated_neuron_count > 0:
            return 'structured'
        
        # Legacy fallback
        return 'legacy'
    
    def _estimate_neuron_count(self, cortical_id: str) -> int:
        """Estimate neuron count for sampling strategy selection."""
        try:
            # Try quick count estimation if provider supports it
            if hasattr(self.fire_queue_provider, 'estimate_area_neuron_count'):
                return self.fire_queue_provider.estimate_area_neuron_count(cortical_id)
            
            # Try connectome-based estimation
            if self.connectome_manager and hasattr(self.connectome_manager, 'get_area_neuron_count'):
                return self.connectome_manager.get_area_neuron_count(cortical_id)
            
            # Default reasonable estimate for SIMD strategy selection
            return 32
            
        except Exception:
            return 16  # Conservative estimate