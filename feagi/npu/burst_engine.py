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
                # Use centralized SIMD configuration from State Manager
                from feagi.core.state_manager import get_state_manager
                state_manager = get_state_manager()
                self.simd_config = state_manager.get_simd_configuration()
                
                if self.simd_config['available']:
                    # Initialize SIMD components with centralized config
                    self.simd_detector = get_simd_detector()
                    self.simd_profiler = get_profiler()
                    
                    # Log using centralized configuration
                    logger.info(f"[BURST] Using centralized SIMD: {self.simd_config['backend']} "
                               f"(vector_width={self.simd_config['vector_width']}, "
                               f"alignment={self.simd_config['alignment']})")
                else:
                    logger.info("[BURST] Centralized SIMD reports: acceleration not available")
                    self.simd_detector = None
                    self.simd_profiler = None
                    
            except Exception as e:
                logger.warning(f"Failed to access centralized SIMD configuration: {e}")
                self.simd_detector = None
                self.simd_config = {'available': False, 'backend': 'SCALAR', 'vector_width': 1}
        else:
            self.simd_detector = None
            self.simd_config = {'available': False, 'backend': 'SCALAR', 'vector_width': 1}

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

class UnifiedFQSampler:
    """
    High-performance Fire Queue Sampler for FEAGI NPU.
    
    This unified sampler combines zero-copy optimizations with SIMD acceleration and
    centralized configuration. Can be instantiated independently for different purposes:
    - Motor sampling: High-frequency OPU area sampling
    - Visualization sampling: Configurable-rate full brain sampling
    - Custom sampling: Application-specific sampling patterns
    
    Features:
    - Zero-copy operations with direct SoA access
    - SIMD acceleration via centralized configuration  
    - Pre-allocated buffers for real-time performance
    - RTOS-compatible deterministic timing
    - Independent frequency configuration per instance
    """
    
    def __init__(self, 
                 fire_queue_provider,
                 sample_frequency_hz: float,
                 output_queue=None,
                 connectome_manager=None,
                 sampling_mode: str = 'global',  # 'global', 'motor_only', 'areas_only', 'custom'
                 target_areas: Optional[List[str]] = None,
                 max_retries: int = 3,
                 neuron_type_filter: Optional[str] = None,
                 enable_simd: bool = True,
                 enable_zero_copy: bool = True,
                 buffer_size: int = 100_000):
        """
        Initialize the unified FQ sampler.
        
        Args:
            fire_queue_provider: Source for fire queue data
            sample_frequency_hz: Sampling frequency in Hz
            output_queue: Optional output queue for samples
            connectome_manager: Optional connectome manager for area info
            sampling_mode: 'global' (all areas), 'motor_only' (OPU areas), 'areas_only' (specific areas), 'custom'
            target_areas: Specific areas to sample (for 'areas_only' or 'custom' modes)
            max_retries: Maximum retry attempts for failed operations
            neuron_type_filter: Optional filter for specific neuron types
            enable_simd: Enable SIMD acceleration if available
            enable_zero_copy: Enable zero-copy optimizations
            buffer_size: Pre-allocated buffer size for neurons
        """
        self.fire_queue_provider = fire_queue_provider
        self.sample_frequency = sample_frequency_hz
        self.sample_interval = 1.0 / sample_frequency_hz if sample_frequency_hz > 0 else 0.1
        self.output_queue = output_queue
        self.connectome_manager = connectome_manager
        self.sampling_mode = sampling_mode.lower()
        self.target_areas = target_areas or []
        self.max_retries = max_retries
        self.neuron_type_filter = neuron_type_filter
        self.enable_simd = enable_simd
        self.enable_zero_copy = enable_zero_copy
        self.running = False
        
        # Performance tracking
        self._last_sample_time = 0.0
        self._sample_count = 0
        self._performance_stats = {
            'samples_generated': 0,
            'zero_copy_hits': 0,
            'simd_operations': 0,
            'average_sample_time': 0.0
        }
        
        # Pre-allocate buffers for zero-copy operations
        self.max_neurons_per_sample = buffer_size
        self.output_buffer = np.empty((self.max_neurons_per_sample, 6), dtype=np.float32)
        
        # Initialize SIMD configuration from centralized state manager
        self._initialize_simd_configuration()
        
        # Initialize direct access to optimized structures
        self._initialize_optimized_access()
        
        # Cache OPU areas for motor sampling
        self._cached_opu_areas = None
        self._opu_cache_timestamp = 0.0
        
        logger.info(f"UnifiedFQSampler initialized: mode={self.sampling_mode}, "
                   f"frequency={self.sample_frequency}Hz, simd={self.simd_config['available']}, "
                   f"zero_copy={self.enable_zero_copy}")

    def _initialize_simd_configuration(self) -> None:
        """Initialize SIMD configuration from centralized state manager."""
        try:
            # Use centralized SIMD configuration from State Manager
            from feagi.core.state_manager import get_state_manager
            state_manager = get_state_manager()
            self.simd_config = state_manager.get_simd_configuration()
            
            # Initialize membrane processor if SIMD is available
            if self.simd_config['available'] and self.enable_simd:
                from feagi.npu.optimized_membrane_operations import SIMDMembraneProcessor
                self.membrane_processor = SIMDMembraneProcessor(
                    backend=self.simd_config['backend'],
                    vector_width=self.simd_config['vector_width']
                )
                logger.info(f"UnifiedFQSampler using centralized SIMD: {self.simd_config['backend']} "
                           f"(vector_width={self.simd_config['vector_width']})")
            else:
                self.membrane_processor = None
                if self.enable_simd:
                    logger.info("UnifiedFQSampler: SIMD not available, using scalar operations")
                else:
                    logger.info("UnifiedFQSampler: SIMD disabled by configuration")
                    
        except ImportError:
            logger.warning("UnifiedFQSampler: State Manager not available, SIMD disabled")
            self.simd_config = {'available': False, 'backend': 'SCALAR', 'vector_width': 1}
            self.membrane_processor = None
        except Exception as e:
            logger.error(f"UnifiedFQSampler: Error accessing SIMD configuration: {e}")
            self.simd_config = {'available': False, 'backend': 'SCALAR', 'vector_width': 1}
            self.membrane_processor = None

    def _initialize_optimized_access(self) -> None:
        """Initialize direct access to optimized SoA structures."""
        try:
            # Get direct access to FCL manager and neuron arrays
            self.fcl_manager = getattr(self.connectome_manager, 'fcl_manager', None) if self.connectome_manager else None
            self.gna = getattr(self.connectome_manager, 'neuron_array', None) if self.connectome_manager else None
            
            if self.fcl_manager and self.enable_zero_copy:
                logger.info("UnifiedFQSampler: Zero-copy mode enabled with direct SoA access")
            else:
                logger.info("UnifiedFQSampler: Using standard fire queue access")
                
        except Exception as e:
            logger.warning(f"UnifiedFQSampler: Could not initialize optimized access: {e}")
            self.fcl_manager = None
            self.gna = None

    def sample_direct(self) -> Optional[bytes]:
        """
        High-performance direct sampling with zero-copy optimizations.
        
        Returns:
            Binary encoded sample data or None if no firing neurons
        """
        start_time = time.perf_counter()
        
        try:
            # Get fire queue data based on sampling mode
            brain_data = self._extract_brain_data_by_mode()
            
            if brain_data is None or len(brain_data) == 0:
                return None
            
            # Add timestamp column
            timestamped_data = np.column_stack((
                brain_data,
                np.full(len(brain_data), time.time(), dtype=np.float32)
            ))
            
            # Update performance stats
            self._performance_stats['samples_generated'] += 1
            if self.enable_zero_copy and self.fcl_manager:
                self._performance_stats['zero_copy_hits'] += 1
            if self.simd_config['available'] and self.membrane_processor:
                self._performance_stats['simd_operations'] += 1
            
            # Track timing
            sample_time = time.perf_counter() - start_time
            self._performance_stats['average_sample_time'] = (
                (self._performance_stats['average_sample_time'] * self._sample_count + sample_time) / 
                (self._sample_count + 1)
            )
            self._sample_count += 1
            
            # Direct binary encoding
            return self._encode_brain_data_binary(timestamped_data)
            
        except Exception as e:
            logger.error(f"Error in direct sampling: {e}")
            return None

    def _extract_brain_data_by_mode(self) -> Optional[np.ndarray]:
        """Extract brain data based on the configured sampling mode."""
        if self.sampling_mode == 'global':
            return self._extract_global_brain_data()
        elif self.sampling_mode == 'motor_only':
            return self._extract_motor_brain_data()
        elif self.sampling_mode == 'areas_only':
            return self._extract_areas_brain_data(self.target_areas)
        elif self.sampling_mode == 'custom':
            return self._extract_custom_brain_data()
        else:
            logger.error(f"Unknown sampling mode: {self.sampling_mode}")
            return None

    def _extract_global_brain_data(self) -> Optional[np.ndarray]:
        """Extract brain data from global fire queue."""
        if not self.fcl_manager:
            return None
            
        global_fcl = self.fcl_manager.get_global_fcl()
        return self._extract_brain_data_from_fcl(global_fcl)

    def _extract_motor_brain_data(self) -> Optional[np.ndarray]:
        """Extract brain data from motor/OPU areas only."""
        if not self.fcl_manager:
            return None
        
        # Cache OPU areas with timeout to avoid repeated lookups
        current_time = time.time()
        if (self._cached_opu_areas is None or 
            current_time - self._opu_cache_timestamp > 5.0):  # 5 second cache
            self._cached_opu_areas = self._get_opu_areas_fast()
            self._opu_cache_timestamp = current_time
        
        if not self._cached_opu_areas:
            return None
        
        # Combine FCLs from all OPU areas using bitmap operations
        combined_fcl = None
        
        for area_id in self._cached_opu_areas:
            area_fcl = self.fcl_manager.get_cortical_fcl(area_id)
            if not area_fcl.is_empty():
                if combined_fcl is None:
                    combined_fcl = area_fcl
                else:
                    combined_fcl = combined_fcl | area_fcl
        
        if combined_fcl is None or combined_fcl.is_empty():
            return None
        
        return self._extract_brain_data_from_fcl(combined_fcl)

    def _extract_areas_brain_data(self, area_ids: List[str]) -> Optional[np.ndarray]:
        """Extract brain data from specific cortical areas."""
        if not self.fcl_manager or not area_ids:
            return None
        
        # Combine FCLs from specified areas
        combined_fcl = None
        
        for area_id in area_ids:
            try:
                area_fcl = self.fcl_manager.get_cortical_fcl(area_id)
                if not area_fcl.is_empty():
                    if combined_fcl is None:
                        combined_fcl = area_fcl
                    else:
                        combined_fcl = combined_fcl | area_fcl
            except Exception as e:
                logger.warning(f"Error accessing FCL for area {area_id}: {e}")
                continue
        
        if combined_fcl is None or combined_fcl.is_empty():
            return None
        
        return self._extract_brain_data_from_fcl(combined_fcl)

    def _extract_custom_brain_data(self) -> Optional[np.ndarray]:
        """Extract brain data using custom sampling logic."""
        # Default implementation - can be overridden by subclasses
        return self._extract_global_brain_data()

    def _extract_brain_data_from_fcl(self, fcl) -> Optional[np.ndarray]:
        """Extract brain data from a given FCL using direct SoA access."""
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

    def _get_opu_areas_fast(self) -> List[str]:
        """Fast lookup of OPU (Output Processing Unit) cortical areas."""
        try:
            if not self.connectome_manager:
                return []
            
            # Fast lookup using cached connectome structure
            if hasattr(self.connectome_manager, 'get_areas_by_type'):
                return self.connectome_manager.get_areas_by_type('OPU')
            
            # Fallback: scan all areas for OPU type
            opu_areas = []
            all_areas = getattr(self.connectome_manager, 'get_cortical_areas', lambda: [])()
            
            for area_id in all_areas:
                try:
                    area_info = self.connectome_manager.get_area_info(area_id)
                    if area_info and area_info.get('type') == 'OPU':
                        opu_areas.append(area_id)
                except Exception:
                    continue  # Skip areas we can't access
            
            return opu_areas
            
        except Exception as e:
            logger.warning(f"Error getting OPU areas: {e}")
            return []

    def _encode_brain_data_binary(self, data: np.ndarray) -> bytes:
        """Encode brain data directly to binary format."""
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

    def run(self) -> None:
        """Main sampling loop with deterministic timing."""
        logger.info(f"UnifiedFQSampler started: mode={self.sampling_mode}, frequency={self.sample_frequency}Hz")
        self.running = True
        
        while self.running:
            start = time.perf_counter()
            
            # Generate sample
            sample_data = self.sample_direct()
            
            # Store sample in output queue if available
            if sample_data and self.output_queue:
                try:
                    self.output_queue.put_nowait(sample_data)
                except Exception as e:
                    logger.warning(f"Failed to queue sample: {e}")
            
            # RTOS-compatible deterministic timing
            elapsed = time.perf_counter() - start
            if elapsed < self.sample_interval:
                target_end_time = start + self.sample_interval
                while time.perf_counter() < target_end_time:
                    pass  # Busy-wait for remainder of sample interval

    def stop(self) -> None:
        """Stop the sampler."""
        self.running = False
        logger.info(f"UnifiedFQSampler stopped. Performance stats: {self._performance_stats}")

    def get_performance_stats(self) -> Dict[str, Any]:
        """Get performance statistics."""
        stats = self._performance_stats.copy()
        stats['sample_frequency'] = self.sample_frequency
        stats['sampling_mode'] = self.sampling_mode
        stats['simd_enabled'] = self.simd_config['available']
        stats['zero_copy_enabled'] = self.enable_zero_copy and self.fcl_manager is not None
        return stats

    def set_sample_frequency(self, frequency_hz: float) -> None:
        """Update sampling frequency."""
        if frequency_hz > 0:
            self.sample_frequency = frequency_hz
            self.sample_interval = 1.0 / frequency_hz
            logger.info(f"UnifiedFQSampler frequency updated to {frequency_hz}Hz")

    def set_target_areas(self, area_ids: List[str]) -> None:
        """Update target areas for area-specific sampling."""
        self.target_areas = area_ids
        logger.info(f"UnifiedFQSampler target areas updated: {area_ids}")

# Backward compatibility aliases
FQSampler = UnifiedFQSampler
OptimizedFQSampler = UnifiedFQSampler