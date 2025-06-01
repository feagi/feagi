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

# Import the new modular components
from .burst_engine_debug import BurstEngineDebugMixin
from .burst_engine_performance import BurstEnginePerformanceMixin

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
- Modular Architecture: Uses mixins for debug and performance functionality

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

class BurstEngine(BurstEngineDebugMixin, BurstEnginePerformanceMixin):
    """
    RTOS/Rust-friendly burst engine for FEAGI neural simulation.
    - No dynamic allocation in the main loop
    - All configuration and memory allocation happens before entering the loop
    - Main loop is a single, clear sequence of steps
    - Supports graceful shutdown
    - New: Initializes in standby mode without requiring a genome
    - New: Supports special area handling including power area injection
    - Singleton: Only one instance can exist at any time
    - Modular: Uses mixins for debug and performance functionality
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
        
        # Manually initialize mixins (not through super() to avoid multiple inheritance issues)
        # Initialize performance mixin
        BurstEnginePerformanceMixin.__init__(self)
        # Initialize debug mixin 
        BurstEngineDebugMixin.__init__(self)
        
        # Mark as initialized
        self._initialized = True
        
        # Core processing components
        self.scheduler = None
        self.fire_queue_provider = None
        
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
                # Check if it has any batches prepared
                try:
                    preview = self.fcl_injection_service.get_power_injection_preview()
                    logger.debug(f"[DEBUG] BURST ENGINE: Injection preview: {preview}")
                except Exception as e:
                    logger.debug(f"[DEBUG] BURST ENGINE: Error getting injection preview: {e}")
            else:
                logger.debug(f"[DEBUG] BURST ENGINE: NO INJECTION SERVICE!")
        
        # 1. Pre-burst power injection
        if self.fcl_injection_service and self.power_injection_timing == 'pre_burst':
            if self.debug_npu:
                logger.debug(f"[DEBUG] BURST ENGINE: Calling pre-burst injection with timing {self.power_injection_timing}")
            injected = self.fcl_injection_service.inject_pre_burst(self.burst_count)
            if self.debug_npu:
                logger.info(f"[DEBUG] BURST ENGINE: Pre-burst injection returned {injected} neurons")
        elif self.fcl_injection_service:
            if self.debug_npu:
                logger.debug(f"[DEBUG] BURST ENGINE: Skipping pre-burst injection - timing is {self.power_injection_timing}")
        
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
            injected = self.fcl_injection_service.inject_during_burst(self.burst_count)
            if self.debug_npu:
                logger.info(f"[DEBUG] BURST ENGINE: During-burst injection returned {injected} neurons")
        
        # 4. Post-burst injection  
        if self.fcl_injection_service and self.power_injection_timing == 'post_burst':
            if self.debug_npu:
                logger.debug(f"[DEBUG] BURST ENGINE: Calling post-burst injection")
            injected = self.fcl_injection_service.inject_post_burst(self.burst_count)
            if self.debug_npu:
                logger.info(f"[DEBUG] BURST ENGINE: Post-burst injection returned {injected} neurons")
        
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
        Main burst engine loop.
        
        This method runs the main burst engine loop with precise timing control.
        Uses RTOS-compatible timing for deterministic performance.
        """
        if self._running:
            logger.warning("Burst engine is already running")
            return
        
        # State transition
        self.state_manager.set_burst_engine_state(ServiceState.INITIALIZING)
        
        logger.info(f"Starting burst engine with frequency: {self.desired_frequency}Hz", status="[START]")
        self._running = True
        
        # State transition
        self.state_manager.set_burst_engine_state(ServiceState.READY)
        
        try:
            while self._running:
                burst_cycle_start = time.perf_counter()
                processing_start = time.perf_counter()
                
                # Execute burst processing
                try:
                    fired_neurons = self._process_burst()
                    processing_end = time.perf_counter()
                    processing_duration = processing_end - processing_start
                    
                    # Record processing timing for performance measurement
                    self._record_processing_timing(processing_duration)
                    
                    # Debug performance if enabled
                    if hasattr(self, 'debug_burst_performance'):
                        burst_cycle_end = time.perf_counter()
                        burst_duration = burst_cycle_end - burst_cycle_start
                        self.debug_burst_performance(burst_duration, processing_duration)
                    
                except Exception as e:
                    logger.error(f"Error in burst processing: {e}")
                    processing_duration = time.perf_counter() - processing_start
                
                self.burst_count += 1
                self.last_burst_time = time.time()
                
                # RTOS-COMPATIBLE: Precise timing control
                cycle_end = time.perf_counter()
                cycle_duration = cycle_end - burst_cycle_start
                
                # Record full cycle timing for frequency measurement
                self._record_burst_timing(cycle_duration)
                
                # Calculate sleep time to maintain target frequency
                if cycle_duration < self.burst_interval:
                    sleep_time = self.burst_interval - cycle_duration
                    # RTOS-COMPATIBLE: Use deterministic timing
                    target_time = burst_cycle_start + self.burst_interval
                    while time.perf_counter() < target_time:
                        pass  # Busy-wait for precise timing
                else:
                    # Running behind schedule
                    if self.debug_npu:
                        logger.warning(f"Burst cycle took {cycle_duration*1000:.2f}ms (target: {self.burst_interval*1000:.2f}ms)")
                        
        except Exception as e:
            logger.error(f"Fatal error in burst engine: {e}")
            self.state_manager.set_burst_engine_state(ServiceState.ERROR)
            raise
        finally:
            self._running = False
            self.state_manager.set_burst_engine_state(ServiceState.STOPPED)
            logger.info("Burst engine stopped", status="[STOP]")

    def stop(self) -> None:
        """Stop the Burst Engine gracefully."""
        logger.info("Stopping burst engine...", status="[STOP]")
        self._running = False
        self.state_manager.set_burst_engine_state(ServiceState.STOPPED)

    def run_test(self) -> List[int]:
        """
        Run a single test burst for testing purposes.
        
        Returns:
            List of fired neuron IDs
        """
        if not self.genome_loaded:
            logger.warning("Cannot run test burst - no genome loaded")
            return []
        
        logger.info("Running test burst...", status="[TEST]")
        
        test_start = time.perf_counter()
        
        try:
            # Run a single burst cycle
            fired_neurons = self._process_burst()
            
            test_duration = time.perf_counter() - test_start
            
            logger.info(f"Test burst completed in {test_duration*1000:.2f}ms, "
                       f"{len(fired_neurons)} neurons fired", status="[TEST]")
            
            # Increment burst count for test bursts too
            self.burst_count += 1
            
            return fired_neurons
            
        except Exception as e:
            logger.error(f"Test burst failed: {e}")
            return []

    def update_with_genome(self) -> None:
        """
        Update the burst engine configuration when a new genome is loaded.
        
        This method should be called after a new genome is loaded into the
        connectome manager to refresh the engine's understanding of the neural network.
        """
        logger.info("Updating burst engine with new genome", status="[CONFIG]")
        
        try:
            # Update cortical areas from connectome
            self.cortical_areas = list(self.connectome_manager.cortical_areas.values()) if hasattr(self.connectome_manager, 'cortical_areas') else []
            self.shed_areas = set(area.id for area in self.cortical_areas if area.properties.get('__shed', False))
            
            # Re-initialize special area services with new genome
            self._initialize_special_area_services()
            
            # Mark genome as loaded
            self.genome_loaded = True
            
            logger.info(f"Burst engine updated: {len(self.cortical_areas)} cortical areas, "
                       f"{len(self.shed_areas)} shed areas", status="[CONFIG]")
            
        except Exception as e:
            logger.error(f"Error updating burst engine with genome: {e}")
            self.genome_loaded = False

    def refresh_special_areas(self) -> None:
        """
        Refresh special area detection and configuration.
        
        This method can be called to re-detect special areas after configuration changes.
        """
        logger.info("Refreshing special area configuration", status="[CONFIG]")
        
        try:
            if self.special_area_handler:
                self.special_area_handler.detect_special_areas()
                
                # Reinitialize injection service if needed
                power_areas = self.special_area_handler.get_power_areas()
                if power_areas and not self.fcl_injection_service:
                    self.fcl_injection_service = FCLInjectionService(
                        fcl_manager=self.fcl_manager,
                        special_area_handler=self.special_area_handler,
                        config=self.config.get('fcl_injection_config', {})
                    )
                    logger.info(f"Re-initialized injection service for {len(power_areas)} power areas")
                    
        except Exception as e:
            logger.error(f"Error refreshing special areas: {e}")

    def get_power_injection_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about power area injection.
        
        Returns:
            Dictionary containing injection statistics
        """
        if not self.fcl_injection_service:
            return {"error": "Power injection service not available"}
        
        try:
            return self.fcl_injection_service.get_injection_statistics()
        except Exception as e:
            logger.error(f"Error getting power injection statistics: {e}")
            return {"error": str(e)}

    def set_power_injection_enabled(self, cortical_id: str, enabled: bool) -> bool:
        """
        Enable or disable power injection for a specific cortical area.
        
        Args:
            cortical_id: ID of the cortical area
            enabled: Whether to enable power injection
            
        Returns:
            True if successful, False otherwise
        """
        if not self.fcl_injection_service:
            logger.warning("Power injection service not available")
            return False
        
        try:
            return self.fcl_injection_service.set_power_injection_enabled(cortical_id, enabled)
        except Exception as e:
            logger.error(f"Error setting power injection for {cortical_id}: {e}")
            return False

    def run_with_fire_queue(self, mpf: bool = True, puf: bool = False, max_consecutive_fires: int = 10) -> bool:
        """
        Run fire queue processing with membrane potential and plasticity updates.
        
        Args:
            mpf: Membrane potential flag
            puf: Plasticity update flag  
            max_consecutive_fires: Maximum consecutive fires allowed
            
        Returns:
            True if successful, False otherwise
        """
        if not self.genome_loaded:
            logger.warning("Cannot run fire queue processing - no genome loaded")
            return False
        
        try:
            # FCL manager uses sliding window with current timestep always 0
            current_timestep = 0  # Fixed: always use 0 for current timestep
            
            start_time = time.perf_counter()
            
            # Enhanced burst processing with power injection
            fired_neurons = self._process_burst_with_power_injection(current_timestep)
            
            processing_time = time.perf_counter() - start_time
            
            if self.debug_npu:
                logger.debug(f"Fire queue processing completed in {processing_time*1000:.2f}ms, "
                           f"{len(fired_neurons)} neurons fired")
            
            return True
            
        except Exception as e:
            logger.error(f"Error in fire queue processing: {e}")
            return False


# Export the main class and import UnifiedFQSampler from the dedicated module
from .fq_sampler import UnifiedFQSampler

# Backward compatibility aliases
FQSampler = UnifiedFQSampler
OptimizedFQSampler = UnifiedFQSampler

# Public API
__all__ = ['BurstEngine', 'UnifiedFQSampler', 'FQSampler', 'OptimizedFQSampler']