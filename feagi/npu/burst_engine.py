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

import logging
import time
from typing import Any, Dict, List, Optional

import numpy as np

# RTOS-COMPATIBLE: Removed signal and threading imports - not available in RTOS
# import signal  # REMOVED: Not compatible with RTOS
# import threading  # REMOVED: Not compatible with RTOS - use RTOS task primitives instead
# WGPU-COMPATIBLE: Remove os import to eliminate environment variable dependencies
# import os  # REMOVED: Environment variables not available in WGPU contexts
from feagi.core.state_manager import FeagiStateManager, ServiceState
from feagi.npu.fcl_injection_service import FCLInjectionService

# New imports for power area injection
from feagi.utils.logger import setup_logger

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

    def __new__(
        cls,
        connectome_manager: Any,
        fcl_manager: Optional[Any] = None,
        config: Optional[Dict[str, Any]] = None,
    ):
        """
        Singleton pattern implementation to ensure only one BurstEngine instance exists.
        """
        if cls._instance is None:
            cls._instance = super(BurstEngine, cls).__new__(cls)
            cls._instance_id = _generate_instance_id()
            # WGPU-COMPATIBLE: Use logger instead of print for debug output
            logger.info(
                f"[DEBUG] BURST ENGINE: Creating NEW singleton instance {cls._instance_id}"
            )
        else:
            # Removed log spam: no longer log when returning existing singleton
            pass
        return cls._instance

    @property
    def _running(self):
        """Get the running state with debug tracking."""
        return getattr(self, "_running_state", False)

    @_running.setter
    def _running(self, value):
        """Setter for _running with debug logging."""
        old_value = getattr(self, "_running_state", None)
        self._running_state = value

        # WGPU-COMPATIBLE: Check debug_npu config instead of environment variable
        if hasattr(self, "debug_npu") and self.debug_npu and old_value != value:
            # WGPU-COMPATIBLE: Use logger instead of print for debug output
            logger.debug(
                f"[DEBUG] BURST ENGINE: Instance {self._instance_id} _running changed: {old_value} -> {value}"
            )
            import traceback

            logger.debug("[DEBUG] BURST ENGINE: Stack trace:")
            for line in traceback.format_stack():
                logger.debug(f"    {line.strip()}")

    def __init__(
        self,
        connectome_manager: Any,
        fcl_manager: Optional[Any] = None,
        config: Optional[Dict[str, Any]] = None,
    ) -> None:
        """
        Initialize the Burst Engine.

        Args:
            connectome_manager: The connectome manager
            fcl_manager: FCL manager (optional)
            config: Configuration parameters (optional)
                   - debug_npu: Enable debug logging (replaces FEAGI_DEBUG_NPU env var)
                   - enable_injection: Enable FCL injection service for special areas
                   - desired_frequency_hz: Target frequency in Hz
        """
        # Prevent re-initialization if already initialized
        if hasattr(self, "_initialized") and self._initialized:
            # WGPU-COMPATIBLE: Use logger instead of print for debug output
            logger.info(
                f"[DEBUG] BURST ENGINE: Instance {self._instance_id} already initialized, skipping"
            )
            return

        # WGPU-COMPATIBLE: Use logger instead of print for debug output
        logger.info(
            f"[DEBUG] BURST ENGINE: Initializing singleton instance {self._instance_id}"
        )

        # Initialize logger for this instance
        self.logger = logging.getLogger(__name__ + f".BurstEngine.{self._instance_id}")

        self.connectome_manager = connectome_manager
        self.fcl_manager = fcl_manager or connectome_manager.fcl_manager
        self.config = config or {}

        # WGPU-COMPATIBLE: Check debug_npu from config only (no environment variables)
        self.debug_npu = self.config.get("debug_npu", False)

        # Log debug NPU status when enabled
        if self.debug_npu:
            logger.info(
                "[DEBUG] NPU debug mode enabled - will show detailed fire queue contents during bursts"
            )

        self.genome_loaded = False
        self._running = False  # This will now trigger the setter with debug logging
        # ARCHITECTURAL FIX: Remove internal timestep - use FeagiStateManager as single source of truth
        # self.burst_count = 0  # Replaced by state_manager.get_current_timestep()
        self.last_burst_time = 0.0

        # Initialize generic injection service (area-agnostic)
        self.injection_service: Optional[FCLInjectionService] = None

        # Generic injection configuration (no area-specific logic)
        self.enable_injection = self.config.get("enable_injection", True)

        # Initialize in a valid but inactive state
        # Will become fully operational when a genome is loaded
        logger.info("Burst Engine initialized in standby mode", status="[FAST]")

        self.state_manager = FeagiStateManager.instance()

        # Support both parameter names for backward compatibility
        self.desired_frequency = self.config.get(
            "desired_frequency_hz", self.config.get("target_frequency", 10.0)
        )  # Default to 10Hz instead of 1Hz for better performance
        self.target_frequency = self.desired_frequency  # For backward compatibility

        # Ensure frequency is never zero to avoid division by zero
        if self.desired_frequency <= 0:
            logger.warning(
                f"Invalid frequency {self.desired_frequency}Hz, using default 10Hz"
            )
            self.desired_frequency = 10.0
            self.target_frequency = 10.0

        self.burst_interval = 1.0 / self.desired_frequency

        # Log the frequency configuration for debugging
        if self.debug_npu:
            logger.info(
                f"[DEBUG] BURST ENGINE: Frequency configured: {self.desired_frequency}Hz, interval: {self.burst_interval:.4f}s"
            )

        # Use cortical_areas instead of _areas - fix the attribute name
        self.cortical_areas = (
            list(self.connectome_manager.cortical_areas.values())
            if hasattr(self.connectome_manager, "cortical_areas")
            else []
        )
        self.shed_areas = set(
            area.id
            for area in self.cortical_areas
            if area.properties.get("__shed", False)
        )

        # Initialize injection service if a genome is already loaded
        if self.cortical_areas:
            self._initialize_injection_service()

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
        logger.info(
            f"[DEBUG] BURST ENGINE: Instance {self._instance_id} initialization complete"
        )

    @classmethod
    def get_instance(cls) -> Optional["BurstEngine"]:
        """Get the current singleton instance if it exists."""
        return cls._instance

    @classmethod
    def reset_singleton(cls):
        """Reset the singleton instance. USE WITH EXTREME CAUTION - for testing only."""
        cls._instance = None
        cls._instance_id = None

    def _initialize_injection_service(self) -> None:
        """
        Initialize the FCL injection service for special area handling.

        This method sets up the injection service that handles power areas and other
        special cortical areas that need to inject neurons into the FCL during burst processing.
        """
        try:
            # Import here to avoid circular dependencies
            from feagi.npu.fcl_injection_service import FCLInjectionService
            from feagi.npu.special_area_handler import SpecialAreaHandler

            # Create special area handler
            self.special_area_handler = SpecialAreaHandler(self.connectome_manager)
            logger.info(
                f"[INJECTION INIT] Created SpecialAreaHandler for burst engine instance {self._instance_id}"
            )

            # Create FCL injection service
            self.injection_service = FCLInjectionService(
                fcl_manager=self.fcl_manager, special_area_handler=self.special_area_handler
            )
            logger.info(
                f"[INJECTION INIT] Created FCLInjectionService for burst engine instance {self._instance_id}"
            )

            # Test power area detection immediately
            try:
                power_neurons = self.special_area_handler.get_power_area_neurons()
                if power_neurons:
                    logger.info(
                        f"[INJECTION INIT] Successfully detected {len(power_neurons)} power area neurons during initialization"
                    )
                else:
                    logger.warning(
                        "[INJECTION INIT] No power area neurons detected during initialization - this may be normal if genome not loaded yet"
                    )
            except Exception as e:
                logger.error(
                    f"[INJECTION INIT] Error testing power area detection during initialization: {e}"
                )

            logger.info(
                f"[INJECTION INIT] Injection service initialization complete for burst engine {self._instance_id}"
            )

        except Exception as e:
            logger.error(
                f"[INJECTION INIT] Failed to initialize injection service for burst engine {self._instance_id}: {e}"
            )
            self.injection_service = None
            self.special_area_handler = None

    def _process_burst(self):
        """Core burst processing with embedded optimization.

        This method now uses the ultra-high-performance embedded-optimized neural update
        providing:
        - SIMD-vectorized neural operations
        - Cache-aligned memory access
        - Block-sparse connectivity optimization
        - Zero-allocation operation paths

        Designed for 10M neurons at 15Hz on single-core embedded systems.
        """
        try:
            import time

            burst_start_time = time.perf_counter()

            # PHASE 1: FCL debugging and logging
            fcl_debug_start = time.perf_counter()
            # PERFORMANCE: Simplified FCL logging - removed expensive array processing
            if hasattr(self, "fcl_manager") and self.fcl_manager and self.debug_npu:
                try:
                    # Only log FCL size occasionally instead of processing arrays every burst
                    if self.state_manager.get_current_timestep() % 100 == 0:  # Every 100 bursts
                        fcl_t_minus_1 = self.fcl_manager.get_fcl(offset=-1)
                        if fcl_t_minus_1 and not fcl_t_minus_1.is_empty():
                            logger.debug(f"🔥 FCL t-1 CONTENT: {len(fcl_t_minus_1)} total neurons")
                        else:
                            logger.debug("🔥 FCL t-1 CONTENT: EMPTY")
                except Exception as e:
                    logger.debug(f"🔥 FCL t-1 LOGGING ERROR: {e}")
            fcl_debug_time = time.perf_counter() - fcl_debug_start

            # PHASE 2: External candidates injection (all special area types)
            injection_pre_start = time.perf_counter()
            if self.injection_service and self.enable_injection:
                self.injection_service.inject_pre_burst(self.state_manager.get_current_timestep())
            injection_pre_time = time.perf_counter() - injection_pre_start

            # PHASE 3: Neural processing using new FCL auto-fire architecture  
            neural_start = time.perf_counter()
            
            # ARCHITECTURAL FIX: FeagiStateManager is now the single source of truth for timestep
            # No need to set connectome_manager.current_timestep - components get it from state manager
            pre_timestamp_start = time.perf_counter()
            current_timestep = self.state_manager.get_current_timestep()
            pre_timestamp_time = time.perf_counter() - pre_timestamp_start
            
            # MICRO-TIMING: Attribute access overhead
            attr_access_start = time.perf_counter()
            neural_processor = self.connectome_manager._neural_processor
            attr_access_time = time.perf_counter() - attr_access_start
            
            # MICRO-TIMING: Direct method call (this should be 0.04ms)
            method_call_start = time.perf_counter()
            next_burst_neurons = neural_processor.process_neural_burst(current_timestep)
            method_call_time = time.perf_counter() - method_call_start
            
            neural_time = time.perf_counter() - neural_start
            
            # Optional micro-timing (disabled for performance)
            # if neural_time > 0.005:  # Only log exceptionally slow bursts
            #     logger.info(f"BURST MICRO-TIMING (slow={neural_time*1000:.2f}ms): method={method_call_time*1000:.2f}ms")
            
            # SINGLE SOURCE OF TRUTH: Get fired data from canonical NPU tracking
            fired_count = self.connectome_manager._neural_processor.get_current_burst_fired_count()
            fired_neurons = self.connectome_manager._neural_processor.get_current_burst_fired_neurons()

            # PHASE 4: Additional injection phases if needed
            injection_during_start = time.perf_counter()
            if self.injection_service and self.enable_injection:
                self.injection_service.inject_during_burst(current_timestep)
                self.injection_service.inject_post_burst(current_timestep)
            injection_during_time = time.perf_counter() - injection_during_start

            # PHASE 5: Debug output if enabled
            debug_start = time.perf_counter()
            if self.debug_npu:
                self._debug_fire_queue_output()
            debug_time = time.perf_counter() - debug_start

            # PERFORMANCE: Log detailed burst timing breakdown
            total_burst_time = time.perf_counter() - burst_start_time
            if fired_count > 0 or total_burst_time > 0.005:
                logger.info(f"BURST ENGINE TIMING (fired={fired_count:.0f}): "
                           f"Total={total_burst_time*1000:.2f}ms, "
                           f"FCL_debug={fcl_debug_time*1000:.2f}ms, "
                           f"Pre_inject={injection_pre_time*1000:.2f}ms, "
                           f"Neural={neural_time*1000:.2f}ms, "
                           f"Post_inject={injection_during_time*1000:.2f}ms, "
                           f"Debug={debug_time*1000:.2f}ms")

            return fired_neurons

        except Exception as e:
            logger.error(f"Error in burst processing: {e}")
            if self.debug_npu:
                import traceback

                logger.error(f"Burst processing traceback: {traceback.format_exc()}")
            return []

    def _process_burst_with_power_injection(self, current_timestep: int) -> List[int]:
        """
        Enhanced burst processing with unified FCL injection model.

        This method uses the same clean architecture as _process_burst() but with
        explicit timestep parameter for compatibility. Implements the unified FCL
        candidate model where injection service adds candidates and connectome
        manager processes all candidates together.

        Args:
            current_timestep: Current simulation timestep (should be 0 for current)

        Returns:
            List of neuron IDs that fired in this burst
        """
        # PERFORMANCE: Removed disk I/O from hot path - was causing 187ms delays
        
        # Debug logging if --debug-npu is enabled
        if self.debug_npu:
            logger.debug(
                f"[DEBUG] BURST ENGINE _process_burst_with_power_injection called! Instance {self._instance_id}, Timestep: {current_timestep}"
            )

            # Check injection service availability
            if self.injection_service:
                logger.debug(
                    "[DEBUG] BURST ENGINE: Enhanced injection service AVAILABLE"
                )
            else:
                logger.debug("[DEBUG] BURST ENGINE: NO ENHANCED INJECTION SERVICE!")

        # 1. External candidates injection (pre-burst phase)
        #    Add candidates to FCL for external sources (power areas, sensory input, etc.)
        if self.injection_service:
            if self.debug_npu:
                logger.debug(
                    "[DEBUG] BURST ENGINE: Adding enhanced pre-burst candidates to FCL"
                )
            self.injection_service.inject_pre_burst(current_timestep)

        # 2. Core neural computation (synaptic propagation)
        #    Process ALL FCL candidates (internal + external) in one unified sweep
        if self.debug_npu:
            logger.debug(
                "[DEBUG] BURST ENGINE: Processing all enhanced FCL candidates (internal + external)"
            )

        fired_neurons = self.connectome_manager.update_membrane_potentials()

        if self.debug_npu:
            fired_count = len(fired_neurons) if fired_neurons else 0
            logger.debug(
                f"[DEBUG] BURST ENGINE: Enhanced processing - {fired_count} neurons fired from FCL"
            )

        # 3. Additional external injections (during-burst phase)
        #    For modulator areas or other special processing during burst
        if self.injection_service:
            if self.debug_npu:
                logger.debug(
                    "[DEBUG] BURST ENGINE: Adding enhanced during-burst candidates to FCL"
                )
            self.injection_service.inject_during_burst(current_timestep)

        # 4. Post-burst external injections
        #    For cleanup, memory consolidation, or other post-processing
        if self.injection_service:
            if self.debug_npu:
                logger.debug(
                    "[DEBUG] BURST ENGINE: Adding enhanced post-burst candidates to FCL"
                )
            self.injection_service.inject_post_burst(current_timestep)

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

        logger.info(
            f"Starting burst engine with frequency: {self.desired_frequency}Hz",
            status="[START]",
        )
        self._running = True

        # State transition
        self.state_manager.set_burst_engine_state(ServiceState.READY)

        try:
            while self._running:
                burst_cycle_start = time.perf_counter()
                processing_start = time.perf_counter()

                # Execute burst processing
                try:
                    # PERFORMANCE: Removed disk I/O proof logging - was causing delays
                    
                    # fired_neurons = self._process_burst()  # Unused variable removed
                    self._process_burst()
                    processing_end = time.perf_counter()
                    processing_duration = processing_end - processing_start

                    # Record processing timing for performance measurement
                    self._record_processing_timing(processing_duration)

                    # Debug performance if enabled
                    if hasattr(self, "debug_burst_performance"):
                        burst_cycle_end = time.perf_counter()
                        burst_duration = burst_cycle_end - burst_cycle_start
                        self.debug_burst_performance(
                            burst_duration, processing_duration
                        )

                except Exception as e:
                    logger.error(f"Error in burst processing: {e}")
                    processing_duration = time.perf_counter() - processing_start

                # ARCHITECTURAL FIX: Use FeagiStateManager as single source of truth for timestep
                current_timestep = self.state_manager.advance_timestep()
                self.last_burst_time = time.time()

                # RTOS-COMPATIBLE: Precise timing control
                cycle_end = time.perf_counter()
                cycle_duration = cycle_end - burst_cycle_start

                # Record full cycle timing for frequency measurement
                self._record_burst_timing(cycle_duration)

                # Calculate sleep time to maintain target frequency
                if cycle_duration < self.burst_interval:
                    # sleep_time = self.burst_interval - cycle_duration  # Unused variable removed
                    # RTOS-COMPATIBLE: Use deterministic timing
                    target_time = burst_cycle_start + self.burst_interval
                    while time.perf_counter() < target_time:
                        pass  # Busy-wait for precise timing
                else:
                    # Running behind schedule
                    if self.debug_npu:
                        logger.warning(
                            f"Burst cycle took {cycle_duration * 1000:.2f}ms (target: {self.burst_interval * 1000:.2f}ms)"
                        )

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

            logger.info(
                f"Test burst completed in {test_duration * 1000:.2f}ms, "
                f"{len(fired_neurons)} neurons fired",
                status="[TEST]",
            )

            # ARCHITECTURAL FIX: Use FeagiStateManager as single source of truth for timestep
            self.state_manager.advance_timestep()

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
        # PERFORMANCE: Removed disk I/O from hot path
        
        try:
            # Reinitialize injection service with new genome
            self._initialize_injection_service()
            logger.info("Burst engine updated with new genome successfully")
        except Exception as e:
            logger.error(f"Failed to update burst engine with new genome: {e}")

        # Also update line 772 area
        if hasattr(self, 'injection_service') and self.injection_service:
            service_type = type(self.injection_service).__name__
            logger.info(
                f"[INJECTION INIT] Injection service initialized: {service_type}"
            )
            # PERFORMANCE: Removed disk I/O debug logging - violates RTOS/embedded rules

        # Get current cortical areas for comparison
        new_cortical_areas = (
            list(self.connectome_manager.cortical_areas.values())
            if hasattr(self.connectome_manager, "cortical_areas")
            else []
        )
        new_shed_areas = set(
            area.id
            for area in new_cortical_areas
            if area.properties.get("__shed", False)
        )

        # Always ensure injection service is initialized when genome is loaded
        # Update cortical areas from connectome
        self.cortical_areas = new_cortical_areas
        self.shed_areas = new_shed_areas

        # Always initialize injection service to ensure proper special area detection
        logger.debug(
            "[DEBUG] BURST ENGINE: Re-initializing injection service with genome data"
        )

        self._initialize_injection_service()

        service_type = (
            type(self.injection_service).__name__
            if self.injection_service
            else "None"
        )
        logger.debug(
            f"[DEBUG] BURST ENGINE: Injection service re-initialized, current service: {service_type}"
        )
        # PERFORMANCE: Removed disk I/O debug logging - violates RTOS/embedded rules

        # Mark genome as loaded
        self.genome_loaded = True

        logger.info(
            f"Burst engine updated: {len(self.cortical_areas)} cortical areas, "
            f"{len(self.shed_areas)} shed areas",
            status="[CONFIG]",
        )

    def refresh_special_areas(self) -> None:
        """
        Refresh special area detection and injection service configuration.

        This method can be called to re-detect special areas after configuration changes.
        Completely area-agnostic - handles all special area types (power, modulator, sensory, etc.)
        """
        logger.info("Refreshing injection service configuration", status="[CONFIG]")

        try:
            if self.injection_service:
                # Refresh injection batches for all detected special areas
                self.injection_service.refresh_injection_batches()
            else:
                # Re-initialize injection service if not already initialized
                self._initialize_injection_service()

        except Exception as e:
            logger.error(f"Error refreshing injection service: {e}")

    def get_injection_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about FCL injection service (all special area types).

        Returns:
            Dictionary containing injection statistics for all special areas
        """
        if not self.injection_service:
            return {"error": "Injection service not available"}

        try:
            return self.injection_service.get_statistics()
        except Exception as e:
            logger.error(f"Error getting injection statistics: {e}")
            return {"error": str(e)}

    def set_injection_enabled(self, cortical_id: str, enabled: bool) -> bool:
        """
        Enable or disable injection for a specific cortical area.

        Works for any special area type (power, modulator, sensory, etc.)

        Args:
            cortical_id: ID of the cortical area
            enabled: Whether to enable injection for this area

        Returns:
            True if successful, False otherwise
        """
        if not self.injection_service:
            logger.warning("Injection service not available")
            return False

        try:
            return self.injection_service.set_injection_enabled(cortical_id, enabled)
        except Exception as e:
            logger.error(f"Error setting injection for {cortical_id}: {e}")
            return False

    def update_frequency(self, frequency_hz: float) -> bool:
        """
        Update burst frequency - RTOS-safe, no dynamic allocation.

        Args:
            frequency_hz: New frequency in Hz (must be > 0)

        Returns:
            True if successful, False otherwise
        """
        # RTOS-SAFE: Validate input without exceptions in normal case
        if frequency_hz <= 0.0 or frequency_hz > 10000.0:  # Max 10kHz for safety
            return False

        # RTOS-SAFE: Atomic updates, no intermediate invalid state
        self.desired_frequency = frequency_hz
        self.target_frequency = frequency_hz
        self.burst_interval = 1.0 / frequency_hz

        # RTOS-SAFE: Minimal logging only if debug enabled
        if self.debug_npu:
            logger.info(f"[DEBUG] BURST ENGINE: Frequency updated to {frequency_hz}Hz")

        return True

    def get_frequency_config(self) -> Dict[str, float]:
        """
        Get current frequency configuration - RTOS-safe, no allocation.

        Returns:
            Dictionary with current frequency settings
        """
        return {
            "current_frequency_hz": self.desired_frequency,
            "burst_interval_seconds": self.burst_interval,
            "target_frequency_hz": self.target_frequency,
        }

    def run_with_fire_queue(
        self, mpf: bool = True, puf: bool = False, max_consecutive_fires: int = 10
    ) -> bool:
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
            # PERFORMANCE: Removed disk I/O debug logging - violates RTOS/embedded rules

            # FCL manager uses sliding window with current timestep always 0
            current_timestep = 0  # Fixed: always use 0 for current timestep

            start_time = time.perf_counter()

            # Enhanced burst processing with power injection
            fired_neurons = self._process_burst_with_power_injection(current_timestep)

            processing_time = time.perf_counter() - start_time

            if self.debug_npu:
                logger.debug(
                    f"Fire queue processing completed in {processing_time * 1000:.2f}ms, "
                    f"{len(fired_neurons)} neurons fired"
                )

            return True

        except Exception as e:
            logger.error(f"Error in fire queue processing: {e}")
            return False


# Export the main class and import UnifiedFQSampler from the dedicated module
from .fq_sampler import UnifiedFQSampler

# Public API
__all__ = ["BurstEngine", "UnifiedFQSampler"]
