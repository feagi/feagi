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
from feagi.npu.memory_processor import MemoryProcessor

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
    - Power Area Support: Handles special cortical areas like "_power" with automatic injection
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
        # Check if NPU debug mode is enabled via --debug-npu flag
        state_manager = FeagiStateManager.instance()
        if state_manager.is_debug_npu_enabled() and old_value != value:
            # WGPU-COMPATIBLE: Use logger instead of print for debug output
            logger.info(
                f"[NPU-DEBUG] BURST ENGINE: Instance {self._instance_id} _running changed: {old_value} -> {value}"
            )
            import traceback

            logger.info("[NPU-DEBUG] BURST ENGINE: Stack trace:")
            for line in traceback.format_stack():
                logger.info(f"    {line.strip()}")

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
        self.logger = logging.getLogger(
            __name__ + f".BurstEngine.{self._instance_id}"
        )

        self.connectome_manager = connectome_manager
        self.fcl_manager = fcl_manager or connectome_manager.fcl_manager
        self.config = config or {}

        # Initialize MemoryProcessor for memory cortical areas
        self.memory_processor = None
        self._initialize_memory_processor()

        # WGPU-COMPATIBLE: Check debug_npu from config only (no environment variables)
        self.debug_npu = self.config.get("debug_npu", False)

        # Log debug NPU status when enabled
        if self.debug_npu:
            logger.info(
                "[DEBUG] NPU debug mode enabled - will show detailed fire queue contents during bursts"
            )

        self.genome_loaded = False
        self._running = (
            False  # This will now trigger the setter with debug logging
        )
        self.burst_count = 0
        self.last_burst_time = 0.0

        # Initialize generic injection service (area-agnostic)
        self.injection_service: Optional[FCLInjectionService] = None

        # Generic injection configuration (no area-specific logic)
        self.enable_injection = self.config.get("enable_injection", True)

        # Initialize in a valid but inactive state
        # Will become fully operational when a genome is loaded
        logger.info(
            "Burst Engine initialized in standby mode", status="[FAST]"
        )

        self.state_manager = FeagiStateManager.instance()

        # STATE MANAGER is the SINGLE SOURCE OF TRUTH for burst frequency
        # Only use config as emergency fallback during initialization
        config_frequency = self.config.get(
            "desired_frequency_hz", self.config.get("target_frequency", 10.0)
        )

        # Get frequency from state manager (authoritative source)
        try:
            state_frequency = self.state_manager.get_burst_frequency()
            if state_frequency and state_frequency > 0:
                self.desired_frequency = state_frequency
                logger.info(
                    f"[BURST ENGINE] Using state manager frequency: {state_frequency}Hz"
                )
            else:
                # Emergency fallback: use config and update state manager
                self.desired_frequency = config_frequency
                self.state_manager.set_burst_frequency(config_frequency)
                logger.warning(
                    f"[BURST ENGINE] State manager frequency invalid ({state_frequency}Hz) - "
                    f"using config fallback: {config_frequency}Hz and updating state manager"
                )
        except Exception as e:
            # Emergency fallback: use config frequency
            self.desired_frequency = config_frequency
            logger.warning(
                f"[BURST ENGINE] Failed to get frequency from state manager ({e}) - "
                f"using config fallback: {config_frequency}Hz"
            )

        self.target_frequency = (
            self.desired_frequency
        )  # For backward compatibility

        # Ensure frequency is never zero to avoid division by zero
        if self.desired_frequency <= 0:
            logger.warning(
                f"Invalid frequency {self.desired_frequency}Hz, using default 10Hz"
            )
            self.desired_frequency = 10.0
            self.target_frequency = 10.0
            # Update state manager with corrected value
            try:
                self.state_manager.set_burst_frequency(10.0)
            except Exception:
                pass  # Emergency fallback - don't fail initialization

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
            special_area_handler = SpecialAreaHandler(self.connectome_manager)
            logger.info(
                f"[INJECTION INIT] Created SpecialAreaHandler for burst engine instance {self._instance_id}"
            )

            # Create FCL injection service
            self.injection_service = FCLInjectionService(
                fcl_manager=self.fcl_manager,
                special_area_handler=special_area_handler,
            )
            logger.info(
                f"[INJECTION INIT] Created FCLInjectionService for burst engine instance {self._instance_id}"
            )

            # Test power area detection immediately
            try:
                power_neurons = special_area_handler.get_power_area_neurons()
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
        # CRITICAL FIX: Initialize state_manager once at method start to prevent
        # "cannot access local variable" errors when exceptions occur
        state_manager = FeagiStateManager.instance()

        try:
            import time

            burst_start_time = time.perf_counter()

            # LOG RAW FCL t-1 CONTENT FOR DEBUGGING VISUALIZATION ISSUES
            if (
                hasattr(self, "fcl_manager")
                and self.fcl_manager
                and state_manager.is_debug_npu_enabled()
            ):
                try:
                    # Get FCL from previous timestep (t-1) - this is what FQ sampler reads
                    fcl_t_minus_1 = self.fcl_manager.get_fcl(offset=-1)
                    if fcl_t_minus_1 and not fcl_t_minus_1.is_empty():
                        neuron_list = list(fcl_t_minus_1)[
                            :10
                        ]  # Show first 10 neurons
                        logger.info(
                            f"🔥 [NPU-DEBUG] FCL t-1 CONTENT: {len(fcl_t_minus_1)} total neurons, first 10: {neuron_list}"
                        )

                        # Show which cortical areas these neurons belong to using vectorized operation
                        if hasattr(
                            self.connectome_manager, "neuron_array"
                        ) and hasattr(
                            self.connectome_manager.neuron_array,
                            "cortical_area_id",
                        ):
                            # Convert FCL to numpy array for vectorized processing
                            neuron_ids_array = np.array(
                                list(fcl_t_minus_1), dtype=np.int32
                            )
                            max_idx = len(
                                self.connectome_manager.neuron_array.cortical_area_id
                            )

                            # Vectorized bounds checking and area extraction
                            valid_mask = neuron_ids_array < max_idx
                            valid_neuron_ids = neuron_ids_array[valid_mask]

                            if len(valid_neuron_ids) > 0:
                                # Vectorized area extraction
                                areas = self.connectome_manager.neuron_array.cortical_area_id[
                                    valid_neuron_ids
                                ]

                                # Vectorized string conversion
                                area_strs = np.array(
                                    [
                                        (
                                            area.decode("utf-8")
                                            if isinstance(area, bytes)
                                            else str(area)
                                        )
                                        for area in areas
                                    ]
                                )

                                # Count unique areas using NumPy
                                unique_areas, counts = np.unique(
                                    area_strs, return_counts=True
                                )
                                area_count = dict(
                                    zip(unique_areas[:5], counts[:5])
                                )  # First 5 areas

                                logger.info(
                                    f"🔥 [NPU-DEBUG] FCL t-1 AREAS: {area_count}..."
                                )  # Show first 5 areas
                    else:
                        logger.info("🔥 [NPU-DEBUG] FCL t-1 CONTENT: EMPTY")
                except Exception as e:
                    logger.info(f"🔥 [NPU-DEBUG] FCL t-1 LOGGING ERROR: {e}")

            # 1. External candidates injection (all special area types)
            if self.injection_service and self.enable_injection:
                self.injection_service.inject_pre_burst(self.burst_count)

            # 2. Unified neural computation using embedded optimizations
            # This now automatically uses SIMD, cache-aligned arrays, and block-sparse matrices
            fired_neurons = self.connectome_manager.update_membrane_potentials(
                current_timestep=self.burst_count
            )

            # 3. Additional injection phases if needed
            if self.injection_service and self.enable_injection:
                self.injection_service.inject_during_burst(self.burst_count)
                self.injection_service.inject_post_burst(self.burst_count)

            # 4. Debug output if enabled
            if self.debug_npu:
                self._debug_fire_queue_output()

            # 5. Performance tracking for embedded optimization
            burst_time = time.perf_counter() - burst_start_time

            # Log performance periodically for embedded systems
            if self.burst_count % 100 == 0:  # Every 100 bursts
                perf_summary = self.connectome_manager.neuron_array.get_performance_summary()
                avg_burst_time_ms = burst_time * 1000

                if avg_burst_time_ms < 66.7:  # Under 15Hz target
                    status = "✅ TARGET"
                elif avg_burst_time_ms < 100:  # Under 10Hz
                    status = "⚠️  CLOSE"
                else:
                    status = "❌ SLOW"

                logger.info(
                    f"EMBEDDED BURST PERFORMANCE [Burst {self.burst_count}]: "
                    f"{avg_burst_time_ms:.2f}ms ({status}), "
                    f"fired: {len(fired_neurons)}, "
                    f"SIMD: {perf_summary.get('simd_enabled', False)}"
                )

            return fired_neurons

        except Exception as e:
            logger.error(f"Error in burst processing: {e}")
            # CRITICAL DEBUG: Always log traceback for state_manager errors to identify source
            import traceback

            full_traceback = traceback.format_exc()
            logger.error(f"BURST PROCESSING TRACEBACK:\n{full_traceback}")

            if self.debug_npu:
                logger.error(
                    f"Additional burst processing debug info: {full_traceback}"
                )
            return []

    def _process_burst_with_power_injection(
        self, current_timestep: int
    ) -> List[int]:
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
        # CRITICAL FIX: Initialize state_manager to prevent NameError
        state_manager = FeagiStateManager.instance()

        # Debug-only proof of execution
        try:
            if FeagiStateManager.instance().is_debug_npu_enabled():
                import datetime
                import os
                import tempfile

                log_dir = tempfile.gettempdir()
                log_path = os.path.join(
                    log_dir, "feagi_enhanced_burst--temp.log"
                )
                with open(log_path, "a") as f:
                    f.write(
                        f"{datetime.datetime.now()}: _process_burst_with_power_injection called, "
                        f"timestep={current_timestep}, injection_service="
                        f"{type(self.injection_service).__name__ if self.injection_service else 'None'}\n"
                    )
        except Exception:
            pass
        # Debug logging if --debug-npu is enabled
        if state_manager.is_debug_npu_enabled():
            logger.info(
                f"[NPU-DEBUG] BURST ENGINE _process_burst_with_power_injection called! Instance {self._instance_id}, Timestep: {current_timestep}"
            )

            # Check injection service availability
            if self.injection_service:
                logger.info(
                    "[NPU-DEBUG] BURST ENGINE: Enhanced injection service AVAILABLE"
                )
            else:
                logger.info(
                    "[NPU-DEBUG] BURST ENGINE: NO ENHANCED INJECTION SERVICE!"
                )

        # 1. External candidates injection (pre-burst phase)
        #    Add candidates to FCL for external sources (power areas, sensory input, etc.)
        if self.injection_service:
            if state_manager.is_debug_npu_enabled():
                logger.info(
                    "[NPU-DEBUG] BURST ENGINE: Adding enhanced pre-burst candidates to FCL"
                )
            self.injection_service.inject_pre_burst(current_timestep)

        # 2. Core neural computation (synaptic propagation)
        #    Process ALL FCL candidates (internal + external) in one unified sweep
        if state_manager.is_debug_npu_enabled():
            logger.info(
                "[NPU-DEBUG] BURST ENGINE: Processing all enhanced FCL candidates (internal + external)"
            )

        fired_neurons = self.connectome_manager.update_membrane_potentials()

        if state_manager.is_debug_npu_enabled():
            fired_count = len(fired_neurons) if fired_neurons else 0
            logger.info(
                f"[NPU-DEBUG] BURST ENGINE: Enhanced processing - {fired_count} neurons fired from FCL"
            )

        # 3. Additional external injections (during-burst phase)
        #    For modulator areas or other special processing during burst
        if self.injection_service:
            if state_manager.is_debug_npu_enabled():
                logger.info(
                    "[NPU-DEBUG] BURST ENGINE: Adding enhanced during-burst candidates to FCL"
                )
            self.injection_service.inject_during_burst(current_timestep)

        # 4. Post-burst external injections
        #    For cleanup, memory consolidation, or other post-processing
        if self.injection_service:
            if state_manager.is_debug_npu_enabled():
                logger.info(
                    "[NPU-DEBUG] BURST ENGINE: Adding enhanced post-burst candidates to FCL"
                )
            self.injection_service.inject_post_burst(current_timestep)

        # 5. Memory processing for memory cortical areas
        #    Process temporal patterns and manage memory neuron lifecycle
        if self.memory_processor:
            if state_manager.is_debug_npu_enabled():
                logger.info(
                    "[NPU-DEBUG] BURST ENGINE: Processing memory areas for temporal patterns"
                )
                logger.info(
                    f"[NPU-DEBUG] Active memory areas: {list(self.memory_processor.active_memory_areas)}"
                )
            self._process_memory_areas(current_timestep)
        else:
            if state_manager.is_debug_npu_enabled():
                logger.info(
                    "[NPU-DEBUG] BURST ENGINE: No MemoryProcessor - skipping memory processing"
                )

        # 6. Debug fire queue output if --debug-npu flag is enabled
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
                    # Unconditional proof that run loop is executing
                    if (
                        self.burst_count % 100 == 0
                    ):  # Every 100 bursts to avoid spam
                        try:
                            from feagi.core.state_manager import (
                                FeagiStateManager,
                            )

                            if FeagiStateManager.instance().is_debug_npu_enabled():
                                import datetime
                                import os
                                import tempfile

                                log_path = os.path.join(
                                    tempfile.gettempdir(),
                                    "feagi_run_loop--temp.log",
                                )
                                with open(log_path, "a") as f:
                                    f.write(
                                        f"{datetime.datetime.now()}: run() loop executing, about to call _process_burst(), burst_count={self.burst_count}\n"
                                    )
                        except Exception:
                            pass

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
                    # CRITICAL DEBUG: Always log traceback for state_manager errors to identify source
                    import traceback

                    full_traceback = traceback.format_exc()
                    logger.error(
                        f"BURST PROCESSING TRACEBACK (run loop):\n{full_traceback}"
                    )
                    processing_duration = (
                        time.perf_counter() - processing_start
                    )

                self.burst_count += 1
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
        # Debug-only file write for development tracking
        try:
            from feagi.core.state_manager import FeagiStateManager

            if FeagiStateManager.instance().is_debug_npu_enabled():
                import datetime
                import os
                import tempfile

                log_path = os.path.join(
                    tempfile.gettempdir(), "feagi_injection_debug--temp.log"
                )
                with open(log_path, "a") as f:
                    f.write(
                        f"{datetime.datetime.now()}: update_with_genome() called\n"
                    )
        except Exception:
            pass

        try:
            # CRITICAL FIX: Synchronize neuron array data to prevent size mismatches
            # This fixes the "(13452,) (13846,) (13452,)" broadcasting error
            if hasattr(self.connectome_manager, "neuron_array"):
                neuron_array = self.connectome_manager.neuron_array
                actual_neuron_count = neuron_array.neuron_count
                valid_neurons = (
                    int(np.sum(neuron_array.valid_mask))
                    if hasattr(neuron_array, "valid_mask")
                    else actual_neuron_count
                )

                logger.info(
                    f"[SYNC FIX] Neuron array sync: {actual_neuron_count} total neurons, {valid_neurons} valid neurons"
                )

                # CRITICAL: Ensure valid_mask consistency with neuron_count
                if valid_neurons != actual_neuron_count:
                    logger.warning(
                        f"[SYNC FIX] Valid mask mismatch detected: {valid_neurons} valid vs {actual_neuron_count} total"
                    )

                    # Force valid_mask synchronization
                    valid_mask = (
                        self.connectome_manager.neuron_array.backend.to_numpy(
                            neuron_array.valid_mask
                        )
                    )

                    # Count actual valid entries in ID mapping as source of truth
                    actual_valid_count = len(
                        self.connectome_manager.neuron_id_to_index
                    )
                    logger.info(
                        f"[SYNC FIX] ID mapping reports {actual_valid_count} neurons"
                    )

                    # Rebuild valid_mask based on actual ID mappings (source of truth)
                    corrected_valid_mask = np.zeros_like(
                        valid_mask, dtype=bool
                    )
                    # GPU/SIMD-friendly vectorized operation - no Python loops!
                    indices = np.array(
                        list(
                            self.connectome_manager.neuron_id_to_index.values()
                        )
                    )
                    valid_indices = indices[
                        (indices >= 0) & (indices < len(corrected_valid_mask))
                    ]
                    corrected_valid_mask[valid_indices] = True

                    # Update the valid_mask in the neuron array
                    neuron_array.valid_mask = neuron_array.backend.array(
                        corrected_valid_mask
                    )
                    neuron_array.neuron_count = actual_valid_count

                    logger.info(
                        f"[SYNC FIX] Corrected valid_mask: {np.sum(corrected_valid_mask)} valid neurons"
                    )

                # CRITICAL: Invalidate any cached arrays in burst engine to force refresh
                if hasattr(self, "_cached_valid_neurons"):
                    delattr(self, "_cached_valid_neurons")
                if hasattr(self, "_cached_neuron_count"):
                    delattr(self, "_cached_neuron_count")

                logger.info(
                    "[SYNC FIX] Neuron array synchronization completed",
                    status="[OK]",
                )

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
            state_manager = FeagiStateManager.instance()
            if state_manager.is_debug_npu_enabled():
                logger.info(
                    "[NPU-DEBUG] BURST ENGINE: Re-initializing injection service with genome data"
                )

            self._initialize_injection_service()

            service_type = (
                type(self.injection_service).__name__
                if self.injection_service
                else "None"
            )
            if state_manager.is_debug_npu_enabled():
                logger.info(
                    f"[NPU-DEBUG] BURST ENGINE: Injection service re-initialized, current service: {service_type}"
                )
            # Debug-only file write
            try:
                from feagi.core.state_manager import FeagiStateManager

                if FeagiStateManager.instance().is_debug_npu_enabled():
                    import datetime
                    import os
                    import tempfile

                    log_path = os.path.join(
                        tempfile.gettempdir(),
                        "feagi_injection_debug--temp.log",
                    )
                    with open(log_path, "a") as f:
                        f.write(
                            f"{datetime.datetime.now()}: Injection service after init: {service_type}\n"
                        )
            except Exception:
                pass

            # Mark genome as loaded
            self.genome_loaded = True

            logger.info(
                f"Burst engine updated: {len(self.cortical_areas)} cortical areas, "
                f"{len(self.shed_areas)} shed areas",
                status="[CONFIG]",
            )

        except Exception as e:
            logger.error(f"Error updating burst engine with genome: {e}")
            self.genome_loaded = False

    def refresh_special_areas(self) -> None:
        """
        Refresh special area detection and injection service configuration.

        This method can be called to re-detect special areas after configuration changes.
        Completely area-agnostic - handles all special area types (power, modulator, sensory, etc.)
        """
        logger.info(
            "Refreshing injection service configuration", status="[CONFIG]"
        )

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
            return self.injection_service.set_injection_enabled(
                cortical_id, enabled
            )
        except Exception as e:
            logger.error(f"Error setting injection for {cortical_id}: {e}")
            return False

    def update_frequency(self, frequency_hz: float) -> bool:
        """
        Update burst frequency - RTOS-safe, no dynamic allocation.

        IMPORTANT: This should only be called by CoreAPIService which manages
        the state manager update. The frequency should come FROM state manager.

        Args:
            frequency_hz: New frequency in Hz (must be > 0)

        Returns:
            True if successful, False otherwise
        """
        # RTOS-SAFE: Validate input without exceptions in normal case
        if (
            frequency_hz <= 0.0 or frequency_hz > 10000.0
        ):  # Max 10kHz for safety
            return False

        # RTOS-SAFE: Atomic updates, no intermediate invalid state
        self.desired_frequency = frequency_hz
        self.target_frequency = frequency_hz
        self.burst_interval = 1.0 / frequency_hz

        # RTOS-SAFE: Minimal logging only if debug enabled
        if self.debug_npu:
            logger.info(
                f"[DEBUG] BURST ENGINE: Local frequency updated to {frequency_hz}Hz (from state manager)"
            )

        return True

    def get_frequency_config(self) -> Dict[str, float]:
        """
        Get current frequency configuration from STATE MANAGER (authoritative source).

        Returns:
            Dictionary with current frequency settings from state manager
        """
        try:
            # STATE MANAGER is the single source of truth
            state_frequency = self.state_manager.get_burst_frequency()
            if state_frequency and state_frequency > 0:
                return {
                    "current_frequency_hz": state_frequency,
                    "burst_interval_seconds": 1.0 / state_frequency,
                    "target_frequency_hz": state_frequency,
                }
            else:
                # Emergency fallback to local values
                return {
                    "current_frequency_hz": self.desired_frequency,
                    "burst_interval_seconds": self.burst_interval,
                    "target_frequency_hz": self.target_frequency,
                    "warning": "Using local fallback - state manager frequency invalid",
                }
        except Exception as e:
            # Emergency fallback to local values
            return {
                "current_frequency_hz": self.desired_frequency,
                "burst_interval_seconds": self.burst_interval,
                "target_frequency_hz": self.target_frequency,
                "error": f"Failed to get frequency from state manager: {e}",
            }

    def run_with_fire_queue(
        self,
        mpf: bool = True,
        puf: bool = False,
        max_consecutive_fires: int = 10,
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
            logger.warning(
                "Cannot run fire queue processing - no genome loaded"
            )
            return False

        try:
            # Unconditional proof that run_with_fire_queue is being called
            try:
                with open("/tmp/feagi_fire_queue.log", "a") as f:
                    import datetime

                    f.write(
                        f"{datetime.datetime.now()}: run_with_fire_queue called, about to call _process_burst_with_power_injection\n"
                    )
            except Exception:
                pass

            # Derive current timestep from FCL manager if available; otherwise start at 0
            current_timestep = (
                self.fcl_manager.current_timestep + 1
                if self.fcl_manager
                else 0
            )

            start_time = time.perf_counter()

            # Enhanced burst processing with power injection
            fired_neurons = self._process_burst_with_power_injection(
                current_timestep
            )

            processing_time = time.perf_counter() - start_time

            state_manager = FeagiStateManager.instance()
            if state_manager.is_debug_npu_enabled():
                logger.info(
                    f"[NPU-DEBUG] Fire queue processing completed in {processing_time * 1000:.2f}ms, "
                    f"{len(fired_neurons)} neurons fired"
                )

            return True

        except Exception as e:
            logger.error(f"Error in fire queue processing: {e}")
            return False

    def _initialize_memory_processor(self) -> None:
        """Initialize the memory processor if ConnectomeManager has memory_neuron_array."""
        try:
            # Check if we have access to the memory neuron array
            if hasattr(self.connectome_manager, "memory_neuron_array"):
                memory_config = self.config.get("memory_processing", {})
                batch_size = memory_config.get("batch_size", 100)
                cache_size = memory_config.get("pattern_cache_size", 10000)

                logger.info(
                    "[MEMORY-INIT] Starting MemoryProcessor initialization..."
                )
                logger.info(
                    f"[MEMORY-INIT] Config: batch_size={batch_size}, cache_size={cache_size}"
                )
                logger.info(
                    f"[MEMORY-INIT] ConnectomeManager type: {type(self.connectome_manager)}"
                )
                logger.info(
                    f"[MEMORY-INIT] Has memory_neuron_array: {hasattr(self.connectome_manager, 'memory_neuron_array')}"
                )

                if hasattr(self.connectome_manager, "memory_neuron_array"):
                    array_capacity = getattr(
                        self.connectome_manager.memory_neuron_array,
                        "capacity",
                        "unknown",
                    )
                    logger.info(
                        f"[MEMORY-INIT] Memory neuron array capacity: {array_capacity}"
                    )

                # DEBUG: Check parameters before MemoryProcessor call
                logger.info("[MEMORY-INIT] About to create MemoryProcessor...")
                logger.info(
                    f"[MEMORY-INIT] memory_neuron_array type: {type(self.connectome_manager.memory_neuron_array)}"
                )
                logger.info(
                    f"[MEMORY-INIT] fcl_manager type: {type(self.fcl_manager)}"
                )
                logger.info(
                    f"[MEMORY-INIT] fcl_manager is None: {self.fcl_manager is None}"
                )

                self.memory_processor = MemoryProcessor(
                    memory_neuron_array=self.connectome_manager.memory_neuron_array,
                    fcl_manager=self.fcl_manager,
                    batch_size=batch_size,
                    pattern_cache_size=cache_size,
                    connectome_manager=self.connectome_manager,
                )

                logger.info(
                    "[MEMORY-INIT] MemoryProcessor constructor completed successfully"
                )

                logger.info(
                    f"[OK] MemoryProcessor initialized with batch_size={batch_size}, cache_size={cache_size}"
                )
            else:
                logger.info(
                    "[MEMORY-INIT] ConnectomeManager doesn't have memory_neuron_array - MemoryProcessor not initialized"
                )
                self.memory_processor = None

        except Exception as e:
            logger.error(
                f"🧠 [MEMORY] Error initializing MemoryProcessor: {e}"
            )
            self.memory_processor = None

    def _process_memory_areas(self, current_timestep: int) -> None:
        """Process memory areas for temporal pattern detection."""
        try:
            npu_debug = (
                self.state_manager.is_debug_npu_enabled()
                if self.state_manager
                else False
            )

            if npu_debug:
                logger.info(
                    "🧠 [MEMORY] BURST ENGINE: Processing memory areas for temporal patterns"
                )

            if not self.memory_processor:
                if npu_debug:
                    logger.info(
                        "🧠 [MEMORY] BURST ENGINE: No MemoryProcessor - skipping memory processing"
                    )
                return

            if npu_debug:
                active_areas = (
                    list(self.memory_processor.active_memory_areas)
                    if hasattr(self.memory_processor, "active_memory_areas")
                    else []
                )
                logger.info(f"🧠 [MEMORY] Active memory areas: {active_areas}")

            # Process memory areas aligned with FCL's current timestep to avoid window mismatches
            fcl_current_timestep = (
                self.fcl_manager.current_timestep
                if self.fcl_manager
                else current_timestep
            )
            memory_stats = self.memory_processor.process_memory_areas_batch(
                fcl_current_timestep
            )

            if npu_debug:
                logger.info(
                    f"🧠 [MEMORY] Memory processing: {memory_stats}, time: {memory_stats.get('processing_time_ms', 0):.2f}ms"
                )

        except Exception as e:
            logger.error(f"🧠 [MEMORY] Error starting memory processing: {e}")

    def register_memory_area_with_processor(
        self, cortical_id: str, properties: Dict[str, Any]
    ) -> bool:
        """Register a memory area with the memory processor."""
        if not self.memory_processor:
            # CRITICAL FIX: Retry MemoryProcessor initialization if it failed due to timing
            logger.info(
                "🔧 [MEMORY-FIX] MemoryProcessor is None, attempting reinitialization..."
            )
            self._initialize_memory_processor()

            if not self.memory_processor:
                logger.error(
                    "🔧 [MEMORY-FIX] MemoryProcessor reinitialization failed"
                )
                return False
            else:
                logger.info(
                    "🔧 [MEMORY-FIX] MemoryProcessor reinitialization SUCCESS!"
                )

        if not self.memory_processor:
            return False

        try:
            # Extract memory properties
            temporal_depth = properties.get("temporal_depth", 1)
            initial_lifespan = properties.get("init_lifespan", 9)
            lifespan_growth_rate = properties.get("lifespan_growth_rate", 1.0)
            longterm_threshold = properties.get("longterm_mem_threshold", 100)

            # Get upstream areas from ConnectomeManager
            upstream_areas = set()
            if hasattr(
                self.connectome_manager, "get_upstream_areas_for_memory"
            ):
                upstream_areas = (
                    self.connectome_manager.get_upstream_areas_for_memory(
                        cortical_id
                    )
                )

            return self.memory_processor.register_memory_area(
                cortical_id=cortical_id,
                temporal_depth=temporal_depth,
                initial_lifespan=initial_lifespan,
                lifespan_growth_rate=lifespan_growth_rate,
                longterm_threshold=longterm_threshold,
                upstream_areas=upstream_areas,
            )
        except Exception as e:
            logger.error(
                f"🧠 [MEMORY] Error registering memory area {cortical_id}: {e}"
            )
            return False

    def unregister_memory_area_from_processor(self, cortical_id: str) -> bool:
        """Unregister a memory area from the memory processor."""
        if not self.memory_processor:
            return False

        try:
            return self.memory_processor.unregister_memory_area(cortical_id)
        except Exception as e:
            logger.error(
                f"🧠 [MEMORY] Error unregistering memory area {cortical_id}: {e}"
            )
            return False

    def get_memory_processing_statistics(self) -> Optional[Dict[str, Any]]:
        """Get memory processing statistics."""
        if not self.memory_processor:
            return None

        try:
            return self.memory_processor.get_processing_statistics()
        except Exception as e:
            logger.error(f"Error getting memory processing statistics: {e}")
            return None


# Export the main class and import UnifiedFQSampler from the dedicated module
from .fq_sampler import UnifiedFQSampler

# Public API
__all__ = ["BurstEngine", "UnifiedFQSampler"]
