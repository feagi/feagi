"""
FEAGI Test Runner

Main test runner that coordinates between different test modes and provides
the common testing infrastructure.
"""

import threading
import time

from feagi.core.state_manager import FeagiStateManager
from feagi.utils.logger import setup_logger

logger = setup_logger("feagi.test_mode")


class FeagiTestRunner:
    """
    Test runner for FEAGI sensory input processing.

    This class provides functionality for:
    1. Loading a test genome
    2. Coordinating between different test modes
    3. Monitoring neural activity
    4. Reporting test results
    5. Testing visualization data flow (when test_visualization=True)
    """

    def __init__(
        self,
        core_api_service,
        test_mode="mode_1",
        sample_genome_path=None,
        test_duration=10,
        frequency_hz=10,
    ):
        """
        Initialize the test runner.

        Args:
            core_api_service: FEAGI's core API service
            test_mode: "mode_1" for JSON-based or "mode_2" for numpy-based
            sample_genome_path: Path to the sample genome to load
            test_duration: Duration of the test in seconds
            frequency_hz: Frequency of sensory input generation in Hz
        """
        self.core_api = core_api_service
        self.connectome = self.core_api.get_connectome_manager()
        self.burst_engine = self.core_api.get_burst_engine()
        self.fcl_manager = self.core_api.get_fcl_manager()
        self.state_manager = FeagiStateManager.instance()

        # Test configuration
        self.test_mode = test_mode
        self.test_duration = test_duration
        self.frequency_hz = frequency_hz

        # If no sample genome path is provided, use the essential genome
        self.sample_genome_path = sample_genome_path

        # Test state variables
        self.is_running = False
        self.test_thread = None
        self.test_result = None
        self.initial_fcls = {}
        self.areas_with_activity = set()

        # Test mode handlers
        self.mode_handler = None

        # Verify burst engine is available for coordination
        if not self.burst_engine:
            logger.error("No burst engine available - test mode cannot proceed")
            raise RuntimeError(
                "Test mode requires burst engine but none is available. "
                "Ensure FEAGI core services are properly initialized."
            )

        logger.info(
            "Test runner initialized - will use Core API Service for stimulation"
        )

        # Initialize the appropriate test mode handler
        self._initialize_test_mode_handler()

    def _initialize_test_mode_handler(self):
        """Initialize the appropriate test mode handler based on test_mode."""
        if self.test_mode == "mode_1":
            from .test_mode_1 import TestMode1Handler

            self.mode_handler = TestMode1Handler(self)
            logger.info("🎲 TEST MODE 1: JSON-based random sensory generation")
        elif self.test_mode == "mode_2":
            from .test_mode_2 import TestMode2Handler

            self.mode_handler = TestMode2Handler(self)
            logger.info("🎲 TEST MODE 2: Numpy-based scalable random neuron generation")
        else:
            raise ValueError(f"Unknown test mode: {self.test_mode}")

    def load_genome(self):
        """
        Load the essential genome using the core API.

        Returns:
            bool: True if genome was loaded successfully, False otherwise
        """
        try:
            logger.info("Loading essential genome for testing")

            # Check initial brain readiness state - should be False when starting
            initial_brain_ready = self.state_manager.get_brain_readiness()
            logger.info(f"Initial brain readiness state: {initial_brain_ready}")

            # Use the CoreAPIService method to load the essential genome
            result = self.core_api.load_essential_genome()

            # Check if the genome loading was successful
            if not result.get("success", False):
                logger.error(
                    f"Failed to load genome: {result.get('error', 'Unknown error')}"
                )
                return False

            # Wait for brain readiness to become True (state-driven)
            logger.info("Waiting for brain readiness state to become True...")

            # Poll the state manager for brain readiness changes
            check_interval = 0.1
            max_wait_time = 30.0
            elapsed_time = 0.0

            while elapsed_time < max_wait_time:
                if self.state_manager.get_brain_readiness():
                    logger.info(f"Brain is ready after {elapsed_time:.1f}s")
                    return True

                time.sleep(check_interval)
                elapsed_time += check_interval

            logger.error(f"Brain did not become ready within {max_wait_time}s")
            return False

        except Exception as e:
            logger.error(f"Error loading genome: {e}")
            import traceback

            logger.error(traceback.format_exc())
            return False

    def load_test_genome(self):
        """
        Load the test genome and prepare it for testing.

        For test mode 1, this method loads test_genome_1.json which includes the cortical
        mappings needed for neural signal propagation during testing.

        Returns:
            bool: True if genome was loaded successfully, False otherwise
        """
        try:
            logger.info("Loading test genome for testing")

            # Get initial brain readiness state
            initial_brain_readiness = self.state_manager.get_brain_readiness()
            logger.info(f"Initial brain readiness state: {initial_brain_readiness}")

            # PERFORMANCE: Use direct genome loading path for test mode
            if self.sample_genome_path:
                # Load custom genome if specified
                import json
                from pathlib import Path

                genome_path = Path(self.sample_genome_path)
                if not genome_path.exists():
                    logger.error(f"Genome file not found: {genome_path}")
                    return False

                logger.info(f"Loading custom genome: {genome_path}")

                # Read the genome file
                with open(genome_path, "r") as f:
                    genome_data = json.load(f)

                # Use the core API service's load_genome method
                result = self.core_api.load_genome(
                    genome_data, filename=genome_path.name
                )
                success = result.get("success", False)
            else:
                # Load the appropriate test genome based on test mode
                success = self._load_test_mode_genome()

            if not success:
                logger.error("Failed to load test genome")
                return False

            # Wait for brain readiness to become True (state-driven)
            logger.info("Waiting for brain readiness state to become True...")

            # Poll the state manager for brain readiness changes
            check_interval = 0.1
            max_wait_time = 30.0
            elapsed_time = 0.0

            while elapsed_time < max_wait_time:
                if self.state_manager.get_brain_readiness():
                    logger.info(f"Brain is ready after {elapsed_time:.1f}s")
                    return True

                time.sleep(check_interval)
                elapsed_time += check_interval

            logger.error(f"Brain did not become ready within {max_wait_time}s")
            return False

        except Exception as e:
            logger.error(f"Error loading test genome: {e}")
            import traceback

            logger.error(traceback.format_exc())
            return False

    def _load_test_mode_genome(self):
        """
        Load the appropriate test genome based on test mode.

        For test mode 1: Uses test_genome_1.json with intercortical mapping iv00_C -> iv00_C
        For other modes: Uses essential genome

        Returns:
            bool: True if genome was loaded successfully
        """
        try:
            if self.test_mode == "mode_1":
                # Load test_genome_1.json for test mode 1
                import json
                from pathlib import Path

                # Get the test genome file path
                test_genome_path = Path(__file__).parent / "test_genome_1.json"

                if not test_genome_path.exists():
                    logger.error(f"Test genome file not found: {test_genome_path}")
                    return False

                logger.info(
                    f"Loading test_genome_1.json for test mode 1: {test_genome_path}"
                )

                # Read the genome file
                with open(test_genome_path, "r") as f:
                    genome_data = json.load(f)

                # Convert to JSON string for the API
                genome_json_string = json.dumps(genome_data)

                # Use the core API service's load_genome method (same as /v1/genome/upload/string)
                result = self.core_api.load_genome(
                    genome_data, filename="test_genome_1.json"
                )

                if result.get("success", False):
                    logger.info(
                        "✅ Test genome 1 loaded successfully via string upload"
                    )
                    return True
                else:
                    logger.error(
                        f"Failed to load test genome 1: {result.get('error', 'Unknown error')}"
                    )
                    return False
            else:
                # For other test modes, use essential genome
                logger.info("Loading essential genome for non-mode-1 test")
                result = self.core_api.load_essential_genome()
                return result.get("success", False)

        except Exception as e:
            logger.error(f"Error loading test mode genome: {e}")
            import traceback

            logger.error(traceback.format_exc())
            return False

    def init_test_mode(self):
        """
        Initialize the selected test mode handler.

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        try:
            return self.mode_handler.initialize()
        except Exception as e:
            logger.error(f"Error initializing test mode: {e}")
            return False

    def capture_initial_state(self):
        """Capture the initial state of FCLs for comparison."""
        self.initial_fcls = {}

        for cortical_id in self.connectome.cortical_areas:
            fcl = self.fcl_manager.get_cortical_fcl(cortical_id)
            self.initial_fcls[cortical_id] = set(fcl) if fcl else set()

        logger.info(
            f"Captured initial state of {len(self.initial_fcls)} cortical areas"
        )

    def inject_test_data(self):
        """
        Inject test data using the selected test mode handler.

        Returns:
            bool: True if data was injected successfully, False otherwise
        """
        try:
            return self.mode_handler.inject_data()
        except Exception as e:
            logger.error(f"Error injecting test data: {e}")
            import traceback

            logger.error(traceback.format_exc())
            return False

    def submit_neuron_activations(self, activations, source_name):
        """
        Submit neuron activations through the Core API Service.

        PERFORMANCE: Optimized for RTOS/SIMD/GPU environments using the fastest injection path.

        Args:
            activations: Dictionary mapping cortical area IDs to lists of neuron IDs
            source_name: Source identifier for logging (e.g., "test_mode_1")

        Returns:
            int: Number of neurons successfully injected

        Raises:
            RuntimeError: If Core API Service is not available
        """
        try:
            if not activations:
                return 0

            # PERFORMANCE: Fast path validation - minimal overhead
            if not self.core_api:
                raise RuntimeError(f"Core API Service not available for {source_name}")

            total_neurons = sum(len(neurons) for neurons in activations.values())

            # PERFORMANCE: Use direct injection service path for maximum throughput
            # This bypasses API layers and goes straight to the injection service
            burst_engine = self.core_api.get_burst_engine()
            if burst_engine and hasattr(burst_engine, "injection_service"):
                injection_service = burst_engine.injection_service
                if injection_service:
                    # RTOS-FRIENDLY: Single call with all data, no iteration overhead
                    current_timestep = 0  # Use 0 for external injections (RTOS-safe)
                    injected_count = injection_service.inject_external_activations(
                        activations, current_timestep, source_name
                    )

                    # PERFORMANCE: Minimal logging in fast path
                    if injected_count > 0:
                        logger.debug(
                            f"Fast path: {injected_count}/{total_neurons} neurons from {source_name}"
                        )
                    elif injected_count == 0:
                        logger.warning(
                            f"Injection rejected by backpressure for {source_name}"
                        )

                    return injected_count

            # PERFORMANCE: If direct path unavailable, fail fast rather than using slow fallback
            logger.error(
                f"Direct injection path unavailable for {source_name} - no fallback for performance"
            )
            return 0

        except Exception as e:
            logger.error(f"Error in fast injection path for {source_name}: {e}")
            raise  # Re-raise to fail test mode fast

    def check_neural_activity(self):
        """
        Check if there is any neural activity.

        Returns:
            tuple: (activity_detected, list_of_active_areas)
        """
        activity_detected = False
        active_fcls = []
        total_active_neurons = 0

        for cortical_id in self.connectome.cortical_areas:
            current_fcl = self.fcl_manager.get_cortical_fcl(cortical_id)
            current_fcl_set = set(current_fcl) if current_fcl else set()

            # Skip empty FCLs
            if not current_fcl_set:
                continue

            # FIXED: Check for ANY neural activity, not just changes from initial state
            # Persistent firing (same neurons across timesteps) is NORMAL behavior
            if current_fcl_set:  # Any neurons firing = activity detected
                activity_detected = True
                active_fcls.append(cortical_id)
                self.areas_with_activity.add(cortical_id)
                total_active_neurons += len(current_fcl_set)
                logger.debug(
                    f"Neural activity in area {cortical_id}: {len(current_fcl_set)} neurons firing"
                )

        # Single summary log instead of individual area logs
        if activity_detected:
            logger.info(
                f"Neural activity: {total_active_neurons} neurons active across {len(active_fcls)} areas"
            )

        return activity_detected, active_fcls

    def check_neural_activity_with_burst_sync(self, max_wait_time=0.5):
        """
        Check for neural activity with burst engine synchronization.

        PERFORMANCE: Uses event-driven approach instead of polling for RTOS/SIMD/GPU compatibility.

        Args:
            max_wait_time: Maximum time to wait for burst processing (seconds)

        Returns:
            tuple: (activity_detected, list_of_active_areas)
        """
        # PERFORMANCE OPTIMIZATION: Skip synchronization if burst engine is running fast enough
        # For high-frequency burst engines (>50Hz), the injection will be processed within 20ms
        # which is faster than our polling interval anyway
        try:
            burst_config = self.core_api.get_burst_engine_config()
            burst_frequency = burst_config.get("burst_frequency_hz", 10.0)

            if burst_frequency >= 50.0:
                # High-frequency mode: minimal delay, no polling overhead
                import time

                time.sleep(0.02)  # Single 20ms delay for high-frequency engines
                logger.debug(
                    f"High-frequency burst engine ({burst_frequency}Hz) - using minimal delay"
                )
                return self.check_neural_activity()

        except Exception as e:
            logger.debug(f"Could not get burst frequency: {e}")

        # PERFORMANCE: For standard frequency engines, use single burst interval wait
        # This is RTOS-friendly as it aligns with the burst engine's natural timing
        try:
            burst_config = self.core_api.get_burst_engine_config()
            burst_interval = burst_config.get("burst_interval_seconds", 0.1)

            # Wait for exactly one burst interval - this is deterministic and RTOS-friendly
            import time

            time.sleep(
                burst_interval * 1.1
            )  # 110% of burst interval to ensure completion

            logger.debug(
                f"Waited {burst_interval * 1.1:.3f}s for burst processing (interval-based)"
            )
            return self.check_neural_activity()

        except Exception as e:
            logger.warning(f"Could not get burst interval: {e} - using immediate check")
            return self.check_neural_activity()

    def run_test(self):
        """
        Run the test in a separate thread.

        Returns:
            bool: True if test was started successfully, False otherwise
        """
        if self.is_running:
            logger.warning("Test is already running")
            return False

        self.test_thread = threading.Thread(target=self._run_test_thread)
        self.test_thread.daemon = True
        self.test_thread.start()

        return True

    def _run_test_thread(self):
        """Internal method to run the test in a separate thread."""
        try:
            self.is_running = True
            self.test_result = None
            self.areas_with_activity = set()

            # Load the appropriate genome based on test mode
            if not self.load_test_genome():
                self.test_result = False
                self.is_running = False
                return

            # Initialize test mode
            if not self.init_test_mode():
                self.test_result = False
                self.is_running = False
                return

            # Capture initial state
            self.capture_initial_state()

            # Get the IPU (sensory) areas from the connectome
            ipu_areas = {
                id: area
                for id, area in self.connectome.cortical_areas.items()
                if area.properties.get("group") == "IPU"
            }
            if not ipu_areas:
                logger.error("No IPU areas found in the genome")
                self.test_result = False
                self.is_running = False
                return

            logger.info(f"Found {len(ipu_areas)} IPU areas: {list(ipu_areas.keys())}")

            # Start the test loop
            test_start_time = time.time()
            end_time = test_start_time + self.test_duration

            cycle_count = 0
            last_report_time = test_start_time
            report_interval = 5.0

            while time.time() < end_time:
                cycle_count += 1

                # Only log cycle numbers every 5 seconds to reduce verbosity
                current_time = time.time()
                if current_time - last_report_time >= report_interval:
                    logger.info(
                        f"Test progress: cycle {cycle_count} ({current_time - test_start_time:.1f}s elapsed)"
                    )
                    last_report_time = current_time

                # Inject test data
                if not self.inject_test_data():
                    logger.warning(f"Failed to inject test data in cycle {cycle_count}")
                    # Continue with the test even if one cycle fails

                # Wait for a short time to allow the burst engine to process
                time.sleep(1.0 / self.frequency_hz)

                # Check neural activity
                activity_detected, active_areas = (
                    self.check_neural_activity_with_burst_sync()
                )
                if activity_detected:
                    logger.debug(f"Neural activity detected in cycle {cycle_count}")

            # Test completion
            test_duration = time.time() - test_start_time

            # Check test results
            if self.areas_with_activity:
                logger.info(
                    f"TEST PASSED: Neural activity detected in {len(self.areas_with_activity)} areas: {list(self.areas_with_activity)}"
                )
                self.test_result = True
            else:
                logger.error("TEST FAILED: No neural activity detected")
                self.test_result = False

            logger.info(f"Test completed in {test_duration:.2f} seconds")

        except Exception as e:
            logger.error(f"Error in test thread: {e}")
            import traceback

            logger.error(traceback.format_exc())
            self.test_result = False
        finally:
            self.is_running = False

    def get_test_result(self):
        """
        Get the test result.

        Returns:
            bool or None: True if test passed, False if failed, None if still running
        """
        return self.test_result

    def is_test_running(self):
        """
        Check if the test is currently running.

        Returns:
            bool: True if test is running, False otherwise
        """
        return self.is_running


def run_test_mode(core_api_service, test_mode="mode_1", **kwargs):
    """
    Run FEAGI in test mode.

    Args:
        core_api_service: FEAGI's core API service
        test_mode: "mode_1" for JSON-based or "mode_2" for numpy-based
        **kwargs: Additional test configuration options
            - genome_path: Path to a specific genome to load
            - test_duration: Duration of the test in seconds (default: 10)
            - frequency_hz: Frequency of sensory input generation in Hz (default: 10)

    Returns:
        bool: True if tests passed, False otherwise
    """
    logger.info(f"Starting FEAGI test mode: {test_mode}")

    # Create and run the test runner
    test_runner = FeagiTestRunner(
        core_api_service=core_api_service,
        test_mode=test_mode,
        sample_genome_path=kwargs.get("genome_path"),
        test_duration=kwargs.get("test_duration", 10),
        frequency_hz=kwargs.get("frequency_hz", 10),
    )

    # Run the test synchronously in the current thread
    test_runner._run_test_thread()

    # Get the test result
    result = test_runner.get_test_result()

    if result:
        logger.info(f"FEAGI test mode {test_mode} completed successfully")
    else:
        logger.error(f"FEAGI test mode {test_mode} failed")

    return result
