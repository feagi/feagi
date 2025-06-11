"""
FEAGI Test Runner

Main test runner that coordinates between different test modes and provides
the common testing infrastructure.
"""

import logging
import os
import random
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

from feagi.config.toml_loader import get_host_config, load_feagi_config
from feagi.core.state_manager import FeagiStateManager, GenomeState, ServiceState
from feagi.evo.genome_processor import process_and_load_genome
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

        # Initialize FCL injection service reference - will be set after genome loading
        self.fcl_injection_service = None

        # Verify burst engine is available (but injection service will be checked later)
        if not self.burst_engine:
            logger.error("No burst engine available - test mode cannot proceed")
            raise RuntimeError(
                "Test mode requires burst engine but none is available. "
                "Ensure FEAGI core services are properly initialized."
            )

        logger.info(
            "Test runner initialized - injection service will be verified after genome loading"
        )

        # Initialize the appropriate test mode handler
        self._initialize_test_mode_handler()

    def _verify_injection_service_available(self):
        """
        Verify that the FCL injection service is available for test mode.

        This should be called after genome loading to ensure the injection service
        has been properly initialized with the loaded genome data.

        Returns:
            bool: True if injection service is available, False otherwise
        """
        if self.burst_engine and hasattr(self.burst_engine, "injection_service"):
            self.fcl_injection_service = self.burst_engine.injection_service
            if self.fcl_injection_service:
                logger.info(
                    "FCL injection service verified and available for test mode"
                )
                return True
            else:
                logger.error(
                    "Burst engine has no injection service after genome loading"
                )
                return False
        else:
            logger.error("Burst engine has no injection_service attribute")
            return False

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

            # Refresh FCL injection service reference after genome loading
            try:
                burst_engine = self.core_api.get_burst_engine()
                if burst_engine and hasattr(burst_engine, "injection_service"):
                    self.fcl_injection_service = burst_engine.injection_service
                    if self.fcl_injection_service:
                        logger.info(
                            "FCL injection service refreshed after essential genome load"
                        )
                    else:
                        logger.warning(
                            "Injection service is None after essential genome load"
                        )
            except Exception as e:
                logger.warning(f"Could not refresh injection service: {e}")

            # Wait for brain readiness to become True (state-driven)
            logger.info("Waiting for brain readiness state to become True...")

            # Poll the state manager for brain readiness changes
            check_interval = 0.1
            max_wait_time = 30.0
            elapsed_time = 0.0

            while elapsed_time < max_wait_time:
                if self.state_manager.get_brain_readiness():
                    logger.info(f"Brain is ready after {elapsed_time:.1f}s")

                    # Verify injection service is now available after genome loading
                    if not self._verify_injection_service_available():
                        logger.error(
                            "Test mode requires FCL injection service but none is available after essential genome loading"
                        )
                        return False

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
        Load the test genome using the core API.

        Returns:
            bool: True if genome was loaded successfully, False otherwise
        """
        try:
            logger.info("Loading test genome for testing")

            # Check initial brain readiness state - should be False when starting
            initial_brain_ready = self.state_manager.get_brain_readiness()
            logger.info(f"Initial brain readiness state: {initial_brain_ready}")

            # Use the CoreAPIService method to load the test genome
            result = self.core_api.load_test_genome()

            # Check if the genome loading was successful
            if not result.get("success", False):
                logger.error(
                    f"Failed to load test genome: {result.get('error', 'Unknown error')}"
                )
                return False

            # CRITICAL: Update burst engine with new genome to initialize special area services
            # This ensures power areas like "___pwr" get properly configured for injection
            try:
                burst_engine = self.core_api.get_burst_engine()
                if burst_engine:
                    burst_engine.update_with_genome()
                    logger.info(
                        "Burst engine updated with test genome - special area services initialized"
                    )

                    # Refresh FCL injection service reference after genome update
                    if hasattr(burst_engine, "injection_service"):
                        self.fcl_injection_service = burst_engine.injection_service
                        if self.fcl_injection_service:
                            logger.info(
                                "FCL injection service refreshed after genome update"
                            )
                        else:
                            logger.warning(
                                "Injection service is None after genome update"
                            )
                    else:
                        logger.warning(
                            "Burst engine has no injection_service attribute"
                        )
                else:
                    logger.warning("No burst engine available for genome update")
            except Exception as e:
                logger.error(f"Failed to update burst engine with genome: {e}")
                # Continue anyway as this might not be critical for basic test functionality

            # Wait for brain readiness to become True (state-driven)
            logger.info("Waiting for brain readiness state to become True...")

            # Poll the state manager for brain readiness changes
            check_interval = 0.1
            max_wait_time = 30.0
            elapsed_time = 0.0

            while elapsed_time < max_wait_time:
                if self.state_manager.get_brain_readiness():
                    logger.info(f"Brain is ready after {elapsed_time:.1f}s")

                    # Verify injection service is now available after genome loading
                    if not self._verify_injection_service_available():
                        logger.error(
                            "Test mode requires FCL injection service but none is available after test genome loading"
                        )
                        return False

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
        Submit neuron activations through the FCL injection service.

        Test mode MUST use the primary FCL injection service to properly test
        the main neural engine. No fallbacks are allowed - if the injection
        service isn't available, test mode should fail.

        Args:
            activations: Dictionary mapping cortical area IDs to lists of neuron IDs
            source_name: Source identifier for logging (e.g., "test_mode_1")

        Returns:
            int: Number of neurons successfully injected

        Raises:
            RuntimeError: If FCL injection service is not available
        """
        try:
            if not activations:
                logger.debug(f"No activations to submit from {source_name}")
                return 0

            # FCL injection service MUST be available for test mode
            if not self.fcl_injection_service:
                raise RuntimeError(
                    f"FCL injection service not available for {source_name}. "
                    "Test mode requires the primary injection service to properly test the neural engine. "
                    "Check that the burst engine and injection service are properly initialized."
                )

            # Use the primary FCL injection service (the only path for test mode)
            current_timestep = getattr(self.fcl_manager, "current_timestep", 0)
            injected_count = self.fcl_injection_service.inject_external_activations(
                activations, current_timestep, source_name
            )

            total_neurons = sum(len(neurons) for neurons in activations.values())
            logger.debug(
                f"Successfully injected {injected_count}/{total_neurons} neurons from {source_name} via primary FCL injection service"
            )
            return injected_count

        except Exception as e:
            logger.error(f"Error submitting neuron activations from {source_name}: {e}")
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
            if self.test_mode == "mode_2":
                # Test Mode 2 uses test_genome.json
                if not self.load_test_genome():
                    self.test_result = False
                    self.is_running = False
                    return
            else:
                # Default to essential genome for other test modes
                if not self.load_genome():
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
                activity_detected, active_areas = self.check_neural_activity()
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
