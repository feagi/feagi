"""
FEAGI Test Mode Module.

This module provides test functionality for FEAGI, allowing automated testing of
sensory input processing using the activity_generator from feagi_sim.

It is designed to be called from the main FEAGI process when the --test flag is provided.
"""
import os
import json
import logging
import time
import threading
from typing import Dict, Any, Optional, List

from feagi.core.state_manager import FeagiStateManager, ServiceState, GenomeState
from feagi.utils.logger import setup_logger
from feagi.evo.genome_processor import process_and_load_genome

# Use feagi_sim for activity generation
from feagi_sim.activity_generator import (
    CorticalType, 
    OutputFormat,
    timed_cortical_activity_generator
)

logger = setup_logger("feagi.test_mode")

class FeagiTestRunner:
    """
    Test runner for FEAGI sensory input processing.
    
    This class provides functionality for:
    1. Loading a test genome
    2. Generating and injecting synthetic sensory data
    3. Monitoring neural activity
    4. Reporting test results
    """
    
    def __init__(self, core_api_service, sample_genome_path=None, test_duration=10, frequency_hz=10):
        """
        Initialize the test runner.
        
        Args:
            core_api_service: FEAGI's core API service
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
        self.test_duration = test_duration
        self.frequency_hz = frequency_hz
        
        # If no sample genome path is provided, use the essential genome
        self.sample_genome_path = sample_genome_path
        
        # Test state variables
        self.is_running = False
        self.test_thread = None
        self.test_result = None
        self.sensory_data_generator = None
        self.initial_fcls = {}
        self.areas_with_activity = set()
    
    def load_genome(self):
        """
        Load the essential genome using the core API.
        
        Returns:
            bool: True if genome was loaded successfully, False otherwise
        """
        try:
            logger.info("Loading essential genome for testing")
            
            # Use the CoreAPIService method to load the essential genome
            result = self.core_api.load_essential_genome()
            
            # Verify that the genome was loaded by checking if there are cortical areas
            if len(self.connectome.cortical_areas) > 0:
                logger.info("Essential genome loaded successfully")
                logger.info(f"Loaded genome has {len(self.connectome.cortical_areas)} cortical areas")
                return True
            else:
                logger.error("Genome was processed but no cortical areas found")
                return False
                
        except Exception as e:
            logger.error(f"Error loading essential genome: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False
    
    def init_sensory_data_generator(self):
        """
        Initialize the sensory data generator.
        
        Returns:
            bool: True if initialization was successful, False otherwise
        """
        try:
            # Get the genome data from the connectome
            genome_data = self.core_api.get_genome()
            
            if not genome_data:
                logger.error("No genome data available for sensory data generation")
                return False
                
            # Create a generator for sensory data using feagi_sim
            logger.info(f"Creating sensory data generator at {self.frequency_hz}Hz with LIST format")
            self.sensory_data_generator = timed_cortical_activity_generator(
                genome_data=genome_data,
                frequency_hz=self.frequency_hz,
                output_format=OutputFormat.LIST_FORMAT,
                cortical_type=CorticalType.SENSORY,
                sparsity=0.05
            )
            
            return True
            
        except Exception as e:
            logger.error(f"Error initializing sensory data generator: {e}")
            return False
    
    def capture_initial_state(self):
        """Capture the initial state of FCLs for comparison."""
        self.initial_fcls = {}
        
        for cortical_id in self.connectome.cortical_areas:
            fcl = self.fcl_manager.get_cortical_fcl(cortical_id)
            self.initial_fcls[cortical_id] = set(fcl) if fcl else set()
            
        logger.info(f"Captured initial state of {len(self.initial_fcls)} cortical areas")
    
    def inject_sensory_data(self):
        """
        Inject one batch of sensory data into FCLs.
        
        Returns:
            bool: True if data was injected successfully, False otherwise
        """
        try:
            # Get the next batch of sensory data
            sensory_data = next(self.sensory_data_generator)
            
            if not sensory_data:
                logger.warning("No sensory data generated")
                return False
                
            logger.info(f"Generated sensory data for {len(sensory_data)} cortical areas")
            
            # Process each sensory area's data
            for cortical_id, data in sensory_data.items():
                # Find the cortical area in the connectome
                cortical_area = self.connectome.cortical_areas.get(cortical_id)
                if not cortical_area:
                    logger.warning(f"Cortical area {cortical_id} not found in connectome")
                    continue
                
                # Extract the data components (in list format)
                x_coords, y_coords, z_coords, potentials = data
                
                # Process active neurons and collect their IDs
                active_neuron_ids = set()
                
                # Process each neuron
                for i in range(len(x_coords)):
                    x, y, z = x_coords[i], y_coords[i], z_coords[i]
                    potential = potentials[i]
                    
                    # Skip neurons with low potential
                    if potential <= 0.01:
                        continue
                        
                    # Find neurons at this position
                    position = (x, y, z)
                    neurons = cortical_area.get_neurons_at_position(position)
                    
                    # Add all neurons at this position to our collection
                    active_neuron_ids.update(neurons)
                
                # Create bitmap from collected neuron IDs (if available in this version)
                from feagi.npu.fcl_manager import BitMap
                bitmap = BitMap(active_neuron_ids)
                
                # Add the bitmap to FCL updates
                if len(bitmap) > 0:
                    logger.info(f"Adding {len(bitmap)} neurons to FCL for area {cortical_id}")
                    self.fcl_manager.update_fcl(self.fcl_manager.current_timestep, {cortical_id: bitmap})
            
            return True
            
        except Exception as e:
            logger.error(f"Error injecting sensory data: {e}")
            return False
    
    def check_neural_activity(self):
        """
        Check if there is any neural activity.
        
        Returns:
            tuple: (activity_detected, list_of_active_areas)
        """
        changed_fcls = False
        active_fcls = []
        
        for cortical_id in self.connectome.cortical_areas:
            current_fcl = self.fcl_manager.get_cortical_fcl(cortical_id)
            current_fcl_set = set(current_fcl) if current_fcl else set()
            
            # Skip empty FCLs
            if not current_fcl_set:
                continue
                
            # Check if the FCL has changed
            if current_fcl_set != self.initial_fcls.get(cortical_id, set()):
                changed_fcls = True
                active_fcls.append(cortical_id)
                self.areas_with_activity.add(cortical_id)
                logger.info(f"FCL for area {cortical_id} changed: {len(current_fcl_set)} neurons active")
        
        return changed_fcls, active_fcls
    
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
            
            # Load the genome
            if not self.load_genome():
                self.test_result = False
                self.is_running = False
                return
                
            # Initialize sensory data generator
            if not self.init_sensory_data_generator():
                self.test_result = False
                self.is_running = False
                return
                
            # Capture initial state
            self.capture_initial_state()
            
            # Get the IPU (sensory) areas from the connectome
            ipu_areas = {id: area for id, area in self.connectome.cortical_areas.items() 
                        if area.properties.get('group') == 'IPU'}
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
            while time.time() < end_time:
                cycle_count += 1
                logger.info(f"Test cycle {cycle_count}")
                
                # Inject sensory data
                if not self.inject_sensory_data():
                    logger.warning(f"Failed to inject sensory data in cycle {cycle_count}")
                    # Continue with the test even if one cycle fails
                
                # Wait for a short time to allow the burst engine to process
                time.sleep(1.0 / self.frequency_hz)
                
                # Check neural activity
                activity_detected, active_areas = self.check_neural_activity()
                if activity_detected:
                    logger.info(f"Neural activity detected in cycle {cycle_count}")
            
            # Test completion
            test_duration = time.time() - test_start_time
            
            if self.areas_with_activity:
                logger.info(f"TEST PASSED: Neural activity detected in {len(self.areas_with_activity)} areas: {list(self.areas_with_activity)}")
                self.test_result = True
            else:
                logger.error("TEST FAILED: No neural activity detected")
                self.test_result = False
                
            logger.info(f"Test completed in {test_duration:.2f} seconds ({cycle_count} cycles)")
            
        except Exception as e:
            logger.error(f"Error during test execution: {e}")
            import traceback
            logger.error(traceback.format_exc())
            self.test_result = False
            
        finally:
            self.is_running = False
    
    def wait_for_completion(self, timeout=None):
        """
        Wait for the test to complete.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            bool: Test result (True for pass, False for fail, None if not completed)
        """
        if self.test_thread and self.test_thread.is_alive():
            self.test_thread.join(timeout)
            
        return self.test_result
    
    def get_test_result(self):
        """
        Get the test result.
        
        Returns:
            bool: Test result (True for pass, False for fail, None if not completed)
        """
        return self.test_result


def run_test_mode(core_api_service, **kwargs):
    """
    Run FEAGI in test mode.
    
    Args:
        core_api_service: FEAGI's core API service
        **kwargs: Additional test configuration options
        
    Returns:
        bool: True if tests passed, False otherwise
    """
    logger.info("Starting FEAGI test mode")
    
    # Create and run the test runner
    test_runner = FeagiTestRunner(
        core_api_service=core_api_service,
        sample_genome_path=kwargs.get('genome_path'),
        test_duration=kwargs.get('test_duration', 10),
        frequency_hz=kwargs.get('frequency_hz', 10)
    )
    
    # Run the test synchronously in the current thread
    test_runner._run_test_thread()
    
    # Get the test result
    result = test_runner.get_test_result()
    
    if result:
        logger.info("FEAGI test mode completed successfully")
    else:
        logger.error("FEAGI test mode failed")
        
    return result 