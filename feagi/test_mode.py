"""
FEAGI Test Mode Module.

This module provides test functionality for FEAGI, allowing automated testing of
sensory input processing using the activity_generator from feagi_sim.

Key Testing Features:
1. Loading a test genome (essential genome by default)
2. Generating and injecting synthetic sensory data into FCL
3. Monitoring neural activity across cortical areas
4. Reporting test results

Visualization Testing Mode:
When the --test-visualization flag is enabled, test_mode will:
1. Register a fake visualization agent with FEAGI
2. Set the visualization_subscribers flag on the FCL sampler to TRUE
3. This causes the FCL sampler to collect visualization data as if real clients were connected
4. No custom data processing is performed in test_mode - the existing FCL sampler handles this

Important Notes:
- Test_mode is designed to be minimal and non-intrusive to the main FEAGI code
- It enables testing of the visualization data flow without requiring actual ZMQ connections
- All visualization data processing happens in the main FEAGI codebase, not in test_mode
- The actual visualization logging occurs in the main FEAGI logging system

This module is designed to be called from the main FEAGI process when the 
--test or --test-visualization flags are provided.
"""
import os
import json
import logging
import time
import threading
from typing import Dict, Any, Optional, List, Set

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
    5. Testing visualization data flow (when test_visualization=True)
    """
    
    def __init__(self, core_api_service, sample_genome_path=None, test_duration=10, frequency_hz=10, test_visualization=False):
        """
        Initialize the test runner.
        
        Args:
            core_api_service: FEAGI's core API service
            sample_genome_path: Path to the sample genome to load
            test_duration: Duration of the test in seconds
            frequency_hz: Frequency of sensory input generation in Hz
            test_visualization: Whether to test visualization data flow
        """
        self.core_api = core_api_service
        self.connectome = self.core_api.get_connectome_manager()
        self.burst_engine = self.core_api.get_burst_engine()
        self.fcl_manager = self.core_api.get_fcl_manager()
        self.state_manager = FeagiStateManager.instance()
        
        # Test configuration
        self.test_duration = test_duration
        self.frequency_hz = frequency_hz
        self.test_visualization = test_visualization
        
        # If no sample genome path is provided, use the essential genome
        self.sample_genome_path = sample_genome_path
        
        # Test state variables
        self.is_running = False
        self.test_thread = None
        self.test_result = None
        self.sensory_data_generator = None
        self.initial_fcls = {}
        self.areas_with_activity = set()
        
        # Visualization test variables
        self.visualization_agent_id = "test_viz_agent"
        self.is_visualization_agent_registered = False
        self.heartbeat_thread = None
        self.send_heartbeats = False
    
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
    
    def register_visualization_agent(self):
        """
        Register a fake visualization agent for testing.
        
        Returns:
            bool: True if agent was registered successfully, False otherwise
        """
        if self.is_visualization_agent_registered:
            return True
            
        try:
            # Check if the method exists (as it might not in all FEAGI versions)
            if not hasattr(self.core_api, 'register_agent'):
                logger.warning("register_agent method not available, skipping visualization agent registration")
                return False
                
            # Register a fake visualization agent
            result = self.core_api.register_agent(
                agent_id=self.visualization_agent_id,
                agent_type="visualization",
                agent_ip="127.0.0.1",
                agent_data_port=5555,
                agent_version="1.0.0",
                controller_version="1.0.0",
                capabilities={"visualization": True}
            )
            
            if result:
                self.is_visualization_agent_registered = True
                logger.info(f"Registered test visualization agent: {self.visualization_agent_id}")
                
                # Start sending heartbeats
                self.start_heartbeat_thread()
                return True
            else:
                logger.error("Failed to register visualization agent")
                return False
                
        except Exception as e:
            logger.error(f"Error registering visualization agent: {e}")
            return False
            
    def start_heartbeat_thread(self):
        """Start a thread to send periodic heartbeats for the test visualization agent."""
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            logger.debug("Heartbeat thread already running")
            return
            
        self.send_heartbeats = True
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_thread_func)
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()
        logger.info("Started visualization agent heartbeat thread")
        
    def stop_heartbeat_thread(self):
        """Stop the heartbeat thread."""
        self.send_heartbeats = False
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(1.0)  # Wait for up to 1 second
            logger.info("Stopped visualization agent heartbeat thread")
            
    def _heartbeat_thread_func(self):
        """Thread function for sending periodic heartbeats."""
        logger.info(f"Heartbeat thread started for test visualization agent: {self.visualization_agent_id}")
        
        while self.send_heartbeats:
            try:
                # Sleep first to allow the heartbeat to be processed
                time.sleep(5.0)
                
                if not self.send_heartbeats:
                    break
                    
                # Check if the core API is available
                if hasattr(self.core_api, 'register_agent_heartbeat'):
                    self.core_api.register_agent_heartbeat(self.visualization_agent_id)
                    logger.debug(f"Sent heartbeat for test visualization agent: {self.visualization_agent_id}")
                    
                    # Also send a test activity message every 3rd heartbeat (15 seconds)
                    if hasattr(self, '_heartbeat_counter'):
                        self._heartbeat_counter += 1
                    else:
                        self._heartbeat_counter = 1
                        
                    if self._heartbeat_counter % 3 == 0:
                        self.send_test_activity_data()
                        
            except Exception as e:
                logger.error(f"Error sending test visualization agent heartbeat: {e}")
                
        logger.info("Heartbeat thread stopped")
        
    def send_test_activity_data(self):
        """Send test activity data directly to the visualization stream."""
        try:
            logger.info("Sending TEST ACTIVITY DATA to visualization stream")
            print("\n==================================================")
            print("SENDING TEST ACTIVITY DATA TO VISUALIZATION STREAM")
            print("==================================================\n")
            
            # Find the visualization stream if available
            viz_stream = None
            
            # Try different paths to find it
            if hasattr(self.core_api, 'visualization_stream'):
                viz_stream = self.core_api.visualization_stream
                logger.info("Found visualization stream in core_api.visualization_stream")
            
            if not viz_stream:
                # Check if we can access it through process manager
                try:
                    from feagi.process_manager import get_process_manager
                    pm = get_process_manager()
                    if pm and hasattr(pm, '_visualization_stream'):
                        viz_stream = pm._visualization_stream
                        logger.info("Found visualization stream in process_manager._visualization_stream")
                except Exception as e:
                    logger.error(f"Error finding visualization stream through process manager: {e}")
            
            if not viz_stream:
                logger.error("Could not find visualization stream, cannot send test activity data")
                return
                
            # Create a simple test activity data structure using feagi_bytes
            try:
                from feagi_bytes import ByteStructureEncoder
                
                # Create the encoder
                encoder = ByteStructureEncoder()
                
                # Create simple test data - a 3x3x3 grid of neurons with random activity
                cortical_ids = []
                x_coords = []
                y_coords = []
                z_coords = []
                potentials = []
                
                # Fill with test data
                import random
                for x in range(3):
                    for y in range(3):
                        for z in range(3):
                            cortical_ids.append("TEST_A")  # Test cortical area
                            x_coords.append(x)
                            y_coords.append(y)
                            z_coords.append(z)
                            # Random potential between 0 and 1
                            potentials.append(random.random())
                
                # Add a few more areas for testing
                for i in range(10):
                    cortical_ids.append("TEST_B")
                    x_coords.append(random.randint(0, 5))
                    y_coords.append(random.randint(0, 5))
                    z_coords.append(random.randint(0, 5))
                    potentials.append(random.random())
                
                # Encode the neuron data
                if hasattr(encoder, 'encode_neuron_flat'):
                    bytes_data = encoder.encode_neuron_flat(
                        cortical_ids=cortical_ids,
                        x_coords=x_coords,
                        y_coords=y_coords,
                        z_coords=z_coords,
                        potentials=potentials
                    )
                    
                    # Log the bytes data 
                    hex_dump = ' '.join([f'{b:02x}' for b in bytes_data[:50]])
                    logger.info(f"Encoded test activity data ({len(bytes_data)} bytes): {hex_dump}")
                    
                    # Schedule the data to be sent asynchronously
                    if hasattr(viz_stream, 'send_visualization_data'):
                        # Need to run in event loop for async functions
                        import asyncio
                        
                        # Create a task to send the data
                        async def send_data():
                            await viz_stream.send_visualization_data(bytes_data)
                            logger.info("Test activity data sent to visualization stream")
                            
                        # Run the task in the current event loop or create a new one
                        try:
                            loop = asyncio.get_event_loop()
                            if loop.is_running():
                                loop.create_task(send_data())
                            else:
                                asyncio.run(send_data())
                        except Exception as e:
                            logger.error(f"Error sending data in event loop: {e}")
                    else:
                        logger.error("Visualization stream does not have send_visualization_data method")
                else:
                    logger.error("ByteStructureEncoder does not have encode_neuron_flat method")
                
            except Exception as e:
                logger.error(f"Error creating test activity data: {e}")
                import traceback
                logger.error(traceback.format_exc())
                
        except Exception as e:
            logger.error(f"Error in send_test_activity_data: {e}")
            import traceback
            logger.error(traceback.format_exc())
    
    def hook_fq_sampler(self):
        """
        Enable visualization on the FQ sampler without modifying its behavior.
        
        This method simply tells the FQ sampler that there are visualization
        subscribers, causing it to sample data without our code intercepting it.
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Find the FQ sampler if it exists
            fq_sampler = None
            
            # Check if we can access the FQ sampler through different paths
            if hasattr(self.core_api, 'fq_sampler'):
                fq_sampler = self.core_api.fq_sampler
                logger.info("Found FQ sampler in core_api.fq_sampler")
            elif hasattr(self.connectome, 'fq_sampler'):
                fq_sampler = self.connectome.fq_sampler
                logger.info("Found FQ sampler in connectome.fq_sampler")
            
            # Try to get it from the process manager if available
            if not fq_sampler:
                from feagi.process_manager import get_process_manager
                process_manager = get_process_manager()
                if process_manager and hasattr(process_manager, '_fq_sampler'):
                    fq_sampler = process_manager._fq_sampler
                    logger.info("Found FQ sampler in process_manager._fq_sampler")
            
            if not fq_sampler:
                logger.warning("FQ sampler not found, cannot enable visualization data")
                return False
            
            # Log FQ sampler attributes
            logger.info(f"FQ sampler attributes: {dir(fq_sampler)}")
            
            # Enable visualization subscribers mode - this is the only change we make
            if hasattr(fq_sampler, 'set_visualization_subscribers'):
                fq_sampler.set_visualization_subscribers(True)
                logger.info("Enabled visualization subscribers on FQ sampler")
                
                # Also check if the FQ sampler has a queue and verify its state
                if hasattr(fq_sampler, 'visualization_queue'):
                    logger.info(f"FQ sampler has visualization_queue: {fq_sampler.visualization_queue}")
                    
                # Check if the FQ sampler has a callback and verify its state
                if hasattr(fq_sampler, 'viz_data_callback'):
                    logger.info(f"FQ sampler has viz_data_callback: {fq_sampler.viz_data_callback}")
                
                return True
            else:
                logger.warning("FQ sampler does not support visualization subscribers")
                return False
            
        except Exception as e:
            logger.error(f"Error enabling visualization on FQ sampler: {e}")
            import traceback
            logger.error(traceback.format_exc())
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
            
            # If testing visualization, register a fake visualization agent
            if self.test_visualization:
                logger.info("Setting up visualization testing")
                # Enable test visualization mode in the state manager
                self.state_manager.set_test_visualization_mode(True)
                self.register_visualization_agent()
                self.hook_fq_sampler()
            
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
            
            # Check test results
            if self.areas_with_activity:
                logger.info(f"TEST PASSED: Neural activity detected in {len(self.areas_with_activity)} areas: {list(self.areas_with_activity)}")
                self.test_result = True
            else:
                logger.error("TEST FAILED: No neural activity detected")
                self.test_result = False
            
            # Report visualization test results if applicable
            if self.test_visualization:
                logger.info("Visualization test completed")
                if self.is_visualization_agent_registered:
                    logger.info("Visualization test PASSED: Successfully registered visualization agent")
                else:
                    logger.warning("Visualization test WARNING: Failed to register visualization agent")
                
                # Stop heartbeat thread
                self.stop_heartbeat_thread()
                
            logger.info(f"Test completed in {test_duration:.2f} seconds ({cycle_count} cycles)")
            
        except Exception as e:
            logger.error(f"Error during test execution: {e}")
            import traceback
            logger.error(traceback.format_exc())
            self.test_result = False
            
        finally:
            if self.test_visualization:
                self.stop_heartbeat_thread()
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
            - genome_path: Path to a specific genome to load
            - test_duration: Duration of the test in seconds (default: 10)
            - frequency_hz: Frequency of sensory input generation in Hz (default: 10)
            - test_visualization: Whether to test visualization data flow (default: False)
        
    Returns:
        bool: True if tests passed, False otherwise
    """
    logger.info("Starting FEAGI test mode")
    
    # Create and run the test runner
    test_runner = FeagiTestRunner(
        core_api_service=core_api_service,
        sample_genome_path=kwargs.get('genome_path'),
        test_duration=kwargs.get('test_duration', 10),
        frequency_hz=kwargs.get('frequency_hz', 10),
        test_visualization=kwargs.get('test_visualization', False)
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