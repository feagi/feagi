"""
FEAGI Test Mode Module.

This module provides test functionality for FEAGI, allowing automated testing of
sensory input processing using the activity_generator from feagi_sim.

Key Testing Features:
1. Loading a test genome (essential genome by default)
2. Generating and injecting synthetic sensory data into FCL
3. Monitoring neural activity across cortical areas
4. Reporting test results
5. Predictable neuron activation from JSON file

Predictable Neuron Activation:
If a file named 'test_mode_activations.json' exists in the same folder as this module,
it will be loaded and used for predictable neuron injection instead of random selection.

JSON Format:
{
  "cortical_id": [[x,y,z], [x,y,z], [x,y,z]],
  "cortical_id": [[x,y,z], [x,y,z], [x,y,z]],
  ...
}

When this file is present and contains valid data:
- ONLY the specified neurons at the given coordinates will be activated
- NO random neuron selection will occur
- The test becomes fully deterministic and repeatable

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
# from feagi_sim.activity_generator import (\n    CorticalType, \n    OutputFormat,\n    timed_cortical_activity_generator\n)

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
        
        # Predictable neuron activation support
        self.test_activations_data = None
        self.use_predictable_activations = False
        
        # Visualization test variables
        self.visualization_agent_id = "test_viz_agent"
        self.is_visualization_agent_registered = False
        self.heartbeat_thread = None
        self.send_heartbeats = False
        
        # Try to load test activations JSON file
        self.load_test_activations_json()
    
    def load_test_activations_json(self):
        """
        Load predictable neuron activations from test_mode_activations.json.
        
        If the file exists in the same folder as this module, it will be loaded
        and used for predictable neuron injection instead of random selection.
        
        Returns:
            bool: True if JSON file was loaded successfully, False otherwise
        """
        try:
            # Get the directory where this module is located
            module_dir = os.path.dirname(os.path.abspath(__file__))
            json_path = os.path.join(module_dir, "test_mode_activations.json")
            
            if not os.path.exists(json_path):
                logger.info("No test_mode_activations.json found - using random neuron injection")
                self.use_predictable_activations = False
                return False
            
            logger.info(f"Loading predictable neuron activations from: {json_path}")
            
            with open(json_path, 'r') as f:
                self.test_activations_data = json.load(f)
            
            # Validate the JSON structure
            if not isinstance(self.test_activations_data, dict):
                logger.error("Invalid JSON format: root should be a dictionary")
                self.use_predictable_activations = False
                return False
            
            # Validate each cortical area entry
            total_neurons = 0
            valid_areas = 0
            
            for cortical_id, coordinates in self.test_activations_data.items():
                if not isinstance(coordinates, list):
                    logger.warning(f"Invalid coordinates for {cortical_id}: should be a list")
                    continue
                
                valid_coords = 0
                for coord in coordinates:
                    if isinstance(coord, list) and len(coord) == 3:
                        # Validate that coordinates are numbers
                        try:
                            x, y, z = coord
                            if all(isinstance(c, (int, float)) for c in [x, y, z]):
                                valid_coords += 1
                            else:
                                logger.warning(f"Invalid coordinate in {cortical_id}: {coord} - coordinates must be numbers")
                        except (ValueError, TypeError):
                            logger.warning(f"Invalid coordinate in {cortical_id}: {coord}")
                    else:
                        logger.warning(f"Invalid coordinate format in {cortical_id}: {coord} - should be [x,y,z]")
                
                if valid_coords > 0:
                    valid_areas += 1
                    total_neurons += valid_coords
                    logger.debug(f"Loaded {valid_coords} valid coordinates for cortical area {cortical_id}")
                else:
                    logger.warning(f"No valid coordinates found for cortical area {cortical_id}")
            
            if valid_areas > 0:
                self.use_predictable_activations = True
                logger.info(f"✅ Predictable neuron injection enabled:")
                logger.info(f"   📊 {valid_areas} cortical areas with {total_neurons} total neurons to activate")
                logger.info(f"   🎯 Will inject ONLY these neurons (no random selection)")
                return True
            else:
                logger.error("No valid cortical areas found in JSON - falling back to random injection")
                self.use_predictable_activations = False
                return False
                
        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON in test_mode_activations.json: {e}")
            self.use_predictable_activations = False
            return False
        except Exception as e:
            logger.error(f"Error loading test_mode_activations.json: {e}")
            self.use_predictable_activations = False
            return False
    
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
        Validate that genome is available for real neuron injection.
        
        Returns:
            bool: True if genome is available, False otherwise
        """
        try:
            # Get the genome data from the connectome
            genome_data = self.core_api.get_genome()
            
            if not genome_data:
                logger.error("No genome data available for neuron injection")
                return False
                
            # Verify we have cortical areas with neurons
            if not self.connectome.cortical_areas:
                logger.error("No cortical areas found in connectome")
                return False
                
            logger.info(f"Genome validation successful: {len(self.connectome.cortical_areas)} cortical areas available")
            return True
            
        except Exception as e:
            logger.error(f"Error validating genome for neuron injection: {e}")
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
        Inject actual neurons from cortical areas into FCLs.
        
        This method:
        1. If predictable activations are enabled, uses coordinates from JSON file
        2. Otherwise, selects actual neurons from each cortical area randomly
        3. Uses their real coordinates and membrane potentials 
        4. Injects them into the FCL
        
        Returns:
            bool: True if data was injected successfully, False otherwise
        """
        try:
            # Check if we should use predictable activations from JSON
            if self.use_predictable_activations and self.test_activations_data:
                return self._inject_predictable_activations()
            else:
                return self._inject_random_activations()
            
        except Exception as e:
            logger.error(f"Error injecting sensory data: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False
    
    def _inject_predictable_activations(self):
        """
        Inject predictable neuron activations from the JSON file.
        
        Returns:
            bool: True if data was injected successfully, False otherwise
        """
        try:
            total_active_neurons = 0
            active_areas = []
            
            logger.debug(f"Injecting predictable activations for {len(self.test_activations_data)} cortical areas")
            
            for cortical_id, coordinates_list in self.test_activations_data.items():
                try:
                    # Check if this cortical area exists in the connectome
                    if cortical_id not in self.connectome.cortical_areas:
                        logger.warning(f"Cortical area {cortical_id} from JSON not found in connectome - skipping")
                        continue
                    
                    cortical_area = self.connectome.cortical_areas[cortical_id]
                    
                    # Convert coordinates to neuron IDs
                    selected_neurons = []
                    
                    for coord in coordinates_list:
                        if isinstance(coord, list) and len(coord) == 3:
                            try:
                                x, y, z = int(coord[0]), int(coord[1]), int(coord[2])
                                
                                # Find neurons at this coordinate using the correct API
                                neurons_at_position = cortical_area.get_neurons_at_position((x, y, z))
                                if neurons_at_position:
                                    selected_neurons.extend(neurons_at_position)
                                    logger.debug(f"Found {len(neurons_at_position)} neurons at ({x},{y},{z}) in {cortical_id}")
                                else:
                                    logger.debug(f"No neurons found at coordinate ({x},{y},{z}) in {cortical_id}")
                                    
                            except (ValueError, TypeError) as e:
                                logger.warning(f"Invalid coordinate {coord} in {cortical_id}: {e}")
                                continue
                        else:
                            logger.warning(f"Invalid coordinate format {coord} in {cortical_id}")
                            continue
                    
                    if selected_neurons:
                        # Remove duplicates while preserving order
                        selected_neurons = list(dict.fromkeys(selected_neurons))
                        
                        # Create bitmap from selected neuron IDs
                        from feagi.npu.fcl_manager import BitMap
                        bitmap = BitMap(selected_neurons)
                        
                        # Add the bitmap to FCL updates
                        if len(bitmap) > 0:
                            total_active_neurons += len(bitmap)
                            active_areas.append(cortical_id)
                            self.fcl_manager.update_fcl(self.fcl_manager.current_timestep, {cortical_id: bitmap})
                            logger.debug(f"Injected {len(bitmap)} predictable neurons in {cortical_id}")
                    else:
                        logger.warning(f"No valid neurons found for coordinates in {cortical_id}")
                        
                except Exception as e:
                    logger.error(f"Error processing predictable activations for {cortical_id}: {e}")
                    continue
            
            # Summary log
            if total_active_neurons > 0:
                logger.info(f"🎯 Injected {total_active_neurons} PREDICTABLE neurons across {len(active_areas)} areas")
                return True
            else:
                logger.warning("No predictable neurons were successfully injected")
                return False
                
        except Exception as e:
            logger.error(f"Error injecting predictable activations: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False
    
    def _inject_random_activations(self):
        """
        Inject random neuron activations (original behavior).
        
        Returns:
            bool: True if data was injected successfully, False otherwise
        """
        try:
            import random
            
            # Get all cortical areas from the loaded connectome
            cortical_areas = list(self.connectome.cortical_areas.keys())
            
            if not cortical_areas:
                logger.error("No cortical areas found in connectome")
                return False
                
            logger.debug(f"Found {len(cortical_areas)} cortical areas in connectome")
            
            # Process each cortical area
            total_active_neurons = 0
            active_areas = []
            
            for cortical_id in cortical_areas:
                try:
                    # Get the cortical area object
                    cortical_area = self.connectome.cortical_areas[cortical_id]
                    
                    # Get all neurons in this cortical area
                    all_neurons = cortical_area.get_all_neurons()
                    
                    if not all_neurons:
                        logger.debug(f"No neurons found in cortical area {cortical_id}")
                        continue
                        
                    # Randomly select a subset of neurons (5-15% of total)
                    selection_percentage = random.uniform(0.05, 0.15)
                    num_to_select = max(1, int(len(all_neurons) * selection_percentage))
                    selected_neurons = random.sample(list(all_neurons), num_to_select)
                    
                    logger.debug(f"Selected {len(selected_neurons)}/{len(all_neurons)} neurons from {cortical_id}")
                    
                    # Create bitmap from selected neuron IDs
                    from feagi.npu.fcl_manager import BitMap
                    bitmap = BitMap(selected_neurons)
                    
                    # Add the bitmap to FCL updates
                    if len(bitmap) > 0:
                        total_active_neurons += len(bitmap)
                        active_areas.append(cortical_id)
                        self.fcl_manager.update_fcl(self.fcl_manager.current_timestep, {cortical_id: bitmap})
                        
                except Exception as e:
                    logger.error(f"Error processing cortical area {cortical_id}: {e}")
                    continue
            
            # Single summary log
            if total_active_neurons > 0:
                logger.info(f"🎲 Injected {total_active_neurons} RANDOM neurons across {len(active_areas)} areas")
                return True
            else:
                logger.warning("No random neurons were successfully injected")
                return False
                
        except Exception as e:
            logger.error(f"Error injecting random activations: {e}")
            import traceback
            logger.error(traceback.format_exc())
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
                    
                    # Note: Test mode should NOT send visualization data directly
                    # All visualization data should come naturally from FCL -> FQ sampler -> visualization stream
                        
            except Exception as e:
                logger.error(f"Error sending test visualization agent heartbeat: {e}")
                
        logger.info("Heartbeat thread stopped")
    
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
        total_active_neurons = 0
        
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
                total_active_neurons += len(current_fcl_set)
                logger.debug(f"FCL for area {cortical_id} changed: {len(current_fcl_set)} neurons active")
        
        # Single summary log instead of individual area logs
        if changed_fcls:
            logger.info(f"Neural activity: {total_active_neurons} neurons active across {len(active_fcls)} areas")
        
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
            last_report_time = test_start_time
            report_interval = 5.0  # Report every 5 seconds
            
            while time.time() < end_time:
                cycle_count += 1
                
                # Only log cycle numbers every 5 seconds to reduce verbosity
                current_time = time.time()
                if current_time - last_report_time >= report_interval:
                    logger.info(f"Test progress: cycle {cycle_count} ({current_time - test_start_time:.1f}s elapsed)")
                    last_report_time = current_time
                
                # Inject sensory data
                if not self.inject_sensory_data():
                    logger.warning(f"Failed to inject sensory data in cycle {cycle_count}")
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