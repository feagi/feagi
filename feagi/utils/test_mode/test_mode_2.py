"""
Test Mode 2: Numpy-based Scalable Random Neuron Generation

This module handles large-scale random neuron stimulation generation using
numpy for scalability testing and performance evaluation.

Test Mode 2 specifically uses the test_genome.json file instead of the essential
genome to provide more cortical areas and neurons for testing scalability.
"""

import logging
import random
from typing import Dict, List, Any, Optional, Tuple
import numpy as np

from feagi.utils.logger import setup_logger

logger = setup_logger("feagi.test_mode.mode_2")


class TestMode2Handler:
    """
    Handler for Test Mode 2: Numpy-based scalable random neuron generation.
    
    This mode uses numpy to generate large random neuron stimulations that
    fit within the boundaries of available cortical areas for scalability testing.
    """
    
    def __init__(self, test_runner):
        """
        Initialize the Test Mode 2 handler.
        
        Args:
            test_runner: Reference to the main FeagiTestRunner instance
        """
        self.test_runner = test_runner
        self.connectome = test_runner.connectome
        self.fcl_manager = test_runner.fcl_manager
        
        # Simple defaults for numpy-based random generation
        self.neurons_per_area_min = 100
        self.neurons_per_area_max = 2000
        self.area_selection_percentage = 1.0
        
        # Statistics and state
        self.cortical_area_info = {}
        self.total_available_neurons = 0
        self.selected_areas = []
    
    def initialize(self):
        """
        Initialize Test Mode 2 by analyzing available cortical areas.
        
        Returns:
            bool: True if initialization was successful, False otherwise
        """
        try:
            # Double-check brain readiness before proceeding
            if not self.test_runner.state_manager.get_brain_readiness():
                logger.error("Brain is not ready for neuron injection")
                return False
            
            # Get the genome data from the core API - should be test_genome.json for test mode 2
            genome_data = self.test_runner.core_api.get_genome()
            
            if not genome_data:
                logger.error("No genome data available for neuron injection")
                return False
                
            # Verify we have cortical areas with neurons
            if not self.connectome.cortical_areas:
                logger.error("No cortical areas found in connectome")
                return False
            
            # Analyze cortical areas and build information map
            self._analyze_cortical_areas()
            
            if self.total_available_neurons == 0:
                logger.error("No neurons found in any cortical area - neuroembryogenesis may be incomplete")
                return False
            
            # Select areas for testing
            self._select_test_areas()
            
            if not self.selected_areas:
                logger.error("No cortical areas selected for testing")
                return False
            
            # Log configuration summary
            self._log_configuration_summary()
            
            return True
            
        except Exception as e:
            logger.error(f"Error initializing Test Mode 2: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False
    
    def _analyze_cortical_areas(self):
        """Analyze all cortical areas and collect information about neuron counts and dimensions."""
        self.cortical_area_info = {}
        self.total_available_neurons = 0
        
        for cortical_id, area in self.connectome.cortical_areas.items():
            try:
                # Get all neurons in this cortical area
                all_neurons = area.get_all_neurons()
                neuron_count = len(all_neurons)
                
                if neuron_count > 0:
                    # Get area dimensions
                    dimensions = getattr(area, 'dimensions_3D', (1, 1, 1))
                    if hasattr(dimensions, '__iter__'):
                        dimensions = tuple(dimensions)
                    else:
                        dimensions = (1, 1, 1)
                    
                    # Get area properties
                    properties = getattr(area, 'properties', {})
                    area_type = properties.get('group', 'Unknown')
                    
                    self.cortical_area_info[cortical_id] = {
                        'neuron_count': neuron_count,
                        'dimensions': dimensions,
                        'area_type': area_type,
                        'all_neurons': list(all_neurons),
                        'volume': dimensions[0] * dimensions[1] * dimensions[2],
                        'density': neuron_count / (dimensions[0] * dimensions[1] * dimensions[2])
                    }
                    
                    self.total_available_neurons += neuron_count
                    
                    logger.debug(f"Area {cortical_id}: {neuron_count} neurons, "
                               f"dimensions {dimensions}, type {area_type}")
                else:
                    logger.debug(f"Area {cortical_id}: No neurons found")
                    
            except Exception as e:
                logger.warning(f"Error analyzing cortical area {cortical_id}: {e}")
                continue
        
        logger.info(f"Analyzed {len(self.cortical_area_info)} cortical areas with neurons")
        logger.info(f"Total available neurons: {self.total_available_neurons}")
    
    def _select_test_areas(self):
        """Select cortical areas for testing based on configuration."""
        available_areas = list(self.cortical_area_info.keys())
        
        if not available_areas:
            self.selected_areas = []
            return
        
        # Calculate number of areas to select
        num_areas_to_select = max(1, int(len(available_areas) * self.area_selection_percentage))
        
        # Simple random selection
        self.selected_areas = random.sample(available_areas, num_areas_to_select)
        
        logger.info(f"Selected {len(self.selected_areas)} areas for testing: {self.selected_areas}")
    
    def _log_configuration_summary(self):
        """Log a summary of the test configuration."""
        logger.info("🎲 TEST MODE 2: Numpy-based scalable random neuron generation (using test_genome.json)")
        logger.info(f"   📊 Available cortical areas: {len(self.cortical_area_info)}")
        logger.info(f"   🧠 Total available neurons: {self.total_available_neurons}")
        logger.info(f"   🎯 Selected areas for testing: {len(self.selected_areas)}")
        logger.info(f"   🔢 Neurons per area range: {self.neurons_per_area_min}-{self.neurons_per_area_max}")
    
    def inject_data(self):
        """
        Generate and inject large-scale random neuron activations.
        
        Returns:
            bool: True if data was injected successfully, False otherwise
        """
        try:
            return self._inject_numpy_generated_activations()
        except Exception as e:
            logger.error(f"Error injecting Test Mode 2 data: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False
    
    def _inject_numpy_generated_activations(self):
        """
        Generate and inject random activations using numpy for scalability.
        
        Returns:
            bool: True if data was injected successfully, False otherwise
        """
        try:
            total_active_neurons = 0
            active_areas = []
            
            logger.debug(f"Generating random activations for {len(self.selected_areas)} cortical areas")
            
            for area_id in self.selected_areas:
                try:
                    area_info = self.cortical_area_info[area_id]
                    available_neurons = area_info['all_neurons']
                    
                    if not available_neurons:
                        logger.debug(f"No neurons available in area {area_id}")
                        continue
                    
                    # Determine number of neurons to activate - simple random within range
                    max_neurons = min(len(available_neurons), self.neurons_per_area_max)
                    min_neurons = min(self.neurons_per_area_min, max_neurons)
                    num_to_activate = np.random.randint(min_neurons, max_neurons + 1)
                    
                    if num_to_activate <= 0:
                        continue
                    
                    # Use numpy for efficient random selection
                    selected_indices = np.random.choice(
                        len(available_neurons),
                        size=min(num_to_activate, len(available_neurons)),
                        replace=False
                    )
                    
                    selected_neurons = [available_neurons[i] for i in selected_indices]
                    
                    # Create bitmap from selected neuron IDs
                    from feagi.npu.fcl_manager import BitMap
                    bitmap = BitMap(selected_neurons)
                    
                    # Add the bitmap to FCL updates
                    if len(bitmap) > 0:
                        total_active_neurons += len(bitmap)
                        active_areas.append(area_id)
                        self.fcl_manager.update_fcl(
                            self.fcl_manager.current_timestep, 
                            {area_id: bitmap}
                        )
                        
                        logger.debug(f"Generated {len(bitmap)} random neurons in {area_id} "
                                   f"(density: {len(bitmap)/area_info['neuron_count']:.2%})")
                    
                        
                except Exception as e:
                    logger.error(f"Error generating activations for {area_id}: {e}")
                    continue
            
            # Summary log
            if total_active_neurons > 0:
                logger.info(f"🎲 Generated {total_active_neurons} RANDOM neurons across {len(active_areas)} areas")
                return True
            else:
                logger.warning("No random neurons were successfully generated")
                return False
                
        except Exception as e:
            logger.error(f"Error generating numpy-based activations: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False
    
