"""
Test Mode 2: Numpy-based Scalable Random Neuron Generation

This module handles large-scale random neuron stimulation generation using
numpy for scalability testing and performance evaluation.

Test Mode 2 specifically uses the test_genome.json file instead of the essential
genome to provide more cortical areas and neurons for testing scalability.
"""

import random

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

        # Test Mode 2 configuration: sample 20% of existing neurons per area
        self.neuron_sample_percentage = 20.0  # Percentage of neurons to sample per area
        self.area_selection_percentage = 1.0  # Percentage of areas to include

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
                logger.error(
                    "No neurons found in any cortical area - neuroembryogenesis may be incomplete"
                )
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
        """Analyze all cortical areas and collect information about dimensions for coordinate generation."""
        self.cortical_area_info = {}
        self.total_available_volume = 0
        self.total_available_neurons = 0  # CRITICAL FIX: Initialize neuron counter

        for cortical_id, area in self.connectome.cortical_areas.items():
            try:
                # Get area dimensions for coordinate generation
                dimensions = getattr(area, "dimensions_3D", (1, 1, 1))
                if hasattr(dimensions, "__iter__"):
                    dimensions = tuple(dimensions)
                else:
                    dimensions = (1, 1, 1)

                # Get area properties
                properties = getattr(area, "properties", {})
                area_type = properties.get("group", "Unknown")

                # Calculate volume for coordinate space
                volume = dimensions[0] * dimensions[1] * dimensions[2]

                # CRITICAL FIX: Count actual neurons in this area (same as test mode 1)
                neuron_count = len(area.get_all_neurons())
                self.total_available_neurons += neuron_count

                self.cortical_area_info[cortical_id] = {
                    "dimensions": dimensions,
                    "area_type": area_type,
                    "volume": volume,
                    "neuron_count": neuron_count,  # Track neuron count per area
                }

                self.total_available_volume += volume

                logger.debug(
                    f"Area {cortical_id}: dimensions {dimensions}, "
                    f"volume {volume}, neurons {neuron_count}, type {area_type}"
                )

            except Exception as e:
                logger.warning(f"Error analyzing cortical area {cortical_id}: {e}")
                continue

        logger.info(
            f"Analyzed {len(self.cortical_area_info)} cortical areas"
        )
        logger.info(f"Total coordinate space volume: {self.total_available_volume}")
        logger.info(f"Total available neurons: {self.total_available_neurons}")  # Log neuron count

    def _select_test_areas(self):
        """Select cortical areas for testing based on configuration."""
        available_areas = list(self.cortical_area_info.keys())

        if not available_areas:
            self.selected_areas = []
            return

        # Calculate number of areas to select
        num_areas_to_select = max(
            1, int(len(available_areas) * self.area_selection_percentage)
        )

        # Simple random selection
        self.selected_areas = random.sample(available_areas, num_areas_to_select)

        logger.info(
            f"Selected {len(self.selected_areas)} areas for testing: {self.selected_areas}"
        )

    def _log_configuration_summary(self):
        """Log a summary of the test configuration."""
        logger.info(
            "🎯 TEST MODE 2: Neuron sampling from existing cortical areas (using test_genome.json)"
        )
        logger.info(f"   📊 Available cortical areas: {len(self.cortical_area_info)}")
        logger.info(f"   🧠 Total available neurons: {self.total_available_neurons}")
        logger.info(f"   🎯 Selected areas for testing: {len(self.selected_areas)}")
        logger.info(
            f"   📊 Neuron sampling percentage: {self.neuron_sample_percentage}% of existing neurons per area"
        )

    def inject_data(self):
        """
        Generate and inject large-scale random neuron activations.

        Returns:
            bool: True if data was injected successfully, False otherwise
        """
        try:
            return self._inject_direct_neuron_activations()
        except Exception as e:
            logger.error(f"Error injecting Test Mode 2 data: {e}")
            import traceback

            logger.error(traceback.format_exc())
            return False

    def _inject_direct_neuron_activations(self):
        """
        Sample and directly stimulate neurons from each cortical area, bypassing coordinate conversion.

        This method samples actual neurons from genome-created cortical areas, sets their
        membrane potentials directly, and injects them into the FCL without any coordinate
        conversion overhead. This is the most efficient approach for sensory data testing.

        Returns:
            bool: True if neurons were stimulated and injected successfully, False otherwise
        """
        try:
            neuron_activations = {}  # Dictionary to hold neuron IDs for direct FCL injection

            logger.debug(
                f"Sampling {self.neuron_sample_percentage}% of existing neurons from {len(self.selected_areas)} cortical areas"
            )

            for area_id in self.selected_areas:
                try:
                    # Get the cortical area object
                    area = self.connectome.cortical_areas.get(area_id)
                    if not area:
                        logger.warning(f"Cortical area {area_id} not found in connectome")
                        continue

                    # Get all existing neurons in this area
                    all_neurons = area.get_all_neurons()
                    if not all_neurons:
                        logger.warning(f"No neurons found in cortical area {area_id}")
                        continue

                    # Calculate sample size based on configured percentage
                    sample_size = max(1, int(len(all_neurons) * (self.neuron_sample_percentage / 100.0)))
                    sample_size = min(sample_size, len(all_neurons))

                    # Randomly sample the configured percentage of neurons
                    sampled_neurons = np.random.choice(all_neurons, size=sample_size, replace=False)

                    # Directly stimulate sampled neurons (bypass coordinate conversion)
                    if len(sampled_neurons) > 0:
                        # Set membrane potentials directly on sampled neurons
                        membrane_potential = 3.0  # High stimulation value
                        
                        for neuron_id in sampled_neurons:
                            try:
                                self.connectome.set_neuron_property(neuron_id, "membrane_potential", membrane_potential)
                            except Exception as e:
                                logger.warning(f"Failed to set membrane potential for neuron {neuron_id}: {e}")
                        
                        # Collect neurons for FCL injection
                        if area_id not in neuron_activations:
                            neuron_activations[area_id] = []
                        neuron_activations[area_id].extend(sampled_neurons)

                        logger.info(
                            f"Stimulated {len(sampled_neurons)} neurons ({sample_size}/{len(all_neurons)} = "
                            f"{len(sampled_neurons)/len(all_neurons)*100:.1f}%) in {area_id}"
                        )

                except Exception as e:
                    logger.error(f"Error sampling neurons from {area_id}: {e}")
                    continue

            # Directly inject neurons into FCL (bypass coordinate conversion entirely)
            if neuron_activations:
                total_neurons_stimulated = sum(len(neurons) for neurons in neuron_activations.values())
                logger.info(
                    f"🎯 Directly injecting {total_neurons_stimulated} STIMULATED neurons ({self.neuron_sample_percentage}% sample) into FCL across {len(neuron_activations)} areas"
                )

                # Get FCL injection service from burst engine
                try:
                    burst_engine = self.test_runner.burst_engine
                    if burst_engine and hasattr(burst_engine, 'injection_service'):
                        fcl_service = burst_engine.injection_service
                        
                        # Get current timestep from state manager
                        from feagi.core.state_manager import FeagiStateManager
                        state_manager = FeagiStateManager.instance()
                        current_timestep = state_manager.get_current_timestep()
                        
                        # Direct FCL injection (bypass coordinate conversion)
                        injected_count = fcl_service.inject_external_activations(
                            activations=neuron_activations,  # Contains neuron IDs directly - no conversion needed
                            current_timestep=current_timestep,
                            source="test_mode_2_direct"
                        )
                        
                        if injected_count > 0:
                            logger.info(
                                f"✅ Successfully injected {injected_count} neurons directly into FCL (100% hit rate, zero conversion overhead)"
                            )
                            return True
                        else:
                            logger.warning("Failed to inject neurons into FCL")
                            return False
                    else:
                        logger.error("FCL injection service not available")
                        return False
                        
                except Exception as e:
                    logger.error(f"Error in direct FCL injection: {e}")
                    return False
            else:
                logger.warning("No neurons stimulated for injection")
                return False

        except Exception as e:
            logger.error(f"Error generating numpy-based coordinate activations: {e}")
            import traceback

            logger.error(traceback.format_exc())
            return False
