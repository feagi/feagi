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

        # Simple defaults for numpy-based random generation
        self.neurons_per_area_min = 100
        self.neurons_per_area_max = 1000
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

                self.cortical_area_info[cortical_id] = {
                    "dimensions": dimensions,
                    "area_type": area_type,
                    "volume": volume,
                }

                self.total_available_volume += volume

                logger.debug(
                    f"Area {cortical_id}: dimensions {dimensions}, "
                    f"volume {volume}, type {area_type}"
                )

            except Exception as e:
                logger.warning(f"Error analyzing cortical area {cortical_id}: {e}")
                continue

        logger.info(
            f"Analyzed {len(self.cortical_area_info)} cortical areas"
        )
        logger.info(f"Total coordinate space volume: {self.total_available_volume}")

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
            "🎲 TEST MODE 2: Numpy-based scalable random coordinate generation (using test_genome.json)"
        )
        logger.info(f"   📊 Available cortical areas: {len(self.cortical_area_info)}")
        logger.info(f"   🧠 Total coordinate space volume: {self.total_available_volume}")
        logger.info(f"   🎯 Selected areas for testing: {len(self.selected_areas)}")
        logger.info(
            f"   🔢 Coordinates per area range: {self.neurons_per_area_min}-{self.neurons_per_area_max}"
        )

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
        Generate random coordinate activations using numpy for scalability and submit them via test runner.

        This method acts as a pure sensory data generator, working only with coordinates 
        and membrane potentials, completely unaware of neuron IDs.

        Returns:
            bool: True if data was injected successfully, False otherwise
        """
        try:
            coordinate_activations = {}  # Dictionary to hold coordinate activations for submission

            logger.debug(
                f"Generating random coordinate activations for {len(self.selected_areas)} cortical areas"
            )

            for area_id in self.selected_areas:
                try:
                    area_info = self.cortical_area_info[area_id]
                    dimensions = area_info["dimensions"]
                    width, height, depth = dimensions

                    # Determine number of coordinates to activate - simple random within range
                    num_to_activate = np.random.randint(self.neurons_per_area_min, self.neurons_per_area_max + 1)

                    if num_to_activate <= 0:
                        continue

                    # Generate random coordinates within the cortical area bounds using numpy
                    random_coordinates = []
                    for _ in range(num_to_activate):
                        x = np.random.randint(0, width)
                        y = np.random.randint(0, height)
                        z = np.random.randint(0, depth)
                        random_coordinates.append((x, y, z))

                    coordinate_activations[area_id] = random_coordinates

                    logger.debug(
                        f"Generated {len(random_coordinates)} random coordinates in {area_id} "
                        f"(area dimensions: {width}x{height}x{depth})"
                    )

                except Exception as e:
                    logger.error(f"Error generating coordinate activations for {area_id}: {e}")
                    continue

            # Submit coordinate activations via test runner (proper architecture)
            if coordinate_activations:
                total_coordinates = sum(len(coords) for coords in coordinate_activations.values())
                logger.info(
                    f"🎲 Submitting {total_coordinates} NUMPY-GENERATED coordinates across {len(coordinate_activations)} areas via unified neural stimulation"
                )

                injected_count = self.test_runner.submit_coordinate_activations(
                    coordinate_activations, "test_mode_2_numpy"
                )

                if injected_count > 0:
                    logger.info(
                        f"✅ Successfully injected {injected_count} numpy-generated coordinates"
                    )
                    return True
                else:
                    logger.warning("Failed to inject numpy-generated coordinates")
                    return False
            else:
                logger.warning("No numpy-generated coordinate activations generated")
                return False

        except Exception as e:
            logger.error(f"Error generating numpy-based coordinate activations: {e}")
            import traceback

            logger.error(traceback.format_exc())
            return False
