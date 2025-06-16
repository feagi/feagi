"""
Test Mode 1: JSON-based Predictable Neuron Activations

This module handles the original JSON-based test mode that uses
test_mode_activations.json for predictable neuron injection.
"""

import json
from pathlib import Path

from feagi.utils.logger import setup_logger

logger = setup_logger("feagi.test_mode.mode_1")


class TestMode1Handler:
    """
    Handler for Test Mode 1: JSON-based predictable neuron activations.

    This mode uses a JSON file (test_mode_activations.json) to specify exactly
    which neurons should be activated at specific coordinates, providing
    deterministic and repeatable testing.
    """

    def __init__(self, test_runner):
        """
        Initialize the Test Mode 1 handler.

        Args:
            test_runner: Reference to the main FeagiTestRunner instance
        """
        self.test_runner = test_runner
        self.connectome = test_runner.connectome
        self.fcl_manager = test_runner.fcl_manager

        # Test activations data
        self.test_activations_data = None
        self.use_predictable_activations = False

        # Statistics
        self.total_neurons_to_activate = 0
        self.valid_areas_count = 0

    def initialize(self):
        """
        Initialize Test Mode 1 by loading the JSON file.

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        try:
            # Double-check brain readiness before proceeding
            if not self.test_runner.state_manager.get_brain_readiness():
                logger.error("Brain is not ready for neuron injection")
                return False

            # Get the genome data from the core API
            genome_data = self.test_runner.core_api.get_genome()

            if not genome_data:
                logger.error("No genome data available for neuron injection")
                return False

            # Verify we have cortical areas with neurons
            if not self.connectome.cortical_areas:
                logger.error("No cortical areas found in connectome")
                return False

            # Verify cortical areas actually have neurons (neuroembryogenesis completed)
            total_neurons = 0
            areas_with_neurons = 0
            for _cortical_id, area in self.connectome.cortical_areas.items():
                neuron_count = len(area.get_all_neurons())
                total_neurons += neuron_count
                if neuron_count > 0:
                    areas_with_neurons += 1

            if total_neurons == 0:
                logger.error(
                    "No neurons found in any cortical area - neuroembryogenesis may be incomplete"
                )
                return False

            logger.info(
                f"Genome validation successful: {len(self.connectome.cortical_areas)} cortical areas, {total_neurons} neurons total"
            )
            logger.info(
                f"Areas with neurons: {areas_with_neurons}/{len(self.connectome.cortical_areas)}"
            )

            # Load the JSON file
            return self.load_test_activations_json()

        except Exception as e:
            logger.error(f"Error initializing Test Mode 1: {e}")
            import traceback

            logger.error(traceback.format_exc())
            return False

    def load_test_activations_json(self):
        """
        Load predictable neuron activations from test_mode_activations.json.

        If the file exists in the same folder as the main test_mode.py module,
        it will be loaded and used for predictable neuron injection.

        Returns:
            bool: True if JSON file was loaded successfully, False otherwise
        """
        try:
            # Get the directory where the original test_mode.py module is located
            # This ensures backwards compatibility with existing JSON files
            from feagi import test_mode as legacy_test_mode

            module_dir = Path(legacy_test_mode.__file__).parent
            json_path = module_dir / "test_mode_activations.json"

            if not json_path.exists():
                logger.info(
                    "No test_mode_activations.json found - using random neuron injection fallback"
                )
                self.use_predictable_activations = False
                return True  # Not an error - we can fall back to random

            logger.info(f"Loading predictable neuron activations from: {json_path}")

            with json_path.open("r") as f:
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
                    logger.warning(
                        f"Invalid coordinates for {cortical_id}: should be a list"
                    )
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
                                logger.warning(
                                    f"Invalid coordinate in {cortical_id}: {coord} - coordinates must be numbers"
                                )
                        except (ValueError, TypeError):
                            logger.warning(
                                f"Invalid coordinate in {cortical_id}: {coord}"
                            )
                    else:
                        logger.warning(
                            f"Invalid coordinate format in {cortical_id}: {coord} - should be [x,y,z]"
                        )

                if valid_coords > 0:
                    valid_areas += 1
                    total_neurons += valid_coords
                    logger.debug(
                        f"Loaded {valid_coords} valid coordinates for cortical area {cortical_id}"
                    )
                else:
                    logger.warning(
                        f"No valid coordinates found for cortical area {cortical_id}"
                    )

            if valid_areas > 0:
                self.use_predictable_activations = True
                self.valid_areas_count = valid_areas
                self.total_neurons_to_activate = total_neurons

                logger.info("✅ Predictable neuron injection enabled:")
                logger.info(
                    f"   📊 {valid_areas} cortical areas with {total_neurons} total neurons to activate"
                )
                logger.info(
                    "   🎯 Will inject ONLY these neurons (no random selection)"
                )
                return True
            else:
                logger.error(
                    "No valid cortical areas found in JSON - falling back to random injection"
                )
                self.use_predictable_activations = False
                return True  # Not a failure - we can fall back

        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON in test_mode_activations.json: {e}")
            self.use_predictable_activations = False
            return False
        except Exception as e:
            logger.error(f"Error loading test_mode_activations.json: {e}")
            import traceback

            logger.error(traceback.format_exc())
            self.use_predictable_activations = False
            return False

    def inject_data(self):
        """
        Inject neuron activations based on the JSON configuration.

        Returns:
            bool: True if data was injected successfully, False otherwise
        """
        try:
            if self.use_predictable_activations and self.test_activations_data:
                return self._inject_predictable_activations()
            else:
                return self._inject_random_activations_fallback()
        except Exception as e:
            logger.error(f"Error injecting Test Mode 1 data: {e}")
            import traceback

            logger.error(traceback.format_exc())
            return False

    def _inject_predictable_activations(self):
        """
        Generate predictable neuron activations from the JSON file and submit them via test runner.

        This method separates neuron selection logic from injection mechanism,
        following proper architectural separation of concerns.

        Returns:
            bool: True if data was injected successfully, False otherwise
        """
        try:
            activations = {}  # Dictionary to hold activations for submission

            logger.debug(
                f"Generating predictable activations for {len(self.test_activations_data)} cortical areas"
            )

            for cortical_id, coordinates_list in self.test_activations_data.items():
                try:
                    # Check if this cortical area exists in the connectome
                    if cortical_id not in self.connectome.cortical_areas:
                        logger.warning(
                            f"Cortical area {cortical_id} from JSON not found in connectome - skipping"
                        )
                        continue

                    cortical_area = self.connectome.cortical_areas[cortical_id]

                    # Convert coordinates to neuron IDs
                    selected_neurons = []

                    for coord in coordinates_list:
                        if isinstance(coord, list) and len(coord) == 3:
                            try:
                                x, y, z = int(coord[0]), int(coord[1]), int(coord[2])

                                # Find neurons at this coordinate using the correct API
                                neurons_at_position = (
                                    cortical_area.get_neurons_at_position((x, y, z))
                                )
                                if neurons_at_position:
                                    selected_neurons.extend(neurons_at_position)
                                    logger.debug(
                                        f"Found {len(neurons_at_position)} neurons at ({x},{y},{z}) in {cortical_id}"
                                    )
                                else:
                                    logger.debug(
                                        f"No neurons found at coordinate ({x},{y},{z}) in {cortical_id}"
                                    )

                            except (ValueError, TypeError) as e:
                                logger.warning(
                                    f"Invalid coordinate {coord} in {cortical_id}: {e}"
                                )
                                continue
                        else:
                            logger.warning(
                                f"Invalid coordinate format {coord} in {cortical_id}"
                            )
                            continue

                    if selected_neurons:
                        # Remove duplicates while preserving order
                        selected_neurons = list(dict.fromkeys(selected_neurons))
                        activations[cortical_id] = selected_neurons
                        logger.debug(
                            f"Prepared {len(selected_neurons)} neurons for activation in {cortical_id}"
                        )
                    else:
                        logger.warning(
                            f"No valid neurons found for coordinates in {cortical_id}"
                        )

                except Exception as e:
                    logger.error(
                        f"Error processing predictable activations for {cortical_id}: {e}"
                    )
                    continue

            # Submit activations via test runner (proper architecture)
            if activations:
                total_neurons = sum(len(neurons) for neurons in activations.values())
                logger.debug(
                    f"🎯 Submitting {total_neurons} PREDICTABLE neurons across {len(activations)} areas via FCL injection service"
                )

                injected_count = self.test_runner.submit_neuron_activations(
                    activations, "test_mode_1_predictable"
                )

                if injected_count > 0:
                    logger.debug(
                        f"✅ Successfully injected {injected_count} predictable neurons"
                    )
                    return True
                else:
                    logger.warning("Failed to inject predictable neurons")
                    return False
            else:
                logger.warning("No predictable activations generated")
                return False

        except Exception as e:
            logger.error(f"Error generating predictable activations: {e}")
            import traceback

            logger.error(traceback.format_exc())
            return False

    def _inject_random_activations_fallback(self):
        """
        Generate random neuron activations and submit them via test runner.

        This method separates neuron selection logic from injection mechanism,
        following proper architectural separation of concerns.

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

            # Generate activations
            activations = {}

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

                    activations[cortical_id] = selected_neurons
                    logger.debug(
                        f"Selected {len(selected_neurons)}/{len(all_neurons)} neurons from {cortical_id}"
                    )

                except Exception as e:
                    logger.error(f"Error processing cortical area {cortical_id}: {e}")
                    continue

            # Submit activations via test runner (proper architecture)
            if activations:
                total_neurons = sum(len(neurons) for neurons in activations.values())
                logger.debug(
                    f"🎲 Submitting {total_neurons} RANDOM neurons across {len(activations)} areas via FCL injection service"
                )

                injected_count = self.test_runner.submit_neuron_activations(
                    activations, "test_mode_1_random"
                )

                if injected_count > 0:
                    logger.debug(
                        f"✅ Successfully injected {injected_count} random neurons (fallback mode)"
                    )
                    return True
                else:
                    logger.warning("Failed to inject random neurons")
                    return False
            else:
                logger.warning("No random activations generated")
                return False

        except Exception as e:
            logger.error(f"Error generating random activations: {e}")
            import traceback

            logger.error(traceback.format_exc())
            return False
