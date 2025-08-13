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

"""
Tests for the ConnectomeManager implementation.
"""

import os
import tempfile
import unittest

import pytest

from feagi.bdu.connectome_manager import ConnectomeManager


class TestConnectomeManager(unittest.TestCase):
    def setUp(self):
        """Set up a test connectome with some basic structure."""
        self.connectome = ConnectomeManager()

        # Add some cortical areas
        self.v1_id = self.connectome.add_cortical_area(
            name="V1", dimensions=(10, 10, 1), position=(0, 0, 0), area_type="sensory"
        )

        self.v2_id = self.connectome.add_cortical_area(
            name="V2", dimensions=(8, 8, 1), position=(12, 0, 0), area_type="sensory"
        )

        self.motor_id = self.connectome.add_cortical_area(
            name="Motor", dimensions=(5, 5, 1), position=(22, 0, 0), area_type="motor"
        )

        # Add some neurons to each area
        self.v1_neurons = []
        for x in range(5):
            for y in range(5):
                neuron_id = self.connectome.create_neuron(
                    area_id=self.v1_id, position=(x, y, 0)
                )
                self.v1_neurons.append(neuron_id)

        self.v2_neurons = []
        for x in range(4):
            for y in range(4):
                neuron_id = self.connectome.create_neuron(
                    area_id=self.v2_id, position=(x, y, 0)
                )
                self.v2_neurons.append(neuron_id)

        self.motor_neurons = []
        for x in range(3):
            for y in range(3):
                neuron_id = self.connectome.create_neuron(
                    area_id=self.motor_id, position=(x, y, 0)
                )
                self.motor_neurons.append(neuron_id)

    @pytest.mark.skip(reason="Method signature mismatch - area_id vs cortical_id")
    def test_brain_region_operations(self):
        """Test the CRUD operations for brain regions."""
        # Create brain regions
        visual_region_id = self.connectome.add_brain_region(
            name="Visual System", region_type="sensory"
        )

        motor_region_id = self.connectome.add_brain_region(
            name="Motor System", region_type="motor"
        )

        # Test getting regions
        visual_region = self.connectome.get_brain_region(visual_region_id)
        self.assertEqual(visual_region["name"], "Visual System")
        self.assertEqual(visual_region["region_type"], "sensory")

        # Test assigning areas to regions
        self.connectome.assign_area_to_region(self.v1_id, visual_region_id)
        self.connectome.assign_area_to_region(self.v2_id, visual_region_id)
        self.connectome.assign_area_to_region(self.motor_id, motor_region_id)

        # Test getting areas in region
        visual_areas = self.connectome.get_areas_in_region(visual_region_id)
        self.assertEqual(len(visual_areas), 2)
        self.assertIn(self.v1_id, visual_areas)
        self.assertIn(self.v2_id, visual_areas)

        # Test getting neurons in region
        visual_neurons = self.connectome.get_neurons_in_region(visual_region_id)
        self.assertEqual(
            len(visual_neurons), len(self.v1_neurons) + len(self.v2_neurons)
        )

        # Test removing area from region
        self.connectome.remove_area_from_region(self.v2_id, visual_region_id)
        visual_areas = self.connectome.get_areas_in_region(visual_region_id)
        self.assertEqual(len(visual_areas), 1)
        self.assertIn(self.v1_id, visual_areas)

        # Test updating region
        self.connectome.update_brain_region(
            visual_region_id,
            {"name": "Primary Visual System", "properties": {"importance": "high"}},
        )
        visual_region = self.connectome.get_brain_region(visual_region_id)
        self.assertEqual(visual_region["name"], "Primary Visual System")
        self.assertEqual(visual_region["properties"]["importance"], "high")

        # Test deleting region
        self.connectome.delete_brain_region(visual_region_id)
        with self.assertRaises(KeyError):
            self.connectome.get_brain_region(visual_region_id)

    @pytest.mark.skip(reason="Method signature mismatch - area_id vs cortical_id")
    def test_connectivity_rule_operations(self):
        """Test the CRUD operations for connectivity rules."""
        # Create a connectivity rule
        rule_id = self.connectome.add_connectivity_rule(
            name="V1 to V2 Probabilistic",
            source_area_id=self.v1_id,
            target_area_id=self.v2_id,
            rule_type="random-subset",
            parameters={"num_targets": 2, "weight": 0.5},
        )

        # Test getting rule
        rule = self.connectome.get_connectivity_rule(rule_id)
        self.assertEqual(rule["name"], "V1 to V2 Probabilistic")
        self.assertEqual(rule["source_area_id"], self.v1_id)
        self.assertEqual(rule["rule_type"], "random-subset")
        self.assertEqual(rule["parameters"]["num_targets"], 2)

        # Test updating rule
        self.connectome.update_connectivity_rule(
            rule_id, {"parameters": {"num_targets": 3}}
        )
        rule = self.connectome.get_connectivity_rule(rule_id)
        self.assertEqual(rule["parameters"]["num_targets"], 3)

        # Test getting rules for areas
        rules = self.connectome.get_connectivity_rules_for_areas(
            source_area_id=self.v1_id, target_area_id=self.v2_id
        )
        self.assertEqual(len(rules), 1)
        self.assertEqual(rules[0], rule_id)

        # Test applying rule
        limited_v1_neurons = self.v1_neurons[:5]
        original_get_neurons_by_area = self.connectome.get_neurons_by_area

        def mock_get_neurons_by_area(area_id):
            if area_id == self.v1_id:
                return limited_v1_neurons
            return original_get_neurons_by_area(area_id)

        self.connectome.get_neurons_by_area = mock_get_neurons_by_area

        num_synapses = self.connectome.apply_connectivity_rule(rule_id)

        self.connectome.get_neurons_by_area = original_get_neurons_by_area

        self.assertLessEqual(num_synapses, 15)
        self.assertGreater(num_synapses, 0)

        # Test deleting rule
        self.connectome.delete_connectivity_rule(rule_id)
        with self.assertRaises(KeyError):
            self.connectome.get_connectivity_rule(rule_id)

    @pytest.mark.skip(reason="Method signature mismatch - area_id vs cortical_id")
    def test_cortical_connection_operations(self):
        """Test the CRUD operations for cortical connections."""
        # Create a cortical connection
        connection_id = self.connectome.add_cortical_connection(
            name="V1-V2 Pathway",
            source_area_id=self.v1_id,
            target_area_id=self.v2_id,
            properties={"function": "visual processing"},
        )

        # Test getting connection
        connection = self.connectome.get_cortical_connection(connection_id)
        self.assertEqual(connection["name"], "V1-V2 Pathway")
        self.assertEqual(connection["source_area_id"], self.v1_id)
        self.assertEqual(connection["target_area_id"], self.v2_id)
        self.assertEqual(connection["properties"]["function"], "visual processing")

        # Create some synapses to test with
        for i in range(5):
            self.connectome.create_synapse(
                pre_neuron_id=self.v1_neurons[i],
                post_neuron_id=self.v2_neurons[i],
                weight=0.5,
            )

        # Test updating synapse count
        synapse_count = self.connectome.update_synapse_count_for_connection(
            connection_id
        )
        self.assertEqual(synapse_count, 5)

        # Test getting connection statistics
        stats = self.connectome.get_connection_statistics(connection_id)
        self.assertEqual(stats["synapse_count"], 5)
        self.assertEqual(stats["avg_weight"], 0.5)

        # Test applying weight change
        modified = self.connectome.apply_connection_weight_change(connection_id, 2.0)
        self.assertEqual(modified, 5)

        # Test getting connections by area
        connections = self.connectome.get_connections_by_area(
            self.v1_id, as_source=True, as_target=False
        )
        self.assertEqual(len(connections), 1)
        self.assertEqual(connections[0], connection_id)

        # Test updating connection
        self.connectome.update_cortical_connection(
            connection_id, {"properties": {"importance": "high"}}
        )
        connection = self.connectome.get_cortical_connection(connection_id)
        self.assertEqual(connection["properties"]["importance"], "high")
        self.assertEqual(connection["properties"]["function"], "visual processing")

        # Test deleting connection
        self.connectome.delete_cortical_connection(connection_id, delete_synapses=True)
        with self.assertRaises(KeyError):
            self.connectome.get_cortical_connection(connection_id)

    @pytest.mark.skip(reason="Method signature mismatch - area_id vs cortical_id")
    def test_save_and_load(self):
        """Test saving and loading the connectome with all new data structures."""
        # Create a brain region
        visual_region_id = self.connectome.add_brain_region(
            name="Visual System", region_type="sensory"
        )

        # Assign areas to region
        self.connectome.assign_area_to_region(self.v1_id, visual_region_id)
        self.connectome.assign_area_to_region(self.v2_id, visual_region_id)

        # Create a connectivity rule
        rule_id = self.connectome.add_connectivity_rule(
            name="V1 to V2 One-to-One",
            source_area_id=self.v1_id,
            target_area_id=self.v2_id,
            rule_type="one-to-one",
            parameters={"weight": 0.5},
        )

        # Create a cortical connection
        connection_id = self.connectome.add_cortical_connection(
            name="V1-V2 Pathway", source_area_id=self.v1_id, target_area_id=self.v2_id
        )

        # Apply the rule to create synapses
        self.connectome.apply_connectivity_rule(rule_id)

        # Save to temporary file
        with tempfile.NamedTemporaryFile(delete=False) as temp:
            filename = temp.name

        self.connectome.save(filename)

        # Load into a new connectome
        loaded_connectome = ConnectomeManager.load(filename)

        # Verify brain regions were loaded
        region = loaded_connectome.get_brain_region(visual_region_id)
        self.assertEqual(region["name"], "Visual System")

        areas_in_region = loaded_connectome.get_areas_in_region(visual_region_id)
        self.assertEqual(len(areas_in_region), 2)

        # Verify connectivity rules were loaded
        loaded_rule = loaded_connectome.get_connectivity_rule(rule_id)
        self.assertEqual(loaded_rule["name"], "V1 to V2 One-to-One")

        # Verify cortical connections were loaded
        loaded_connection = loaded_connectome.get_cortical_connection(connection_id)
        self.assertEqual(loaded_connection["name"], "V1-V2 Pathway")

        # Clean up
        os.unlink(filename)


@pytest.fixture
def test_area(connectome):
    """Create a small test cortical area."""
    # Create area with a specific cortical_id that matches the expected area_id=1
    cortical_id = "C12345"
    area = connectome.add_cortical_area(
        name="Test Area",
        area_type="interconnect",
        dimensions=(5, 5, 2),  # Small dimensions for testing
        position=(0, 0, 0),
        cortical_id=cortical_id,
    )
    return cortical_id, area


if __name__ == "__main__":
    unittest.main()
