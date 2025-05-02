"""
Tests for the ConnectomeManager implementation.
"""

import unittest
import numpy as np
import sys
import os
import threading
import time
import tempfile
import logging
from feagi.bdu.connectome_manager import ConnectomeManager, NeuronPropertyType, CorticalArea
from feagi.utils.config import FeagiConfig

class TestConnectomeManager(unittest.TestCase):
    
    def setUp(self):
        # Set up logging
        setup_start = time.time()
        
        logging.basicConfig(level=logging.INFO)  # Reduce logging level
        
        # Create a ConnectomeManager instance with small capacities for testing
        cm_start = time.time()
        self.connectome = ConnectomeManager(max_test_neurons=100)  # Use a smaller array size for faster tests
        cm_end = time.time()
        print(f"ConnectomeManager creation time: {cm_end - cm_start:.6f} seconds")
        
        # Add a test cortical area
        area_start = time.time()
        self.area_id = 1
        self.area = self.connectome.add_cortical_area(
            area_id=self.area_id,
            name="Test Area",
            area_type="interconnect",
            dimensions=(10, 10, 5),
            position=(0, 0, 0)
        )
        area_end = time.time()
        print(f"Test area creation time: {area_end - area_start:.6f} seconds")
        
        # Add an area with extreme dimensions for testing
        extreme_start = time.time()
        self.extreme_area_id = 2
        self.extreme_area = self.connectome.add_cortical_area(
            area_id=self.extreme_area_id,
            name="Extreme Area",
            area_type="interconnect",
            dimensions=(20000, 1, 1),
            position=(0, 10, 0)
        )
        extreme_end = time.time()
        print(f"Extreme area creation time: {extreme_end - extreme_start:.6f} seconds")
        
        setup_end = time.time()
        print(f"Total setup time: {setup_end - setup_start:.6f} seconds")
    
    def test_create_neuron(self):
        """Test neuron creation and retrieval."""
        # Create a neuron
        neuron_id = self.connectome.create_neuron(
            area_id=self.area_id,
            position=(5, 5, 2),
            threshold=1.0,
            refractory_period=5,
            decay_rate=0.9,
            resting_potential=0.0
        )
        
        # Verify neuron exists
        self.assertIn(neuron_id, self.connectome._neuron_id_to_index)
        
        # Verify neuron properties
        threshold = self.connectome.get_neuron_property(neuron_id, NeuronPropertyType.THRESHOLD)
        self.assertEqual(threshold, 1.0)
        
        # Verify neuron position
        position = self.connectome.get_neuron_position(neuron_id)
        self.assertEqual(position, (5, 5, 2))
        
        # Verify area assignment
        neurons_in_area = self.connectome.get_neurons_by_area(self.area_id)
        self.assertIn(neuron_id, neurons_in_area)
    
    def test_create_multiple_neurons(self):
        """Test creation of multiple neurons."""
        # Measure overall time
        start_time = time.time()
        
        # Create a grid of positions
        positions = []
        for x in range(5):
            for y in range(5):
                positions.append((x, y, 0))
        
        # Measure batch creation time
        pre_create = time.time()
        neuron_ids = self.connectome.batch_create_neurons(
            area_id=self.area_id,
            positions=positions,
            threshold=1.0,
            refractory_period=5,
            decay_rate=0.9,
            resting_potential=0.0
        )
        post_create = time.time()
        print(f"Batch creation time: {post_create - pre_create:.6f} seconds")
        
        # Measure verification time
        pre_verify = time.time()
        # Verify neuron count
        self.assertEqual(self.connectome.get_neuron_count(), 25)
        post_verify_count = time.time()
        print(f"Verify count time: {post_verify_count - pre_verify:.6f} seconds")
        
        # Verify all neurons are in the area
        pre_verify_area = time.time()
        neurons_in_area = self.connectome.get_neurons_by_area(self.area_id)
        self.assertEqual(len(neurons_in_area), 25)
        post_verify_area = time.time()
        print(f"Verify area time: {post_verify_area - pre_verify_area:.6f} seconds")
        
        # Delete a neuron
        pre_delete = time.time()
        self.connectome.delete_neuron(neuron_ids[0])
        post_delete = time.time()
        print(f"Delete neuron time: {post_delete - pre_delete:.6f} seconds")
        
        # Verify neuron count decreased
        pre_verify_final = time.time()
        self.assertEqual(self.connectome.get_neuron_count(), 24)
        post_verify_final = time.time()
        print(f"Final verify time: {post_verify_final - pre_verify_final:.6f} seconds")
        
        # Print total time
        end_time = time.time()
        print(f"Total test time: {end_time - start_time:.6f} seconds")
    
    def test_create_synapses(self):
        """Test synapse creation and retrieval."""
        # Create some neurons
        pre_id = self.connectome.create_neuron(
            area_id=self.area_id,
            position=(1, 1, 0)
        )
        
        post_id = self.connectome.create_neuron(
            area_id=self.area_id,
            position=(2, 2, 0)
        )
        
        # Create a synapse
        result = self.connectome.create_synapse(
            pre_neuron_id=pre_id,
            post_neuron_id=post_id,
            weight=1.5,
            is_plastic=True,
            plasticity_coeff=0.1,
            plasticity_decay=0.01
        )
        
        # Verify synapse creation succeeded
        self.assertTrue(result)
        
        # Verify outgoing connections
        outgoing = self.connectome.get_outgoing_connections(pre_id)
        self.assertEqual(len(outgoing), 1)
        post_id_result, weight = outgoing[0]
        self.assertEqual(post_id_result, post_id)
        self.assertEqual(weight, 1.5)
        
        # Verify incoming connections
        incoming = self.connectome.get_incoming_connections(post_id)
        self.assertEqual(len(incoming), 1)
        pre_id_result, weight = incoming[0]
        self.assertEqual(pre_id_result, pre_id)
        self.assertEqual(weight, 1.5)
    
    def test_membrane_potential_update(self):
        """Test updating membrane potentials using the two-phase approach."""
        # Create some neurons
        pre_id = self.connectome.create_neuron(
            area_id=self.area_id,
            position=(1, 1, 0),
            threshold=1.0
        )
        
        post_id = self.connectome.create_neuron(
            area_id=self.area_id,
            position=(2, 2, 0),
            threshold=0.5  # Lower threshold to ensure firing
        )
        
        # Create a synapse from pre to post
        self.connectome.create_synapse(
            pre_neuron_id=pre_id,
            post_neuron_id=post_id,
            weight=1.5  # Weight strong enough to trigger firing
        )
        
        # Get internal indices
        pre_idx = self.connectome._neuron_id_to_index[pre_id]
        post_idx = self.connectome._neuron_id_to_index[post_id]
        
        # Timestep 1: Add pre_neuron to FCL manually (simulating it firing)
        self.connectome.fcl_manager.add_to_current_fcl([pre_idx])
        
        # Verify pre_neuron is in FCL at t=0
        self.assertIn(pre_idx, self.connectome.fcl_manager.get_fcl(0))
        
        # Timestep 2: Update membrane potentials, which should process pre_neuron's firing
        firing_neurons = self.connectome.update_membrane_potentials(current_timestep=1)
        
        # Verify that post_neuron received synaptic input and fired
        self.assertIn(post_id, firing_neurons)
        
        # Verify post_neuron's membrane potential was reset after firing
        self.assertEqual(self.connectome.membrane_potentials[post_idx], 0.0)  # resting potential
        
        # Verify post_neuron is now in the current FCL (t=1)
        self.assertIn(post_idx, self.connectome.fcl_manager.get_fcl(0))
        
        # Verify that pre_neuron is now in FCL at t-1
        self.assertIn(pre_idx, self.connectome.fcl_manager.get_fcl(-1))
        
        # Verify that last_fired was updated for post_neuron
        self.assertEqual(self.connectome.last_fired[post_idx], 1)  # current timestep
    
    def test_neuron_queries(self):
        """Test neuron query methods."""
        # Create neurons with different properties
        for i in range(10):
            self.connectome.create_neuron(
                area_id=self.area_id,
                position=(i, 0, 0),
                threshold=0.5 + i * 0.1,
                refractory_period=5,
                decay_rate=0.9,
                resting_potential=0.0
            )
        
        # Query by threshold range
        low_threshold_neurons = self.connectome.query_neurons_by_threshold_range(0.5, 0.7)
        self.assertEqual(len(low_threshold_neurons), 3)
        
        # Query by position
        position_neurons = self.connectome.query_neurons_by_area_and_position(
            area_id=self.area_id,
            x_range=(5, 9)
        )
        self.assertEqual(len(position_neurons), 5)
        
        # Query by multiple criteria
        multi_query_neurons = self.connectome.query_neurons_by_multiple_criteria(
            area_id=self.area_id,
            position_ranges={'x': (0, 5)},
            threshold_range=(0.5, 0.8)
        )
        self.assertEqual(len(multi_query_neurons), 4)
    
    def test_serialization(self):
        """Test brain state serialization and deserialization."""
        # Create some neurons and synapses
        pre_id = self.connectome.create_neuron(
            area_id=self.area_id,
            position=(1, 1, 0)
        )
        
        post_id = self.connectome.create_neuron(
            area_id=self.area_id,
            position=(2, 2, 0)
        )
        
        self.connectome.create_synapse(
            pre_neuron_id=pre_id,
            post_neuron_id=post_id,
            weight=1.5
        )
        
        # Serialize to a temporary file
        with tempfile.NamedTemporaryFile(suffix='.npz', delete=False) as tmp:
            temp_path = tmp.name
        
        try:
            # Serialize
            success = self.connectome.serialize_brain_state(temp_path)
            self.assertTrue(success)
            
            # Create a new connectome
            new_connectome = ConnectomeManager(max_test_neurons=100)
            
            # Deserialize
            success = new_connectome.deserialize_brain_state(temp_path)
            self.assertTrue(success)
            
            # Verify area was reconstructed
            self.assertIn(self.area_id, new_connectome._areas)
            
            # Verify neurons were reconstructed
            self.assertEqual(new_connectome.get_neuron_count(), 2)
            
            # Verify synapse was reconstructed
            neurons = new_connectome.get_neurons_by_area(self.area_id)
            if len(neurons) >= 2:
                outgoing = new_connectome.get_outgoing_connections(neurons[0])
                # At least one neuron should have an outgoing connection
                has_connection = len(outgoing) > 0 or len(new_connectome.get_outgoing_connections(neurons[1])) > 0
                self.assertTrue(has_connection)
        
        finally:
            # Clean up
            if os.path.exists(temp_path):
                os.unlink(temp_path)

    def test_multiple_neurons_per_voxel(self):
        """Test creating and managing multiple neurons per voxel."""
        # Create multiple neurons at the same position but with different indices
        neuron1 = self.connectome.create_neuron(
            area_id=self.area_id, 
            position=(1, 1, 1), 
            neuron_index=0
        )
        
        neuron2 = self.connectome.create_neuron(
            area_id=self.area_id, 
            position=(1, 1, 1), 
            neuron_index=1
        )
        
        neuron3 = self.connectome.create_neuron(
            area_id=self.area_id, 
            position=(1, 1, 1), 
            neuron_index=2
        )
        
        # Verify all neurons were created
        neurons_at_pos = self.connectome.get_neurons_at_position(self.area_id, (1, 1, 1))
        self.assertEqual(len(neurons_at_pos), 3)
        self.assertIn(neuron1, neurons_at_pos)
        self.assertIn(neuron2, neurons_at_pos)
        self.assertIn(neuron3, neurons_at_pos)
        
        # Verify get_neuron_at_position with specific index
        self.assertEqual(
            self.connectome.get_neuron_at_position(self.area_id, (1, 1, 1), neuron_index=0), 
            neuron1
        )
        self.assertEqual(
            self.connectome.get_neuron_at_position(self.area_id, (1, 1, 1), neuron_index=1), 
            neuron2
        )
        self.assertEqual(
            self.connectome.get_neuron_at_position(self.area_id, (1, 1, 1), neuron_index=2), 
            neuron3
        )
        
        # Verify creating a neuron with an existing index fails
        with self.assertRaises(ValueError):
            self.connectome.create_neuron(
                area_id=self.area_id, 
                position=(1, 1, 1), 
                neuron_index=1
            )
        
        # Delete one neuron and verify others remain
        self.connectome.delete_neuron(neuron2)
        
        neurons_at_pos = self.connectome.get_neurons_at_position(self.area_id, (1, 1, 1))
        self.assertEqual(len(neurons_at_pos), 2)
        self.assertIn(neuron1, neurons_at_pos)
        self.assertIn(neuron3, neurons_at_pos)
        self.assertNotIn(neuron2, neurons_at_pos)
        
        # Now we can create a new neuron with the previously used index
        neuron4 = self.connectome.create_neuron(
            area_id=self.area_id, 
            position=(1, 1, 1), 
            neuron_index=1
        )
        self.assertNotEqual(neuron4, neuron2)  # Should be a different ID
        
        neurons_at_pos = self.connectome.get_neurons_at_position(self.area_id, (1, 1, 1))
        self.assertEqual(len(neurons_at_pos), 3)
    
    def test_extreme_dimension_area(self):
        """Test neurons in an area with extreme dimensions using specialized tracking."""
        # Create neurons at various positions in the extreme dimension area
        neuron1 = self.connectome.create_neuron(
            area_id=self.extreme_area_id, 
            position=(500, 0, 0)
        )
        
        neuron2 = self.connectome.create_neuron(
            area_id=self.extreme_area_id, 
            position=(15000, 0, 0)
        )
        
        neuron3 = self.connectome.create_neuron(
            area_id=self.extreme_area_id, 
            position=(19999, 0, 0)
        )
        
        # Verify positions
        self.assertEqual(self.connectome.get_neuron_position(neuron1), (500, 0, 0))
        self.assertEqual(self.connectome.get_neuron_position(neuron2), (15000, 0, 0))
        self.assertEqual(self.connectome.get_neuron_position(neuron3), (19999, 0, 0))
        
        # Test querying by position range in extreme dimension
        # Small range query
        neurons = self.connectome.query_neurons_by_area_and_position(
            area_id=self.extreme_area_id,
            x_range=(400, 600)
        )
        self.assertEqual(len(neurons), 1)
        self.assertIn(neuron1, neurons)
        
        # Large range query
        neurons = self.connectome.query_neurons_by_area_and_position(
            area_id=self.extreme_area_id,
            x_range=(10000, 19999)
        )
        self.assertEqual(len(neurons), 2)
        self.assertIn(neuron2, neurons)
        self.assertIn(neuron3, neurons)
    
    def test_position_linearization(self):
        """Test the internal position linearization and delinearization."""
        # Test linearization
        linear_pos = self.connectome._linearize_position(self.area_id, 1, 2, 2)
        
        # Test delinearization (should get back the original position)
        x, y, z = self.connectome._delinearize_position(self.area_id, linear_pos)
        self.assertEqual((x, y, z), (1, 2, 2))
        
        # Test with a different position
        linear_pos2 = self.connectome._linearize_position(self.area_id, 7, 8, 3)
        x2, y2, z2 = self.connectome._delinearize_position(self.area_id, linear_pos2)
        self.assertEqual((x2, y2, z2), (7, 8, 3))
        
        # Test with extreme area
        linear_pos3 = self.connectome._linearize_position(self.extreme_area_id, 15000, 0, 0)
        x3, y3, z3 = self.connectome._delinearize_position(self.extreme_area_id, linear_pos3)
        self.assertEqual((x3, y3, z3), (15000, 0, 0))


if __name__ == '__main__':
    unittest.main() 