"""
Integration test for optimized data structures.

This test demonstrates how to use the optimized data structures in a real-world simulation.
"""

import pytest
import random
import numpy as np
import logging
from typing import List, Dict, Any, Union

# Try to import optimized integration
try:
    from feagi.npu.optimized_integration import (
        create_optimized_core,
        get_core_property,
        set_core_property,
        step_simulation,
        propagate_activations,
        add_connection,
        get_membrane_potential,
        set_membrane_potential,
    )
    OPTIMIZED_AVAILABLE = True
except ImportError:
    OPTIMIZED_AVAILABLE = False
    pytest.skip("Optimized structures not available", allow_module_level=True)

class TestOptimizedIntegration:
    """Test the integration of optimized data structures into a simple simulation."""
    
    @pytest.fixture
    def small_network(self):
        """Create a small network for testing."""
        # Create a core with 1000 neurons
        core = create_optimized_core(1000)
        
        # Create a simple feedforward network:
        # Layer 1 (input): Neurons 0-99
        # Layer 2 (hidden): Neurons 100-199 
        # Layer 3 (output): Neurons 200-299
        
        # Connect input to hidden (fully connected)
        for source in range(100):
            for target in range(100, 200):
                # Random weight between 0.1 and 0.5
                weight = 0.1 + 0.4 * random.random()
                add_connection(core, source, target, weight)
        
        # Connect hidden to output (fully connected)
        for source in range(100, 200):
            for target in range(200, 300):
                # Random weight between 0.1 and 0.5
                weight = 0.1 + 0.4 * random.random()
                add_connection(core, source, target, weight)
        
        return core
    
    def test_network_structure(self, small_network):
        """Test that the network structure is correct."""
        # Skip this test if no direct way to get connection count
        pytest.skip("No direct way to get connection count from optimized core")
    
    def test_single_step(self, small_network):
        """Test a single step of the simulation."""
        # Set some input neurons to high membrane potential
        for i in range(10):
            # Choose a random input neuron
            neuron_id = random.randint(0, 99)
            # Set it to 2.0 (above threshold)
            set_membrane_potential(small_network, neuron_id, 2.0)
        
        # Before the step, the core should be at timestep 0
        assert get_core_property(small_network, "current_timestep") == 0
        
        # Step the simulation
        step_simulation(small_network)
        
        # After the step, the core should be at timestep 1
        assert get_core_property(small_network, "current_timestep") == 1
        
        # Input neurons that were above threshold should now be at 0.0
        for i in range(100):
            if get_membrane_potential(small_network, i) > 1.0:
                # This neuron should have fired
                assert get_membrane_potential(small_network, i) == 0.0
    
    def test_activation_propagation(self, small_network):
        """Test that activations propagate correctly through the network."""
        # Set all input neurons to high membrane potential
        for i in range(100):
            set_membrane_potential(small_network, i, 2.0)
        
        # Step the simulation
        step_simulation(small_network)
        
        # Propagate activations
        activations = propagate_activations(small_network)
        
        # Hidden neurons should now have non-zero activations
        hidden_neurons_active = False
        for i in range(100, 200):
            if activations[i] > 0.0:
                hidden_neurons_active = True
                break
        
        assert hidden_neurons_active, "Hidden neurons should be active after propagation"
    
    def test_multi_step_simulation(self, small_network):
        """Test a multi-step simulation."""
        # Set a pattern of input neurons
        for i in range(0, 100, 2):  # Every other input neuron
            set_membrane_potential(small_network, i, 2.0)
        
        # Run for 5 timesteps
        for i in range(5):
            step_simulation(small_network)
            propagate_activations(small_network)
        
        # Core should now be at timestep 5
        assert get_core_property(small_network, "current_timestep") == 5
    
    def test_with_noisy_input(self, small_network):
        """Test the simulation with noisy input."""
        # Run a longer simulation with random input
        for step in range(20):
            # Reset input neurons
            for i in range(100):
                set_membrane_potential(small_network, i, 0.0)
            
            # Set random input neurons to high membrane potential
            num_active = random.randint(5, 20)  # Random number of active neurons
            for _ in range(num_active):
                neuron_id = random.randint(0, 99)
                set_membrane_potential(small_network, neuron_id, 2.0)
            
            # Step the simulation
            step_simulation(small_network)
            
            # Propagate activations
            propagate_activations(small_network)
        
        # Core should now be at timestep 20
        assert get_core_property(small_network, "current_timestep") == 20
    
    @pytest.mark.skipif(not OPTIMIZED_AVAILABLE, reason="Full optimized structures not available")
    def test_pytorch_integration(self, small_network):
        """Test integration with PyTorch for downstream processing."""
        try:
            import torch
            has_torch = True
        except ImportError:
            has_torch = False
            pytest.skip("PyTorch not available")
        
        if has_torch:
            # Set a pattern of input neurons
            for i in range(0, 100, 2):  # Every other input neuron
                set_membrane_potential(small_network, i, 2.0)
            
            # Step the simulation
            step_simulation(small_network)
            
            # Get activations
            activations = propagate_activations(small_network)
            
            # Convert to PyTorch tensor
            activations_tensor = torch.tensor(activations, dtype=torch.float32)
            
            # Extract output layer activations
            output_activations = activations_tensor[200:300]
            
            # Verify shape
            assert output_activations.shape == (100,) 