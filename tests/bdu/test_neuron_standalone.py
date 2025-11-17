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
Standalone test file for Neuron and NeuronArray classes.

This test directly defines the necessary classes without requiring imports.
"""

import numpy as np
import pytest


class Neuron:
    """Wrapper class for representing a neuron with a friendly API."""

    def __init__(
        self,
        neuron_id,
        area_id,
        position,
        threshold=1.0,
        membrane_potential=0.0,
        leak_coefficient=0.5,
        **kwargs,
    ):
        """
        Initialize a neuron object.

        Args:
            neuron_id: Unique identifier for this neuron
            area_id: ID of the cortical area this neuron belongs to
            position: 3D coordinates (x, y, z) within the cortical area
            threshold: Firing threshold
            membrane_potential: Initial membrane potential
            decay_rate: Membrane potential decay rate
            **kwargs: Additional properties
        """
        self.id = neuron_id
        self.area_id = area_id
        self.position = position
        self.threshold = threshold
        self.membrane_potential = membrane_potential
        self.decay_rate = decay_rate

        # Store any additional properties
        for key, value in kwargs.items():
            setattr(self, key, value)

    @classmethod
    def from_dict(cls, data):
        """
        Create a neuron from a dictionary representation.

        Args:
            data: Dictionary containing neuron properties

        Returns:
            A new Neuron instance
        """
        # Extract required properties
        neuron_id = data.get("id")
        area_id = data.get("area_id")
        position = data.get("position")

        # Extract optional properties with defaults
        threshold = data.get("threshold", 1.0)
        membrane_potential = data.get("membrane_potential", 0.0)
        decay_rate = data.get("decay_rate", 0.5)

        # Create the neuron
        neuron = cls(
            neuron_id=neuron_id,
            area_id=area_id,
            position=position,
            threshold=threshold,
            membrane_potential=membrane_potential,
            leak_coefficient=decay_rate,
        )

        # Add any additional properties
        for key, value in data.items():
            if key not in [
                "id",
                "area_id",
                "position",
                "threshold",
                "membrane_potential",
                "decay_rate",
            ]:
                setattr(neuron, key, value)

        return neuron

    def to_dict(self):
        """
        Convert the neuron to a dictionary representation.

        Returns:
            Dictionary of neuron properties
        """
        return {
            "id": self.id,
            "area_id": self.area_id,
            "position": self.position,
            "threshold": self.threshold,
            "membrane_potential": self.membrane_potential,
            "decay_rate": self.decay_rate,
        }


class NeuronArray:
    """
    Standalone test version of NeuronArray for isolated testing.
    Uses coordinates_x/y/z naming convention for consistency.
    """

    def __init__(self, max_neurons: int):
        """Initialize the NeuronArray with specified capacity."""
        self.max_neurons = max_neurons
        self.neuron_count = 0

        # Initialize arrays - using coordinates_x/y/z for consistency
        self.valid_mask = np.zeros(max_neurons, dtype=bool)
        self.membrane_potentials = np.zeros(max_neurons, dtype=np.float32)
        self.thresholds = np.ones(max_neurons, dtype=np.float32)
        self.cortical_idxs = np.zeros(max_neurons, dtype=np.uint16)
        self.coordinates_x = np.zeros(
            max_neurons, dtype=np.uint16
        )  # ✅ OPTIMIZED: Use uint16 coordinates
        self.coordinates_y = np.zeros(
            max_neurons, dtype=np.uint16
        )  # ✅ OPTIMIZED: Use uint16 coordinates
        self.coordinates_z = np.zeros(
            max_neurons, dtype=np.uint16
        )  # ✅ OPTIMIZED: Use uint16 coordinates

        # Area mapping and activation  
        self.cortical_idxs = np.zeros(max_neurons, dtype=np.uint16)
        self.is_active = np.zeros(max_neurons, dtype=np.bool_)

        # Core neuron properties (numeric)
        self.decay_rates = np.ones(max_neurons, dtype=np.float32) * 0.5
        self.refractory_periods = np.ones(max_neurons, dtype=np.int32)
        self.refractory_counters = np.zeros(max_neurons, dtype=np.int32)

        # Area IDs
        self.area_ids = np.zeros(max_neurons, dtype=object)

        # Next available index
        self.next_index = 0

        # Device flag (CPU by default)
        self.device = "cpu"

    def create_neuron(
        self, area_id, position, threshold=1.0, membrane_potential=0.0, leak_coefficient=0.5
    ):
        """
        Create a new neuron and store it in the array.

        Args:
            area_id: ID of the cortical area
            position: 3D position tuple (x, y, z)
            threshold: Firing threshold
            membrane_potential: Initial membrane potential
            decay_rate: Potential decay rate

        Returns:
            Index (ID) of the created neuron
        """
        # Check if array is full
        if self.next_index >= self.max_neurons:
            return None

        # Get next available index
        neuron_id = self.next_index
        self.next_index += 1

        # Set values
        self.area_ids[neuron_id] = area_id
        self.coordinates_x[neuron_id] = position[0]
        self.coordinates_y[neuron_id] = position[1]
        self.coordinates_z[neuron_id] = position[2]
        self.thresholds[neuron_id] = threshold
        self.membrane_potentials[neuron_id] = membrane_potential
        self.decay_rates[neuron_id] = decay_rate
        self.valid_mask[neuron_id] = True

        return neuron_id

    def get_neuron_property(self, neuron_id, property_name):
        """
        Get a property of a neuron by ID.

        Args:
            neuron_id: ID of the neuron
            property_name: Name of the property to get

        Returns:
            Value of the requested property

        Raises:
            KeyError: If neuron_id is invalid or property doesn't exist
        """
        # Check if neuron ID is valid
        if neuron_id >= self.max_neurons or neuron_id < 0:
            raise KeyError(f"Neuron {neuron_id} does not exist (index out of range)")

        # Check if neuron exists
        if not self.valid_mask[neuron_id]:
            raise KeyError(f"Neuron {neuron_id} does not exist")

        # Return requested property
        if property_name == "area_id":
            return self.area_ids[neuron_id]
        elif property_name == "position":
            return (
                self.coordinates_x[neuron_id],
                self.coordinates_y[neuron_id],
                self.coordinates_z[neuron_id],
            )
        elif property_name == "threshold":
            return self.thresholds[neuron_id]
        elif property_name == "membrane_potential":
            return self.membrane_potentials[neuron_id]
        elif property_name == "decay_rate":
            return self.decay_rates[neuron_id]
        else:
            raise KeyError(f"Unknown property: {property_name}")

    def set_neuron_property(self, neuron_id, property_name, value):
        """
        Set a property of a neuron by ID.

        Args:
            neuron_id: ID of the neuron
            property_name: Name of the property to set
            value: New value for the property

        Raises:
            KeyError: If neuron_id is invalid or property doesn't exist
            ValueError: If the value is invalid for the property
        """
        # Check if neuron ID is valid
        if neuron_id >= self.max_neurons or neuron_id < 0:
            raise KeyError(f"Neuron {neuron_id} does not exist (index out of range)")

        # Check if neuron exists
        if not self.valid_mask[neuron_id]:
            raise KeyError(f"Neuron {neuron_id} does not exist")

        # Set the property
        if property_name == "area_id":
            self.area_ids[neuron_id] = value
        elif property_name == "position":
            if not isinstance(value, tuple) or len(value) != 3:
                raise ValueError("Position must be a 3-tuple (x, y, z)")
            self.coordinates_x[neuron_id] = value[0]
            self.coordinates_y[neuron_id] = value[1]
            self.coordinates_z[neuron_id] = value[2]
        elif property_name == "threshold":
            self.thresholds[neuron_id] = value
        elif property_name == "membrane_potential":
            self.membrane_potentials[neuron_id] = value
        elif property_name == "decay_rate":
            self.decay_rates[neuron_id] = value
        else:
            raise KeyError(f"Unknown property: {property_name}")

    def delete_neuron(self, neuron_id):
        """
        Delete a neuron by ID.

        Args:
            neuron_id: ID of the neuron to delete

        Returns:
            True if neuron was deleted, False if it didn't exist
        """
        # Check if neuron exists
        if not self.valid_mask[neuron_id]:
            return False

        # Mark as invalid
        self.valid_mask[neuron_id] = False

        # Reset values to defaults
        self.membrane_potentials[neuron_id] = 0.0
        self.thresholds[neuron_id] = 1.0
        self.decay_rates[neuron_id] = 0.5
        self.refractory_counters[neuron_id] = 0

        return True

    def get_neurons_by_area(self, area_id):
        """
        Get all neurons in a specific cortical area.

        Args:
            area_id: ID of the cortical area

        Returns:
            List of neuron IDs in the specified area
        """
        # Find neurons in this area
        neuron_ids = []
        for neuron_id in range(self.next_index):
            if self.valid_mask[neuron_id] and self.area_ids[neuron_id] == area_id:
                neuron_ids.append(neuron_id)

        return neuron_ids

    def get_neuron_count(self):
        """
        Get the total number of valid neurons.

        Returns:
            Count of valid neurons
        """
        return np.sum(self.valid_mask)

    def to_gpu(self):
        """
        Transfer all neuron data to GPU.

        Returns:
            True if transfer was successful, False if PyTorch/CUDA not available
        """
        try:
            import torch

            if not torch.cuda.is_available():
                return False

            # Convert numpy arrays to CUDA tensors
            self.membrane_potentials = torch.tensor(
                self.membrane_potentials, device="cuda", dtype=torch.float32
            )
            self.thresholds = torch.tensor(
                self.thresholds, device="cuda", dtype=torch.float32
            )
            self.decay_rates = torch.tensor(
                self.decay_rates, device="cuda", dtype=torch.float32
            )
            self.refractory_periods = torch.tensor(
                self.refractory_periods, device="cuda", dtype=torch.int32
            )
            self.refractory_counters = torch.tensor(
                self.refractory_counters, device="cuda", dtype=torch.int32
            )
            self.coordinates_x = torch.tensor(
                self.coordinates_x, device="cuda", dtype=torch.uint16
            )
            self.coordinates_y = torch.tensor(
                self.coordinates_y, device="cuda", dtype=torch.uint16
            )
            self.coordinates_z = torch.tensor(
                self.coordinates_z, device="cuda", dtype=torch.uint16
            )
            self.valid_mask = torch.tensor(
                self.valid_mask, device="cuda", dtype=torch.bool
            )

            # Update device flag
            self.device = "cuda"
            return True
        except (ImportError, Exception):
            return False

    def to_cpu(self):
        """
        Transfer all neuron data back to CPU.

        Returns:
            Always returns True
        """
        if self.device == "cpu":
            return True

        try:
            # Convert tensors back to numpy arrays
            self.membrane_potentials = self.membrane_potentials.cpu().numpy()
            self.thresholds = self.thresholds.cpu().numpy()
            self.decay_rates = self.decay_rates.cpu().numpy()
            self.refractory_periods = self.refractory_periods.cpu().numpy()
            self.refractory_counters = self.refractory_counters.cpu().numpy()
            self.coordinates_x = self.coordinates_x.cpu().numpy()
            self.coordinates_y = self.coordinates_y.cpu().numpy()
            self.coordinates_z = self.coordinates_z.cpu().numpy()
            self.valid_mask = self.valid_mask.cpu().numpy()

            # Update device flag
            self.device = "cpu"
        except Exception:
            pass

        return True


@pytest.fixture
def neuron_array():
    """Create a test NeuronArray with a small capacity for testing."""
    return NeuronArray(max_neurons=100)


@pytest.fixture
def populated_neuron_array():
    """Create a test NeuronArray with some neurons already created."""
    na = NeuronArray(max_neurons=100)

    # Create test neurons
    na.create_neuron(area_id=1, position=(0, 0, 0), threshold=1.0)
    na.create_neuron(area_id=1, position=(1, 0, 0), threshold=0.8)
    na.create_neuron(area_id=2, position=(0, 0, 0), threshold=1.2)

    return na


def test_neuron_array_init():
    """Test NeuronArray initialization."""
    na = NeuronArray(max_neurons=1000)

    # Check array sizes
    assert len(na.membrane_potentials) == 1000
    assert len(na.thresholds) == 1000
    assert len(na.valid_mask) == 1000

    # Check default values
    assert np.all(na.membrane_potentials == 0.0)
    assert np.all(na.thresholds == 1.0)
    assert np.all(~na.valid_mask)


def test_create_neuron(neuron_array):
    """Test neuron creation in NeuronArray."""
    na = neuron_array

    # Create a neuron
    neuron_id = na.create_neuron(
        area_id=1,
        position=(1, 2, 3),
        threshold=0.7,
        membrane_potential=0.2,
        leak_coefficient=0.4,
    )

    # Check if the neuron was created successfully
    assert neuron_id == 0  # First neuron has ID 0
    assert na.valid_mask[0]

    # Check if properties were set correctly
    assert na.area_ids[0] == 1
    assert na.coordinates_x[0] == 1
    assert na.coordinates_y[0] == 2
    assert na.coordinates_z[0] == 3
    assert na.thresholds[0] == 0.7
    assert na.membrane_potentials[0] == 0.2
    assert na.decay_rates[0] == 0.4


def test_get_neuron_property(populated_neuron_array):
    """Test getting neuron properties."""
    na = populated_neuron_array

    # Check properties for a specific neuron
    assert na.get_neuron_property(0, "area_id") == 1
    assert na.get_neuron_property(0, "position") == (0, 0, 0)
    assert na.get_neuron_property(0, "threshold") == 1.0

    # Check for neuron with different properties
    assert na.get_neuron_property(1, "threshold") == 0.8

    # Check error for non-existent neuron
    with pytest.raises(KeyError):
        na.get_neuron_property(999, "threshold")

    # Check error for non-existent property
    with pytest.raises(KeyError):
        na.get_neuron_property(0, "nonexistent_property")


def test_set_neuron_property(populated_neuron_array):
    """Test setting neuron properties."""
    na = populated_neuron_array

    # Set properties for a neuron
    na.set_neuron_property(0, "threshold", 0.5)
    na.set_neuron_property(0, "position", (5, 6, 7))

    # Check if properties were updated
    assert na.get_neuron_property(0, "threshold") == 0.5
    assert na.get_neuron_property(0, "position") == (5, 6, 7)

    # Check error for non-existent neuron
    with pytest.raises(KeyError):
        na.set_neuron_property(999, "threshold", 0.5)

    # Check error for non-existent property
    with pytest.raises(KeyError):
        na.set_neuron_property(0, "nonexistent_property", 0.5)

    # Check error for invalid position
    with pytest.raises(ValueError):
        na.set_neuron_property(0, "position", 0.5)  # Not a tuple


def test_delete_neuron(populated_neuron_array):
    """Test neuron deletion."""
    na = populated_neuron_array

    # Check initial count
    assert np.sum(na.valid_mask) == 3

    # Delete a neuron
    result = na.delete_neuron(0)

    # Check result
    assert result

    # Check if neuron was deleted
    assert np.sum(na.valid_mask) == 2

    # Check if neuron properties are reset
    with pytest.raises(KeyError):
        na.get_neuron_property(0, "threshold")

    # Try to delete the same neuron again
    result = na.delete_neuron(0)
    assert result == False  # Should return False for non-existent neuron


def test_get_neurons_by_area(populated_neuron_array):
    """Test getting neurons by area."""
    na = populated_neuron_array

    # Get neurons in area 1
    area1_neurons = na.get_neurons_by_area(1)
    assert len(area1_neurons) == 2
    assert 0 in area1_neurons
    assert 1 in area1_neurons

    # Get neurons in area 2
    area2_neurons = na.get_neurons_by_area(2)
    assert len(area2_neurons) == 1
    assert 2 in area2_neurons

    # Get neurons in non-existent area
    area3_neurons = na.get_neurons_by_area(3)
    assert len(area3_neurons) == 0


def test_get_neuron_count(populated_neuron_array):
    """Test getting neuron count."""
    na = populated_neuron_array

    # Check initial count
    assert na.get_neuron_count() == 3

    # Delete a neuron
    na.delete_neuron(0)

    # Check updated count
    assert na.get_neuron_count() == 2


def test_neuron_wrapper_class():
    """Test the Neuron wrapper class for API compatibility."""
    # Create a Neuron object
    neuron = Neuron(
        neuron_id=42,
        area_id="test_area",
        position=(1, 2, 3),
        threshold=0.5,
        membrane_potential=0.2,
        leak_coefficient=0.4,
    )

    # Check properties
    assert neuron.id == 42
    assert neuron.area_id == "test_area"
    assert neuron.position == (1, 2, 3)
    assert neuron.threshold == 0.5
    assert neuron.membrane_potential == 0.2
    assert neuron.decay_rate == 0.4

    # Test to_dict method
    neuron_dict = neuron.to_dict()
    assert neuron_dict["id"] == 42
    assert neuron_dict["area_id"] == "test_area"
    assert neuron_dict["position"] == (1, 2, 3)

    # Test from_dict method
    recreated_neuron = Neuron.from_dict(neuron_dict)
    assert recreated_neuron.id == 42
    assert recreated_neuron.area_id == "test_area"
    assert recreated_neuron.position == (1, 2, 3)


def test_gpu_transfer():
    """Test GPU transfer methods (conditionally skips if no GPU available)."""
    na = NeuronArray(max_neurons=10)

    # Create some test neurons
    for i in range(3):
        na.create_neuron(area_id=1, position=(i, 0, 0))

    # Skip test if torch.cuda is not available
    try:
        import torch

        if not torch.cuda.is_available():
            pytest.skip("CUDA not available, skipping GPU transfer test")
    except ImportError:
        pytest.skip("PyTorch not available, skipping GPU transfer test")

    # Try transferring to GPU
    result = na.to_gpu()

    # If transfer was successful
    if result:
        # Check if device is set to cuda
        assert na.device == "cuda"

        # Check tensor types
        assert isinstance(na.membrane_potentials, torch.Tensor)

        # Transfer back to CPU
        na.to_cpu()
        assert na.device == "cpu"
        assert isinstance(na.membrane_potentials, np.ndarray)
