#!/usr/bin/env python3
"""
Simple geometric tests for morphology function implementations.

This test suite validates the core geometric mapping logic of morphology functions
without getting bogged down in FEAGI logistics.
"""

from typing import List

import pytest

# Import syn_projector from rules.functions
from feagi.bdu.connectivity.rules.functions import syn_projector

# Import from the synaptogenesis __init__.py file
from feagi.bdu.connectivity.synaptogenesis import (
    MorphologyFunction,
    syn_block_connection,
    syn_expander_x,
    syn_last_to_first,
    syn_lateral_pairs_x,
    syn_memory,
    syn_randomizer,
    syn_reducer_x,
)


class SimpleArea:
    """Simple area for testing."""

    def __init__(self, dimensions):
        self.dimensions = dimensions


class SimpleConnectome:
    """Simple connectome for testing geometric mappings."""

    def __init__(self):
        self.areas = {}
        self.cortical_areas = {}  # Alias for areas
        self.positions = {}

    def add_area(self, area_id, dimensions):
        area = SimpleArea(dimensions)
        self.areas[area_id] = area
        self.cortical_areas[area_id] = area  # Keep both for compatibility

    def add_neuron(self, neuron_id, position):
        self.positions[neuron_id] = position

    def get_area(self, area_id):
        return self.areas.get(area_id)

    def get_cortical_area(self, area_id):
        """Alias for get_area to match expected interface."""
        return self.get_area(area_id)

    def get_neuron_position(self, neuron_id):
        return self.positions.get(neuron_id)


class TestProjectorGeometry:
    """Test projector geometric mapping logic."""

    def test_projector_1x2x1_to_2x2x2_scaling_up(self):
        """Test your specific example: 1x2x1 area to 2x2x2 area."""
        cm = SimpleConnectome()
        cm.add_area("src", (1, 2, 1))
        cm.add_area("dst", (2, 2, 2))

        # Test neuron at (0,0,0)
        cm.add_neuron(1, (0, 0, 0))
        positions = syn_projector(
            src_area_id="src",
            dst_area_id="dst",
            src_neuron_id=1,
            src_subregion=[(0, 0, 0), (1, 2, 1)],
            connectome_manager=cm,
        )

        # Expected: X: 1->2 (neuron 0 maps to 0,1), Y: 2->2 (neuron 0 maps to 0), Z: 1->2 (neuron 0 maps to 0,1)
        expected = [(0, 0, 0), (1, 0, 0), (0, 0, 1), (1, 0, 1)]
        assert sorted(positions) == sorted(expected)

        # Test neuron at (0,1,0)
        cm.add_neuron(2, (0, 1, 0))
        positions = syn_projector(
            src_area_id="src",
            dst_area_id="dst",
            src_neuron_id=2,
            src_subregion=[(0, 0, 0), (1, 2, 1)],
            connectome_manager=cm,
        )

        # Expected: X: same, Y: neuron 1 maps to 1, Z: same
        expected = [(0, 1, 0), (1, 1, 0), (0, 1, 1), (1, 1, 1)]
        assert sorted(positions) == sorted(expected)

    def test_projector_2x1x1_to_3x3x3_user_example(self):
        """Test user's specific example: neuron at (0,0,0) should map to specific positions."""
        cm = SimpleConnectome()
        cm.add_area("src", (2, 1, 1))
        cm.add_area("dst", (3, 3, 3))

        # Test neuron at (0,0,0)
        cm.add_neuron(1, (0, 0, 0))
        positions = syn_projector(
            src_area_id="src",
            dst_area_id="dst",
            src_neuron_id=1,
            src_subregion=[(0, 0, 0), (2, 1, 1)],
            connectome_manager=cm,
        )

        # Expected: X: 2->3 (neuron 0 maps to 0,1), Y: 1->3 (neuron 0 maps to 0,1,2), Z: 1->3 (neuron 0 maps to 0,1,2)
        expected = []
        for x in [0, 1]:
            for y in [0, 1, 2]:
                for z in [0, 1, 2]:
                    expected.append((x, y, z))

        assert len(positions) == 18
        assert sorted(positions) == sorted(expected)

        # Test neuron at (1,0,0)
        cm.add_neuron(2, (1, 0, 0))
        positions = syn_projector(
            src_area_id="src",
            dst_area_id="dst",
            src_neuron_id=2,
            src_subregion=[(0, 0, 0), (2, 1, 1)],
            connectome_manager=cm,
        )

        # Expected: X: neuron 1 maps to 2, Y: same, Z: same
        expected = []
        for x in [2]:
            for y in [0, 1, 2]:
                for z in [0, 1, 2]:
                    expected.append((x, y, z))

        assert len(positions) == 9
        assert sorted(positions) == sorted(expected)

    def test_projector_4x4x4_to_2x2x2_scaling_down(self):
        """Test scaling down: 4x4x4 to 2x2x2."""
        cm = SimpleConnectome()
        cm.add_area("src", (4, 4, 4))
        cm.add_area("dst", (2, 2, 2))

        # Test neuron at (0,0,0) -> should map to (0,0,0)
        cm.add_neuron(1, (0, 0, 0))
        positions = syn_projector(
            src_area_id="src",
            dst_area_id="dst",
            src_neuron_id=1,
            src_subregion=[(0, 0, 0), (4, 4, 4)],
            connectome_manager=cm,
        )
        assert positions == [(0, 0, 0)]

        # Test neuron at (1,1,1) -> should map to (0,0,0)
        cm.add_neuron(2, (1, 1, 1))
        positions = syn_projector(
            src_area_id="src",
            dst_area_id="dst",
            src_neuron_id=2,
            src_subregion=[(0, 0, 0), (4, 4, 4)],
            connectome_manager=cm,
        )
        assert positions == [(0, 0, 0)]

        # Test neuron at (2,2,2) -> should map to (1,1,1)
        cm.add_neuron(3, (2, 2, 2))
        positions = syn_projector(
            src_area_id="src",
            dst_area_id="dst",
            src_neuron_id=3,
            src_subregion=[(0, 0, 0), (4, 4, 4)],
            connectome_manager=cm,
        )
        assert positions == [(1, 1, 1)]

    def test_projector_1x1x1_to_4x1x1_power_to_motor(self):
        """Test user's specific power to motor mapping: 1x1x1 to 4x1x1."""
        cm = SimpleConnectome()
        cm.add_area("power", (1, 1, 1))
        cm.add_area("motor", (4, 1, 1))

        # Test neuron at (0,0,0) in power area
        cm.add_neuron(1, (0, 0, 0))
        positions = syn_projector(
            src_area_id="power",
            dst_area_id="motor",
            src_neuron_id=1,
            src_subregion=[(0, 0, 0), (1, 1, 1)],
            connectome_manager=cm,
        )

        # Expected: X: 1->4 (neuron 0 maps to 0,1,2,3), Y: same, Z: same
        expected = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]
        assert len(positions) == 4
        assert sorted(positions) == sorted(expected)


class TestExpanderGeometry:
    """Test expander geometric logic."""

    def test_expander_x_default_factor(self):
        """Test expander with default factor 2.0."""
        cm = SimpleConnectome()
        cm.add_area("src", (2, 1, 1))
        cm.add_area("dst", (4, 1, 1))

        # Test (0,0,0) -> (0,0,0)
        cm.add_neuron(1, (0, 0, 0))
        positions = syn_expander_x(
            "src", "dst", 1, [(0, 0, 0), (2, 1, 1)], connectome_manager=cm
        )
        assert list(positions) == [(0, 0, 0)]

        # Test (1,0,0) -> (2,0,0)
        cm.add_neuron(2, (1, 0, 0))
        positions = syn_expander_x(
            "src", "dst", 2, [(0, 0, 0), (2, 1, 1)], connectome_manager=cm
        )
        assert list(positions) == [(2, 0, 0)]

    def test_expander_x_custom_factor(self):
        """Test expander with custom factor."""
        cm = SimpleConnectome()
        cm.add_area("src", (2, 1, 1))
        cm.add_area("dst", (6, 1, 1))

        # Test (1,0,0) -> (3,0,0)
        cm.add_neuron(1, (1, 0, 0))
        positions = syn_expander_x(
            "src", "dst", 1, [(0, 0, 0), (2, 1, 1)], connectome_manager=cm
        )
        assert list(positions) == [(3, 0, 0)]


class TestReducerGeometry:
    """Test reducer geometric logic."""

    def test_reducer_x_default_factor(self):
        """Test reducer with default factor 0.5."""
        cm = SimpleConnectome()
        cm.add_area("src", (4, 1, 1))
        cm.add_area("dst", (2, 1, 1))

        # Test (1,0,0) -> (1,0,0) (binary "1" padded to "01", bit 1 is set)
        cm.add_neuron(1, (1, 0, 0))
        positions = syn_reducer_x(
            "src", "dst", 1, [(0, 0, 0), (4, 1, 1)], connectome_manager=cm
        )
        assert positions == [(1, 0, 0)]

        # Test (2,0,0) -> (0,0,0) (binary "10", bit 0 is set)
        cm.add_neuron(2, (2, 0, 0))
        positions = syn_reducer_x(
            "src", "dst", 2, [(0, 0, 0), (4, 1, 1)], connectome_manager=cm
        )
        assert positions == [(0, 0, 0)]

    def test_reducer_x_custom_factor(self):
        """Test reducer with custom factor."""
        cm = SimpleConnectome()
        cm.add_area("src", (4, 1, 1))
        cm.add_area("dst", (1, 1, 1))

        # Test (3,0,0) -> (0,0,0)
        cm.add_neuron(1, (3, 0, 0))
        positions = syn_reducer_x(
            "src", "dst", 1, [(0, 0, 0), (4, 1, 1)], connectome_manager=cm
        )
        assert positions == [(0, 0, 0)]


class TestRandomizerGeometry:
    """Test randomizer bounds validation."""

    def test_randomizer_bounds(self):
        """Test randomizer stays within bounds."""
        cm = SimpleConnectome()
        cm.add_area("dst", (3, 3, 3))

        # Test multiple times
        for _ in range(10):
            position = syn_randomizer(dst_area_id="dst", connectome_manager=cm)
            assert isinstance(position, tuple)
            assert len(position) == 3
            x, y, z = position
            assert 0 <= x < 3
            assert 0 <= y < 3
            assert 0 <= z < 3


class TestLateralPairsGeometry:
    """Test lateral pairs geometric logic."""

    def test_lateral_pairs_x_logic(self):
        """Test lateral pairs even/odd logic."""
        cm = SimpleConnectome()
        cm.add_area("area", (5, 1, 1))

        # Even position (0,0,0) -> (1,0,0)
        cm.add_neuron(1, (0, 0, 0))
        position = syn_lateral_pairs_x(
            1, "area", [(0, 0, 0), (5, 1, 1)], connectome_manager=cm
        )
        assert position == (1, 0, 0)

        # Odd position (1,0,0) -> (0,0,0)
        cm.add_neuron(2, (1, 0, 0))
        position = syn_lateral_pairs_x(
            2, "area", [(0, 0, 0), (5, 1, 1)], connectome_manager=cm
        )
        assert position == (0, 0, 0)

        # Even position (2,0,0) -> (3,0,0)
        cm.add_neuron(3, (2, 0, 0))
        position = syn_lateral_pairs_x(
            3, "area", [(0, 0, 0), (5, 1, 1)], connectome_manager=cm
        )
        assert position == (3, 0, 0)

        # Odd position (3,0,0) -> (2,0,0)
        cm.add_neuron(4, (3, 0, 0))
        position = syn_lateral_pairs_x(
            4, "area", [(0, 0, 0), (5, 1, 1)], connectome_manager=cm
        )
        assert position == (2, 0, 0)

        # Boundary: even position (4,0,0) -> None (out of bounds)
        cm.add_neuron(5, (4, 0, 0))
        position = syn_lateral_pairs_x(
            5, "area", [(0, 0, 0), (5, 1, 1)], connectome_manager=cm
        )
        assert position is None


class TestBlockConnectionGeometry:
    """Test block connection geometric logic."""

    def test_block_connection_default_scaling(self):
        """Test block connection with default scaling factor 10."""
        cm = SimpleConnectome()
        cm.add_area("src", (20, 1, 1))
        cm.add_area("dst", (2, 1, 1))

        # Test (5,0,0) -> (0,0,0)
        cm.add_neuron(1, (5, 0, 0))
        position = syn_block_connection(
            "src", "dst", 1, [(0, 0, 0), (20, 1, 1)], connectome_manager=cm
        )
        assert position == (0, 0, 0)

        # Test (15,0,0) -> (1,0,0)
        cm.add_neuron(2, (15, 0, 0))
        position = syn_block_connection(
            "src", "dst", 2, [(0, 0, 0), (20, 1, 1)], connectome_manager=cm
        )
        assert position == (1, 0, 0)

    def test_block_connection_custom_scaling(self):
        """Test block connection with custom scaling factor."""
        cm = SimpleConnectome()
        cm.add_area("src", (8, 1, 1))
        cm.add_area("dst", (4, 1, 1))

        # Test (6,0,0) with scaling factor 2 -> (3,0,0)
        cm.add_neuron(1, (6, 0, 0))
        position = syn_block_connection(
            "src",
            "dst",
            1,
            [(0, 0, 0), (8, 1, 1)],
            connectome_manager=cm,
            scaling_factor=2,
        )
        assert position == (3, 0, 0)


class TestMemoryGeometry:
    """Test memory function logic."""

    def test_memory_registration(self):
        """Test memory registration logic."""
        memory_register = {}

        result = syn_memory("src1", "dst1", memory_register)
        assert result is None
        assert "dst1" in memory_register
        assert "src1" in memory_register["dst1"]

        # Add another source to same destination
        syn_memory("src2", "dst1", memory_register)
        assert len(memory_register["dst1"]) == 2
        assert "src2" in memory_register["dst1"]


class TestLastToFirstGeometry:
    """Test last_to_first function logic."""

    def test_last_to_first_always_origin(self):
        """Test last_to_first always returns origin."""
        cm = SimpleConnectome()
        cm.add_area("area1", (5, 5, 5))
        cm.add_area("area2", (10, 10, 10))

        # Should always return [(0, 0, 0)] regardless of area
        assert syn_last_to_first("area1", connectome_manager=cm) == [(0, 0, 0)]
        assert syn_last_to_first("area2", connectome_manager=cm) == [(0, 0, 0)]


class TestMorphologyFunctionEnum:
    """Test morphology function enum completeness."""

    def test_enum_coverage(self):
        """Test all expected morphology functions are defined."""
        expected = [
            "expander_x",
            "reducer_x",
            "randomizer",
            "lateral_pairs_x",
            "block_connection",
            "projector",
            "projector_xy",
            "projector_xz",
            "projector_yz",
            "project_from_end_x",
            "project_from_end_y",
            "project_from_end_z",
            "memory",
            "last_to_first",
        ]

        for func_name in expected:
            assert hasattr(MorphologyFunction, func_name.upper())
            assert getattr(MorphologyFunction, func_name.upper()).value == func_name


@pytest.fixture(scope="module")
def synaptogenesis_file(request):
    return request.config.getoption("synaptogenesis_file", None)


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
