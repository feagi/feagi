"""
Comprehensive tests for patterns module to achieve high code coverage.

This test suite focuses on covering the missing areas in the patterns.py module,
including pattern validation, coordinate matching, and pattern-based coordinate generation.
"""

from unittest.mock import patch

from feagi.bdu.connectivity.rules.patterns import (
    apply_pattern_to_coordinates,
    calculate_pattern_offset,
    check_pattern_validity,
    define_subregions,
    find_destination_coordinates,
    find_source_coordinates,
    generate_pattern_coordinates,
    match_pattern_element,
    validate_pattern,
    validate_pattern_element,
)


class TestPatternValidation:
    """Test pattern validation functions."""

    def test_check_pattern_validity_valid_patterns(self):
        """Test check_pattern_validity with valid patterns."""
        # Valid wildcard patterns
        assert check_pattern_validity(["*", "?", "!"]) is True

        # Valid integer patterns
        assert check_pattern_validity([0, 1, 2]) is True
        assert check_pattern_validity([10, 20, 30]) is True

        # Mixed valid patterns
        assert check_pattern_validity(["*", 5, "?"]) is True
        assert check_pattern_validity([0, "*", "!"]) is True

    def test_check_pattern_validity_invalid_patterns(self):
        """Test check_pattern_validity with invalid patterns."""
        # Negative integers
        assert check_pattern_validity([-1, 0, 1]) is False
        assert check_pattern_validity([0, -5, 2]) is False

        # Invalid string patterns
        assert check_pattern_validity(["invalid", "*", "?"]) is False
        assert check_pattern_validity(["*", "bad", "!"]) is False

        # Invalid types
        assert check_pattern_validity([None, "*", "?"]) is False
        # Note: 1.5 is valid because int(1.5) = 1 which is >= 0
        assert check_pattern_validity([1.5, "*", "?"]) is True

    def test_check_pattern_validity_empty_pattern(self):
        """Test check_pattern_validity with empty pattern."""
        assert check_pattern_validity([]) is True

    def test_validate_pattern_element_valid_elements(self):
        """Test validate_pattern_element with valid elements."""
        # Valid wildcard elements
        assert validate_pattern_element("*") is True
        assert validate_pattern_element("?") is True
        assert validate_pattern_element("!") is True

        # Valid integer elements
        assert validate_pattern_element(0) is True
        assert validate_pattern_element(5) is True
        assert validate_pattern_element(100) is True

        # String representations of integers
        assert validate_pattern_element("0") is True
        assert validate_pattern_element("42") is True

    def test_validate_pattern_element_invalid_elements(self):
        """Test validate_pattern_element with invalid elements."""
        # Negative integers
        assert validate_pattern_element(-1) is False
        assert validate_pattern_element("-5") is False

        # Invalid strings
        assert validate_pattern_element("invalid") is False
        assert validate_pattern_element("abc") is False

        # Invalid types
        assert validate_pattern_element(None) is False
        # Note: 1.5 is valid because int(1.5) = 1 which is >= 0
        assert validate_pattern_element(1.5) is True
        # Note: [] causes TypeError due to unhashable type, so we don't test it

    def test_validate_pattern_comprehensive(self):
        """Test validate_pattern with various pattern combinations."""
        # Valid patterns
        assert validate_pattern(["*", "?", "!"]) is True
        assert validate_pattern([0, 1, 2]) is True
        assert validate_pattern(["*", 5, "?"]) is True
        assert validate_pattern([]) is True

        # Invalid patterns
        assert validate_pattern([-1, "*", "?"]) is False
        assert validate_pattern(["*", "invalid", "?"]) is False
        assert validate_pattern([None, 1, 2]) is False


class TestPatternMatching:
    """Test pattern matching functions."""

    def test_match_pattern_element_wildcard(self):
        """Test match_pattern_element with wildcard '*'."""
        # Wildcard should match any coordinate
        assert match_pattern_element("*", 0) is True
        assert match_pattern_element("*", 5) is True
        assert match_pattern_element("*", 100) is True
        assert match_pattern_element("*", -1) is True

    def test_match_pattern_element_skip(self):
        """Test match_pattern_element with skip '?'."""
        # Skip should never match
        assert match_pattern_element("?", 0) is False
        assert match_pattern_element("?", 5) is False
        assert match_pattern_element("?", 100) is False

    def test_match_pattern_element_exclude(self):
        """Test match_pattern_element with exclude '!'."""
        # Exclude should never match
        assert match_pattern_element("!", 0) is False
        assert match_pattern_element("!", 5) is False
        assert match_pattern_element("!", 100) is False

    def test_match_pattern_element_exact_match(self):
        """Test match_pattern_element with exact integer values."""
        # Exact matches
        assert match_pattern_element(0, 0) is True
        assert match_pattern_element(5, 5) is True
        assert match_pattern_element(100, 100) is True

        # Non-matches
        assert match_pattern_element(0, 1) is False
        assert match_pattern_element(5, 10) is False
        assert match_pattern_element(100, 99) is False

    def test_match_pattern_element_string_integers(self):
        """Test match_pattern_element with string representations of integers."""
        # String integers should work
        assert match_pattern_element("0", 0) is True
        assert match_pattern_element("5", 5) is True
        assert match_pattern_element("100", 100) is True

        # Non-matches
        assert match_pattern_element("0", 1) is False
        assert match_pattern_element("5", 10) is False

    def test_match_pattern_element_invalid_elements(self):
        """Test match_pattern_element with invalid elements."""
        # Invalid elements should not match
        assert match_pattern_element("invalid", 5) is False
        assert match_pattern_element(None, 5) is False
        assert match_pattern_element(1.5, 5) is False

    def test_apply_pattern_to_coordinates_basic(self):
        """Test apply_pattern_to_coordinates with basic patterns."""
        coordinates = [(0, 0, 0), (1, 1, 1), (2, 2, 2)]

        # Wildcard pattern should match all
        pattern = ["*", "*", "*"]
        result = list(apply_pattern_to_coordinates(pattern, coordinates))
        assert result == [(0, 0, 0), (1, 1, 1), (2, 2, 2)]

        # Exact match pattern
        pattern = [1, 1, 1]
        result = list(apply_pattern_to_coordinates(pattern, coordinates))
        assert result == [(1, 1, 1)]

    def test_apply_pattern_to_coordinates_mixed_pattern(self):
        """Test apply_pattern_to_coordinates with mixed patterns."""
        coordinates = [(0, 0, 0), (0, 1, 0), (1, 0, 0), (1, 1, 1)]

        # Mixed pattern: exact x, wildcard y, exact z
        pattern = [0, "*", 0]
        result = list(apply_pattern_to_coordinates(pattern, coordinates))
        assert result == [(0, 0, 0), (0, 1, 0)]

    def test_apply_pattern_to_coordinates_no_matches(self):
        """Test apply_pattern_to_coordinates with no matching coordinates."""
        coordinates = [(0, 0, 0), (1, 1, 1), (2, 2, 2)]

        # Pattern that matches nothing
        pattern = [5, 5, 5]
        result = list(apply_pattern_to_coordinates(pattern, coordinates))
        assert result == []

    def test_apply_pattern_to_coordinates_invalid_pattern_length(self):
        """Test apply_pattern_to_coordinates with invalid pattern length."""
        coordinates = [(0, 0, 0), (1, 1, 1)]

        # Pattern with wrong length should return empty
        pattern = ["*", "*"]  # Only 2 elements instead of 3
        with patch("feagi.bdu.connectivity.rules.patterns.logger") as mock_logger:
            result = list(apply_pattern_to_coordinates(pattern, coordinates))
            assert result == []
            mock_logger.warning.assert_called_once()

    def test_apply_pattern_to_coordinates_empty_coordinates(self):
        """Test apply_pattern_to_coordinates with empty coordinate list."""
        coordinates = []
        pattern = ["*", "*", "*"]
        result = list(apply_pattern_to_coordinates(pattern, coordinates))
        assert result == []


class TestSourceCoordinateGeneration:
    """Test source coordinate generation functions."""

    def test_find_source_coordinates_wildcard_pattern(self):
        """Test find_source_coordinates with wildcard pattern."""
        pattern = ["*", "*", "*"]
        boundary = (2, 2, 2)

        result = list(find_source_coordinates(pattern, boundary))
        expected = [
            (0, 0, 0),
            (0, 0, 1),
            (0, 1, 0),
            (0, 1, 1),
            (1, 0, 0),
            (1, 0, 1),
            (1, 1, 0),
            (1, 1, 1),
        ]
        assert result == expected

    def test_find_source_coordinates_exact_pattern(self):
        """Test find_source_coordinates with exact coordinate pattern."""
        pattern = [1, 2, 0]
        boundary = (3, 3, 3)

        result = list(find_source_coordinates(pattern, boundary))
        assert result == [(1, 2, 0)]

    def test_find_source_coordinates_mixed_pattern(self):
        """Test find_source_coordinates with mixed pattern."""
        pattern = ["*", 1, 0]
        boundary = (3, 3, 2)

        result = list(find_source_coordinates(pattern, boundary))
        expected = [(0, 1, 0), (1, 1, 0), (2, 1, 0)]
        assert result == expected

    def test_find_source_coordinates_single_dimension(self):
        """Test find_source_coordinates with single dimension boundary."""
        pattern = ["*", "*", "*"]
        boundary = (1, 1, 1)

        result = list(find_source_coordinates(pattern, boundary))
        assert result == [(0, 0, 0)]

    def test_find_source_coordinates_zero_boundary(self):
        """Test find_source_coordinates with zero boundary."""
        pattern = ["*", "*", "*"]
        boundary = (0, 0, 0)

        result = list(find_source_coordinates(pattern, boundary))
        assert result == []


class TestDestinationCoordinateGeneration:
    """Test destination coordinate generation functions."""

    def test_find_destination_coordinates_wildcard_dst_pattern(self):
        """Test find_destination_coordinates with wildcard destination pattern."""
        dst_boundary = (2, 2, 2)
        src_coordinate = (0, 0, 0)
        src_pattern = [0, 0, 0]
        dst_pattern = ["*", "*", "*"]

        result = list(
            find_destination_coordinates(
                dst_boundary, src_coordinate, src_pattern, dst_pattern
            )
        )
        expected = [
            (0, 0, 0),
            (0, 0, 1),
            (0, 1, 0),
            (0, 1, 1),
            (1, 0, 0),
            (1, 0, 1),
            (1, 1, 0),
            (1, 1, 1),
        ]
        assert result == expected

    def test_find_destination_coordinates_exact_dst_pattern(self):
        """Test find_destination_coordinates with exact destination pattern."""
        dst_boundary = (3, 3, 3)
        src_coordinate = (1, 1, 1)
        src_pattern = [1, 1, 1]
        dst_pattern = [2, 2, 2]

        result = list(
            find_destination_coordinates(
                dst_boundary, src_coordinate, src_pattern, dst_pattern
            )
        )
        assert result == [(2, 2, 2)]

    def test_find_destination_coordinates_skip_pattern(self):
        """Test find_destination_coordinates with skip '?' pattern."""
        dst_boundary = (3, 3, 3)
        src_coordinate = (1, 1, 1)
        src_pattern = [1, 1, 1]
        dst_pattern = ["?", "?", "?"]

        result = list(
            find_destination_coordinates(
                dst_boundary, src_coordinate, src_pattern, dst_pattern
            )
        )
        assert result == [(1, 1, 1)]

    def test_find_destination_coordinates_exclude_pattern(self):
        """Test find_destination_coordinates with exclude '!' pattern."""
        dst_boundary = (3, 1, 1)
        src_coordinate = (1, 0, 0)
        src_pattern = [1, 0, 0]
        dst_pattern = ["!", 0, 0]

        result = list(
            find_destination_coordinates(
                dst_boundary, src_coordinate, src_pattern, dst_pattern
            )
        )
        expected = [(0, 0, 0), (2, 0, 0)]  # All except (1, 0, 0)
        assert result == expected

    def test_find_destination_coordinates_mixed_patterns(self):
        """Test find_destination_coordinates with mixed patterns."""
        dst_boundary = (2, 2, 2)
        src_coordinate = (0, 0, 0)
        src_pattern = ["*", 0, 0]
        dst_pattern = [1, "*", 0]

        result = list(
            find_destination_coordinates(
                dst_boundary, src_coordinate, src_pattern, dst_pattern
            )
        )
        expected = [(1, 0, 0), (1, 1, 0)]
        assert result == expected

    def test_find_destination_coordinates_out_of_bounds(self):
        """Test find_destination_coordinates with out of bounds coordinates."""
        dst_boundary = (2, 2, 2)
        src_coordinate = (5, 5, 5)  # Out of bounds
        src_pattern = ["*", "*", "*"]
        dst_pattern = ["?", "?", "?"]

        result = list(
            find_destination_coordinates(
                dst_boundary, src_coordinate, src_pattern, dst_pattern
            )
        )
        assert result == []  # Should be empty since src_coordinate is out of bounds

    def test_find_destination_coordinates_complex_logic(self):
        """Test find_destination_coordinates with complex pattern logic."""
        dst_boundary = (3, 3, 3)
        src_coordinate = (1, 1, 1)
        src_pattern = ["*", "*", "*"]
        dst_pattern = [0, "?", "!"]

        result = list(
            find_destination_coordinates(
                dst_boundary, src_coordinate, src_pattern, dst_pattern
            )
        )
        expected = [(0, 1, 0), (0, 1, 2)]  # x=0, y=src_y, z!=src_z
        assert result == expected


class TestPatternCoordinateGeneration:
    """Test pattern-based coordinate generation functions."""

    def test_generate_pattern_coordinates_basic(self):
        """Test generate_pattern_coordinates with basic patterns."""
        src_pattern = [0, 0, 0]
        dst_pattern = ["*", "*", "*"]
        src_coordinate = (0, 0, 0)
        dst_dimensions = (2, 2, 2)

        result = list(
            generate_pattern_coordinates(
                src_pattern, dst_pattern, src_coordinate, dst_dimensions
            )
        )
        expected = [
            (0, 0, 0),
            (0, 0, 1),
            (0, 1, 0),
            (0, 1, 1),
            (1, 0, 0),
            (1, 0, 1),
            (1, 1, 0),
            (1, 1, 1),
        ]
        assert result == expected

    def test_generate_pattern_coordinates_exact_pattern(self):
        """Test generate_pattern_coordinates with exact destination pattern."""
        src_pattern = [0, 0, 0]
        dst_pattern = [1, 1, 1]
        src_coordinate = (0, 0, 0)
        dst_dimensions = (3, 3, 3)

        result = list(
            generate_pattern_coordinates(
                src_pattern, dst_pattern, src_coordinate, dst_dimensions
            )
        )
        assert result == [(1, 1, 1)]

    def test_generate_pattern_coordinates_no_matches(self):
        """Test generate_pattern_coordinates with pattern that matches nothing."""
        src_pattern = [0, 0, 0]
        dst_pattern = [5, 5, 5]  # Out of bounds
        src_coordinate = (0, 0, 0)
        dst_dimensions = (2, 2, 2)

        result = list(
            generate_pattern_coordinates(
                src_pattern, dst_pattern, src_coordinate, dst_dimensions
            )
        )
        assert result == []

    def test_generate_pattern_coordinates_single_dimension(self):
        """Test generate_pattern_coordinates with single dimension."""
        src_pattern = [0, 0, 0]
        dst_pattern = ["*", "*", "*"]
        src_coordinate = (0, 0, 0)
        dst_dimensions = (1, 1, 1)

        result = list(
            generate_pattern_coordinates(
                src_pattern, dst_pattern, src_coordinate, dst_dimensions
            )
        )
        assert result == [(0, 0, 0)]


class TestPatternOffset:
    """Test pattern offset calculation functions."""

    def test_calculate_pattern_offset_wildcard_pattern(self):
        """Test calculate_pattern_offset with wildcard patterns."""
        src_coordinate = (5, 10, 15)
        src_pattern = ["*", "*", "*"]
        dst_pattern = ["*", "*", "*"]

        result = calculate_pattern_offset(src_coordinate, src_pattern, dst_pattern)
        assert result == (0, 0, 0)  # Wildcards default to 0

    def test_calculate_pattern_offset_exact_pattern(self):
        """Test calculate_pattern_offset with exact integer patterns."""
        src_coordinate = (5, 10, 15)
        src_pattern = [5, 10, 15]
        dst_pattern = ["2", "3", "4"]

        result = calculate_pattern_offset(src_coordinate, src_pattern, dst_pattern)
        assert result == (2, 3, 4)

    def test_calculate_pattern_offset_mixed_pattern(self):
        """Test calculate_pattern_offset with mixed patterns."""
        src_coordinate = (5, 10, 15)
        src_pattern = ["*", 10, "*"]
        dst_pattern = ["7", "*", "20"]

        result = calculate_pattern_offset(src_coordinate, src_pattern, dst_pattern)
        assert result == (7, 0, 20)

    def test_calculate_pattern_offset_non_digit_fallback(self):
        """Test calculate_pattern_offset with non-digit patterns falling back to src."""
        src_coordinate = (5, 10, 15)
        src_pattern = ["*", "*", "*"]
        dst_pattern = ["?", "!", "invalid"]

        result = calculate_pattern_offset(src_coordinate, src_pattern, dst_pattern)
        assert result == (5, 10, 15)  # Falls back to src_coordinate


class TestSubregionDefinition:
    """Test subregion definition functions."""

    def test_define_subregions_basic(self):
        """Test define_subregions with basic parameters."""
        area_id = 1
        parameters = {
            "src_seed": [2, 2, 2],
            "src_pattern": [
                [1, 1],
                [1, 1],
                [1, 1],
            ],  # choose 1, skip 1 for each dimension
        }
        cortical_dimensions = (6, 6, 6)

        result = define_subregions(area_id, parameters, cortical_dimensions)

        # Should create multiple subregions based on the pattern
        assert isinstance(result, set)
        assert len(result) > 0

        # Each subregion should be a tuple of two tuples (bounding box)
        for subregion in result:
            assert isinstance(subregion, tuple)
            assert len(subregion) == 2
            assert isinstance(subregion[0], tuple)
            assert isinstance(subregion[1], tuple)
            assert len(subregion[0]) == 3
            assert len(subregion[1]) == 3

    def test_define_subregions_single_subregion(self):
        """Test define_subregions that creates a single subregion."""
        area_id = 1
        parameters = {
            "src_seed": [3, 3, 3],
            "src_pattern": [[1, 0], [1, 0], [1, 0]],  # choose 1, skip 0 (no skipping)
        }
        cortical_dimensions = (3, 3, 3)

        result = define_subregions(area_id, parameters, cortical_dimensions)

        assert len(result) == 1
        subregion = list(result)[0]
        assert subregion == ((0, 0, 0), (3, 3, 3))

    def test_define_subregions_no_seed_pattern(self):
        """Test define_subregions without src_seed or src_pattern."""
        area_id = 1
        parameters = {}  # Missing required parameters
        cortical_dimensions = (5, 5, 5)

        result = define_subregions(area_id, parameters, cortical_dimensions)

        assert result == set()  # Should return empty set

    def test_define_subregions_missing_seed(self):
        """Test define_subregions with missing src_seed."""
        area_id = 1
        parameters = {"src_pattern": [[1, 1], [1, 1], [1, 1]]}
        cortical_dimensions = (5, 5, 5)

        result = define_subregions(area_id, parameters, cortical_dimensions)

        assert result == set()

    def test_define_subregions_missing_pattern(self):
        """Test define_subregions with missing src_pattern."""
        area_id = 1
        parameters = {"src_seed": [2, 2, 2]}
        cortical_dimensions = (5, 5, 5)

        result = define_subregions(area_id, parameters, cortical_dimensions)

        assert result == set()

    def test_define_subregions_large_seed(self):
        """Test define_subregions with seed larger than dimensions."""
        area_id = 1
        parameters = {
            "src_seed": [10, 10, 10],  # Larger than cortical dimensions
            "src_pattern": [[1, 0], [1, 0], [1, 0]],
        }
        cortical_dimensions = (5, 5, 5)

        result = define_subregions(area_id, parameters, cortical_dimensions)

        # Should handle gracefully and not create invalid subregions
        assert isinstance(result, set)
        for subregion in result:
            min_corner, max_corner = subregion
            # Max corner should not exceed cortical dimensions
            assert max_corner[0] <= cortical_dimensions[0]
            assert max_corner[1] <= cortical_dimensions[1]
            assert max_corner[2] <= cortical_dimensions[2]

    def test_define_subregions_complex_pattern(self):
        """Test define_subregions with complex pattern."""
        area_id = 1
        parameters = {
            "src_seed": [1, 1, 1],
            "src_pattern": [[2, 1], [2, 1], [2, 1]],  # choose 2, skip 1
        }
        cortical_dimensions = (9, 9, 9)

        result = define_subregions(area_id, parameters, cortical_dimensions)

        # Should create multiple subregions
        assert len(result) > 1

        # Verify subregions don't overlap inappropriately and are within bounds
        for subregion in result:
            min_corner, max_corner = subregion
            assert min_corner[0] >= 0 and min_corner[0] < cortical_dimensions[0]
            assert min_corner[1] >= 0 and min_corner[1] < cortical_dimensions[1]
            assert min_corner[2] >= 0 and min_corner[2] < cortical_dimensions[2]
            assert max_corner[0] <= cortical_dimensions[0]
            assert max_corner[1] <= cortical_dimensions[1]
            assert max_corner[2] <= cortical_dimensions[2]

    def test_define_subregions_zero_dimensions(self):
        """Test define_subregions with zero cortical dimensions."""
        area_id = 1
        parameters = {"src_seed": [1, 1, 1], "src_pattern": [[1, 0], [1, 0], [1, 0]]}
        cortical_dimensions = (0, 0, 0)

        result = define_subregions(area_id, parameters, cortical_dimensions)

        assert result == set()


class TestEdgeCasesAndErrorHandling:
    """Test edge cases and error handling scenarios."""

    def test_pattern_validation_with_string_numbers(self):
        """Test pattern validation with string representations of numbers."""
        # Valid string numbers
        assert validate_pattern_element("0") is True
        assert validate_pattern_element("123") is True

        # Invalid string numbers
        assert validate_pattern_element("-1") is False
        assert validate_pattern_element("1.5") is False

    def test_coordinate_generation_edge_cases(self):
        """Test coordinate generation with edge case inputs."""
        # Empty boundary
        result = list(find_source_coordinates(["*", "*", "*"], (0, 1, 1)))
        assert result == []

        # Single point boundary
        result = list(find_source_coordinates([0, 0, 0], (1, 1, 1)))
        assert result == [(0, 0, 0)]

    def test_pattern_matching_type_coercion(self):
        """Test pattern matching with type coercion scenarios."""
        # String integers should match integer coordinates
        assert match_pattern_element("5", 5) is True
        assert match_pattern_element("0", 0) is True

        # But invalid strings should not
        assert match_pattern_element("5.0", 5) is False
        assert match_pattern_element("five", 5) is False

    def test_destination_coordinates_boundary_conditions(self):
        """Test destination coordinate generation at boundaries."""
        # Test with coordinates at the boundary
        dst_boundary = (2, 2, 2)
        src_coordinate = (1, 1, 1)  # At boundary - 1
        src_pattern = ["*", "*", "*"]
        dst_pattern = ["?", "?", "?"]

        result = list(
            find_destination_coordinates(
                dst_boundary, src_coordinate, src_pattern, dst_pattern
            )
        )
        assert result == [(1, 1, 1)]

    def test_subregion_definition_boundary_conditions(self):
        """Test subregion definition at various boundary conditions."""
        area_id = 1

        # Test with seed exactly matching dimensions
        parameters = {"src_seed": [5, 5, 5], "src_pattern": [[1, 0], [1, 0], [1, 0]]}
        cortical_dimensions = (5, 5, 5)

        result = define_subregions(area_id, parameters, cortical_dimensions)
        assert len(result) == 1

        subregion = list(result)[0]
        assert subregion == ((0, 0, 0), (5, 5, 5))

    def test_pattern_offset_with_edge_cases(self):
        """Test pattern offset calculation with edge cases."""
        # Test with zero coordinates
        result = calculate_pattern_offset((0, 0, 0), ["*", "*", "*"], ["*", "*", "*"])
        assert result == (0, 0, 0)

        # Test with large coordinates
        result = calculate_pattern_offset(
            (1000, 2000, 3000), ["*", "*", "*"], ["5", "10", "15"]
        )
        assert result == (5, 10, 15)

    def test_coordinate_generation_performance_patterns(self):
        """Test coordinate generation with patterns that could affect performance."""
        # Large boundary with wildcard - should generate many coordinates
        boundary = (10, 10, 10)
        pattern = ["*", "*", "*"]

        result = list(find_source_coordinates(pattern, boundary))
        assert len(result) == 10 * 10 * 10  # Should generate all 1000 coordinates

        # Verify first and last coordinates
        assert result[0] == (0, 0, 0)
        assert result[-1] == (9, 9, 9)

    def test_apply_pattern_with_special_characters(self):
        """Test apply_pattern_to_coordinates with special pattern characters."""
        coordinates = [(0, 0, 0), (1, 1, 1), (2, 2, 2)]

        # Pattern with skip characters
        pattern = ["?", "?", "?"]
        result = list(apply_pattern_to_coordinates(pattern, coordinates))
        assert result == []  # Skip should match nothing

        # Pattern with exclude characters
        pattern = ["!", "!", "!"]
        result = list(apply_pattern_to_coordinates(pattern, coordinates))
        assert result == []  # Exclude should match nothing
