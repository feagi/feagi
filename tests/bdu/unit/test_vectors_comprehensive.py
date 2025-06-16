"""
Comprehensive tests for feagi.bdu.connectivity.rules.vectors module.

This module tests vector-based connectivity rules including expression evaluation,
vector operations, position validation, and candidate generation.
"""

from unittest.mock import Mock, patch

import pytest

from feagi.bdu.connectivity.rules.vectors import (
    apply_vector_offset,
    evaluate_expression,
    generate_vector_candidates,
    match_vectors,
    preprocess_expression,
    validate_vector_position,
)


class TestExpressionProcessing:
    """Test expression preprocessing and evaluation functions."""

    def test_preprocess_expression_implicit_multiplication(self):
        """Test preprocessing of implicit multiplication."""
        assert preprocess_expression("2x") == "2*x"
        assert preprocess_expression("3y") == "3*y"
        assert preprocess_expression("10z") == "10*z"
        assert preprocess_expression("2x + 3y") == "2*x + 3*y"

    def test_preprocess_expression_exponentiation(self):
        """Test preprocessing of exponentiation operator."""
        assert preprocess_expression("x^2") == "x**2"
        assert preprocess_expression("y^3") == "y**3"
        assert preprocess_expression("2^x") == "2**x"
        assert preprocess_expression("x^2 + y^3") == "x**2 + y**3"

    def test_preprocess_expression_combined(self):
        """Test preprocessing with both implicit multiplication and exponentiation."""
        assert preprocess_expression("2x^2") == "2*x**2"
        assert preprocess_expression("3y^2 + 4z") == "3*y**2 + 4*z"

    def test_preprocess_expression_no_changes(self):
        """Test preprocessing expressions that don't need changes."""
        assert preprocess_expression("x + y") == "x + y"
        assert preprocess_expression("x * y") == "x * y"
        assert preprocess_expression("x ** 2") == "x ** 2"

    def test_evaluate_expression_integer_input(self):
        """Test evaluate_expression with integer input."""
        assert evaluate_expression(5, {}) == 5
        assert evaluate_expression(0, {}) == 0
        assert evaluate_expression(-3, {}) == -3

    def test_evaluate_expression_float_input(self):
        """Test evaluate_expression with float input."""
        assert evaluate_expression(5.7, {}) == 5
        assert evaluate_expression(3.14, {}) == 3
        assert evaluate_expression(-2.8, {}) == -2

    def test_evaluate_expression_simple_variables(self):
        """Test evaluate_expression with simple variable substitution."""
        variables = {"x": 5, "y": 3, "z": 2}
        assert evaluate_expression("x", variables) == 5
        assert evaluate_expression("y", variables) == 3
        assert evaluate_expression("z", variables) == 2

    def test_evaluate_expression_arithmetic(self):
        """Test evaluate_expression with arithmetic operations."""
        variables = {"x": 5, "y": 3}
        assert evaluate_expression("x + y", variables) == 8
        assert evaluate_expression("x - y", variables) == 2
        assert evaluate_expression("x * y", variables) == 15
        assert evaluate_expression("x / y", variables) == 1  # Integer division

    def test_evaluate_expression_complex(self):
        """Test evaluate_expression with complex expressions."""
        variables = {"x": 4, "y": 2, "scalar": 2}
        assert evaluate_expression("2*x + y", variables) == 10
        assert evaluate_expression("x**2", variables) == 16
        assert evaluate_expression("x * scalar", variables) == 8

    def test_evaluate_expression_error_handling(self):
        """Test evaluate_expression error handling."""
        with patch("feagi.bdu.connectivity.rules.vectors.logger") as mock_logger:
            result = evaluate_expression("invalid_expr", {})
            assert result == 0
            mock_logger.error.assert_called_once()


class TestVectorOperations:
    """Test vector operation functions."""

    def test_apply_vector_offset_tuple_vector(self):
        """Test apply_vector_offset with tuple vector."""
        src_pos = (1, 2, 3)
        vector = (2, -1, 1)
        result = apply_vector_offset(src_pos, vector)
        assert result == (3, 1, 4)

    def test_apply_vector_offset_with_scalar(self):
        """Test apply_vector_offset with morphology scalar."""
        src_pos = (0, 0, 0)
        vector = (1, 2, 3)
        scalar = 2.0
        result = apply_vector_offset(src_pos, vector, scalar)
        assert result == (2, 4, 6)

    def test_apply_vector_offset_string_vector_with_parentheses(self):
        """Test apply_vector_offset with string vector in parentheses."""
        src_pos = (1, 1, 1)
        vector = "(2,3,4)"
        result = apply_vector_offset(src_pos, vector)
        assert result == (3, 4, 5)

    def test_apply_vector_offset_string_vector_without_parentheses(self):
        """Test apply_vector_offset with string vector without parentheses."""
        src_pos = (0, 0, 0)
        vector = "1,2,3"
        result = apply_vector_offset(src_pos, vector)
        assert result == (1, 2, 3)

    def test_apply_vector_offset_negative_values(self):
        """Test apply_vector_offset with negative values."""
        src_pos = (5, 5, 5)
        vector = (-2, -3, -1)
        result = apply_vector_offset(src_pos, vector)
        assert result == (3, 2, 4)

    def test_apply_vector_offset_zero_scalar(self):
        """Test apply_vector_offset with zero scalar."""
        src_pos = (1, 2, 3)
        vector = (5, 10, 15)
        scalar = 0.0
        result = apply_vector_offset(src_pos, vector, scalar)
        assert result == (1, 2, 3)

    def test_validate_vector_position_valid(self):
        """Test validate_vector_position with valid positions."""
        dimensions = (10, 10, 10)
        assert validate_vector_position((0, 0, 0), dimensions) is True
        assert validate_vector_position((5, 5, 5), dimensions) is True
        assert validate_vector_position((9, 9, 9), dimensions) is True

    def test_validate_vector_position_boundary(self):
        """Test validate_vector_position at boundaries."""
        dimensions = (5, 5, 5)
        assert validate_vector_position((4, 4, 4), dimensions) is True
        assert validate_vector_position((5, 4, 4), dimensions) is False
        assert validate_vector_position((4, 5, 4), dimensions) is False
        assert validate_vector_position((4, 4, 5), dimensions) is False

    def test_validate_vector_position_negative(self):
        """Test validate_vector_position with negative coordinates."""
        dimensions = (10, 10, 10)
        assert validate_vector_position((-1, 5, 5), dimensions) is False
        assert validate_vector_position((5, -1, 5), dimensions) is False
        assert validate_vector_position((5, 5, -1), dimensions) is False

    def test_validate_vector_position_zero_dimensions(self):
        """Test validate_vector_position with zero dimensions."""
        dimensions = (0, 0, 0)
        assert validate_vector_position((0, 0, 0), dimensions) is False
        assert validate_vector_position((-1, -1, -1), dimensions) is False


class TestCandidateGeneration:
    """Test candidate generation functions."""

    def test_generate_vector_candidates_single_vector(self):
        """Test generate_vector_candidates with single vector."""
        src_pos = (1, 1, 1)
        vectors = [(1, 0, 0)]
        scalar = 1.0
        dimensions = (10, 10, 10)

        candidates = generate_vector_candidates(src_pos, vectors, scalar, dimensions)
        assert candidates == {(2, 1, 1)}

    def test_generate_vector_candidates_multiple_vectors(self):
        """Test generate_vector_candidates with multiple vectors."""
        src_pos = (2, 2, 2)
        vectors = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]
        scalar = 1.0
        dimensions = (10, 10, 10)

        candidates = generate_vector_candidates(src_pos, vectors, scalar, dimensions)
        expected = {(3, 2, 2), (2, 3, 2), (2, 2, 3)}
        assert candidates == expected

    def test_generate_vector_candidates_with_scalar(self):
        """Test generate_vector_candidates with morphology scalar."""
        src_pos = (0, 0, 0)
        vectors = [(1, 1, 1)]
        scalar = 3.0
        dimensions = (10, 10, 10)

        candidates = generate_vector_candidates(src_pos, vectors, scalar, dimensions)
        assert candidates == {(3, 3, 3)}

    def test_generate_vector_candidates_out_of_bounds(self):
        """Test generate_vector_candidates with out-of-bounds results."""
        src_pos = (8, 8, 8)
        vectors = [(5, 5, 5)]
        scalar = 1.0
        dimensions = (10, 10, 10)

        with patch("feagi.bdu.connectivity.rules.vectors.logger") as mock_logger:
            candidates = generate_vector_candidates(
                src_pos, vectors, scalar, dimensions
            )
            assert candidates == set()
            mock_logger.debug.assert_called()

    def test_generate_vector_candidates_string_vectors(self):
        """Test generate_vector_candidates with string vectors."""
        src_pos = (1, 1, 1)
        vectors = ["(1,0,0)", "0,1,0"]
        scalar = 1.0
        dimensions = (10, 10, 10)

        candidates = generate_vector_candidates(src_pos, vectors, scalar, dimensions)
        expected = {(2, 1, 1), (1, 2, 1)}
        assert candidates == expected

    def test_generate_vector_candidates_mixed_valid_invalid(self):
        """Test generate_vector_candidates with mix of valid and invalid positions."""
        src_pos = (0, 0, 0)
        vectors = [(1, 1, 1), (-1, -1, -1), (2, 2, 2)]
        scalar = 1.0
        dimensions = (5, 5, 5)

        candidates = generate_vector_candidates(src_pos, vectors, scalar, dimensions)
        expected = {(1, 1, 1), (2, 2, 2)}
        assert candidates == expected

    def test_generate_vector_candidates_empty_vectors(self):
        """Test generate_vector_candidates with empty vector list."""
        src_pos = (1, 1, 1)
        vectors = []
        scalar = 1.0
        dimensions = (10, 10, 10)

        candidates = generate_vector_candidates(src_pos, vectors, scalar, dimensions)
        assert candidates == set()


class TestMatchVectors:
    """Test the main match_vectors function."""

    def setup_method(self):
        """Set up test fixtures."""
        self.mock_connectome_manager = Mock()
        self.mock_dst_area = Mock()
        self.mock_dst_area.dimensions = (10, 10, 10)
        self.mock_connectome_manager.cortical_areas = {1: self.mock_dst_area}

    def test_match_vectors_tuple_vector(self):
        """Test match_vectors with tuple vector."""
        src_voxel = (1, 2, 3)
        dst_area_id = 1
        vector = (2, 1, 0)
        scalar = 1.0
        src_subregion = ((0, 0, 0), (5, 5, 5))

        result = match_vectors(
            src_voxel,
            dst_area_id,
            vector,
            scalar,
            src_subregion,
            self.mock_connectome_manager,
        )
        assert result == {(3, 3, 3)}

    def test_match_vectors_list_vector(self):
        """Test match_vectors with list vector."""
        src_voxel = (0, 0, 0)
        dst_area_id = 1
        vector = [1, 2, 3]
        scalar = 2.0
        src_subregion = ((0, 0, 0), (5, 5, 5))

        result = match_vectors(
            src_voxel,
            dst_area_id,
            vector,
            scalar,
            src_subregion,
            self.mock_connectome_manager,
        )
        assert result == {(2, 4, 6)}

    def test_match_vectors_boundary_clamping(self):
        """Test match_vectors with boundary clamping."""
        src_voxel = (8, 8, 8)
        dst_area_id = 1
        vector = (5, 5, 5)
        scalar = 1.0
        src_subregion = ((0, 0, 0), (10, 10, 10))

        result = match_vectors(
            src_voxel,
            dst_area_id,
            vector,
            scalar,
            src_subregion,
            self.mock_connectome_manager,
        )
        # Should clamp to (9, 9, 9) since dimensions are (10, 10, 10)
        assert result == {(9, 9, 9)}

    def test_match_vectors_string_expression_simple(self):
        """Test match_vectors with simple string expression."""
        src_voxel = (2, 3, 4)
        dst_area_id = 1
        vector = "x+1, y+2, z+3"
        scalar = 1.0
        src_subregion = ((0, 0, 0), (5, 5, 5))

        result = match_vectors(
            src_voxel,
            dst_area_id,
            vector,
            scalar,
            src_subregion,
            self.mock_connectome_manager,
        )
        assert result == {(3, 5, 7)}

    def test_match_vectors_string_expression_complex(self):
        """Test match_vectors with complex string expression."""
        src_voxel = (2, 2, 2)
        dst_area_id = 1
        vector = "x*2, y+scalar, z**2"
        scalar = 3.0
        src_subregion = ((0, 0, 0), (5, 5, 5))

        result = match_vectors(
            src_voxel,
            dst_area_id,
            vector,
            scalar,
            src_subregion,
            self.mock_connectome_manager,
        )
        assert result == {(4, 5, 4)}

    def test_match_vectors_string_expression_boundary_clamping(self):
        """Test match_vectors string expression with boundary clamping."""
        src_voxel = (5, 5, 5)
        dst_area_id = 1
        vector = "x+10, y+10, z+10"
        scalar = 1.0
        src_subregion = ((0, 0, 0), (5, 5, 5))

        result = match_vectors(
            src_voxel,
            dst_area_id,
            vector,
            scalar,
            src_subregion,
            self.mock_connectome_manager,
        )
        # Should clamp to (9, 9, 9) since dimensions are (10, 10, 10)
        assert result == {(9, 9, 9)}

    def test_match_vectors_invalid_area_id(self):
        """Test match_vectors with invalid destination area ID."""
        src_voxel = (1, 1, 1)
        dst_area_id = 999  # Non-existent area
        vector = (1, 0, 0)
        scalar = 1.0
        src_subregion = ((0, 0, 0), (5, 5, 5))

        with patch("feagi.bdu.connectivity.rules.vectors.logger") as mock_logger:
            result = match_vectors(
                src_voxel,
                dst_area_id,
                vector,
                scalar,
                src_subregion,
                self.mock_connectome_manager,
            )
            assert result == set()
            mock_logger.error.assert_called_with("Destination area 999 not found")

    def test_match_vectors_invalid_string_expression_components(self):
        """Test match_vectors with invalid string expression (wrong number of components)."""
        src_voxel = (1, 1, 1)
        dst_area_id = 1
        vector = "x+1, y+2"  # Only 2 components instead of 3
        scalar = 1.0
        src_subregion = ((0, 0, 0), (5, 5, 5))

        with patch("feagi.bdu.connectivity.rules.vectors.logger") as mock_logger:
            result = match_vectors(
                src_voxel,
                dst_area_id,
                vector,
                scalar,
                src_subregion,
                self.mock_connectome_manager,
            )
            assert result == set()
            mock_logger.error.assert_called()

    def test_match_vectors_string_expression_evaluation_error(self):
        """Test match_vectors with string expression that causes evaluation error."""
        src_voxel = (1, 1, 1)
        dst_area_id = 1
        vector = "invalid_expr, y, z"
        scalar = 1.0
        src_subregion = ((0, 0, 0), (5, 5, 5))

        with patch("feagi.bdu.connectivity.rules.vectors.logger") as mock_logger:
            result = match_vectors(
                src_voxel,
                dst_area_id,
                vector,
                scalar,
                src_subregion,
                self.mock_connectome_manager,
            )
            # When evaluation fails, evaluate_expression returns 0, so result is (0, y, z)
            assert result == {(0, 1, 1)}
            mock_logger.error.assert_called()

    def test_match_vectors_negative_coordinates_clamping(self):
        """Test match_vectors with negative coordinate clamping."""
        src_voxel = (2, 2, 2)
        dst_area_id = 1
        vector = (-5, -5, -5)
        scalar = 1.0
        src_subregion = ((0, 0, 0), (5, 5, 5))

        result = match_vectors(
            src_voxel,
            dst_area_id,
            vector,
            scalar,
            src_subregion,
            self.mock_connectome_manager,
        )
        # Should clamp to (0, 0, 0) since negative values are clamped to 0
        assert result == {(0, 0, 0)}


class TestEdgeCasesAndErrorHandling:
    """Test edge cases and error handling scenarios."""

    def test_preprocess_expression_empty_string(self):
        """Test preprocessing empty string."""
        assert preprocess_expression("") == ""

    def test_preprocess_expression_whitespace(self):
        """Test preprocessing string with whitespace."""
        assert preprocess_expression("  2x + 3y  ") == "  2*x + 3*y  "

    def test_evaluate_expression_empty_variables(self):
        """Test evaluate_expression with empty variables dict."""
        assert evaluate_expression("5", {}) == 5
        assert evaluate_expression("0", {}) == 0

    def test_apply_vector_offset_string_with_spaces(self):
        """Test apply_vector_offset with string containing spaces."""
        src_pos = (0, 0, 0)
        vector = " 1 , 2 , 3 "
        result = apply_vector_offset(src_pos, vector)
        assert result == (1, 2, 3)

    def test_validate_vector_position_single_dimension_zero(self):
        """Test validate_vector_position with single zero dimension."""
        assert validate_vector_position((0, 0, 0), (1, 0, 1)) is False
        assert validate_vector_position((0, 0, 0), (0, 1, 1)) is False
        assert validate_vector_position((0, 0, 0), (1, 1, 0)) is False

    def test_generate_vector_candidates_fractional_scalar(self):
        """Test generate_vector_candidates with fractional scalar."""
        src_pos = (0, 0, 0)
        vectors = [(3, 3, 3)]
        scalar = 0.5
        dimensions = (10, 10, 10)

        candidates = generate_vector_candidates(src_pos, vectors, scalar, dimensions)
        assert candidates == {(1, 1, 1)}  # int(3 * 0.5) = 1

    def test_match_vectors_tuple_conversion(self):
        """Test match_vectors tuple to list conversion."""
        mock_connectome_manager = Mock()
        mock_dst_area = Mock()
        mock_dst_area.dimensions = (5, 5, 5)
        mock_connectome_manager.cortical_areas = {1: mock_dst_area}

        src_voxel = (1, 1, 1)
        dst_area_id = 1
        vector = (1, 1, 1)  # Tuple input
        scalar = 1.0
        src_subregion = ((0, 0, 0), (3, 3, 3))

        result = match_vectors(
            src_voxel,
            dst_area_id,
            vector,
            scalar,
            src_subregion,
            mock_connectome_manager,
        )
        assert result == {(2, 2, 2)}
