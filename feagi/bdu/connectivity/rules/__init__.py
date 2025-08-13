"""Rules package for BDU connectivity.

This package contains the different types of connectivity rules:
- functions: Function-based morphology rules (syn_* functions)
- patterns: Pattern-based connectivity rules
- vectors: Vector-based connectivity rules
"""

from . import functions, patterns, vectors
from .patterns import (
    check_pattern_validity,
    define_subregions,
    find_destination_coordinates,
    find_source_coordinates,
)

# Import key functions for direct access
from .vectors import match_vectors

__all__ = [
    "functions",
    "patterns",
    "vectors",
    "match_vectors",
    "check_pattern_validity",
    "define_subregions",
    "find_source_coordinates",
    "find_destination_coordinates",
]
