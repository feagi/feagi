"""FEAGI Test Mode Module.

This module provides different test modes for FEAGI neural network testing:
- test_mode_1: JSON-based predictable neuron activations
- test_mode_2: Numpy-based scalable random neuron generation

Usage:
    --test-mode-1: Uses test_mode_activations.json for predictable testing
    --test-mode-2: Uses numpy to generate large random stimulations for scalability testing
"""

from .test_mode_1 import TestMode1Handler
from .test_mode_2 import TestMode2Handler
from .test_runner import FeagiTestRunner, run_test_mode

__all__ = [
    "FeagiTestRunner",
    "run_test_mode",
    "TestMode1Handler",
    "TestMode2Handler",
]
