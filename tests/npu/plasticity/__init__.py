"""
FEAGI Memory System Test Package

Comprehensive test suite for FEAGI's Memory System Version 3.0.

This package contains tests for:
- Fire Ledger integration with per-area temporal depth
- RoaringBitmap pattern detection with SHA-256 hashing  
- Memory neuron lifecycle management
- PlasticityService thread operations
- Global unique ID allocation
- End-to-end memory formation workflows

Test Modules:
- test_memory_system: Main integration tests
- test_pattern_detector: Pattern detection system tests
- test_plasticity_service: PlasticityService thread tests
- run_memory_tests: Comprehensive test runner

Usage:
    # Run all tests
    python tests/npu/plasticity/run_memory_tests.py
    
    # Run with pytest
    pytest tests/npu/plasticity/
    
    # Run specific test
    pytest tests/npu/plasticity/test_memory_system.py::TestMemorySystemIntegration::test_end_to_end_memory_formation

Version: 3.0
"""

__version__ = "3.0"
__author__ = "FEAGI Development Team"

# Test suite metadata
TEST_SUITE_INFO = {
    "name": "FEAGI Memory System Test Suite",
    "version": __version__,
    "components_tested": [
        "Fire Ledger Integration",
        "Pattern Detection System", 
        "Memory Neuron Array",
        "PlasticityService",
        "Neuron ID Manager",
        "End-to-End Workflows"
    ],
    "test_categories": [
        "unit",
        "integration", 
        "performance",
        "memory",
        "pattern",
        "plasticity"
    ]
}

def get_test_info():
    """Get information about the test suite."""
    return TEST_SUITE_INFO
