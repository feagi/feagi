#!/usr/bin/env python3
"""
Test runner for GPU-optimized BDU tests.

This script runs unit tests for the GPU-optimized ConnectomeManager and related components.
"""

import os
import sys
import pytest

if __name__ == "__main__":
    # Add the root directory to sys.path
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    
    # Define the test files to run
    test_files = [
        "tests/unit/bdu/test_connectome_manager_gpu.py",
    ]
    
    # Run the tests
    pytest.main(["-v"] + test_files) 