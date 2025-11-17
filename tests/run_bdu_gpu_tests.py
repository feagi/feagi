#!/usr/bin/env python3
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
