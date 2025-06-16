#!/bin/bash
#
# Copyright 2025 Neuraville Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# Script to run all non-GPU BDU tests
# Usage: ./run_non_gpu_tests.sh [pytest_args]
# Example: ./run_non_gpu_tests.sh -v

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

# Activate virtual environment if it exists
if [ -d "${PROJECT_ROOT}/feagi_core/.venv_new" ]; then
    source "${PROJECT_ROOT}/feagi_core/.venv_new/bin/activate"
fi

# Run all the non-GPU tests
python -m pytest \
    ${SCRIPT_DIR}/unit/test_connectome_manager.py \
    ${SCRIPT_DIR}/unit/test_array_backend.py::TestArrayBackend::test_auto_selection \
    ${SCRIPT_DIR}/unit/test_array_backend.py::TestArrayBackend::test_numpy_backend \
    ${SCRIPT_DIR}/unit/test_multi_gpu.py::TestMultiGPUConfig \
    ${SCRIPT_DIR}/unit/test_multi_gpu.py::TestBrainPartition \
    ${SCRIPT_DIR}/unit/test_multi_gpu.py::TestMultiGPUManagerMock \
    ${SCRIPT_DIR}/unit/test_mixed_precision.py \
    ${SCRIPT_DIR}/performance/test_synaptogenesis_performance.py \
    "$@"
