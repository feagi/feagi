#!/bin/bash
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