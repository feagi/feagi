#!/bin/bash
# Script to run all system-level integration tests
# Usage: ./run_integration_tests.sh [pytest_args]
# Example: ./run_integration_tests.sh -v

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# Activate virtual environment if it exists
if [ -d "${PROJECT_ROOT}/feagi_core/.venv_new" ]; then
    source "${PROJECT_ROOT}/feagi_core/.venv_new/bin/activate"
fi

# Run the system integration tests
python -m pytest ${SCRIPT_DIR}/integration "$@" 