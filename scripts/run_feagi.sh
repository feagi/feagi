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

# FEAGI Runner Script
# This script runs the FEAGI system with proper error handling and cleanup.

# Default settings
API_HOST="127.0.0.1"
API_PORT=8000
ZMQ_HOST="127.0.0.1"
ZMQ_REQ_REP_PORT=5555
ZMQ_PUB_SUB_PORT=5556
ZMQ_PUSH_PULL_PORT=5557
ZMQ_SENSORY_PORT=5558
ZMQ_CONTROL_PORT=5559
ZMQ_VIS_PORT=5560
ZMQ_MOTOR_PORT=5564
USE_GPU=0
LOG_LEVEL="INFO"

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --api-host)
      API_HOST="$2"
      shift 2
      ;;
    --api-port)
      API_PORT="$2"
      shift 2
      ;;
    --zmq-host)
      ZMQ_HOST="$2"
      shift 2
      ;;
    --zmq-req-rep-port)
      ZMQ_REQ_REP_PORT="$2"
      shift 2
      ;;
    --zmq-pub-sub-port)
      ZMQ_PUB_SUB_PORT="$2"
      shift 2
      ;;
    --zmq-push-pull-port)
      ZMQ_PUSH_PULL_PORT="$2"
      shift 2
      ;;
    --zmq-sensory-port)
      ZMQ_SENSORY_PORT="$2"
      shift 2
      ;;
    --zmq-control-port)
      ZMQ_CONTROL_PORT="$2"
      shift 2
      ;;
    --zmq-motor-port)
      ZMQ_MOTOR_PORT="$2"
      shift 2
      ;;
    --zmq-vis-port)
      ZMQ_VIS_PORT="$2"
      shift 2
      ;;
    --log-level)
      LOG_LEVEL="$2"
      shift 2
      ;;
    --gpu)
      USE_GPU=1
      shift
      ;;
    --help)
      echo "FEAGI Runner Script"
      echo "Usage: $0 [options]"
      echo ""
      echo "Options:"
      echo "  --api-host HOST            API server host (default: $API_HOST)"
      echo "  --api-port PORT            API server port (default: $API_PORT)"
      echo "  --zmq-host HOST            ZMQ server host (default: $ZMQ_HOST)"
      echo "  --zmq-req-rep-port PORT    ZMQ request-reply port (default: $ZMQ_REQ_REP_PORT)"
      echo "  --zmq-pub-sub-port PORT    ZMQ publish-subscribe port (default: $ZMQ_PUB_SUB_PORT)"
      echo "  --zmq-push-pull-port PORT  ZMQ push-pull port (default: $ZMQ_PUSH_PULL_PORT)"
      echo "  --zmq-sensory-port PORT    ZMQ sensory port (default: $ZMQ_SENSORY_PORT)"
      echo "  --zmq-control-port PORT    ZMQ control port (default: $ZMQ_CONTROL_PORT)"
      echo "  --zmq-motor-port PORT      ZMQ motor port (default: $ZMQ_MOTOR_PORT)"
      echo "  --zmq-vis-port PORT        ZMQ visualization port (default: $ZMQ_VIS_PORT)"
      echo "  --log-level LEVEL          Logging level (default: $LOG_LEVEL)"
      echo "  --gpu                      Use GPU acceleration if available"
      echo "  --help                     Show this help message"
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      echo "Use --help for usage information"
      exit 1
      ;;
  esac
done

# Ensure we're in the root directory
cd "$(dirname "$0")"

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    echo "Error: Python 3 is required but not found"
    exit 1
fi

# Clean up on exit
function cleanup {
    echo "Cleaning up FEAGI processes..."
    pkill -f "python -m feagi.main" 2>/dev/null
    echo "Done"
}

# Register cleanup function
trap cleanup EXIT

# Kill any existing FEAGI processes
echo "Checking for existing FEAGI processes..."
if pgrep -f "python -m feagi.main" > /dev/null; then
    echo "Found existing FEAGI processes, stopping them..."
    pkill -f "python -m feagi.main"
    sleep 2
fi

# Check if ports are already in use
function check_port {
    if lsof -i:"$1" > /dev/null 2>&1; then
        echo "Error: Port $1 is already in use"
        return 1
    fi
    return 0
}

echo "Checking ports..."
check_port "$API_PORT" || exit 1
check_port "$ZMQ_REQ_REP_PORT" || exit 1
check_port "$ZMQ_PUB_SUB_PORT" || exit 1
check_port "$ZMQ_PUSH_PULL_PORT" || exit 1
check_port "$ZMQ_SENSORY_PORT" || exit 1
check_port "$ZMQ_CONTROL_PORT" || exit 1
check_port "$ZMQ_MOTOR_PORT" || exit 1
check_port "$ZMQ_VIS_PORT" || exit 1

# Build command
CMD="python -m feagi.main"
CMD="$CMD --api-host $API_HOST --api-port $API_PORT"
CMD="$CMD --zmq-host $ZMQ_HOST --zmq-req-rep-port $ZMQ_REQ_REP_PORT --zmq-pub-sub-port $ZMQ_PUB_SUB_PORT --zmq-push-pull-port $ZMQ_PUSH_PULL_PORT"
CMD="$CMD --zmq-sensory-port $ZMQ_SENSORY_PORT --zmq-control-port $ZMQ_CONTROL_PORT --zmq-motor-port $ZMQ_MOTOR_PORT --zmq-vis-port $ZMQ_VIS_PORT"

if [ "$USE_GPU" -eq 1 ]; then
    CMD="$CMD --gpu"
fi

# Start FEAGI
echo "Starting FEAGI with command: $CMD"
echo "Press Ctrl+C to stop"
echo "------------------------------------"
$CMD

exit 0 