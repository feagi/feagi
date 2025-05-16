#!/bin/bash
# FEAGI Runner Script
# This script runs the FEAGI system with proper error handling and cleanup.

# Default settings
API_HOST="127.0.0.1"
API_PORT=8000
ZMQ_HOST="127.0.0.1"
ZMQ_REQ_PORT=5555
ZMQ_PUB_PORT=5556
ZMQ_PUSH_PORT=5557
ZMQ_SENSORIMOTOR_PORT=5558
ZMQ_VIS_BASE_PORT=5560
USE_GPU=0

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
    --zmq-req-port)
      ZMQ_REQ_PORT="$2"
      shift 2
      ;;
    --zmq-pub-port)
      ZMQ_PUB_PORT="$2"
      shift 2
      ;;
    --zmq-push-port)
      ZMQ_PUSH_PORT="$2"
      shift 2
      ;;
    --zmq-sensorimotor-port)
      ZMQ_SENSORIMOTOR_PORT="$2"
      shift 2
      ;;
    --zmq-vis-base-port)
      ZMQ_VIS_BASE_PORT="$2"
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
      echo "  --zmq-req-port PORT        ZMQ REQ/REP port (default: $ZMQ_REQ_PORT)"
      echo "  --zmq-pub-port PORT        ZMQ PUB/SUB port (default: $ZMQ_PUB_PORT)"
      echo "  --zmq-push-port PORT       ZMQ PUSH/PULL port (default: $ZMQ_PUSH_PORT)"
      echo "  --zmq-sensorimotor-port PORT  ZMQ sensorimotor port (default: $ZMQ_SENSORIMOTOR_PORT)"
      echo "  --zmq-vis-base-port PORT   ZMQ visualization base port (default: $ZMQ_VIS_BASE_PORT)"
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
check_port "$ZMQ_REQ_PORT" || exit 1
check_port "$ZMQ_PUB_PORT" || exit 1
check_port "$ZMQ_PUSH_PORT" || exit 1
check_port "$ZMQ_SENSORIMOTOR_PORT" || exit 1
check_port "$ZMQ_VIS_BASE_PORT" || exit 1

# Build command
CMD="python -m feagi.main"
CMD="$CMD --api-host $API_HOST --api-port $API_PORT"
CMD="$CMD --zmq-host $ZMQ_HOST --zmq-req-port $ZMQ_REQ_PORT"
CMD="$CMD --zmq-pub-port $ZMQ_PUB_PORT --zmq-push-port $ZMQ_PUSH_PORT"
CMD="$CMD --zmq-sensorimotor-port $ZMQ_SENSORIMOTOR_PORT --zmq-vis-base-port $ZMQ_VIS_BASE_PORT"

if [ "$USE_GPU" -eq 1 ]; then
    CMD="$CMD --gpu"
fi

# Start FEAGI
echo "Starting FEAGI with command: $CMD"
echo "Press Ctrl+C to stop"
echo "------------------------------------"
$CMD

exit 0 