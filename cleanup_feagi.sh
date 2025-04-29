#!/bin/bash
# cleanup_feagi.sh - Consolidated script to clean up FEAGI processes and resources
# This script replaces both kill_feagi and cleanup_feagi.sh

# Set default options
VERBOSE=false
FORCE=false
CHECK_PORT=true
PORT=8000
CLEAN_SEMAPHORES=true

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Display help
function show_help {
    echo -e "${BLUE}FEAGI Cleanup Tool${NC}"
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -h, --help           Show this help message"
    echo "  -v, --verbose        Show detailed output"
    echo "  -f, --force          Force kill all processes (SIGKILL instead of SIGTERM)"
    echo "  -p, --port PORT      Check if port is free after cleanup (default: 8000)"
    echo "  --no-port-check      Skip port check"
    echo "  --no-semaphore-clean Skip cleaning semaphores"
    echo ""
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -f|--force)
            FORCE=true
            shift
            ;;
        -p|--port)
            PORT="$2"
            shift 2
            ;;
        --no-port-check)
            CHECK_PORT=false
            shift
            ;;
        --no-semaphore-clean)
            CLEAN_SEMAPHORES=false
            shift
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Function to log messages
log() {
    local level=$1
    local message=$2
    local color=$NC

    case $level in
        "INFO")
            color=$GREEN
            ;;
        "WARN")
            color=$YELLOW
            ;;
        "ERROR")
            color=$RED
            ;;
        *)
            color=$BLUE
            ;;
    esac

    echo -e "${color}[$level] $message${NC}"
}

# Only show verbose messages if VERBOSE is true
verbose_log() {
    if $VERBOSE; then
        log "INFO" "$1"
    fi
}

# Discover all FEAGI processes
find_feagi_processes() {
    verbose_log "Looking for FEAGI processes..."
    if [ "$(uname)" == "Darwin" ]; then
        # macOS
        pgrep -fl "feagi.main" || echo ""
    else
        # Linux
        ps aux | grep "feagi.main" | grep -v grep || echo ""
    fi
}

# Function to kill processes
kill_processes() {
    local process_pattern=$1
    local signal=$2
    local desc=$3

    verbose_log "Killing $desc processes with signal $signal..."

    if [ "$(uname)" == "Darwin" ]; then
        # macOS
        pkill $signal -f "$process_pattern" 2>/dev/null
    else
        # Linux
        pkill $signal -f "$process_pattern" 2>/dev/null
    fi
}

# Function to check if port is in use
check_port_in_use() {
    local port=$1
    if [ "$(uname)" == "Darwin" ]; then
        # macOS
        lsof -i:$port -sTCP:LISTEN >/dev/null 2>&1
    else
        # Linux
        netstat -tuln | grep ":$port " >/dev/null 2>&1
    fi
    return $?
}

# Function to release a port if it's in use
release_port() {
    local port=$1
    
    if check_port_in_use $port; then
        log "WARN" "Port $port is still in use. Attempting to release..."
        
        if [ "$(uname)" == "Darwin" ]; then
            # macOS
            pid=$(lsof -i:$port -sTCP:LISTEN -t)
            if [ -n "$pid" ]; then
                log "INFO" "Killing process $pid that is using port $port"
                kill -9 $pid 2>/dev/null
            fi
        else
            # Linux
            pid=$(netstat -tuln | grep ":$port " | awk '{print $7}' | cut -d'/' -f1)
            if [ -n "$pid" ]; then
                log "INFO" "Killing process $pid that is using port $port"
                kill -9 $pid 2>/dev/null
            fi
        fi
    else
        verbose_log "Port $port is free"
    fi
}

# Main cleanup function
cleanup_feagi() {
    log "INFO" "Starting FEAGI cleanup..."
    
    # Display all FEAGI processes if verbose
    if $VERBOSE; then
        processes=$(find_feagi_processes)
        if [ -n "$processes" ]; then
            log "INFO" "Found FEAGI processes:"
            echo "$processes"
        else
            log "INFO" "No FEAGI processes found."
        fi
    fi
    
    # Step 1: First try to terminate processes gracefully
    verbose_log "Step 1: Graceful termination with SIGTERM"
    kill_processes "feagi.main" "-15" "FEAGI"
    
    # Wait a moment for processes to terminate
    sleep 1
    
    # Step 2: Check if processes are still running, use SIGKILL if needed or if force option is set
    if $FORCE || [ -n "$(find_feagi_processes)" ]; then
        verbose_log "Step 2: Forceful termination with SIGKILL"
        kill_processes "feagi.main" "-9" "FEAGI"
    fi
    
    # Step 3: Clean up any Python processes related to FEAGI that might still be running
    if [ -n "$(pgrep -f "python.*feagi")" ]; then
        verbose_log "Step 3: Cleanup related Python processes"
        kill_processes "python.*feagi" "-9" "related Python"
    fi
    
    # Step 4: Check and release the API port if it's still in use
    if $CHECK_PORT; then
        verbose_log "Step 4: Checking API port $PORT"
        release_port $PORT
    fi
    
    # Step 5: Kill ZMQ-related processes that might be used by the visualization
    if [ -n "$(pgrep -f "zmq")" ]; then
        verbose_log "Step 5: Cleanup ZMQ-related processes"
        kill_processes "zmq" "-9" "ZMQ-related"
    fi
    
    # Step 6: Clean up semaphores (only if enabled)
    if $CLEAN_SEMAPHORES; then
        verbose_log "Step 6: Checking for leaked semaphores"
        # Unfortunately, manually cleaning up semaphores requires root on most systems
        # Instead, we'll just warn the user if Python reports leaked semaphores
        log "INFO" "If Python reports leaked semaphores, you may need to restart Python or logout and back in to clean them up"
    fi
    
    # Final check
    if [ -n "$(find_feagi_processes)" ]; then
        log "WARN" "Some FEAGI processes could not be terminated. You may need to restart your system if you continue to have issues."
    else
        log "INFO" "FEAGI cleanup completed successfully!"
    fi
}

# Run the cleanup
cleanup_feagi

# Exit with success
exit 0 