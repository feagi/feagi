#!/bin/bash

# FEAGI Process Killer Script
# Safely finds and terminates all FEAGI-related processes

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🔍 FEAGI Process Killer${NC}"
echo "========================================"

# Function to find FEAGI processes
find_feagi_processes() {
    echo -e "${YELLOW}Searching for FEAGI processes...${NC}"

    # Find processes containing FEAGI-related patterns
    FEAGI_PIDS=$(ps aux | grep -E "(feagi|FEAGI)" | grep -v grep | grep -v "$0" | awk '{print $2}' || true)

    # Also check for Python processes running FEAGI code
    PYTHON_FEAGI_PIDS=$(ps aux | grep -E "python.*feagi" | grep -v grep | grep -v "$0" | awk '{print $2}' || true)

    # Combine and deduplicate PIDs
    ALL_PIDS=$(echo -e "$FEAGI_PIDS\n$PYTHON_FEAGI_PIDS" | sort -u | grep -v '^$' || true)

    if [ -z "$ALL_PIDS" ]; then
        echo -e "${GREEN}✅ No FEAGI processes found running${NC}"
        exit 0
    fi

    echo -e "${RED}Found the following FEAGI processes:${NC}"
    echo "========================================"

    # Show detailed process information
    for pid in $ALL_PIDS; do
        if ps -p $pid > /dev/null 2>&1; then
            echo -e "${YELLOW}PID $pid:${NC}"
            ps -p $pid -o pid,ppid,user,command --no-headers | sed 's/^/  /'
            echo ""
        fi
    done

    echo "$ALL_PIDS"
}

# Function to kill processes gracefully
kill_processes_graceful() {
    local pids="$1"
    echo -e "${YELLOW}Attempting graceful shutdown (SIGTERM)...${NC}"

    for pid in $pids; do
        if ps -p $pid > /dev/null 2>&1; then
            echo "  Sending SIGTERM to PID $pid"
            kill -TERM $pid 2>/dev/null || true
        fi
    done

    # Wait a few seconds for graceful shutdown
    echo "  Waiting 5 seconds for graceful shutdown..."
    sleep 5
}

# Function to kill processes forcefully
kill_processes_force() {
    local pids="$1"
    echo -e "${RED}Force killing remaining processes (SIGKILL)...${NC}"

    for pid in $pids; do
        if ps -p $pid > /dev/null 2>&1; then
            echo "  Force killing PID $pid"
            kill -KILL $pid 2>/dev/null || true
        fi
    done

    sleep 2
}

# Function to check if processes are still running
check_remaining_processes() {
    local pids="$1"
    local remaining=""

    for pid in $pids; do
        if ps -p $pid > /dev/null 2>&1; then
            remaining="$remaining $pid"
        fi
    done

    echo "$remaining"
}

# Main execution
main() {
    # Find FEAGI processes
    PIDS=$(find_feagi_processes)

    if [ -z "$PIDS" ]; then
        exit 0
    fi

    # Ask for confirmation unless --force flag is used
    if [ "$1" != "--force" ] && [ "$1" != "-f" ]; then
        echo -e "${YELLOW}Do you want to kill these FEAGI processes? (y/N):${NC}"
        read -r response
        case "$response" in
            [yY][eE][sS]|[yY])
                echo "Proceeding with process termination..."
                ;;
            *)
                echo -e "${GREEN}Cancelled. No processes were killed.${NC}"
                exit 0
                ;;
        esac
    fi

    # Kill processes gracefully first
    kill_processes_graceful "$PIDS"

    # Check which processes are still running
    REMAINING=$(check_remaining_processes "$PIDS")

    if [ -n "$REMAINING" ]; then
        echo -e "${YELLOW}Some processes are still running. Force killing...${NC}"
        kill_processes_force "$REMAINING"

        # Final check
        FINAL_REMAINING=$(check_remaining_processes "$REMAINING")
        if [ -n "$FINAL_REMAINING" ]; then
            echo -e "${RED}⚠️  Warning: Some processes could not be killed:${NC}"
            for pid in $FINAL_REMAINING; do
                if ps -p $pid > /dev/null 2>&1; then
                    ps -p $pid -o pid,user,command --no-headers | sed 's/^/  /'
                fi
            done
            exit 1
        fi
    fi

    echo -e "${GREEN}✅ All FEAGI processes have been terminated successfully${NC}"

    # Check for any lingering FEAGI processes one more time
    echo -e "${BLUE}Final verification...${NC}"
    sleep 1
    FINAL_CHECK=$(find_feagi_processes 2>/dev/null || true)
}

# Handle script arguments
case "$1" in
    --help|-h)
        echo "FEAGI Process Killer"
        echo "Usage: $0 [OPTIONS]"
        echo ""
        echo "OPTIONS:"
        echo "  --force, -f    Kill processes without confirmation"
        echo "  --help,  -h    Show this help message"
        echo ""
        echo "This script finds and safely terminates all FEAGI-related processes."
        exit 0
        ;;
    *)
        main "$1"
        ;;
esac
