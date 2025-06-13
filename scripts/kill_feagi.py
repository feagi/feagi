#!/usr/bin/env python3
"""
FEAGI Process Killer - Python Version
Safely finds and terminates all FEAGI-related processes across platforms
"""

import argparse
import os
import sys
import time
from typing import List, Tuple

import psutil

# Patterns to identify FEAGI processes
FEAGI_PATTERNS = [
    "feagi",
    "main.py",
    "burst_engine",
    "connectome",
    "api_server",
    "zmq_server",
    "npu",
    "fcl",
    "fire_queue",
    "neuroembryogenesis",
    "synaptogenesis",
    "feagi_core",
    "feagi_bridge",
    "feagi_connector",
    "feagi_sim",
]


class Colors:
    """ANSI color codes for terminal output"""

    RED = "\033[0;31m"
    GREEN = "\033[0;32m"
    YELLOW = "\033[1;33m"
    BLUE = "\033[0;34m"
    NC = "\033[0m"  # No Color

    @classmethod
    def disable(cls):
        """Disable colors for Windows or when piping output"""
        cls.RED = cls.GREEN = cls.YELLOW = cls.BLUE = cls.NC = ""


def print_colored(message: str, color: str = Colors.NC, end: str = "\n"):
    """Print message with color"""
    print(f"{color}{message}{Colors.NC}", end=end)


def find_feagi_processes() -> List[Tuple[int, str]]:
    """
    Find all FEAGI-related processes

    Returns:
        List of tuples (pid, command_line)
    """
    print_colored("Searching for FEAGI processes...", Colors.YELLOW)

    feagi_processes = []

    try:
        # Get all running processes
        all_processes = psutil.process_iter(["pid", "name", "cmdline"])

        for proc in all_processes:
            try:
                proc_info = proc.info
                pid = proc_info["pid"]
                # name = proc_info["name"]  # Unused variable removed
                cmdline = proc_info["cmdline"]

                if not cmdline:
                    continue

                # Convert cmdline list to string for easier searching
                cmdline_str = " ".join(cmdline)

                # Check if this is a FEAGI process
                is_feagi = any(
                    pattern in cmdline_str.lower() for pattern in FEAGI_PATTERNS
                )

                if is_feagi:
                    feagi_processes.append((pid, cmdline_str))

            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                # Process disappeared or we don't have access
                continue

    except Exception as e:
        print_colored(f"Error searching for processes: {e}", Colors.RED)
        return []

    return feagi_processes


def display_processes(processes: List[Tuple[int, str]]) -> None:
    """Display found processes in a formatted way"""
    if not processes:
        print_colored("No FEAGI processes found running", Colors.GREEN)
        return

    print_colored("Found the following FEAGI processes:", Colors.RED)
    print("=" * 60)

    for pid, cmdline in processes:
        try:
            proc = psutil.Process(pid)
            print_colored(f"PID {pid}:", Colors.YELLOW)
            print(f"  User: {proc.username()}")
            print(f"  Command: {cmdline[:100]}{'...' if len(cmdline) > 100 else ''}")
            print(f"  Memory: {proc.memory_info().rss / 1024 / 1024:.1f} MB")
            print()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            print_colored(f"PID {pid}: Process no longer accessible", Colors.YELLOW)
            print()


def kill_processes_graceful(pids: List[int]) -> List[int]:
    """
    Attempt to kill processes gracefully using SIGTERM

    Args:
        pids: List of process IDs to kill

    Returns:
        List of PIDs that are still running
    """
    print_colored("Attempting graceful shutdown (SIGTERM)...", Colors.YELLOW)

    for pid in pids:
        try:
            proc = psutil.Process(pid)
            print(f"  Sending SIGTERM to PID {pid}")
            proc.terminate()
        except psutil.NoSuchProcess:
            print(f"  PID {pid} already terminated")
        except psutil.AccessDenied:
            print(f"  Access denied for PID {pid}")
        except Exception as e:
            print(f"  Error terminating PID {pid}: {e}")

    # Wait for graceful shutdown
    print("  Waiting 5 seconds for graceful shutdown...")
    time.sleep(5)

    # Check which processes are still running
    still_running = []
    for pid in pids:
        try:
            proc = psutil.Process(pid)
            if proc.is_running():
                still_running.append(pid)
        except psutil.NoSuchProcess:
            # Process is gone - good!
            pass

    return still_running


def kill_processes_force(pids: List[int]) -> List[int]:
    """
    Force kill processes using SIGKILL

    Args:
        pids: List of process IDs to kill

    Returns:
        List of PIDs that are still running
    """
    print_colored("Force killing remaining processes (SIGKILL)...", Colors.RED)

    for pid in pids:
        try:
            proc = psutil.Process(pid)
            print(f"  Force killing PID {pid}")
            proc.kill()
        except psutil.NoSuchProcess:
            print(f"  PID {pid} already terminated")
        except psutil.AccessDenied:
            print(f"  Access denied for PID {pid}")
        except Exception as e:
            print(f"  Error force killing PID {pid}: {e}")

    time.sleep(2)

    # Check which processes are still running
    still_running = []
    for pid in pids:
        try:
            proc = psutil.Process(pid)
            if proc.is_running():
                still_running.append(pid)
        except psutil.NoSuchProcess:
            # Process is gone - good!
            pass

    return still_running


def confirm_kill(force: bool = False) -> bool:
    """Ask user for confirmation unless force is True"""
    if force:
        return True

    print_colored(
        "Do you want to kill these FEAGI processes? (y/N): ", Colors.YELLOW, end=""
    )
    response = input().strip().lower()
    return response in ["y", "yes"]


def main():
    """Main execution function"""
    parser = argparse.ArgumentParser(
        description="FEAGI Process Killer - Safely terminate all FEAGI processes"
    )
    parser.add_argument(
        "--force", "-f", action="store_true", help="Kill processes without confirmation"
    )
    parser.add_argument(
        "--no-color", action="store_true", help="Disable colored output"
    )

    args = parser.parse_args()

    # Disable colors on Windows or if requested
    if os.name == "nt" or args.no_color:
        Colors.disable()

    print_colored("FEAGI Process Killer", Colors.BLUE)
    print("=" * 40)

    # Find FEAGI processes
    processes = find_feagi_processes()

    if not processes:
        return 0

    # Display found processes
    display_processes(processes)

    # Ask for confirmation
    if not confirm_kill(args.force):
        print_colored("Cancelled. No processes were killed.", Colors.GREEN)
        return 0

    # Extract PIDs
    pids = [pid for pid, _ in processes]

    # Try graceful shutdown first
    remaining_pids = kill_processes_graceful(pids)

    if remaining_pids:
        print_colored(
            f"{len(remaining_pids)} processes still running. Force killing...",
            Colors.YELLOW,
        )
        final_remaining = kill_processes_force(remaining_pids)

        if final_remaining:
            print_colored("Warning: Some processes could not be killed:", Colors.RED)
            for pid in final_remaining:
                try:
                    proc = psutil.Process(pid)
                    print(f"  PID {pid}: {' '.join(proc.cmdline())}")
                except psutil.NoSuchProcess:
                    pass
            return 1

    print_colored("All FEAGI processes have been terminated successfully", Colors.GREEN)

    # Final verification
    print_colored("Final verification...", Colors.BLUE)
    time.sleep(1)
    final_check = find_feagi_processes()
    if final_check:
        print_colored(
            f"Warning: {len(final_check)} FEAGI processes still detected", Colors.YELLOW
        )
        display_processes(final_check)

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print_colored("\nOperation cancelled by user", Colors.YELLOW)
        sys.exit(1)
    except Exception as e:
        print_colored(f"Unexpected error: {e}", Colors.RED)
        sys.exit(1)
