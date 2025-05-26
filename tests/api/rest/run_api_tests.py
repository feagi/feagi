#!/usr/bin/env python
"""Script to run all FEAGI REST API tests."""

import os
import sys
import subprocess
import argparse
from pathlib import Path

def run_tests(pattern=None, verbose=False):
    """
    Run the API tests with the specified pattern.
    
    Args:
        pattern: Optional pattern to filter test files.
        verbose: Whether to run tests in verbose mode.
    
    Returns:
        Exit code from pytest.
    """
    # Get the directory of this script
    script_dir = Path(os.path.dirname(os.path.abspath(__file__)))
    
    # Build the command
    cmd = ["pytest"]
    
    # Add verbosity
    if verbose:
        cmd.append("-v")
    
    # Add coverage if requested
    cmd.extend(["--cov=feagi.api.rest", "--cov-report=term"])
    
    # Add pattern if specified, otherwise run all test_* files
    if pattern:
        cmd.append(f"test_{pattern}_api.py")
    else:
        cmd.append("test_*_api.py")
    
    # Print the command
    print(f"Running: {' '.join(cmd)}")
    print(f"Directory: {script_dir}")
    print("=" * 80)
    
    # Run the command
    return subprocess.call(cmd, cwd=script_dir)

def main():
    """Run the tests with command line arguments."""
    parser = argparse.ArgumentParser(description="Run FEAGI REST API tests.")
    parser.add_argument("-p", "--pattern", help="Test pattern to run (e.g., 'region' to run test_region_api.py)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Run tests in verbose mode")
    args = parser.parse_args()
    
    try:
        exit_code = run_tests(args.pattern, args.verbose)
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\nTest run canceled by user.")
        sys.exit(130)  # 130 is the standard exit code for SIGINT
if __name__ == "__main__":
    main() 
