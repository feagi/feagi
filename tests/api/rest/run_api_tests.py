#!/usr/bin/env python
"""Main script to run FEAGI API tests across different API versions."""

import os
import sys
import argparse
import subprocess
from pathlib import Path

def main():
    """Parse arguments and dispatch to appropriate version-specific test runner."""
    parser = argparse.ArgumentParser(
        description="Run FEAGI REST API tests across different API versions."
    )
    parser.add_argument("-p", "--pattern", help="Test pattern to run (e.g., 'genome')")
    parser.add_argument("-g", "--group", help="API group to filter tests (e.g., 'brain_state', 'genome')")
    parser.add_argument("-v", "--verbose", action="store_true", help="Run tests in verbose mode")
    parser.add_argument("-j", "--parallel", type=int, default=1, 
                       help="Number of parallel processes to use (requires pytest-xdist)")
    parser.add_argument("--version", choices=["v1", "v2", "all"], default="v1",
                       help="API version to test (default: v1)")
    parser.add_argument("--list-groups", action="store_true", 
                       help="List available API test groups and exit")
    parser.add_argument("--use-patched", action="store_true",
                       help="Use the patched conftest with lightweight mocks")
    args = parser.parse_args()
    
    # Get the directory of this script
    script_dir = Path(os.path.dirname(os.path.abspath(__file__)))
    
    # Define the versions to test
    if args.version == "all":
        versions = ["v1", "v2"]
    else:
        versions = [args.version]
    
    # Handle list-groups command - use v1 for this since it has the most groups
    if args.list_groups:
        v1_script = script_dir / "v1" / "run_patched_tests.py"
        if v1_script.exists():
            cmd = [sys.executable, str(v1_script), "--list-groups"]
            subprocess.call(cmd)
        else:
            print("Available API test groups:")
            print("  burst_engine - Tests for the Burst Engine API")
            print("  brain_state - Tests for the Brain State API")
            print("  genome - Tests for the Genome API")
            print("  mapping - Tests for the Cortical Mapping API")
            print("  region - Tests for the Brain Region API")
            print("  system - Tests for the System API")
            print("  simulation - Tests for the Simulation API")
            print("  insights - Tests for the Insights API")
            print("  inputs - Tests for the Inputs API")
        return 0
    
    # Run tests for each version
    exit_codes = []
    
    for version in versions:
        version_dir = script_dir / version
        if not version_dir.exists() or not version_dir.is_dir():
            print(f"Warning: {version} directory not found at {version_dir}")
            continue
            
        # Use the version-specific runner if it exists and patched mode is enabled
        if args.use_patched:
            patched_runner = version_dir / "run_patched_tests.py"
            if patched_runner.exists():
                cmd = [sys.executable, str(patched_runner)]
                
                # Add common arguments
                if args.pattern:
                    cmd.extend(["-p", args.pattern])
                if args.group:
                    cmd.extend(["-g", args.group])
                if args.verbose:
                    cmd.append("-v")
                if args.parallel > 1:
                    cmd.extend(["-j", str(args.parallel)])
                
                print(f"\n{'='*80}")
                print(f"Running {version} tests with patched conftest")
                print(f"{'='*80}")
                
                exit_code = subprocess.call(cmd)
                exit_codes.append(exit_code)
                continue
        
        # If no patched runner or patched mode not requested, use direct pytest
        cmd = ["pytest"]
        
        # Add verbosity
        if args.verbose:
            cmd.append("-v")
        
        # Add parallelism if requested
        if args.parallel > 1:
            cmd.append(f"-n={args.parallel}")
        
        # Build tests pattern
        test_pattern = version_dir
        if args.pattern:
            pattern_to_match = f"*{args.pattern}*.py"
            test_pattern = str(version_dir / pattern_to_match)
        
        # Add API group marker if specified
        if args.group:
            cmd.extend(["-m", f"api_group({args.group})"])
            
        # Add the test pattern
        cmd.append(str(test_pattern))
        
        print(f"\n{'='*80}")
        print(f"Running {version} tests: {' '.join(cmd)}")
        print(f"{'='*80}")
        
        exit_code = subprocess.call(cmd)
        exit_codes.append(exit_code)
    
    # Return non-zero if any version's tests failed
    return 1 if any(code != 0 for code in exit_codes) else 0
        
if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nTest run canceled by user.")
        sys.exit(130)
    except Exception as e:
        print(f"\nAn error occurred: {str(e)}")
        sys.exit(1)
