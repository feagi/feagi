#!/usr/bin/env python
"""Script to run FEAGI API tests with lightweight mocks, avoiding heavy dependencies."""

import os
import sys
import subprocess
import argparse
import shutil
import glob
from pathlib import Path

def run_tests(pattern=None, group=None, verbose=False, parallel=1, version="v1"):
    """
    Run the API tests with the specified pattern or group, using the patched conftest.
    
    Args:
        pattern: Optional pattern to filter test files (e.g., 'region' for test_region_api.py)
        group: Optional API group to filter tests (e.g., 'brain_state', 'genome', etc.)
        verbose: Whether to run tests in verbose mode
        parallel: Number of parallel processes to use for testing
    
    Returns:
        Exit code from pytest.
    """
    # Get the directory of this script
    script_dir = Path(os.path.dirname(os.path.abspath(__file__)))
    
    # File paths
    original_conftest = script_dir / "conftest.py"
    backup_conftest = script_dir / "conftest.py.bak"
    patched_conftest = script_dir / "conftest_patched.py"
    
    # Check if files exist
    if not patched_conftest.exists():
        print(f"Error: {patched_conftest} does not exist")
        return 1
    
    # Backup original conftest.py if it exists and swap with patched version
    original_existed = original_conftest.exists()
    if original_existed:
        print(f"Backing up {original_conftest} to {backup_conftest}")
        shutil.copy2(original_conftest, backup_conftest)
    
    # Copy patched conftest to the standard location
    print(f"Using patched conftest for testing")
    shutil.copy2(patched_conftest, original_conftest)
    
    try:
        # Determine which test files to run
        test_files = []
        
        if pattern:
            # Run a specific test file
            test_path = script_dir / f"test_{pattern}_api.py"
            if test_path.exists():
                test_files.append(str(test_path.name))
            else:
                print(f"Warning: No test file matching {test_path}")
        elif group:
            # Map groups to their corresponding test files
            group_to_files = {
                "mapping": ["test_cortical_mapping_api.py"],
                "brain_state": ["test_brain_api.py"],
                "genome": ["test_genome_api.py"],
                "burst_engine": ["test_burst_engine_api.py"],
                "region": ["test_region_api.py"],
                "system": ["test_system_api.py"],
                "simulation": ["test_simulation_api.py"],
                "insights": ["test_insights_api.py"],
                "inputs": ["test_inputs_api.py"]
            }
            
            if group in group_to_files:
                for file in group_to_files[group]:
                    file_path = script_dir / file
                    if file_path.exists():
                        test_files.append(file)
            else:
                print(f"Warning: Unknown group '{group}'")
        else:
            # Find all test_*_api.py files
            test_files = [os.path.basename(f) for f in glob.glob(str(script_dir / "test_*_api.py"))]
        
        if not test_files:
            print("No test files found!")
            return 1
        
        # Build the command
        cmd = ["pytest"]
        
        # Add verbosity
        if verbose:
            cmd.append("-v")
        
        # Add parallelism if requested
        if parallel > 1:
            cmd.append(f"-n={parallel}")
            
        # Add test files to command
        cmd.extend(test_files)
        
        # Print the command
        print(f"Running with patched conftest: {' '.join(cmd)}")
        print(f"Directory: {script_dir}")
        print("=" * 80)
        
        # Run the command
        return subprocess.call(cmd, cwd=script_dir)
    finally:
        # Restore original conftest.py if it existed
        if original_existed:
            print(f"Restoring original conftest from {backup_conftest}")
            shutil.copy2(backup_conftest, original_conftest)
            os.remove(backup_conftest)
        else:
            # Remove the temporary conftest.py
            os.remove(original_conftest)

def main():
    """Run the tests with command line arguments."""
    parser = argparse.ArgumentParser(
        description="Run FEAGI REST API tests with lightweight mocks, avoiding heavy dependencies."
    )
    parser.add_argument("-p", "--pattern", help="Test pattern to run (e.g., 'region' to run test_region_api.py)")
    parser.add_argument("-g", "--group", help="API group to filter tests (e.g., 'brain_state', 'genome')")
    parser.add_argument("-v", "--verbose", action="store_true", help="Run tests in verbose mode")
    parser.add_argument("-j", "--parallel", type=int, default=1, 
                       help="Number of parallel processes to use (requires pytest-xdist)")
    parser.add_argument("--list-groups", action="store_true", 
                       help="List available API test groups and exit")
    args = parser.parse_args()
    
    if args.list_groups:
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
        sys.exit(0)
    
    try:
        exit_code = run_tests(args.pattern, args.group, args.verbose, args.parallel)
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\nTest run canceled by user.")
        sys.exit(130)  # 130 is the standard exit code for SIGINT
    except Exception as e:
        print(f"\nAn error occurred: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main() 