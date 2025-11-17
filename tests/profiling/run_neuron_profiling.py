#!/usr/bin/env python3
"""
Neuron Coordinate Extraction Profiling Runner

Quick script to run all neuron coordinate extraction performance tests
and display a summary of results.

Usage:
    python tests/profiling/run_neuron_profiling.py
"""

import json
import subprocess
import sys
import time
from pathlib import Path


def run_profiling_tests():
    """Run the neuron coordinate extraction profiling tests."""
    print("🚀 FEAGI Neuron Coordinate Extraction Performance Profiling")
    print("=" * 60)
    print()

    # Run the comprehensive benchmark test
    print("Running comprehensive benchmark suite...")

    cmd = [
        sys.executable,
        "-m",
        "pytest",
        "tests/profiling/test_neuron_coordinate_extraction.py::test_coordinate_extraction_benchmark_suite",
        "-v",
        "--tb=short",
    ]

    start_time = time.time()
    result = subprocess.run(
        cmd, capture_output=True, text=True, cwd=Path(__file__).parent.parent.parent
    )
    end_time = time.time()

    if result.returncode == 0:
        print("✅ Benchmark completed successfully!")
        print(f"⏱️  Total runtime: {end_time - start_time:.1f} seconds")
        print()

        # Find and parse the latest results file
        logs_dir = Path(__file__).parent / "logs"
        result_files = list(
            logs_dir.glob("neuron_coordinate_extraction_profile_*.json")
        )

        if result_files:
            latest_file = max(result_files, key=lambda f: f.stat().st_mtime)

            try:
                with open(latest_file, "r") as f:
                    data = json.load(f)

                print("📊 PERFORMANCE SUMMARY")
                print("-" * 30)

                # Setup results
                if "setup_results" in data:
                    setup = data["setup_results"]
                    print(f"Setup: {setup['neuron_count']:,} neurons created")
                    print(
                        f"Setup rate: {setup['neurons_per_second_setup']:,.0f} neurons/sec"
                    )
                    print()

                # Scaling analysis
                if "scaling_analysis" in data:
                    print("Performance by method:")
                    for method, analysis in data["scaling_analysis"].items():
                        print(
                            f"  • {method}: {analysis['optimal_performance']:,.0f} neurons/sec"
                        )
                        print(
                            f"    (optimal batch size: {analysis['optimal_batch_size']:,})"
                        )
                    print()

                # Recommendation
                if "optimal_batch_size" in data:
                    print(
                        f"🎯 Recommended batch size: {data['optimal_batch_size']:,} neurons"
                    )

                print(f"💾 Full results: {latest_file}")

            except Exception as e:
                print(f"⚠️  Could not parse results file: {e}")
        else:
            print("⚠️  No results file found")

    else:
        print("❌ Benchmark failed!")
        print("STDOUT:", result.stdout)
        print("STDERR:", result.stderr)
        return False

    return True


def run_quick_test():
    """Run a quick performance test."""
    print()
    print("🏃 Quick Performance Test")
    print("-" * 25)

    cmd = [
        sys.executable,
        "-m",
        "pytest",
        "tests/profiling/test_neuron_coordinate_extraction.py::test_small_batch_coordinate_extraction",
        "-v",
        "-s",
    ]

    result = subprocess.run(cmd, cwd=Path(__file__).parent.parent.parent)
    return result.returncode == 0


if __name__ == "__main__":
    print()
    success = True

    # Run comprehensive profiling
    if not run_profiling_tests():
        success = False

    # Run quick test
    if not run_quick_test():
        success = False

    print()
    if success:
        print("✅ All profiling tests completed successfully!")
        print()
        print("📋 Available test commands:")
        print("  # Run all profiling tests:")
        print(
            "  python -m pytest tests/profiling/test_neuron_coordinate_extraction.py -v"
        )
        print()
        print("  # Run just the comprehensive benchmark:")
        print(
            "  python -m pytest tests/profiling/test_neuron_coordinate_extraction.py::test_coordinate_extraction_benchmark_suite -v"
        )
        print()
        print("  # Run quick small batch test:")
        print(
            "  python -m pytest tests/profiling/test_neuron_coordinate_extraction.py::test_small_batch_coordinate_extraction -v"
        )
    else:
        print("❌ Some tests failed. Check output above for details.")
        sys.exit(1)
