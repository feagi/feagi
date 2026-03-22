#!/usr/bin/env python3
"""
Test that feagi.genome.validate_genome and feagi_rust_py_libs.genome work correctly.
Run from feagi-python-sdk: python scripts/test_genome_import.py
"""
import json
import sys


def test_feagi_rust_py_libs_genome():
    """Direct import from feagi_rust_py_libs.genome"""
    from feagi_rust_py_libs.genome import validate_genome, auto_fix_genome

    genome = {
        "version": "2.0",
        "genome_id": "g-test",
        "blueprint": {},
        "neuron_morphologies": {},
        "physiology": {"simulation_timestep": 0.01, "max_age": 100},
    }
    result = validate_genome(json.dumps(genome))
    assert result.valid, result.errors
    print("feagi_rust_py_libs.genome: OK")


def test_feagi_genome():
    """Import via feagi.genome (as used by nrs-composer)"""
    from feagi.genome import validate_genome

    genome = {
        "version": "2.0",
        "genome_id": "g-test",
        "blueprint": {},
        "neuron_morphologies": {},
        "physiology": {"simulation_timestep": 0.01, "max_age": 100},
    }
    valid, errors = validate_genome(genome)
    assert valid, errors
    print("feagi.genome.validate_genome: OK")


def main():
    try:
        test_feagi_rust_py_libs_genome()
        test_feagi_genome()
        print("\nAll genome import tests passed.")
        return 0
    except Exception as e:
        print(f"\nFAILED: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
