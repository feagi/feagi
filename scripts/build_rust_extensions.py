#!/usr/bin/env python3
"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Build script for FEAGI Rust extensions.

This script builds the Rust extensions for FEAGI, optimized with SIMD and WebGPU support.
Run this script before trying to use the optimized data structures.
"""

import argparse
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path


def detect_features():
    """Detect CPU features for optimization."""
    features = []

    system = platform.system()
    machine = platform.machine()

    # Check for AVX2 support on x86_64
    if machine in ["x86_64", "AMD64", "amd64"]:
        try:
            # Try to detect AVX2 support on Linux
            if system == "Linux":
                with open("/proc/cpuinfo", "r") as f:
                    if "avx2" in f.read().lower():
                        features.append("simd-avx2")
            # Try to detect AVX2 support on macOS or Windows
            elif system in ["Darwin", "Windows"]:
                # This is a very basic check, not guaranteed to work on all systems
                result = subprocess.run(
                    (
                        ["sysctl", "-a"]
                        if system == "Darwin"
                        else ["wmic", "cpu", "get", "Name"]
                    ),
                    capture_output=True,
                    text=True,
                )
                if "avx2" in result.stdout.lower():
                    features.append("simd-avx2")
        except Exception:
            pass

    # Check for ARM NEON support
    elif machine in ["arm64", "aarch64"]:
        # ARM64 always has NEON
        features.append("simd-neon")

    return features


def build_rust_extension(use_webgpu=False):
    """Build the Rust extension with optimized features."""
    print("Building Rust extensions for FEAGI...")

    # Get the project root directory
    project_root = Path(__file__).parent.parent.absolute()
    rust_dir = project_root / "feagi" / "rust"

    # Check if Rust directory exists
    if not rust_dir.exists():
        print(f"Error: Rust directory not found at {rust_dir}")
        return False

    # Change to the Rust directory
    os.chdir(rust_dir)

    # Detect features
    features = detect_features()

    # Add WebGPU feature if requested
    if use_webgpu:
        features.append("webgpu")

    features_str = " ".join(features)
    print(f"Detected features: {features_str}")

    # Build the Rust extension with detected features
    cmd = ["cargo", "build", "--release"]
    if features:
        cmd.extend(["--features", ",".join(features)])

    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        print("Error building Rust extension:")
        print(result.stderr)
        return False

    print("Rust extension built successfully.")

    # Copy the compiled library to the feagi directory
    target_dir = rust_dir / "target" / "release"

    # Determine the extension based on the platform
    if platform.system() == "Windows":
        extension = "dll"
    elif platform.system() == "Darwin":
        extension = "dylib"
    else:  # Linux
        extension = "so"

    lib_name = f"libfeagi_rust.{extension}"
    python_lib_name = "feagi_rust.{ext}".format(
        ext="pyd" if platform.system() == "Windows" else "so"
    )

    source = target_dir / lib_name
    destination = project_root / "feagi" / python_lib_name

    if not source.exists():
        print(f"Error: Compiled library not found at {source}")
        return False

    print(f"Copying {source} to {destination}")
    shutil.copy2(source, destination)

    print("Rust extension installed successfully.")
    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Build FEAGI Rust extensions")
    parser.add_argument(
        "--webgpu", action="store_true", help="Build with WebGPU support"
    )
    args = parser.parse_args()

    success = build_rust_extension(use_webgpu=args.webgpu)
    sys.exit(0 if success else 1)
