import os
import sys
from setuptools import setup, find_packages

# Check if setuptools-rust is available
try:
    from setuptools_rust import Binding, RustExtension
    HAS_SETUPTOOLS_RUST = True
except ImportError:
    HAS_SETUPTOOLS_RUST = False

# Custom build settings for Rust extension
if HAS_SETUPTOOLS_RUST:
    rust_extensions = [
        RustExtension(
            "feagi.rust.feagi_rust",
            path="feagi-rust/Cargo.toml",
            binding=Binding.PyO3,
            debug=False,
        )
    ]
else:
    rust_extensions = []
    print("Warning: setuptools-rust is not available. Rust extensions will not be built.")

setup(
    name="feagi",
    packages=find_packages(),
    rust_extensions=rust_extensions,
    zip_safe=False,  # Required for Rust extensions
    include_package_data=True,
    entry_points={
        "console_scripts": [
            "feagi-api=feagi.api.server:main",
        ],
    },
) 