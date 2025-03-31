import os
import sys
from setuptools import setup, find_packages

# Always disable Rust extensions for testing
HAS_SETUPTOOLS_RUST = False
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
            "feagi-zmq=feagi.zmq.server:main",
        ],
    },
) 