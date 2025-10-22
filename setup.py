# Copyright 2025 Neuraville Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Setup configuration for FEAGI with Rust extensions.

This file configures setuptools-rust to compile the Rust components
during package installation. Pre-built binary wheels are provided for
common platforms via CI/CD, so most users won't need Rust installed.
"""

from setuptools import setup
from setuptools_rust import Binding, RustExtension

setup(
    rust_extensions=[
        RustExtension(
            "feagi.feagi_rust",
            path="feagi-rust/crates/feagi-python/Cargo.toml",
            binding=Binding.PyO3,
            debug=False,
            features=["extension-module"],
        )
    ],
    # Ensure Rust extensions are built before Python package
    zip_safe=False,
)

