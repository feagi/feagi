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

from setuptools import find_packages, setup

# Always disable Rust extensions for testing
HAS_SETUPTOOLS_RUST = False
rust_extensions = []
print("Warning: setuptools-rust is not available. Rust extensions will not be built.")

setup(
    name="feagi",
    version="0.1.0",
    description="Framework for Evolutionary Artificial General Intelligence",
    long_description="FEAGI: Framework for Evolutionary Artificial General Intelligence. A comprehensive platform for creating, training, and deploying artificial general intelligence models that evolve over time.",
    long_description_content_type="text/plain",
    author="Neuraville Inc.",
    author_email="feagi@neuraville.com",
    url="https://github.com/feagi/feagi",
    license="Apache 2.0",
    packages=find_packages(),
    rust_extensions=rust_extensions,
    zip_safe=False,  # Required for Rust extensions
    include_package_data=True,
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Programming Language :: Rust",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    python_requires=">=3.8",
    install_requires=[
        "pydantic",
        "toml",
        "numpy",
        "zmq",
        "capnp",
        "toml",
        "feagi-data-processing>=0.0.30",  # Replace feagi_bytes with high-performance feagi-data-processing
        "tomli",
        "tomli-w",
        "pathlib",
        "asyncio",
        "uvloop;platform_system!='Windows'",
        "pydantic-settings",
        "influxdb-client",
        "tqdm",
        "pytest",
        "pytest-asyncio",
        "pytest-timeout",
        "pytest-cov",
        "loguru",
        "msgpack",
        "websockets",
        "pynvml",
        "psutil",
        "fastapi",
        "httpx",
        "uvicorn",
        "aiofiles",
        "websockets",
    ],
    entry_points={
        "console_scripts": [
            "feagi-api=feagi.api.server:main",
            "feagi-zmq=feagi.zmq.server:main",
            "feagi=feagi.main:main",
        ],
    },
)
