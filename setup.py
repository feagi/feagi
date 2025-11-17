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
Setup configuration for FEAGI.

NOTE: Rust extensions have been moved to the feagi-rust-py-libs package.
Install separately: pip install feagi-rust-py-libs
"""

from setuptools import setup

setup(
    zip_safe=False,
)

