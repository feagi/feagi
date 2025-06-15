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
Synapse rules module.

This module provides concrete implementations of synapse formation rules
for different connection patterns between cortical areas.
"""

from .distance_based import DistanceBasedRule
from .one_to_one import OneToOneRule
from .random import RandomRule

__all__ = [
    "OneToOneRule",
    "RandomRule",
    "DistanceBasedRule",
]
