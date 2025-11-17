"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""Module for neuron-related constants.

ARCHITECTURE NOTE:
All neuron data is stored in Rust NPU (Structure-of-Arrays layout).
Python never stores neuron data - only metadata and configuration.

Legacy interfaces removed - neurons follow same architecture as synapses:
- No Python interface layer (was redundant)
- No Python tracking dictionaries (neuron_id == array_index always)
- Direct access to Rust NPU for all neuron operations
"""

# MEMORY OPTIMIZATION: Invalid cortical area index for uint16 optimization
INVALID_CORTICAL_IDX = 65535  # Max value for uint16, used instead of -1
