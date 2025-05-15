#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
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
# ==============================================================================

"""
FEAGI Protocol package for binary communication with agents.
"""

from feagi.api.protocols.base import ProtocolID, VersionedProtocol, ProtocolRegistry

# Import protocol modules and their registration functions
from feagi.api.protocols.fsmp import register_protocols as register_fsmp_protocols

def register_all_protocols():
    """Register all protocol versions with the registry."""
    register_fsmp_protocols()
    # Future registrations for other protocols:
    # register_fcp_protocols()
    # register_fvp_protocols()

# Run the registration on import
register_all_protocols() 