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
FEAGI Control Protocol (FCP) module.

This module provides access to all FCP versions and helper functions.
"""

import uuid
from typing import Dict, Any

# Import common FCP components
from feagi.api.protocols.fcp.common import FCPCommandType, FCPMessageFormat

# Import all versions
from feagi.api.protocols.fcp.v1 import FCPv1

# Re-export helper functions

def create_register_message(agent_id: str, agent_type: str, capabilities: Dict[str, Any], 
                           version: str = "1.0") -> Dict[str, Any]:
    """
    Create an agent registration message.
    
    Args:
        agent_id: Unique agent identifier
        agent_type: Agent type (e.g., "monitor", "robot", etc.)
        capabilities: Dictionary of agent capabilities
        version: Agent version
        
    Returns:
        FCP message data
    """
    return {
        "command_type": FCPCommandType.REGISTER,
        "payload": {
            "agent_id": agent_id,
            "agent_type": agent_type,
            "capabilities": capabilities,
            "version": version,
            "timestamp": None,  # Will be filled in by the protocol translator
            "message_id": str(uuid.uuid4())
        }
    }


def create_heartbeat_message(agent_id: str, status: Dict[str, Any]) -> Dict[str, Any]:
    """
    Create a heartbeat message.
    
    Args:
        agent_id: Agent identifier
        status: Status information
        
    Returns:
        FCP message data
    """
    return {
        "command_type": FCPCommandType.HEARTBEAT,
        "payload": {
            "agent_id": agent_id,
            "status": status,
            "timestamp": None,  # Will be filled in by the protocol translator
            "message_id": str(uuid.uuid4())
        }
    }


def register_protocols():
    """Register FCP protocol versions with the registry."""
    from feagi.api.protocols.base import ProtocolRegistry
    registry = ProtocolRegistry()
    registry.register(FCPv1)


# Define all exported symbols
__all__ = [
    'FCPCommandType',
    'FCPMessageFormat',
    'FCPv1',
    'create_register_message',
    'create_heartbeat_message',
    'register_protocols'
] 