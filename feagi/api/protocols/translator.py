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
Protocol Translator for FEAGI binary protocols.

This module provides translation services between binary protocol messages
and FEAGI's internal data structures.
"""

import datetime
import time
from typing import Dict, Any, Optional, Tuple, List, Union

from feagi.utils.logger import setup_logger
from feagi.api.protocols.base import ProtocolManager, ProtocolID, ProtocolRegistry

logger = setup_logger()


class ProtocolTranslator:
    """
    Protocol translator for FEAGI binary protocols.
    
    This class handles the translation between binary protocol messages and 
    FEAGI's internal data structures. It manages protocol versioning and
    negotiates compatible versions during connection setup.
    """
    
    def __init__(self):
        """Initialize protocol translator."""
        self.protocol_manager = ProtocolManager()
        self.agent_protocols: Dict[str, Dict[str, int]] = {}  # agent_id -> {protocol_name -> version}
    
    def register_agent(self, agent_id: str, protocol_versions: Dict[str, Union[int, List[int]]]) -> Dict[str, int]:
        """
        Register an agent with the translator and negotiate compatible protocol versions.
        
        Args:
            agent_id: Unique agent identifier
            protocol_versions: Dictionary mapping protocol names to supported version numbers
                               (either a single int or a list of supported versions)
        
        Returns:
            Dictionary mapping protocol names to negotiated version numbers
            
        Raises:
            ValueError: If no compatible protocol versions are found
        """
        compatible_versions: Dict[str, int] = {}
        
        for protocol_name, versions in protocol_versions.items():
            try:
                # Convert protocol name to ID
                protocol_id = ProtocolID[protocol_name]
                
                # Convert single version to list if needed
                client_versions = [versions] if isinstance(versions, int) else versions
                
                # Find highest compatible version
                compatible_version = self.protocol_manager.registry.get_compatible_version(
                    protocol_id, client_versions
                )
                
                if compatible_version is not None:
                    compatible_versions[protocol_name] = compatible_version
                else:
                    logger.warning(f"No compatible version found for protocol {protocol_name}")
                    
            except (KeyError, ValueError) as e:
                logger.warning(f"Unsupported protocol {protocol_name}: {str(e)}")
        
        if not compatible_versions:
            raise ValueError("No compatible protocol versions found")
            
        # Store negotiated protocols for this agent
        self.agent_protocols[agent_id] = compatible_versions
        
        return compatible_versions
    
    def deregister_agent(self, agent_id: str) -> bool:
        """
        Deregister an agent from the translator.
        
        Args:
            agent_id: Agent identifier
            
        Returns:
            True if agent was deregistered, False if agent was not registered
        """
        if agent_id in self.agent_protocols:
            del self.agent_protocols[agent_id]
            return True
        return False
    
    def encode(self, agent_id: str, data: Any, protocol_name: str) -> bytes:
        """
        Encode internal data to binary protocol format.
        
        Args:
            agent_id: Agent identifier
            data: Internal data to encode
            protocol_name: Protocol to use for encoding
            
        Returns:
            Binary protocol message
            
        Raises:
            ValueError: If agent is not registered or protocol is not supported
        """
        if agent_id not in self.agent_protocols:
            raise ValueError(f"Agent {agent_id} is not registered")
            
        if protocol_name not in self.agent_protocols[agent_id]:
            raise ValueError(f"Protocol {protocol_name} not negotiated for agent {agent_id}")
            
        try:
            protocol_id = ProtocolID[protocol_name]
            version = self.agent_protocols[agent_id][protocol_name]
            
            # Preprocess data if needed (e.g., add timestamps)
            processed_data = self._preprocess_outgoing_data(data, protocol_id)
            
            # Encode using protocol manager
            return self.protocol_manager.encode_message(processed_data, protocol_id, version)
            
        except Exception as e:
            logger.error(f"Error encoding message for protocol {protocol_name}: {str(e)}")
            raise
    
    def decode(self, binary_data: bytes) -> Tuple[Any, ProtocolID, int]:
        """
        Decode binary protocol message to internal data.
        
        Args:
            binary_data: Binary protocol message
            
        Returns:
            Tuple of (decoded data, protocol ID, version)
            
        Raises:
            ValueError: If message format is invalid or unsupported
        """
        try:
            # Decode using protocol manager
            decoded_data, protocol_id, version = self.protocol_manager.decode_message(binary_data)
            
            # Postprocess data if needed
            processed_data = self._postprocess_incoming_data(decoded_data, protocol_id)
            
            return processed_data, protocol_id, version
            
        except Exception as e:
            logger.error(f"Error decoding message: {str(e)}")
            raise
    
    def get_supported_protocols(self) -> Dict[str, List[int]]:
        """
        Get all supported protocols and their versions.
        
        Returns:
            Dictionary mapping protocol names to lists of supported versions
        """
        return self.protocol_manager.registry.list_protocols()
    
    def get_agent_protocols(self, agent_id: str) -> Optional[Dict[str, int]]:
        """
        Get negotiated protocol versions for an agent.
        
        Args:
            agent_id: Agent identifier
            
        Returns:
            Dictionary mapping protocol names to negotiated versions,
            or None if agent is not registered
        """
        return self.agent_protocols.get(agent_id)
    
    def _preprocess_outgoing_data(self, data: Any, protocol_id: ProtocolID) -> Any:
        """
        Preprocess data before encoding.
        
        Args:
            data: Internal data to preprocess
            protocol_id: Protocol identifier
            
        Returns:
            Preprocessed data
        """
        # Handle protocol-specific preprocessing
        if protocol_id == ProtocolID.FCP and isinstance(data, dict):
            # Add timestamp to FCP messages if not present
            if "payload" in data and isinstance(data["payload"], dict):
                if data["payload"].get("timestamp") is None:
                    data["payload"]["timestamp"] = datetime.datetime.utcnow().isoformat()
        
        return data
    
    def _postprocess_incoming_data(self, data: Any, protocol_id: ProtocolID) -> Any:
        """
        Postprocess data after decoding.
        
        Args:
            data: Decoded data to postprocess
            protocol_id: Protocol identifier
            
        Returns:
            Postprocessed data
        """
        # Handle protocol-specific postprocessing
        # For now, just return the data as-is
        return data 