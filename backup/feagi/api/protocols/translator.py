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
Protocol translator for Cap'n Proto schemas.

This module provides functionality for loading Cap'n Proto schemas,
encoding and decoding messages, and converting between Cap'n Proto
and internal data structures.
"""

import logging
import os
from typing import Any, Callable, Dict, Optional, Type

import capnp
from capnp import KjException
from capnp.lib import capnp as capnp_lib

# Configure logging
logger = logging.getLogger(__name__)


class ProtocolTranslator:
    """
    Translator for Cap'n Proto-based protocols.

    This class handles loading Cap'n Proto schemas and provides methods
    for encoding and decoding messages for different protocols.
    """

    def __init__(self, schema_path: Optional[str] = None):
        """
        Initialize the protocol translator.

        Args:
            schema_path: Path to the directory containing Cap'n Proto schemas
                         If None, will attempt to auto-detect based on the module path
        """
        # Detect schema path if not provided
        if not schema_path:
            # Look for feagi_capnp directory next to the feagi package
            package_dir = os.path.dirname(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            )
            schema_path = os.path.join(os.path.dirname(package_dir), "feagi_capnp")

            # If not found, try current directory + feagi_capnp
            if not os.path.exists(schema_path):
                schema_path = os.path.join(
                    os.path.dirname(os.path.abspath(__file__)),
                    "..",
                    "..",
                    "..",
                    "feagi_capnp",
                )

        self.schema_path = schema_path
        logger.info(f"Using Cap'n Proto schemas from: {self.schema_path}")

        # Load schemas
        try:
            self._load_schemas()
            logger.info("Cap'n Proto schemas loaded successfully")
        except (KjException, capnp_lib.KjException) as e:
            logger.error(f"Failed to load Cap'n Proto schemas: {e}")
            raise

        # Create schema loader functions
        self.schema_loaders = {
            "constants": self._load_constants,
            "handshake": self._load_handshake,
            "fcp": self._load_fcp,
            "fsmp": self._load_fsmp,
            "fvp": self._load_fvp,
        }

    def _load_schemas(self):
        """Load all Cap'n Proto schemas."""
        try:
            self.constants_schema = capnp.load(
                os.path.join(self.schema_path, "common", "constants.capnp")
            )
            self.handshake_schema = capnp.load(
                os.path.join(self.schema_path, "handshake", "v1", "handshake.capnp")
            )
            self.fcp_schema = capnp.load(
                os.path.join(self.schema_path, "fcp", "v1", "fcp.capnp")
            )
            self.fsmp_schema = capnp.load(
                os.path.join(self.schema_path, "fsmp", "v1", "fsmp.capnp")
            )
            self.fvp_schema = capnp.load(
                os.path.join(self.schema_path, "fvp", "v1", "fvp.capnp")
            )
        except Exception as e:
            logger.error(f"Error loading Cap'n Proto schemas: {e}")
            raise

    def _load_constants(self):
        """Load constants schema."""
        return self.constants_schema

    def _load_handshake(self):
        """Load handshake schema."""
        return self.handshake_schema

    def _load_fcp(self):
        """Load FCP schema."""
        return self.fcp_schema

    def _load_fsmp(self):
        """Load FSMP schema."""
        return self.fsmp_schema

    def _load_fvp(self):
        """Load FVP schema."""
        return self.fvp_schema

    def get_schema_loader(self, protocol_type: str) -> Callable:
        """
        Get a schema loader function for the specified protocol.

        Args:
            protocol_type: Protocol type ("constants", "handshake", "fcp", "fsmp", or "fvp")

        Returns:
            Function that returns the loaded schema
        """
        if protocol_type not in self.schema_loaders:
            raise ValueError(f"Unknown protocol type: {protocol_type}")

        return self.schema_loaders[protocol_type]

    def decode_message(self, message_data: bytes, protocol_type: str) -> Any:
        """
        Decode a Cap'n Proto message.

        Args:
            message_data: Raw message data
            protocol_type: Protocol type ("handshake", "fcp", "fsmp", or "fvp")

        Returns:
            Decoded Cap'n Proto message
        """
        try:
            if protocol_type == "handshake":
                return self.handshake_schema.HandshakeMessage.from_bytes(message_data)
            elif protocol_type == "fcp":
                return self.fcp_schema.FCPMessage.from_bytes(message_data)
            elif protocol_type == "fsmp":
                return self.fsmp_schema.FSMPMessage.from_bytes(message_data)
            elif protocol_type == "fvp":
                return self.fvp_schema.FVPMessage.from_bytes(message_data)
            else:
                raise ValueError(f"Unknown protocol type: {protocol_type}")
        except Exception as e:
            logger.error(f"Error decoding {protocol_type} message: {e}")
            raise

    def create_timestamp(self) -> Any:
        """
        Create a timestamp message.

        Returns:
            Cap'n Proto timestamp message
        """
        import time

        return self.constants_schema.Timestamp.new_message(
            timeMs=int(time.time() * 1000)
        )

    def create_handshake_hello(self, agent_id: str, agent_type: str) -> Any:
        """
        Create a handshake hello message.

        Args:
            agent_id: Agent identifier
            agent_type: Agent type

        Returns:
            Cap'n Proto handshake message
        """
        # Create timestamp
        current_time = self.create_timestamp()

        # Create hello message
        hello = self.handshake_schema.HelloMessage.new_message(
            agentId=agent_id, agentType=agent_type
        )
        hello.timestamp = current_time

        # Create handshake message
        handshake_msg = self.handshake_schema.HandshakeMessage.new_message(
            protocolId=self.constants_schema.ProtocolID.fcp,
            version=1,
            type=self.handshake_schema.HandshakeMessageType.hello,
        )
        handshake_msg.hello = hello

        return handshake_msg

    def create_handshake_welcome(
        self, server_id: str, message: str = "Welcome to FEAGI"
    ) -> Any:
        """
        Create a handshake welcome message.

        Args:
            server_id: Server identifier
            message: Welcome message

        Returns:
            Cap'n Proto handshake message
        """
        # Create timestamp
        current_time = self.create_timestamp()

        # Create welcome message
        welcome = self.handshake_schema.WelcomeMessage.new_message(
            serverId=server_id, message=message
        )
        welcome.timestamp = current_time

        # Create handshake message
        handshake_msg = self.handshake_schema.HandshakeMessage.new_message(
            protocolId=self.constants_schema.ProtocolID.fcp,
            version=1,
            type=self.handshake_schema.HandshakeMessageType.welcome,
        )
        handshake_msg.welcome = welcome

        return handshake_msg

    def create_handshake_capabilities(
        self,
        supported_sensory: list,
        supported_motor: list,
        protocol_versions: Dict[str, int],
    ) -> Any:
        """
        Create a handshake capabilities message.

        Args:
            supported_sensory: List of supported sensory channel IDs
            supported_motor: List of supported motor channel IDs
            protocol_versions: Dictionary mapping protocol names to version numbers

        Returns:
            Cap'n Proto handshake message
        """
        # Create timestamp
        current_time = self.create_timestamp()

        # Create protocol versions
        versions = self.handshake_schema.ProtocolVersion.new_message(
            fcpVersion=protocol_versions.get("fcp", 1),
            fsmpVersion=protocol_versions.get("fsmp", 1),
            fvpVersion=protocol_versions.get("fvp", 1),
        )

        # Create capabilities message
        capabilities = self.handshake_schema.CapabilitiesMessage.new_message(
            supportedSensoryChannels=supported_sensory,
            supportedMotorChannels=supported_motor,
        )
        capabilities.protocolVersions = versions
        capabilities.timestamp = current_time

        # Create handshake message
        handshake_msg = self.handshake_schema.HandshakeMessage.new_message(
            protocolId=self.constants_schema.ProtocolID.fcp,
            version=1,
            type=self.handshake_schema.HandshakeMessageType.capabilities,
        )
        handshake_msg.capabilities = capabilities

        return handshake_msg

    def create_handshake_configuration(self, server_config: Dict[str, Any]) -> Any:
        """
        Create a handshake configuration message.

        Args:
            server_config: Server configuration dictionary

        Returns:
            Cap'n Proto handshake message
        """
        # Create timestamp
        current_time = self.create_timestamp()

        # Create configuration message
        config = self.handshake_schema.ConfigurationMessage.new_message()
        config.timestamp = current_time

        # TODO: Add configuration parameters based on server_config

        # Create handshake message
        handshake_msg = self.handshake_schema.HandshakeMessage.new_message(
            protocolId=self.constants_schema.ProtocolID.fcp,
            version=1,
            type=self.handshake_schema.HandshakeMessageType.configuration,
        )
        handshake_msg.configuration = config

        return handshake_msg

    def handshake_message_to_dict(self, message: Any) -> Dict[str, Any]:
        """
        Convert a handshake message to a dictionary.

        Args:
            message: Cap'n Proto handshake message

        Returns:
            Dictionary representation of the message
        """
        result = {
            "protocol_id": str(message.protocolId),
            "version": message.version,
            "type": str(message.type),
        }

        # Extract message-type-specific data
        if message.type == self.handshake_schema.HandshakeMessageType.hello:
            result["hello"] = {
                "agent_id": message.hello.agentId,
                "agent_type": message.hello.agentType,
                "timestamp": message.hello.timestamp.timeMs,
            }
        elif message.type == self.handshake_schema.HandshakeMessageType.welcome:
            result["welcome"] = {
                "server_id": message.welcome.serverId,
                "message": message.welcome.message,
                "timestamp": message.welcome.timestamp.timeMs,
            }
        elif message.type == self.handshake_schema.HandshakeMessageType.capabilities:
            result["capabilities"] = {
                "sensory_channels": list(message.capabilities.supportedSensoryChannels),
                "motor_channels": list(message.capabilities.supportedMotorChannels),
                "protocol_versions": {
                    "fcp": message.capabilities.protocolVersions.fcpVersion,
                    "fsmp": message.capabilities.protocolVersions.fsmpVersion,
                    "fvp": message.capabilities.protocolVersions.fvpVersion,
                },
                "timestamp": message.capabilities.timestamp.timeMs,
            }

            # Extract features
            features = []
            for i in range(len(message.capabilities.features)):
                feature = message.capabilities.features[i]
                features.append({"name": feature.name, "enabled": feature.enabled})
            result["capabilities"]["features"] = features

        elif message.type == self.handshake_schema.HandshakeMessageType.configuration:
            result["configuration"] = {
                "timestamp": message.configuration.timestamp.timeMs
                # Add more configuration fields as needed
            }

        return result
