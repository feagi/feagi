"""
FEAGI Protocol Package

This package provides Protocol Buffer definitions and client interfaces
for FEAGI's communication protocols.
"""

# Version information
__version__ = "2.0.0"
__protocol_versions__ = {
    "handshake": 1,
    "fcp": 1,
    "fsmp": 1,
    "fvp": 1
}

# Import common definitions
from protocol.common.constants_pb2 import (
    ProtocolID,
    Timestamp
)

# Import handshake protocol
from protocol.handshake.v1.handshake_pb2 import (
    ProtocolVersion,
    HandshakeMessageType,
    HandshakeMessage,
    HelloMessage,
    WelcomeMessage,
    CapabilitiesMessage, 
    ConfigurationMessage,
    ReadyMessage,
    StartMessage,
    ErrorMessage
)

# Import FCP (FEAGI Control Protocol)
from protocol.fcp.v1.fcp_pb2 import (
    MessageType as FCPMessageType,
    Message as FCPMessage,
    RegisterConfirmMessage,
    DeregisterRequest,
    DeregisterResponse,
    StatusRequest, 
    StatusResponse,
    ConfigureRequest,
    ConfigureResponse,
    HeartbeatRequest,
    HeartbeatResponse,
    ErrorResponse as FCPErrorResponse
)

# Import FSMP (FEAGI Sensorimotor Protocol)
from protocol.fsmp.v1.fsmp_pb2 import (
    SensoryChannelType,
    MotorChannelType,
    MessageType as FSMPMessageType,
    Message as FSMPMessage,
    SensoryData,
    MotorData,
    StreamConfig
)

# Import FVP (FEAGI Visualization Protocol)
from protocol.fvp.v1.fvp_pb2 import (
    MessageType as FVPMessageType,
    Message as FVPMessage,
    StructureData,
    ActivityData,
    CorticalArea,
    Connection,
    ActivityGroup,
    ConfigMessage,
    ColorScheme,
    VisualizationHint
)

__all__ = [
    # Common
    'ProtocolID',
    'Timestamp',
    
    # Handshake
    'ProtocolVersion',
    'HandshakeMessageType',
    'HandshakeMessage',
    'HelloMessage',
    'WelcomeMessage',
    'CapabilitiesMessage',
    'ConfigurationMessage',
    'ReadyMessage',
    'StartMessage',
    'ErrorMessage',
    
    # FCP
    'FCPMessageType',
    'FCPMessage',
    'RegisterConfirmMessage',
    'DeregisterRequest',
    'DeregisterResponse',
    'StatusRequest',
    'StatusResponse',
    'ConfigureRequest',
    'ConfigureResponse',
    'HeartbeatRequest',
    'HeartbeatResponse',
    'FCPErrorResponse',
    
    # FSMP
    'FSMPMessageType',
    'SensoryChannelType',
    'MotorChannelType',
    'FSMPMessage',
    'SensoryData',
    'MotorData',
    'StreamConfig',
    
    # FVP
    'FVPMessageType',
    'FVPMessage',
    'StructureData',
    'ActivityData',
    'CorticalArea',
    'Connection',
    'ActivityGroup',
    'ConfigMessage',
    'ColorScheme',
    'VisualizationHint',
] 