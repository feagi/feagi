"""
FEAGI Cap'n Proto Protocol Definitions

This package provides Cap'n Proto schema definitions and client interfaces
for FEAGI's communication protocols.
"""

import os
import capnp

# Path to schema files
_SCHEMA_PATH = os.path.dirname(os.path.abspath(__file__))

# Load the schemas
_common_constants = capnp.load(os.path.join(_SCHEMA_PATH, 'common/constants.capnp'))
_handshake = capnp.load(os.path.join(_SCHEMA_PATH, 'handshake/v1/handshake.capnp'))
_fcp = capnp.load(os.path.join(_SCHEMA_PATH, 'fcp/v1/fcp.capnp'))
_fsmp = capnp.load(os.path.join(_SCHEMA_PATH, 'fsmp/v1/fsmp.capnp'))
_fvp = capnp.load(os.path.join(_SCHEMA_PATH, 'fvp/v1/fvp.capnp'))

# Version information
__version__ = "2.0.0"
__protocol_versions__ = {
    "handshake": 1,
    "fcp": 1,
    "fsmp": 1,
    "fvp": 1
}

# Export common
ProtocolID = _common_constants.ProtocolID
Timestamp = _common_constants.Timestamp

# Export handshake
HandshakeMessageType = _handshake.HandshakeMessageType
HandshakeMessage = _handshake.HandshakeMessage
ProtocolVersion = _handshake.ProtocolVersion
HelloMessage = _handshake.HelloMessage
WelcomeMessage = _handshake.WelcomeMessage
CapabilitiesMessage = _handshake.CapabilitiesMessage
ConfigurationMessage = _handshake.ConfigurationMessage
ReadyMessage = _handshake.ReadyMessage
StartMessage = _handshake.StartMessage
ErrorMessage = _handshake.ErrorMessage

# Export FCP
FCPMessageType = _fcp.MessageType
FCPMessage = _fcp.Message
RegisterConfirmMessage = _fcp.RegisterConfirmMessage
DeregisterRequest = _fcp.DeregisterRequest
DeregisterResponse = _fcp.DeregisterResponse
StatusRequest = _fcp.StatusRequest
StatusResponse = _fcp.StatusResponse
ConfigureRequest = _fcp.ConfigureRequest
ConfigureResponse = _fcp.ConfigureResponse
HeartbeatRequest = _fcp.HeartbeatRequest
HeartbeatResponse = _fcp.HeartbeatResponse
FCPErrorResponse = _fcp.ErrorResponse

# Export FSMP
SensoryChannelType = _fsmp.SensoryChannelType
MotorChannelType = _fsmp.MotorChannelType
FSMPMessageType = _fsmp.MessageType
FSMPMessage = _fsmp.Message
SensoryData = _fsmp.SensoryData
MotorData = _fsmp.MotorData
StreamConfig = _fsmp.StreamConfig

# Export FVP
FVPMessageType = _fvp.MessageType
FVPMessage = _fvp.Message
StructureData = _fvp.StructureData
ActivityData = _fvp.ActivityData
CorticalArea = _fvp.CorticalArea
Connection = _fvp.Connection
ActivityGroup = _fvp.ActivityGroup
ConfigMessage = _fvp.ConfigMessage
ColorScheme = _fvp.ColorScheme
VisualizationHint = _fvp.VisualizationHint

__all__ = [
    # Version info
    '__version__',
    '__protocol_versions__',
    
    # Common
    'ProtocolID',
    'Timestamp',
    
    # Handshake
    'HandshakeMessageType',
    'HandshakeMessage',
    'ProtocolVersion',
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