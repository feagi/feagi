@0xf91d3896f8c4dc9b;  # Unique ID for this schema file

using Common = import "../../common/constants.capnp";

# Protocol version record used in handshake
struct ProtocolVersion {
  fcpVersion @0 :UInt32;
  fsmpVersion @1 :UInt32;
  fvpVersion @2 :UInt32;
}

# Message type for handshake protocol
enum HandshakeMessageType {
  unknown @0;
  hello @1;             # Initial client greeting
  welcome @2;           # Server welcome response
  capabilities @3;      # Client capabilities
  configuration @4;     # Server configuration response
  ready @5;             # Client ready signal
  start @6;             # Server start signal
  error @7;             # Error condition
}

# Initial client greeting
struct HelloMessage {
  agentId @0 :Text;     # Unique identifier for this agent
  agentType @1 :Text;   # Type of agent (for categorization)
  timestamp @2 :Common.Timestamp;
}

# Server welcome response
struct WelcomeMessage {
  serverId @0 :Text;
  message @1 :Text;
  timestamp @2 :Common.Timestamp;
}

# Client capabilities
struct CapabilitiesMessage {
  # Protocol versions supported by client
  protocolVersions @0 :ProtocolVersion;
  
  # Sensory capabilities
  supportedSensoryChannels @1 :List(UInt32);
  
  # Motor capabilities
  supportedMotorChannels @2 :List(UInt32);
  
  # Features
  features @3 :List(Feature);
  
  timestamp @4 :Common.Timestamp;
  
  struct Feature {
    name @0 :Text;
    enabled @1 :Bool;
  }
}

# Server configuration
struct ConfigurationMessage {
  # Protocol versions chosen by server (based on client capabilities)
  protocolVersions @0 :ProtocolVersion;
  
  # Negotiated configuration
  config @1 :List(ConfigItem);
  
  timestamp @2 :Common.Timestamp;
  
  struct ConfigItem {
    key @0 :Text;
    value @1 :Text;
  }
}

# Client is ready
struct ReadyMessage {
  timestamp @0 :Common.Timestamp;
}

# Server says start
struct StartMessage {
  # Connection parameters
  connectionParams @0 :List(ConnectionParam);
  
  timestamp @1 :Common.Timestamp;
  
  struct ConnectionParam {
    key @0 :Text;
    value @1 :Text;
  }
}

# Error condition
struct ErrorMessage {
  errorCode @0 :UInt32;
  errorMessage @1 :Text;
  timestamp @2 :Common.Timestamp;
}

# Handshake message wrapper
struct HandshakeMessage {
  # Protocol identifier and version
  protocolId @0 :Common.ProtocolID;  # Always set to ProtocolID.FCP for handshake
  version @1 :UInt32;                # Handshake protocol version
  type @2 :HandshakeMessageType;
  
  # Payload options (only one will be present)
  union {
    hello @3 :HelloMessage;
    welcome @4 :WelcomeMessage;
    capabilities @5 :CapabilitiesMessage;
    configuration @6 :ConfigurationMessage;
    ready @7 :ReadyMessage;
    start @8 :StartMessage;
    error @9 :ErrorMessage;
  }
} 