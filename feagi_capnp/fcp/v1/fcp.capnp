@0xb4e51e1f8700c307;  # Unique ID for this schema file

using Common = import "../../common/constants.capnp";

# FCP (FEAGI Control Protocol) message types
enum MessageType {
  unknown @0;
  
  # Registration confirmation (after handshake)
  registerConfirm @1;
  
  # Deregistration
  deregister @2;
  deregisterResponse @3;
  
  # Status
  statusRequest @4;
  statusResponse @5;
  
  # Configuration
  configure @6;
  configureResponse @7;
  
  # Heartbeat
  heartbeat @8;
  heartbeatResponse @9;
  
  # Error
  error @10;
}

# Registration confirmation (after successful handshake)
struct RegisterConfirmMessage {
  status @0 :Text;  # "active" or similar
  message @1 :Text; # Optional confirmation message
  timestamp @2 :Common.Timestamp;
}

# Deregistration request
struct DeregisterRequest {
  agentId @0 :Text;
  timestamp @1 :Common.Timestamp;
}

# Deregistration response
struct DeregisterResponse {
  status @0 :Text;  # "success" or "error"
  message @1 :Text; # Optional message
  timestamp @2 :Common.Timestamp;
}

# Status request
struct StatusRequest {
  timestamp @0 :Common.Timestamp;
}

# Status response
struct StatusResponse {
  struct RuntimeStatus {
    cpuUsage @0 :Float32;
    memoryUsage @1 :Float32;
    uptimeSeconds @2 :UInt32;
  }
  
  status @0 :Text;  # "ok", "degraded", "error"
  runtime @1 :RuntimeStatus;
  agentCount @2 :UInt32;
  timestamp @3 :Common.Timestamp;
}

# Configure request
struct ConfigureRequest {
  config @0 :List(ConfigItem);
  timestamp @1 :Common.Timestamp;
  
  struct ConfigItem {
    key @0 :Text;
    value @1 :Text;
  }
}

# Configure response
struct ConfigureResponse {
  status @0 :Text;  # "success" or "error"
  message @1 :Text; # Optional message
  timestamp @2 :Common.Timestamp;
}

# Heartbeat request
struct HeartbeatRequest {
  agentId @0 :Text;
  timestamp @1 :Common.Timestamp;
}

# Heartbeat response
struct HeartbeatResponse {
  status @0 :Text;  # "ok" or "error"
  timestamp @1 :Common.Timestamp;
}

# Error response
struct ErrorResponse {
  code @0 :UInt32;
  message @1 :Text;
  timestamp @2 :Common.Timestamp;
}

# FCP Message wrapper - No protocol headers needed since ZMQ socket already identifies protocol
struct Message {
  type @0 :MessageType;
  
  # Only one will be present
  union {
    registerConfirm @1 :RegisterConfirmMessage;
    deregisterRequest @2 :DeregisterRequest;
    deregisterResponse @3 :DeregisterResponse;
    statusRequest @4 :StatusRequest;
    statusResponse @5 :StatusResponse;
    configureRequest @6 :ConfigureRequest;
    configureResponse @7 :ConfigureResponse;
    heartbeatRequest @8 :HeartbeatRequest;
    heartbeatResponse @9 :HeartbeatResponse;
    error @10 :ErrorResponse;
  }
} 