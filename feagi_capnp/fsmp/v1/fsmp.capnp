@0xd8b315f892c38cd3;  # Unique ID for this schema file

using Common = import "../../common/constants.capnp";

# Default sensory channel definitions
enum SensoryChannelType {
  unknownSensory @0;
  vision @1;
  audio @2;
  proprioception @3;
  tactile @4;
  # Add more as needed
}

# Default motor channel definitions
enum MotorChannelType {
  unknownMotor @0;
  movement @1;
  speech @2;
  manipulation @3;
  # Add more as needed
}

# FSMP message types
enum MessageType {
  unknown @0;
  sensory @1;
  motor @2;
}

# Sensory data message - optimized for high throughput
struct SensoryData {
  # Channel ID can be from standard channels or custom channels
  channelId @0 :UInt32;
  
  # Raw binary data (e.g., image bytes, audio samples)
  data @1 :Data;
  
  # Properties provides context for interpreting the data (optional)
  properties @2 :List(Property);
  
  # Timestamp of when the sensory data was captured
  timestamp @3 :Common.Timestamp;
  
  struct Property {
    key @0 :Text;
    value @1 :Text;
  }
}

# Motor data message - optimized for high throughput
struct MotorData {
  # Channel ID can be from standard channels or custom channels
  channelId @0 :UInt32;
  
  # Raw binary data representing motor commands
  data @1 :Data;
  
  # Properties provides context for interpreting the data (optional)
  properties @2 :List(Property);
  
  # Timestamp of when the motor command was generated
  timestamp @3 :Common.Timestamp;
  
  struct Property {
    key @0 :Text;
    value @1 :Text;
  }
}

# FSMP (FEAGI Sensorimotor Protocol) message wrapper
struct Message {
  type @0 :MessageType;
  
  # Use union to represent different message types (only one will be present)
  union {
    sensoryData @1 :SensoryData;
    motorData @2 :MotorData;
  }
}

# Stream configuration (used during handshake)
struct StreamConfig {
  # Available sensory channels
  sensoryChannels @0 :List(UInt32);
  
  # Available motor channels
  motorChannels @1 :List(UInt32);
  
  # Channel properties (e.g. image dimensions, data format)
  channelProperties @2 :List(ChannelProperties);
  
  struct ChannelProperties {
    channelId @0 :UInt32;
    properties @1 :List(Property);
    
    struct Property {
      key @0 :Text;
      value @1 :Text;
    }
  }
} 