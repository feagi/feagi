@0xd2a2e5ec280a1e60;  # Unique ID for this schema file

# Protocol identifiers
enum ProtocolID {
  unknown @0;
  fcp @1;     # FEAGI Control Protocol
  fvp @2;     # FEAGI Visualization Protocol
  fsmp @3;    # FEAGI Sensorimotor Protocol
}

# Timestamp used in various messages
struct Timestamp {
  timeMs @0 :UInt64;  # Milliseconds since epoch
} 