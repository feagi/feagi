@0xd1ac0885e97defd9;  # Unique ID for this schema file

using Common = import "../../common/constants.capnp";

# FVP (FEAGI Visualization Protocol) message types
enum MessageType {
  unknown @0;
  structure @1;  # Brain structure data
  activity @2;   # Neural activity data
  config @3;     # Visualization configuration
}

# Definition of a cortical area
struct CorticalArea {
  id @0 :Text;
  name @1 :Text;
  
  # Spatial dimensions
  x @2 :UInt32;
  y @3 :UInt32;
  z @4 :UInt32;
  width @5 :UInt32;
  height @6 :UInt32;
  depth @7 :UInt32;
  
  # Properties
  properties @8 :List(Property);
  
  struct Property {
    key @0 :Text;
    value @1 :Text;
  }
}

# A connection between two cortical areas
struct Connection {
  sourceId @0 :Text;
  targetId @1 :Text;
  strength @2 :Float32;
  properties @3 :List(Property);
  
  struct Property {
    key @0 :Text;
    value @1 :Text;
  }
}

# Brain structure data
struct StructureData {
  # Structure information with cortical areas, neural groups, etc.
  
  # Map of cortical area IDs to their definitions
  corticalAreas @0 :List(CorticalAreaEntry);
  
  # Connections between cortical areas
  connections @1 :List(Connection);
  
  # Timestamp when this structure snapshot was taken
  timestamp @2 :Common.Timestamp;
  
  struct CorticalAreaEntry {
    id @0 :Text;
    area @1 :CorticalArea;
  }
}

# Activity data for a cortical area - optimized for efficient transmission
struct ActivityGroup {
  corticalAreaId @0 :Text;
  
  # Raw activity data
  data @1 :Data;
  
  # Encoding information
  encodingFormat @2 :Text;  # e.g., "binary", "float32", etc.
  
  # Optional compression information
  compression @3 :Text;      # e.g., "none", "zlib", etc.
}

# Visualization hint for special rendering
struct VisualizationHint {
  enum HintType {
    none @0;
    highlight @1;
    focus @2;
    alert @3;
    custom @4;
  }
  
  type @0 :HintType;
  targets @1 :List(Text);  # Cortical area IDs to apply hint to
  parameters @2 :List(Parameter);
  
  struct Parameter {
    key @0 :Text;
    value @1 :Text;
  }
}

# Neural activity data - optimized for high throughput
struct ActivityData {
  # Activity data by cortical area
  activity @0 :List(ActivityGroup);
  
  # Additional information about the activity frame
  frameId @1 :UInt64;
  
  # Timestamp when this activity snapshot was taken
  timestamp @2 :Common.Timestamp;
  
  # Optionally include a visualization hint
  hint @3 :VisualizationHint;
}

# Color scheme definition
struct ColorScheme {
  name @0 :Text;
  
  struct Color {
    r @0 :UInt8;
    g @1 :UInt8;
    b @2 :UInt8;
    a @3 :UInt8; # Alpha transparency
  }
  
  background @1 :Color;
  inactive @2 :Color;
  activeLow @3 :Color;
  activeMedium @4 :Color;
  activeHigh @5 :Color;
  
  # Additional colors by name
  namedColors @6 :List(NamedColor);
  
  struct NamedColor {
    name @0 :Text;
    color @1 :Color;
  }
}

# Visualization configuration message
struct ConfigMessage {
  # Visualization settings
  settings @0 :List(Setting);
  
  # Color scheme
  colorScheme @1 :ColorScheme;
  
  timestamp @2 :Common.Timestamp;
  
  struct Setting {
    key @0 :Text;
    value @1 :Text;
  }
}

# FVP message wrapper
struct Message {
  type @0 :MessageType;
  
  # Use union to represent different message types (only one will be present)
  union {
    structureData @1 :StructureData;
    activityData @2 :ActivityData;
    config @3 :ConfigMessage;
  }
} 