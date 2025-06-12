# FEAGI Processes Architecture Diagram

## Overview

This document contains the comprehensive architecture diagram showing all major FEAGI processes and their relationships. The diagram includes the critical State Management Layer and Connectome Management components that form the core of FEAGI's neural processing capabilities.

## Files

- **Raw Mermaid File**: `feagi-processes-architecture.mmd` - Contains the complete Mermaid diagram source
- **JPEG Generation**: Instructions below for creating the visual diagram

## Diagram Components

### **External Layer**
- **Brain Visualizer**: Godot-based 3D visualization client for neural activity
- **Agents**: External applications and simulators connecting to FEAGI
- **Simulators**: Gazebo and custom simulation environments

### **FEAGI Bridge**
- **Brain Visualizer Plugin**: Handles visualization-specific communication protocols
- **ZMQ Client**: Manages ZeroMQ protocol communication with FEAGI core
- **WebSocket Server**: Real-time bidirectional communication with brain visualizer
- **REST Client**: HTTP-based communication for configuration and control

### **State Management Layer** ⭐ **CRITICAL**
- **FeagiStateManager**: Singleton managing all system states across FEAGI
- **State File**: Binary storage for persistent state information
- **Service States**: Tracks status of BurstEngine and other core services

### **Connectome Management Layer** ⭐ **CRITICAL**
- **ConnectomeManager**: Singleton managing neural connectivity and structure
- **ArrayBackend**: PyTorch/CPU backend for efficient neural array operations
- **NeuronArray**: 10 million neuron capacity array structure
- **Connectivity**: Structure of Arrays (SoA) for neural connections

### **Process Management**
- **Process Manager**: Central orchestrator with 3-tier priority system
  - **Priority 1 (Critical)**: Burst Engine, State Manager, Connectome Manager
  - **Priority 2 (Important)**: ZMQ Server, FQ Samplers
  - **Priority 3 (Optional)**: Health Monitor, Log Manager

### **Core Services**
- **Burst Engine**: 10Hz neural processing engine driving brain simulation
- **Genome Service**: Manages brain configuration and neural topology
- **Brain Service**: Coordinates between services and manages lifecycle
- **Core API Service**: Provides REST endpoints for system control

### **Neural Processing Unit (NPU)**
- **NPU Core**: Central neural computation engine
- **Injection Service**: Manages power area stimulation and neural activation
- **Motor FQ Sampler**: 100Hz sampling for Output Processing Units (OPU)
- **Visualization FQ Sampler**: 30Hz sampling for brain visualization

### **Communication Layers**
- **ZMQ Server**: Multi-port ZeroMQ message broker
  - REQ/REP Pattern (Port 5555)
  - PUB/SUB Pattern (Port 5556)
  - PUSH/PULL Pattern (Port 5557)
- **Data Streams**: Specialized processing streams
  - Sensory Neural Stream (Port 5558)
  - Motor Stream (Port 5564)
  - Visualization Stream (Port 5562)
  - REST Stream (Port 5563)

### **Agent Management**
- **Agent Registry**: Tracks connected visualization and motor agents
- **Agent API**: Handles registration, deregistration, and health monitoring
- **FQ Sampler Control**: Dynamically enables/disables samplers based on agent presence

### **REST API Layer**
- **FastAPI Server**: Main HTTP server (Port 8000)
- **v1 Endpoints**: Versioned API for system control and monitoring
- **Health Check**: System status and diagnostic endpoints

### **Backend Services**
- **Resource Manager**: System resource monitoring and allocation
- **Health Monitor**: Continuous system health surveillance
- **Log Manager**: Centralized logging and debugging support

## Key Architectural Principles

1. **Singleton Pattern**: Critical components (StateManager, ConnectomeManager) use singleton pattern for consistency
2. **Priority-Based Startup**: Process Manager ensures critical components start first
3. **Event-Driven**: FQ Samplers respond to agent connection/disconnection events
4. **Modular Design**: Clear separation between neural processing, communication, and management layers
5. **Scalable Communication**: ZMQ patterns support high-throughput, low-latency messaging

## Data Flow

1. **External Input**: Agents/simulators send sensory data via ZMQ or REST
2. **Neural Processing**: Burst Engine processes neural computation every 100ms
3. **State Management**: All state changes tracked by FeagiStateManager
4. **Output Generation**: Motor and visualization data sampled by FQ Samplers
5. **Client Delivery**: Processed data sent to connected agents via appropriate streams

## Generating JPEG Version

To create a JPEG version of this diagram, you can use one of these methods:

### Method 1: Mermaid CLI
```bash
# Install Mermaid CLI if not already installed
npm install -g @mermaid-js/mermaid-cli

# Generate JPEG from the Mermaid file
mmdc -i feagi-processes-architecture.mmd -o feagi-processes-architecture.jpeg -t neutral -b white
```

### Method 2: Online Mermaid Editor
1. Go to https://mermaid.live/
2. Copy the contents of `feagi-processes-architecture.mmd`
3. Paste into the editor
4. Click "Actions" → "Export as JPEG"
5. Save as `feagi-processes-architecture.jpeg`

### Method 3: Mermaid VS Code Extension
1. Install "Mermaid Preview" extension in VS Code
2. Open `feagi-processes-architecture.mmd`
3. Use Command Palette: "Mermaid Preview: Export Current Diagram"
4. Choose JPEG format

## Usage in Documentation

This diagram serves as the definitive reference for:
- **System Architecture Reviews**: Understanding component relationships
- **Debugging Sessions**: Tracing data flow and identifying bottlenecks
- **Development Planning**: Adding new features and components
- **Onboarding**: Helping new developers understand FEAGI's structure

## Related Documentation

- `arch-system-overview.md` - High-level system architecture
- `arch-state-management.md` - Detailed state management design
- `arch-zmq.md` - ZMQ communication patterns
- `arch-burst-engine-lifecycle.md` - Burst engine operational details

---

**Last Updated**: 2025-06-08
**Diagram Version**: 1.0
**Components**: 47 major processes and subsystems
