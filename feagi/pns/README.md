# FEAGI Peripheral Nervous System (PNS)

**Architecture**: Hybrid Python/Rust (Primary I/O in Rust, Orchestration in Python)

The Peripheral Nervous System provides the interface layer between FEAGI's neural processing and external agents/environments, enabling embodied cognition through sensory input and motor output.

---

## Architecture Overview

### Rust PNS (Primary I/O Layer)

**Location**: `feagi_core/feagi-rust/crates/feagi-pns/`

The Rust PNS handles all performance-critical I/O operations:

**Core Components**:
- **agent_registry.rs**: Tracks all registered agents and their state
- **registration.rs**: Processes agent registration/deregistration requests
- **heartbeat.rs**: Monitors agent health and deregisters stale agents
- **zmq/rest.rs**: ROUTER socket for agent registration and heartbeat (port 5563)
- **zmq/sensory.rs**: PULL socket for receiving binary XYZP sensory data (port 5558)
- **zmq/motor.rs**: PUB socket for broadcasting motor commands (port 30005)
- **zmq/visualization.rs**: PUB socket for neural activity visualization (port 5562)
- **shm.rs**: Shared memory I/O for high-performance local communication

**Key Benefits**:
- 10-100x faster binary data processing than Python
- Direct Rust-to-Rust paths (Agent → PNS → NPU) with no Python FFI overhead
- Consistent architecture with all I/O streams in one place
- RTOS-ready design for embedded systems migration

### Python PNS (Orchestration Layer)

**Location**: `feagi_core/feagi/pns/`

The Python layer provides orchestration and integration with FEAGI's subsystems:

**Core Component**:
- **registration_manager.py**: Orchestrates agent registration over Rust PyAgentRegistry

**Responsibilities**:
- **Agent Storage**: Delegates to Rust PyAgentRegistry (single source of truth)
- **FQ Sampler Coordination**: Automatically enables/disables Fire Queue samplers based on connected agent capabilities
- **Capability Rate Management**: Registers agent data rates with the capability rate manager
- **State Synchronization**: Updates global state manager with agent registry changes
- **Process Management Integration**: Coordinates FQ sampler lifecycle with process manager
- **Event Notification**: Notifies listeners of registration state changes
- **Validation & Error Handling**: Validates requests and provides clear error messages
- **Type Mapping**: Converts between Python and Rust data structures

---

## Data Flow Architecture

### Agent Registration Flow

1. Agent connects to Rust PNS REST stream (port 5563)
2. Rust registry stores agent data (PyAgentRegistry - single source of truth)
3. Python orchestration layer coordinates subsystems:
   - Enables/disables FQ samplers based on agent capabilities
   - Registers capability rates
   - Updates state manager
   - Notifies event listeners

### Sensory Data Flow (Agent → FEAGI)

**Pure Rust Path** (10-100x faster than Python):

1. Agent serializes sensory data to binary XYZP format using feagi-data-processing
2. Agent sends via ZMQ PUSH to port 5558
3. Rust PNS sensory stream receives (PULL socket)
4. Rust deserializes XYZP binary data
5. Rust injects directly to Rust NPU (no Python FFI!)

**Key Advantages**:
- Binary XYZP format for compact, fast serialization
- Pure Rust processing eliminates Python overhead
- Direct NPU injection bypasses slow Python FFI calls
- Significantly faster for high-frequency sensory data

### Motor Data Flow (FEAGI → Agent)

1. Rust NPU generates motor commands
2. Rust PNS motor stream broadcasts via ZMQ PUB (port 30005)
3. Agents receive motor commands and execute actions

### Visualization Data Flow (FEAGI → Brain Visualizer)

1. Rust NPU samples Fire Queue (neural activity)
2. Rust PNS visualization stream broadcasts via ZMQ PUB (port 5562)
3. Brain Visualizer receives and displays neural activity

---

## Agent Registration & Coordination System

### Automatic FQ Sampler Coordination

The registration system provides **zero-intervention** coordination between connected agents and FEAGI's Fire Queue samplers:

**Visualization FQ Sampler**:
- Automatically enabled when first visualization-capable agent connects
- Samples all cortical areas at configured rate (default 60Hz)
- Broadcasts to port 5562
- Automatically disabled when last visualization agent disconnects

**Motor FQ Sampler**:
- Automatically enabled when first motor-capable agent connects
- Samples OPU (Output Processing Unit) areas only
- Broadcasts to port 30005
- Automatically disabled when last motor agent disconnects

**Benefits**:
- **Resource Efficiency**: CPU/memory used only when agents need data
- **Zero Configuration**: No manual sampler management required
- **Automatic Cleanup**: Samplers disabled when agents disconnect
- **Capability-Driven**: Samplers respond to actual agent needs

### Capability Detection

The system detects agent capabilities during registration and automatically coordinates resources:

**Visualization Capability**: Triggers visualization FQ sampler enablement
- Used by: Brain visualizers, monitoring tools, debugging interfaces

**Motor Capability**: Triggers motor FQ sampler enablement
- Used by: Robotic controllers, virtual environment agents, actuator interfaces

**Sensory Capability**: Registers agent as data provider
- Used by: Camera agents, sensor arrays, environmental input systems

### Heartbeat Monitoring

The Rust heartbeat tracker monitors agent health:
- Agents send periodic heartbeats to maintain registration
- Stale agents (no heartbeat for timeout period) are automatically deregistered
- Automatic cleanup prevents resource leaks from crashed/disconnected agents
- Thread-safe with configurable timeout intervals

---

## ZMQ Communication Streams

### Stream Architecture

All ZMQ streams are managed by the Rust PNS for consistency and performance:

**REST Stream** (Port 5563):
- Socket Type: ROUTER (bi-directional request-reply)
- Purpose: Agent registration, deregistration, and heartbeat
- Format: JSON messages
- Direction: Bi-directional

**Sensory Stream** (Port 5558):
- Socket Type: PULL (agents use PUSH)
- Purpose: Receive sensory data from agents
- Format: Binary XYZP (feagi-data-processing crate)
- Direction: Agent → FEAGI
- Performance: Pure Rust, direct NPU injection

**Motor Stream** (Port 30005):
- Socket Type: PUB (agents use SUB)
- Purpose: Broadcast motor commands to agents
- Format: Binary motor command structures
- Direction: FEAGI → Agents

**Visualization Stream** (Port 5562):
- Socket Type: PUB (visualizers use SUB)
- Purpose: Broadcast neural activity for visualization
- Format: Binary neural activity data
- Direction: FEAGI → Visualizers

### Socket Configuration

All streams use optimized socket settings:
- Non-blocking I/O with configurable timeouts
- High water marks for buffer management
- Conflation for real-time data (keeps only latest message)
- Zero linger for clean shutdown
- Thread-safe operation

---

## XYZP Binary Format

### Purpose

XYZP (X, Y, Z, Potential) is FEAGI's binary format for efficiently transmitting neuron data:

**Components**:
- **X, Y, Z**: 3D spatial coordinates identifying neuron position in cortical area
- **P**: Membrane potential value (typically 0.0 to 100.0)

**Advantages**:
- Compact binary representation (much smaller than JSON)
- Fast serialization/deserialization in Rust
- Natural mapping to 3D cortical structure
- Supports multiple cortical areas in single message

### Data Structure

**CorticalMappedXYZPNeuronData**:
- Top-level structure mapping cortical area IDs to neuron data
- Each cortical area contains arrays of X, Y, Z, P values
- Serialized using feagi-data-processing Rust crate
- Deserialized by Rust PNS sensory stream

### Usage

Agents convert their sensor data to XYZP coordinates based on registered dimensions:
- Visual agents: Map pixels to XY grid (Z = color channel)
- Audio agents: Map frequency bands to positions
- Custom sensors: Map arbitrary data to 3D space

The Rust PNS deserializes XYZP and injects to the appropriate cortical areas in the NPU.

---

## Integration with FEAGI Subsystems

### Rust NPU Integration

**Direct Paths** (no Python):
- Sensory injection: Rust PNS → Rust NPU (inject_sensory_with_potentials)
- FQ sampling: Rust NPU → Rust PNS → ZMQ broadcast
- Motor commands: Rust NPU → Rust PNS → Agents

**Benefits**: Eliminates Python FFI overhead, 10-100x performance improvement

### State Manager Integration

The Python orchestration layer synchronizes agent registry state:
- Agent registration/deregistration updates state manager
- Provides global view of connected agents
- Enables REST API queries for agent status
- Maintains consistency across FEAGI subsystems

### Process Manager Integration

FQ sampler lifecycle coordination:
- Registration manager calls process manager to create/disable samplers
- Process manager manages sampler processes/threads
- Automatic coordination based on agent capabilities
- Clean shutdown of samplers when no longer needed

### Capability Rate Manager Integration

Per-agent rate limiting:
- Agents specify desired data rates during registration
- Rate manager enforces limits to prevent overload
- Approved rates enable optimal resource allocation
- Rejected rates logged for debugging

### BDU (Brain Development Unit) Integration

Future: Sensorimotor pathway creation:
- PNS will coordinate with BDU for dynamic cortical mapping
- Agent capabilities will drive synaptogenesis patterns
- Vision/motor areas created based on registered agents

---

## Configuration

All PNS configuration is centralized in `feagi_configuration.toml`:

**Agent Configuration**:
- registration_port: ZMQ REST stream port (default 5563)
- sensory_port: ZMQ sensory stream port (default 5558)
- motor_port: ZMQ motor stream port (default 30005)
- visualization_port: ZMQ visualization stream port (default 5562)
- heartbeat_timeout_ms: Agent heartbeat timeout (default 60000)

**Network Configuration**:
- All ports must be explicitly configured (no hardcoded defaults)
- Hosts configured via zmq_host setting
- Supports Docker, Kubernetes, cloud, and embedded deployments

**Architecture Compliance**:
- No hardcoded values permitted
- All configuration from TOML
- Fail-fast on missing configuration
- No fallback values (deterministic behavior)

---

## FEAGI 2.0 Architecture Principles

### Design Goals

1. **Cross-Platform**: Runs on Docker, Kubernetes, cloud, embedded, and desktop
2. **Rust-First I/O**: All performance-critical paths in Rust
3. **Python Orchestration**: Integration logic in Python until Rust migration complete
4. **No Fallbacks**: Fail-fast with clear errors, no hidden defaults
5. **Single Source of Truth**: Rust PyAgentRegistry for agent storage
6. **Deterministic**: Configuration-driven behavior, no runtime surprises

### Current State

**Rust Components** (Complete):
- ✅ Agent registry and storage
- ✅ All ZMQ streams (REST, sensory, motor, visualization)
- ✅ Heartbeat monitoring
- ✅ Binary data processing

**Python Components** (Orchestration):
- ✅ FQ sampler coordination
- ✅ Subsystem integration (state, process, capability managers)
- ✅ Request validation and error handling
- ✅ Event notification system

**Migration Status**: Primary I/O complete in Rust, orchestration remains in Python for subsystem compatibility

### Future Architecture

**Target State**: Full Rust PNS with minimal Python wrapper

**Migration Path**:
1. Move FQ sampler management to Rust
2. Move capability rate management to Rust
3. Move state manager integration to Rust
4. Reduce Python layer to thin REST API adapter

**Timeline**: Dependent on broader FEAGI subsystem Rust migration

---

## Development & Testing

### Running Tests

**Rust PNS Tests**:
```bash
cd feagi_core/feagi-rust/crates/feagi-pns
cargo test
```

**Python Integration Tests**:
```bash
cd feagi_core
pytest tests/pns/
pytest tests/system/test_registration_manager_integration.py
```

### Adding New Agents

To integrate a new agent type:

1. Implement agent using feagi-agent-sdk (Rust) or feagi-connector (Python)
2. Define capabilities during registration (sensory/motor/visualization)
3. Serialize sensory data to XYZP binary format
4. Connect to appropriate ZMQ streams:
   - Registration: tcp://host:5563
   - Sensory: tcp://host:5558 (PUSH socket)
   - Motor: tcp://host:30005 (SUB socket)
   - Visualization: tcp://host:5562 (SUB socket)
5. Send periodic heartbeats to maintain registration

The PNS will automatically coordinate resources based on agent capabilities.

### Debugging

**Common Issues**:

**Port conflicts**: Check `feagi_configuration.toml` and ensure ports are available
- Solution: Configure different ports or stop conflicting processes

**Stale agents**: Agents not sending heartbeats get auto-deregistered
- Solution: Ensure agent sends heartbeats within timeout period

**XYZP deserialization errors**: Binary format mismatch
- Solution: Use feagi-data-processing crate for serialization

**FQ samplers not enabling**: Capabilities not detected
- Solution: Verify capability format in registration request matches expected schema

---

## Deprecated Components

The following have been **removed** and migrated to Rust:

- **zmq_sensory_listener.py**: Replaced by feagi-pns/src/zmq/sensory.rs
- **zmq_registration_listener.py**: Replaced by feagi-pns/src/zmq/rest.rs
- **vision.py**: Dead code, completely unused

**Rationale**: All ZMQ I/O must be in Rust for performance, consistency, and architectural cleanliness.

---

## References

- **Rust PNS Source**: `feagi_core/feagi-rust/crates/feagi-pns/`
- **Python Orchestration**: `feagi_core/feagi/pns/registration_manager.py`
- **Configuration**: `feagi_core/feagi_configuration.toml`
- **Architecture Rules**: `.cursorrules` (project root)
- **Agent SDK**: `feagi_core/feagi-rust/crates/feagi-agent-sdk/`
- **Python Connector**: `feagi-connector/` (new connector with Rust backend)
