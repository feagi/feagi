# FEAGI Agent SDK - Architecture

This document describes the architecture of the FEAGI Agent SDK system, including the Rust core SDK, Python bindings, and how they integrate with FEAGI.

---

## 🏗️ **System Overview**

The FEAGI Agent SDK provides a production-ready, cross-platform client library for building agents that connect to FEAGI. The SDK is built in Rust for performance and reliability, with language bindings (starting with Python) that wrap the core functionality.

```
┌────────────────────────────────────────────────────┐
│              Agent Applications                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────┐ │
│  │ Pure Rust    │  │ Python       │  │ Future:  │ │
│  │ Agents       │  │ Agents       │  │ JS/C++   │ │
│  └──────┬───────┘  └──────┬───────┘  └────┬─────┘ │
└─────────┼──────────────────┼────────────────┼───────┘
          │                  │                │
          ↓                  ↓                ↓
┌─────────────────┐  ┌──────────────────┐  ┌────────┐
│ feagi-agent-sdk │  │ feagi-agent-     │  │ ...    │
│ (Rust)          │  │ sdk-py (PyO3)    │  │        │
│ - Core logic    │  │ - Python wrapper │  │        │
│ - Registration  │←─┤ - Type conv      │  │        │
│ - Heartbeat     │  │ - Exceptions     │  │        │
│ - Reconnection  │  └──────────────────┘  │        │
│ - ZMQ I/O       │                        │        │
└─────────┬───────┘                        │        │
          │                                │        │
          ↓                                ↓        ↓
┌─────────────────────────────────────────────────────┐
│         feagi-agent-registry (Rust)                 │
│         - Transport-agnostic core                   │
│         - Agent lifecycle management                │
└─────────────────────────────────────────────────────┘
          ↑                    ↑
          │                    │
   ┌──────┴────────┐    ┌─────┴──────┐
   │ Python FEAGI  │    │ Rust       │
   │ (via PyO3)    │    │ Engine     │
   └───────────────┘    └────────────┘
```

---

## 📦 **Component Architecture**

### **1. feagi-agent-registry (Rust Crate)**

**Purpose:** Server-side agent management (transport-agnostic)

**Responsibilities:**
- Agent registration/deregistration
- Agent lifecycle tracking
- Capability validation
- Activity monitoring
- Timeout-based pruning

**Key Types:**
- `AgentRegistry` - Core registry managing agent state
- `AgentInfo` - Agent metadata and capabilities
- `AgentType` - Enum: Sensory, Motor, Both
- `AgentTransport` - Trait for different communication protocols

**Usage:**
- Used by Python FEAGI (via PyO3 bindings)
- Used by Rust inference engine directly
- **Never run both simultaneously** (mutually exclusive deployment modes)

---

### **2. feagi-agent-sdk (Rust Crate)** ⭐ **CLIENT SDK**

**Purpose:** Production-ready client library for building agents

**Responsibilities:**
- Agent connection management
- Automatic registration with retry logic
- Background heartbeat service
- Reconnection with exponential backoff
- ZMQ communication (PUSH for sensory, SUB for motor)
- Graceful shutdown and deregistration

**Architecture:**

```
AgentClient
  ├── AgentConfig         (configuration builder)
  ├── RegistrationSocket  (ZMQ REQ for registration/heartbeat)
  ├── SensorySocket       (ZMQ PUSH for sensory data)
  ├── MotorSocket         (ZMQ SUB for motor commands)
  └── HeartbeatService    (background thread)
      └── ReconnectionStrategy (exponential backoff)
```

**Key Components:**

#### **AgentConfig (config.rs)**
- Builder pattern for configuration
- Type-safe capability definitions
- Validation before use
- Cloneable for multiple agents

#### **AgentClient (client.rs)**
- Main interface for agents
- Thread-safe (`Arc<Mutex<>>` internally)
- Auto-deregistration on drop
- Non-blocking motor data receive

#### **HeartbeatService (heartbeat.rs)**
- Runs in dedicated background thread (daemon)
- Configurable interval (default: 5 seconds)
- Automatic start/stop
- Prevents agent from being pruned

#### **ReconnectionStrategy (reconnect.rs)**
- Exponential backoff: `base * 2^attempt`
- Maximum backoff cap (60 seconds)
- Configurable max attempts
- Automatic reset on success

#### **Error Handling (error.rs)**
- `SdkError` with retryable classification
- Clear error messages
- Context preservation

---

### **3. feagi-agent-sdk-py (Python Extension)** 🐍

**Purpose:** Python bindings for the Rust SDK (via PyO3)

**Architecture:**

```python
Python Application
      ↓
PyAgentClient (Python wrapper)
      ↓ PyO3 FFI
AgentClient (Rust implementation)
      ↓
feagi-agent-registry (Rust)
```

**Key Features:**
- **Zero-Copy Data Transfer** - Efficient Python↔Rust
- **GIL Release** - Rust operations don't block Python threads
- **Type Conversion** - Automatic Python↔Rust type mapping
- **Exception Translation** - Rust errors → Python exceptions
- **Pythonic API** - Natural Python interface

**Python API:**

```python
from feagi_agent_sdk_py import PyAgentClient, PyAgentConfig, AgentType

# Configuration
config = PyAgentConfig("agent_id", AgentType.Sensory)
config.with_feagi_host("localhost")
config.with_vision_capability("camera", 640, 480, 3, "i_vision")
config.with_heartbeat_interval(5.0)

# Client
client = PyAgentClient(config)
client.connect()

# Send data
client.send_sensory_data([(neuron_id, potential), ...])

# Receive motor commands (for motor agents)
motor_data = client.receive_motor_data()  # Non-blocking
```

---

## 🔄 **Data Flow**

### **Agent Registration Flow:**

```
Agent Application
    │
    ├─1─→ Create AgentConfig
    │     - Set agent_id, type, capabilities
    │     - Configure endpoints, timeouts
    │
    ├─2─→ Create AgentClient(config)
    │     - Validates configuration
    │     - Creates ZMQ context
    │
    ├─3─→ client.connect()
    │     ├─→ Create ZMQ sockets (REQ, PUSH, SUB)
    │     ├─→ Register with FEAGI (with retry)
    │     │   ├─→ Send registration JSON via ZMQ REQ
    │     │   ├─→ Wait for response
    │     │   └─→ Handle "already registered" → auto-deregister
    │     └─→ Start HeartbeatService
    │         └─→ Background thread sends heartbeat every N seconds
    │
    └─4─→ client.send_sensory_data()
          └─→ ZMQ PUSH to FEAGI sensory endpoint
```

### **Heartbeat Flow:**

```
HeartbeatService (Background Thread)
    │
    ├─→ Sleep(interval) [5 seconds]
    │
    ├─→ Send heartbeat JSON
    │   {"type": "heartbeat", "agent_id": "...", "timestamp": ...}
    │
    ├─→ Wait for response (1 second timeout)
    │   ├─→ Success → log ✓
    │   └─→ Timeout → log warning (don't fail)
    │
    └─→ Repeat until stopped
```

### **Sensory Data Flow:**

```
Agent Application
    │
    ├─→ Process sensor input (camera, lidar, etc.)
    │
    ├─→ Convert to neuron activations
    │   [(neuron_id: int, potential: float), ...]
    │
    ├─→ client.send_sensory_data(neuron_pairs)
    │   │
    │   └─→ Build JSON: {
    │         "neuron_id_potential_pairs": [[id, pot], ...],
    │         "agent_id": "...",
    │         "frame_number": N
    │       }
    │
    └─→ ZMQ PUSH to FEAGI
        └─→ FEAGI receives → injects into NPU
```

### **Motor Data Flow (for motor agents):**

```
FEAGI NPU
    │
    ├─→ Neural processing produces motor outputs
    │
    ├─→ FEAGI publishes via ZMQ PUB
    │   {agent_id, motor_commands: [...]}
    │
    └─→ Agent receives via ZMQ SUB
        │
        └─→ client.receive_motor_data() [non-blocking]
            ├─→ Returns Some(data) if available
            └─→ Returns None if no data
```

---

## 🌐 **Network Protocol (ZMQ)**

### **Endpoints:**

| Socket Type | Direction | Default Port | Purpose |
|-------------|-----------|--------------|---------|
| REQ/REP | Agent ↔ FEAGI | 30001 | Registration & Heartbeat |
| PUSH/PULL | Agent → FEAGI | 5555 | Sensory Data |
| SUB/PUB | FEAGI → Agent | 30005 | Motor Commands |

### **Registration Protocol (REQ/REP):**

**Request (Agent → FEAGI):**
```json
{
  "type": "register",
  "agent_id": "video_camera_01",
  "agent_type": "sensory",
  "capabilities": {
    "vision": {
      "modality": "camera",
      "dimensions": [640, 480],
      "channels": 3,
      "target_cortical_area": "i_vision"
    }
  }
}
```

**Response (FEAGI → Agent):**
```json
{
  "status": "success",
  "agent_id": "video_camera_01",
  "message": "Agent registered successfully",
  "endpoints": {
    "sensory_endpoint": "tcp://0.0.0.0:5555",
    "motor_endpoint": "tcp://0.0.0.0:30005"
  }
}
```

### **Heartbeat Protocol (REQ/REP):**

**Request:**
```json
{
  "type": "heartbeat",
  "agent_id": "video_camera_01",
  "timestamp": 1234567890
}
```

**Response:**
```json
{
  "status": "success",
  "agent_id": "video_camera_01"
}
```

### **Sensory Data Protocol (PUSH):**

**Message:**
```json
{
  "neuron_id_potential_pairs": [
    [0, 50.0],
    [1, 75.0],
    [2, 30.0]
  ],
  "agent_id": "video_camera_01",
  "frame_number": 42
}
```

---

## ⚙️ **Configuration System**

### **Agent Configuration:**

```rust
AgentConfig {
    // Identity
    agent_id: String,
    agent_type: AgentType,
    
    // Network
    registration_endpoint: String,  // tcp://host:30001
    sensory_endpoint: String,       // tcp://host:5555
    motor_endpoint: String,         // tcp://host:30005
    
    // Reliability
    heartbeat_interval: f64,        // seconds (0 = disabled)
    connection_timeout_ms: u64,     // milliseconds
    registration_retries: u32,      // max attempts
    retry_backoff_ms: u64,          // initial backoff
    
    // Capabilities
    capabilities: AgentCapabilities {
        vision: Option<VisionCapability>,
        motor: Option<MotorCapability>,
        custom: Map<String, Value>,
    }
}
```

### **Recommended Settings:**

**Development:**
```rust
AgentConfig::new("agent", AgentType::Sensory)
    .with_heartbeat_interval(5.0)    // 5 second heartbeat
    .with_connection_timeout_ms(5000) // 5 second timeout
    .with_registration_retries(3)     // Try 3 times
```

**Production:**
```rust
AgentConfig::new("agent", AgentType::Sensory)
    .with_heartbeat_interval(10.0)    // 10 second heartbeat
    .with_connection_timeout_ms(10000) // 10 second timeout
    .with_registration_retries(5)      // Try 5 times
```

**Embedded/Constrained:**
```rust
AgentConfig::new("agent", AgentType::Sensory)
    .with_heartbeat_interval(30.0)    // 30 second heartbeat
    .with_connection_timeout_ms(30000) // 30 second timeout
    .with_registration_retries(10)     // Try 10 times
```

---

## 🔒 **Thread Safety**

### **Rust SDK:**
- `AgentClient` uses `Arc<Mutex<>>` for internal state
- Safe to clone and share across threads
- ZMQ sockets are NOT thread-safe, but protected by mutexes
- Heartbeat runs in dedicated background thread

### **Python Bindings:**
- `PyAgentClient` wraps Rust `AgentClient` with `Arc<Mutex<>>`
- Safe to use from multiple Python threads
- GIL released during Rust operations (no Python thread blocking)

---

## 🚨 **Error Handling Strategy**

### **Classification:**

**Retryable Errors:**
- Network timeouts
- Connection failures
- ZMQ socket errors
- Registration "already registered" (auto-deregister + retry)

**Non-Retryable Errors:**
- Invalid configuration
- Validation failures
- Agent not registered (when trying to send data)
- Malformed JSON

### **Retry Logic:**

```
Attempt 1: Immediate
Attempt 2: Wait 1s  (base_backoff)
Attempt 3: Wait 2s  (base_backoff * 2)
Attempt 4: Wait 4s  (base_backoff * 4)
Attempt 5: Wait 8s  (base_backoff * 8)
...
Max Wait: 60s (capped)
```

---

## 📊 **Performance Characteristics**

### **Rust SDK:**
- **Registration**: ~5-50ms (network dependent)
- **Heartbeat**: ~1-10ms per heartbeat
- **Send Data**: ~0.1-1ms per message (ZMQ PUSH)
- **Receive Data**: ~0.1-1ms per poll (ZMQ SUB)
- **Memory**: ~1-2MB per agent (including ZMQ buffers)

### **Python Bindings:**
- **Overhead**: <100μs per Python→Rust call (PyO3)
- **Data Transfer**: Zero-copy for most operations
- **GIL**: Released during Rust operations

### **Scalability:**
- **Agents per FEAGI**: 1000+ (tested)
- **Messages per Second**: 10,000+ per agent (hardware dependent)
- **Heartbeat Overhead**: Negligible (<0.1% CPU)

---

## 🔧 **Deployment Modes**

### **Mode 1: Python FEAGI (Most Common)**

```
Python FEAGI Process
    ├── PyO3 bindings to feagi_rust
    ├── Uses PyAgentRegistry (Rust-backed)
    └── ZmqRegistrationListener handles agents

Agents (separate processes)
    ├── Rust agents use feagi-agent-sdk directly
    └── Python agents use feagi-agent-sdk-py (wraps Rust SDK)
```

### **Mode 2: Standalone Rust Inference Engine**

```
Rust Inference Engine Process
    ├── Uses AgentRegistry directly
    └── Built-in ZMQ registration listener

Agents (separate processes)
    └── Same as Mode 1
```

**Note:** Mode 1 and Mode 2 are **mutually exclusive** - never run both simultaneously.

---

## 🛠️ **Development Workflow**

### **Building Rust SDK:**
```bash
cd feagi_core/feagi-rust/crates/feagi-agent-sdk
cargo build --release
cargo test
cargo run --example simple_sensory_agent
```

### **Building Python Bindings:**
```bash
cd feagi_core/feagi-rust/crates/feagi-agent-sdk-py
maturin develop --release
python test_bindings.py
```

### **Using in Python Projects:**
```python
# Add to requirements.txt or install directly
pip install feagi-agent-sdk-py  # (when published)

# Or install from local source:
cd feagi_core/feagi-rust/crates/feagi-agent-sdk-py
maturin develop --release
```

---

## 📚 **Further Reading**

- [Rust SDK README](./README.md) - Rust-specific documentation
- [Python Bindings README](../feagi-agent-sdk-py/README.md) - Python-specific documentation
- [Agent Registry Documentation](../feagi-agent-registry/src/lib.rs) - Server-side registry
- [ZMQ Guide](https://zguide.zeromq.org/) - ZeroMQ documentation

---

## 🤝 **Contributing**

See [CONTRIBUTING.md](../../../../CONTRIBUTING.md) for development guidelines.

---

## 📄 **License**

Apache-2.0

