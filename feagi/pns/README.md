# FEAGI Peripheral Nervous System (PNS) Module

**Architecture Status**: Hybrid Python/Rust (Migrating to Rust)

The Peripheral Nervous System (PNS) module provides sensorimotor interfaces between FEAGI and the external world, allowing for embodied cognition through various sensory inputs and motor outputs.

## 🦀 Rust PNS Architecture (PRIMARY)

**Location**: `feagi_core/feagi-rust/crates/feagi-pns/`

The Rust PNS is the **primary** I/O layer for FEAGI, handling all ZMQ communication with agents:

### Core Components (Rust)

```
┌─────────────────────────────────────────────────────────┐
│ Rust PNS (feagi-pns crate)                             │
├─────────────────────────────────────────────────────────┤
│ • agent_registry.rs      → Agent tracking               │
│ • registration.rs        → Registration handler         │
│ • heartbeat.rs           → Heartbeat monitor            │
│ • zmq/rest.rs           → REST/registration stream      │
│ • zmq/motor.rs          → Motor output stream           │
│ • zmq/visualization.rs  → Visualization output stream   │
│ • zmq/sensory.rs        → Sensory input stream (PULL)   │
│ • shm.rs                → Shared memory I/O             │
└─────────────────────────────────────────────────────────┘
```

### Rust PNS Benefits

1. **Performance**: 10-100x faster than Python for binary data processing
2. **Direct Paths**: Rust agent → Rust PNS → Rust NPU (no FFI overhead)
3. **Architecture**: All I/O in one place, consistent patterns
4. **RTOS-Ready**: Designed for embedded systems migration

### ZMQ Streams (Rust)

| Stream | Socket | Port | Direction | Purpose |
|--------|--------|------|-----------|---------|
| REST | ROUTER | 5563 | Bi-directional | Agent registration/heartbeat |
| Sensory | PULL | 5558 | Agent → FEAGI | Binary XYZP sensory data |
| Motor | PUB | 30005 | FEAGI → Agent | Motor commands |
| Visualization | PUB | 5562 | FEAGI → Agent | Neural activity data |

## 🐍 Python PNS Layer (ORCHESTRATION)

**Location**: `feagi_core/feagi/pns/`

The Python PNS layer provides **orchestration** over the Rust core:

### Python Components

```
feagi/pns/
├── registration_manager.py  ✅ Orchestration over Rust PyAgentRegistry
├── vision.py               ✅ Domain logic (genome integration)
├── agent-coordination.md   ✅ Documentation
├── arch-pns.md            ✅ Architecture guide
└── README.md              ✅ This file
```

### Division of Labor

**Rust (Core I/O)**:
- ✅ All agent storage (PyAgentRegistry)
- ✅ All ZMQ communication
- ✅ Binary data processing (XYZP)
- ✅ Heartbeat tracking
- ✅ Performance-critical paths

**Python (Orchestration)**:
- ✅ FQ sampler coordination
- ✅ Capability rate management
- ✅ State manager integration
- ✅ Process manager integration
- ✅ Domain logic (vision config)

## Data Flow Architecture

### Sensory Input (Agent → FEAGI)

```
┌──────────┐                    ┌──────────────┐                    ┌──────────┐
│  Agent   │  XYZP Binary       │  Rust PNS    │  Direct Injection  │ Rust NPU │
│ (Rust/   │  ──────────────>   │  Sensory     │  ──────────────>   │          │
│  Python) │  ZMQ PUSH (5558)   │  Stream      │  (No Python FFI!)  │          │
└──────────┘                    └──────────────┘                    └──────────┘
              10-100x faster than Python path
              Pure Rust: Agent → ZMQ → Deserialize → NPU
```

### Motor Output (FEAGI → Agent)

```
┌──────────┐                    ┌──────────────┐                    ┌──────────┐
│ Rust NPU │  Motor Commands    │  Rust PNS    │  ZMQ PUB (30005)   │  Agent   │
│          │  ──────────────>   │  Motor       │  ──────────────>   │          │
│          │                    │  Stream      │                    │          │
└──────────┘                    └──────────────┘                    └──────────┘
```

### Visualization (FEAGI → Brain Visualizer)

```
┌──────────┐                    ┌──────────────┐                    ┌──────────┐
│ Rust NPU │  FQ Sample         │  Rust PNS    │  ZMQ PUB (5562)    │   BV     │
│          │  ──────────────>   │  Viz         │  ──────────────>   │          │
│          │                    │  Stream      │                    │          │
└──────────┘                    └──────────────┘                    └──────────┘
```

## Agent Registration & Coordination

The agent registration system provides intelligent management of connected agents and automatic coordination with FEAGI's Fire Queue (FQ) samplers.

### Registration Flow

1. **Agent connects** → Rust PNS REST stream (port 5563)
2. **Rust registry stores** → PyAgentRegistry (single source of truth)
3. **Python orchestrates** → FQ sampler enable/disable
4. **State updates** → State manager, capability rates

### Automatic FQ Sampler Coordination

- **Visualization FQ**: Auto-enabled when visualization agent connects
- **Motor FQ**: Auto-enabled when motor agent connects
- **Auto-disable**: When last agent of type disconnects
- **Zero intervention**: Fully automatic, no manual management

For details, see [agent-coordination.md](agent-coordination.md)

## Vision System

The vision component processes visual inputs, converting them into neural signals:

### Vision Components

- **Central Vision**: High-resolution processing (cortical area: `iic400`)
- **Peripheral Vision**: Lower-resolution (8 regions: `iic000`-`iic800`)
- **Visual Enhancement**: Brightness, contrast, shadow adjustments
- **Visual Modulation**: Eccentricity and modulation parameters

### Configuration

Vision configuration is genome-driven and managed by `vision.py`:

```python
from feagi.pns import vision

# Get current vision configuration
config = vision.generate_vision_configuration()

# Reconfigure vision system
vision.reconfigure_vision(vision_parameters, connectome)
```

## Architecture Compliance

### ✅ FEAGI 2.0 Principles

1. **No Hardcoded Values**: All ports/hosts from TOML config
2. **Single Source of Truth**: Rust PyAgentRegistry for agent storage
3. **No Fallbacks**: Fail fast with clear errors
4. **Rust Migration Ready**: All I/O in Rust, ready for full migration
5. **Cross-Platform**: Docker, Kubernetes, embedded, desktop

### ❌ Deprecated Components

The following Python components have been **removed** (moved to Rust):

- ~~`zmq_sensory_listener.py`~~ → `feagi-pns/src/zmq/sensory.rs`
- ~~`zmq_registration_listener.py`~~ → `feagi-pns/src/zmq/rest.rs`

**Rationale**: All ZMQ I/O must be in Rust for:
- Performance (10-100x faster binary processing)
- Consistency (all streams in one place)
- Architecture (eliminate Python FFI bottlenecks)

## Integration with FEAGI

The PNS module integrates with other FEAGI components:

- **Rust NPU**: Direct injection/sampling (no Python FFI)
- **BDU**: Coordinates sensorimotor pathway creation
- **State Manager**: Agent registry state synchronization
- **Process Manager**: Lifecycle management and coordination
- **Core API**: REST API integration for agent management

## Future Architecture

### Planned Rust Migration

- [x] Agent registry → Rust
- [x] ZMQ registration → Rust
- [x] ZMQ sensory → Rust
- [x] ZMQ motor → Rust
- [x] ZMQ visualization → Rust
- [ ] Vision processing → Rust (domain logic can stay Python)
- [ ] Auditory processing → Rust
- [ ] Tactile processing → Rust

### Target State

```
┌─────────────────────────────────────────────────────────┐
│ Python Layer (Minimal Orchestration)                    │
│ • Registration orchestration (FQ coordination)          │
│ • Domain logic (vision config, genome integration)      │
│ • REST API integration                                  │
└─────────────────────────────────────────────────────────┘
                          ↓↑
┌─────────────────────────────────────────────────────────┐
│ Rust PNS (Complete I/O Layer)                           │
│ • All ZMQ streams                                       │
│ • All SHM I/O                                           │
│ • Binary data processing                                │
│ • Agent management                                      │
└─────────────────────────────────────────────────────────┘
                          ↓↑
┌─────────────────────────────────────────────────────────┐
│ Rust NPU (Neural Processing)                            │
│ • Burst engine                                          │
│ • Neural dynamics                                       │
│ • Synaptic propagation                                  │
└─────────────────────────────────────────────────────────┘
```

## Development

### Running Tests

**Rust PNS:**
```bash
cd feagi_core/feagi-rust/crates/feagi-pns
cargo test
```

**Python Orchestration:**
```bash
cd feagi_core
pytest tests/pns/
```

### Adding New Sensory Systems

1. Add Rust stream in `feagi-pns/src/zmq/` or `feagi-pns/src/shm/`
2. Add Python orchestration in `registration_manager.py` if needed
3. Update configuration in `feagi_configuration.toml`
4. Add tests

## References

- Rust PNS: `feagi_core/feagi-rust/crates/feagi-pns/`
- Python PNS: `feagi_core/feagi/pns/`
- Configuration: `feagi_core/feagi_configuration.toml`
- Architecture compliance: `.cursorrules`
- Agent coordination: [agent-coordination.md](agent-coordination.md)
- Architecture guide: [arch-pns.md](arch-pns.md)
