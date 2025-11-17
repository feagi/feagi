# Changelog

All notable changes to the FEAGI Python SDK will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [3.0.0] - 2025-01-XX

### 🎉 Complete Rewrite - Clean Architecture

This is a major release with breaking changes. The package has been completely restructured with a clean, modern architecture and no legacy code.

### Added

- **New modular structure:**
  - `feagi.engine` - Engine control (Rust FFI) [Phase 2]
  - `feagi.agent` - Agent framework with BaseAgent class
  - `feagi.genome` - Runtime genome manipulation [Phase 3]
  - `feagi.connectome` - Runtime connectome operations [Phase 3]
  - `feagi.packaging` - Marketplace package building [Phase 4]
  - `feagi.pns` - Peripheral Nervous System (communication layer)
  - `feagi.cli` - Command-line tools [Phase 4]

- **BaseAgent framework:**
  - Abstract base class for all agents
  - Template pattern for hardware initialization
  - Sensor/motor mapping abstraction
  - Async agent loop management

- **Clean PNS module:**
  - Modern FeagiAgentClient (Rust-backed)
  - CapabilitiesManager
  - MotorProcessor
  - VisionProcessor and SegmentedVisionProcessor
  - Logging and utilities

### Changed

- **Package renamed:** `feagi_connector` → `feagi`
- **Version:** 2.1.0 → 3.0.0
- **Python requirement:** 3.8+ → 3.10+
- **Import paths:** `from feagi_connector import X` → `from feagi.pns import X`

### Removed

- ❌ All legacy code removed:
  - `FeagiClient` (legacy ZMQ client)
  - `FeagiAgentConnector` (legacy connector)
  - Old API structure (`api/`)
  - Old connection state (`state/`)
  - Old caching system (`cache/`)
  - Old interfaces (`feagi_interfaces/`)
  - All deprecated functions and fallbacks

- ❌ Backward compatibility code removed
- ❌ Dead code and unused modules removed

### Migration Guide

#### Import Changes

```python
# Old (2.x)
from feagi_connector import FeagiAgentClient, AgentType
from feagi_connector import CapabilitiesManager
from feagi_connector import MotorProcessor

# New (3.0)
from feagi.pns import FeagiAgentClient, AgentType
from feagi.pns import CapabilitiesManager
from feagi.pns import MotorProcessor
```

#### Agent Development

```python
# Old (2.x) - Manual client management
from feagi_connector import FeagiAgentClient

client = FeagiAgentClient("agent-01", AgentType.BOTH)
client.configure(...)
await client.connect()
# Manual sensor/motor loop

# New (3.0) - BaseAgent framework
from feagi.agent import BaseAgent

class MyAgent(BaseAgent):
    def initialize_hardware(self):
        # Hardware setup
        pass
    
    def map_sensors(self, hw_data):
        return {"camera": image_bytes}
    
    def map_motors(self, feagi_output):
        return motor_commands

agent = MyAgent("agent-01")
await agent.connect()
await agent.run()  # Handles loop automatically
```

### Dependencies

- Required: numpy>=1.20.0, pyzmq>=24.0.0, aiohttp>=3.9.0, toml>=0.10.2, requests>=2.31.0
- Optional: opencv-python>=4.9.0 (for video processing)
- Optional: feagi_rust_py_libs (for FeagiAgentClient)

### Notes

- **No fallbacks:** Package fails fast with clear error messages
- **No dead code:** Only modern, maintained code included
- **Rust SDK:** FeagiAgentClient requires feagi_rust_py_libs
- **BaseAgent:** Core framework works without Rust SDK

### Roadmap

- **Phase 2:** Engine control (feagi.engine)
- **Phase 3:** Genome/connectome APIs
- **Phase 4:** Packaging and CLI tools

---

## [2.1.0] - 2024-XX-XX (DEPRECATED)

Last version of old `feagi_connector` package.
Deprecated. Upgrade to 3.0.0.

---

For older changelog entries, see the legacy repository.

