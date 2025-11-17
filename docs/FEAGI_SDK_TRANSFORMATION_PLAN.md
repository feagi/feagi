# FEAGI Python SDK Transformation Plan

**Version:** 1.0  
**Date:** November 8, 2025  
**Status:** Planning Phase  

## Executive Summary

This document outlines the transformation of `feagi-connector` into a comprehensive Python SDK for FEAGI 2.0. The goal is to create a unified, intuitive, and fully-featured SDK that enables developers to interact with FEAGI through native Python APIs, with full IDE support, type hints, and eventual AI agent integration via MCP (Model Context Protocol).

## Vision Statement

Transform `feagi-connector` from a specialized agent communication library into **the official Python SDK for FEAGI 2.0**, providing:

1. **Unified Installation**: `pip install feagi` for everything
2. **Engine Management**: Start/stop Rust FEAGI engine from Python
3. **Configuration Management**: Programmatic TOML configuration with IDE support
4. **Connector API**: Sensorimotor registration and agent management
5. **Genomics API**: Full genome manipulation capabilities
6. **Connectome API**: Connectome metrics and interactions
7. **Embedded Support**: Export for ESP32, Arduino, etc.
8. **Full IDE Integration**: Type hints, autocomplete, inline docs
9. **MCP Server Support**: AI agent integration
10. **REST API Wrapper**: Native Python methods for all REST operations

---

## Current State Analysis

### What Exists Today

#### feagi-connector (Python Package)
- ✅ Agent communication (ZMQ, WebSocket)
- ✅ Sensory data transmission
- ✅ Motor data reception
- ✅ Vision processing (segmented, gaze control)
- ✅ Configuration loading (TOML)
- ✅ Rust-backed client (`FeagiAgentClient`)
- ✅ Shared memory support
- ✅ REST API helpers (basic)

**Package Structure:**
```
feagi_connector/
├── agent_client.py          # Rust-backed client
├── agent_connector.py       # Legacy connector
├── client.py                # Legacy ZMQ client
├── agent_logging/           # Logging utilities
├── api/                     # Protocol definitions
├── capabilities/            # Device management
├── motor/                   # Motor processing
├── vision/                  # Vision processing
├── utils/                   # Config, SHM, REST helpers
└── ...
```

#### feagi (Rust Binary)
- ✅ Full FEAGI server application
- ✅ REST API (60+ endpoints)
- ✅ ZMQ streams (PNS)
- ✅ Burst engine (NPU)
- ✅ Genome loading (BDU)
- ✅ Configuration-driven (`feagi_configuration.toml`)
- ✅ Cross-platform (Linux, macOS, Windows, Docker, K8s)
- ✅ Library mode available (`lib.rs` with `FeagiInstance`)

**Crate Structure:**
```
feagi-core/crates/
├── feagi-api/              # REST API layer
├── feagi-bdu/              # Brain Development Unit
├── feagi-burst-engine/     # Neural Processing Unit
├── feagi-evo/              # Genome I/O
├── feagi-pns/              # Peripheral Nervous System
├── feagi-services/         # Service orchestration
├── feagi-config/           # Configuration management
├── feagi-types/            # Core data types
└── ...
```

### Gaps to Address

1. **No Engine Management from Python**
   - Cannot start/stop Rust FEAGI from Python
   - No subprocess management wrapper
   - No lifecycle control

2. **Limited Configuration API**
   - Only loading, no programmatic modification
   - No validation helpers
   - No IDE-friendly configuration builders

3. **No Genomics API**
   - Cannot create/modify genomes from Python
   - No cortical area CRUD operations
   - No neuron/synapse manipulation

4. **No Connectome API**
   - Cannot query connectome structure
   - No metrics extraction
   - No topology analysis

5. **No Embedded Export API**
   - Cannot generate embedded-ready connectomes
   - No platform-specific optimization
   - No flash utilities

6. **Incomplete REST API Wrapper**
   - Only a few helper functions exist
   - Not comprehensive
   - Not typed

7. **No MCP Server**
   - No AI agent integration layer
   - Cannot expose FEAGI operations to LLMs

8. **Package Naming Mismatch**
   - `feagi-connector` != `feagi`
   - Users must `pip install feagi-connector` instead of `pip install feagi`

---

## Proposed Architecture

### Package Structure

Transform `feagi-connector` into `feagi` with modular sub-packages:

```
feagi/
├── __init__.py                 # Main entry point
├── engine/                     # Engine management (NEW)
│   ├── __init__.py
│   ├── manager.py             # FeagiEngineManager class
│   ├── process.py             # Subprocess control
│   └── lifecycle.py           # Start/stop/restart logic
├── config/                     # Configuration API (ENHANCED)
│   ├── __init__.py
│   ├── loader.py              # Existing: load_agent_config, etc.
│   ├── manager.py             # NEW: ConfigManager class
│   ├── builders.py            # NEW: Fluent config builders
│   └── validation.py          # NEW: Validation helpers
├── connector/                  # Connector API (REFACTORED)
│   ├── __init__.py
│   ├── client.py              # Existing: FeagiAgentClient, etc.
│   ├── sensory.py             # NEW: Sensory registration helpers
│   ├── motor.py               # NEW: Motor registration helpers
│   └── agent.py               # NEW: Agent lifecycle
├── genomics/                   # Genomics API (NEW)
│   ├── __init__.py
│   ├── genome.py              # Genome class
│   ├── cortical_area.py       # CorticalArea CRUD
│   ├── neurons.py             # Neuron manipulation
│   ├── synapses.py            # Synapse manipulation
│   └── templates.py           # Genome templates
├── connectome/                 # Connectome API (NEW)
│   ├── __init__.py
│   ├── query.py               # Connectome queries
│   ├── metrics.py             # Metrics calculation
│   ├── topology.py            # Topology analysis
│   └── visualization.py       # Connectome visualization helpers
├── embedded/                   # Embedded API (NEW)
│   ├── __init__.py
│   ├── exporter.py            # Export connectome for embedded
│   ├── platforms/             # Platform-specific modules
│   │   ├── esp32.py
│   │   ├── arduino.py
│   │   ├── rpi_pico.py
│   │   └── stm32.py
│   └── flash.py               # Flashing utilities
├── api/                        # REST API Wrapper (ENHANCED)
│   ├── __init__.py
│   ├── client.py              # Base REST client
│   ├── genome.py              # Genome endpoints
│   ├── cortical_areas.py      # Cortical area endpoints
│   ├── neurons.py             # Neuron endpoints
│   ├── runtime.py             # Runtime control endpoints
│   ├── agents.py              # Agent management endpoints
│   └── ...                    # Other endpoint groups
├── mcp/                        # MCP Server (NEW)
│   ├── __init__.py
│   ├── server.py              # MCP server implementation
│   ├── tools.py               # Tool definitions
│   └── schemas.py             # JSON schemas for tools
├── types/                      # Type definitions (NEW)
│   ├── __init__.py
│   ├── genome.py              # TypedDict/Pydantic models for genomes
│   ├── connectome.py          # TypedDict/Pydantic models for connectome
│   ├── config.py              # TypedDict/Pydantic models for config
│   └── api.py                 # TypedDict/Pydantic models for API responses
└── utils/                      # Utilities (EXISTING + ENHANCED)
    ├── __init__.py
    ├── rest_helpers.py        # Existing
    ├── shm.py                 # Existing
    ├── zero_serialization.py  # Existing
    └── ...
```

### Import Hierarchy

Users will interact with FEAGI through intuitive imports:

```python
# Engine management
from feagi import engine
engine.start()
engine.stop()
engine.restart()

# Configuration
from feagi import config
cfg = config.load("feagi_configuration.toml")
cfg.api.port = 9000
cfg.save()

# Or fluent builder
from feagi.config import ConfigBuilder
cfg = (ConfigBuilder()
    .api(host="0.0.0.0", port=8000)
    .neural(timestep=0.1)
    .build())

# Connector API
from feagi import connector
agent = connector.create_agent("my_agent", agent_type="vision")
agent.register_sensory("camera", cortical_area="v1")
agent.send_sensory_data("camera", image_bytes)

# Genomics API
from feagi import genomics
genome = genomics.create_genome("my_brain")
v1 = genome.add_cortical_area("v1", dimensions=(128, 128, 10))
v2 = genome.add_cortical_area("v2", dimensions=(64, 64, 20))
genome.connect(v1, v2, weight=0.5)
genome.save("my_brain.json")

# Connectome API
from feagi import connectome
metrics = connectome.get_metrics()
print(f"Neurons: {metrics.neuron_count}")
print(f"Synapses: {metrics.synapse_count}")
print(f"Connectivity: {metrics.connectivity_density}")

# Embedded export
from feagi import embedded
esp32_config = embedded.platforms.esp32.ESP32Config(
    memory_budget=512_000,  # 512KB
    quantization="int8"
)
embedded.export("my_brain.json", "esp32", esp32_config)
embedded.flash("esp32", port="/dev/ttyUSB0")
```

---

## Implementation Phases

### Phase 1: Foundation & Package Renaming (Weeks 1-2)

#### Goals
- Rename package from `feagi-connector` to `feagi`
- Establish new directory structure
- Migrate existing functionality
- Set up comprehensive type hints
- Establish testing infrastructure

#### Tasks

1. **Package Renaming**
   - [ ] Update `pyproject.toml`: `name = "feagi"`
   - [ ] Create new directory structure
   - [ ] Move existing `feagi_connector/` to `feagi/connector/`
   - [ ] Update all internal imports
   - [ ] Update documentation

2. **Type System Setup**
   - [ ] Create `feagi/types/` module
   - [ ] Define TypedDict/Pydantic models for:
     - Genome structures
     - Cortical areas
     - Neurons/synapses
     - API requests/responses
     - Configuration
   - [ ] Add `py.typed` marker file
   - [ ] Configure mypy strict mode

3. **Testing Infrastructure**
   - [ ] Set up pytest with type checking
   - [ ] Create test fixtures for:
     - Mock FEAGI server
     - Sample genomes
     - Sample configurations
   - [ ] Establish coverage targets (>85%)

4. **Documentation Foundation**
   - [ ] Update README with new vision
   - [ ] Create API reference skeleton
   - [ ] Set up docusaurus or mkdocs
   - [ ] Write contribution guidelines

#### Deliverables
- ✅ Package renamed and published as `feagi` on PyPI
- ✅ All existing tests passing
- ✅ Type hints on all public APIs
- ✅ Documentation site skeleton

---

### Phase 2: Engine Management API & Binary Distribution (Weeks 3-5)

#### Goals
- Bundle pre-built FEAGI Rust binaries in platform-specific wheels
- Enable starting/stopping FEAGI engine from Python
- Provide lifecycle management
- Support configuration overrides
- Handle subprocess monitoring
- Set up CI/CD for cross-platform builds

#### Tasks

1. **Build System Setup (NEW - Critical for Option 1)**
   - [ ] Create `build.py` or configure `maturin`
   - [ ] Set up Rust cross-compilation:
     ```python
     # setup.py or build.py
     from setuptools import setup
     from setuptools_rust import RustExtension, Binding
     
     setup(
         rust_extensions=[
             RustExtension(
                 "feagi.engine.bin.feagi",
                 path="../feagi/Cargo.toml",
                 binding=Binding.Exec,  # Build as executable, not library
                 strip=True,            # Strip debug symbols (50MB → 50MB optimized)
             )
         ],
     )
     ```
   - [ ] Configure platform-specific build targets
   - [ ] Add post-build binary optimization (strip, optional UPX)

2. **CI/CD Pipeline for Multi-Platform Builds**
   - [ ] Create `.github/workflows/build-wheels.yml`:
     ```yaml
     name: Build Platform Wheels
     
     on:
       push:
         tags:
           - 'v*'
       workflow_dispatch:
     
     jobs:
       build:
         name: Build wheels on ${{ matrix.os }}
         runs-on: ${{ matrix.os }}
         strategy:
           matrix:
             include:
               # macOS ARM64 (Apple Silicon)
               - os: macos-14
                 target: aarch64-apple-darwin
                 platform: macosx_11_0_arm64
               
               # macOS x86_64 (Intel)
               - os: macos-13
                 target: x86_64-apple-darwin
                 platform: macosx_10_9_x86_64
               
               # Linux x86_64
               - os: ubuntu-latest
                 target: x86_64-unknown-linux-gnu
                 platform: manylinux_2_17_x86_64
               
               # Windows x86_64
               - os: windows-latest
                 target: x86_64-pc-windows-msvc
                 platform: win_amd64
         
         steps:
           - uses: actions/checkout@v4
           
           - name: Set up Python
             uses: actions/setup-python@v4
             with:
               python-version: '3.10'
           
           - name: Set up Rust
             uses: actions-rs/toolchain@v1
             with:
               toolchain: stable
               target: ${{ matrix.target }}
               override: true
           
           - name: Install build dependencies
             run: |
               python -m pip install --upgrade pip
               pip install maturin setuptools-rust wheel
           
           - name: Build FEAGI Rust binary
             working-directory: feagi
             run: |
               cargo build --release --target ${{ matrix.target }}
           
           - name: Copy binary to Python package
             run: |
               mkdir -p feagi-connector/feagi/engine/bin
               # Platform-specific copy commands
               # (see detailed script below)
           
           - name: Build Python wheel
             working-directory: feagi-connector
             run: |
               python -m pip install build
               python -m build --wheel
           
           - name: Upload wheel
             uses: actions/upload-artifact@v3
             with:
               name: wheels
               path: feagi-connector/dist/*.whl
       
       publish:
         needs: build
         runs-on: ubuntu-latest
         steps:
           - uses: actions/download-artifact@v3
           - name: Publish to PyPI
             uses: pypa/gh-action-pypi-publish@release/v1
             with:
               password: ${{ secrets.PYPI_API_TOKEN }}
               packages-dir: wheels/
     ```
   
   - [ ] Add build optimization script
   - [ ] Test builds on all platforms
   - [ ] Document build process

3. **Engine Manager Core**
   - [ ] Create `feagi/engine/manager.py`
   - [ ] Implement `FeagiEngineManager` class:
     ```python
     class FeagiEngineManager:
         def __init__(self, config_path: Optional[str] = None):
             """Initialize engine manager with optional config"""
         
         def start(self, 
                   genome_path: Optional[str] = None,
                   api_port: Optional[int] = None,
                   background: bool = True) -> bool:
             """Start FEAGI engine"""
         
         def stop(self, timeout: float = 10.0) -> bool:
             """Stop FEAGI engine gracefully"""
         
         def restart(self) -> bool:
             """Restart FEAGI engine"""
         
         def is_running(self) -> bool:
             """Check if engine is running"""
         
         def get_status(self) -> EngineStatus:
             """Get detailed engine status"""
         
         def wait_for_ready(self, timeout: float = 30.0) -> bool:
             """Wait until engine is ready to accept connections"""
     ```

4. **Process Management**
   - [ ] Create `feagi/engine/process.py`
   - [ ] Implement subprocess control:
     - Launch bundled Rust binary
     - Monitor health via HTTP health endpoint
     - Handle stdout/stderr logging
     - Graceful shutdown with SIGTERM
     - Force kill on timeout

5. **Binary Location & Loading**
   - [ ] Create `feagi/engine/binaries.py`
   - [ ] Implement binary locator:
     ```python
     def get_bundled_binary() -> Path:
         """Get path to bundled FEAGI binary"""
         import sys
         from pathlib import Path
         
         # Binary is bundled in package
         package_dir = Path(__file__).parent
         
         if sys.platform == "win32":
             binary = package_dir / "bin" / "feagi.exe"
         else:
             binary = package_dir / "bin" / "feagi"
         
         if not binary.exists():
             raise RuntimeError(
                 f"FEAGI binary not found at {binary}.\n"
                 f"Your platform may not be supported.\n"
                 f"Supported: macOS (ARM64/Intel), Linux (x86_64), Windows (x86_64)\n"
                 f"\n"
                 f"Manual installation:\n"
                 f"  cargo install feagi\n"
                 f"  OR download from https://github.com/Neuraville/FEAGI-2.0/releases"
             )
         
         # Verify binary is executable
         if not os.access(binary, os.X_OK):
             binary.chmod(0o755)
         
         return binary
     
     def check_binary_version(binary: Path) -> str:
         """Verify binary version matches SDK"""
         result = subprocess.run(
             [str(binary), "--version"],
             capture_output=True,
             text=True,
             timeout=5
         )
         version = result.stdout.strip().split()[-1]
         
         SDK_VERSION = "3.0.0"
         if not version.startswith("2."):  # Compatible with FEAGI 2.x
             raise RuntimeError(
                 f"Binary version {version} incompatible with SDK {SDK_VERSION}"
             )
         
         return version
     ```

6. **Integration with Config**
   - [ ] Allow overriding config values programmatically
   - [ ] Generate temporary config files if needed
   - [ ] Pass config via environment variables

7. **Testing**
   - [ ] Unit tests with mock subprocess
   - [ ] Integration tests with bundled binary
   - [ ] Test all lifecycle scenarios (start, stop, restart, crash recovery)
   - [ ] Test on all supported platforms
   - [ ] Test graceful degradation for unsupported platforms

8. **Platform-Specific Package Metadata**
   - [ ] Update `pyproject.toml` with platform classifiers:
     ```toml
     [project]
     classifiers = [
         "Operating System :: MacOS :: MacOS X",
         "Operating System :: POSIX :: Linux", 
         "Operating System :: Microsoft :: Windows",
     ]
     ```

#### Deliverables
- ✅ CI/CD pipeline building platform-specific wheels
- ✅ Bundled binaries in each wheel (~50MB per platform)
- ✅ `feagi.engine.start()` works out of the box
- ✅ `feagi.engine.stop()` works
- ✅ `feagi.engine.is_running()` works
- ✅ Full lifecycle tests pass on all platforms
- ✅ PyPI hosting with 4 platform wheels
- ✅ Documentation with examples

---

### Phase 3: Configuration API Enhancement (Weeks 5-6)

#### Goals
- Provide programmatic configuration modification
- Full IDE support (autocomplete, type hints)
- Validation and error messages
- Fluent builder pattern

#### Tasks

1. **Configuration Manager**
   - [ ] Create `feagi/config/manager.py`
   - [ ] Implement `ConfigManager` class:
     ```python
     class ConfigManager:
         def __init__(self, config: FeagiConfig):
             """Initialize with loaded config"""
         
         def set_api_port(self, port: int) -> None:
             """Set API port with validation"""
         
         def set_zmq_host(self, host: str) -> None:
             """Set ZMQ host"""
         
         def set_burst_timestep(self, timestep: float) -> None:
             """Set burst engine timestep"""
         
         def enable_gpu(self) -> None:
             """Enable GPU acceleration"""
         
         def save(self, path: Optional[str] = None) -> None:
             """Save config to file"""
         
         def validate(self) -> List[str]:
             """Validate configuration, return errors"""
     ```

2. **Fluent Builder**
   - [ ] Create `feagi/config/builders.py`
   - [ ] Implement fluent API:
     ```python
     from feagi.config import ConfigBuilder
     
     config = (ConfigBuilder()
         .api(host="0.0.0.0", port=8000)
         .zmq(host="0.0.0.0")
         .neural(timestep=0.1, max_cores=8)
         .gpu(enabled=True, device_id=0)
         .logging(level="DEBUG")
         .build())
     ```

3. **Type Definitions**
   - [ ] Create `feagi/types/config.py`
   - [ ] Use TypedDict or Pydantic for all config sections:
     - `SystemConfig`
     - `ApiConfig`
     - `ZmqConfig`
     - `NeuralConfig`
     - `GpuConfig`
     - `LoggingConfig`
     - etc.

4. **Validation**
   - [ ] Create `feagi/config/validation.py`
   - [ ] Implement validation rules:
     - Port range checks (1024-65535)
     - Host format validation
     - Timestep bounds
     - File path existence
     - Architecture compliance checks

5. **Testing**
   - [ ] Test all config modifications
   - [ ] Test validation rules
   - [ ] Test builder pattern
   - [ ] Test TOML round-trip (load → modify → save → load)

#### Deliverables
- ✅ `ConfigManager` with full API
- ✅ Fluent `ConfigBuilder`
- ✅ All config parameters typed
- ✅ Validation with helpful error messages
- ✅ Documentation with examples

---

### Phase 4: REST API Wrapper (Weeks 7-9)

#### Goals
- Wrap all 60+ REST API endpoints
- Provide native Python methods
- Full type hints and documentation
- Error handling and retries

#### Tasks

1. **Base REST Client**
   - [ ] Create `feagi/api/client.py`
   - [ ] Implement `FeagiAPIClient` base class:
     ```python
     class FeagiAPIClient:
         def __init__(self, base_url: str = "http://localhost:8000"):
             """Initialize API client"""
         
         def get(self, endpoint: str, **params) -> Any:
             """GET request"""
         
         def post(self, endpoint: str, json: Any = None) -> Any:
             """POST request"""
         
         def put(self, endpoint: str, json: Any = None) -> Any:
             """PUT request"""
         
         def delete(self, endpoint: str) -> Any:
             """DELETE request"""
     ```

2. **Endpoint Groups**
   
   For each REST API category, create a dedicated module:
   
   **Genome API** (`feagi/api/genome.py`):
   ```python
   class GenomeAPI:
       def load_genome(self, genome_path: str) -> GenomeLoadResponse:
           """Load a genome file"""
       
       def unload_genome(self) -> None:
           """Unload current genome"""
       
       def get_genome_info(self) -> GenomeInfo:
           """Get loaded genome information"""
       
       def validate_genome(self, genome_path: str) -> ValidationResult:
           """Validate a genome file"""
   ```
   
   **Cortical Area API** (`feagi/api/cortical_areas.py`):
   ```python
   class CorticalAreaAPI:
       def list_cortical_areas(self) -> List[CorticalArea]:
           """List all cortical areas"""
       
       def get_cortical_area(self, area_id: str) -> CorticalArea:
           """Get cortical area details"""
       
       def create_cortical_area(self, spec: CorticalAreaSpec) -> CorticalArea:
           """Create new cortical area"""
       
       def update_cortical_area(self, area_id: str, 
                                spec: CorticalAreaSpec) -> CorticalArea:
           """Update cortical area"""
       
       def delete_cortical_area(self, area_id: str) -> None:
           """Delete cortical area"""
   ```
   
   **Neuron API** (`feagi/api/neurons.py`):
   ```python
   class NeuronAPI:
       def create_neuron(self, spec: NeuronSpec) -> Neuron:
           """Create a neuron"""
       
       def get_neuron(self, neuron_id: str) -> Neuron:
           """Get neuron details"""
       
       def update_neuron(self, neuron_id: str, spec: NeuronSpec) -> Neuron:
           """Update neuron parameters"""
       
       def delete_neuron(self, neuron_id: str) -> None:
           """Delete neuron"""
   ```
   
   **Runtime API** (`feagi/api/runtime.py`):
   ```python
   class RuntimeAPI:
       def start_burst_engine(self) -> None:
           """Start the burst engine"""
       
       def stop_burst_engine(self) -> None:
           """Stop the burst engine"""
       
       def get_burst_status(self) -> BurstEngineStatus:
           """Get burst engine status"""
       
       def set_burst_frequency(self, hz: float) -> None:
           """Set burst frequency"""
   ```
   
   **Agent API** (`feagi/api/agents.py`):
   ```python
   class AgentAPI:
       def list_agents(self) -> List[Agent]:
           """List all registered agents"""
       
       def get_agent(self, agent_id: str) -> Agent:
           """Get agent details"""
       
       def register_agent(self, spec: AgentSpec) -> Agent:
           """Register a new agent"""
       
       def unregister_agent(self, agent_id: str) -> None:
           """Unregister agent"""
   ```

3. **Unified API Facade**
   - [ ] Create `feagi/api/__init__.py`
   - [ ] Aggregate all endpoint groups:
     ```python
     class FeagiAPI:
         def __init__(self, base_url: str = "http://localhost:8000"):
             self.genome = GenomeAPI(base_url)
             self.cortical_areas = CorticalAreaAPI(base_url)
             self.neurons = NeuronAPI(base_url)
             self.runtime = RuntimeAPI(base_url)
             self.agents = AgentAPI(base_url)
             # ... etc
     ```

4. **Type Models**
   - [ ] Create `feagi/types/api.py`
   - [ ] Define Pydantic or TypedDict models for:
     - All request payloads
     - All response payloads
     - Enums for constants
   - [ ] Match Rust API DTOs exactly

5. **Error Handling**
   - [ ] Create custom exception hierarchy:
     - `FeagiAPIError`
     - `FeagiConnectionError`
     - `FeagiAuthError`
     - `FeagiValidationError`
   - [ ] Wrap HTTP errors with meaningful messages
   - [ ] Add retry logic with exponential backoff

6. **Testing**
   - [ ] Mock all REST endpoints
   - [ ] Test all API methods
   - [ ] Test error handling
   - [ ] Test retry logic
   - [ ] Integration tests with real FEAGI server

#### Deliverables
- ✅ All 60+ REST endpoints wrapped
- ✅ Full type hints on all methods
- ✅ Comprehensive error handling
- ✅ 100% test coverage
- ✅ API reference documentation

---

### Phase 5: Genomics API (Weeks 10-12)

#### Goals
- High-level Python API for genome manipulation
- Create, modify, save genomes
- Cortical area CRUD operations
- Neuron and synapse manipulation

#### Tasks

1. **Genome Class**
   - [ ] Create `feagi/genomics/genome.py`
   - [ ] Implement `Genome` class:
     ```python
     class Genome:
         def __init__(self, name: str):
             """Create a new genome"""
         
         @classmethod
         def load(cls, path: str) -> "Genome":
             """Load genome from file"""
         
         def save(self, path: str) -> None:
             """Save genome to file"""
         
         def add_cortical_area(self, name: str, 
                               dimensions: Tuple[int, int, int],
                               **kwargs) -> CorticalArea:
             """Add a cortical area"""
         
         def remove_cortical_area(self, name: str) -> None:
             """Remove a cortical area"""
         
         def get_cortical_area(self, name: str) -> CorticalArea:
             """Get cortical area by name"""
         
         def connect(self, source: str, target: str, **kwargs) -> None:
             """Create connection between cortical areas"""
         
         def validate(self) -> List[str]:
             """Validate genome structure"""
     ```

2. **Cortical Area Class**
   - [ ] Create `feagi/genomics/cortical_area.py`
   - [ ] Implement `CorticalArea` class:
     ```python
     class CorticalArea:
         def __init__(self, name: str, dimensions: Tuple[int, int, int]):
             """Create cortical area"""
         
         def set_group_id(self, group_id: str) -> None:
             """Set cortical group"""
         
         def set_coordinates_2d(self, x: int, y: int) -> None:
             """Set 2D brain coordinates"""
         
         def set_coordinates_3d(self, x: int, y: int, z: int) -> None:
             """Set 3D brain coordinates"""
         
         def add_neuron(self, position: Tuple[int, int, int], 
                       **kwargs) -> Neuron:
             """Add neuron to this cortical area"""
     ```

3. **Neuron Manipulation**
   - [ ] Create `feagi/genomics/neurons.py`
   - [ ] Implement neuron utilities:
     ```python
     def create_neuron(cortical_area: str, 
                       position: Tuple[int, int, int],
                       neuron_type: str = "EXCITATORY") -> Neuron:
         """Create a neuron"""
     
     def bulk_create_neurons(cortical_area: str,
                             count: int,
                             distribution: str = "uniform") -> List[Neuron]:
         """Create multiple neurons"""
     ```

4. **Synapse Manipulation**
   - [ ] Create `feagi/genomics/synapses.py`
   - [ ] Implement synapse utilities:
     ```python
     def connect_neurons(source: Neuron, 
                        target: Neuron,
                        weight: float = 1.0,
                        plasticity: Optional[PlasticityRule] = None) -> Synapse:
         """Connect two neurons"""
     
     def connect_areas(source_area: CorticalArea,
                      target_area: CorticalArea,
                      connectivity: float = 0.1,
                      weight_distribution: str = "gaussian") -> List[Synapse]:
         """Connect two cortical areas"""
     ```

5. **Genome Templates**
   - [ ] Create `feagi/genomics/templates.py`
   - [ ] Implement template functions:
     ```python
     def vision_template() -> Genome:
         """Create a basic vision-enabled genome"""
     
     def motor_control_template() -> Genome:
         """Create a basic motor control genome"""
     
     def sensorimotor_template() -> Genome:
         """Create a full sensorimotor genome"""
     
     def custom_template(**kwargs) -> Genome:
         """Create genome from custom spec"""
     ```

6. **Integration with REST API**
   - [ ] Connect `Genome` class to REST API
   - [ ] Implement `.apply()` method to push changes to running FEAGI
   - [ ] Implement `.pull()` method to fetch current state

7. **Testing**
   - [ ] Test genome creation
   - [ ] Test cortical area operations
   - [ ] Test neuron/synapse creation
   - [ ] Test templates
   - [ ] Test file I/O (JSON genome format)
   - [ ] Test validation

#### Deliverables
- ✅ Full genomics API
- ✅ Cortical area CRUD
- ✅ Neuron/synapse manipulation
- ✅ Genome templates
- ✅ Integration with REST API
- ✅ Documentation with examples

---

### Phase 6: Connectome API (Weeks 13-14)

#### Goals
- Query connectome structure
- Extract metrics
- Analyze topology
- Provide visualization helpers

#### Tasks

1. **Connectome Query API**
   - [ ] Create `feagi/connectome/query.py`
   - [ ] Implement query functions:
     ```python
     def get_neuron_count() -> int:
         """Get total neuron count"""
     
     def get_synapse_count() -> int:
         """Get total synapse count"""
     
     def get_neurons_in_area(area_id: str) -> List[Neuron]:
         """Get all neurons in cortical area"""
     
     def get_synapses_from_neuron(neuron_id: str) -> List[Synapse]:
         """Get all synapses from a neuron"""
     
     def get_synapses_to_neuron(neuron_id: str) -> List[Synapse]:
         """Get all synapses to a neuron"""
     
     def get_connectivity_matrix(source_area: str, 
                                 target_area: str) -> np.ndarray:
         """Get connectivity matrix between two areas"""
     ```

2. **Metrics API**
   - [ ] Create `feagi/connectome/metrics.py`
   - [ ] Implement metrics functions:
     ```python
     def calculate_connectivity_density(area_id: str) -> float:
         """Calculate connectivity density"""
     
     def calculate_firing_rate(area_id: str, 
                               window_ms: float = 100.0) -> float:
         """Calculate average firing rate"""
     
     def calculate_network_efficiency() -> float:
         """Calculate global network efficiency"""
     
     def get_area_statistics(area_id: str) -> AreaStatistics:
         """Get comprehensive area statistics"""
     ```

3. **Topology Analysis**
   - [ ] Create `feagi/connectome/topology.py`
   - [ ] Implement topology functions:
     ```python
     def find_connected_components() -> List[Set[str]]:
         """Find connected components in network"""
     
     def find_shortest_path(source_area: str, 
                           target_area: str) -> List[str]:
         """Find shortest path between areas"""
     
     def calculate_clustering_coefficient(area_id: str) -> float:
         """Calculate clustering coefficient"""
     
     def detect_modules() -> Dict[str, List[str]]:
         """Detect functional modules"""
     ```

4. **Visualization Helpers**
   - [ ] Create `feagi/connectome/visualization.py`
   - [ ] Implement visualization utilities:
     ```python
     def export_for_graphviz(output_path: str) -> None:
         """Export connectome as Graphviz DOT file"""
     
     def export_for_cytoscape(output_path: str) -> None:
         """Export connectome for Cytoscape"""
     
     def generate_connectivity_heatmap(area1: str, 
                                       area2: str) -> np.ndarray:
         """Generate connectivity heatmap"""
     ```

5. **Testing**
   - [ ] Test all query functions
   - [ ] Test metrics calculations
   - [ ] Test topology analysis
   - [ ] Test visualization exports

#### Deliverables
- ✅ Connectome query API
- ✅ Metrics calculation
- ✅ Topology analysis
- ✅ Visualization helpers
- ✅ Documentation with examples

---

### Phase 7: Embedded API (Weeks 15-17)

#### Goals
- Export connectomes for embedded devices
- Support ESP32, Arduino, STM32, RPi Pico
- Provide flashing utilities
- Memory optimization and quantization

#### Tasks

1. **Base Exporter**
   - [ ] Create `feagi/embedded/exporter.py`
   - [ ] Implement `ConnectomeExporter` class:
     ```python
     class ConnectomeExporter:
         def __init__(self, genome_path: str):
             """Initialize exporter with genome"""
         
         def export(self, 
                   platform: str,
                   output_path: str,
                   config: EmbeddedConfig) -> ExportResult:
             """Export connectome for platform"""
         
         def estimate_memory(self, platform: str) -> MemoryEstimate:
             """Estimate memory requirements"""
         
         def optimize(self, config: OptimizationConfig) -> None:
             """Optimize connectome for embedded"""
     ```

2. **Platform-Specific Modules**
   
   **ESP32** (`feagi/embedded/platforms/esp32.py`):
   ```python
   class ESP32Config:
       memory_budget: int  # bytes
       quantization: str   # "int8", "int16", "fp16"
       flash_size: int     # bytes
       
   class ESP32Exporter:
       def export(self, genome: Genome, 
                 output_dir: str,
                 config: ESP32Config) -> None:
           """Export for ESP32"""
       
       def flash(self, binary_path: str, port: str) -> None:
           """Flash ESP32 device"""
   ```
   
   **Arduino** (`feagi/embedded/platforms/arduino.py`):
   ```python
   class ArduinoConfig:
       board: str          # "uno", "due", "mega"
       memory_budget: int
       
   class ArduinoExporter:
       def export(self, genome: Genome,
                 output_dir: str,
                 config: ArduinoConfig) -> None:
           """Export for Arduino"""
   ```
   
   **STM32** (`feagi/embedded/platforms/stm32.py`):
   ```python
   class STM32Config:
       chip: str           # "stm32f4", "stm32f7"
       memory_budget: int
       use_dma: bool
       
   class STM32Exporter:
       def export(self, genome: Genome,
                 output_dir: str,
                 config: STM32Config) -> None:
           """Export for STM32"""
   ```
   
   **Raspberry Pi Pico** (`feagi/embedded/platforms/rpi_pico.py`):
   ```python
   class RPiPicoConfig:
       memory_budget: int
       use_flash: bool
       
   class RPiPicoExporter:
       def export(self, genome: Genome,
                 output_dir: str,
                 config: RPiPicoConfig) -> None:
           """Export for RPi Pico"""
   ```

3. **Flashing Utilities**
   - [ ] Create `feagi/embedded/flash.py`
   - [ ] Implement flashing functions:
     ```python
     def flash_esp32(binary: str, port: str, 
                    baud_rate: int = 921600) -> bool:
         """Flash ESP32 via esptool"""
     
     def flash_arduino(hex_file: str, port: str, 
                      board: str) -> bool:
         """Flash Arduino via avrdude"""
     
     def flash_stm32(binary: str, interface: str = "swd") -> bool:
         """Flash STM32 via st-link"""
     
     def detect_devices() -> List[Device]:
         """Detect connected embedded devices"""
     ```

4. **Memory Optimization**
   - [ ] Implement quantization (FP32 → INT8/INT16)
   - [ ] Implement pruning (remove low-weight synapses)
   - [ ] Implement compression (sparse matrix formats)

5. **Testing**
   - [ ] Test export for all platforms
   - [ ] Test memory estimation
   - [ ] Test optimization
   - [ ] Integration tests with actual hardware (if available)

#### Deliverables
- ✅ Embedded export API
- ✅ Platform-specific exporters (ESP32, Arduino, STM32, RPi Pico)
- ✅ Flashing utilities
- ✅ Memory optimization
- ✅ Documentation with hardware guides

---

### Phase 8: MCP Server (Weeks 18-19)

#### Goals
- Create MCP (Model Context Protocol) server
- Expose FEAGI operations to AI agents
- Enable LLMs to interact with FEAGI
- Support tool discovery and invocation

#### Tasks

1. **MCP Server Implementation**
   - [ ] Create `feagi/mcp/server.py`
   - [ ] Implement MCP protocol:
     ```python
     class FeagiMCPServer:
         def __init__(self, feagi_url: str = "http://localhost:8000"):
             """Initialize MCP server"""
         
         def start(self, port: int = 3000) -> None:
             """Start MCP server"""
         
         def register_tools(self) -> None:
             """Register all FEAGI tools"""
     ```

2. **Tool Definitions**
   - [ ] Create `feagi/mcp/tools.py`
   - [ ] Define MCP tools for:
     - Engine control (start, stop, status)
     - Genome loading
     - Cortical area creation
     - Neuron manipulation
     - Connectome queries
     - Runtime control
     - Configuration management

3. **Tool Schemas**
   - [ ] Create `feagi/mcp/schemas.py`
   - [ ] Define JSON schemas for all tools
   - [ ] Include descriptions, parameters, return types

4. **Example Tool Definition**
   ```python
   @mcp_tool
   def create_cortical_area(
       name: str,
       width: int,
       height: int,
       depth: int,
       group_id: Optional[str] = None
   ) -> Dict[str, Any]:
       """
       Create a new cortical area in the FEAGI brain.
       
       Args:
           name: Unique name for the cortical area
           width: Width dimension
           height: Height dimension
           depth: Depth dimension (number of layers)
           group_id: Optional cortical group ID
       
       Returns:
           Created cortical area details
       """
       # Implementation
   ```

5. **CLI Entry Point**
   - [ ] Add `feagi-mcp` command:
     ```bash
     feagi-mcp start --port 3000 --feagi-url http://localhost:8000
     ```

6. **Testing**
   - [ ] Test MCP protocol implementation
   - [ ] Test tool invocation
   - [ ] Test with actual MCP clients (Claude Desktop, etc.)

#### Deliverables
- ✅ MCP server implementation
- ✅ All FEAGI operations exposed as MCP tools
- ✅ CLI entry point
- ✅ Documentation for AI agent integration
- ✅ Example MCP configurations

---

### Phase 9: Documentation & Examples (Weeks 20-21)

#### Goals
- Comprehensive documentation
- API reference
- Tutorials and guides
- Example applications

#### Tasks

1. **API Reference**
   - [ ] Use Sphinx or MkDocs
   - [ ] Auto-generate from docstrings
   - [ ] Cover all public APIs
   - [ ] Include type hints

2. **Tutorials**
   - [ ] Getting Started
   - [ ] Engine Management
   - [ ] Building Your First Agent
   - [ ] Creating Custom Genomes
   - [ ] Analyzing Connectomes
   - [ ] Deploying to Embedded Devices
   - [ ] Using with AI Agents (MCP)

3. **Examples**
   - [ ] Simple vision agent
   - [ ] Motor control agent
   - [ ] Custom genome creation
   - [ ] Connectome analysis script
   - [ ] ESP32 deployment
   - [ ] MCP integration with Claude

4. **Migration Guides**
   - [ ] Migrating from `feagi-connector` to `feagi`
   - [ ] Breaking changes
   - [ ] Deprecation warnings

#### Deliverables
- ✅ Complete API reference
- ✅ Comprehensive tutorials
- ✅ Working examples
- ✅ Migration guides

---

### Phase 10: Testing, Polish & Release (Weeks 22-24)

#### Goals
- Comprehensive testing
- Performance optimization
- CI/CD pipeline
- PyPI release

#### Tasks

1. **Testing**
   - [ ] Unit tests (>90% coverage)
   - [ ] Integration tests
   - [ ] End-to-end tests
   - [ ] Performance benchmarks
   - [ ] Type checking (mypy strict)
   - [ ] Linting (ruff)

2. **CI/CD**
   - [ ] GitHub Actions workflow
   - [ ] Automated testing on push
   - [ ] Automated PyPI release on tag
   - [ ] Documentation deployment

3. **Performance**
   - [ ] Profile critical paths
   - [ ] Optimize REST API calls (caching, batching)
   - [ ] Optimize data serialization

4. **Polish**
   - [ ] Review all error messages
   - [ ] Improve logging
   - [ ] Add progress bars (for long operations)
   - [ ] Add deprecation warnings for old APIs

5. **Release**
   - [ ] Final version bump
   - [ ] CHANGELOG update
   - [ ] Release notes
   - [ ] PyPI publication
   - [ ] Announcement

#### Deliverables
- ✅ Full test suite passing
- ✅ CI/CD pipeline operational
- ✅ Performance optimized
- ✅ PyPI release: `pip install feagi`

---

## Package Distribution Strategy

### PyPI Package: `feagi`

**Package Name:** `feagi` (NOT `feagi-connector`)

**Version:** `3.0.0` (major version bump to indicate complete rewrite)

**Installation Options:**

```bash
# Default installation - automatically gets the best option for your platform
pip install feagi

# What you get depends on your platform:
# - Supported platforms (macOS/Linux/Windows x64): Platform wheel with bundled binary (~50MB)
# - Other platforms: Pure Python wheel without binary (~5MB)

# Optional extras (add functionality):
pip install "feagi[video]"        # + Vision/video processing (OpenCV)
pip install "feagi[embedded]"     # + Embedded device support (esptool, etc.)
pip install "feagi[mcp]"          # + MCP server for AI agents
pip install "feagi[dev]"          # + Development tools (pytest, mypy, ruff)
pip install "feagi[all]"          # + Everything

# Combine extras:
pip install "feagi[video,embedded,mcp]"
```

**What Gets Installed:**

| Your Platform | Default `pip install feagi` | Size | Bundled Binary |
|---------------|----------------------------|------|----------------|
| **macOS ARM64** | Platform wheel (best experience) | ~50MB | ✅ Yes |
| **macOS Intel** | Platform wheel (best experience) | ~50MB | ✅ Yes |
| **Linux x86_64** | Platform wheel (best experience) | ~50MB | ✅ Yes |
| **Windows x64** | Platform wheel (best experience) | ~50MB | ✅ Yes |
| **Other platforms** | Pure Python wheel (fallback) | ~5MB | ❌ No |

**Note:** If you get the pure Python wheel (no binary), you'll need to:
- Install FEAGI separately: `cargo install feagi`, OR
- Connect to remote FEAGI: `export FEAGI_REMOTE_URL=http://...`

**Advanced: Force Pure Python Wheel (Client-Only Mode)**

```bash
# For advanced users who want lightweight install even on supported platforms:
pip install feagi --no-binary feagi

# Use cases:
# - Already have FEAGI installed (cargo install feagi)
# - Connecting to remote FEAGI server only
# - Building minimal Docker images
# - CI/CD pipelines (faster installs)
```

### Rust Binary Distribution: **Platform-Specific Wheels (APPROVED)**

**Decision:** Use bundled pre-built binaries in platform-specific wheels for optimal user experience.

**Rationale:**
- ✅ Best user experience: Just `pip install feagi` and it works
- ✅ No external dependencies required
- ✅ Guaranteed binary-SDK version compatibility
- ✅ PyPI hosting is free for open source
- ✅ Zero monetary cost (only developer time investment)

**Platform Support:**
```
feagi-3.0.0-py3-none-macosx_11_0_arm64.whl      (~50MB) - macOS Apple Silicon
feagi-3.0.0-py3-none-macosx_10_9_x86_64.whl    (~50MB) - macOS Intel
feagi-3.0.0-py3-none-manylinux_2_17_x86_64.whl (~50MB) - Linux x86_64
feagi-3.0.0-py3-none-win_amd64.whl             (~50MB) - Windows x86_64
```

**Total PyPI Storage:** ~200MB per version (well within 10GB limit)

**Fallback Strategy:**
If user's platform is not supported by pre-built wheels:
1. Display helpful error message
2. Point to manual installation: `cargo install feagi`
3. Provide links to GitHub releases
4. Document how to build from source

**User Experience:**
```bash
# User runs on any supported platform:
pip install feagi
# ✅ Automatically downloads correct wheel for their platform
# ✅ Binary is bundled and ready to use
# ✅ No additional steps required

# Python code just works:
python
>>> from feagi import engine
>>> engine.start()  # Uses bundled binary immediately
🦀 FEAGI Engine starting...
✅ Engine ready at http://localhost:8000
```

---

## Binary Distribution: Implementation Details

### Architecture Decision: Platform-Specific Wheels with Bundled Binaries

**Status:** ✅ **APPROVED** - Prioritizing user experience

**Cost Analysis:**
- **PyPI Storage**: $0 (free for open source)
- **GitHub Actions CI/CD**: $0 (within free tier limits)
- **Developer Time**: 60-80 hours initial setup, 2-4 hours per release
- **Total Monetary Cost**: **$0/month** ✅

### Build Process Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    GitHub Actions Workflow                   │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  1. Trigger on Git Tag (v3.0.0)                              │
│                                                               │
│  2. Build Matrix (Parallel):                                 │
│     ┌─────────────┬─────────────┬─────────────┬────────────┐│
│     │  macOS ARM  │ macOS Intel │  Linux x64  │ Windows x64││
│     │  (20 min)   │  (20 min)   │  (15 min)   │  (25 min)  ││
│     └─────────────┴─────────────┴─────────────┴────────────┘│
│                                                               │
│  3. For Each Platform:                                       │
│     a. Checkout code                                         │
│     b. Set up Rust + Python                                  │
│     c. Build FEAGI Rust binary (cargo build --release)       │
│     d. Strip debug symbols (50MB → 50MB optimized)           │
│     e. Copy binary to feagi/engine/bin/                      │
│     f. Build Python wheel with bundled binary                │
│     g. Upload artifact                                       │
│                                                               │
│  4. Publish to PyPI:                                         │
│     - feagi-3.0.0-py3-none-macosx_11_0_arm64.whl      50MB  │
│     - feagi-3.0.0-py3-none-macosx_10_9_x86_64.whl    50MB  │
│     - feagi-3.0.0-py3-none-manylinux_2_17_x86_64.whl 50MB  │
│     - feagi-3.0.0-py3-none-win_amd64.whl             50MB  │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

### Package Structure in Each Wheel

```
feagi-3.0.0-py3-none-macosx_11_0_arm64.whl (50MB)
└── feagi/
    ├── __init__.py
    ├── engine/
    │   ├── __init__.py
    │   ├── manager.py
    │   ├── process.py
    │   ├── binaries.py
    │   └── bin/
    │       └── feagi              ← Bundled Rust binary (macOS ARM64)
    ├── config/
    ├── connector/
    ├── genomics/
    ├── connectome/
    ├── embedded/
    ├── api/
    ├── mcp/
    └── types/
```

### CI/CD Build Configuration

**File: `.github/workflows/build-wheels.yml`**

```yaml
name: Build and Publish Platform Wheels

on:
  push:
    tags:
      - 'v*.*.*'
  workflow_dispatch:
    inputs:
      version:
        description: 'Version to build (e.g., 3.0.0)'
        required: true

env:
  CARGO_TERM_COLOR: always
  RUST_BACKTRACE: 1

jobs:
  build-wheels:
    name: Build ${{ matrix.platform }}
    runs-on: ${{ matrix.os }}
    
    strategy:
      fail-fast: false
      matrix:
        include:
          # macOS ARM64 (Apple Silicon M1/M2/M3)
          - os: macos-14
            target: aarch64-apple-darwin
            platform: macosx_11_0_arm64
            binary_name: feagi
          
          # macOS x86_64 (Intel)
          - os: macos-13
            target: x86_64-apple-darwin
            platform: macosx_10_9_x86_64
            binary_name: feagi
          
          # Linux x86_64 (manylinux for wide compatibility)
          - os: ubuntu-20.04
            target: x86_64-unknown-linux-gnu
            platform: manylinux_2_17_x86_64
            binary_name: feagi
          
          # Windows x86_64
          - os: windows-latest
            target: x86_64-pc-windows-msvc
            platform: win_amd64
            binary_name: feagi.exe
    
    steps:
      - name: Checkout repository
        uses: actions/checkout@v4
        with:
          submodules: recursive
      
      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.10'
      
      - name: Set up Rust
        uses: dtolnay/rust-toolchain@stable
        with:
          targets: ${{ matrix.target }}
      
      - name: Cache Rust dependencies
        uses: actions/cache@v3
        with:
          path: |
            ~/.cargo/registry
            ~/.cargo/git
            feagi/target
          key: ${{ runner.os }}-cargo-${{ matrix.target }}-${{ hashFiles('**/Cargo.lock') }}
          restore-keys: |
            ${{ runner.os }}-cargo-${{ matrix.target }}-
      
      - name: Install build tools
        run: |
          python -m pip install --upgrade pip
          pip install build wheel setuptools-rust
      
      - name: Build FEAGI Rust binary
        working-directory: feagi
        run: |
          cargo build --release --target ${{ matrix.target }}
          ls -lh target/${{ matrix.target }}/release/
      
      - name: Strip binary (Unix)
        if: runner.os != 'Windows'
        working-directory: feagi
        run: |
          strip target/${{ matrix.target }}/release/${{ matrix.binary_name }}
          ls -lh target/${{ matrix.target }}/release/${{ matrix.binary_name }}
      
      - name: Prepare Python package with binary
        shell: bash
        run: |
          mkdir -p feagi-connector/feagi/engine/bin
          cp feagi/target/${{ matrix.target }}/release/${{ matrix.binary_name }} \
             feagi-connector/feagi/engine/bin/${{ matrix.binary_name }}
          chmod +x feagi-connector/feagi/engine/bin/${{ matrix.binary_name }} || true
      
      - name: Build Python wheel
        working-directory: feagi-connector
        run: |
          python -m build --wheel
          ls -lh dist/
      
      - name: Verify wheel contents
        working-directory: feagi-connector
        shell: bash
        run: |
          pip install wheel
          python -c "import zipfile; z = zipfile.ZipFile('dist/feagi-3.0.0-py3-none-${{ matrix.platform }}.whl'); print('\n'.join(z.namelist()))"
      
      - name: Upload wheel artifact
        uses: actions/upload-artifact@v4
        with:
          name: wheel-${{ matrix.platform }}
          path: feagi-connector/dist/*.whl
          retention-days: 7
  
  publish-to-pypi:
    name: Publish to PyPI
    needs: build-wheels
    runs-on: ubuntu-latest
    permissions:
      id-token: write  # For trusted publishing
    
    steps:
      - name: Download all wheel artifacts
        uses: actions/download-artifact@v4
        with:
          pattern: wheel-*
          path: dist
          merge-multiple: true
      
      - name: List wheels
        run: |
          ls -lh dist/
          du -sh dist/
      
      - name: Publish to PyPI
        uses: pypa/gh-action-pypi-publish@release/v1
        with:
          packages-dir: dist/
          skip-existing: true
          verbose: true
```

### Single Package, Multiple Wheels Strategy

**One Package Name, Flexible Installation:**

The `feagi` package publishes multiple wheel types:
- **Platform-specific wheels** (with bundled binary) - for common platforms
- **Pure Python wheel** (no binary) - universal fallback

Pip automatically chooses the best wheel for your platform.

**Implementation:**

```
feagi-connector/
├── pyproject.toml              # Single package config
├── feagi/
│   ├── engine/
│   │   ├── manager.py          # Smart engine manager
│   │   └── bin/                # Included in platform wheels only
│   │       └── feagi           # Bundled binary
│   ├── config/
│   ├── connector/
│   ├── genomics/
│   ├── connectome/
│   ├── embedded/
│   └── api/
```

**Build Process:**

```bash
# Build platform-specific wheels (with binary) - one per platform
python -m build --wheel  # Creates feagi-3.0.0-py3-none-macosx_11_0_arm64.whl

# Build universal pure Python wheel (no binary) - once
python -m build --wheel --config-setting=pure-python=true  # Creates feagi-3.0.0-py3-none-any.whl
```

**Smart Engine Manager:**

The engine manager automatically detects the available options:

```python
# feagi/engine/manager.py

class FeagiEngineManager:
    def __init__(self, remote_url: Optional[str] = None):
        """
        Initialize engine manager.
        
        Args:
            remote_url: If provided, connects to remote FEAGI server
                       instead of starting local engine.
        """
        self.remote_url = remote_url or os.getenv("FEAGI_REMOTE_URL")
        self._binary_path = None
        self._process = None
    
    def start(self, **kwargs) -> bool:
        """
        Start FEAGI engine using best available method:
        1. If remote_url is set → connect to remote (no local start)
        2. If bundled binary exists → use bundled binary
        3. If system binary in PATH → use system binary
        4. Raise error with installation instructions
        """
        # Remote mode: just validate connection
        if self.remote_url:
            return self._connect_to_remote()
        
        # Local mode: find and start binary
        binary = self._find_binary()
        return self._start_local(binary, **kwargs)
    
    def _find_binary(self) -> Path:
        """Find FEAGI binary (bundled, system, or error)"""
        # 1. Try bundled binary (only in `feagi` package)
        try:
            bundled = self._get_bundled_binary()
            if bundled.exists():
                logger.info(f"Using bundled FEAGI binary: {bundled}")
                return bundled
        except RuntimeError:
            pass  # Bundled binary not available (feagi-client)
        
        # 2. Try system binary (from PATH)
        system_binary = shutil.which("feagi")
        if system_binary:
            logger.info(f"Using system FEAGI binary: {system_binary}")
            return Path(system_binary)
        
        # 3. Give up with helpful error
        raise RuntimeError(
            "FEAGI engine binary not found.\n"
            "\n"
            "Options:\n"
            "1. Install full package: pip uninstall feagi-client && pip install feagi\n"
            "2. Install Rust binary: cargo install feagi\n"
            "3. Connect to remote: set FEAGI_REMOTE_URL environment variable\n"
            "4. Download from: https://github.com/Neuraville/FEAGI-2.0/releases\n"
        )
    
    def _connect_to_remote(self) -> bool:
        """Connect to remote FEAGI server"""
        try:
            response = requests.get(f"{self.remote_url}/v1/health", timeout=5)
            response.raise_for_status()
            logger.info(f"✅ Connected to remote FEAGI at {self.remote_url}")
            return True
        except Exception as e:
            raise RuntimeError(
                f"Failed to connect to remote FEAGI at {self.remote_url}: {e}"
            )
```

**Usage Examples:**

```python
# Example 1: Platform wheel with bundled binary (automatic on supported platforms)
# pip install feagi
from feagi import engine
engine.start()  # Uses bundled binary automatically

# Example 2: Pure Python wheel with system binary
# cargo install feagi  # Install FEAGI via Rust
# pip install feagi --no-binary feagi  # Force pure Python wheel
from feagi import engine
engine.start()  # Uses system binary from PATH

# Example 3: Pure Python wheel with remote server
# pip install feagi --no-binary feagi  # Lightweight install
import os
os.environ["FEAGI_REMOTE_URL"] = "http://feagi-server.example.com:8000"
from feagi import engine
engine.start()  # Connects to remote, doesn't start local

# Example 4: Explicit remote connection (works with any wheel type)
from feagi import engine
engine_manager = engine.FeagiEngineManager(
    remote_url="http://192.168.1.100:8000"
)
engine_manager.start()

# All other SDK features work the same regardless of wheel type:
from feagi import genomics, connectome, embedded
genome = genomics.create_genome("brain")
metrics = connectome.get_metrics()
embedded.export("brain.json", "esp32")  # No engine needed
```

**Package Build Configuration:**

```toml
# pyproject.toml
[project]
name = "feagi"
version = "3.0.0"

[tool.setuptools.packages.find]
include = ["feagi*"]

# For platform wheels: include bundled binaries
[tool.setuptools.package-data]
feagi = ["engine/bin/*"]

# For pure Python wheel: exclude binaries (set package-data to empty in build script)
```

**CI/CD for Multiple Wheel Types:**

```yaml
# .github/workflows/build-wheels.yml (extended)
jobs:
  # Job 1: Build platform-specific wheels with binaries
  build-platform-wheels:
    # ... (existing workflow)
    # Produces: feagi-3.0.0-py3-none-{platform}.whl (50MB each)
  
  # Job 2: Build pure Python wheel without binary
  build-universal-wheel:
    name: Build universal wheel (no binary)
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.10'
      
      - name: Build pure Python wheel
        working-directory: feagi-connector
        run: |
          # Temporarily exclude binary from package data
          python scripts/build_pure_wheel.py
          ls -lh dist/  # Should be ~5MB, py3-none-any.whl
      
      - name: Upload universal wheel
        uses: actions/upload-artifact@v4
        with:
          name: universal-wheel
          path: feagi-connector/dist/*-py3-none-any.whl
  
  publish:
    needs: [build-platform-wheels, build-universal-wheel]
    # Publishes all wheels to PyPI
    # Pip will automatically choose the best one
```

**Pip Resolution Priority:**

When you run `pip install feagi`, pip selects wheels in this order:

1. **Platform-specific wheel** (if available for your platform) - **~50MB with binary**
2. **Pure Python wheel** (universal fallback) - **~5MB without binary**

To force pure Python wheel: `pip install feagi --no-binary feagi`

**Benefits of Single Package Approach:**

| Benefit | Platform Wheel | Pure Python Wheel |
|---------|----------------|-------------------|
| **Quick start** | ✅ Install and go | ❌ Requires setup |
| **Small downloads** | ❌ 50MB | ✅ 5MB |
| **Works offline** | ✅ Yes | ✅ If binary installed |
| **Remote-only use** | ✅ Yes | ✅ Yes (optimized) |
| **Docker-friendly** | ⚠️ Larger layers | ✅ Smaller layers |
| **CI/CD** | ⚠️ Slower installs | ✅ Faster installs |
| **Embedded dev** | ✅ Yes | ✅ Yes (same SDK) |

**Target Audiences:**

**Platform Wheel (Automatic on macOS/Linux/Windows):**
- Data scientists and researchers
- Students and educators
- Local development
- Tutorials and workshops
- Quick prototyping

**Pure Python Wheel (Automatic on other platforms, or via `--no-binary`):**
- Production deployments with separate FEAGI server
- Microservices connecting to FEAGI cluster
- CI/CD pipelines (faster installs)
- Minimal Docker containers
- Embedded device development (no local engine needed)
- Users with cargo-installed FEAGI
- ARM64 Linux servers (until platform wheel added)

### Advanced Installation Examples

**Scenario 1: Genomics Development with Remote FEAGI**
```bash
# Lightweight install (pure Python wheel)
pip install feagi --no-binary feagi

# Use genomics API with remote FEAGI server
export FEAGI_REMOTE_URL="http://feagi-cluster.local:8000"
```

```python
from feagi import genomics, api

# Create genome programmatically (no engine needed)
genome = genomics.create_genome("custom_brain")
genome.add_cortical_area("v1", dimensions=(128, 128, 10))
genome.save("custom_brain.json")

# Upload to remote FEAGI
api_client = api.FeagiAPI("http://feagi-cluster.local:8000")
api_client.genome.load_genome("custom_brain.json")
```

**Scenario 2: Embedded Device Development**
```bash
# Lightweight install with embedded tools
pip install "feagi[embedded]" --no-binary feagi
# Smaller install: ~8MB vs 50MB
```

```python
from feagi import embedded

# Export for ESP32 (no local FEAGI engine needed)
esp32_config = embedded.platforms.esp32.ESP32Config(
    memory_budget=512_000,
    quantization="int8"
)

# This works without FEAGI engine running
embedded.export("brain.json", "esp32", esp32_config)
embedded.flash("esp32", port="/dev/ttyUSB0")
```

**Scenario 3: Connectome Analysis Microservice**
```bash
# Lightweight microservice connecting to remote FEAGI
pip install feagi --no-binary feagi
```

```python
from feagi import connectome, api
from fastapi import FastAPI

app = FastAPI()

# Connect to FEAGI cluster
api.set_base_url("http://feagi-cluster.local:8000")

@app.get("/metrics")
def get_brain_metrics():
    # Query remote FEAGI connectome
    metrics = connectome.get_metrics()
    return {
        "neurons": metrics.neuron_count,
        "synapses": metrics.synapse_count,
        "density": metrics.connectivity_density
    }

@app.get("/topology/{area_id}")
def get_area_topology(area_id: str):
    # Analyze specific cortical area
    stats = connectome.get_area_statistics(area_id)
    return stats
```

**Scenario 4: CI/CD Pipeline**
```bash
# .gitlab-ci.yml or .github/workflows/test.yml
pip install feagi --no-binary feagi  # Fast install, 5MB
pytest tests/  # Tests that validate genomes, connectomes, etc.
```

**Scenario 5: Docker Multi-Stage Build**
```dockerfile
# Stage 1: Build/test with pure Python wheel
FROM python:3.10-slim AS builder
RUN pip install feagi --no-binary feagi  # Lightweight
COPY . /app
WORKDIR /app
RUN pytest

# Stage 2: Production with platform wheel
FROM python:3.10-slim AS production
RUN pip install feagi  # Gets platform wheel with binary automatically
COPY --from=builder /app /app
WORKDIR /app
CMD ["python", "main.py"]
```

**Scenario 6: Custom Deployment with Separate FEAGI**
```bash
# Deploy FEAGI engine separately (e.g., via Docker Compose)
docker run -d -p 8000:8000 feagi/feagi:latest

# Install lightweight SDK in your application
pip install feagi --no-binary feagi

# Your app connects to the containerized FEAGI
export FEAGI_REMOTE_URL="http://localhost:8000"
```

```python
# app.py
from feagi import connector, api

# All SDK functionality works with remote FEAGI
agent = connector.create_agent("robot_agent")
agent.register_sensory("camera", cortical_area="v1")

# Send data to remote FEAGI
agent.send_sensory_data("camera", image_bytes)
```

### Installation Decision Tree

```
Do you need to run FEAGI locally?
│
├─ YES → pip install feagi
│         (gets platform wheel with bundled binary automatically)
│
└─ NO (using remote FEAGI) → pip install feagi --no-binary feagi
                              (gets lightweight pure Python wheel)

Do you need specific extras?
│
├─ Video processing → add [video]
├─ Embedded export  → add [embedded]
├─ MCP server       → add [mcp]
├─ Development tools→ add [dev]
├─ Everything       → add [all]
└─ None             → use base package

Examples:
  # Most users (automatic best choice):
  pip install feagi
  
  # With extras:
  pip install "feagi[video,embedded]"
  
  # Lightweight for remote FEAGI:
  pip install feagi --no-binary feagi
  
  # Lightweight with extras:
  pip install "feagi[embedded]" --no-binary feagi
```

**Key Insight:** The `--no-binary feagi` flag is the "advanced user" mode. Most users just run `pip install feagi` and get the best experience automatically.

### Build Optimization Strategies

**1. Binary Size Reduction**

```toml
# feagi/Cargo.toml
[profile.release]
opt-level = 3           # Maximum optimization
lto = "fat"             # Link-time optimization
codegen-units = 1       # Single codegen unit for better optimization
strip = true            # Strip debug symbols
panic = 'abort'         # Reduce binary size slightly
```

**Impact:**
- Unoptimized debug build: ~150MB
- Release build: ~80MB
- With strip: ~50MB
- With UPX (optional): ~15MB

**2. Caching Strategy**

GitHub Actions caches:
- Rust dependencies: `~/.cargo`
- Build artifacts: `feagi/target`
- Reduces build time: 25 min → 10 min on cache hit

**3. Parallel Builds**

All 4 platforms build in parallel:
- **Wall time**: ~25 minutes (longest build)
- **Total CPU time**: ~80 minutes (4 × 20 min average)
- **Cost**: $0 (within GitHub Actions free tier)

### Version Compatibility Management

**SDK to Binary Version Mapping:**

```python
# feagi/engine/version.py

SDK_VERSION = "3.0.0"

# Map SDK versions to compatible FEAGI binary versions
COMPATIBILITY_MATRIX = {
    "3.0.0": ["2.0.0", "2.0.1"],
    "3.0.1": ["2.0.1", "2.0.2"],
    "3.1.0": ["2.1.0", "2.1.1"],
}

def check_compatibility(binary_version: str) -> bool:
    """Check if binary version is compatible with this SDK"""
    compatible = COMPATIBILITY_MATRIX.get(SDK_VERSION, [])
    return binary_version in compatible

def get_required_binary_version() -> str:
    """Get the recommended binary version for this SDK"""
    compatible = COMPATIBILITY_MATRIX.get(SDK_VERSION, [])
    return compatible[0] if compatible else "2.0.0"
```

### Testing Strategy

**1. Local Testing (Pre-CI)**
```bash
# Test building wheel for current platform
cd feagi
cargo build --release
cd ../feagi-connector
mkdir -p feagi/engine/bin
cp ../feagi/target/release/feagi feagi/engine/bin/
python -m build --wheel
pip install dist/*.whl
python -c "from feagi import engine; print(engine.get_bundled_binary())"
```

**2. CI Testing (Automated)**
- Build on all 4 platforms
- Verify binary is executable
- Check binary version compatibility
- Test `engine.start()` and `engine.stop()`

**3. Post-Release Testing**
```bash
# Verify PyPI distribution
pip install feagi --no-cache-dir
python -c "from feagi import engine; engine.start()"
```

### Deployment Checklist

**Before each release:**

- [ ] Update version in `feagi-connector/pyproject.toml`
- [ ] Update version in `feagi/Cargo.toml`
- [ ] Update `COMPATIBILITY_MATRIX` in `version.py`
- [ ] Test build locally on at least one platform
- [ ] Update CHANGELOG.md
- [ ] Create git tag: `git tag v3.0.0`
- [ ] Push tag: `git push origin v3.0.0`
- [ ] Monitor GitHub Actions build
- [ ] Verify all 4 wheels uploaded successfully
- [ ] Test installation: `pip install feagi`
- [ ] Announce release

### Platform Support Matrix

| Platform | Arch | Status | Wheel Name | Binary Size |
|----------|------|--------|------------|-------------|
| macOS | ARM64 (M1/M2/M3) | ✅ Supported | `macosx_11_0_arm64` | ~50MB |
| macOS | x86_64 (Intel) | ✅ Supported | `macosx_10_9_x86_64` | ~50MB |
| Linux | x86_64 | ✅ Supported | `manylinux_2_17_x86_64` | ~50MB |
| Windows | x86_64 | ✅ Supported | `win_amd64` | ~50MB |
| Linux | ARM64 | 🔄 Future | `manylinux_2_17_aarch64` | ~50MB |
| Linux | ARM32 | 🔄 Future | - | - |
| Windows | ARM64 | 🔄 Future | `win_arm64` | ~50MB |

**Unsupported platforms:**
Users can still use FEAGI by:
1. Installing Rust: `cargo install feagi`
2. Building from source
3. Downloading binary from GitHub releases

### Cost Breakdown Summary

| Component | Free Tier Limit | Our Usage | Cost |
|-----------|-----------------|-----------|------|
| **PyPI Storage** | Unlimited | ~200MB/version | **$0** |
| **PyPI Bandwidth** | Unlimited | ~200MB × downloads | **$0** |
| **GitHub Actions - Linux** | Unlimited | ~15 min/build | **$0** |
| **GitHub Actions - macOS** | 2,000 min/mo | ~40 min/build | **$0** |
| **GitHub Actions - Windows** | 2,000 min/mo | ~25 min/build | **$0** |
| **GitHub Releases** | 2GB/file | ~200MB/release | **$0** |
| **TOTAL** | - | - | **$0/month** ✅ |

**Build capacity (assuming 4 releases/month):**
- macOS: 4 × 40 min = 160 min ✅ (well under 2,000 limit)
- Windows: 4 × 25 min = 100 min ✅ (well under 2,000 limit)

---

## Dependency Management

### Core Dependencies
- `numpy>=1.20.0`
- `aiohttp>=3.9.0`
- `requests>=2.31.0`
- `toml>=0.10.2`
- `pyzmq>=24.0.0`
- `pydantic>=2.0.0` (for type validation)
- `typing-extensions>=4.0.0` (for Python 3.8 compatibility)

### Optional Dependencies
- **video:** `opencv-python>=4.9.0`
- **embedded:** `esptool>=4.5.0`, `pyserial>=3.5`
- **mcp:** `mcp-server>=0.1.0`, `fastapi>=0.100.0`
- **dev:** `pytest`, `mypy`, `ruff`, `black`, `sphinx`

---

## Breaking Changes from `feagi-connector`

### Package Name
- **Old:** `pip install feagi-connector`
- **New:** `pip install feagi`

### Import Paths
- **Old:** `from feagi_connector import FeagiClient`
- **New:** `from feagi.connector import FeagiAgentClient`

### Deprecation Strategy
1. Keep `feagi-connector` package on PyPI with deprecation warning
2. `feagi-connector` 2.x → transitional release that imports from `feagi`
3. `feagi-connector` 3.x → removed, point users to `feagi`

---

## Success Metrics

### Adoption
- [ ] 1000+ PyPI downloads in first month
- [ ] 10+ GitHub stars
- [ ] 5+ community contributions

### Quality
- [ ] >90% test coverage
- [ ] 100% type hint coverage on public APIs
- [ ] <5 critical bugs in first 3 months

### Documentation
- [ ] Complete API reference
- [ ] 10+ tutorials
- [ ] 20+ code examples

### Integration
- [ ] Successfully deployed on embedded device
- [ ] MCP server working with Claude
- [ ] Used in at least 3 community projects

---

## Risk Assessment

### Technical Risks

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Rust binary distribution complexity | Medium | High | Start with separate installation |
| Breaking changes affect existing users | High | Medium | Clear migration guide, deprecation warnings |
| REST API changes | Low | High | Version API endpoints, maintain compatibility |
| Performance issues with large genomes | Medium | Medium | Implement caching, pagination, lazy loading |
| MCP protocol compatibility | Low | Low | Follow MCP spec strictly |

### Project Risks

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Scope creep | Medium | High | Stick to phased plan, defer non-critical features |
| Timeline slippage | Medium | Medium | Build incrementally, release early phases |
| Insufficient testing | Low | High | Enforce >90% coverage, CI/CD gates |
| Documentation lag | Medium | Medium | Write docs alongside code, not after |

---

## Open Questions

1. **Rust Binary Distribution**: Bundle or separate installation?
   - **Recommendation**: Separate initially, evaluate bundling later

2. **Type System**: TypedDict or Pydantic?
   - **Recommendation**: Pydantic for runtime validation, better error messages

3. **Async vs Sync API**: Provide both?
   - **Recommendation**: Primary API is sync (easier for beginners), async available via `feagi.async_api`

4. **Backward Compatibility**: Support old `feagi-connector` imports?
   - **Recommendation**: Yes, via transitional release that re-exports from new package

5. **Embedded Binary Compilation**: Python SDK triggers Rust compilation?
   - **Recommendation**: No, use pre-built binaries or external compilation

6. **MCP Server Hosting**: Standalone service or embedded?
   - **Recommendation**: Both options available

---

## Next Steps

### Immediate Actions (Week 1)
1. Review and approve this plan
2. Set up new GitHub repository structure
3. Create project board with all tasks
4. Set up CI/CD pipeline skeleton
5. Begin Phase 1: Package renaming

### Key Decisions Needed
1. Approve package name change: `feagi-connector` → `feagi`
2. Approve version bump: `2.x` → `3.0.0`
3. Approve Rust binary distribution strategy
4. Approve type system choice (TypedDict vs Pydantic)
5. Approve async/sync API strategy

### Stakeholder Review
- [ ] Technical lead approval
- [ ] Architecture team review
- [ ] Community feedback (via RFC/discussion)

---

## Conclusion

This plan transforms `feagi-connector` into a comprehensive, production-ready Python SDK for FEAGI 2.0. The phased approach allows for incremental delivery, with each phase building on the previous one. The SDK will provide:

1. **Unified installation** via `pip install feagi`
2. **Engine management** from Python
3. **Complete configuration API**
4. **Full REST API wrapper**
5. **High-level genomics API**
6. **Connectome analysis tools**
7. **Embedded device support**
8. **MCP server for AI agents**
9. **Comprehensive documentation**
10. **Full IDE integration**

The resulting SDK will make FEAGI accessible to a broader audience, from beginners building simple agents to advanced users deploying on embedded devices, and AI agents leveraging FEAGI through MCP.

**Estimated Timeline:** 24 weeks (6 months)  
**Estimated Effort:** 1-2 full-time developers  
**Target Release:** Q2 2026 (feagi v3.0.0)

