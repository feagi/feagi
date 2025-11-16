# FEAGI Python SDK - Executive Summary

**Date:** November 8, 2025  
**Version:** 1.0  

## Vision in Brief

Transform `feagi-connector` into **the official Python SDK for FEAGI 2.0** with a single, intuitive import structure:

```python
from feagi import engine, config, connector, genomics, connectome, embedded

# Start FEAGI engine
engine.start()

# Configure
cfg = config.load()
cfg.api.port = 9000

# Build genome
genome = genomics.create_genome("my_brain")
genome.add_cortical_area("v1", dimensions=(128, 128, 10))

# Deploy to ESP32
embedded.export("my_brain.json", "esp32")
```

## Key Goals

1. ✅ **Unified Installation**: `pip install feagi` (not `feagi-connector`)
2. ✅ **Engine Control**: Start/stop Rust FEAGI from Python
3. ✅ **Config API**: Programmatic TOML manipulation
4. ✅ **Genomics API**: Create/modify genomes natively
5. ✅ **Connectome API**: Query brain structure and metrics
6. ✅ **Embedded Support**: Export to ESP32, Arduino, etc.
7. ✅ **Full Type Hints**: Complete IDE integration
8. ✅ **MCP Server**: AI agent integration (Claude, etc.)
9. ✅ **REST Wrapper**: All 60+ endpoints as Python methods
10. ✅ **Extensible**: Easy to add new features

## Current State

### What Works Today
- Agent communication (ZMQ, WebSocket)
- Sensory/motor data exchange
- Vision processing
- Config loading (read-only)
- Basic REST helpers

### What's Missing
- Engine management
- Config modification
- Genomics API
- Connectome analysis
- Embedded export
- Comprehensive REST wrapper
- MCP server
- Package is named `feagi-connector` not `feagi`

## Proposed Package Structure

```
feagi/
├── engine/          # Engine management (NEW)
├── config/          # Config API (ENHANCED)
├── connector/       # Agent communication (REFACTORED)
├── genomics/        # Genome manipulation (NEW)
├── connectome/      # Connectome analysis (NEW)
├── embedded/        # Embedded export (NEW)
├── api/             # REST wrapper (ENHANCED)
├── mcp/             # MCP server (NEW)
└── types/           # Type definitions (NEW)
```

## Implementation Timeline

| Phase | Duration | Deliverable |
|-------|----------|-------------|
| 1. Foundation & Renaming | 2 weeks | Package renamed to `feagi`, structure established |
| 2. Engine + Binary Distribution | 3 weeks | **CI/CD for platform wheels**, `engine.start()` working |
| 3. Configuration API | 2 weeks | Programmatic config modification |
| 4. REST API Wrapper | 3 weeks | All 60+ endpoints wrapped |
| 5. Genomics API | 3 weeks | Full genome manipulation |
| 6. Connectome API | 2 weeks | Query and analysis tools |
| 7. Embedded API | 3 weeks | Export to ESP32, Arduino, STM32, RPi Pico |
| 8. MCP Server | 2 weeks | AI agent integration |
| 9. Documentation | 2 weeks | Tutorials, API reference, examples |
| 10. Testing & Release | 3 weeks | CI/CD, PyPI release |
| **TOTAL** | **25 weeks** | **Production-ready SDK** |

**Note:** Phase 2 extended by 1 week to set up CI/CD for multi-platform binary distribution.

## Breaking Changes

### Package Name
- **Old**: `pip install feagi-connector`
- **New**: `pip install feagi`

### Import Paths
- **Old**: `from feagi_connector import FeagiClient`
- **New**: `from feagi.connector import FeagiAgentClient`

### Mitigation
- Keep `feagi-connector` 2.x on PyPI with deprecation warning
- Transitional release re-exports from new package
- Clear migration guide

## Success Metrics

### Adoption
- 1000+ PyPI downloads/month within 3 months
- 10+ GitHub stars
- 5+ community contributions

### Quality
- >90% test coverage
- 100% type hint coverage on public APIs
- <5 critical bugs in first 3 months

### Integration
- Successfully deployed on embedded device
- MCP server working with Claude
- Used in 3+ community projects

## Resource Requirements

### Development
- 1-2 full-time developers
- 6 months development time
- Code review from architecture team

### Infrastructure
- CI/CD pipeline (GitHub Actions)
- PyPI account
- Documentation hosting (Read the Docs / GitHub Pages)

### Hardware (Optional)
- ESP32, Arduino, STM32, RPi Pico for testing
- GPU-enabled machine for burst engine testing

## Risks & Mitigation

| Risk | Mitigation |
|------|------------|
| Breaking changes affect users | Migration guide, deprecation warnings, transitional release |
| Rust binary distribution complexity | Start with separate install, add convenience later |
| Timeline slippage | Incremental releases, defer non-critical features |
| REST API changes | Version endpoints, maintain compatibility layer |

## Key Decisions Needed

1. **Package Name**: Approve `feagi-connector` → `feagi`
2. **Version Bump**: Approve `2.x` → `3.0.0`
3. **Binary Distribution**: Separate install vs bundled vs auto-download
4. **Type System**: TypedDict vs Pydantic (recommend Pydantic)
5. **Async API**: Primary sync with async option (recommend yes)

## Dependencies

### Current
- numpy, pyzmq, aiohttp, requests, toml

### New
- pydantic (type validation)
- esptool (embedded flashing)
- mcp-server (AI integration)

### Rust Binary
- Separate installation: `cargo install feagi`
- Or download from GitHub releases
- Python SDK detects in PATH

## Example Use Cases

### 1. Simple Agent with Engine Control
```python
from feagi import engine, connector

# Start engine
engine.start(genome="vision_genome.json")

# Create agent
agent = connector.create_agent("my_agent")
agent.send_sensory_data("camera", image_bytes)
```

### 2. Custom Genome Creation
```python
from feagi import genomics

genome = genomics.create_genome("robot_brain")
v1 = genome.add_cortical_area("v1", dimensions=(128, 128, 10))
m1 = genome.add_cortical_area("m1", dimensions=(32, 32, 5))
genome.connect(v1, m1, weight=0.8)
genome.save("robot_brain.json")
```

### 3. Embedded Deployment
```python
from feagi import embedded

# Configure ESP32
config = embedded.platforms.esp32.ESP32Config(
    memory_budget=512_000,  # 512KB
    quantization="int8"
)

# Export and flash
embedded.export("robot_brain.json", "esp32", config)
embedded.flash("esp32", port="/dev/ttyUSB0")
```

### 4. Connectome Analysis
```python
from feagi import connectome

metrics = connectome.get_metrics()
print(f"Neurons: {metrics.neuron_count}")
print(f"Connectivity: {metrics.connectivity_density:.2%}")

# Find critical paths
path = connectome.find_shortest_path("v1", "m1")
print(f"Path: {' → '.join(path)}")
```

### 5. MCP Integration (AI Agents)
```bash
# Start MCP server
feagi-mcp start --port 3000

# Claude can now:
# - "Create a cortical area named 'visual_cortex' with 100x100x10 dimensions"
# - "Connect visual_cortex to motor_cortex with 0.5 weight"
# - "Show me the neuron count in visual_cortex"
```

## Benefits

### For Developers
- ✅ Single `pip install feagi` for everything
- ✅ Intuitive Python API
- ✅ Full IDE support (autocomplete, type hints)
- ✅ Comprehensive documentation
- ✅ Easy genome creation
- ✅ Native embedded deployment

### For Researchers
- ✅ Programmatic genome manipulation
- ✅ Connectome analysis tools
- ✅ Metrics extraction
- ✅ Reproducible experiments

### For Production
- ✅ Engine lifecycle control
- ✅ Configuration management
- ✅ REST API wrapper (no raw HTTP)
- ✅ Type-safe operations
- ✅ Embedded deployment

### For AI Agents
- ✅ MCP server integration
- ✅ LLMs can interact with FEAGI
- ✅ Autonomous brain development
- ✅ Experiment automation

## Competitive Advantages

1. **Unified SDK**: One package for everything
2. **Engine Control**: Start/stop FEAGI from Python
3. **Genomics API**: No manual JSON editing
4. **Embedded**: Easy deployment to edge devices
5. **MCP Support**: First neuromorphic platform with AI agent integration
6. **Type-Safe**: Full IDE support, catch errors early
7. **Extensible**: Easy to add new features

## Next Steps

### Week 1
1. Review and approve plan
2. Create GitHub project board
3. Set up CI/CD skeleton
4. Begin package renaming

### Week 2-4
- Implement engine management
- Enhance configuration API
- Update documentation

### Month 2-3
- Complete REST API wrapper
- Build genomics API
- Add connectome analysis

### Month 4-5
- Implement embedded support
- Create MCP server
- Write comprehensive docs

### Month 6
- Testing and polish
- PyPI release
- Community announcement

## Binary Distribution Decision

### ✅ APPROVED: Platform-Specific Wheels with Bundled Binaries

**Rationale:**
- User experience is the top priority
- Zero monetary cost (PyPI and GitHub Actions are free for open source)
- Guaranteed version compatibility
- No external dependencies for end users

**Implementation:**
```bash
# Users just run:
pip install feagi

# Automatically gets the right wheel for their platform:
# - macOS ARM64:  feagi-3.0.0-py3-none-macosx_11_0_arm64.whl (50MB)
# - macOS Intel:  feagi-3.0.0-py3-none-macosx_10_9_x86_64.whl (50MB)
# - Linux x64:    feagi-3.0.0-py3-none-manylinux_2_17_x86_64.whl (50MB)
# - Windows x64:  feagi-3.0.0-py3-none-win_amd64.whl (50MB)

# Then it just works:
python
>>> from feagi import engine
>>> engine.start()
✅ Engine ready at http://localhost:8000
```

**Cost Breakdown:**

| Component | Free Tier | Our Usage | Monthly Cost |
|-----------|-----------|-----------|--------------|
| PyPI Storage | Unlimited | ~200MB/version | **$0** |
| PyPI Bandwidth | Unlimited | ~200MB × downloads | **$0** |
| GitHub Actions (Linux) | Unlimited | 15 min/build | **$0** |
| GitHub Actions (macOS) | 2,000 min/mo | 160 min/mo | **$0** |
| GitHub Actions (Windows) | 2,000 min/mo | 100 min/mo | **$0** |
| **TOTAL** | - | - | **$0/month** ✅ |

**Investment:**
- Initial setup: 60-80 developer hours (CI/CD pipeline, build configuration)
- Ongoing: 2-4 hours per release (monitoring, troubleshooting)

**Platforms Supported:**
- ✅ macOS ARM64 (Apple Silicon M1/M2/M3)
- ✅ macOS x86_64 (Intel)
- ✅ Linux x86_64
- ✅ Windows x86_64

### Single Package, Multiple Wheels Strategy

**One package name, automatic best choice:**

**Installation:**
```bash
# Default (RECOMMENDED for most users) - pip chooses best wheel automatically
pip install feagi

# What you get:
# - Supported platforms: Platform wheel with bundled binary (~50MB)
# - Other platforms: Pure Python wheel without binary (~5MB)

# Advanced: Force lightweight install
pip install feagi --no-binary feagi

# With extras (works with both wheel types):
pip install "feagi[video,embedded]"
pip install "feagi[video]" --no-binary feagi
```

**Wheel Types Published:**

| Wheel Type | Platforms | Size | Binary | Auto-Selected? |
|------------|-----------|------|--------|----------------|
| **Platform wheels** | macOS (ARM/Intel), Linux, Windows x64 | ~50MB | ✅ Yes | ✅ Default |
| **Pure Python wheel** | All platforms (universal fallback) | ~5MB | ❌ No | Only if no platform match |

**When to use `--no-binary feagi` (advanced):**
- Connecting to remote FEAGI server only
- Already have FEAGI installed (`cargo install feagi`)
- Building microservices that connect to FEAGI
- CI/CD pipelines (faster installs)
- Minimal Docker containers (smaller images)
- Embedded device development (no local engine needed)

**The smart engine manager automatically:**
1. Uses bundled binary if available (platform wheels)
2. Falls back to system binary if installed
3. Connects to remote via `FEAGI_REMOTE_URL` env var
4. Provides helpful error messages if none available

**See full implementation details in:** `FEAGI_SDK_TRANSFORMATION_PLAN.md` → "Single Package, Multiple Wheels Strategy"

---

## Questions?

For detailed technical plan, see: `FEAGI_SDK_TRANSFORMATION_PLAN.md`

**Contact**: FEAGI Development Team  
**Repository**: https://github.com/Neuraville/FEAGI-2.0  
**Discussion**: [Link to GitHub Discussion/RFC]

