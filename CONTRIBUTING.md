# Contributing to FEAGI 2.0

Thank you for contributing to FEAGI (Framework for Evolutionary Artificial General Intelligence)! This guide covers everything you need to know.

## 🚀 Quick Start

1. **Fork and clone** the repository
2. **Create virtual environment**: `python -m venv .venv && source .venv/bin/activate`
3. **Install dependencies**: `pip install -e ".[dev]"`
4. **Read architecture compliance rules** below (CRITICAL)
5. **Create feature branch**: `git checkout -b feature/your-feature-name`

## 🚨 Architecture Compliance (CRITICAL)

**FEAGI 2.0 MUST work across Docker, Kubernetes, cloud, embedded, and desktop environments.**

### ❌ FORBIDDEN PATTERNS
```python
# Never hardcode these in operational code:
host = "127.0.0.1" or "localhost"     # ❌ 
timeout = 30 or time.sleep(5)         # ❌
config.get('host', 'localhost')       # ❌
os.environ.get("HOST", "127.0.0.1")   # ❌
```

### ✅ REQUIRED PATTERNS
```python
# Always use TOML configuration:
from feagi.config.toml_loader import get_host_config, get_timeout_config
config = load_feagi_config()
host_config = get_host_config(config)      # Will fail if not configured
timeout_config = get_timeout_config(config)
```

### 🔧 Emergency Fallbacks: `# @architecture:acceptable`

**Only when explicitly annotated with valid reasons:**

```python
try:
    config = load_feagi_config()
    timeout = timeout_config.graceful_shutdown
except Exception:
    timeout = 8.0  # @architecture:acceptable - emergency fallback
```

**Valid reasons:**
- `emergency fallback` - Config unavailable (shutdown, signal handlers)
- `test isolation` - Unit tests need specific behavior
- `client default` - Client library defaults

**Invalid reasons:**
- `convenience`, `default`, `temporary` - **NOT ALLOWED**

## 🧪 Testing Requirements

**ALWAYS run before committing:**
```bash
# Test architecture compliance (CRITICAL)
pytest tests/system/test_architecture_compliance.py -v

# Run all tests
python -m pytest tests/

# Check code style
black feagi/ && flake8 feagi/
```

**Required for all new code:**
- Unit tests in `/tests/` mirroring source structure
- 80%+ coverage for critical components
- Architecture compliance for network/timeout code

## 🏗️ Code Standards

### Structure & Style
- **Files < 500 lines** - Keep modules focused
- **PEP 8 compliance** - Use black and flake8
- **Type hints everywhere** - Rust migration ready
- **Google-style docstrings** - Document all public functions
- **Clear, descriptive names** - Self-documenting code

### Project Layout
```
feagi/              # Main package
├── core/           # Core functionality
├── bdu/            # Brain Development Unit  
├── npu/            # Neural Processing Unit (GPU-compatible)
├── api/            # API interfaces
└── pns/            # Peripheral Nervous System
tests/              # Tests mirror feagi/ structure
docs/               # Documentation
```

### Component-Specific Rules
- **feagi_core/**: Must use configuration system, no hardcoded fallbacks
- **feagi_npu/**: GPU-compatible (no recursion, vectorized only)
- **feagi_bridge/**: Must support dynamic host/port configuration  
- **feagi_connector/**: Client defaults allowed with `@architecture:acceptable - client default`

## 🔄 Contribution Workflow

### 1. Select an Issue
- Browse [open issues](https://github.com/feagi/feagi/issues) for something to work on
- Filter by labels like "good first issue" or "help wanted"
- Comment on issue to express interest before starting work
- If you have a new idea, create an issue first to discuss it

### 2. Before Writing Code
- [ ] Check existing code for similar patterns
- [ ] Use `get_host_config()` and `get_timeout_config()`
- [ ] Never hardcode network values
- [ ] Think: "Will this work in Docker/K8s?"

### 3. Branch Naming
- `feature/your-feature-name` - New features
- `fix/bug-description` - Bug fixes  
- `docs/what-changed` - Documentation
- `refactor/component-name` - Code refactoring

### 4. Commit Guidelines
```bash
# Good commit messages (imperative mood):
git commit -m "Add neural burst detection to NPU"
git commit -m "Fix memory leak in FCL manager"
git commit -m "Update API documentation for v2.0"

# Reference issues:
git commit -m "Fix #123: Address timeout in ZMQ server"
```

### 5. Pull Request Process
- Push your branch: `git push origin feature/your-feature-name`
- Create pull request against main repository
- Provide clear description and reference related issues
- Address feedback and ensure CI/CD checks pass
- Your contribution will be merged once approved

### 6. Pull Request Checklist
- [ ] **Architecture compliance tests pass**
- [ ] **No hardcoded hosts/timeouts** (unless properly annotated)
- [ ] **Unit tests included** and passing
- [ ] **Documentation updated**
- [ ] **Clear PR description** with issue references
- [ ] **Small, focused changes** (prefer multiple small PRs)

### 7. Emergency Hardcoding Checklist
- [ ] Is this truly emergency/test/signal handler?
- [ ] Used correct format: `# @architecture:acceptable - [reason]`
- [ ] Reason is from valid list (not "convenience")
- [ ] Architecture tests still pass

## 🎯 Types of Contributions

### Code Contributions
- **Feature development** - New FEAGI capabilities
- **Bug fixes** - Resolve issues and improve stability  
- **Performance optimizations** - Enhance neural processing speed
- **Test coverage improvements** - Increase code reliability

### Documentation Contributions
- **Improving existing documentation** - Clarify and enhance guides
- **Creating new tutorials** - Help users learn FEAGI
- **Adding code examples** - Demonstrate usage patterns
- **Fixing typos** - Improve readability

### Other Contributions
- **Issue reporting** - Identify bugs and enhancement opportunities
- **UX/UI improvements** - Enhance user experience
- **Build system enhancements** - Improve development workflow
- **Community support** - Help other contributors

## 📚 Documentation Standards

**Required for all public functions:**
```python
def process_neural_data(neurons: List[int], threshold: float = 0.5) -> Dict[str, Any]:
    """
    Process neural data and return firing patterns.
    
    Args:
        neurons: List of neuron IDs to process
        threshold: Firing threshold (0.0-1.0)
        
    Returns:
        Dictionary with firing patterns and metadata
        
    Raises:
        ValueError: If threshold is outside valid range
    """
```

## 🎯 Performance Guidelines

### Rust/RTOS Migration Ready
- **Use type hints everywhere**
- **Prefer functions over classes**
- **Avoid global state**
- **Use NumPy arrays** (not lists)
- **Minimize dynamic behavior**

### GPU Compatibility (`feagi/npu/`)
- **No recursion** - GPU threads can't recurse
- **Vectorized operations only** - No loops over large data
- **Plan for WebGPU/WebAssembly** - Consider FFI safety

## 🗂️ Module-Specific Guidelines

### Core Module
- Focus on stability and error handling
- Minimize dependencies between components
- Thread safety for concurrent access

### NPU Module (Neural Processing)
- Performance-critical code
- GPU/WebGPU compatible patterns
- Comprehensive benchmarking

### BDU Module (Brain Development)
- Genome/connectome separation
- Memory-efficient structures
- Evolutionary algorithm compliance

### API Module
- REST API best practices
- Backward compatibility
- Comprehensive OpenAPI docs

### PNS Module (Sensorimotor)
- Protocol specification compliance
- Multi-modal sensor support
- Real-time performance

## 🚨 Architecture Decision Records (ADRs)

For significant architectural changes:
1. Copy template from `docs/templates/adr-template.md`
2. Name it `adr-your-feature-name.md`
3. Fill in all sections explaining decision, alternatives, and rationale
4. Submit with your PR or as separate PR for discussion

## 📢 Communication Channels

- **GitHub Issues**: Bug reports, feature requests, and general discussion
- **Discord**: Real-time communication and community support
- **Weekly Dev Meetings**: Join our open development meetings (schedule in README)
- **Pull Request Reviews**: Collaborate on code improvements

## 🆘 Getting Help

1. **Check existing code** for similar patterns
2. **Run architecture tests** to catch violations
3. **Search GitHub issues** for related discussions
4. **Create new issue** for questions or proposals
5. **Join development meetings** (schedule in README)

## 🎖️ Code of Conduct

- **Be respectful and inclusive**
- **Focus on the best solution, not personal preferences**
- **Provide constructive feedback**
- **Support fellow contributors**
- **Be patient with new contributors**

## 📄 License

By contributing to FEAGI, you agree that your contributions will be licensed under the project's Apache 2.0 License.

---

**Goal: Platform-agnostic FEAGI that deploys anywhere without configuration changes.**

**Remember: Every hardcoded value is a potential deployment failure!** 