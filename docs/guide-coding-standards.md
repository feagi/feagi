# Coding Standards - Quick Reference

*See [DEVELOPERS.md](DEVELOPERS.md) for complete guidelines*

## 🚨 Critical Architecture Rules

**NO hardcoded hosts/timeouts in operational code:**
```python
❌ host = "127.0.0.1"
❌ timeout = 30
✅ host_config = get_host_config(config)
✅ timeout = timeout_config.processing_timeout
```

**Emergency fallbacks only:**
```python
timeout = 8.0  # @architecture:acceptable - emergency fallback
```

## 🧪 Testing Before Commit
```bash
cd feagi_core && pytest tests/system/test_architecture_compliance.py -v
```

## 📚 Documentation
- Google-style docstrings for all public functions
- Type hints everywhere
- Include examples for complex functionality

## 🏗️ Structure
- Files < 500 lines
- Clear, descriptive names
- Follow PEP 8
- Rust/RTOS migration ready

## Related Documentation

- **[DEVELOPERS.md](DEVELOPERS.md)** - Complete developer guide
- [Documentation Standards](guide-documentation-standards.md)
- [Naming Conventions](guide-naming-conventions.md) 