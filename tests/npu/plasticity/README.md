# FEAGI Memory System Test Suite

Comprehensive test suite for FEAGI's Memory System Version 3.0, validating all aspects of memory formation, pattern detection, and plasticity operations.

## Overview

This test suite validates the complete memory system including:

- **Fire Ledger Integration**: Historical data management with per-area window sizing
- **Pattern Detection**: RoaringBitmap-based temporal pattern detection with SHA-256 hashing
- **Memory Neuron Lifecycle**: Creation, aging, reactivation, and long-term memory conversion
- **PlasticityService**: Independent thread operations and command queue management
- **Global ID Management**: Range-partitioned neuron ID allocation
- **End-to-End Workflows**: Complete memory formation processes

## Test Files

### `test_memory_system.py`
**Main integration test suite**
- End-to-end memory formation workflow
- Per-area temporal depth configuration
- Memory neuron lifecycle management
- Pattern detection determinism
- Memory system statistics

### `test_pattern_detector.py`
**Pattern detection system tests**
- RoaringBitmap temporal pattern extraction
- SHA-256 deterministic hashing
- Per-area temporal depth configuration
- Pattern caching and LRU eviction
- Batch pattern detection
- Performance characteristics

### `test_plasticity_service.py`
**PlasticityService thread tests**
- Thread lifecycle management
- Burst notification system
- Command queue operations
- Drop-on-full policy
- Memory area registration
- Error handling and robustness

### `run_memory_tests.py`
**Test runner script**
- Comprehensive test execution
- Performance benchmarking
- Integration test workflows
- Detailed reporting

## Running Tests

### Quick Test Run
```bash
# Run all tests
python run_memory_tests.py

# Verbose output
python run_memory_tests.py --verbose

# Integration tests only
python run_memory_tests.py --integration-only

# Include performance tests
python run_memory_tests.py --performance
```

### Using pytest
```bash
# Run all memory system tests
pytest tests/npu/plasticity/

# Run specific test file
pytest tests/npu/plasticity/test_memory_system.py

# Run with coverage
pytest tests/npu/plasticity/ --cov=feagi.npu.plasticity

# Run performance tests only
pytest tests/npu/plasticity/ -m performance

# Run integration tests only
pytest tests/npu/plasticity/ -m integration
```

### Test Categories

Tests are organized by markers:

- `@pytest.mark.unit`: Unit tests for individual components
- `@pytest.mark.integration`: Integration tests for complete workflows
- `@pytest.mark.performance`: Performance and scalability tests
- `@pytest.mark.memory`: Memory system specific tests
- `@pytest.mark.pattern`: Pattern detection tests
- `@pytest.mark.plasticity`: Plasticity service tests

## Test Coverage

### Core Components Tested

#### Fire Ledger Integration
- ✅ Per-area window size configuration
- ✅ Memory area registration
- ✅ Temporal pattern sequence extraction
- ✅ Combined upstream activity aggregation
- ✅ Area-specific activity retrieval

#### Pattern Detection System
- ✅ RoaringBitmap temporal pattern extraction
- ✅ SHA-256 deterministic hashing
- ✅ Per-area temporal depth configuration
- ✅ Pattern caching with LRU eviction
- ✅ Batch processing of multiple areas
- ✅ Empty pattern handling
- ✅ Performance characteristics

#### Memory Neuron Array
- ✅ Memory neuron creation with pattern hashes
- ✅ Vectorized aging operations
- ✅ Reactivation with lifespan growth
- ✅ Long-term memory conversion
- ✅ Pattern-based neuron lookup
- ✅ Statistics collection
- ✅ SoA data structure integrity

#### PlasticityService
- ✅ Thread lifecycle management
- ✅ Burst notification handling
- ✅ Memory area registration
- ✅ Command queue operations
- ✅ Drop-on-full policy
- ✅ Error handling and robustness
- ✅ Statistics collection

#### Neuron ID Management
- ✅ Range-partitioned ID allocation
- ✅ Global uniqueness guarantees
- ✅ Thread-safe operations
- ✅ Memory vs regular neuron identification
- ✅ Allocation statistics

### Integration Workflows Tested

#### End-to-End Memory Formation
1. Memory area configuration in Fire Ledger
2. Neural activity simulation and archiving
3. Temporal pattern detection
4. Memory neuron creation
5. Pattern reactivation and lifespan growth
6. Long-term memory conversion
7. Command queue operations

#### Per-Area Temporal Depth
1. Different temporal depths for different memory areas
2. Pattern detector configuration
3. Fire Ledger window size management
4. Pattern detection with area-specific depths

#### Memory Lifecycle Management
1. Initial memory neuron creation
2. Aging process with lifespan reduction
3. Reactivation with lifespan growth
4. Long-term memory conversion threshold
5. Statistics tracking throughout lifecycle

## Performance Benchmarks

### Expected Performance Characteristics

| Operation | Target Performance | Test Coverage |
|-----------|-------------------|---------------|
| Pattern Detection | < 10ms per pattern | ✅ Tested |
| Memory Neuron Aging | < 10ms for 1M neurons | ✅ Tested |
| Pattern Lookup (cache hit) | < 1ms | ✅ Tested |
| Batch Pattern Detection | < 100ms for 100 areas | ✅ Tested |
| ID Allocation | < 1μs per ID | ✅ Tested |

### Scalability Tests

- **Memory Areas**: Up to 100 memory areas with different temporal depths
- **Temporal Depth**: Up to 50 timesteps per area
- **Activity Volume**: Up to 1000 neurons per area per timestep
- **Pattern Cache**: Up to 10,000 cached patterns
- **Memory Neurons**: Up to 1,000,000 memory neurons

## Test Data and Fixtures

### Fire Ledger Test Data
```python
# Example temporal patterns
test_patterns = [
    {1: [10, 11, 12], 2: [20, 21], 3: [30]},      # Timestep 0
    {1: [11, 12, 13], 2: [21, 22], 3: [31]},      # Timestep 1
    {1: [12, 13, 14], 2: [22, 23], 3: [32]},      # Timestep 2
    {1: [10, 11, 12], 2: [20, 21], 3: [30]},      # Timestep 3 (repeat)
]
```

### Memory Area Configurations
```python
memory_areas = {
    10: {'temporal_depth': 3, 'upstream_areas': [1, 2]},    # Short-term
    20: {'temporal_depth': 7, 'upstream_areas': [3, 4]},    # Medium-term
    30: {'temporal_depth': 15, 'upstream_areas': [5, 6]},   # Long-term
}
```

### Lifecycle Configurations
```python
lifecycle_config = MemoryNeuronLifecycleConfig(
    initial_lifespan=20,        # 20 burst cycles
    lifespan_growth_rate=3.0,   # +3 bursts per reactivation
    longterm_threshold=100      # 100 bursts for LTM conversion
)
```

## Debugging and Troubleshooting

### Common Test Failures

#### Pattern Detection Issues
```python
# Check Fire Ledger data
assert fire_ledger.current_timestep >= temporal_depth
assert len(upstream_areas) > 0
assert fire_ledger.get_area_activity(area_idx, timestep) is not None
```

#### Memory Neuron Lifecycle Issues
```python
# Check memory neuron array state
stats = memory_neuron_array.get_stats()
assert stats.active_neurons > 0
assert stats.total_capacity > stats.active_neurons
```

#### PlasticityService Issues
```python
# Check service state
assert service._running
assert service._latest_timestep >= 0
assert len(service._memory_areas) > 0
```

### Test Environment Setup

#### Required Dependencies
```bash
pip install pytest numpy hashlib threading queue unittest.mock
```

#### Optional Dependencies
```bash
pip install pytest-cov pytest-benchmark pytest-xdist
```

### Continuous Integration

#### GitHub Actions Example
```yaml
name: Memory System Tests
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Set up Python
        uses: actions/setup-python@v2
        with:
          python-version: 3.9
      - name: Install dependencies
        run: pip install -r requirements-dev.txt
      - name: Run memory system tests
        run: python tests/npu/plasticity/run_memory_tests.py --verbose
```

## Contributing

### Adding New Tests

1. **Follow naming convention**: `test_*.py` for files, `test_*` for methods
2. **Use appropriate markers**: `@pytest.mark.unit`, `@pytest.mark.integration`, etc.
3. **Include docstrings**: Describe what the test validates
4. **Use fixtures**: Create reusable test data and configurations
5. **Assert meaningful conditions**: Test both success and failure cases
6. **Include performance assertions**: Validate timing requirements

### Test Quality Guidelines

1. **Deterministic**: Tests should produce consistent results
2. **Independent**: Tests should not depend on other tests
3. **Fast**: Unit tests should complete in < 1 second
4. **Comprehensive**: Cover both happy path and edge cases
5. **Readable**: Clear test names and documentation

## Architecture Compliance

These tests validate compliance with FEAGI's architecture principles:

- ✅ **Deterministic Behavior**: All operations produce consistent results
- ✅ **Performance Isolation**: Memory processing doesn't block neural computation
- ✅ **Cross-Platform Compatibility**: Tests run on Linux, macOS, and Windows
- ✅ **Rust/RTOS Readiness**: Simple data types and bounded operations
- ✅ **Configuration-Driven**: All parameters configurable via TOML
- ✅ **Error Handling**: Robust error handling without system crashes

## Support

For questions about the memory system tests:

1. Check test documentation and comments
2. Run tests with `--verbose` flag for detailed output
3. Review the memory system architecture documentation
4. Contact the FEAGI development team

---

*Last Updated: September 2025 | Version: 3.0*
