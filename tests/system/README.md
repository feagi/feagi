# System-Level Tests

This directory contains system-level tests that span multiple modules of FEAGI. These tests verify the interaction and integration between different components across module boundaries.

## Test Structure

```
system/
  ├── integration/           # System integration tests
  │   ├── test_neuroembryogenesis.py      # Tests for brain development process
  │   └── test_synaptogenesis.py          # Tests for synapse formation across modules
  └── performance/           # System-level performance benchmarks
      └── (future performance tests)
```

## Running Tests

### Integration Tests

```bash
# Use the provided script
./run_integration_tests.sh

# With additional pytest arguments
./run_integration_tests.sh -v
```

## Purpose of System Tests

System-level tests differ from module-specific tests in that they:

1. **Cross module boundaries**: Test interactions between BDU, NPU, API and other modules
2. **Exercise complete workflows**: Test end-to-end processes like genome loading through brain development
3. **Validate system behavior**: Ensure the system behaves correctly as a whole

## Test Guidelines

1. **Focused scope**: While these tests span multiple modules, each should still have a focused purpose
2. **Minimal dependencies**: Minimize external dependencies to make tests more reliable
3. **Realistic scenarios**: Test scenarios should reflect actual usage patterns
4. **Clear expectations**: Define clear pass/fail criteria

## System Test Coverage

The system tests aim to cover:

- End-to-end brain development from genome to functioning brain
- Cross-module interactions like BDU to NPU handoff
- Integration of neurogenesis and synaptogenesis
- Complete workflows from API through core functionality
- System resiliency and error handling

## Adding New System Tests

When adding new system tests:

1. Place the test in the appropriate subdirectory based on its purpose (integration, performance)
2. Focus on testing cross-module functionality rather than internal details
3. Use descriptive test names that explain what system behavior is being tested
4. Consider adding the test to the run script if it should be run regularly 