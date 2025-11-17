# NPU RTOS/Rust Compatibility Monitoring

## Overview

The NPU RTOS/Rust compatibility monitoring system (`test_rtos_rust_compatibility.py`) is a comprehensive static analysis tool that evaluates all NPU Python modules for patterns that would be incompatible with RTOS environments or difficult to migrate to Rust.

## Purpose

As FEAGI moves toward embedded systems and real-time applications, we need to ensure the NPU codebase is compatible with:
- **Real-Time Operating Systems (RTOS)** with strict timing requirements
- **Embedded systems** with limited resources and no dynamic allocation
- **Rust migration** for memory safety and performance
- **No-std environments** without standard library dependencies

## Usage

### Running the Test

```bash
# Run the full compatibility analysis (will fail if critical issues found)
pytest tests/npu/test_rtos_rust_compatibility.py::test_npu_rtos_rust_compatibility -v -s

# Run analysis for specific modules only
pytest tests/npu/test_rtos_rust_compatibility.py::test_specific_module_compatibility -v -s

# Generate report without failing the build
python tests/npu/test_rtos_rust_compatibility.py /path/to/npu/modules
```

### Understanding the Results

The test categorizes issues into 5 levels:

| Level | Score Impact | Description | Examples |
|-------|-------------|-------------|----------|
| 🔴 **CRITICAL** | -25 points | Completely incompatible with RTOS | `threading`, `signal`, `time.sleep()` |
| 🟠 **HIGH** | -15 points | Major compatibility issues | `numpy`, dynamic allocation, file I/O |
| 🟡 **MEDIUM** | -8 points | Some compatibility concerns | exceptions, `random`, reflection |
| 🟢 **LOW** | -3 points | Minor issues | certain stdlib functions |
| ✅ **COMPATIBLE** | -0 points | RTOS/Rust friendly | static typing, fixed allocations |

### Current NPU Status (Latest Analysis)

```
📊 Modules Analyzed: 7
🔴 burst_engine.py           Score:   0.0/100 (CRITICAL)
🟡 fcl_injection_service.py  Score:   4.0/100 (MEDIUM)
🟠 fcl_manager.py            Score:   0.0/100 (HIGH)
🟠 gpu_fcl_adapter.py        Score:   0.0/100 (HIGH)
🟠 optimized_integration.py  Score:   0.0/100 (HIGH)
🟠 optimized_structures.py   Score:   0.0/100 (HIGH)
🟡 special_area_handler.py   Score:  28.0/100 (MEDIUM)

Average Compatibility Score: 4.6/100
```

## Issue Categories Detected

### 🔴 Critical Issues (Must Fix Before RTOS)

1. **Threading Operations**
   - `import threading`
   - `threading.Thread()`, `threading.Lock()`
   - RTOS replacement: Use RTOS task primitives

2. **Signal Handling**
   - `import signal`
   - `signal.signal()`
   - RTOS replacement: Use RTOS signal mechanisms

3. **Blocking Sleep Operations**
   - `time.sleep()` in main loops
   - RTOS replacement: Deterministic timing wheels

### 🟠 High-Risk Issues (Major Refactoring Needed)

1. **NumPy Dependencies**
   - `import numpy`
   - Heavy library not available in embedded systems
   - Solution: Replace with fixed-size arrays or conditional imports

2. **Dynamic Memory Allocation**
   - `numpy.resize()`, `list.append()` in loops
   - `collections.deque()` with unbounded growth
   - Solution: Pre-allocate all data structures at initialization

3. **File I/O Operations**
   - `open()`, `file.read()`, `file.write()`
   - Solution: Move to initialization phase or use memory-mapped files

### 🟡 Medium-Risk Issues (Gradual Improvement)

1. **Exception Handling in Critical Paths**
   - `try/except` blocks in main loops
   - Solution: Use `Result<T, E>` patterns or fail-fast approaches

2. **Reflection and Dynamic Typing**
   - `isinstance()`, `hasattr()`, `getattr()`
   - Solution: Use static typing and compile-time dispatch

3. **Print Statements**
   - 200+ `print()` calls throughout codebase
   - Solution: Replace with structured logging or conditional compilation

4. **Random Operations**
   - `random.random()`, `random.randint()`
   - Solution: Use deterministic PRNGs with known seeds

## Migration Strategy

### Phase 1: Critical Issues (Required for RTOS)
1. **Replace threading with RTOS tasks**
   ```python
   # Current (incompatible)
   import threading
   thread = threading.Thread(target=worker)

   # RTOS-compatible
   # Use RTOS-specific task creation APIs
   ```

2. **Eliminate blocking sleep calls**
   ```python
   # Current (incompatible)
   time.sleep(interval)

   # RTOS-compatible
   # Use RTOS timer or event-based scheduling
   ```

3. **Remove signal handlers**
   ```python
   # Current (incompatible)
   signal.signal(signal.SIGINT, handler)

   # RTOS-compatible
   # Use RTOS interrupt handling
   ```

### Phase 2: High-Risk Issues (Memory Management)
1. **Pre-allocate data structures**
   ```python
   # Current (problematic)
   self.data = []
   for item in stream:
       self.data.append(item)  # Dynamic growth

   # RTOS-compatible
   self.data = [None] * MAX_SIZE  # Fixed allocation
   self.data_index = 0
   ```

2. **Replace NumPy with fixed arrays**
   ```python
   # Current (problematic)
   import numpy as np
   arr = np.resize(old_arr, new_size)

   # RTOS-compatible
   arr = [0.0] * FIXED_SIZE  # Compile-time known size
   ```

### Phase 3: Medium-Risk Issues (Code Quality)
1. **Replace exceptions with error codes**
   ```python
   # Current (non-deterministic)
   try:
       result = risky_operation()
   except Exception as e:
       handle_error(e)

   # RTOS-compatible
   result, error = safe_operation()
   if error:
       handle_error(error)
       return error
   ```

2. **Use static typing**
   ```python
   # Current (dynamic)
   if isinstance(obj, SomeType):
       return obj.method()

   # RTOS-compatible
   def process_some_type(obj: SomeType) -> Result:
       return obj.method()
   ```

## CI Integration

The test is designed to fail the build when critical issues are introduced:

```python
# In CI pipeline
if critical_modules:
    pytest.fail(
        f"CRITICAL RTOS compatibility issues found in modules: {', '.join(critical_modules)}. "
        f"These must be fixed before RTOS migration."
    )
```

This prevents new incompatible code from entering the codebase.

## Adding New Compatibility Checks

To add new compatibility patterns:

1. **Add to pattern lists**:
   ```python
   CRITICAL_FUNCTIONS = {
       'your.new.function',  # Add here
   }
   ```

2. **Create specialized checks**:
   ```python
   def _check_your_pattern(self, node, func_name):
       if 'your_pattern' in func_name:
           self._add_issue(...)
   ```

3. **Update documentation** with new patterns and solutions

## Best Practices for RTOS-Compatible Code

1. **Always use fixed-size allocations**
2. **Avoid dynamic imports and reflection**
3. **Use deterministic algorithms and timing**
4. **Prefer static typing over dynamic**
5. **Handle errors without exceptions in hot paths**
6. **Use monotonic time sources**
7. **Minimize global state**
8. **Design for fail-fast behavior**

## Related Documentation

- `/docs/architecture/rtos_migration_plan.md` - Overall RTOS migration strategy
- `/docs/coding_guidelines.md` - General coding standards
- `/docs/architecture/rust_integration.md` - Rust migration guidelines
- `/tests/npu/README.md` - NPU testing strategy

---

*This monitoring system ensures FEAGI NPU remains compatible with embedded systems and real-time requirements while facilitating the eventual migration to Rust.*
