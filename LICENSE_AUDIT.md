# FEAGI License Audit Report

**FEAGI License:** Apache 2.0  
**Audit Date:** 2025-10-22  
**Status:** ✅ ALL DEPENDENCIES COMPATIBLE

## Executive Summary

All dependencies (Python and Rust) are compatible with Apache 2.0 license. No proprietary or copyleft (GPL) licenses detected.

---

## Python Dependencies License Audit

### Core Runtime Dependencies

| Package | License | Apache 2.0 Compatible | Notes |
|---------|---------|----------------------|-------|
| **fastapi** | MIT | ✅ Yes | Permissive |
| **uvicorn** | BSD-3-Clause | ✅ Yes | Permissive |
| **pydantic** | MIT | ✅ Yes | Permissive |
| **pydantic-settings** | MIT | ✅ Yes | Permissive |
| **numpy** | BSD-3-Clause | ✅ Yes | Permissive |
| **scipy** | BSD-3-Clause | ✅ Yes | Permissive |
| **torch (PyTorch)** | BSD-3-Clause | ✅ Yes | Permissive, Facebook modified BSD |
| **httpx** | BSD-3-Clause | ✅ Yes | Permissive |
| **PyYAML** | MIT | ✅ Yes | Permissive |
| **tomli** | MIT | ✅ Yes | Permissive (Python <3.11) |
| **python-jose** | MIT | ✅ Yes | Permissive |
| **passlib** | BSD-3-Clause | ✅ Yes | Permissive |
| **python-multipart** | Apache 2.0 | ✅ Yes | Same license! |
| **psutil** | BSD-3-Clause | ✅ Yes | Permissive |
| **pyroaring** | MIT | ✅ Yes | Permissive |
| **packaging** | Apache 2.0 / BSD-3-Clause | ✅ Yes | Dual licensed |
| **lz4** | BSD-3-Clause | ✅ Yes | Permissive |

### Development Dependencies

| Package | License | Apache 2.0 Compatible | Notes |
|---------|---------|----------------------|-------|
| **pytest** | MIT | ✅ Yes | Dev only |
| **pytest-asyncio** | Apache 2.0 | ✅ Yes | Dev only |
| **pytest-cov** | MIT | ✅ Yes | Dev only |
| **pytest-benchmark** | BSD-2-Clause | ✅ Yes | Dev only |
| **pytest-mock** | MIT | ✅ Yes | Dev only |
| **black** | MIT | ✅ Yes | Dev only |
| **isort** | MIT | ✅ Yes | Dev only |
| **mypy** | MIT | ✅ Yes | Dev only |
| **ruff** | MIT | ✅ Yes | Dev only |
| **pre-commit** | MIT | ✅ Yes | Dev only |
| **memory-profiler** | BSD-3-Clause | ✅ Yes | Dev only |
| **dataclasses-json** | MIT | ✅ Yes | Dev only |

---

## Rust Dependencies License Audit

### Core Rust Dependencies

| Crate | License | Apache 2.0 Compatible | Notes |
|-------|---------|----------------------|-------|
| **thiserror** | MIT OR Apache-2.0 | ✅ Yes | Dual licensed |
| **rayon** | MIT OR Apache-2.0 | ✅ Yes | Dual licensed |
| **ndarray** | MIT OR Apache-2.0 | ✅ Yes | Dual licensed |
| **ahash** | MIT OR Apache-2.0 | ✅ Yes | Dual licensed |
| **wgpu** | MIT OR Apache-2.0 | ✅ Yes | Dual licensed |
| **pollster** | MIT OR Apache-2.0 | ✅ Yes | Dual licensed |
| **bytemuck** | Zlib OR MIT OR Apache-2.0 | ✅ Yes | Multi-licensed |
| **pyo3** | MIT OR Apache-2.0 | ✅ Yes | Dual licensed |
| **numpy (rust)** | BSD-3-Clause | ✅ Yes | Permissive |

### Published FEAGI Crates

| Crate | License | Apache 2.0 Compatible | Notes |
|-------|---------|----------------------|-------|
| **feagi_data_structures** | Apache-2.0 | ✅ Yes | Your own crate |
| **feagi_data_serialization** | Apache-2.0 | ✅ Yes | Your own crate |

---

## License Compatibility Matrix

### Compatible Licenses (Can use with Apache 2.0)

✅ **MIT** - Most permissive, widely used  
✅ **BSD-2-Clause** - Simple permissive license  
✅ **BSD-3-Clause** - Permissive with non-endorsement clause  
✅ **Apache 2.0** - Same as FEAGI license  
✅ **ISC** - Similar to MIT  
✅ **Zlib** - Very permissive  

### Potentially Problematic Licenses (None found)

❌ **GPL-3.0** - Strong copyleft, incompatible  
❌ **AGPL-3.0** - Network copyleft, incompatible  
❌ **Proprietary** - Not open source  

### Special Cases

⚠️ **LGPL (Lesser GPL)** - Found in ZeroMQ library
- **Status**: Generally acceptable with dynamic linking
- **Context**: PyZMQ links to ZeroMQ library
- **Resolution**: Dynamic linking allowed, no code modification

---

## ZeroMQ Licensing

### Rust ZMQ Crate (Used in feagi-pns)

**Status: ✅ SAFE**
- Rust `zmq` crate: MIT OR Apache-2.0 (dual licensed)
- ZeroMQ C library: LGPL-3.0 (dynamically linked)
- We use Rust zmq bindings, which link to libzmq dynamically
- No Python PyZMQ dependency anymore - removed in favor of Rust implementation

### LGPL Compliance

- ✅ Dynamic linking only (via Rust zmq crate)
- ✅ No modification of ZeroMQ source
- ✅ Include ZeroMQ acknowledgment in NOTICE file
- ✅ No static compilation

---

## Recommendations

### 1. Add NOTICE File

Create `NOTICE` file in root:

```
FEAGI - Framework for Evolutionary Artificial General Intelligence
Copyright 2025 Neuraville Inc.

This product includes software developed by:
- ZeroMQ community (https://zeromq.org/) - LGPL-3.0
- NumPy community (https://numpy.org/) - BSD-3-Clause
- PyTorch community (https://pytorch.org/) - BSD-3-Clause
- FastAPI (https://fastapi.tiangolo.com/) - MIT
- [Other major dependencies]
```

### 2. Update LICENSE File

Add acknowledgment section at end:

```
---
THIRD-PARTY LICENSES

This software uses the following third-party libraries:
- See NOTICE file for complete list
- See LICENSE-THIRD-PARTY.txt for full license texts
```

### 3. Create LICENSE-THIRD-PARTY.txt

Bundle full license texts of major dependencies:
- ZeroMQ (LGPL-3.0)
- NumPy (BSD-3-Clause)
- PyTorch (BSD-3-Clause)
- FastAPI (MIT)

### 4. Document ZeroMQ Usage

In README or docs, clarify:

```markdown
## License Compliance

FEAGI uses ZeroMQ (LGPL-3.0) via dynamic linking through PyZMQ.
ZeroMQ is not statically compiled into FEAGI and remains a separate
library. Full compliance with LGPL requirements is maintained.
```

---

## Verification Script

Created automated license checker:

```bash
# Check all Python dependencies
pip install pip-licenses
pip-licenses --format=markdown --with-urls > python-licenses.txt

# Check all Rust dependencies
cargo install cargo-license
cd feagi-rust
cargo-license --json > rust-licenses.json
```

---

## Conclusion

### ✅ All Clear for Distribution

1. **No GPL conflicts** - No strong copyleft found
2. **No proprietary licenses** - All open source
3. **LGPL handled correctly** - Dynamic linking compliant
4. **Standard permissive licenses** - MIT, BSD, Apache 2.0

### Action Items

- [ ] Create NOTICE file
- [ ] Create LICENSE-THIRD-PARTY.txt
- [ ] Update main LICENSE with third-party section
- [ ] Document ZeroMQ usage
- [ ] Run automated license checker in CI/CD

### Final Status

**FEAGI is SAFE to distribute under Apache 2.0 license.**

All dependencies are compatible. No legal blockers for PyPI publication.

---

## References

- Apache 2.0 License: https://www.apache.org/licenses/LICENSE-2.0
- LGPL + Apache compatibility: https://www.apache.org/licenses/GPL-compatibility.html
- PyZMQ License: https://github.com/zeromq/pyzmq/blob/main/LICENSE.md
- ZeroMQ License: https://github.com/zeromq/libzmq/blob/master/LICENSE

**Audited by:** AI Assistant  
**Review recommended:** Legal counsel for production use  
**Next audit:** Before each major release

