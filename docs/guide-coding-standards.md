# Coding Standards and Guidelines

*Last Updated: May 15, 2025*

This document outlines the coding standards and guidelines for the FEAGI project to ensure consistent, maintainable, and high-quality code.

## 📁 Project Structure

- All source code lives under `/feagi/`
- All tests live under `/tests/` and follow the same hierarchy as `/feagi/`
- All system-wide documentation and architecture go under `/docs/`
- Each module folder (e.g., `/feagi/npu/`) must include a `README.md` explaining:
  - Module purpose
  - High-level design
  - Dependencies or constraints

## ⚙️ Runtime & Architecture

- Design with **Rust/RTOS migration** in mind:
  - Avoid dynamic typing and runtime reflection
  - Prefer stateless, pure functions
  - Minimize side effects and global state
- Code under `/feagi/npu/` must be **GPU-compatible**, targeting WebGPU/WebAssembly compatibility:
  - Avoid recursion
  - Prefer NumPy/vectorized operations over loops
  - Plan for FFI safety

## 🧠 Modularity & Style

- Keep modules < 500 lines where possible
- Use **clear, descriptive names** for files, classes, and functions
- Maintain consistent folder layout within modules
- Follow Python PEP 8 style guidelines

## 📚 Documentation Standards

- All public functions must have docstrings
- Use Google-style docstrings format
- Include parameter descriptions and return types
- Document exceptions that may be raised
- Provide examples for complex functionality

## 🔍 Testing Requirements

- All new code must have associated unit tests
- Tests should be placed in the `/tests` directory, mirroring the source structure
- Use pytest for all tests
- Strive for >80% test coverage for critical components

## Related Documentation

- [Documentation Standards](guide-documentation-standards.md)
- [Naming Conventions](guide-naming-conventions.md) 