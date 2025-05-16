# Coding Guidelines

## 📁 Project Structure

- All source code lives under `/feagi/`
- All tests live under `/tests/` and follow the same hierarchy as `/feagi/`
- All system-wide documentation and architecture go under `/docs/`
- Each module folder (e.g., `/feagi/npu/`) must include a `README.md` explaining:
  - Module purpose
  - High-level design
  - Dependencies or constraints

---

## ⚙️ Runtime & Architecture

- Design with **Rust/RTOS migration** in mind:
  - Avoid dynamic typing and runtime reflection.
  - Prefer stateless, pure functions.
  - Minimize side effects and global state.
- Code under `/feagi/npu/` must be **GPU-compatible**, targeting WebGPU/WebAssembly compatibility.
  - Avoid recursion.
  - Prefer NumPy/vectorized operations over loops.
  - Plan for FFI safety.

---

## 🧠 Modularity & Style

- Keep modules < 500 lines where possible.
- Use **clear, descriptive names** for files, classes, and functions.
- Maintain consistent folder layout:

