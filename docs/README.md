# FEAGI Connector Documentation

This directory contains documentation for the FEAGI Connector library.

## Documentation Structure

- **User Guides**: How to use the FEAGI Connector
  - [Connector Usage Guide](guide-connector-usage.md)
  - [ZMQ Communication Guide](guide-zmq-communication.md)
  
- **Technical Documentation**: Design and implementation details
  - [Architecture Overview](arch-connector-overview.md)
  - [Protocol Specifications](spec-protocols.md)
  
- **API Reference**: Generated from source code docstrings
  - Generated using mkdocstrings

## Building the Documentation

The documentation is built using MkDocs with the Material theme and mkdocstrings for API reference generation.

```bash
# Install documentation dependencies
pip install -e .[docs]

# Build the documentation
mkdocs build

# Serve the documentation locally
mkdocs serve
```

## Documentation Standards

All documentation follows these standards:

1. **File naming conventions**:
   - `guide-*`: User and developer guides
   - `arch-*`: Architecture documents
   - `spec-*`: Technical specifications
   - `adr-*`: Architecture Decision Records

2. **Markdown formatting**:
   - Use ATX-style headers (`#` for h1, `##` for h2, etc.)
   - Code blocks with language-specific syntax highlighting
   - Tables for structured data

3. **Code examples**:
   - All code examples should be runnable
   - Include import statements
   - Use type hints
   - Include comments explaining key concepts 