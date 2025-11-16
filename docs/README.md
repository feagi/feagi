# FEAGI Connector Documentation

This directory contains documentation for the FEAGI Connector library.

## Documentation Structure

- **Strategic Planning**: Future vision and transformation roadmap
  - [SDK Transformation Plan](FEAGI_SDK_TRANSFORMATION_PLAN.md) - Comprehensive 25-week plan to transform feagi-connector into the official FEAGI Python SDK
  - [Executive Summary](FEAGI_SDK_EXECUTIVE_SUMMARY.md) - High-level overview of the SDK transformation
  - [Codebase Evaluation & Strategy](CODEBASE_EVALUATION_AND_STRATEGY.md) - **⭐ RECOMMENDATION: Evolve existing codebase (80% reuse)**

- **User Guides**: How to use the FEAGI Connector
  - [Connector Usage Guide](guide-connector-usage.md)
  - [ZMQ Communication Guide](guide-zmq-communication.md)
  - [Gaze Control Guide](guide-gaze-control.md)
  
- **Technical Documentation**: Design and implementation details
  - [Architecture Overview](arch-connector-overview.md)
  - [Protocol Specifications](spec-protocols.md)
  - [Configuration](CONFIGURATION.md)
  - [Dynamic Motor Registration](DYNAMIC_MOTOR_REGISTRATION.md)
  
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