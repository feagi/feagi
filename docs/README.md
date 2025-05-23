# FEAGI Documentation

This directory contains system-level architecture and design documentation for the FEAGI project.

## Documentation Structure

FEAGI follows a structured documentation approach:
- **System-level documentation** is stored in this `/docs` folder
- **Module-specific documentation** is stored in each module's directory

## Document Types

Documents follow a standardized naming convention with prefixes indicating their type:

| Prefix | Type | Purpose |
|--------|------|---------|
| `arch-` | Architecture | System design and architecture documents |
| `spec-` | Specification | Technical specifications and protocols |
| `guide-` | Guide | User and developer guides |
| `adr-` | Architecture Decision Record | Records of architectural decisions |
| `plan-` | Planning | Project planning documents |

## Key Documents

### Architecture
- [System Overview](arch-system-overview.md) - Core FEAGI architecture
- [Burst Engine Lifecycle](arch-burst-engine-lifecycle.md) - **NEW** - Burst engine state management and workflow
- [GPU Architecture](arch-gpu.md) - GPU acceleration design
- [IPC Architecture](arch-ipc.md) - Inter-process communication
- [State Management](arch-state-management.md) - System state management
- [ZMQ Architecture](arch-zmq.md) - ZeroMQ communication architecture

### Specifications
- [Shared Memory Protocol](spec-shared-memory.md) - Memory protocol specification
- [Communication Protocols](spec-protocols.md) - FCP, FVP, and FSMP protocol specifications
- [API Formats](spec-api-formats.md) - API response formats and compatibility

### Guides
- [Documentation Standards](guide-documentation-standards.md) - How to write documentation
- [Coding Standards](guide-coding-standards.md) - Code quality guidelines
- [Naming Conventions](guide-naming-conventions.md) - Naming rules

### Architecture Decision Records
- [API Refactoring](adr-api-refactoring.md) - API refactoring decisions

### Planning
- [Testing Strategy](plan-testing-strategy.md) - Test coverage planning
- [Documentation Restructuring](plan-documentation-restructuring.md) - Documentation reorganization plan
- [Documentation Progress](plan-documentation-restructuring-progress.md) - Current status of restructuring

## Assets

Diagrams and other visual assets are stored in the `/docs/assets` directory.

## Documentation Standards

Please follow the [Documentation Standards](guide-documentation-standards.md) when creating or updating documents. 