# FEAGI Documentation

## Architecture Updates

FEAGI's architecture has been significantly improved:

- **Dependency Injection**: Replaced global variables with explicit dependency injection
- **State Management**: Added comprehensive state tracking with emoji-based logging
- **Initialization Flow**: Redesigned to eliminate circular dependencies
- **Process Priority System**: Three-tier priority system for resource allocation

The burst engine now initializes in standby mode and doesn't require a genome to be loaded first, resolving previous circular dependencies between genome loading and system initialization.

See [Architecture](./architecture.md) for detailed information. 