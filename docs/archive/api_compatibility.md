# FEAGI 2.1 API Backward Compatibility

This document explains how FEAGI 2.1 maintains backward compatibility with legacy FEAGI API clients.

## Design Philosophy

The FEAGI 2.1 API is designed with the following principles:

1. **Direct Endpoint Replication**: Rather than creating a compatibility layer or bridge between old and new APIs, FEAGI 2.1 directly replicates the exact endpoints from legacy FEAGI as V1 super route.

2. **Process Separation**: The API server runs as a separate process from the burst engine to ensure:
   - API requests don't disrupt neural simulations
   - Different CPU priority allocations can be applied
   - The API can be stopped/restarted without affecting the burst engine
   - The API can be completely decoupled and removed for select use-cases

3. **High-Performance IPC**: Inter-process communication between the API and burst engine uses efficient shared memory mechanisms rather than socket-based communication.

4. **Rust Migration Ready**: The architecture is designed to facilitate future migration to Rust, with clean boundaries and minimal dependencies.

## URL Structure

All endpoints follow the legacy FEAGI path structure:

- `/v1/cortical_areas`
- `/v1/cortical_area/{id}`
- `/v1/cortical_area_types`
- `/v1/genome/file_name`
- `/v1/burst_engine/config`

The implementation also accepts kebab-case variants (e.g., `/v1/cortical-areas`), but the primary paths use the original underscore format.

## Path Format Standardization

To comply with industry best practices, FEAGI 2.0 is adopting the kebob-case aka. dash-case instead of snake_case format
The legacy FEAGI API used snake_case for path segments (e.g., `/v1/cortical_area`, `/v1/burst_engine`). The FEAGI 2.1 API maintains this naming convention for all routes while accepting hyphenated alternatives:

- `/v1/cortical_area` and `/v1/cortical-area` are both valid
- `/v1/burst_engine` and `/v1/burst-engine` are both valid

This ensures that clients using either format will work properly.

## Inter-Process Communication

The API server and burst engine communicate through:

1. **Shared Memory**: Data structures like the connectome are shared between processes using memory-mapped files, eliminating serialization/deserialization overhead.

2. **Event-based Updates**: Instead of constant polling, the API service receives updates through an event system when data changes.

3. **Priority-based Process Management**: As detailed in `docs/feagi_processes.md`, process priorities ensure burst engine operations take precedence over API handling.

4. **Clean Component Boundaries**: Each component has a well-defined API surface to facilitate future language migration.

## Implementation Details

1. **Direct Data Access**: Where possible, the API accesses data structures directly through shared memory.

2. **Component Isolation**: Each functional area (cortical areas, genome, burst engine) has isolated implementation to facilitate independent development.

3. **Cache Management**: Frequently accessed data is cached in the API service to reduce IPC overhead.

4. **Efficient Serialization**: When data must be serialized for external clients, modern efficient serialization is used.

## Rust Migration Path

The architecture is designed to facilitate incremental migration to Rust:

1. **Clean Language Boundaries**: Components interact through well-defined interfaces that can be implemented in either Python or Rust.

2. **FFI-Friendly Types**: Data structures use types that can be easily marshalled between languages.

3. **Minimal External Dependencies**: The core implementation minimizes external dependencies that would complicate language migration.

4. **Shared Memory Focus**: The shared memory approach works well across language boundaries.

## Deployment Considerations

- The API server can be run on a separate CPU core from the burst engine
- Process priority can be set differently for API vs. burst engine
- The API server can be restarted without affecting ongoing simulations
- For development, the entire system can run in a single process

## Performance Considerations

- Eliminates multiple serialization/deserialization cycles from socket-based IPC
- Avoids constant polling that creates unnecessary CPU usage
- Reduces memory overhead by directly sharing data structures
- Simplifies the codebase by removing complex redirection logic

## Legacy Endpoint Support

The following legacy endpoints are directly supported:

### Cortical Areas

- GET `/v1/cortical_areas` - List all cortical areas
- GET `/v1/cortical_area/{area_id}` - Get a specific cortical area
- GET `/v1/cortical_area_types` - List all cortical area types

### Genome

- GET `/v1/genome/file_name` - Get the current genome filename
- POST `/v1/genome/upload/barebones` - Upload a barebones genome

### Burst Engine

- GET `/v1/burst_engine/config` - Get burst engine configuration
