---
id: brain-snapshot
title: Brain Snapshot and Restore
slug: /brain-snapshot
---

This document describes the Brain Snapshot feature: capturing and restoring the full FEAGI brain state deterministically.

## Goals
- Deterministic, cross-platform snapshot of the brain state.
- Exclude transient components (FCL windows, agent registration).
- Rust/RTOS friendly formats.
- Config-driven; no hardcoded paths.

## What is captured
- Genome + connectome structure and indices.
- Neurons (SoA arrays): membrane potentials, thresholds, refractory counters, flags, coordinates, allocation pointers.
- Synapses: indices, weights, delays, plasticity traces.
- Memory system: MemoryNeuronArray SoA, pattern digest maps, flags.
- State Manager: brain stats, current burst/timestep, debug flags, cumulative counters (policy configurable on restore).

Excluded: FCL content, agent registry, sockets/queues.

## Snapshot layout
A snapshot is a directory containing:
- `manifest.json`: schema version, metadata, and explicit file list
- `connectome.json`: static structure and mappings
- `state.json`: StateManager and minimal process markers
- `neurons/*.npz`: per-area or global neuron SoA chunks
- `synapses/*.npz`: synapse arrays (COO/CSR)
- `memory/*.npz`: memory neurons SoA and pattern maps

Deterministic conventions:
- Stable key ordering; fixed dtypes and little-endian
- Files listed explicitly in `manifest.json`

## API endpoints

Create snapshot
- POST `/v1/snapshots`
- Body:
  ```json
  { "format": "zip" | "fc", "compression": "store" | "deflate", "persist_artifact": false }
  ```
- Always creates the folder snapshot (manifest + connectome.json + state.json). If `persist_artifact` is true, also creates `<id>/<id>.fc` or `<id>/<id>.zip` as requested.
- Response: `{ "snapshot_id": "...", "path": "...", "formats_available": {"fc":"<id>.fc", "zip":"<id>.zip"} }`

Download artifact
- GET `/v1/snapshots/{snapshot_id}/artifact/{fmt}` where `{fmt}` is `fc` or `zip`
- `fc`: returns `<id>/<id>.fc`; builds on-demand if missing
- `zip`: builds to temp and streams; not persisted by default

Restore snapshot
- POST `/v1/snapshots/{snapshot_id}/restore`
- Prefers `<id>/<id>.fc` if present; otherwise uses folder manifest

## Configuration
The following TOML config keys are required (no fallbacks):

```toml
[snapshot]
output_dir = "/absolute/path/to/snapshots"
temp_dir = "/absolute/path/to/tmp"
zip_compression = "deflate"  # or "store"
```

These are read via `feagi.config.toml_loader.load_feagi_config()`.

## Integrity and determinism
- Only files from `manifest.json` are included.
- Deterministic ordering and compression.
- Optional checksums/signatures can be added in a future iteration.

## Examples
- Create and persist fc:
  ```bash
  curl -s -X POST http://<host>:<port>/v1/snapshots \
    -H 'content-type: application/json' \
    -d '{"format":"fc","compression":"store","persist_artifact":true}'
  ```
- Download fc:
  ```bash
  curl -OJ http://<host>:<port>/v1/snapshots/<id>/artifact/fc
  ```
- Restore:
  ```bash
  curl -s -X POST http://<host>:<port>/v1/snapshots/<id>/restore
  ```

## Testing
- Pytest validates ZIP packaging and content matches the manifest.
- Negative cases cover missing manifest or files.
- End-to-end tests validate creation, on-demand artifact build, and restore.

## Future work
- Snapshot creation (quiesce, serialize, checksum) endpoint
- Restore endpoint
- Streaming zip without temp file
- Incremental snapshots and signing 