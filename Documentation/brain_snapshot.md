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

## Profiles
- `model` (default): structure/topology + long-term parameters. Dynamic state is reinitialized on restore.
- `stateful`: everything in `model` plus dynamic runtime state (e.g., memory lifespans current, activation counters). Use when resuming an exact session.

## API endpoints

Create snapshot
- POST `/v1/snapshots`
- Body:
  ```json
  { "stateful": false, "compression": true }
  ```
- Always creates the folder snapshot (manifest + connectome.json + state.json). If `compression` is true, persists a container file: `.fgc` for model (stateful=false), `.fgs` for stateful (stateful=true).
- Response: `{ "snapshot_id": "...", "path": "...", "formats_available": {"fgc":"<id>.fgc"} }`

Download artifact
- GET `/v1/snapshots/{snapshot_id}/artifact/{fmt}` where `{fmt}` is `fgc` (model), `fgs` (stateful) or `zip`
- `fgc`: returns `<id>/<id>.fgc`; builds on-demand if missing
- `fgs`: returns `<id>/<id>.fgs` (must exist)
- `zip`: builds to temp and streams; not persisted by default

Stream artifact (no persistence)
- GET `/v1/snapshots/stream?stateful=false&compression=true`
- Builds a snapshot folder, then writes `.fgc`/`.fgs` directly to `[snapshot].temp_dir` and streams it with cleanup. No lingering files inside the snapshot directory.

Restore snapshot
- POST `/v1/snapshots/{snapshot_id}/restore`
- Body (optional): `{ "mode": "mmap" | "load", "profile": "model" | "stateful" }`
  - If profile is omitted, default is `model`.
  - If mode is omitted, default is read from `[snapshot].fc_restore_mode` in `feagi_configuration.toml`.
- If `profile":"stateful"`, requires `<id>/<id>.fgs`.
- If `profile":"model"`, uses `<id>/<id>.fgc` if present; otherwise uses folder manifest.

Upload and restore
- POST `/v1/snapshots/upload` (multipart form)
- Form fields: `file` (.fgc/.fgs), optional `mode` (`load`|`mmap`)
- The file is staged to `[snapshot].temp_dir`, then moved under the snapshot output directory and restored according to the container magic (`fgc`→model, `fgs`→stateful).

## Configuration
The following TOML config keys are required (no fallbacks):

```toml
[snapshot]
output_dir = "/absolute/path/to/snapshots"
temp_dir = "/absolute/path/to/tmp"
zip_compression = "deflate"  # or "store"
# Restore mode for .fc files: "load" copies into RAM; "mmap" uses zero-copy views
fc_restore_mode = "load"  # or "mmap"
```

These are read via `feagi.config.toml_loader.load_feagi_config()`.

## Integrity and determinism
- Only files from `manifest.json` are included.
- Deterministic ordering and compression.
- Folder snapshots include blake2b-256 checksums for `connectome.json` and `state.json`, verified on restore.
- `.fgc/.fgs` include per-chunk checksums (header) and per-chunk CRC32 (footer), plus a file-level SHA-256 in the footer. The `state.json` chunk checksum is verified on restore.

## Mmap-based fast-path
- `.fgc/.fgs` chunks are 4096-byte aligned and versioned for zero-copy access.
- For performance, prefer `store` encoding for large SoA arrays to enable zero-copy views.
- Helpers: `map_fc()`, `get_chunk_view()`, `get_array_from_chunk()`, `unmap_fc()`.
- In mmap mode, the artifact file must remain present until unmapped; deletion while mapped is not allowed.

## Streaming and compression
- Streaming read is supported via `stream_chunk()` for resource-constrained targets (e.g., RTOS). Supports `store`, `deflate`, and `lz4`.
- Compression options:
  - `store`: optimal for mmap and fastest load time
  - `deflate`: compact, widely supported
  - `lz4`: lightweight compression for constrained targets (requires lz4)

## Examples
- Create and persist fc:
  ```bash
  curl -s -X POST http://<host>:<port>/v1/snapshots \
    -H 'content-type: application/json' \
    -d '{"stateful":false, "compression":true}'
  ```
- Create and persist stateful:
  ```bash
  curl -s -X POST http://<host>:<port>/v1/snapshots \
    -H 'content-type: application/json' \
    -d '{"stateful":true, "compression":true}'
  ```
- Stream without persisting:
  ```bash
  curl -OJ "http://<host>:<port>/v1/snapshots/stream?stateful=false&compression=true"
  ```
- Download fc:
  ```bash
  curl -OJ http://<host>:<port>/v1/snapshots/<id>/artifact/fgc
  ```
- Restore (default mode from config):
  ```bash
  curl -s -X POST http://<host>:<port>/v1/snapshots/<id>/restore
  ```
- Restore with explicit mmap mode:
  ```bash
  curl -s -X POST http://<host>:<port>/v1/snapshots/<id>/restore \
    -H 'content-type: application/json' \
    -d '{"mode":"mmap"}'
  ```
- Download stateful:
  ```bash
  curl -OJ http://<host>:<port>/v1/snapshots/<id>/artifact/fgs
  ```
- Restore stateful:
  ```bash
  curl -s -X POST http://<host>:<port>/v1/snapshots/<id>/restore \
    -H 'content-type: application/json' \
    -d '{"profile":"stateful"}'
  ```
- Upload and restore:
  ```bash
  curl -s -X POST http://<host>:<port>/v1/snapshots/upload \
    -F file=@/path/to/model.fgc -F mode=load
  ```

## Testing
- Pytest validates ZIP packaging and content matches the manifest.
- Negative cases cover missing manifest or files.
- End-to-end tests validate creation, on-demand artifact build, and restore.
- Stateful profile tests validate `.fgs` download and restore, and error on profile mismatches.

## Future work
- Snapshot creation (quiesce, serialize, checksum) endpoint
- Restore endpoint
- Streaming zip without temp file
- Incremental snapshots and signing 