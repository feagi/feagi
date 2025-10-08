## FEAGI NPU (Neural Processing Unit) – Architecture Guide

This document defines the FEAGI NPU architecture as of the 2.0 refactor. It replaces the legacy material in `npu_old_delete/` and is written for future Rust/RTOS migration. NPU is the single source of truth for runtime neural data and computation.

### Goals
- Deterministic, bounded-time burst processing (no implicit fallbacks)
- Clear separation of concerns with well-defined data flow
- Rust/WebGPU friendly Structure-of-Arrays (SoA) layout for neurons and synapses
- Stable interfaces to sampling/streams without cross-module leakage

## High-level Data Flow
1) External/internal candidate accumulation → Fire Candidate List (FCL)
2) Burst Engine processes FCL → computes actual firing → Fire Queue (current burst)
3) Fire Ledger archives per-burst results for temporal queries/history

Visually: `FCL (candidates) → [BurstEngine] → Fire Queue (fired) → Fire Ledger (history)`

## Core Components

### 1) Fire Candidate List (FCL) – Rust Implementation
- Role: Pre-burst collector of neurons that should be evaluated this burst (NOW IN RUST).
- Model: SoA at the candidate level; per-area buckets of `(neuron_id, delta_potential, is_excitatory)`.
- Implementation: Fully implemented in Rust `feagi-burst-engine` crate, accessed via Python FFI wrapper.
- Determinism:
  - No threshold/decay decisions here; only accumulation of intended deltas.
  - All processing is deterministic and bounded.
- Responsibilities:
  - Add candidates (SoA) per cortical index
  - Provide candidate counts and statistics for diagnostics
  - Reset each burst after use (ephemeral)

Typical lifecycle per burst:
1. Rust NPU clears FCL at the end of the previous burst.
2. Accumulate candidates (power areas, sensory, synaptic pre-computations for next burst, etc.).
3. Rust Burst Engine processes FCL for membrane integration and neural dynamics.

### 2) Fire Queue – `npu/fire_queue.py`
- Role: Holds neurons that actually fired in the current timestep (after dynamics).
- Model: Per-area SoA of fired neurons with attached metadata:
  - `neuron_id`, `cortical_idx`, `membrane_potential`, `(x,y,z)`, `threshold`, `consecutive_fire_count`, `refractory_counter`.
- Responsibilities:
  - Provide a canonical fired set for downstream consumers
  - Enable efficient area-based queries and zero-copy accessors for samplers
  - Export simple structures for stream publishing or analytics

### 3) Fire Ledger – `npu/fire_ledger.py` (Python interface; future Rust crate)
- Role: Historical storage and queries over prior fired sets.
- Model: Rolling window per cortical area using roaring bitmap semantics (Python wrapper; Rust planned).
- Responsibilities:
  - Archive last-N timesteps by cortical area and globally
  - Provide temporal queries (recently fired, consistent patterns, XOR/delta)
  - Maintain configurable per-area window sizes in future (incl. memory areas)

### 4) Burst Engine – `npu/burst_engine.py`
- Role: Orchestrates the burst cycle deterministically.
- Responsibilities (per burst):
  1. Inject pre-burst candidates into FCL (e.g., power areas; buffered external activations if present)
  2. Apply FCL candidate deltas to neuron SoA (membrane integration)
  3. Run SIMD neural dynamics (decay, leak, refractory, excitability, firing check)
  4. Emit Fire Queue (fired set for current burst)
  5. Archive to Fire Ledger (historical view) and rotate windows
  6. Prepare next-burst candidates as needed (e.g., from internal propagation results)

Determinism notes:
- No randomness unless explicitly configured (e.g., probabilistic excitability); default path is fully deterministic.
- All arrays are pre-allocated; no dynamic memory in the hot path.
- Strict, bounded work per burst; rate is governed by `desired_frequency_hz`.

## NPU SoA – Runtime Data Ownership

NPU owns the runtime SoA for neurons (and exposes SoA for synapses via the NPU interface). This ensures a single source of truth with cache-friendly, vectorizable memory layout.

### Neuron SoA – `npu/data_structures.py::NeuronArray`
For `max_neurons` capacity, the following parallel arrays exist (types in parentheses):
- membrane_potentials (float32)
- thresholds (float32)
- decay_rates (float32)
- leak_coefficients (float32)
- resting_potentials (float32)
- neuron_types (int32)
- positions_x/positions_y/positions_z (int32) – logical positions
- coordinates_x/coordinates_y/coordinates_z (uint32) – fast access layout for SIMD/GPU
- refractory_periods (uint8)
- refractory_counters (uint8)
- consecutive_fire_counts (uint16)
- consecutive_fire_limits (uint16)
- cortical_idxs (uint16) – area mapping
- excitabilities (float32)
- valid_mask (bool)
- neuron_id_to_index / index_to_neuron_id (Python dicts; will become FFI-safe maps in Rust)

Mandatory genome-provided fields per neuron at creation time:
- thresholds, decay_rates, leak_coefficients, resting_potentials, excitabilities,
- refractory_periods, consecutive_fire_limits, neuron_types, (x,y,z) positions.

Invariants:
- `0 <= neuron_count <= max_neurons`, `count` is an alias for compatibility.
- `valid_mask[i]` true only for initialized, in-range neurons.
- `neuron_id_to_index` and `index_to_neuron_id` remain consistent; vectorized helpers map indices↔ids.

### Synapse SoA – Interface and Migration
While the primary synapse SoA resides in the NPU interface (and will be a Rust-owned structure), the same principles apply:
- source_neurons, target_neurons (uint32)
- weights (float32)
- delays (int32/uint16 as configured)
- plasticity metadata (typed SoA)

The Burst Engine’s propagation step reads from synapse SoA to propose next-burst candidates; the write path is decoupled from the hot burst loop.

## Burst Processing – Deterministic Steps

1) Candidate Accumulation (FCL)
- Power areas (always-on injection) and any pre-burst external activations append candidates.
- Each candidate contributes a delta to a neuron’s membrane potential; threshold checks are not performed here.

2) Membrane Integration (SoA update)
- FCL candidates are applied to `membrane_potentials[idx]` for mapped neurons.
- Vectorized index lookups and bounds checks ensure O(N) with good cache locality.

3) Neural Dynamics (SIMD)
- Decay/leak to resting potentials
- Refractory counters update
- Threshold comparison with optional per-area excitability
- Fired indices are converted to neuron IDs and emitted to the Fire Queue

4) Archival and Rotation
- Fire Queue is archived to Fire Ledger for the current timestep.
- FCL is cleared; next-burst candidate queues (if any) are rotated.

## FQ Sampler – `npu/fq_sampler.py`
- Reads the current Fire Queue at its own rate (e.g., visualization 10–60 Hz, motor at burst rate) without blocking the Burst Engine.
- Exposes APIs for per-area reads and zero-copy SOA views where possible.
- Subscriber-aware activation can be layered on top by the process manager.

Sampling contract:
- Sample “fired” neurons from the latest available Fire Queue (not candidates).
- Data is always reported by cortical area to simplify downstream filtering and transport.

## Interfaces and External Inputs

External sensory inputs should be translated into per-area coordinate→neuron_id mappings by the existing conversion utilities and then appended as FCL candidates with explicit deltas. Deterministic behavior requires:
- No implicit scaling or heuristic fallbacks in NPU; upstream producers must define deltas precisely.
- If deltas are below the neuron’s threshold (after dynamics), the neuron deterministically will not fire in that burst.

## Rust / Crate Boundaries (Suggested)
- `feagi-npu-core`: Neuron/Synapse SoA, SIMD kernels, neural dynamics, Burst Engine core
- `feagi-fire-ledger`: Historical storage and temporal queries (Roaring bitmap or equivalent)
- `feagi-fcl`: Candidate structures and consolidation utilities
- `feagi-fq-sampler`: Consumer-facing sampling with zero-copy accessors
- `feagi-npu-ffi`: C-ABI interfaces for Python and embedded platforms

Design notes for migration:
- Keep SoA field order/types stable; use `repr(C)` packs in Rust.
- Replace Python dict maps with fixed-capacity hash maps or perfect hashing for index↔id.
- Move SIMD operations to Rust (or GPU backends) with identical semantics and tests.

## Determinism, Testing, and Safety
- No hidden randomness; excitability/probabilistic behavior must be explicit and testable.
- All hot-path arrays are pre-allocated; bounds checks are enforced.
- Unit tests should validate:
  - Candidate application → expected membrane updates
  - Thresholding → expected fired set (Fire Queue contents)
  - Archival → Fire Ledger window rotation
  - Sampler invariants (area-grouped outputs, rates)

## Operational Notes
- Burst frequency is configured centrally (state manager) and read once per run loop.
- The engine never silently drops candidates; overflow conditions must be surfaced as errors or bounded by configuration.
- Memory growth is bounded by `max_neurons`, `max_synapses`, and Fire Ledger window sizes.

## Glossary
- FCL (Fire Candidate List): Pre-burst candidate collection (deltas, no thresholding)
- Fire Queue: Fired neurons for the current timestep (post-dynamics)
- Fire Ledger: Historical storage of fired sets for temporal analysis
- SoA (Structure of Arrays): Parallel arrays per field for SIMD/GPU and cache locality
