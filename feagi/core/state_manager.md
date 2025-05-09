# FEAGI State Manager

## Overview

The FEAGI State Manager provides a high-performance, cross-language mechanism for tracking and updating FEAGI's global states. It uses memory-mapped files to achieve near-zero overhead state access between Python and Rust components.

## Design Goals

1. **Ultra-fast state access**: Optimized for high-frequency checks (every clock cycle)
2. **Cross-language compatibility**: Equal access from both Python and Rust
3. **Thread safety**: Multiple components can read/write concurrently
4. **Clear semantics**: Enum-based states with well-defined transitions
5. **Low overhead**: Minimal memory and CPU usage

## Implementation Details

### Memory-Mapped Architecture

The state manager uses a memory-mapped file as a shared memory region: 