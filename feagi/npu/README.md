# FEAGI NPU - Clean Architecture

This is a complete rewrite with clean separation of concerns:

## Components
- Fire Candidate List (FCL): Pre-burst collection  
- Fire Queue: Current firing neurons
- Fire Ledger: Historical data (future Rust)
- Burst Engine: Clean orchestration

## Data Flow
FCL → Fire Queue → Fire Ledger

## Benefits
- Single responsibility per component
- Rust-friendly SoA format
- No timing conflicts
- Ready for high-performance Rust migration
