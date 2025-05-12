"""
FCL Sampler for FEAGI.

The FCLSampler provides an efficient way to sample the Fire Candidate List at
configurable frequencies for visualization and motor output processing.

Key features:
- Configurable global sampling frequency
- Area-specific sampling rates
- Non-blocking output queue
- Support for multiple downstream consumers
"""

class FCLSampler:
    """
    FCLSampler: Samples the latest FCL at a configurable frequency and forwards
    it to consumers (e.g., visualization, motor output).
    
    - Supports per-area sample rates using the 'fcl_sample_rate' property
    - RTOS/Rust-friendly: runs as a periodic task/thread
    - No dynamic allocation in the main loop
    - Supports graceful shutdown
    """ 