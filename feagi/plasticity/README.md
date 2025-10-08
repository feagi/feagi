Plasticity subsystem (Fire Ledger–backed)

This package implements deterministic, RTOS-friendly, Rust-ready plasticity logic that reads spike history from the Fire Ledger and emits bounded updates for the NPU to apply at burst boundaries.

Design goals
- Deterministic and bounded-time; no sleep or unbounded loops
- Clean separation: read-only history + compute here; apply inside NPU only
- Rust-ready APIs: POD arrays, stateless functions, minimal global state
- GPU-friendly: no device sync from plasticity; NPU batches a single sync after apply

Components
- stdp.py: STDPConfig (params), STDPComputer (activity/timing factors from ledger)
- memory.py: MemoryConfig, MemoryFormationManager (temporal pattern keys)
- orchestrator.py: PlasticityOrchestrator (unifies STDP/memory, deterministic grouping)
- service.py: PlasticityService (compute every burst, enqueue commands; drop-on-full)

Threading model
- ProcessManager spawns PlasticityService
- BurstEngine calls service.notify_burst(current_timestep) after ledger archival
- Service computes read-only and enqueues commands to NPU’s bounded queue
- NPU applies commands once per burst at the end-of-burst safe point

Queue & apply policy
- Queue capacity: bounded; drop-on-full, increment StateManager.increment_plasticity_dropped_ops(count)
- Apply budget: plasticity.max_ops_per_burst (commands per burst) with stable ordering

Configuration (feagi_configuration.toml)
[plasticity]
queue_capacity = 4096
max_ops_per_burst = 1024

[plasticity.stdp]
lookback_steps = 20
tau_pre = 20.0
tau_post = 20.0
a_plus = 0.01
a_minus = 0.012

[plasticity.memory]
lookback_steps = 50
pattern_duration = 10
min_activation_count = 3

Testing
- Unit tests cover STDP/memory compute determinism and queue/apply behavior
- Integration: verify per-burst compute → enqueue → end-of-burst apply

Migration to Rust
- This package becomes a compute crate; NPU remains the owner/apply crate; Fire Ledger as a shared history crate


