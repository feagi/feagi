## Sleep Manager (Low-Activity Maintenance)

The Sleep Manager runs background maintenance (aging, long-term conversion re-check, and GC).
It is now triggered by genome physiology parameters rather than FCL history size alone:

- physiology.sleep_trigger_inactivity_window (bursts)
- physiology.sleep_trigger_neural_activity_max (neurons)

Mechanism:
- During each FCL update, FEAGI increments cumulative counters in State Manager:
  - cumulative_activity_bursts
  - cumulative_activity_neurons (sum of all firing neurons)
- Sleep Manager wakes periodically and checks:
  - if cumulative_activity_bursts >= window and cumulative_activity_neurons <= max
  - If true, it performs maintenance (vectorized aging via age_by_bursts, LTM re-check, GC) and resets the counters.

Notes:
- The counters and thresholds are safe to tune at runtime via /v1/physiology.
- Vectorized aging and LTM conversion remain additive-lifespan and bitmask-based for Rust migration readiness. 