# FEAGI Shared Memory Protocol

## Overview
The FEAGI shared memory protocol enables high-performance, low-latency inter-process communication (IPC) between the FEAGI core and API components. It is designed for compatibility with both Python and Rust (and other languages), and is suitable for RTOS and embedded environments.

---

## 1. Shared Memory File Structure
- **Memory-mapped files** are used for IPC, created in a configurable temporary directory (default: `/tmp`).
- Each logical data region (e.g., config, events) is stored in a separate file, named with a prefix (e.g., `feagi_shared_config.dat`).
- Files are locked using OS-level file locks to ensure atomic updates and prevent race conditions.

### Example Files:
- `feagi_shared_feagi_config.dat` — Main configuration dictionary
- `feagi_shared_events.dat` — Event queue/notification region

---

## 2. Event Notification System
- **EventNotificationSystem** uses a simple event queue in shared memory, with event types defined as strings (e.g., `CONFIG_UPDATED`, `GENOME_LOADED`).
- Each event is a JSON-serialized object with:
  - `event_type`: string (e.g., `CONFIG_UPDATED`)
  - `source`: string (process name)
  - `data`: dict (event payload)
  - `timestamp`: float (UNIX time)
- Event handlers are registered per event type and are invoked when new events are detected.

---

## 3. Shared Configuration Dictionary Schema
- The main config dict is stored as a JSON-encoded object in shared memory.
- Example keys:
  - `burst_engine_config`: dict (burst engine parameters)
  - `cortical_areas`: list of dicts (cortical area metadata)
  - `genome`: dict (current genome)
  - `genome_filename`: string
- All values must be JSON-serializable for cross-language compatibility.

---

## 4. Cross-Language (Python/Rust) Compatibility
- **Serialization:** All data is serialized as UTF-8 encoded JSON.
- **Atomicity:** File locks or atomic file replace operations are used for safe concurrent access.
- **Event Loop:** Both Python and Rust implementations should poll or watch for new events and process them accordingly.
- **Extensibility:** New event types or config keys can be added as needed, as long as they are documented and JSON-compatible.

---

## 5. Example Event (JSON)
```json
{
  "event_type": "CONFIG_UPDATED",
  "source": "api_server",
  "data": {"type": "burst_engine"},
  "timestamp": 1721234567.123
}
```

---

## 6. Example Config Dict (JSON)
```json
{
  "burst_engine_config": {
    "burst_duration": 10,
    "inter_burst_interval": 5,
    "maximum_firing_rate": 100,
    "threshold": 0.5
  },
  "cortical_areas": [
    {"id": "1", "name": "Visual Cortex", "type": "sensory", ...}
  ],
  "genome": { ... },
  "genome_filename": "essential_genome.json"
}
```

---

## 7. Implementation Notes
- Use the same file naming and locking conventions in all languages.
- Always validate and handle JSON decode errors gracefully.
- For RTOS/embedded, ensure the event loop is non-blocking and uses minimal resources.

---

## 8. References
- See `feagi/api/shared_memory/feagi_gateway.py` for the Python reference implementation.
- See `tests/api/core/test_shared_memory.py` for usage and test cases.
