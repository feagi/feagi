# State Management

FEAGI implements a centralized state management system that provides:

- Thread-safe access to system state
- Consistent state transition logging
- Visual indicators via emojis
- Clean separation of concerns

## Service States

All services track their state using the `ServiceState` enum:

- **UNAVAILABLE**: Service not started or has been shut down
- **INITIALIZING**: Service is starting but not ready
- **READY**: Service is operational and available
- **ERROR**: Service encountered an error
- **PAUSED**: Service is temporarily suspended

## Tracked Systems and Their Emojis

| System | Description | Emoji |
|--------|-------------|-------|
| Burst Engine | Neural firing dynamics | ⚡ |
| Connectome | Neuron and synapse data | 🧠 |
| API Service | REST API availability | 🌐 |
| ZMQ Service | Messaging system | 📡 |
| FCL Sampler | Fire Candidate List sampling | 🔥 |
| Genome | Current genome status | 🧬 |
| Brain Readiness | Overall system readiness | 🟢 |

## Logging Format 