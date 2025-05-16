# FEAGI System Architecture Overview

*Last Updated: May 15, 2025*

## Project Overview

The Framework for Evolutionary Artificial General Intelligence (FEAGI) is designed to provide a flexible and extensible platform for developing brain-inspired artificial general intelligence.

## Core Features

- **Multi-backend support** (CPU/GPU)
- **FastAPI-based REST API**
- **High-performance ZMQ messaging** for bidirectional communication
- **Modular architecture** separating compute-intensive operations (neural processing) from async operations (neural development, visualization)
- **Evolutionary optimization** capabilities
- **Configurable visualization sampling** for high-frequency simulations

FEAGI uses a message-based architecture for its BurstEngine implementation, providing improved reliability and performance. This architecture prevents server hangs during heavy load and provides better timeout handling for neuron injection operations.

## Project Structure

```
./
├── feagi/                  # Main package
│   ├── core/               # Core functionality and resource management
│   │   ├── resource_mgr.py # Resource management
│   │   ├── zmq/            # ZMQ handler
│   │   ├── sec/            # Authentication and encryption modules
│   │   └── api/            # API implementations
│   ├── bdu/                # Brain Developmental Unit
│   ├── npu/                # Neural Processing Unit
│   ├── evo/                # Evolutionary Unit 
│   ├── pns/                # Peripheral Nervous System and sensorimotor IO modules
│   └── viz/                # Visualization data transformation
├── tests/                  # Unit, integration, and functional tests
├── docs/                   # Documentation
├── examples/               # Example scripts
└── requirements.txt        # Dependencies
```

## Compute Resource Strategy

- A subset of operations under Neural Processing Unit associated with neuron firing will be designed to support both CPU and GPU backends. The rest of the application will be running on CPU.
- The entire application is designed to run in a highly parallel and performant fashion
- Code is written to be easily ported to Rust in the future
- Quality of Service priority is assigned to various tasks ensuring critical processes have sufficient resources

## Security Considerations

### Authentication
FEAGI API and ZMQ are equipped with authentication enabling secure communication on all communication methods.

### Encryption
Encryption can negatively impact the transmission of sensorimotor data by adding latency but might be essential for select use-cases. Both API and ZMQ support encryption as an option.

## Major System Modules

### Core (CORE)
- API/Webserver (API)  
- ZMQ message handler

### Peripheral Nervous System (PNS)
- Sensory processor
- Motor processor

### Brain Developmental Unit (BDU)
- Handles neural structure development

### Neural Processing Unit (NPU)
- Burst Engine
- Neural computation

### Memory & Learning Unit (MLU)
- Learning mechanisms

### Evolutionary Unit (EVO)
- Evolutionary optimization

### Sleep (SLP)
- Memory consolidation
- Neural development

## Key Components

### REST API
Organized in version folders to enable future maintainability. Routes are defined enabling endpoints to be organized by functional area.

### ZMQ Handler
Initiates a ZMQ server with the ability to support multiple topics enabling multithreaded communication to and from FEAGI.

### Resource Manager
FEAGI consists of independent processes with various requirements (CPU vs GPU, time-sensitivity, etc.). The resource manager is responsible for starting, terminating, and orchestrating all FEAGI processes and initialization of critical data structures.

## Related Documentation
- [GPU Architecture](arch-gpu.md)
- [IPC Architecture](arch-ipc.md)
- [State Management](arch-state-management.md) 