# FEAGI 2.0 Glossary

*Definitions of key terms and concepts in FEAGI*

## A

**Agent** - An external client that connects to FEAGI to provide sensory input or receive motor output (e.g., robots, simulators, monitoring tools).

**API Gateway** - Central interface that routes requests between different API protocols (REST, ZMQ) and the core FEAGI system.

**Architecture Compliance** - Strict requirements ensuring FEAGI works across Docker, Kubernetes, cloud, embedded, and desktop environments without hardcoded values.

## B

**BDU (Brain Development Unit)** - Module responsible for brain structure evolution, genome management, and connectome generation.

**Burst Engine** - Core neural processing component that handles neuron firing patterns and neural simulation execution.

**Byte Structures** - Binary communication protocols (FCP, FSMP, FVP) used for efficient data exchange between FEAGI and agents.

## C

**Cap'n Proto** - Serialization protocol used for efficient binary communication in FEAGI's networking layer.

**Connectome** - The complete map of neural connections in a brain, generated from genome specifications.

**Connectome Manager** - Singleton component that manages brain structure and neural connectivity patterns.

**Cortical Area** - A region of the brain with specific functionality (e.g., sensory processing, motor control, memory).

**Cursor Rules** - Real-time development guidelines that prevent architecture violations during coding.

## D

**Docker** - Containerization platform; FEAGI must work in Docker environments without localhost assumptions.

**Dynamic Backend Selection** - FEAGI's ability to automatically choose optimal compute backend (CPU, GPU, WebGPU) based on available hardware.

## E

**Embedded Mode** - Lightweight FEAGI deployment for resource-constrained environments (no web UI, minimal services).

**Emergency Fallback** - Hardcoded values allowed only in shutdown/signal handlers, marked with `@architecture:acceptable` annotation.

**Evolutionary Unit (EVO)** - Component handling brain evolution, mutation, and fitness evaluation.

## F

**FCL (Fire Candidate List)** - Data structure tracking neurons ready to fire, organized by timesteps for temporal processing.

**FCP (FEAGI Control Protocol)** - Binary protocol for agent registration, control commands, and system management.

**FEAGI** - Framework for Evolutionary Artificial General Intelligence; platform for creating and evolving artificial brains.

**Fire Queue (FQ)** - Queue of neurons scheduled to fire, processed by the burst engine during neural simulation.

**FSMP (FEAGI Sensorimotor Protocol)** - Binary protocol for exchanging sensory input and motor output data.

**FVP (FEAGI Visualization Protocol)** - Binary protocol for streaming neural activity data to visualization clients.

## G

**Genome** - Genetic specification that defines brain architecture, cortical areas, and initial connectivity patterns.

**GNA (Global Neuron Array)** - Structure-of-Arrays data layout for all neurons, optimized for SIMD and GPU processing.

**GPU Backend** - WebGPU-based compute backend for accelerated neural processing on graphics hardware.

**Godot Bridge** - Integration layer connecting FEAGI to Godot game engine for 3D brain visualization.

## H

**Host Configuration** - Network host settings loaded from TOML config; never hardcoded to ensure deployment flexibility.

**Hardcoding Violation** - Forbidden practice of embedding network hosts, ports, or timeouts directly in code.

## I

**IPC (Inter-Process Communication)** - Communication between FEAGI components using ZeroMQ patterns and shared memory.

## K

**Kubernetes** - Container orchestration platform; FEAGI must work with K8s service discovery without static IPs.

## M

**Membrane Potential** - Electrical potential of a neuron that determines firing probability.

**Motor Output** - Commands sent from FEAGI to agents for actuator control (movement, actions).

## N

**Neural Processing** - Core computation involving neuron firing, synapse updates, and learning algorithms.

**NPU (Neural Processing Unit)** - GPU-compatible module handling neural simulation and computation.

**Neuron ID** - Unique identifier for neurons within the global neuron array.

## O

**OpenAPI** - Specification format for FEAGI's REST API documentation.

## P

**Platform Agnostic** - Core FEAGI principle ensuring deployment works across all target environments without modification.

**PNS (Peripheral Nervous System)** - Module managing sensory input and motor output interfaces.

**Protocol Translator** - Component converting between different binary protocols (FCP, FSMP, FVP).

## R

**REST API** - HTTP-based interface for system management, configuration, and monitoring.

**RTOS (Real-Time Operating System)** - Target platform for future Rust migration; influences current Python design choices.

**Rust Migration** - Planned future transition from Python to Rust for performance and embedded compatibility.

## S

**Sensory Input** - Data received from agents representing sensor readings (vision, audio, touch, etc.).

**SIMD (Single Instruction, Multiple Data)** - Vectorized processing pattern used for efficient neural computation.

**SoA (Structure of Arrays)** - Memory layout optimizing data access patterns for parallel processing.

**Synapse** - Connection between neurons with associated weight and learning parameters.

## T

**Timeout Configuration** - Configurable timing values loaded from TOML; never hardcoded for deployment flexibility.

**TOML Configuration** - Human-readable configuration format used for all FEAGI settings.

## W

**WebGPU** - Cross-platform GPU API targeted for neural processing acceleration.

**WebAssembly (WASM)** - Compilation target for future Rust components, enabling browser deployment.

## Z

**ZeroMQ (ZMQ)** - High-performance messaging library used for FEAGI's communication layer.

**ZMQ Patterns** - Communication patterns (REQ/REP, PUB/SUB, PUSH/PULL) used for different data flows.

**ZMQ Server** - Component managing all ZeroMQ-based communication with agents and clients.

---

*For detailed information on any term, see the relevant documentation in `/docs/` or the main `CONTRIBUTING.md` guide.* 