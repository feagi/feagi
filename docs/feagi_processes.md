# FEAGI Process Architecture

This document outlines the system workflow and architecture for FEAGI (Framework for Evolutionary Artificial General Intelligence), with a focus on process management, performance, and quality of service.

## Overview

FEAGI employs a process-based architecture designed for high performance and efficient resource utilization. The architecture is built on three principles:

1. **Process Separation**: Critical computation is separated from non-critical processes to ensure consistent neural simulation performance.
2. **Priority-based Resource Allocation**: System resources are allocated based on process priority.
3. **Independent Process Management**: Each major system component operates as an independent process with clear responsibilities.

## Process Types and Priorities

FEAGI processes are categorized into three priority levels:

### Priority 1 (Critical - Real-time)
These processes handle the core neural simulation and must maintain real-time performance:

1. **Burst Engine**: Manages neuron firing dynamics, threshold detection, and refractory periods.
2. **Connectome Manager**: Handles access to neuron and synapse data structures.
3. **FCL Manager**: Maintains the Fire Candidate List and provides efficient queries.
4. **Memory & Learning Manager**: Applies plasticity rules to synaptic weights.

### Priority 2 (Important - Near Real-time)
These processes handle important but less time-critical operations:

1. **FCL Sampler**: Periodically extracts data from the FCL for visualization and motor output.
2. **PNS Message Broker**: Manages communication with peripherals and external systems.
3. **Resource Manager**: Monitors and allocates system resources based on process demands.

### Priority 3 (Background - Best Effort)
These processes handle optional or background operations:

1. **Web Server**: Provides API endpoints for monitoring and control.
2. **Stem Cell Manager**: Handles neurogenesis and synaptogenesis.
3. **Sleep Manager**: Manages memory consolidation during inactive periods.

## Process Manager

The Process Manager is responsible for:

1. **Process Creation**: Launches processes with appropriate resources and parameters.
2. **Process Monitoring**: Monitors process health and performance.
3. **Resource Allocation**: Distributes computing resources based on priority.
4. **Fault Tolerance**: Restarts failed processes and maintains system integrity.

## Resource Management

FEAGI employs a dynamic resource management strategy:

1. **CPU Allocation**:
   - Priority 1 processes receive dedicated CPU cores
   - Priority 2 processes share remaining cores
   - Priority 3 processes use opportunistic scheduling

2. **Memory Allocation**:
   - Memory is allocated based on neural network size
   - Shared memory used for inter-process communication
   - Memory-mapped files for large datasets

3. **GPU Utilization**:
   - Primary allocation to Burst Engine for neural dynamics
   - Secondary allocation to visualization and processing
   - Dynamic time-sharing for non-critical computations

4. **Checkpointing**:
   - Regular state saves for recovery after crashes
   - Ability to resume from last stable checkpoint
   - Incremental checkpoints to minimize performance impact

## System Workflow

The typical FEAGI system workflow follows these steps:

1. **Initialization**:
   - Resource Manager starts and allocates resources
   - Critical Priority 1 processes are launched
   - Initial connectome is loaded or created
   - Priority 2 and 3 processes are started

2. **Runtime Operation**:
   - Burst Engine continuously processes neural activity
   - FCL Manager maintains and updates firing history
   - Memory & Learning Manager applies plasticity rules
   - FCL Sampler periodically extracts data for visualization and motor control
   - PNS Message Broker handles communication with peripherals

3. **Development and Learning**:
   - Stem Cell Manager periodically runs to create new neurons and connections
   - Memory & Learning Manager continuously updates synaptic weights
   - Sleep Manager activates during specified periods for memory consolidation

4. **Monitoring and Management**:
   - Web Server provides API for monitoring and control
   - Resource Manager monitors system health and adjusts resource allocation

## Deployment Considerations

FEAGI supports multiple deployment scenarios:

1. **Single Machine**:
   - All processes run on one physical or virtual machine
   - Resource allocation optimized for available hardware
   - Suitable for development and smaller simulations

2. **Distributed Deployment**:
   - Critical processes run on high-performance compute nodes
   - Visualization and API services run on separate nodes
   - ZMQ messaging for inter-node communication
   - Suitable for large-scale simulations

3. **Container-Based Deployment**:
   - Each process group runs in a separate container
   - Kubernetes or Docker Compose for orchestration
   - Resource limits enforced at container level
   - Enables easy scaling and management

## Implementation Roadmap

1. **Phase 1: Core Framework**:
   - Resource Manager implementation
   - Process prioritization framework
   - Basic inter-process communication

2. **Phase 2: Critical Processes**:
   - Burst Engine with CPU and GPU paths
   - Connectome Manager with efficient data structures
   - FCL Manager with hierarchical support
   - Memory & Learning Manager for basic plasticity

3. **Phase 3: Secondary Processes**:
   - FCL Sampler for visualization and motor control
   - PNS Message Broker for peripheral communication
   - Web Server with basic API endpoints

4. **Phase 4: Background Processes**:
   - Stem Cell Manager for neurogenesis
   - Sleep Manager for memory consolidation
   - Advanced monitoring and management

5. **Phase 5: Performance Optimization**:
   - Profiling and optimization of critical paths
   - Advanced GPU acceleration
   - Distributed deployment support

## Implementation Checklist

For the detailed implementation checklist and current status of each component, please refer to the [Implementation Checklist](implementation_checklist.md) document.

## Conclusion

The FEAGI process architecture is designed to maximize performance, ensure proper resource allocation, and enable scalable brain simulation. By separating processes based on priority and leveraging GPU acceleration where appropriate, FEAGI can efficiently simulate complex neural networks while providing rich visualization and control capabilities.

This architecture supports the evolutionary approach to artificial general intelligence, allowing the system to grow, adapt, and learn from experience while maintaining high performance and stability. 