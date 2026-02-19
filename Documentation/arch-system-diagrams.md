# FEAGI System Architecture Diagrams

This document provides visual representations of FEAGI's architecture using Mermaid diagrams, illustrating the system's components and their interactions.

## Overall System Architecture

```mermaid
graph TD
    subgraph "FEAGI System"
        API[API Layer]
        BDU[Brain Development Unit]
        NPU[Neural Processing Unit]
        PNS[Peripheral Nervous System]
        CORE[Core System]

        API --- CORE
        BDU --- CORE
        NPU --- CORE
        PNS --- CORE

        BDU --- NPU
        PNS --- NPU

        subgraph "Data Structures"
            GENOME[Genome]
            CONNECTOME[Connectome]
            FCL[Fire Candidate Lists]
        end

        BDU --- GENOME
        BDU --- CONNECTOME
        NPU --- CONNECTOME
        NPU --- FCL
    end

    subgraph "External Systems"
        AGENTS[Agents/Clients]
        HARDWARE[Hardware Resources]
        UI[User Interfaces]
    end

    AGENTS --- API
    AGENTS --- PNS
    UI --- API
    HARDWARE --- CORE
```

## Process Architecture

```mermaid
flowchart TD
    subgraph "FEAGI Processes"
        API_PROC[API Server Process]
        NPU_PROC[Neural Processing Process]
        BDU_PROC[Brain Development Process]
        PNS_PROC[Sensorimotor Process]
    end

    subgraph "IPC Mechanisms"
        ZMQ[ZeroMQ Messaging]
        SHM[Shared Memory]
        REST[REST API]
    end

    API_PROC --- REST
    API_PROC --- ZMQ
    NPU_PROC --- ZMQ
    NPU_PROC --- SHM
    BDU_PROC --- ZMQ
    BDU_PROC --- SHM
    PNS_PROC --- ZMQ

    subgraph "External Processes"
        AGENT_PROC[Agent Processes]
        VIZ_PROC[Visualization Process]
        MONITOR_PROC[Monitoring Process]
    end

    AGENT_PROC --- REST
    AGENT_PROC --- ZMQ
    VIZ_PROC --- REST
    MONITOR_PROC --- REST
```

## Data Flow Architecture

```mermaid
flowchart LR
    subgraph "Input Flow"
        SI[Sensory Input] --> PNS_IN[PNS Encoders]
        PNS_IN --> INP_NPU[NPU Input Processing]
        CONFIG[Configuration] --> API_IN[API Input Handler]
        API_IN --> CONF_PROC[Configuration Processing]
    end

    subgraph "Processing Flow"
        INP_NPU --> NEURAL[Neural Computation]
        NEURAL --> FCL[Fire Candidate Lists]
        FCL --> PLASTIC[Plasticity & Learning]

        CONF_PROC --> BDU_PROC[BDU Processing]
        BDU_PROC --> GENOME[Genome Updates]
        GENOME --> CONNECT[Connectome Generation]
        CONNECT --> NEURAL
    end

    subgraph "Output Flow"
        NEURAL --> OUT_NPU[NPU Output Processing]
        OUT_NPU --> PNS_OUT[PNS Decoders]
        PNS_OUT --> MO[Motor Output]

        NEURAL --> STATE[State Updates]
        STATE --> API_OUT[API Output Handler]
        API_OUT --> MONITOR[Monitoring Data]
    end
```

## Component Interaction - Neural Processing

```mermaid
sequenceDiagram
    participant PNS as Peripheral Nervous System
    participant FCL as Fire Candidate Lists
    participant NPU as Neural Processing Unit
    participant CONN as Connectome
    participant BDU as Brain Development Unit

    PNS->>FCL: Submit sensory neuron activations
    FCL->>NPU: Provide active neurons
    NPU->>CONN: Query synaptic connections
    CONN->>NPU: Return connectivity data
    NPU->>NPU: Compute neural updates
    NPU->>FCL: Update fire candidates
    NPU->>PNS: Send motor neuron activations

    par Asynchronous Updates
        BDU->>CONN: Update connectome structure
        CONN->>NPU: Signal structural changes
    end
```

## Deployment Architecture

```mermaid
flowchart TD
    subgraph "Development Environment"
        DEV_API[API Server]
        DEV_CORE[Development Core]
        DEV_UI[Developer UI]

        DEV_API --- DEV_CORE
        DEV_UI --- DEV_API
    end

    subgraph "Production Environment"
        subgraph "Main Server"
            PROD_API[API Server]
            PROD_BDU[BDU Service]
            PROD_CORE[Core Services]

            PROD_API --- PROD_CORE
            PROD_BDU --- PROD_CORE
        end

        subgraph "Processing Cluster"
            PROD_NPU1[NPU Node 1]
            PROD_NPU2[NPU Node 2]
            PROD_NPU3[NPU Node 3]

            PROD_CORE --- PROD_NPU1
            PROD_CORE --- PROD_NPU2
            PROD_CORE --- PROD_NPU3
        end

        subgraph "I/O Layer"
            PROD_PNS[Sensorimotor Service]
            PROD_UI[Visualization UI]

            PROD_PNS --- PROD_CORE
            PROD_UI --- PROD_API
        end
    end

    subgraph "External Agents"
        AGENT1[Agent 1]
        AGENT2[Agent 2]

        AGENT1 --- PROD_API
        AGENT1 --- PROD_PNS
        AGENT2 --- PROD_API
        AGENT2 --- PROD_PNS
    end
```

## Protocol Stack Architecture

```mermaid
flowchart TB
    subgraph "Protocol Stack"
        REST[REST API]
        FSMP[FEAGI Sensorimotor Protocol]
        FCP[FEAGI Control Protocol]
        FVP[FEAGI Visualization Protocol]
        ZMQ[ZeroMQ Transport]
        SHM[Shared Memory Protocol]

        REST --- FCP
        FSMP --- ZMQ
        FCP --- ZMQ
        FVP --- ZMQ
        ZMQ --- SHM
    end

    subgraph "Client Applications"
        AGENT[Agents]
        UI[User Interface]
        ADMIN[Admin Tools]

        AGENT --- FSMP
        AGENT --- FCP
        UI --- FVP
        UI --- REST
        ADMIN --- REST
        ADMIN --- FCP
    end

    subgraph "FEAGI Components"
        API_COMP[API Component]
        BDU_COMP[BDU Component]
        NPU_COMP[NPU Component]
        PNS_COMP[PNS Component]
        CORE_COMP[Core Component]

        REST --- API_COMP
        FCP --- API_COMP
        FSMP --- PNS_COMP
        FVP --- API_COMP

        API_COMP --- ZMQ
        BDU_COMP --- ZMQ
        NPU_COMP --- ZMQ
        PNS_COMP --- ZMQ
        CORE_COMP --- ZMQ

        NPU_COMP --- SHM
        BDU_COMP --- SHM
        CORE_COMP --- SHM
    end
```

## Resource Management Architecture

```mermaid
flowchart TD
    subgraph "Hardware Resources"
        CPU[CPU Cores]
        GPU[GPU Processing]
        MEM[Memory]
        IO[I/O Resources]
    end

    subgraph "Resource Manager"
        RES_DETECT[Resource Detection]
        RES_ALLOC[Resource Allocation]
        RES_MONITOR[Resource Monitoring]
        PROC_MGR[Process Management]

        RES_DETECT --- RES_ALLOC
        RES_ALLOC --- RES_MONITOR
        RES_ALLOC --- PROC_MGR
    end

    subgraph "FEAGI Processes"
        NPU_P[NPU Process]
        BDU_P[BDU Process]
        API_P[API Process]
        PNS_P[PNS Process]

        PROC_MGR --- NPU_P
        PROC_MGR --- BDU_P
        PROC_MGR --- API_P
        PROC_MGR --- PNS_P
    end

    CPU --- RES_DETECT
    GPU --- RES_DETECT
    MEM --- RES_DETECT
    IO --- RES_DETECT

    NPU_P --- CPU
    NPU_P --- GPU
    NPU_P --- MEM
    BDU_P --- CPU
    BDU_P --- MEM
    API_P --- CPU
    API_P --- IO
    PNS_P --- CPU
    PNS_P --- IO
```

## Development and Runtime Flow

```mermaid
flowchart LR
    subgraph "Development Phase"
        GENOME_DESIGN[Genome Design]
        CORTICAL_MAP[Cortical Area Mapping]
        SYNAPSE_RULES[Synaptogenesis Rules]

        GENOME_DESIGN --> CORTICAL_MAP
        CORTICAL_MAP --> SYNAPSE_RULES
    end

    subgraph "Initialization Phase"
        GENOME_LOAD[Load Genome]
        VALIDATE[Validate Structure]
        GEN_CONNECT[Generate Connectome]

        GENOME_LOAD --> VALIDATE
        VALIDATE --> GEN_CONNECT
    end

    subgraph "Runtime Phase"
        SENSORY_INPUT[Process Sensory Input]
        FCL_UPDATE[Update FCL]
        NEURAL_COMP[Neural Computation]
        PLASTICITY[Apply Plasticity]
        MOTOR_OUTPUT[Generate Motor Output]

        SENSORY_INPUT --> FCL_UPDATE
        FCL_UPDATE --> NEURAL_COMP
        NEURAL_COMP --> PLASTICITY
        PLASTICITY --> FCL_UPDATE
        NEURAL_COMP --> MOTOR_OUTPUT
    end

    SYNAPSE_RULES --> GENOME_LOAD
    GEN_CONNECT --> SENSORY_INPUT

    subgraph "Runtime Modification"
        RUNTIME_MOD[Runtime Modification]
        GENOME_UPDATE[Update Genome]
        CONNECT_SYNC[Synchronize Connectome]

        RUNTIME_MOD --> GENOME_UPDATE
        GENOME_UPDATE --> CONNECT_SYNC
        CONNECT_SYNC --> NEURAL_COMP
    end
```

## Notes on Using These Diagrams

- These diagrams are designed for inclusion in documentation and presentations
- The Mermaid format allows easy updates as the architecture evolves
- Diagrams can be rendered directly in GitHub markdown and many documentation systems
- For complex interactions, consider using the sequence diagram format
- Use consistent naming and color schemes across diagrams for clarity
