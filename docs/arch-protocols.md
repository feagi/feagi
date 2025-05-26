# FEAGI Protocol Architecture Diagram

*Last Updated: May 15, 2025*

This file contains Mermaid diagrams illustrating the protocol architecture of FEAGI.

## Protocol Architecture Overview

```mermaid
flowchart TB
    subgraph "FEAGI Core"
        CoreAPIService["CoreAPIService"]
        ByteTranslator["ByteStructure Translator"]
        
        subgraph "Protocol Handlers"
            FCPHandler["FCP Handler"]
            FVPHandler["FVP Handler"] 
            FSMPHandler["FSMP Handler"]
        end
        
        subgraph "ZMQ Router Server"
            ControlSocket["ROUTER Socket\n(FCP)"]
            VisualizationSocket["ROUTER Socket\n(FVP)"]
            SensorimotorSocket["ROUTER Socket\n(FSMP)"]
        end
    end
    
    subgraph "External Agent"
        AgentApp["Agent Application"]
        
        subgraph "ZMQ Client"
            ControlClient["DEALER Socket\n(FCP)"]
            VisualizationClient["DEALER Socket\n(FVP)"]
            SensorimotorClient["DEALER Socket\n(FSMP)"]
        end
    end
    
    %% Core connections
    CoreAPIService --> ByteTranslator
    ByteTranslator --> FCPHandler & FVPHandler & FSMPHandler
    FCPHandler --> ControlSocket
    FVPHandler --> VisualizationSocket
    FSMPHandler --> SensorimotorSocket
    
    %% ZMQ connections
    ControlSocket <---> ControlClient
    VisualizationSocket <---> VisualizationClient
    SensorimotorSocket <---> SensorimotorClient
    
    %% Agent connections
    ControlClient & VisualizationClient & SensorimotorClient --- AgentApp
    
    %% Styling
    classDef core fill:#f9f,stroke:#333,stroke-width:2px
    classDef protocol fill:#bbf,stroke:#333,stroke-width:1px
    classDef zmq fill:#bfb,stroke:#333,stroke-width:1px
    classDef agent fill:#fbb,stroke:#333,stroke-width:1px
    
    class CoreAPIService,ByteTranslator core
    class FCPHandler,FVPHandler,FSMPHandler protocol
    class ControlSocket,VisualizationSocket,SensorimotorSocket,ControlClient,VisualizationClient,SensorimotorClient zmq
    class AgentApp agent
```

## Protocol Message Flow

```mermaid
sequenceDiagram
    participant Agent as External Agent
    participant ZMQ as ZMQ Router Server
    participant Translator as ByteStructure Translator
    participant Core as CoreAPIService
    
    %% Handshake sequence
    Agent->>ZMQ: HELLO Message
    ZMQ->>Translator: Raw Message
    Translator->>Core: Decoded Message
    Core->>Translator: Welcome Response
    Translator->>ZMQ: Encoded Message
    ZMQ->>Agent: WELCOME Message
    
    Agent->>ZMQ: CAPABILITIES Message
    ZMQ->>Translator: Raw Message
    Translator->>Core: Decoded Message
    Core->>Translator: Config Response
    Translator->>ZMQ: Encoded Message
    ZMQ->>Agent: CONFIG Message
    
    %% Registration sequence
    Agent->>ZMQ: REGISTER Message
    ZMQ->>Translator: Raw Message
    Translator->>Core: Decoded Message
    Core->>Translator: Registration Response
    Translator->>ZMQ: Encoded Message
    ZMQ->>Agent: REGISTER_RESPONSE Message
    
    %% Regular communication
    loop Communication
        Agent->>ZMQ: Protocol-specific Messages
        ZMQ->>Translator: Raw Message
        Translator->>Core: Decoded Message
        Core->>Translator: Response Data
        Translator->>ZMQ: Encoded Message
        ZMQ->>Agent: Response Message
    end
    
    %% Heartbeat
    loop Every 15 seconds
        Agent->>ZMQ: HEARTBEAT Message
        ZMQ->>Agent: HEARTBEAT Response
    end
    
    %% Disconnection
    Agent->>ZMQ: DISCONNECT Message
    ZMQ->>Translator: Raw Message
    Translator->>Core: Decoded Message
    Core->>Core: Cleanup Resources
```

## Protocol Binary Structure

```mermaid
classDiagram
    class MessageHeader {
        +uint32 protocol_id
        +uint8 message_type
        +uint32 payload_length
    }
    
    class FCPMessage {
        +MessageHeader header
        +FCPPayload payload
    }
    
    class FVPMessage {
        +MessageHeader header
        +FVPPayload payload
    }
    
    class FSMPMessage {
        +MessageHeader header
        +FSMPPayload payload
    }
    
    class FCPPayload {
        +varies by message_type
    }
    
    class FVPPayload {
        +uint64 timestamp
        +varies by message_type
    }
    
    class FSMPPayload {
        +uint64 timestamp
        +uint16 channel_id
        +uint8[] data
    }
    
    MessageHeader <|-- FCPMessage
    MessageHeader <|-- FVPMessage
    MessageHeader <|-- FSMPMessage
    
    FCPMessage o-- FCPPayload
    FVPMessage o-- FVPPayload
    FSMPMessage o-- FSMPPayload
``` 