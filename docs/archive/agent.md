# FEAGI Agent System

## Overview

The FEAGI Agent System enables external components to register, communicate, and interact with the FEAGI brain. Agents serve as interface points between FEAGI's neural processing system and the outside world, including sensors, actuators, monitoring tools, and other external systems.

## Core Concepts

### Agent Types

FEAGI supports various agent types:

- **Monitor Agents**: Connect to FEAGI to observe neural activity without modifying it
- **Robot/Device Agents**: Interface with physical devices providing sensory inputs and motor outputs
- **Custom Agents**: Any specialized interface components with defined capabilities

### Registration Process

Agents register with FEAGI by:

1. Providing identity information (agent_id, type, version)
2. Declaring their capabilities (sensors, actuators, etc.)
3. Receiving connection information (ports, addresses) from FEAGI
4. Establishing a communication channel based on this information

### Communication Channels

FEAGI uses a combination of:

- **ZeroMQ (ZMQ)** sockets for high-speed message passing
- **REST API** endpoints for administrative operations
- **WebSockets** for monitoring and visualization data

### Capability Mapping

When agents register with capabilities, FEAGI can:

1. Auto-create corresponding neural structures (PNS areas)
2. Map agent inputs to specific cortical areas
3. Connect outputs from cortical areas to agent actuators

## Architecture

```
┌──────────────────┐     ┌───────────────────┐
│  External Agent  │     │      FEAGI        │
│                  │     │                   │
│  ┌────────────┐  │     │  ┌────────────┐   │
│  │ Sensors/   │  │     │  │  Agent     │   │
│  │ Actuators  │◄─┼─────┼─►│  Registry  │   │
│  └────────────┘  │     │  └────────────┘   │
│                  │     │        │          │
│  ┌────────────┐  │     │        ▼          │
│  │ ZMQ Client │◄─┼─────┼─►┌────────────┐   │
│  └────────────┘  │     │  │  Neural    │   │
└──────────────────┘     │  │  Processing│   │
                         │  └────────────┘   │
                         └───────────────────┘
```

## Agent Registration Flow

1. Agent sends registration request with capabilities
2. FEAGI validates the agent type and capabilities
3. FEAGI assigns communication parameters (port, address)
4. FEAGI creates necessary neural structures (if auto-creation enabled)
5. Agent begins regular communication with FEAGI

## Agent Capabilities

Capabilities are defined as a structured collection of:

- **Sensors**: Input devices that generate signals (e.g., camera, microphone)
- **Actuators**: Output mechanisms (e.g., motors, speakers)
- **Processing**: Special computational capabilities
- **Configuration**: Specific parameters the agent supports

## Implementation

The CoreAPIService serves as the central point for agent management with methods for:

- `get_agent_list()`: Retrieve all registered agents
- `get_agent_properties(agent_id)`: Get properties of a specific agent
- `register_agent(...)`: Register a new agent with FEAGI
- `deregister_agent(agent_id)`: Remove an agent from the registry
- `update_robot_controller(...)`: Update parameters for robot controllers
- `update_robot_model(...)`: Update robot model configuration
- `trigger_manual_stimulation(...)`: Send one-time stimulation patterns
- `trigger_sustained_stimulation(...)`: Send continuous stimulation patterns

## Security Considerations

- Agent connections should be authenticated
- Communication channels can be encrypted
- Permissions for agent actions are configurable

## Future Directions

- Agent discovery protocol for automatic connection
- Enhanced capability negotiation
- Distributed agent networks with mesh topologies
- Learning from agent interaction patterns 