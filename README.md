# FEAGI - Flexible & Extensible Artificial General Intelligence

FEAGI (Flexible & Extensible Artificial General Intelligence) is a brain-inspired cognitive architecture and framework.

## Architecture Overview

FEAGI is built with a modular architecture consisting of:

- **Core Cognitive System**: The main FEAGI brain system
- **Protocol Layer**: Language-agnostic communication protocols
- **API Layer**: REST, WebSocket and ZeroMQ interfaces
- **Client Connectors**: Libraries for different languages to connect to FEAGI

## Communication Protocols

FEAGI uses binary protocols for efficient communication:

1. **FCP (FEAGI Control Protocol)**: For agent registration, heartbeats, and control
2. **FSMP (FEAGI Sensorimotor Protocol)**: For exchanging sensory and motor data
3. **FVP (FEAGI Visualization Protocol)**: For exchanging brain visualization data

## Documentation

FEAGI documentation follows a structured approach:
- **System-level documentation** is maintained in the `/docs` folder
- **Module-specific documentation** is stored in each module's directory

All documentation follows our [Documentation Standards](docs/guide-documentation-standards.md) with consistent naming conventions:
- `arch-*`: Architecture documents
- `spec-*`: Technical specifications
- `guide-*`: User and developer guides
- `adr-*`: Architecture Decision Records
- `plan-*`: Project planning documents

For more details, see the [Documentation Restructuring Plan](docs/plan-documentation-restructuring.md).

## Getting Started

### Prerequisites

- Python 3.9+
- ZeroMQ

### Installation

```bash
# Clone the repository
git clone https://github.com/your-username/feagi.git
cd feagi

# Install dependencies
pip install -r requirements.txt
```

### Running FEAGI

```bash
./run_feagi.sh
```

## Connecting to FEAGI

FEAGI provides client connectors for various languages:

### Python

```python
from feagi_connector import FeagiClient

async def main():
    client = FeagiClient(host="localhost")
    await client.connect()
    
    # Send sensory data
    await client.send_sensory_data(channel_id=1, data=my_data)
    
    await client.disconnect()
```

### Other Languages

For other languages, use the Protocol Buffer definitions in `protocol/` to generate client libraries:

#### JavaScript

```javascript
// Using generated protobuf code
const feagi = require('./generated/feagi_protocol');

// Create client and connect
const client = new feagi.Client('localhost');
client.connect();

// Send sensory data
client.sendSensoryData(1, myData);
```

#### Rust

```rust
// Using generated protobuf code
use feagi_client::FeagiClient;

async fn main() -> Result<(), Box<dyn Error>> {
    let mut client = FeagiClient::new("localhost").await?;
    client.connect().await?;
    
    // Send sensory data
    client.send_sensory_data(1, &my_data).await?;
    
    client.disconnect().await?;
    Ok(())
}
```

## License

This project is licensed under the MIT License - see the LICENSE file for details. 