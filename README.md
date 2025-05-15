# FEAGI - Flexible & Extensible Artificial General Intelligence

FEAGI (Flexible & Extensible Artificial General Intelligence) is a brain-inspired cognitive architecture and framework.

## Architecture Overview

FEAGI is built with a modular architecture consisting of:

- **Core Cognitive System**: The main FEAGI brain system
- **Protocol Layer**: Language-agnostic communication protocols
- **API Layer**: REST, WebSocket and ZeroMQ interfaces
- **Client Connectors**: Libraries for different languages to connect to FEAGI

## Communication Protocols

FEAGI uses Protocol Buffers to define language-agnostic communication protocols:

1. **FCP (FEAGI Control Protocol)**: For agent registration, heartbeats, and control
2. **FSMP (FEAGI Sensorimotor Protocol)**: For exchanging sensory and motor data
3. **FVP (FEAGI Visualization Protocol)**: For exchanging brain visualization data

These protocols are defined in the `protocol/` directory and can be used to generate client libraries for any language that supports Protocol Buffers.

## Getting Started

### Prerequisites

- Python 3.9+
- Protocol Buffers compiler (protoc)

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
python run_feagi.sh
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

## Documentation

Full documentation is available in the `docs/` directory.

## License

This project is licensed under the MIT License - see the LICENSE file for details. 