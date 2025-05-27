# FEAGI - Framework for Evolutionary Artificial General Intelligence

FEAGI (Framework for Evolutionary Artificial General Intelligence) is a brain-inspired cognitive architecture and framework.

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
4. **Enhanced FQ Sampler**: Differentiated sampling for visualization vs. motor streams

### Differentiated Data Streams

FEAGI implements **dual-path FQ sampling** for optimized data delivery:

- **Visualization Stream (Port 5562)**: Comprehensive brain state monitoring with all cortical areas at configurable rates
- **Motor Stream (Port 5564)**: Real-time motor control with OPU (Output Processing Unit) areas only at burst frequency

This architecture ensures optimal performance for different use cases:
- Research and analysis tools receive comprehensive neural data via visualization stream
- Robotic controllers receive optimized, low-latency motor data via motor stream

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

### Documentation Website

The documentation is organized using Docusaurus, which provides a centralized documentation website. To set up and run the documentation website:

```bash
# Set up the documentation system
./tools/doc_helpers/setup_docs.sh

# Run the documentation server
./tools/doc_helpers/run_docusaurus.sh
```

The documentation website presents:
- **User Guides** - For end users of FEAGI
- **System Documentation** - Architectural and design documentation
- **Module Documentation** - API, BDU, NPU, and other module-specific documentation

Several utilities are available in `tools/doc_helpers/` to assist with documentation:
- `fix_html_tables.py` - Identifies HTML formatting issues in tables
- `fix_broken_links.py` - Detects and suggests fixes for broken links
- `convert_html_tables.py` - Converts HTML tables to markdown format

## Getting Started

### Prerequisites

- Python 3.9+
- ZeroMQ

### Installation

#### Standard Installation

```bash
# Clone the repository
git clone https://github.com/neuraville/feagi.git
cd feagi_core

# Install using pip (recommended)
pip install -e .
```

#### Alternative Installation (using requirements.txt)

For compatibility with tools that don't support `pyproject.toml`:

```bash
# Install core dependencies only
pip install -r requirements.txt

# Or install with development dependencies
pip install -r requirements-dev.txt
```

**Note**: The `requirements.txt` files are automatically generated from `pyproject.toml` and should be kept in sync. To update them, run:

```bash
python scripts/update_requirements.py
```

#### Dependencies

FEAGI Core requires Python 3.8+ and the following main packages:
- `fastapi>=0.95.0` - Web framework and API
- `uvicorn>=0.21.0` - ASGI server
- `pydantic>=2.0.0` - Data validation and configuration
- `numpy>=1.24.0` - Scientific computing
- `scipy>=1.10.0` - Scientific computing
- `torch>=2.0.0` - Neural processing
- `pyzmq>=26.0.0` - ZMQ communication
- `PyYAML>=6.0.0` - Configuration management
- `feagi-bytes>=0.1.0` - FEAGI byte structures
- `psutil>=5.8.0` - System monitoring
- And other supporting libraries for authentication, security, and data processing

For development work, additional dependencies include testing frameworks (pytest), code formatting tools (black, ruff), and profiling tools.

### Running FEAGI

```bash
./run_feagi.sh
```

## Connecting to FEAGI

FEAGI provides client connectors for various languages:

### Python

```python
from feagi_connector_old import FeagiClient

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

This project is licensed under the Apache 2.0 License - see the LICENSE file for details. 