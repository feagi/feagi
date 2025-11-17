# FEAGI Installation Guide

This guide provides instructions for installing and setting up FEAGI (Flexible & Extensible Artificial General Intelligence).

## Prerequisites

Before installing FEAGI, ensure you have the following prerequisites:

- Python 3.9 or higher
- pip (Python package installer)
- Git
- A CUDA-compatible GPU (optional, but recommended for optimal performance)

## Option 1: Standard Installation

### 1. Clone the Repository

```bash
git clone https://github.com/feagi/feagi.git
cd feagi
```

### 2. Create a Virtual Environment

```bash
python -m venv .venv
```

### 3. Activate the Virtual Environment

**On Linux/macOS:**
```bash
source .venv/bin/activate
```

**On Windows:**
```bash
.venv\Scripts\activate
```

### 4. Install FEAGI

```bash
pip install -e .
```

This will install FEAGI in development mode, allowing you to make changes to the code and see them reflected immediately.

## Option 2: Installation Using Requirements File

If you prefer to install specific dependencies:

### 1. Clone the Repository

```bash
git clone https://github.com/feagi/feagi.git
cd feagi
```

### 2. Create and Activate a Virtual Environment (as shown above)

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

## Option 3: Docker Installation

For isolated environments, Docker is recommended:

### 1. Install Docker

Follow the [official Docker installation guide](https://docs.docker.com/get-docker/) for your operating system.

### 2. Pull and Run the FEAGI Docker Image

```bash
docker pull feagi/feagi:latest
docker run -p 8000:8000 -p 5002:5002 feagi/feagi:latest
```

## Verifying Installation

To verify that FEAGI has been installed correctly:

```bash
python -c "import feagi; print(feagi.__version__)"
```

## Running FEAGI

### Standard Run

```bash
python run_feagi.sh
```

### Development Mode

```bash
python run_feagi.sh --dev
```

## Installing FEAGI Connector (Optional)

To install the FEAGI Connector module for building external agents:

```bash
./install_feagi_connector.sh
```

## Configuration

FEAGI's configuration can be customized by editing the configuration files in the `feagi/config` directory. Key configuration files include:

- `default_config.json`: Default configuration values
- `runtime_config.json`: Runtime-specific settings
- `network_config.json`: Network and protocol settings

## Troubleshooting

### Common Issues

#### Missing Dependencies

If you encounter missing dependencies, try:
```bash
pip install -r requirements.txt
```

#### GPU Detection Problems

If FEAGI doesn't detect your GPU:
1. Ensure CUDA is properly installed
2. Check that your GPU drivers are up to date
3. Verify your GPU is compatible with WebGPU

#### Port Conflicts

If you encounter port conflicts:
1. Check if another process is using ports 8000 or 5002
2. Modify the port settings in the configuration files

## Next Steps

After installation, you can:

1. Follow the [Quick Start Guide](guide-usage.md) to run your first FEAGI instance
2. Explore the [Architecture Overview](arch-system-overview.md) to understand the system
3. Check the [API Documentation](spec-api-formats.md) to learn how to interact with FEAGI

## Additional Resources

- [GitHub Repository](https://github.com/feagi/feagi)
- [Issue Tracker](https://github.com/feagi/feagi/issues)
- [Community Forum](https://feagi.community)
