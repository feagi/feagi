---
sidebar_position: 1
---

# Installation Guide

This guide provides step-by-step instructions for installing FEAGI on your system.

## Prerequisites

Before installing FEAGI, ensure you have the following prerequisites:

- **Python**: Version 3.9 or higher
- **Git**: For cloning the repository
- **Docker** (optional): For container-based deployment

## Installation Methods

### Method 1: Direct Installation

1. **Clone the Repository**

   ```bash
   git clone https://github.com/feagi/feagi.git
   cd feagi
   ```

2. **Create a Virtual Environment**

   ```bash
   python -m venv .venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   ```

3. **Install Dependencies**

   ```bash
   pip install -r requirements.txt
   ```

4. **Install FEAGI**

   ```bash
   pip install -e .
   ```

### Method 2: Using Docker

1. **Pull the Docker Image**

   ```bash
   docker pull feagi/feagi:latest
   ```

2. **Run FEAGI Container**

   ```bash
   docker run -p 8000:8000 -p 9000:9000 feagi/feagi:latest
   ```

## Verifying Installation

To verify that FEAGI is installed correctly:

1. **Run the FEAGI Server**

   ```bash
   python run_feagi.py
   ```

2. **Access the Web Interface**

   Open your browser and go to: [http://localhost:8000](http://localhost:8000)

3. **Check API Documentation**

   Access the API documentation at: [http://localhost:8000/docs](http://localhost:8000/docs)

## Next Steps

Now that you have installed FEAGI, proceed to the [Quick Start Guide](quick-start) to begin using it.

## Troubleshooting

If you encounter any issues during installation:

- Check that Python and pip are properly installed
- Ensure your virtual environment is activated
- Verify that all dependencies were installed correctly

For more detailed troubleshooting information, refer to our [System Documentation](/system/guide-usage). 