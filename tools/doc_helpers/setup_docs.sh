#!/bin/bash
# Script to set up the FEAGI documentation system

echo "Setting up FEAGI documentation system..."

# Go to the Docusaurus directory
cd "$(dirname "$0")/../../website/docs" || { echo "Could not navigate to Docusaurus directory"; exit 1; }

# Install Node.js dependencies
echo "Installing Node.js dependencies..."
npm install

# Go back to the main directory
cd "$(dirname "$0")/../../" || { echo "Could not navigate back to main directory"; exit 1; }

# Install Python dependencies for documentation tools
echo "Installing Python dependencies for documentation tools..."
pip install beautifulsoup4

# Create placeholder directories if they don't exist
echo "Creating placeholder directories..."
mkdir -p docs/assets
mkdir -p website/docs/static/img/diagrams
mkdir -p website/docs/static/img/placeholders

# Create placeholder images if they don't exist
if [ ! -f docs/assets/bdu-architecture.png ]; then
    echo "[This is a placeholder for the BDU architecture diagram]" > docs/assets/bdu-architecture.png
    echo "Created placeholder for BDU architecture diagram"
fi

if [ ! -f docs/assets/state-manager-diagram.png ]; then
    echo "[This is a placeholder for the state manager diagram]" > docs/assets/state-manager-diagram.png
    echo "Created placeholder for state manager diagram"
fi

# Make sure all the scripts are executable
echo "Making scripts executable..."
chmod +x tools/doc_helpers/*.py
chmod +x tools/doc_helpers/*.sh

echo "Setup complete! You can now run the Docusaurus server with:"
echo "./tools/doc_helpers/run_docusaurus.sh"
echo ""
echo "If you encounter issues, check the troubleshooting section in:"
echo "tools/doc_helpers/README.md" 