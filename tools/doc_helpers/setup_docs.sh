#!/bin/bash
#
# Copyright 2025 Neuraville Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

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
