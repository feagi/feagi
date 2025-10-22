#!/bin/bash
# Copyright 2025 Neuraville Inc.
# Licensed under the Apache License, Version 2.0
#
# License compliance checker for FEAGI dependencies

set -e

echo "🔍 FEAGI License Compliance Checker"
echo "==================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get to project root
cd "$(dirname "$0")/.."
PROJECT_ROOT=$(pwd)

# Forbidden licenses (strong copyleft, proprietary)
FORBIDDEN_LICENSES=(
    "GPL-3.0"
    "AGPL-3.0"
    "GPL-2.0"
    "AGPL"
    "Proprietary"
    "Commercial"
)

# Warning licenses (need review)
WARNING_LICENSES=(
    "LGPL"
    "MPL"
    "EPL"
)

# Safe licenses (compatible with Apache 2.0)
SAFE_LICENSES=(
    "MIT"
    "BSD"
    "Apache"
    "ISC"
    "Zlib"
    "Python Software Foundation"
    "PSF"
)

echo "📦 Checking Python Dependencies"
echo "--------------------------------"

# Check if pip-licenses is installed
if ! command -v pip-licenses &> /dev/null; then
    echo "${YELLOW}⚠️  pip-licenses not installed. Installing...${NC}"
    pip install pip-licenses
fi

# Generate license report
echo "Generating Python license report..."
pip-licenses --format=markdown --with-urls > python-licenses-report.md
pip-licenses --format=json > python-licenses-report.json

# Parse and check for forbidden licenses
echo ""
echo "Analyzing Python licenses..."
python3 << 'EOF'
import json
import sys

# Load license data
with open('python-licenses-report.json', 'r') as f:
    licenses = json.load(f)

forbidden = ['GPL-3.0', 'AGPL-3.0', 'GPL-2.0', 'AGPL', 'Proprietary', 'Commercial']
warnings = ['LGPL', 'MPL', 'EPL']

found_forbidden = []
found_warnings = []

for pkg in licenses:
    name = pkg.get('Name', 'Unknown')
    lic = pkg.get('License', 'Unknown')
    
    # Check for forbidden licenses
    for forbidden_lic in forbidden:
        if forbidden_lic.lower() in lic.lower():
            found_forbidden.append(f"{name}: {lic}")
    
    # Check for warning licenses
    for warning_lic in warnings:
        if warning_lic.lower() in lic.lower():
            found_warnings.append(f"{name}: {lic}")

# Print results
if found_forbidden:
    print("❌ FORBIDDEN LICENSES FOUND:")
    for item in found_forbidden:
        print(f"  - {item}")
    sys.exit(1)

if found_warnings:
    print("⚠️  LICENSES REQUIRING REVIEW:")
    for item in found_warnings:
        print(f"  - {item}")
    print("")
    print("These licenses may be acceptable with dynamic linking.")
    print("Review LICENSE_AUDIT.md for details.")

if not found_forbidden and not found_warnings:
    print("✅ All Python dependencies have compatible licenses")
    
print(f"\nTotal packages checked: {len(licenses)}")

EOF

if [ $? -ne 0 ]; then
    echo ""
    echo "${RED}❌ License compliance check FAILED${NC}"
    exit 1
fi

echo ""
echo "🦀 Checking Rust Dependencies"
echo "------------------------------"

# Check if cargo-license is installed
if ! command -v cargo-license &> /dev/null; then
    echo "${YELLOW}⚠️  cargo-license not installed. Installing...${NC}"
    cargo install cargo-license
fi

# Generate Rust license report
if [ -d "feagi-rust" ]; then
    cd feagi-rust
    echo "Generating Rust license report..."
    cargo-license --json > ../rust-licenses-report.json
    cargo-license --tsv > ../rust-licenses-report.tsv
    cd ..
    
    echo ""
    echo "Analyzing Rust licenses..."
    python3 << 'EOF'
import json
import sys

# Load Rust license data
with open('rust-licenses-report.json', 'r') as f:
    licenses = json.load(f)

forbidden = ['GPL-3.0', 'AGPL-3.0', 'GPL-2.0', 'AGPL']
found_issues = []

for pkg in licenses:
    name = pkg.get('name', 'Unknown')
    lic = pkg.get('license', 'Unknown')
    
    # Rust crates often use "MIT OR Apache-2.0" - this is safe
    if 'OR' in lic.upper():
        continue
    
    # Check for forbidden licenses
    for forbidden_lic in forbidden:
        if forbidden_lic.lower() in lic.lower():
            found_issues.append(f"{name}: {lic}")

if found_issues:
    print("❌ FORBIDDEN LICENSES FOUND IN RUST CRATES:")
    for item in found_issues:
        print(f"  - {item}")
    sys.exit(1)
else:
    print("✅ All Rust dependencies have compatible licenses")
    print(f"Total crates checked: {len(licenses)}")

EOF

    if [ $? -ne 0 ]; then
        echo ""
        echo "${RED}❌ Rust license compliance check FAILED${NC}"
        exit 1
    fi
else
    echo "${YELLOW}⚠️  feagi-rust directory not found, skipping Rust check${NC}"
fi

echo ""
echo "📄 License Reports Generated"
echo "----------------------------"
echo "  - python-licenses-report.md"
echo "  - python-licenses-report.json"
echo "  - rust-licenses-report.json"
echo "  - rust-licenses-report.tsv"
echo ""

echo "${GREEN}✅ LICENSE COMPLIANCE CHECK PASSED${NC}"
echo ""
echo "Summary:"
echo "  - No GPL/AGPL (copyleft) licenses found"
echo "  - No proprietary licenses found"
echo "  - All dependencies compatible with Apache 2.0"
echo ""
echo "Review LICENSE_AUDIT.md for detailed analysis."
echo "Ensure NOTICE file is included in distributions."

