#!/bin/bash
# Build and install Rust crates for FEAGI Core
# Usage: 
#   ./build_rust.sh                    # Build all feagi-core crates
#   ./build_rust.sh feagi-python       # Build only specific crate
#   ./build_rust.sh feagi-burst-engine # Build only burst engine

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FEAGI_RUST_DIR="$SCRIPT_DIR/../feagi-rust"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo ""
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_info() {
    echo -e "${YELLOW}📦 $1${NC}"
}

# Install the built library to all Python import locations
install_library() {
    LIB_PATH="$FEAGI_RUST_DIR/target/release/libfeagi_rust.dylib"
    
    if [ ! -f "$LIB_PATH" ]; then
        # Try .so extension (Linux)
        LIB_PATH="$FEAGI_RUST_DIR/target/release/libfeagi_rust.so"
    fi
    
    if [ ! -f "$LIB_PATH" ]; then
        print_error "Built library not found at $LIB_PATH"
        return 1
    fi
    
    print_info "Installing to Python locations..."
    
    local INSTALLED_COUNT=0
    
    # Location 1: feagi/npu/feagi_rust.so
    DEST1="$SCRIPT_DIR/../feagi/npu/feagi_rust.so"
    mkdir -p "$(dirname "$DEST1")"
    cp "$LIB_PATH" "$DEST1"
    print_success "Installed: feagi/npu/feagi_rust.so"
    INSTALLED_COUNT=$((INSTALLED_COUNT + 1))
    
    # Location 2: venv site-packages (single file)
    for VENV_LIB in "$SCRIPT_DIR"/../.venv*/lib/python*/site-packages/feagi_rust.so; do
        if [ -e "$VENV_LIB" ]; then
            cp "$LIB_PATH" "$VENV_LIB"
            print_success "Installed: $(basename $(dirname $(dirname $(dirname "$VENV_LIB"))))/site-packages/feagi_rust.so"
            INSTALLED_COUNT=$((INSTALLED_COUNT + 1))
        fi
    done
    
    # Location 3: venv site-packages (package directory with abi3.so)
    for VENV_PKG in "$SCRIPT_DIR"/../.venv*/lib/python*/site-packages/feagi_rust/feagi_rust.abi3.so; do
        if [ -e "$VENV_PKG" ]; then
            cp "$LIB_PATH" "$VENV_PKG"
            print_success "Installed: $(basename $(dirname $(dirname $(dirname $(dirname "$VENV_PKG")))))/site-packages/feagi_rust/feagi_rust.abi3.so"
            INSTALLED_COUNT=$((INSTALLED_COUNT + 1))
        fi
    done
    
    # Show final status
    echo ""
    print_info "Library details:"
    ls -lh "$LIB_PATH" | awk '{print "  Size: " $5 ", Modified: " $6 " " $7 " " $8}'
    print_success "Installed to $INSTALLED_COUNT location(s)"
}

# Build specific crate or all
build_crate() {
    local CRATE="$1"
    
    if [ ! -d "$FEAGI_RUST_DIR" ]; then
        print_error "FEAGI Rust directory not found: $FEAGI_RUST_DIR"
        exit 1
    fi
    
    cd "$FEAGI_RUST_DIR"
    
    if [ -z "$CRATE" ] || [ "$CRATE" == "all" ]; then
        print_header "Building All FEAGI Core Crates"
        print_info "Building workspace in release mode..."
        
        if cargo build --release; then
            print_success "All crates built successfully"
            install_library
        else
            print_error "Build failed!"
            exit 1
        fi
    else
        print_header "Building FEAGI Crate: $CRATE"
        print_info "Building $CRATE in release mode..."
        
        if cargo build --release -p "$CRATE"; then
            print_success "Crate '$CRATE' built successfully"
            
            # Only install if this is feagi-python (the main Python library)
            if [ "$CRATE" == "feagi-python" ]; then
                install_library
            else
                print_info "Crate '$CRATE' built (no installation needed)"
            fi
        else
            print_error "Build failed for crate '$CRATE'!"
            exit 1
        fi
    fi
}

# Show available crates
show_crates() {
    print_header "Available FEAGI Core Crates"
    
    if [ ! -d "$FEAGI_RUST_DIR" ]; then
        print_error "FEAGI Rust directory not found: $FEAGI_RUST_DIR"
        exit 1
    fi
    
    cd "$FEAGI_RUST_DIR"
    
    echo "Workspace crates:"
    for crate_dir in crates/*/; do
        if [ -f "$crate_dir/Cargo.toml" ]; then
            crate_name=$(basename "$crate_dir")
            echo "  - $crate_name"
        fi
    done
    
    if [ -f "Cargo.toml" ]; then
        echo ""
        echo "Main package:"
        grep '^name = ' Cargo.toml | head -1 | sed 's/name = /  - /' | tr -d '"'
    fi
}

# Main script logic
main() {
    local TARGET="${1:-all}"
    
    print_header "FEAGI Core Rust Build System"
    echo "Working Directory: $FEAGI_RUST_DIR"
    
    case "$TARGET" in
        --list|list|-l)
            show_crates
            ;;
        --help|help|-h)
            echo "Usage: $0 [crate_name|all]"
            echo ""
            echo "Options:"
            echo "  all                              Build all crates (default)"
            echo "  feagi-python                     Build main Python library (includes auto-install)"
            echo "  feagi-burst-engine               Build burst engine"
            echo "  feagi-bdu                        Build Brain Development Unit"
            echo "  feagi-pns                        Build Peripheral Nervous System"
            echo "  feagi-plasticity                 Build plasticity engine"
            echo "  feagi-types                      Build shared type definitions"
            echo "  feagi-connectome-serialization   Build connectome serialization"
            echo "  feagi-agent-registry             Build agent registry"
            echo "  feagi-agent-sdk                  Build agent SDK (Rust)"
            echo "  feagi-agent-sdk-py               Build agent SDK (Python bindings)"
            echo "  feagi-inference-engine           Build inference engine (standalone binary)"
            echo "  --list                           Show all available crates"
            echo ""
            echo "Examples:"
            echo "  $0                           # Build all crates"
            echo "  $0 feagi-python              # Build only Python library"
            echo "  $0 feagi-burst-engine        # Build only burst engine"
            echo "  $0 feagi-inference-engine    # Build inference engine binary"
            ;;
        *)
            build_crate "$TARGET"
            print_header "Build Complete! 🎉"
            ;;
    esac
}

main "$@"
