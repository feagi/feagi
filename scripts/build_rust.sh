#!/bin/bash
# Build and install Rust crates for FEAGI Core
# Usage: 
#   ./build_rust.sh                    # Build all feagi-core crates
#   ./build_rust.sh feagi-python       # Build only specific crate
#   ./build_rust.sh feagi-burst-engine # Build only burst engine
#
# Cross-platform compatible: Linux, macOS, Windows (Git Bash/WSL)

# Don't exit immediately on error - we want to show messages and pause
set +e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FEAGI_RUST_DIR="$SCRIPT_DIR/../feagi-rust"

# Detect OS for platform-specific behavior
OS_TYPE="$(uname -s 2>/dev/null || echo "Windows")"
case "${OS_TYPE}" in
    Linux*)     PLATFORM="Linux";;
    Darwin*)    PLATFORM="macOS";;
    CYGWIN*)    PLATFORM="Windows";;
    MINGW*)     PLATFORM="Windows";;
    MSYS*)      PLATFORM="Windows";;
    *)          PLATFORM="Unknown";;
esac

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

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

# Pause before exit on error (especially useful on Windows)
pause_on_error() {
    local exit_code=$1
    local message="$2"
    
    if [ $exit_code -ne 0 ]; then
        echo ""
        print_error "$message"
        echo ""
        print_warning "Press any key to exit..."
        read -n 1 -s -r 2>/dev/null || pause 2>/dev/null
        exit $exit_code
    fi
}

# Install the built library to all Python import locations
install_library() {
    # Detect library extension based on platform
    case "${PLATFORM}" in
        macOS)
            LIB_EXT="dylib"
            LIB_NAME="libfeagi_rust.dylib"
            ;;
        Linux)
            LIB_EXT="so"
            LIB_NAME="libfeagi_rust.so"
            ;;
        Windows)
            LIB_EXT="dll"
            LIB_NAME="feagi_rust.dll"  # Windows doesn't use lib prefix
            ;;
        *)
            # Try all extensions
            for ext in dylib so dll; do
                if [ -f "$FEAGI_RUST_DIR/target/release/libfeagi_rust.$ext" ]; then
                    LIB_EXT="$ext"
                    LIB_NAME="libfeagi_rust.$ext"
                    break
                elif [ -f "$FEAGI_RUST_DIR/target/release/feagi_rust.$ext" ]; then
                    LIB_EXT="$ext"
                    LIB_NAME="feagi_rust.$ext"
                    break
                fi
            done
            ;;
    esac
    
    LIB_PATH="$FEAGI_RUST_DIR/target/release/$LIB_NAME"
    
    if [ ! -f "$LIB_PATH" ]; then
        print_error "Built library not found at $LIB_PATH"
        print_info "Expected platform: $PLATFORM"
        print_info "Checked for: $LIB_NAME"
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
        pause_on_error 1 "Cannot continue without Rust source directory"
    fi
    
    cd "$FEAGI_RUST_DIR"
    
    print_info "Platform detected: $PLATFORM"
    print_info "Building on: $(rustc --version 2>/dev/null || echo 'Rust not found!')"
    
    if [ -z "$CRATE" ] || [ "$CRATE" == "all" ]; then
        print_header "Building All FEAGI Core Crates"
        print_info "Building workspace and Python extension module..."
        echo ""
        
        # Check if cargo is available
        if ! command -v cargo &> /dev/null; then
            print_error "cargo not found! Please install Rust from https://rustup.rs/"
            pause_on_error 1 "Rust toolchain not available"
        fi
        
        # Build Rust workspace (excluding crates with special requirements)
        # - feagi-agent-sdk-py: Needs separate maturin build
        print_info "Building Rust workspace (this may take a few minutes)..."
        if cargo build --release --workspace --exclude feagi-agent-sdk-py; then
            print_success "All Rust crates built successfully"
            echo ""
            
            # Build and install feagi-python using maturin
            print_info "Building Python extension module with maturin..."
            if command -v maturin &> /dev/null; then
                cd "$FEAGI_RUST_DIR/crates/feagi-python"
                
                # Check if Python venv is active
                if [ -z "$VIRTUAL_ENV" ]; then
                    print_warning "No Python virtual environment detected!"
                    print_warning "The module will be installed to system Python."
                    print_warning "Consider activating a venv first: source .venv_feagi/bin/activate"
                    echo ""
                fi
                
                if maturin develop --release; then
                    print_success "Python extension module installed successfully"
                    
                    # Verify installation
                    if python3 -c "import feagi_rust" 2>/dev/null; then
                        print_success "✓ feagi_rust module can be imported"
                    else
                        print_warning "feagi_rust module built but import failed - check Python environment"
                    fi
                else
                    cd "$FEAGI_RUST_DIR"
                    pause_on_error 1 "Failed to build Python extension module. Try manually: cd feagi-rust/crates/feagi-python && maturin develop --release"
                fi
                cd "$FEAGI_RUST_DIR"
            else
                print_error "maturin not found! Install it with: pip install maturin"
                print_info "Then run: cd feagi-rust/crates/feagi-python && maturin develop --release"
                echo ""
                print_warning "Continuing without Python extension module..."
                print_warning "FEAGI will not work properly without this module!"
            fi
        else
            pause_on_error 1 "Cargo build failed! Check errors above."
        fi
    else
        print_header "Building FEAGI Crate: $CRATE"
        print_info "Building $CRATE in release mode..."
        
        if cargo build --release -p "$CRATE"; then
            print_success "Crate '$CRATE' built successfully"
            
            # For feagi-python, use maturin instead of install_library
            if [ "$CRATE" == "feagi-python" ]; then
                print_info "Building Python extension module with maturin..."
                if command -v maturin &> /dev/null; then
                    cd "$FEAGI_RUST_DIR/crates/feagi-python"
                    if maturin develop --release; then
                        print_success "Python extension module installed successfully"
                    else
                        cd "$FEAGI_RUST_DIR"
                        pause_on_error 1 "Failed to install Python extension module!"
                    fi
                    cd "$FEAGI_RUST_DIR"
                else
                    print_error "maturin not found! Install it with: pip install maturin"
                    pause_on_error 1 "Cannot build feagi-python without maturin"
                fi
            else
                print_info "Crate '$CRATE' built (no installation needed)"
            fi
        else
            pause_on_error 1 "Build failed for crate '$CRATE'! Check errors above."
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
            echo "  feagi-python                     Build main Python library (requires maturin)"
            echo "  feagi-burst-engine               Build burst engine (core neural processing)"
            echo "  feagi-bdu                        Build Brain Development Unit"
            echo "  feagi-pns                        Build Peripheral Nervous System"
            echo "  feagi-plasticity                 Build plasticity engine (STDP, learning)"
            echo "  feagi-types                      Build shared type definitions"
            echo "  feagi-connectome-serialization   Build connectome serialization"
            echo "  feagi-agent-sdk                  Build agent SDK (Rust)"
            echo "  feagi-agent-sdk-py               Build agent SDK (Python bindings, requires maturin)"
            echo "  feagi-inference-engine           Build standalone inference engine (experimental)"
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
            echo ""
            print_info "Platform: $PLATFORM"
            print_info "Next steps:"
            echo "  1. Activate your Python environment"
            echo "  2. Run FEAGI: python -m feagi.main --genome <genome_file>"
            echo ""
            ;;
    esac
}

# Run main function
main "$@"
BUILD_EXIT_CODE=$?

# On Windows or if build failed, pause before exit
if [ "${PLATFORM}" == "Windows" ] || [ $BUILD_EXIT_CODE -ne 0 ]; then
    echo ""
    print_info "Press any key to exit..."
    read -n 1 -s -r 2>/dev/null || pause 2>/dev/null
fi

exit $BUILD_EXIT_CODE
