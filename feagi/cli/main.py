"""
FEAGI CLI Main Entry Point

Command-line tools for FEAGI development.

Commands:
- create-agent: Scaffold new agent from template
- build-package: Build marketplace package

TODO: Implement in Phase 4
"""

import sys


def main():
    """Main CLI entry point"""
    print("FEAGI CLI v3.0.0")
    print("\nAvailable commands:")
    print("  feagi create-agent   - Scaffold new agent from template (Coming in Phase 4)")
    print("  feagi build-package  - Build marketplace package (Coming in Phase 4)")
    print("\nFor more information, visit https://docs.feagi.org")
    sys.exit(0)


if __name__ == "__main__":
    main()

