#!/usr/bin/env python
"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""FEAGI Dependency Checker.

This script checks if all required dependencies are installed with the correct versions.
It can be run directly to verify that the environment is properly set up.
"""
import sys

from feagi.utils.logger import setup_logger

logger = setup_logger()
import argparse
import logging
from pathlib import Path

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("feagi.dependency_checker")


def main():
    """Check if all required dependencies are installed with the correct versions."""
    parser = argparse.ArgumentParser(description="FEAGI Dependency Checker")
    parser.add_argument(
        "--requirements", type=str, help="Path to requirements.txt file"
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Exit with error code if dependencies don't match",
    )
    args = parser.parse_args()

    # Get the path to requirements.txt
    if args.requirements:
        requirements_path = args.requirements
    else:
        # Try to find requirements.txt in the project root
        feagi_root = Path(__file__).parent.parent.parent
        requirements_path = str(feagi_root / "requirements.txt")

    # Import the version checker
    try:
        sys.path.insert(0, str(Path(__file__).parent.parent.parent))
        from feagi.utils.version_checker import verify_dependencies

        # Check dependencies
        is_compatible = verify_dependencies(
            requirements_path, raise_exception=False
        )

        if is_compatible:
            logger.info(
                "[OK]", "All dependencies are compatible with requirements"
            )
            return 0
        else:
            if args.strict:
                return 1
            else:
                # Just warn but return success
                return 0

    except Exception as e:
        logger.error(f"Error checking dependencies: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
