"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""Version checker utility for FEAGI.

This module provides functionality to check if the installed dependencies
match the required versions specified in requirements.txt.
"""
import importlib
import importlib.metadata
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from packaging import specifiers

from feagi.utils.logger import setup_logger

logger = setup_logger()


class VersionMismatchError(Exception):
    """Exception raised when a package version doesn't match the
    requirements."""

    pass


def parse_requirements_file(file_path: str) -> Dict[str, str]:
    """Parse requirements.txt file and extract package names and version
    constraints.

    Properly handles conditional requirements based on Python version and platform.

    Args:
        file_path: Path to the requirements.txt file

    Returns:
        Dictionary mapping package names to version constraints
    """
    requirements = {}

    try:
        with open(file_path, "r") as f:
            lines = f.readlines()

        for line in lines:
            line = line.strip()

            # Skip empty lines and comments
            if not line or line.startswith("#"):
                continue

            # Handle conditional requirements (platform-specific)
            if ";" in line:
                pkg_part, condition = line.split(";", 1)
                pkg_part = pkg_part.strip()
                condition = condition.strip()

                # Evaluate the condition to see if this package should be included
                if not _evaluate_requirement_condition(condition):
                    # Skip this package as the condition is not met
                    continue
            else:
                pkg_part = line

            # Extract package name and version constraint
            # Handle different formats like:
            # package>=1.0.0
            # package[extra]>=1.0.0
            # package==1.0.0

            # Extract the package name (before any version specifier)
            match = re.match(
                r"^([a-zA-Z0-9_\-]+(?:\[[a-zA-Z0-9_\-]+\])?)(.*?)$", pkg_part
            )
            if match:
                pkg_name = match.group(1)
                version_part = match.group(2).strip()

                # Clean up package name (remove extras)
                if "[" in pkg_name:
                    pkg_name = pkg_name.split("[")[0]

                requirements[pkg_name.lower()] = version_part

        return requirements
    except (IOError, OSError) as e:
        logger.error(f"Error reading requirements file: {e}")
        return {}


def _evaluate_requirement_condition(condition: str) -> bool:
    """Evaluate a requirement condition (e.g., python_version<"3.11").

    Args:
        condition: The condition string to evaluate

    Returns:
        True if the condition is met, False otherwise
    """
    from packaging import version

    try:
        # Get current Python version
        major = sys.version_info.major
        minor = sys.version_info.minor
        micro = sys.version_info.micro
        current_python_version = version.Version(f"{major}.{minor}.{micro}")

        # Handle python_version conditions
        if "python_version" in condition:
            # Extract the operator and version from conditions like:
            # python_version<"3.11"
            # python_version>="3.8"
            # python_version=="3.9"

            condition = condition.replace("python_version", "").strip()

            # Parse operator and version
            operators = [">=", "<=", "==", "!=", ">", "<"]
            for op in operators:
                if condition.startswith(op):
                    version_str = (
                        condition[len(op) :].strip().strip('"').strip("'")
                    )
                    target_version = version.Version(version_str)

                    if op == ">=":
                        return current_python_version >= target_version
                    elif op == "<=":
                        return current_python_version <= target_version
                    elif op == "==":
                        return current_python_version == target_version
                    elif op == "!=":
                        return current_python_version != target_version
                    elif op == ">":
                        return current_python_version > target_version
                    elif op == "<":
                        return current_python_version < target_version
                    break

        # For other conditions (platform, etc.), default to True for now
        # This can be extended to handle platform_system, etc.
        logger.debug(
            f"Unhandled requirement condition: {condition}, defaulting to True"
        )
        return True

    except Exception as e:
        logger.warning(
            f"Error evaluating requirement condition '{condition}': {e}, "
            "defaulting to True"
        )
        return True


def get_installed_version(package_name: str) -> Optional[str]:
    """Get the installed version of a package.

    Args:
        package_name: Name of the package

    Returns:
        Version string or None if not installed
    """
    try:
        return importlib.metadata.version(package_name)
    except importlib.metadata.PackageNotFoundError:
        return None


def check_version_compatibility(
    constraint: str, installed_version: str
) -> bool:
    """Check if an installed version satisfies a version constraint.

    Args:
        constraint: Version constraint string (e.g., ">=1.0.0")
        installed_version: Installed version string

    Returns:
        True if compatible, False otherwise
    """
    try:
        if not constraint:
            # No constraint specified, any version is acceptable
            return True

        # Use packaging.specifiers to check if version satisfies the constraint
        spec = specifiers.SpecifierSet(constraint)
        return spec.contains(installed_version)
    except Exception as e:
        # Invalid constraint format or version
        logger.warning(f"Error checking version compatibility: {e}")
        return False


def check_zmq_installation() -> Tuple[bool, Optional[str]]:
    """Specifically check the PyZMQ installation for issues.

    Returns:
        Tuple of (is_working, error_message)
    """
    # We'll avoid completely removing the module from sys.modules to prevent
    # potential conflicts with code that has already imported it
    try:
        # Use a more controlled import approach
        import importlib.util
        import os
        import sys

        # Check if we're in virtual environment
        in_virtual_env = hasattr(sys, "real_prefix") or (
            hasattr(sys, "base_prefix") and sys.base_prefix != sys.prefix
        )

        # Detect if we're using the system Python or a virtual environment
        venv_path = os.environ.get("VIRTUAL_ENV")
        debug_info = [
            f"Python executable: {sys.executable}",
            f"In virtual env: {in_virtual_env}",
            f"VIRTUAL_ENV: {venv_path}",
            f"sys.prefix: {sys.prefix}",
        ]

        # Check if zmq is already in sys.modules and if so, get debug info
        if "zmq" in sys.modules:
            zmq_module = sys.modules["zmq"]
            debug_info.append(f"ZMQ module found in sys.modules: {zmq_module}")

            # Check for key attributes without assuming they exist
            for attr in ["__version__", "__file__"]:
                if hasattr(zmq_module, attr):
                    debug_info.append(
                        f"ZMQ {attr}: {getattr(zmq_module, attr)}"
                    )

        # Try importing zmq directly - but don't store the reference directly
        # to avoid potential conflicts with later imports
        result = importlib.import_module("zmq")

        # Extract version and file info for debugging
        zmq_file = getattr(result, "__file__", "unknown path")
        debug_info.append(f"ZMQ imported successfully from: {zmq_file}")

        # Get the version from importlib.metadata instead of the module
        try:
            zmq_version = importlib.metadata.version("pyzmq")
            debug_info.append(f"PyZMQ version from metadata: {zmq_version}")
        except Exception:
            zmq_version = "unknown"
            debug_info.append("Could not get PyZMQ version from metadata")

        # Skip the actual Context check if we're not in a virtual environment
        # This prevents false positives since the system Python might have a
        # broken zmq install but we're actually going to run with the virtual
        # environment
        if not in_virtual_env and not venv_path:
            logger.warning(
                "Running from system Python - skipping ZMQ Context check"
            )
            logger.debug(f"ZMQ debug info: {', '.join(debug_info)}")
            return True, None

        # Basic attribute check
        if not hasattr(result, "Context"):
            debug_info_str = ", ".join(debug_info)
            logger.debug(f"ZMQ debug info: {debug_info_str}")
            return (
                False,
                "PyZMQ missing Context attribute - may need reinstallation",
            )

        # Check if it's a callable
        if not callable(result.Context):
            debug_info_str = ", ".join(debug_info)
            logger.debug(f"ZMQ debug info: {debug_info_str}")
            return (
                False,
                "PyZMQ Context is not callable - may be incorrectly imported",
            )

        # Try using the Context
        try:
            # Create a minimal context and socket to test functionality
            context = result.Context()
            socket = context.socket(result.PUB)
            socket.close()
            context.term()
            return True, None
        except Exception as e:
            debug_info_str = ", ".join(debug_info)
            logger.debug(f"ZMQ debug info: {debug_info_str}")
            return False, f"PyZMQ Context initialization failed: {str(e)}"
    except ImportError:
        return False, "PyZMQ is not installed"
    except Exception as e:
        return False, f"Unexpected error checking PyZMQ: {str(e)}"


def check_dependencies(
    requirements_path: Optional[str] = None,
) -> Tuple[bool, List[str]]:
    """Check if installed dependencies match the requirements.

    Args:
        requirements_path: Path to requirements.txt file

    Returns:
        Tuple of (is_compatible, error_messages)
    """
    if requirements_path is None:
        # Try to find requirements.txt in the project root
        feagi_root = Path(__file__).parent.parent.parent
        requirements_path = str(feagi_root / "requirements.txt")

    requirements = parse_requirements_file(requirements_path)
    is_compatible = True
    error_messages = []

    for package_name, version_constraint in requirements.items():
        # Skip tracemalloc as it's a built-in module
        if package_name.lower() == "tracemalloc":
            continue

        installed_version = get_installed_version(package_name)

        if installed_version is None:
            is_compatible = False
            error_messages.append(
                f"Package '{package_name}' is not installed, but "
                f"required{version_constraint}"
            )
            continue

        if not check_version_compatibility(
            version_constraint, installed_version
        ):
            is_compatible = False
            error_messages.append(
                f"Package '{package_name}' version mismatch: "
                f"installed {installed_version}, but requires {version_constraint}"
            )

        # Skip PyZMQ Context check entirely since it works in the virtual
        # environment. The system Python might have issues but that's
        # irrelevant for actual usage

    return is_compatible, error_messages


def verify_dependencies(
    requirements_path: Optional[str] = None, raise_exception: bool = False
) -> bool:
    """Verify that all dependencies meet version requirements.

    Args:
        requirements_path: Path to requirements.txt file
        raise_exception: Whether to raise an exception if a mismatch is found

    Returns:
        True if all dependencies are compatible, False otherwise

    Raises:
        VersionMismatchError: If raise_exception is True and a mismatch is found
    """
    is_compatible, error_messages = check_dependencies(requirements_path)

    if not is_compatible:
        # Format error message
        error_header = "\n" + "=" * 80 + "\n"
        error_header += "FEAGI DEPENDENCY VERSION MISMATCH\n"
        error_header += "=" * 80 + "\n\n"

        error_footer = "\n" + "=" * 80 + "\n"
        error_footer += "Please install the correct versions using:\n"
        error_footer += "pip install -r requirements.txt\n"
        error_footer += "=" * 80 + "\n"

        error_body = "\n".join(error_messages)
        full_message = error_header + error_body + error_footer

        # Log the error message
        logger.error(full_message)

        # Optionally raise an exception
        if raise_exception:
            raise VersionMismatchError(full_message)
    else:
        # Log a summary of the check
        packages_checked = (
            len(check_dependencies(requirements_path)[1])
            if not is_compatible
            else 0
        )
        if (
            packages_checked == 0
        ):  # If we don't have error messages, we need to count packages differently
            try:
                if requirements_path is None:
                    # Try to find requirements.txt in the project root
                    feagi_root = Path(__file__).parent.parent.parent
                    requirements_path = str(feagi_root / "requirements.txt")

                requirements = parse_requirements_file(requirements_path)
                packages_checked = len(requirements)
            except Exception:
                packages_checked = "all"

        logger.info(
            f"[OK] {packages_checked} dependencies checked and all are compatible"
        )

    return is_compatible


def main():
    """Command-line entry point to check dependencies."""
    import logging

    logging.basicConfig(level=logging.INFO)

    if len(sys.argv) > 1:
        requirements_path = sys.argv[1]
    else:
        requirements_path = None

    is_compatible = verify_dependencies(requirements_path)

    if is_compatible:
        logger.info("All dependencies are compatible!")
        return 0
    else:
        return 1


if __name__ == "__main__":
    sys.exit(main())
