# -*- coding: utf-8 -*-
#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
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
# ==============================================================================

"""
Port Availability Checker for FEAGI

This module provides utilities to check if required ports are available
and fails fast with clear error messages if there are conflicts.
"""

import logging
import socket
from pathlib import Path
from typing import Dict, List, Optional

logger = logging.getLogger(__name__)


class PortConflictError(Exception):
    """Exception raised when port conflicts are detected."""

    pass


def check_port_availability(host: str, port: int) -> bool:
    """
    Check if a specific port is available on the given host.

    Args:
        host: Host address to check
        port: Port number to check

    Returns:
        True if port is available, False if in use
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(1)  # 1 second timeout
            result = sock.connect_ex((host, port))
            return result != 0  # Port is available if connection fails
    except Exception as e:
        logger.warning(f"Error checking port {port}: {e}")
        return False


def get_process_using_port(port: int) -> Optional[str]:
    """
    Get information about the process using a specific port.

    Args:
        port: Port number to check

    Returns:
        String describing the process, or None if not found
    """
    try:
        import psutil

        for conn in psutil.net_connections():
            if conn.laddr.port == port:
                try:
                    process = psutil.Process(conn.pid)
                    return f"PID {conn.pid}: {process.name()}"
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    return f"PID {conn.pid}: <unknown>"
        return None
    except ImportError:
        # psutil not available, fall back to basic check
        return "unknown process (install psutil for detailed info)"


def check_all_ports_available(host: str, port_config: Dict[str, int]) -> None:
    """
    Check if all required ports are available and fail fast if any conflicts exist.

    Args:
        host: Host address to bind to
        port_config: Dictionary mapping port names to port numbers

    Raises:
        PortConflictError: If any ports are in use with detailed conflict information
    """
    logger.info("Checking port availability...")

    conflicts = []

    for port_name, port_number in port_config.items():
        if not check_port_availability(host, port_number):
            process_info = get_process_using_port(port_number)
            conflicts.append(
                {"name": port_name, "port": port_number, "process": process_info}
            )
            logger.error(
                f"Port conflict detected: {port_name} (port {port_number}) is in use"
            )

    if conflicts:
        error_message = _format_port_conflict_error(conflicts)
        raise PortConflictError(error_message)

    logger.info("All required ports are available")


def _format_port_conflict_error(conflicts: List[Dict]) -> str:
    """
    Format a detailed error message for port conflicts.

    Args:
        conflicts: List of conflict information dictionaries

    Returns:
        Formatted error message with resolution instructions
    """
    error_lines = [
        "[ERR] PORT CONFLICT DETECTED [ERR]",
        "",
        "The following FEAGI ports are already in use:",
        "",
    ]

    for conflict in conflicts:
        port_name = conflict["name"]
        port_number = conflict["port"]
        process_info = conflict["process"] or "unknown process"

        error_lines.append(
            f"  • {port_name}: port {port_number} (used by {process_info})"
        )

    error_lines.extend(
        [
            "",
            "[CONFIG] RESOLUTION:",
            "",
            "1. Stop the processes using these ports, OR",
            "2. Edit the port configuration in feagi_configuration.ini",
            "",
            "To edit ports:",
            "  - Open feagi_configuration.ini in your text editor",
            "  - Go to the [Ports] section",
            "  - Change the conflicting port numbers to available ports",
            "  - Save the file and restart FEAGI",
            "",
            "Example port ranges you can try:",
            "  - 5600-5610 (alternative FEAGI range)",
            "  - 6000-6010 (high range)",
            "  - 7000-7010 (alternative high range)",
            "",
            "[WARN]  Make sure all port numbers are unique in the configuration file!",
            "",
        ]
    )

    return "\n".join(error_lines)


def validate_port_range(port_config: Dict[str, int]) -> None:
    """
    Validate that all ports are in acceptable ranges.

    Args:
        port_config: Dictionary mapping port names to port numbers

    Raises:
        ValueError: If any ports are outside acceptable ranges
    """
    invalid_ports = []

    for port_name, port_number in port_config.items():
        if not isinstance(port_number, int):
            invalid_ports.append(f"{port_name}: {port_number} (not an integer)")
        elif port_number < 1024:
            invalid_ports.append(
                f"{port_name}: {port_number} (below 1024 - privileged range)"
            )
        elif port_number > 65535:
            invalid_ports.append(f"{port_name}: {port_number} (above 65535 - invalid)")

    if invalid_ports:
        error_message = (
            "Invalid port configuration:\n"
            + "\n".join(f"  • {error}" for error in invalid_ports)
            + "\n\nPlease update feagi_configuration.ini with valid port numbers (1024-65535)."
        )
        raise ValueError(error_message)


def check_port_duplicates(port_config: Dict[str, int]) -> None:
    """
    Check for duplicate port assignments.

    Args:
        port_config: Dictionary mapping port names to port numbers

    Raises:
        ValueError: If duplicate ports are found
    """
    ports = list(port_config.values())
    duplicates = []

    for port in set(ports):
        if ports.count(port) > 1:
            port_names = [name for name, p in port_config.items() if p == port]
            duplicates.append(f"Port {port}: {', '.join(port_names)}")

    if duplicates:
        error_message = (
            "Duplicate port assignments found:\n"
            + "\n".join(f"  • {dup}" for dup in duplicates)
            + "\n\nPlease ensure all ports in feagi_configuration.ini are unique."
        )
        raise ValueError(error_message)


def get_config_file_location() -> str:
    """
    Get the expected location of the configuration file for error messages.

    Returns:
        Path to the configuration file
    """
    # Check common locations
    search_paths = [
        "./feagi_configuration.ini",
        Path(__file__).parent.parent / "feagi_configuration.ini",
    ]

    for path in search_paths:
        if Path(path).exists():
            return str(Path(path).resolve())

    # Return the most likely location if not found
    return str(Path("./feagi_configuration.ini").resolve())


def perform_comprehensive_port_check(host: str, port_config: Dict[str, int]) -> None:
    """
    Perform comprehensive port validation including duplicates, ranges, and availability.

    Args:
        host: Host address to bind to
        port_config: Dictionary mapping port names to port numbers

    Raises:
        ValueError: If configuration is invalid
        PortConflictError: If ports are in use
    """
    logger.info("Performing comprehensive port validation...")

    # Step 1: Validate port ranges
    validate_port_range(port_config)

    # Step 2: Check for duplicates
    check_port_duplicates(port_config)

    # Step 3: Check availability
    check_all_ports_available(host, port_config)

    logger.info("[OK] All port checks passed")


if __name__ == "__main__":
    # Test the port checker
    test_ports = {
        "test_port_1": 5555,
        "test_port_2": 5556,
        "test_port_3": 22,  # SSH port - likely in use
    }

    try:
        # Use configuration system instead of hardcoded IP
        from feagi.config.toml_loader import get_host_config, load_feagi_config

        config = load_feagi_config()
        host_config = get_host_config(config)

        perform_comprehensive_port_check(host_config.api_host, test_ports)
        print("All ports available!")
    except (PortConflictError, ValueError) as e:
        print(f"Port check failed:\n{e}")
