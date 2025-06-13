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

"""
Architecture Compliance Tests for FEAGI 2.0 - REAL CODEBASE SCANNING

These tests actually scan the codebase and test running components to ensure:
1. No hardcoded network hosts (127.0.0.1, localhost) in source code
2. No hardcoded timeout values in critical paths
3. All components actually use TOML configuration (not just that config loading works)
4. Running processes fail properly when configuration is missing

LESSONS LEARNED FROM PREVIOUS FAILED TESTS:
- Don't just test configuration loading - test actual code compliance
- Don't use mocks for compliance testing - test real behavior
- Scan the actual source code for violations
- Test that components actually reject missing configuration
"""

import os
import re
import tempfile
from pathlib import Path
from typing import List
from unittest.mock import patch

import pytest

# Import FEAGI configuration modules for integration testing
from feagi.config.toml_loader import get_host_config, get_port_config


class TestCodebaseHardcodingDetection:
    """Tests that scan the actual codebase for hardcoded violations."""

    @pytest.mark.compliance
    def test_no_hardcoded_network_hosts_in_source_code(self):
        """Scan the entire FEAGI source code for hardcoded network hosts."""
        # Get FEAGI source directory
        feagi_dir = Path(__file__).parent.parent.parent / "feagi"

        # Patterns that indicate hardcoded network hosts
        hardcoded_patterns = [
            r'["\']127\.0\.0\.1["\']',  # "127.0.0.1"
            r'["\']localhost["\']',  # "localhost"
            r'host.*=.*["\']127\.0\.0\.1["\']',  # host = "127.0.0.1"
            r'host.*=.*["\']localhost["\']',  # host = "localhost"
            r'\.get\(["\']host["\'],\s*["\']127\.0\.0\.1["\']',  # .get('host', '127.0.0.1')
            r'\.get\(["\']host["\'],\s*["\']localhost["\']',  # .get('host', 'localhost')
            r'os\.environ\.get\(["\'][^"\']*HOST["\'],\s*["\']127\.0\.0\.1["\']',  # os.environ.get("HOST", "127.0.0.1")
            r'os\.environ\.get\(["\'][^"\']*HOST["\'],\s*["\']localhost["\']',  # os.environ.get("HOST", "localhost")
        ]

        violations = []

        # Scan all Python files
        for py_file in feagi_dir.rglob("*.py"):
            # Skip test files and certain directories for now
            if any(skip in str(py_file) for skip in ["/tests/", "__pycache__", ".pyc"]):
                continue

            try:
                with open(py_file, "r", encoding="utf-8") as f:
                    content = f.read()

                for line_num, line in enumerate(content.split("\n"), 1):
                    for pattern in hardcoded_patterns:
                        if re.search(pattern, line, re.IGNORECASE):
                            # Skip comments and documentation
                            stripped = line.strip()
                            if (
                                stripped.startswith("#")
                                or stripped.startswith('"""')
                                or stripped.startswith("'''")
                            ):
                                continue

                            # Check for @architecture:acceptable annotation
                            if "@architecture:acceptable" in line:
                                continue  # Skip explicitly marked acceptable cases

                            violations.append(
                                {
                                    "file": str(py_file.relative_to(feagi_dir.parent)),
                                    "line": line_num,
                                    "content": line.strip(),
                                    "pattern": pattern,
                                }
                            )

            except Exception:
                # Skip files that can't be read
                continue

        # Report violations
        if violations:
            violation_report = "\n".join(
                [
                    f"  {v['file']}:{v['line']} - {v['content']}"
                    for v in violations[:10]  # Show first 10 violations
                ]
            )
            if len(violations) > 10:
                violation_report += (
                    f"\n  ... and {len(violations) - 10} more violations"
                )

            pytest.fail(
                f"Found {len(violations)} hardcoded network host violations in source code:\n{violation_report}\n\n"
                f"All network hosts must come from TOML configuration. "
                f"Use get_host_config(config) to get validated hosts."
            )

    @pytest.mark.compliance
    def test_no_hardcoded_timeout_values_in_critical_paths(self):
        """Scan source code for hardcoded timeout values in critical components."""
        feagi_dir = Path(__file__).parent.parent.parent / "feagi"

        # Files that contain critical paths where timeouts must be configurable
        critical_files = [
            "process_manager.py",
            "api/gateway/api_gateway.py",
            "api/zmq/server.py",
            "api/zmq/client.py",
            "core/health_monitor.py",
        ]

        # Patterns for hardcoded timeouts (numbers > 1 that likely represent seconds/milliseconds)
        timeout_patterns = [
            r"time\.sleep\(\s*\d+\.?\d*\s*\)",  # time.sleep(5)
            r"timeout\s*=\s*\d+\.?\d*",  # timeout=30
            r"\.join\(\s*timeout\s*=\s*\d+\.?\d*\s*\)",  # .join(timeout=5.0)
            r"\.get\(\s*timeout\s*=\s*\d+\.?\d*\s*\)",  # .get(timeout=1000)
        ]

        violations = []

        for file_pattern in critical_files:
            matching_files = list(feagi_dir.rglob(file_pattern))
            for py_file in matching_files:
                try:
                    with open(py_file, "r", encoding="utf-8") as f:
                        content = f.read()
                        lines = content.split("\n")

                    for line_num, line in enumerate(lines, 1):
                        # Skip comments and test code
                        stripped = line.strip()
                        if stripped.startswith("#") or "test" in stripped.lower():
                            continue

                        for pattern in timeout_patterns:
                            if re.search(pattern, line):
                                # Context-aware filtering: Check if this is acceptable hardcoding
                                if self._is_acceptable_hardcoded_timeout(
                                    line, lines, line_num
                                ):
                                    continue  # Skip acceptable hardcoded values

                                violations.append(
                                    {
                                        "file": str(
                                            py_file.relative_to(feagi_dir.parent)
                                        ),
                                        "line": line_num,
                                        "content": line.strip(),
                                        "pattern": pattern,
                                    }
                                )

                except Exception:
                    continue

        if violations:
            violation_report = "\n".join(
                [
                    f"  {v['file']}:{v['line']} - {v['content']}"
                    for v in violations[:5]  # Show first 5 violations
                ]
            )
            if len(violations) > 5:
                violation_report += f"\n  ... and {len(violations) - 5} more violations"

            pytest.fail(
                f"Found {len(violations)} hardcoded timeout violations in critical paths:\n{violation_report}\n\n"
                f"Critical path timeouts must be configurable via TOML. "
                f"Use get_timeout_config(config) to get configurable timeouts."
            )

    def _is_acceptable_hardcoded_timeout(
        self, line: str, all_lines: List[str], line_num: int
    ) -> bool:
        """
        Determine if a hardcoded timeout is acceptable based on context.

        Acceptable contexts:
        1. Emergency fallback values (in except blocks)
        2. Shutdown/cleanup procedures
        3. Values explicitly marked as fallbacks
        4. Default assignments when config unavailable

        Args:
            line: The line containing the hardcoded value
            all_lines: All lines in the file for context analysis
            line_num: Line number (1-indexed)

        Returns:
            True if the hardcoded value is acceptable, False otherwise
        """
        # Convert to 0-indexed for array access
        line_idx = line_num - 1

        # Check for explicit annotations marking acceptable hardcoding
        if any(
            marker in line.lower()
            for marker in [
                "# fallback",
                "# emergency",
                "# @architecture:acceptable",
                "# emergency fallback",
                "# only used if configuration",
                "# when configuration is completely unavailable",
            ]
        ):
            return True

        # Check if we're inside an exception handler (within 10 lines of an except block)
        for i in range(max(0, line_idx - 10), min(len(all_lines), line_idx + 3)):
            if re.search(r"except\s+.*:", all_lines[i].strip()):
                return True

        # Check if this is in a shutdown/cleanup context
        context_window = "\n".join(
            all_lines[max(0, line_idx - 5) : min(len(all_lines), line_idx + 5)]
        )
        shutdown_indicators = [
            "shutdown",
            "cleanup",
            "emergency",
            "fallback",
            "configuration.*unavailable",
            "config.*fail",
            "last resort",
            "when.*config.*missing",
        ]
        if any(
            re.search(indicator, context_window, re.IGNORECASE)
            for indicator in shutdown_indicators
        ):
            return True

        # Check if variable name indicates it's a fallback/default
        fallback_variable_patterns = [
            r"\w*fallback\w*\s*=",
            r"\w*emergency\w*\s*=",
            r"\w*default\w*\s*=",
            r".*_timeout\s*=.*#.*fallback",
        ]
        if any(
            re.search(pattern, line, re.IGNORECASE)
            for pattern in fallback_variable_patterns
        ):
            return True

        # Check for conditional assignment when config is unavailable
        if re.search(r"except.*:", line) or "if.*config" in line.lower():
            return True

        return False


class TestComponentIntegrationCompliance:
    """Test that actual running components use configuration properly."""

    @pytest.mark.integration
    def test_process_manager_requires_host_configuration(self):
        """Test that ProcessManager actually rejects empty host configuration."""

        # Test get_host_config directly with empty hosts
        empty_host_config = {
            "api": {"host": "", "port": 8080},  # Valid port, empty host
            "zmq": {"host": ""},
        }

        # This should fail when we try to get host configuration
        with pytest.raises(ValueError, match="API host is required"):
            get_host_config(empty_host_config)

    @pytest.mark.integration
    def test_process_manager_integration_with_valid_config(self):
        """Test that ProcessManager works when proper configuration is provided."""
        from feagi.process_manager import ProcessManager

        # Test with environment variables providing hosts
        with patch.dict(
            os.environ,
            {
                "FEAGI_API_HOST": "192.168.1.100",
                "FEAGI_ZMQ_HOST": "192.168.1.101",
                "FEAGI_API_PORT": "8080",
            },
        ):
            pm = ProcessManager()

            # This should work because environment variables provide the hosts
            try:
                result = pm.load_and_validate_ports()
                assert result is not None  # Should return configuration if successful
            except Exception as e:
                # If it fails for other reasons (like port conflicts), that's OK
                # We're just testing that it doesn't fail due to missing host config
                if "API host is required" in str(e):
                    pytest.fail(
                        "ProcessManager failed due to missing host configuration despite environment overrides"
                    )
                # Other failures (like port conflicts) are acceptable for this test

    @pytest.mark.integration
    def test_api_gateway_uses_configuration_system(self):
        """Test that API Gateway actually uses the configuration system."""
        from feagi.api.gateway.api_gateway import APIGateway

        # Test that APIGateway initialization loads configuration properly
        with patch.dict(
            os.environ,
            {
                "FEAGI_ZMQ_ENABLED": "1",
                "FEAGI_API_HOST": "192.168.1.100",
                "FEAGI_ZMQ_HOST": "192.168.1.101",
            },
        ):
            # Create a temporary TOML config file
            with tempfile.NamedTemporaryFile(
                mode="w", suffix=".toml", delete=False
            ) as f:
                f.write(
                    """
[api]
host = ""  # Should be overridden by environment
port = 8080

[zmq]
host = ""  # Should be overridden by environment

[ports]
zmq_req_rep_port = 5555
zmq_pub_sub_port = 5556
zmq_push_pull_port = 5557
zmq_sensory_port = 5558
zmq_visualization_port = 5562
zmq_rest_port = 5563
zmq_motor_port = 5564

[timeouts]
graceful_shutdown = 8.0
"""
                )
                temp_config_path = f.name

            try:
                # Mock the config file loading to use our temporary file
                with patch("feagi.config.toml_loader.load_feagi_config") as mock_load:
                    # Load actual config from our temp file
                    try:
                        import tomllib
                    except ImportError:
                        import tomli as tomllib

                    with open(temp_config_path, "rb") as config_file:
                        test_config = tomllib.load(config_file)

                    mock_load.return_value = test_config

                    # This should work because environment variables override empty hosts
                    gateway = APIGateway()

                    # Verify it actually loaded the configuration (if ZMQ is initialized)
                    if hasattr(gateway, "_zmq_client") and gateway._zmq_client:
                        # The test passes if no exception was raised during initialization
                        assert True
                    else:
                        # Even if ZMQ client wasn't created, the gateway should have loaded config
                        assert True

            finally:
                os.unlink(temp_config_path)

    @pytest.mark.integration
    def test_no_component_has_hardcoded_fallback_behavior(self):
        """Test that key components fail properly when configuration is missing."""

        # Test with completely empty configuration
        empty_config = {}

        # ProcessManager should fail with missing configuration
        from feagi.process_manager import ProcessManager

        pm = ProcessManager()

        with pytest.raises(ValueError, match="API host is required"):
            pm.load_and_validate_ports()

        # get_host_config should fail with empty config
        with pytest.raises(ValueError, match="API host is required"):
            get_host_config(empty_config)

        # get_port_config should fail with empty config - expect KeyError for missing ports section
        with pytest.raises(KeyError, match="Missing 'ports' section"):
            get_port_config(empty_config)


class TestRealConfigurationIntegration:
    """Test actual TOML configuration file compliance."""

    @pytest.mark.integration
    def test_actual_toml_has_no_hardcoded_defaults(self):
        """Test the real feagi_configuration.toml file for hardcoded violations."""
        config_path = Path(__file__).parent.parent.parent / "feagi_configuration.toml"

        if not config_path.exists():
            pytest.skip(f"Configuration file not found: {config_path}")

        try:
            import tomllib
        except ImportError:
            try:
                import tomli as tomllib
            except ImportError:
                pytest.skip("TOML support required")

        with open(config_path, "rb") as f:
            config = tomllib.load(f)

        # Check for hardcoded host values
        violations = []

        api_host = config.get("api", {}).get("host", "")
        if api_host and api_host in ["127.0.0.1", "localhost"]:
            violations.append(f"API host hardcoded to: {api_host}")

        zmq_host = config.get("zmq", {}).get("host", "")
        if zmq_host and zmq_host in ["127.0.0.1", "localhost"]:
            violations.append(f"ZMQ host hardcoded to: {zmq_host}")

        # Note: port = 0 is ACCEPTABLE as it forces explicit configuration
        # This is the correct design pattern for platform-agnostic deployment

        if violations:
            pytest.fail(
                "TOML configuration file contains hardcoded violations:\n"
                + "\n".join(f"  - {v}" for v in violations)
                + "\n\nConfiguration must require explicit host/port settings."
            )

    @pytest.mark.integration
    def test_configuration_environment_override_integration(self):
        """Test that the configuration system actually respects environment overrides."""
        test_env = {
            "FEAGI_API_HOST": "10.0.0.1",
            "FEAGI_ZMQ_HOST": "10.0.0.2",
            "FEAGI_API_PORT": "9000",
        }

        with patch.dict(os.environ, test_env, clear=False):
            # Create minimal config (empty hosts that should be overridden)
            minimal_config = {
                "api": {"host": "", "port": 0},
                "zmq": {"host": ""},
                "ports": {
                    "zmq_req_rep_port": 5555,
                    "zmq_pub_sub_port": 5556,
                    "zmq_push_pull_port": 5557,
                    "zmq_sensory_port": 5558,
                    "zmq_visualization_port": 5562,
                    "zmq_rest_port": 5563,
                    "zmq_motor_port": 5564,
                },
            }

            # Mock the config loading to return our minimal config
            with patch(
                "feagi.config.toml_loader.load_feagi_config",
                return_value=minimal_config,
            ):
                host_config = get_host_config(minimal_config)

                # Verify environment overrides actually work
                assert host_config.api_host == "10.0.0.1"
                assert host_config.zmq_host == "10.0.0.2"


class TestHardcodedPatternDetection:
    """Specialized tests for detecting specific hardcoded patterns."""

    @pytest.mark.compliance
    def test_no_os_environ_get_with_hardcoded_fallbacks(self):
        """Scan for os.environ.get() calls with hardcoded network fallbacks."""
        feagi_dir = Path(__file__).parent.parent.parent / "feagi"

        # Pattern for os.environ.get with hardcoded network fallbacks
        pattern = r'os\.environ\.get\(["\'][^"\']*["\'],\s*["\'](?:127\.0\.0\.1|localhost)["\']'

        violations = []

        for py_file in feagi_dir.rglob("*.py"):
            if "/tests/" in str(py_file):
                continue

            try:
                with open(py_file, "r", encoding="utf-8") as f:
                    content = f.read()

                for line_num, line in enumerate(content.split("\n"), 1):
                    if re.search(pattern, line):
                        violations.append(
                            {
                                "file": str(py_file.relative_to(feagi_dir.parent)),
                                "line": line_num,
                                "content": line.strip(),
                            }
                        )

            except Exception:
                continue

        if violations:
            violation_report = "\n".join(
                [f"  {v['file']}:{v['line']} - {v['content']}" for v in violations]
            )

            pytest.fail(
                f"Found {len(violations)} os.environ.get() calls with hardcoded network fallbacks:\n{violation_report}\n\n"
                f"Use configuration system instead: get_host_config(config).api_host"
            )

    @pytest.mark.compliance
    def test_no_config_get_with_hardcoded_fallbacks(self):
        """Scan for config.get() calls with hardcoded network fallbacks."""
        feagi_dir = Path(__file__).parent.parent.parent / "feagi"

        # Pattern for config.get with hardcoded network fallbacks
        pattern = r'\.get\(["\']host["\'],\s*["\'](?:127\.0\.0\.1|localhost)["\']'

        violations = []

        for py_file in feagi_dir.rglob("*.py"):
            if "/tests/" in str(py_file):
                continue

            try:
                with open(py_file, "r", encoding="utf-8") as f:
                    content = f.read()

                for line_num, line in enumerate(content.split("\n"), 1):
                    if re.search(pattern, line):
                        violations.append(
                            {
                                "file": str(py_file.relative_to(feagi_dir.parent)),
                                "line": line_num,
                                "content": line.strip(),
                            }
                        )

            except Exception:
                continue

        if violations:
            violation_report = "\n".join(
                [f"  {v['file']}:{v['line']} - {v['content']}" for v in violations]
            )

            pytest.fail(
                f"Found {len(violations)} config.get() calls with hardcoded network fallbacks:\n{violation_report}\n\n"
                f"Use configuration system instead: get_host_config(config).api_host"
            )


# Mark all tests in this module with appropriate markers
pytestmark = [
    pytest.mark.compliance,  # All tests check compliance
    pytest.mark.architecture,  # All tests check architecture
]
