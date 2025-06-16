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
Unit tests for the FEAGI configuration validation.
"""


from feagi.config.validation import validate_config


class TestConfigValidation:
    """Tests for configuration validation functions."""

    def test_validate_valid_config(self):
        """Test validation with a valid configuration."""
        config = {
            "npu": {
                "backend": "cpu",
                "threads": 4,
                "burst_size": 100,
            },
            "logging": {
                "level": "INFO",
            },
        }

        assert validate_config(config) is True

    def test_validate_invalid_backend(self):
        """Test validation with an invalid backend."""
        config = {
            "npu": {
                "backend": "invalid_backend",
            },
        }

        assert validate_config(config) is False

    def test_validate_empty_config(self):
        """Test validation with an empty configuration."""
        config = {}

        # Empty config should be valid (no required fields)
        assert validate_config(config) is True

    def test_validate_with_exception(self, monkeypatch):
        """Test validation when an exception occurs."""

        # Create a dictionary-like object that raises an exception
        class ExceptionDict(dict):
            def __contains__(self, key):
                raise Exception("Test exception")

        config = ExceptionDict()
        config["npu"] = {}  # This won't raise yet

        # Mock the logger
        mock_error = []

        def mock_log_error(message):
            mock_error.append(message)

        monkeypatch.setattr("feagi.config.validation.logger.error", mock_log_error)

        # Should return False when an exception occurs during validation
        assert validate_config(config) is False

        # Verify the error was logged
        assert len(mock_error) == 1
        assert "Configuration validation failed:" in mock_error[0]
