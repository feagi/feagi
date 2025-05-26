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
Unit tests for the FEAGI configuration loader.
"""

import os
import tempfile
from pathlib import Path
import yaml
import pytest

from feagi.config.loader import ConfigLoader, DEFAULT_CONFIG


# Use a fixture to create a fresh ConfigLoader for each test
@pytest.fixture
def loader():
    """Create a fresh ConfigLoader instance for each test."""
    return ConfigLoader()


@pytest.fixture
def default_loader(loader):
    """Create a ConfigLoader with default config loaded."""
    loader.load_default()
    return loader


class TestConfigLoader:
    """Tests for the ConfigLoader class."""
    
    def test_init(self, loader):
        """Test initialization of ConfigLoader."""
        assert loader._config == {}
    
    def test_load_default(self, loader):
        """Test loading default configuration."""
        config = loader.load_default()
        
        # Verify contents match default
        assert config == DEFAULT_CONFIG
        assert config is not DEFAULT_CONFIG  # Should be a copy, not the same reference
        
        # Verify internal state
        assert loader._config == DEFAULT_CONFIG
    
    def test_load_file(self, default_loader):
        """Test loading configuration from file."""
        loader = default_loader
        
        # Create a temporary config file
        with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix=".yaml") as temp_file:
            test_config = {
                "npu": {
                    "backend": "cuda",
                    "threads": 8,
                }
            }
            yaml.dump(test_config, temp_file)
        
        try:
            # Load the config
            config = loader.load_file(temp_file.name)
            
            # Verify merged config
            assert config["npu"]["backend"] == "cuda"  # From file
            assert config["npu"]["threads"] == 8  # From file
            assert config["npu"]["burst_size"] == 100  # From default
            assert "logging" in config  # From default
        finally:
            # Clean up
            os.unlink(temp_file.name)
    
    def test_get(self, default_loader):
        """Test getting configuration values."""
        loader = default_loader
        
        # Test getting values with dot notation
        assert loader.get("npu.backend") == "auto"
        assert loader.get("npu.threads") == 4
        assert loader.get("logging.level") == "INFO"
        
        # Test getting nested dictionaries
        npu_config = loader.get("npu")
        assert isinstance(npu_config, dict)
        assert "backend" in npu_config
        assert "threads" in npu_config
        
        # Test default values
        assert loader.get("nonexistent.key") is None
        assert loader.get("nonexistent.key", "default") == "default"
    
    def test_set(self, default_loader):
        """Test setting configuration values."""
        loader = default_loader
        
        # Set a value at an existing path
        loader.set("npu.backend", "cuda")
        assert loader.get("npu.backend") == "cuda"
        
        # Set a value at a new path
        loader.set("new.path.key", "value")
        assert loader.get("new.path.key") == "value"
        
        # Set a value that replaces a non-dict with a dict
        loader.set("npu.threads", 8)  # Simple value
        assert loader.get("npu.threads") == 8
        loader.set("npu.threads.custom", True)  # Now threads becomes a dict
        assert isinstance(loader.get("npu.threads"), dict)
        assert loader.get("npu.threads.custom") is True
    
    def test_deep_update(self):
        """Test deep update of configuration dictionaries."""
        # Create dictionaries for testing
        target = {
            "a": 1,
            "b": {
                "c": 2,
                "d": {
                    "e": 3
                }
            }
        }
        
        source = {
            "a": 10,
            "b": {
                "c": 20,
                "f": 30
            },
            "g": 40
        }
        
        # Perform deep update
        ConfigLoader._deep_update(target, source)
        
        # Verify result
        assert target["a"] == 10  # Overwritten
        assert target["b"]["c"] == 20  # Overwritten
        assert target["b"]["d"]["e"] == 3  # Unchanged
        assert target["b"]["f"] == 30  # Added
        assert target["g"] == 40  # Added 