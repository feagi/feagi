"""
Tests for FEAGI config generation and Brain Visualizer config reading.

Verifies that:
1. Default config uses 127.0.0.1 (not 0.0.0.0) for security
2. BV reads config correctly without hardcoded conversions
3. Config is the single source of truth
"""

from pathlib import Path

import pytest
import toml

from feagi.cli.bv import _extract_network_settings, _load_feagi_config
from feagi.config import generate_default_config


class TestConfigGeneration:
    """Test default config generation."""

    def test_default_config_uses_localhost(self, tmp_path):
        """Verify default config uses 127.0.0.1 for API and websocket."""
        config_path = generate_default_config(tmp_path / "test_config.toml", force=True)
        
        config = toml.load(config_path)
        
        # API should use 127.0.0.1
        assert config["api"]["host"] == "127.0.0.1", (
            "Default API host should be 127.0.0.1 for security (no firewall prompts)"
        )
        
        # WebSocket should use 127.0.0.1
        assert config["websocket"]["host"] == "127.0.0.1", (
            "Default WebSocket host should be 127.0.0.1 for security (no firewall prompts)"
        )

    def test_default_config_has_correct_structure(self, tmp_path):
        """Verify default config has all required sections."""
        config_path = generate_default_config(tmp_path / "test_config.toml", force=True)
        config = toml.load(config_path)
        
        # Required sections
        assert "api" in config, "Config must have [api] section"
        assert "websocket" in config, "Config must have [websocket] section"
        
        # Required fields
        assert "host" in config["api"], "API section must have host"
        assert "port" in config["api"], "API section must have port"
        assert "host" in config["websocket"], "WebSocket section must have host"
        assert "visualization_port" in config["websocket"], (
            "WebSocket section must have visualization_port"
        )

    def test_default_config_has_timeouts(self, tmp_path):
        """Verify default config includes startup timeout settings."""
        config_path = generate_default_config(tmp_path / "test_config.toml", force=True)
        config = toml.load(config_path)

        assert "timeouts" in config, "Config must have [timeouts] section"
        assert "service_startup" in config["timeouts"], (
            "Config must define timeouts.service_startup"
        )
        assert isinstance(config["timeouts"]["service_startup"], (int, float))


class TestBVConfigReading:
    """Test Brain Visualizer config reading."""

    def test_bv_reads_config_directly(self, tmp_path):
        """Verify BV reads config values directly without hardcoded conversions."""
        # Create a test config with 127.0.0.1
        config_path = tmp_path / "test_config.toml"
        config_content = """[api]
host = "127.0.0.1"
port = 8000

[websocket]
host = "127.0.0.1"
visualization_port = 9050
"""
        config_path.write_text(config_content)
        
        config = _load_feagi_config(config_path)
        api_host, api_port, ws_host, ws_port = _extract_network_settings(config)
        
        # Should read exactly what's in config
        assert api_host == "127.0.0.1", "Should read 127.0.0.1 directly from config"
        assert ws_host == "127.0.0.1", "Should read 127.0.0.1 directly from config"
        assert api_port == 8000
        assert ws_port == 9050

    def test_bv_reads_custom_host(self, tmp_path):
        """Verify BV can read custom host addresses from config."""
        # Create a test config with custom host (for network deployments)
        config_path = tmp_path / "test_config.toml"
        config_content = """[api]
host = "192.168.1.100"
port = 8000

[websocket]
host = "192.168.1.100"
visualization_port = 9050
"""
        config_path.write_text(config_content)
        
        config = _load_feagi_config(config_path)
        api_host, api_port, ws_host, ws_port = _extract_network_settings(config)
        
        # Should read custom host directly
        assert api_host == "192.168.1.100", "Should read custom host from config"
        assert ws_host == "192.168.1.100", "Should read custom host from config"

    def test_bv_no_hardcoded_conversions(self, tmp_path):
        """Verify BV does NOT convert 0.0.0.0 to 127.0.0.1 (config is source of truth)."""
        # Create a test config with 0.0.0.0 (old config format)
        config_path = tmp_path / "test_config.toml"
        config_content = """[api]
host = "0.0.0.0"
port = 8000

[websocket]
host = "0.0.0.0"
visualization_port = 9050
"""
        config_path.write_text(config_content)
        
        config = _load_feagi_config(config_path)
        api_host, api_port, ws_host, ws_port = _extract_network_settings(config)
        
        # Should read 0.0.0.0 as-is (no conversion)
        # This tests that config is source of truth, even if value is suboptimal
        assert api_host == "0.0.0.0", (
            "Should read 0.0.0.0 as-is from config (no hardcoded conversion)"
        )
        assert ws_host == "0.0.0.0", (
            "Should read 0.0.0.0 as-is from config (no hardcoded conversion)"
        )

    def test_bv_requires_api_and_websocket_sections(self, tmp_path):
        """Verify BV raises error if required sections are missing."""
        # Config missing websocket section
        config_path = tmp_path / "test_config.toml"
        config_content = """[api]
host = "127.0.0.1"
port = 8000
"""
        config_path.write_text(config_content)
        
        config = _load_feagi_config(config_path)
        
        with pytest.raises(Exception):  # Should raise BrainVisualizerLaunchError
            _extract_network_settings(config)

    def test_bv_requires_host_fields(self, tmp_path):
        """Verify BV raises error if host fields are missing."""
        config_path = tmp_path / "test_config.toml"
        config_content = """[api]
port = 8000

[websocket]
visualization_port = 9050
"""
        config_path.write_text(config_content)
        
        config = _load_feagi_config(config_path)
        
        with pytest.raises(Exception):  # Should raise BrainVisualizerLaunchError
            _extract_network_settings(config)


class TestConfigConsistency:
    """Test consistency between default config and example config."""

    def test_example_config_api_host(self):
        """Verify example config uses 127.0.0.1 for API."""
        example_config_path = Path(__file__).parent.parent.parent / "examples" / "feagi_configuration.toml"
        
        if not example_config_path.exists():
            pytest.skip("Example config file not found")
        
        config = toml.load(example_config_path)
        
        # API should use 127.0.0.1 (for BV compatibility)
        assert config["api"]["host"] == "127.0.0.1", (
            "Example config API host should be 127.0.0.1 for security"
        )

    def test_example_config_websocket_host(self):
        """Verify example config uses 127.0.0.1 for WebSocket."""
        example_config_path = Path(__file__).parent.parent.parent / "examples" / "feagi_configuration.toml"
        
        if not example_config_path.exists():
            pytest.skip("Example config file not found")
        
        config = toml.load(example_config_path)
        
        # WebSocket should use 127.0.0.1 (for BV compatibility)
        assert config["websocket"]["host"] == "127.0.0.1", (
            "Example config WebSocket host should be 127.0.0.1 for security"
        )
