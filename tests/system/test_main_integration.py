#!/usr/bin/env python3
"""
Integration tests for main.py to catch critical API contract violations.

These tests ensure that the main entry point can launch successfully
and that all expected API contracts are maintained.
"""

import importlib
import subprocess
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

# Add the feagi directory to the path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "feagi"))


class TestMainIntegration:
    """Test main.py integration and API contracts."""

    def test_main_help_launches_successfully(self):
        """Test that main.py --help launches without errors."""
        try:
            result = subprocess.run(
                [sys.executable, "feagi/main.py", "--help"],
                capture_output=True,
                text=True,
                timeout=30,
                cwd=Path(__file__).parent.parent.parent
            )
            
            # Should exit with code 0 (help successful)
            assert result.returncode == 0, f"main.py --help failed with: {result.stderr}"
            
            # Should contain expected help text
            assert "FEAGI - Framework for Evolutionary Artificial General Intelligence" in result.stdout
            assert "usage:" in result.stdout.lower()
            
        except subprocess.TimeoutExpired:
            pytest.fail("main.py --help timed out - likely hanging on missing method")
        except Exception as e:
            pytest.fail(f"main.py --help failed unexpectedly: {e}")

    def test_state_manager_has_required_methods(self):
        """Test that FeagiStateManager has all methods required by main.py."""
        from feagi.core.state_manager import FeagiStateManager
        
        # Critical methods that main.py expects
        required_methods = [
            'set_debug_config',
            'cleanup', 
            'log_startup_summary',
            'instance',
            'set_exit_condition',
            'get_genome_state',
            'get_brain_readiness',
            'get_burst_engine_state',
            'get_fq_sampler_state'
        ]
        
        state_manager = FeagiStateManager.instance()
        
        for method_name in required_methods:
            assert hasattr(state_manager, method_name), \
                f"FeagiStateManager missing required method: {method_name}"
            
            method = getattr(state_manager, method_name)
            assert callable(method), \
                f"FeagiStateManager.{method_name} is not callable"

    def test_set_debug_config_accepts_config_dict(self):
        """Test that set_debug_config can handle various config formats."""
        from feagi.core.state_manager import FeagiStateManager
        
        state_manager = FeagiStateManager.instance()
        
        # Test with empty config
        state_manager.set_debug_config({})
        
        # Test with debug section
        config_with_debug = {
            "debug": {
                "log_level": "DEBUG",
                "verbose": True
            }
        }
        state_manager.set_debug_config(config_with_debug)
        
        # Test with no debug section
        config_without_debug = {
            "api": {"host": "localhost"},
            "zmq": {"ports": {}}
        }
        state_manager.set_debug_config(config_without_debug)

    def test_cleanup_handles_various_states(self):
        """Test that cleanup method handles various state manager states."""
        from feagi.core.state_manager import FeagiStateManager
        from feagi.core.state_storage import MemoryStorage
        
        # Test cleanup with fresh instance
        storage = MemoryStorage()
        state_manager = FeagiStateManager.instance(storage)
        
        # Should not raise exceptions
        state_manager.cleanup()
        
        # Should be able to call multiple times
        state_manager.cleanup()

    def test_log_startup_summary_handles_missing_data(self):
        """Test that log_startup_summary handles missing or incomplete data."""
        from feagi.core.state_manager import FeagiStateManager
        from feagi.core.state_storage import MemoryStorage
        
        storage = MemoryStorage()
        state_manager = FeagiStateManager.instance(storage)
        
        # Should not raise exceptions even with minimal state
        state_manager.log_startup_summary()

    @patch('feagi.main.FeagiStateManager')
    def test_main_calls_expected_state_manager_methods(self, mock_state_manager_class):
        """Test that main.py calls the expected state manager methods in order."""
        # Setup mock
        mock_instance = MagicMock()
        mock_state_manager_class.instance.return_value = mock_instance
        
        # Mock other dependencies to prevent actual startup
        with patch('feagi.main.get_process_manager') as mock_pm, \
             patch('feagi.main.check_dependencies') as mock_deps, \
             patch('feagi.main.signal.signal'):
            
            mock_deps.return_value = True
            mock_pm.return_value.start.return_value = False  # Fail startup to exit early
            
            # Import and try to run main (will exit early due to mocked failure)
            try:
                from feagi.main import main
                with patch('sys.argv', ['main.py']):
                    main()
            except SystemExit:
                pass  # Expected due to mocked startup failure
            
            # Verify critical methods were called
            mock_instance.set_debug_config.assert_called_once()
            # Note: cleanup and log_startup_summary may not be called due to early exit

    def test_import_main_module_succeeds(self):
        """Test that main.py can be imported without errors."""
        try:
            import feagi.main
            # Should have main function
            assert hasattr(feagi.main, 'main')
            assert callable(feagi.main.main)
        except ImportError as e:
            pytest.fail(f"Failed to import feagi.main: {e}")
        except Exception as e:
            pytest.fail(f"Unexpected error importing feagi.main: {e}")

    def test_main_argument_parsing(self):
        """Test that main.py argument parsing works correctly."""
        try:
            # Test that we can parse arguments without launching
            result = subprocess.run(
                [sys.executable, "feagi/main.py", "--help"],
                capture_output=True,
                text=True,
                timeout=10,
                cwd=Path(__file__).parent.parent.parent
            )
            
            assert result.returncode == 0
            assert "--api-host" in result.stdout
            assert "--zmq-host" in result.stdout
            assert "--config" in result.stdout
            
        except subprocess.TimeoutExpired:
            pytest.fail("Argument parsing timed out")

    def test_critical_imports_available(self):
        """Test that all critical imports used by main.py are available."""
        critical_imports = [
            'feagi.core.state_manager',
            'feagi.bdu.connectome_manager', 
            'feagi.process_manager',
            'feagi.api.rest.dependencies',
            'feagi.utils.test_mode'
        ]
        
        for import_path in critical_imports:
            try:
                importlib.import_module(import_path)
            except ImportError as e:
                pytest.fail(f"Critical import failed: {import_path} - {e}")


class TestStateManagerAPIContract:
    """Test the API contract that other modules expect from FeagiStateManager."""
    
    def test_singleton_pattern_works(self):
        """Test that singleton pattern works correctly."""
        from feagi.core.state_manager import FeagiStateManager
        
        instance1 = FeagiStateManager.instance()
        instance2 = FeagiStateManager.instance()
        
        assert instance1 is instance2, "Singleton pattern broken"

    def test_all_getter_methods_return_valid_types(self):
        """Test that all getter methods return expected types."""
        from feagi.core.state_manager import FeagiStateManager
        
        state_manager = FeagiStateManager.instance()
        
        # Test return types
        assert isinstance(state_manager.get_genome_state(), int)
        assert isinstance(state_manager.get_brain_readiness(), bool)
        assert isinstance(state_manager.get_burst_engine_state(), int)
        assert isinstance(state_manager.get_fq_sampler_state(), int)
        assert isinstance(state_manager.get_exit_condition(), bool)
        assert isinstance(state_manager.get_cortical_list(), list)
        assert isinstance(state_manager.get_connected_agents(), dict)

    def test_setter_methods_handle_validation(self):
        """Test that setter methods handle validation correctly."""
        from feagi.core.state_manager import FeagiStateManager
        
        state_manager = FeagiStateManager.instance()
        
        # Test valid values
        result = state_manager.set_genome_state(1)
        assert result.is_ok, "Valid genome state should succeed"
        
        # Test invalid values
        result = state_manager.set_genome_state(999)
        assert result.is_err, "Invalid genome state should fail"

    def test_state_manager_survives_stress_operations(self):
        """Test that state manager can handle rapid state changes."""
        from feagi.core.state_manager import FeagiStateManager
        
        state_manager = FeagiStateManager.instance()
        
        # Rapid state changes should not crash
        for i in range(100):
            state_manager.set_brain_readiness(i % 2 == 0)
            state_manager.get_brain_readiness()
            
        # Should still be functional
        assert isinstance(state_manager.get_comprehensive_state_report(), dict)


if __name__ == "__main__":
    pytest.main([__file__, "-v"]) 