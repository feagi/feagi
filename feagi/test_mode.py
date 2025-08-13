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
FEAGI Test Mode Module - DEPRECATED

⚠️  DEPRECATION NOTICE ⚠️

This module has been DEPRECATED and replaced by the enhanced test mode system
located in feagi.utils.test_mode.

The new system provides:
- Test Mode 1: JSON-based predictable neuron activations (original functionality)
- Test Mode 2: Numpy-based scalable random neuron generation (new for scalability testing)

Migration Guide:
- Old: feagi --test
- New: feagi --test-mode-1 (or just feagi --test)

- Old: No scalability testing option
- New: feagi --test-mode-2 --test-target-neurons 50000

For full documentation, see: feagi_core/feagi/utils/test_mode/README.md

This legacy module will be removed in a future version.
"""

import warnings

from feagi.utils.logger import setup_logger

logger = setup_logger("feagi.test_mode.legacy")

# Issue deprecation warning
warnings.warn(
    "feagi.test_mode is deprecated. Use feagi.utils.test_mode instead. "
    "See feagi_core/feagi/utils/test_mode/README.md for migration guide.",
    DeprecationWarning,
    stacklevel=2,
)


def run_test_mode(core_api_service, **kwargs):
    """
    DEPRECATED: Legacy test mode function.

    This function redirects to the new enhanced test mode system.

    Args:
        core_api_service: FEAGI's core API service
        **kwargs: Test configuration options

    Returns:
        bool: True if tests passed, False otherwise
    """
    logger.warning(
        "⚠️  DEPRECATION WARNING: feagi.test_mode.run_test_mode is deprecated"
    )
    logger.warning("   Please use feagi.utils.test_mode.run_test_mode instead")
    logger.warning(
        "   This legacy function will be removed in a future version"
    )
    logger.warning(
        "   See feagi_core/feagi/utils/test_mode/README.md for full documentation"
    )

    # Redirect to the new implementation
    from feagi.utils.test_mode import run_test_mode as new_run_test_mode

    # Default to mode_1 for backwards compatibility
    test_mode = kwargs.get("test_mode", "mode_1")

    logger.info(f"Redirecting to new test mode system: {test_mode}")

    return new_run_test_mode(
        core_api_service=core_api_service, test_mode=test_mode, **kwargs
    )


# Import the original class for backwards compatibility
class FeagiTestRunner:
    """DEPRECATED: Legacy test runner class."""

    def __init__(self, *args, **kwargs):
        warnings.warn(
            "FeagiTestRunner from feagi.test_mode is deprecated. "
            "Use feagi.utils.test_mode.FeagiTestRunner instead.",
            DeprecationWarning,
            stacklevel=2,
        )

        # Import and delegate to the new implementation
        from feagi.utils.test_mode import FeagiTestRunner as NewFeagiTestRunner

        # Default to mode_1 for backwards compatibility
        if "test_mode" not in kwargs:
            kwargs["test_mode"] = "mode_1"

        self._new_runner = NewFeagiTestRunner(*args, **kwargs)

    def __getattr__(self, name):
        """Delegate all attribute access to the new runner."""
        return getattr(self._new_runner, name)
