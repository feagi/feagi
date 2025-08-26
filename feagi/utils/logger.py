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

Logging utilities for FEAGI with ASCII-safe status indicators.
"""

import logging
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, List

# -----------------------------------------------------------------------------
# ASCII Status Indicators for Embedded System Compatibility
# -----------------------------------------------------------------------------

#  ASCII status indicators optimized for embedded systems and universal
#  compatibility
ASCII_STATUS_MAP = {
    # Success/Completion indicators
    "✅": "[OK]",
    "☑": "[CHK]",
    "✓": "[OK]",
    "✔": "[OK]",
    # Error/Warning indicators
    "❌": "[ERR]",
    "⚠️": "[WARN]",
    "⚠ ": "[WARN]",  # Handle with space
    "❎": "[FAIL]",
    "⛔": "[STOP]",
    "☣️": "[TOXIC]",
    # Process/Action indicators
    "🚀": "[START]",
    "🔧": "[CONFIG]",
    "⚙️": "[SETUP]",
    "🔄": "[PROC]",
    "⚡": "[FAST]",
    "💥": "[BURST]",
    # Data/Information indicators
    "📊": "[STATS]",
    "📈": "[UP]",
    "📉": "[DOWN]",
    "📝": "[LOG]",
    "📁": "[FOLDER]",
    "💾": "[SAVE]",
    "🗜️": "[COMPRESS]",
    # Brain/Neural indicators
    "🧠": "[BRAIN]",
    "🧬": "[DNA]",
    "🎯": "[TARGET]",
    "🔍": "[SEARCH]",
    "🌐": "[NET]",
    "📡": "[COMM]",
    # Status/State indicators
    "🔥": "[DEBUG]",
    "💡": "[INFO]",
    "⭐": "[STAR]",
    "🎨": "[RENDER]",
    "🛑": "[HALT]",
    # System/Hardware indicators
    "🖥️": "[SYSTEM]",
    "👁️": "[MONITOR]",
    # Arrows/Direction indicators
    "➡️": "[>]",
    "⬅️": "[<]",
    "⬆️": "[^]",
    "⬇️": "[v]",
    "🛤️": "[PATH]",
    # Time/Process indicators
    "⏱️": "[TIME]",
    "▶️": "[PLAY]",
    "⏸️": "[PAUSE]",
    "⏹️": "[STOP]",
    # Additional FEAGI-specific indicators
    "🎮": "[CTRL]",
    "🔗": "[LINK]",
    "♻️": "[RELOAD]",
    "⏭️": "[SKIP]",
    "🏷️": "[TAG]",
    "🗺️": "[MAP]",
    "🎉": "[DONE]",
    "🕸️": "[WEB]",
    "↩️": "[BACK]",
}


def get_ascii_status(emoji_or_text: str) -> str:
    """Convert emoji or text to ASCII equivalent for embedded system
    compatibility.

    Args:
        emoji_or_text: Input emoji or text string

    Returns:
        ASCII-safe equivalent string
    """
    if not emoji_or_text:
        return ""

    # Direct mapping for known emojis
    if emoji_or_text in ASCII_STATUS_MAP:
        return ASCII_STATUS_MAP[emoji_or_text]

    # Handle strings that may contain emojis
    result = emoji_or_text
    for emoji, ascii_equiv in ASCII_STATUS_MAP.items():
        result = result.replace(emoji, ascii_equiv)

    return result


# -----------------------------------------------------------------------------
# Custom LoggerAdapter with ASCII Status Indicators
# -----------------------------------------------------------------------------


class StatusAdapter(logging.LoggerAdapter):
    """Logger adapter that uses ASCII status indicators for embedded system
    compatibility.

    Replaces emojis with performance-optimized ASCII equivalents.
    """

    def process(self, msg, kwargs):
        """Process log message with ASCII status indicator handling."""
        # Handle both old emoji1/emoji2 parameters and new status parameter
        status1 = kwargs.pop("emoji1", kwargs.pop("status", ""))
        status2 = kwargs.pop("emoji2", "")

        # Convert to ASCII equivalents
        ascii_status1 = get_ascii_status(status1)
        ascii_status2 = get_ascii_status(status2)

        # Clean up the main message text as well
        cleaned_msg = get_ascii_status(str(msg))

        extra = kwargs.setdefault("extra", {})
        extra["status1"] = ascii_status1
        extra["status2"] = ascii_status2
        extra["label"] = self.extra.get("label", "")

        return cleaned_msg, kwargs

    def info(self, msg, *args, **kwargs):
        """Log info message with ASCII status indicator."""
        # Default to [INFO] if no status provided
        if "emoji1" not in kwargs and "status" not in kwargs:
            kwargs["status"] = "[INFO]"
        super().info(msg, *args, **kwargs)

    def warning(self, msg, *args, **kwargs):
        """Log warning message with ASCII status indicator."""
        # Default to [WARN] if no status provided
        if "emoji1" not in kwargs and "status" not in kwargs:
            kwargs["status"] = "[WARN]"
        super().warning(msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        """Log error message with ASCII status indicator."""
        # Default to [ERR] if no status provided
        if "emoji1" not in kwargs and "status" not in kwargs:
            kwargs["status"] = "[ERR]"
        super().error(msg, *args, **kwargs)

    def debug(self, msg, *args, **kwargs):
        """Log debug message with ASCII status indicator."""
        # Default to [DEBUG] if no status provided
        if "emoji1" not in kwargs and "status" not in kwargs:
            kwargs["status"] = "[DEBUG]"
        super().debug(msg, *args, **kwargs)


# -----------------------------------------------------------------------------
# Logger setup with ASCII-aligned formatting for embedded systems
# -----------------------------------------------------------------------------

# Global flag to track if we've already shown the main logger setup messages
_MAIN_LOGGER_SETUP_SHOWN = False
# Global storage for deferred setup info
_DEFERRED_SETUP_INFO: Optional[Dict[str, Any]] = None


def clear_logger_cache():
    """Clear any logger caches (currently no-op since caching was removed)."""
    pass


def show_deferred_setup_info():
    """Show the deferred logger setup info if CLI log level allows it."""
    global _DEFERRED_SETUP_INFO
    if _DEFERRED_SETUP_INFO is None:
        return

    # Check CLI log level to see if we should show INFO messages
    cli_level_str = os.environ.get("FEAGI_CLI_LOG_LEVEL")
    should_show = True
    if cli_level_str:
        cli_level = getattr(logging, cli_level_str.upper(), logging.INFO)
        should_show = cli_level <= logging.INFO

    if should_show:
        # Create temporary logger to show setup info
        temp_console = logging.StreamHandler(sys.stdout)
        temp_console.setFormatter(_DEFERRED_SETUP_INFO["formatter"])
        temp_logger = logging.getLogger("temp_setup_deferred")
        temp_logger.handlers.clear()  # Clear any existing handlers
        temp_logger.propagate = (
            False  # Prevent propagation to avoid duplicates
        )
        temp_logger.addHandler(temp_console)
        temp_logger.setLevel(logging.INFO)
        temp_adapter = StatusAdapter(temp_logger, {"label": "logger_setup"})
        temp_adapter.info(
            f"FEAGI run: {_DEFERRED_SETUP_INFO['run_dir']}", status="[FOLDER]"
        )
        temp_adapter.info(
            f"Log file: {_DEFERRED_SETUP_INFO['log_path']}", status="[LOG]"
        )
        temp_logger.removeHandler(temp_console)

    # Clear the deferred info so it's only shown once
    _DEFERRED_SETUP_INFO = None


def _check_module_debug_override(logger_name: str) -> Optional[int]:
    """Check if a module-specific debug flag should override the logger level.
    
    This function checks environment variables set by CLI debug flags to determine
    if a specific logger should be set to DEBUG level regardless of global settings.
    
    Args:
        logger_name: The name of the logger being created
        
    Returns:
        logging.DEBUG if module should be in debug mode, None otherwise
    """
    import logging
    import os
    
    # Define mappings between debug environment variables and logger hierarchies
    debug_env_to_loggers = {
        "FEAGI_DEBUG_NPU": [
            "feagi.npu",
            "feagi.npu.burst_engine",
            "feagi.npu.fcl_manager", 
            "feagi.npu.fcl_injection_service",
            "feagi.npu.special_area_handler",
            "feagi.npu.memory_processor",
            "feagi.npu.fq_sampler"
        ],
        "FEAGI_DEBUG_API": [
            "feagi.api",
            "feagi.api.rest",
            "feagi.api.core",
            "feagi.api.gateway",
            "feagi.api.protocols",
            "feagi.api.transport",
            "feagi.api.zmq"
        ],
        "FEAGI_DEBUG_BDU": [
            "feagi.bdu",
            "feagi.bdu.connectivity",
            "feagi.bdu.embryogenesis",
            "feagi.bdu.models",
            "feagi.bdu.utils"
        ],
        "FEAGI_DEBUG_ZMQ": [
            "feagi.api.zmq",
            "feagi.api.zmq.streams",
            "feagi.api.zmq.neural",
            "feagi.api.zmq.memory",
            "feagi.api.zmq.patterns"
        ],
        "FEAGI_DEBUG_MEM": [
            "feagi.npu.memory_processor",
            "feagi.bdu.models.memory",
            "feagi.core.memory"
        ]
    }
    
    # Check each debug environment variable
    for env_var, logger_hierarchies in debug_env_to_loggers.items():
        if os.environ.get(env_var, "0") == "1":
            # Check if this logger matches any of the hierarchies for this debug flag
            for hierarchy in logger_hierarchies:
                if logger_name == hierarchy or logger_name.startswith(hierarchy + "."):
                    return logging.DEBUG
    
    return None


def setup_logger(
    name: str = "feagi",
    level: Optional[int] = None,  # Changed from WARNING default to None
    log_file: Optional[str] = None,
    console: bool = True,
    tag: Optional[str] = None,
) -> StatusAdapter:
    global _MAIN_LOGGER_SETUP_SHOWN

    #  Determine log level priority: parameter > module debug override > CLI env var > config > default
    final_level = level

    if final_level is None:
        # Check for module-specific debug override FIRST
        debug_override = _check_module_debug_override(name)
        if debug_override is not None:
            final_level = debug_override

    if final_level is None:
        # Check for CLI-provided log level override
        cli_log_level = os.environ.get("FEAGI_CLI_LOG_LEVEL")
        if cli_log_level:
            final_level = getattr(logging, cli_log_level.upper(), None)

    if final_level is None:
        # Try to get log level from FEAGI configuration
        # Note: Removed caching to allow config updates to take effect
        try:
            from feagi.config.toml_loader import load_feagi_config

            config = load_feagi_config()
            config_log_level = config.get("system", {}).get(
                "log_level", "INFO"
            )
            final_level = getattr(
                logging, config_log_level.upper(), logging.INFO
            )
        except (ImportError, Exception):
            #  Fallback to INFO if config is not available (e.g., during
            #  early startup)
            final_level = logging.INFO

    LEVEL_MAP = {
        "DEBUG": "DEBUG   ",
        "INFO": "INFO    ",
        "WARNING": "WARNING ",
        "ERROR": "ERROR   ",
        "CRITICAL": "CRITICAL",
    }

    class ASCIIFormatter(logging.Formatter):
        """ASCII-optimized formatter for embedded system compatibility."""

        # Status indicator padding for consistent alignment
        STATUS_PADDING = {
            "[OK]": "     ",  # 5 spaces after [OK]
            "[CHK]": "    ",  # 4 spaces after [CHK]
            "[ERR]": "    ",  # 4 spaces after [ERR]
            "[WARN]": "   ",  # 3 spaces after [WARN]
            "[FAIL]": "   ",  # 3 spaces after [FAIL]
            "[STOP]": "   ",  # 3 spaces after [STOP]
            "[START]": "  ",  # 2 spaces after [START]
            "[CONFIG]": " ",  # 1 space after [CONFIG]
            "[SETUP]": "  ",  # 2 spaces after [SETUP]
            "[PROC]": "   ",  # 3 spaces after [PROC]
            "[FAST]": "   ",  # 3 spaces after [FAST]
            "[BURST]": "  ",  # 2 spaces after [BURST]
            "[STATS]": "  ",  # 2 spaces after [STATS]
            "[UP]": "      ",  # 6 spaces after [UP]
            "[DOWN]": "   ",  # 3 spaces after [DOWN]
            "[LOG]": "     ",  # 5 spaces after [LOG]
            "[FOLDER]": " ",  # 1 space after [FOLDER]
            "[SAVE]": "   ",  # 3 spaces after [SAVE]
            "[BRAIN]": "  ",  # 2 spaces after [BRAIN]
            "[DNA]": "     ",  # 5 spaces after [DNA]
            "[TARGET]": " ",  # 1 space after [TARGET]
            "[SEARCH]": " ",  # 1 space after [SEARCH]
            "[NET]": "     ",  # 5 spaces after [NET]
            "[COMM]": "   ",  # 3 spaces after [COMM]
            "[DEBUG]": "  ",  # 2 spaces after [DEBUG]
            "[INFO]": "   ",  # 3 spaces after [INFO]
            "[STAR]": "   ",  # 3 spaces after [STAR]
            "[RENDER]": " ",  # 1 space after [RENDER]
            "[HALT]": "   ",  # 3 spaces after [HALT]
            "[>]": "       ",  # 7 spaces after [>]
            "[<]": "       ",  # 7 spaces after [<]
            "[^]": "       ",  # 7 spaces after [^]
            "[v]": "       ",  # 7 spaces after [v]
            "[TIME]": "   ",  # 3 spaces after [TIME]
            "[PLAY]": "   ",  # 3 spaces after [PLAY]
            "[PAUSE]": "  ",  # 2 spaces after [PAUSE]
            "[CTRL]": "   ",  # 3 spaces after [CTRL]
            "[LINK]": "   ",  # 3 spaces after [LINK]
            "[RELOAD]": " ",  # 1 space after [RELOAD]
            "[SKIP]": "   ",  # 3 spaces after [SKIP]
        }

        # Default padding for unknown status indicators
        DEFAULT_PADDING = "  "  # 2 spaces

        def format(self, record):
            # Get status indicators from record
            status1 = getattr(record, "status1", "")
            status2 = getattr(record, "status2", "")

            # Combine status indicators
            status = f"{status1}{status2}"

            # Get specific padding for this status or use default
            padding = self.STATUS_PADDING.get(status, self.DEFAULT_PADDING)

            # Create status block with consistent padding
            status_block = f"{status}{padding}"

            #  If no status indicator, show log level instead with consistent
            #  spacing
            if not status:
                # Format the log level with fixed width for alignment
                level_str = LEVEL_MAP.get(record.levelname, record.levelname)
                status_block = (
                    f"{level_str:<8}"  # Left-align with fixed 8 chars
                )

            # Format timestamp and message
            timestamp = self.formatTime(record, self.datefmt)
            tag_str = (
                f"[{record.__dict__.get('label', '')}] "
                if record.__dict__.get("label")
                else ""
            )
            message = record.getMessage()

            #  Build the final log line - only show status block (which is
            #  either status indicator OR level)
            return f"{status_block}  {timestamp} {tag_str}{message}"

    logger = logging.getLogger(name)

    # Always apply the computed level to the logger, even if handlers exist
    # This ensures CLI/config updates affect existing module loggers reliably
    logger.setLevel(final_level)
    logger.propagate = False

    # Only create handlers once to prevent duplication
    if not logger.handlers or len(logger.handlers) == 0:

        formatter = ASCIIFormatter(datefmt="%Y-%m-%d %H:%M:%S")

        #  ALWAYS create a log file in feagi_core/feagi/logs/run_TIMESTAMP/
        #  directory
        try:
            #  Get the feagi_core directory - look for it in the current
            #  working directory or parents
            feagi_core_dir = None
            current_path = Path.cwd()

            # Look for feagi_core directory up the directory tree
            for path in [current_path] + list(current_path.parents):
                potential_feagi_core = path / "feagi_core"
                if (
                    potential_feagi_core.exists()
                    and potential_feagi_core.is_dir()
                ):
                    feagi_core_dir = potential_feagi_core
                    break

            # If not found, check if we're already in feagi_core
            if feagi_core_dir is None and current_path.name == "feagi_core":
                feagi_core_dir = current_path

            # If still not found, use current directory
            if feagi_core_dir is None:
                feagi_core_dir = current_path

            # Create base logs directory under feagi/
            base_logs_dir = feagi_core_dir / "feagi" / "logs"
            base_logs_dir.mkdir(parents=True, exist_ok=True)

            # Create run-specific directory
            run_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            run_dir = base_logs_dir / f"run_{run_timestamp}"
            run_dir.mkdir(parents=True, exist_ok=True)

            # Generate timestamped log filename
            file_timestamp = datetime.now().strftime("%H%M%S")

            # Use provided log_file or generate one based on logger name
            if log_file:
                log_filename = Path(log_file).name
            else:
                safe_name = name.replace(".", "_").replace("/", "_")
                log_filename = f"{safe_name}_{file_timestamp}.log"

            # Full path for the log file
            full_log_path = run_dir / log_filename

            # Create file handler
            file_handler = logging.FileHandler(full_log_path, encoding="utf-8")
            file_handler.setLevel(final_level)
            file_handler.setFormatter(formatter)
            logger.addHandler(file_handler)

            # Create a symlink to "latest" run for easy access
            latest_run_link = base_logs_dir / "latest_run"
            try:
                if latest_run_link.exists() or latest_run_link.is_symlink():
                    latest_run_link.unlink()
                latest_run_link.symlink_to(f"run_{run_timestamp}")
            except (OSError, NotImplementedError):
                # Symlinks might not be supported on all platforms
                pass

            # Create individual file symlink within the run directory
            latest_file_link = run_dir / f"{safe_name}_latest.log"
            try:
                if latest_file_link.exists() or latest_file_link.is_symlink():
                    latest_file_link.unlink()
                latest_file_link.symlink_to(log_filename)
            except (OSError, NotImplementedError):
                # Symlinks might not be supported on all platforms
                pass

            # Store setup info for later display after CLI parsing
            if console and name == "feagi":
                if not _MAIN_LOGGER_SETUP_SHOWN:
                    # Store the setup information globally for later display
                    global _DEFERRED_SETUP_INFO
                    _DEFERRED_SETUP_INFO = {
                        "run_dir": run_dir,
                        "log_path": full_log_path,
                        "formatter": formatter,
                    }
                    _MAIN_LOGGER_SETUP_SHOWN = True

        except Exception as e:
            # If log file creation fails, just continue with console logging
            if console and name == "feagi":
                if not _MAIN_LOGGER_SETUP_SHOWN:
                    # Check if CLI override will suppress these messages
                    cli_level_str = os.environ.get("FEAGI_CLI_LOG_LEVEL")
                    should_show_warning = True
                    if cli_level_str:
                        cli_level = getattr(
                            logging, cli_level_str.upper(), logging.INFO
                        )
                        should_show_warning = cli_level <= logging.WARNING
                    else:
                        # Use current final_level if no CLI override
                        should_show_warning = final_level <= logging.WARNING

                    if should_show_warning:
                        temp_console = logging.StreamHandler(sys.stdout)
                        temp_console.setFormatter(formatter)
                        temp_logger = logging.getLogger("temp_setup")
                        temp_logger.addHandler(temp_console)
                        temp_logger.setLevel(
                            final_level
                        )  # Use the same level as final_level
                        temp_adapter = StatusAdapter(
                            temp_logger, {"label": "logger_setup"}
                        )
                        temp_adapter.warning(
                            f"Failed to create log file: {e}", status="[WARN]"
                        )
                        temp_logger.removeHandler(temp_console)
                    _MAIN_LOGGER_SETUP_SHOWN = True

        # Add console handler if requested
        if console:
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(final_level)
            console_handler.setFormatter(formatter)
            logger.addHandler(console_handler)
    else:
        # Logger already configured - update handler levels to match final_level
        for handler in logger.handlers:
            handler.setLevel(final_level)

    return StatusAdapter(logger, {"label": tag or name})


# -----------------------------------------------------------------------------
# Subsystem logger application (runtime level sync for loggers and handlers)
# -----------------------------------------------------------------------------

# Centralized subsystem -> logger hierarchies mapping
SUBSYSTEM_LOGGER_HIERARCHIES: Dict[str, List[str]] = {
    "api": [
        "feagi.api",
        "feagi.api.rest",
        "feagi.api.core",
        "feagi.api.gateway",
        "feagi.api.protocols",
        "feagi.api.transport",
        "feagi.api.zmq",
    ],
    "npu": [
        "feagi.npu",
        "feagi.npu.burst_engine",
        "feagi.npu.fcl_manager",
        "feagi.npu.fcl_injection_service",
        "feagi.npu.special_area_handler",
        "feagi.npu.memory_processor",
        "feagi.npu.fq_sampler",
    ],
    "bdu": [
        "feagi.bdu",
        "feagi.bdu.connectivity",
        "feagi.bdu.embryogenesis",
        "feagi.bdu.models",
        "feagi.bdu.utils",
    ],
    "evo": [
        "feagi.evo",
    ],
    "zmq": [
        "feagi.api.zmq",
        "feagi.api.zmq.streams",
        "feagi.api.zmq.neural",
        "feagi.api.zmq.memory",
        "feagi.api.zmq.patterns",
    ],
    "mem": [
        "feagi.npu.memory_processor",
        "feagi.bdu.models.memory",
        "feagi.core.memory",
    ],
}


def apply_subsystem_log_levels(debug_cfg: Dict[str, Any], baseline_level: int) -> None:
    """Apply subsystem log levels live by updating logger and handler levels.

    Args:
        debug_cfg: StateManager debug config dict; expects keys like
                   {'debug_api': bool, 'debug_npu': bool, ...}
        baseline_level: logging.INFO/DEBUG/WARNING/etc for subsystems that are off
    """
    import logging

    # Build subsystem -> enabled map from debug_cfg
    enabled_by_subsystem = {
        "api": bool(debug_cfg.get("debug_api", False)),
        "npu": bool(debug_cfg.get("debug_npu", False)),
        "bdu": bool(debug_cfg.get("debug_bdu", False)),
        "evo": bool(debug_cfg.get("debug_evo", False)),
        "zmq": bool(debug_cfg.get("debug_zmq_inbound", False) or debug_cfg.get("debug_zmq_outbound", False)),
        "mem": bool(debug_cfg.get("mem_debug", False)),
    }

    logger_dict = logging.Logger.manager.loggerDict

    # Helper to set both logger and all handler levels
    def _set_logger_and_handlers(_logger: logging.Logger, level: int) -> None:
        try:
            _logger.setLevel(level)
            for h in getattr(_logger, "handlers", []) or []:
                h.setLevel(level)
        except Exception:
            pass

    for subsystem, hierarchies in SUBSYSTEM_LOGGER_HIERARCHIES.items():
        target_level = logging.DEBUG if enabled_by_subsystem.get(subsystem, False) else baseline_level
        for name, log_obj in logger_dict.items():
            if isinstance(log_obj, logging.Logger):
                for h in hierarchies:
                    if name == h or name.startswith(h + "."):
                        _set_logger_and_handlers(log_obj, target_level)
                        break


# Backward compatibility alias
EmojiAdapter = StatusAdapter
