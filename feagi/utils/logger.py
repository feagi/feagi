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

"""Logging utilities for FEAGI."""
import logging
import os
import sys
import unicodedata
from typing import Optional
from pathlib import Path
from datetime import datetime


# -----------------------------------------------------------------------------
# Width-aware string utilities
# -----------------------------------------------------------------------------

def debug_emoji_width(emoji):
    """
    Debug function to print information about an emoji's Unicode properties.
    """
    print(f"Emoji: {emoji}")
    print(f"Code point: U+{ord(emoji):04X}")
    print(f"Name: {unicodedata.name(emoji, 'Unknown')}")
    print(f"Category: {unicodedata.category(emoji)}")
    print(f"East Asian Width: {unicodedata.east_asian_width(emoji)}")
    
    # Calculate width using our criteria
    code_point = ord(emoji)
    is_emoji_char = (
        emoji in ('✓', '✔', '✅', '☑', '☒', '☐', '☑️', '❌', '⚠️', '⚡', '🔄', '🔍', '➡️', '⬅️', '⬆️', '⬇️')
        or (0x1F300 <= code_point <= 0x1F5FF)
        or (0x1F600 <= code_point <= 0x1F64F)
        or (0x1F680 <= code_point <= 0x1F6FF)
        or (0x1F900 <= code_point <= 0x1F9FF)
        or (0x2700 <= code_point <= 0x27BF)
        or (0x2600 <= code_point <= 0x26FF)
        or (0x2300 <= code_point <= 0x23FF and unicodedata.category(emoji).startswith('S'))
    )
    
    if is_emoji_char:
        width = 2
    elif unicodedata.east_asian_width(emoji) in ('F', 'W'):
        width = 2
    else:
        width = 1
    
    print(f"Display width (our function): {width}")
    print(f"Combining: {unicodedata.combining(emoji)}")
    print("---")

# Force all emojis to be treated as exactly 2 columns wide
EMOJI_WIDTH = 2

def is_emoji(char):
    """
    Determine if a character is likely an emoji.
    """
    code_point = ord(char)
    
    # Check common emoji Unicode ranges
    return (
        # Specific symbols we want to treat as emojis
        char in ('✓', '✔', '✅', '☑', '☒', '☐', '☑️', '❌', '⚠️', '⚡', '🔄', '🔍', '➡️', '⬅️', '⬆️', '⬇️')
        # Emoji & Pictographs
        or (0x1F300 <= code_point <= 0x1F5FF)
        # Emoticons
        or (0x1F600 <= code_point <= 0x1F64F)
        # Transport & Map Symbols
        or (0x1F680 <= code_point <= 0x1F6FF)
        # Supplemental Symbols and Pictographs
        or (0x1F900 <= code_point <= 0x1F9FF)
        # Dingbats
        or (0x2700 <= code_point <= 0x27BF)
        # Miscellaneous Symbols
        or (0x2600 <= code_point <= 0x26FF)
        # Miscellaneous Technical
        or (0x2300 <= code_point <= 0x23FF and unicodedata.category(char).startswith('S'))
    )

def display_width(text: str) -> int:
    """
    Compute the visual width of a string, treating all emoji-like characters consistently.
    
    This function ensures all emojis are treated as having the same width (2) for consistent alignment,
    regardless of how they might be classified by Unicode.
    """
    width = 0
    for char in text:
        if is_emoji(char):
            width += EMOJI_WIDTH
        # Handle CJK and other wide characters
        elif unicodedata.east_asian_width(char) in ('F', 'W'):
            width += 2
        else:
            width += 1
    return width

def pad_display(text: str, width: int) -> str:
    """
    Pads a string with spaces to match a desired display width.
    
    This function ensures consistent padding for all text, including emoji characters,
    to maintain proper alignment in logs.
    """
    # Calculate the actual display width
    actual_width = display_width(text)
    
    # Add padding to reach the desired width
    pad_len = max(0, width - actual_width)
    
    # Return the padded string
    return text + ' ' * pad_len

# -----------------------------------------------------------------------------
# Custom LoggerAdapter
# -----------------------------------------------------------------------------

class EmojiAdapter(logging.LoggerAdapter):
    def process(self, msg, kwargs):
        emoji1 = kwargs.pop("emoji1", '')
        emoji2 = kwargs.pop("emoji2", '')
        extra = kwargs.setdefault("extra", {})
        extra["emoji1"] = emoji1
        extra["emoji2"] = emoji2
        extra["label"] = self.extra.get("label", "")
        return msg, kwargs

# -----------------------------------------------------------------------------
# Logger setup with emoji-aligned formatting
# -----------------------------------------------------------------------------

# Global flag to track if we've already shown the main logger setup messages
_MAIN_LOGGER_SETUP_SHOWN = False

def setup_logger(
    name: str = "feagi",
    level: int = logging.WARNING,
    log_file: Optional[str] = None,
    console: bool = True,
    tag: Optional[str] = None,
) -> EmojiAdapter:
    global _MAIN_LOGGER_SETUP_SHOWN
    
    # Check for CLI-provided log level override
    cli_log_level = os.environ.get('FEAGI_CLI_LOG_LEVEL')
    if cli_log_level:
        level = getattr(logging, cli_log_level, level)
    LEVEL_MAP = {
        "DEBUG":    "DEBUG   ",
        "INFO":     "INFO    ",
        "WARNING":  "WARNING ",
        "ERROR":    "ERROR   ",
        "CRITICAL": "CRITICAL"
    }

    class AlignedFormatter(logging.Formatter):
        # Define emoji padding based on emoji type
        EMOJI_PADDING = {
            # Checkmarks and X marks
            '✓': '   ',   # 3 spaces after checkmark
            '✔': '   ',   # 3 spaces after checkmark
            '✅': '   ',   # 3 spaces after green checkmark
            '☑': '   ',   # 3 spaces after checkbox
            '❌': '   ',   # 3 spaces after red X
            '❎': '   ',   # 3 spaces after green X
            '⛔': '   ',   # 3 spaces after no entry
            '⚠️': '   ',   # 3 spaces after warning
            
            # Common FEAGI emojis
            '🧬': '  ',    # 2 spaces after DNA
            '🧠': '  ',    # 2 spaces after brain
            '⚡': '   ',   # 3 spaces after lightning
            '💥': '  ',    # 2 spaces after explosion
            '🔄': '  ',    # 2 spaces after refresh
            '⚙️': '  ',    # 2 spaces after gear
            '🔍': '  ',    # 2 spaces after magnifying glass
            '📊': '  ',    # 2 spaces after chart
            '🚀': '  ',    # 2 spaces after rocket
            '🛑': '  ',    # 2 spaces after stop sign
            '📝': '  ',    # 2 spaces after memo
            '🔧': '  ',    # 2 spaces after wrench
            '📡': '  ',    # 2 spaces after satellite
            '🌐': '  ',    # 2 spaces after globe
            
            # Status indicators
            'Er': '  ',    # 2 spaces after Error prefix
            'Br': '  ',    # 2 spaces after Brain prefix
            '[C': '  ',    # 2 spaces after Corticogenesis prefix
            '[N': '  ',    # 2 spaces after Neurogenesis prefix
            '[F': '  ',    # 2 spaces after Failed prefix
        }
        
        # Default padding if emoji not in the map
        DEFAULT_PADDING = '  '  # 2 spaces
        
        def format(self, record):
            # Get emojis from record
            emoji1 = getattr(record, 'emoji1', '')
            emoji2 = getattr(record, 'emoji2', '')
            
            # Combine emojis
            emoji = f"{emoji1}{emoji2}"
            
            # Get specific padding for this emoji or use default
            padding = self.EMOJI_PADDING.get(emoji, self.DEFAULT_PADDING)
            
            # Create emoji block with custom padding
            emoji_block = f"{emoji}{padding}"
            
            # If no emoji, use 4 spaces
            if not emoji:
                emoji_block = "    "
                
            # Format the log level with fixed width
            level_str = LEVEL_MAP.get(record.levelname, record.levelname)
            level = f"{level_str:<8}"  # Left-align with fixed 8 chars
            
            # Format timestamp and message
            timestamp = self.formatTime(record, self.datefmt)
            tag_str = f"[{record.__dict__.get('label', '')}] " if record.__dict__.get('label') else ""
            message = record.getMessage()

            # Build the final log line with fixed column widths
            return f"{emoji_block}{level}  {timestamp} {tag_str}{message}"

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.handlers.clear()
    logger.propagate = False

    formatter = AlignedFormatter(datefmt="%Y-%m-%d %H:%M:%S")

    # ALWAYS create a log file in feagi_core/feagi/logs/run_TIMESTAMP/ directory
    # This ensures we can always see what's happening during startup/operation
    # Each run gets its own directory to avoid clutter
    try:
        # Get the feagi_core directory - look for it in the current working directory or parents
        feagi_core_dir = None
        current_path = Path.cwd()
        
        # Look for feagi_core directory up the directory tree
        for path in [current_path] + list(current_path.parents):
            potential_feagi_core = path / "feagi_core"
            if potential_feagi_core.exists() and potential_feagi_core.is_dir():
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
        
        # Generate timestamped log filename (shorter since we have run directory)
        file_timestamp = datetime.now().strftime("%H%M%S")
        
        # Use provided log_file or generate one based on logger name
        if log_file:
            # If user provided a specific file, use it but put it in run directory
            log_filename = Path(log_file).name
        else:
            # Generate filename based on logger name and file timestamp
            safe_name = name.replace(".", "_").replace("/", "_")
            log_filename = f"{safe_name}_{file_timestamp}.log"
        
        # Full path for the log file
        full_log_path = run_dir / log_filename
        
        # Create file handler
        file_handler = logging.FileHandler(full_log_path)
        file_handler.setLevel(level)
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
        
        # Log the file location ONLY ONCE for the main logger to avoid spam
        # Only log setup info for the main "feagi" logger, not every module
        if console and name == "feagi":
            if not _MAIN_LOGGER_SETUP_SHOWN:
                temp_console = logging.StreamHandler(sys.stdout)
                temp_console.setFormatter(formatter)
                temp_logger = logging.getLogger("temp_setup")
                temp_logger.addHandler(temp_console)
                temp_logger.setLevel(logging.INFO)
                temp_adapter = EmojiAdapter(temp_logger, {"label": "logger_setup"})
                temp_adapter.info(f"📁 FEAGI run: {run_dir}", emoji1="📁")
                temp_adapter.info(f"📝 Log file: {full_log_path}", emoji1="📝")
                temp_logger.removeHandler(temp_console)
                _MAIN_LOGGER_SETUP_SHOWN = True
            
    except Exception as e:
        # If log file creation fails, just continue with console logging
        # Only show warning for main logger to avoid spam
        if console and name == "feagi":
            if not _MAIN_LOGGER_SETUP_SHOWN:
                temp_console = logging.StreamHandler(sys.stdout)
                temp_console.setFormatter(formatter)
                temp_logger = logging.getLogger("temp_setup")
                temp_logger.addHandler(temp_console)
                temp_logger.setLevel(logging.WARNING)
                temp_adapter = EmojiAdapter(temp_logger, {"label": "logger_setup"})
                temp_adapter.warning(f"⚠️  Failed to create log file: {e}", emoji1="⚠️")
                temp_logger.removeHandler(temp_console)
                _MAIN_LOGGER_SETUP_SHOWN = True

    # Add console handler if requested
    if console:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(level)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

    return EmojiAdapter(logger, {"label": tag or name})
