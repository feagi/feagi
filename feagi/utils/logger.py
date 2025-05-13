"""Logging utilities for FEAGI."""
import logging
import os
import sys
import unicodedata
from typing import Optional


# -----------------------------------------------------------------------------
# Width-aware string utilities
# -----------------------------------------------------------------------------

def display_width(text: str) -> int:
    """
    Compute the visual width of a string, treating wide and fullwidth Unicode
    characters (e.g., CJK, emojis) as 2 columns, and others as 1.
    """
    width = 0
    for char in text:
        if unicodedata.east_asian_width(char) in ('F', 'W'):
            width += 2
        else:
            width += 1
    return width

def pad_display(text: str, width: int) -> str:
    """
    Pads a string with spaces to match a desired display width.
    """
    pad_len = max(0, width - display_width(text))
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

def setup_logger(
    name: str = "feagi",
    level: int = logging.INFO,
    log_file: Optional[str] = None,
    console: bool = True,
    tag: Optional[str] = None,
) -> EmojiAdapter:
    LEVEL_MAP = {
        "DEBUG":    "DEBUG   ",
        "INFO":     "INFO    ",
        "WARNING":  "WARNING ",
        "ERROR":    "ERROR   ",
        "CRITICAL": "CRITICAL"
    }

    class AlignedFormatter(logging.Formatter):
        def format(self, record):
            emoji1 = getattr(record, 'emoji1', '')
            emoji2 = getattr(record, 'emoji2', '')
            emoji_block = pad_display(f"{emoji1}{emoji2}", 4)

            level = pad_display(LEVEL_MAP.get(record.levelname, record.levelname), 8)
            timestamp = self.formatTime(record, self.datefmt)
            tag_str = f"[{record.__dict__.get('label', '')}] " if record.__dict__.get('label') else ""
            message = record.getMessage()

            return f"{emoji_block}{level}  {timestamp} {tag_str}{message}"

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.handlers.clear()
    logger.propagate = False

    formatter = AlignedFormatter(datefmt="%Y-%m-%d %H:%M:%S")

    if log_file:
        os.makedirs(os.path.dirname(log_file), exist_ok=True)
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    if console:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(level)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

    return EmojiAdapter(logger, {"label": tag or name})
