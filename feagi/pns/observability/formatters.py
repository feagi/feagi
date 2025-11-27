"""
Output Formatters

Formatters for displaying observability data in various formats.
"""

import json
import csv
from typing import Dict, Any, List
from datetime import datetime
from pathlib import Path


class Formatter:
    """Base formatter class."""
    
    def format(self, data: Any) -> str:
        """Format data for output."""
        raise NotImplementedError


class JSONFormatter(Formatter):
    """Format data as JSON."""
    
    def __init__(self, indent: int = 2):
        self.indent = indent
    
    def format(self, data: Any) -> str:
        """Format as JSON string."""
        return json.dumps(data, indent=self.indent, default=str)
    
    def format_to_file(self, data: Any, output_path: str):
        """Write data to JSON file."""
        output_file = Path(output_path)
        output_file.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_file, 'w') as f:
            json.dump(data, f, indent=self.indent, default=str)


class CSVFormatter(Formatter):
    """Format data as CSV."""
    
    def format_to_file(
        self,
        rows: List[Dict[str, Any]],
        output_path: str,
        fieldnames: List[str] = None
    ):
        """
        Write data to CSV file.
        
        Args:
            rows: List of dictionaries
            output_path: Path to output file
            fieldnames: Column names (auto-detected if None)
        """
        if not rows:
            return
        
        output_file = Path(output_path)
        output_file.parent.mkdir(parents=True, exist_ok=True)
        
        if fieldnames is None:
            fieldnames = list(rows[0].keys())
        
        with open(output_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)


class ConsoleFormatter(Formatter):
    """Format data for console output."""
    
    @staticmethod
    def format_title(title: str, width: int = 60) -> str:
        """Format a title with borders."""
        return (
            "\n" + "=" * width + "\n" +
            title.center(width) + "\n" +
            "=" * width + "\n"
        )
    
    @staticmethod
    def format_section(title: str) -> str:
        """Format a section header."""
        return f"\n{title}\n" + "-" * len(title)
    
    @staticmethod
    def format_key_value(key: str, value: Any, indent: int = 2) -> str:
        """Format a key-value pair."""
        indent_str = " " * indent
        return f"{indent_str}{key:<30} {value}"
    
    @staticmethod
    def format_table(
        headers: List[str],
        rows: List[List[Any]],
        col_widths: List[int] = None
    ) -> str:
        """Format data as a table."""
        if not rows:
            return ""
        
        # Auto-calculate column widths if not provided
        if col_widths is None:
            col_widths = [len(str(h)) for h in headers]
            for row in rows:
                for i, cell in enumerate(row):
                    col_widths[i] = max(col_widths[i], len(str(cell)))
        
        # Format header
        header_line = " | ".join(
            str(h).ljust(w) for h, w in zip(headers, col_widths)
        )
        separator = "-" * len(header_line)
        
        # Format rows
        row_lines = []
        for row in rows:
            row_line = " | ".join(
                str(cell).ljust(w) for cell, w in zip(row, col_widths)
            )
            row_lines.append(row_line)
        
        return "\n".join([header_line, separator] + row_lines)
    
    @staticmethod
    def format_progress_bar(
        current: int,
        total: int,
        width: int = 40,
        label: str = ""
    ) -> str:
        """Format a progress bar."""
        if total == 0:
            progress = 0.0
        else:
            progress = current / total
        
        filled = int(width * progress)
        bar = "█" * filled + "░" * (width - filled)
        percentage = progress * 100
        
        if label:
            return f"{label:<20} │ {bar} │ {percentage:>6.1f}%"
        else:
            return f"{bar} {percentage:>6.1f}%"

