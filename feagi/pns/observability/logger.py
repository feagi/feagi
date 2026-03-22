"""
Data Logger

Structured logging of sensory and motor data packets.
"""

import json
import csv
from typing import Dict, Any, Optional, List
from datetime import datetime
from pathlib import Path

from feagi.pns.observability.monitor import Monitor


class DataLogger(Monitor):
    """
    Logs sensory input and motor output data in structured format.
    
    Supports multiple output formats:
    - JSON: One file with array of entries
    - JSONL: JSON Lines (one JSON object per line)
    - CSV: Tabular format
    
    Example:
        logger = DataLogger(
            output_file="agent_data.jsonl",
            format="jsonl",
            log_inputs=True,
            log_outputs=True,
            sample_rate=1.0  # Log 100% of packets
        )
        
        brain_input.attach_monitor(logger)
        brain_output.attach_monitor(logger)
        
        # ... run agent ...
        
        logger.close()  # Flush and close
    """
    
    def __init__(
        self,
        output_file: str = "agent_data.log",
        format: str = "jsonl",
        log_inputs: bool = True,
        log_outputs: bool = True,
        sample_rate: float = 1.0,
        include_data_samples: bool = False,
        max_sample_size: int = 10,
        enabled: bool = True,
        log_level: str = "INFO"
    ):
        """
        Initialize data logger.
        
        Args:
            output_file: Path to output file
            format: Output format - "json", "jsonl", or "csv"
            log_inputs: Whether to log input data
            log_outputs: Whether to log output data
            sample_rate: Fraction of packets to log (0.0 to 1.0)
            include_data_samples: Whether to include actual data samples
            max_sample_size: Maximum number of data points to include in samples
            enabled: Whether logging is active
            log_level: Logging level
        """
        super().__init__(enabled=enabled, log_level=log_level)
        
        self.output_file = Path(output_file)
        self.format = format.lower()
        self.log_inputs = log_inputs
        self.log_outputs = log_outputs
        self.sample_rate = sample_rate
        self.include_data_samples = include_data_samples
        self.max_sample_size = max_sample_size
        
        self._entries: List[Dict[str, Any]] = []
        self._file_handle = None
        self._csv_writer = None
        self._packet_counter = 0
        
        # Create output directory
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        
        # Open file based on format
        if self.format == "jsonl":
            self._file_handle = open(self.output_file, 'w')
        elif self.format == "csv":
            self._file_handle = open(self.output_file, 'w', newline='')
            self._csv_writer = csv.writer(self._file_handle)
            # Write header
            self._csv_writer.writerow([
                'timestamp', 'type', 'cortical_area', 'neuron_count',
                'packet_size_bytes', 'duration_ms', 'command_count'
            ])
        
        self._logger.info(f"Data logger initialized: {output_file} ({format})")
    
    def _should_log_packet(self) -> bool:
        """Determine if this packet should be logged based on sample rate."""
        import random
        return random.random() < self.sample_rate
    
    def on_send_start(self, data: Dict[str, Any]):
        """Log send start (no-op)."""
        pass
    
    def on_send_complete(self, data: Dict[str, Any]):
        """Log sensory input packet."""
        if not self.enabled or not self.log_inputs:
            return
        
        if not self._should_log_packet():
            return
        
        self._packet_counter += 1
        
        entry = {
            "timestamp": datetime.now().isoformat(),
            "packet_id": self._packet_counter,
            "type": "sensory_input",
            "neuron_count": data.get('neuron_count', 0),
            "packet_size_bytes": data.get('packet_size_bytes', 0),
            "duration_ms": data.get('duration_ms', 0.0),
            "cortical_areas": data.get('cortical_areas', []),
        }
        
        if self.include_data_samples and 'data_sample' in data:
            entry['data_sample'] = data['data_sample'][:self.max_sample_size]
        
        self._write_entry(entry)
    
    def on_receive_start(self, data: Dict[str, Any]):
        """Log receive start (no-op)."""
        pass
    
    def on_receive_complete(self, data: Dict[str, Any]):
        """Log motor output commands."""
        if not self.enabled or not self.log_outputs:
            return
        
        if not self._should_log_packet():
            return
        
        self._packet_counter += 1
        
        entry = {
            "timestamp": datetime.now().isoformat(),
            "packet_id": self._packet_counter,
            "type": "motor_output",
            "command_count": data.get('command_count', 0),
            "duration_ms": data.get('duration_ms', 0.0),
        }
        
        if self.include_data_samples and 'commands' in data:
            entry['commands_sample'] = data['commands'][:self.max_sample_size]
        
        self._write_entry(entry)
    
    def _write_entry(self, entry: Dict[str, Any]):
        """Write entry to file based on format."""
        if self.format == "jsonl":
            # JSON Lines format
            self._file_handle.write(json.dumps(entry) + '\n')
            self._file_handle.flush()
        elif self.format == "json":
            # JSON array format (accumulate in memory)
            self._entries.append(entry)
        elif self.format == "csv":
            # CSV format
            self._csv_writer.writerow([
                entry.get('timestamp', ''),
                entry.get('type', ''),
                ','.join(entry.get('cortical_areas', [])),
                entry.get('neuron_count', 0),
                entry.get('packet_size_bytes', 0),
                entry.get('duration_ms', 0.0),
                entry.get('command_count', 0),
            ])
            self._file_handle.flush()
    
    def close(self):
        """Close log file and flush data."""
        if self.format == "json" and self._entries:
            # Write accumulated entries to JSON file
            with open(self.output_file, 'w') as f:
                json.dump(self._entries, f, indent=2)
        
        if self._file_handle:
            self._file_handle.close()
            self._file_handle = None
        
        self._logger.info(f"Data logger closed: {self._packet_counter} packets logged")
    
    def __del__(self):
        """Ensure file is closed on deletion."""
        if self._file_handle:
            self.close()

