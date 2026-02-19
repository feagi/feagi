"""
Metrics Collection

Collects and aggregates statistics on data flow through brain_input and brain_output.
"""

import time
import json
import csv
from typing import Dict, Any, Optional
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path

from feagi.pns.observability.monitor import Monitor


@dataclass
class InputStatistics:
    """Statistics for sensory input."""
    total_packets: int = 0
    total_bytes: int = 0
    total_neurons: int = 0
    total_duration_ms: float = 0.0
    
    @property
    def avg_packet_size(self) -> float:
        """Average packet size in bytes."""
        return self.total_bytes / self.total_packets if self.total_packets > 0 else 0.0
    
    @property
    def avg_neurons_per_packet(self) -> float:
        """Average neurons per packet."""
        return self.total_neurons / self.total_packets if self.total_packets > 0 else 0.0
    
    @property
    def avg_duration_ms(self) -> float:
        """Average send duration in ms."""
        return self.total_duration_ms / self.total_packets if self.total_packets > 0 else 0.0
    
    @property
    def data_rate_mbps(self) -> float:
        """Data rate in MB/s."""
        if self.total_duration_ms == 0:
            return 0.0
        total_seconds = self.total_duration_ms / 1000.0
        return (self.total_bytes / (1024 * 1024)) / total_seconds
    
    @property
    def packets_per_sec(self) -> float:
        """Packets per second."""
        if self.total_duration_ms == 0:
            return 0.0
        total_seconds = self.total_duration_ms / 1000.0
        return self.total_packets / total_seconds


@dataclass
class OutputStatistics:
    """Statistics for motor output."""
    total_commands: int = 0
    total_receives: int = 0
    total_duration_ms: float = 0.0
    
    @property
    def avg_commands_per_receive(self) -> float:
        """Average commands per receive operation."""
        return self.total_commands / self.total_receives if self.total_receives > 0 else 0.0
    
    @property
    def avg_latency_ms(self) -> float:
        """Average receive latency in ms."""
        return self.total_duration_ms / self.total_receives if self.total_receives > 0 else 0.0
    
    @property
    def commands_per_sec(self) -> float:
        """Commands per second."""
        if self.total_duration_ms == 0:
            return 0.0
        total_seconds = self.total_duration_ms / 1000.0
        return self.total_commands / total_seconds


@dataclass
class Statistics:
    """Overall statistics."""
    input: InputStatistics
    output: OutputStatistics
    start_time: datetime
    end_time: Optional[datetime] = None
    
    @property
    def uptime_seconds(self) -> float:
        """Uptime in seconds."""
        end = self.end_time or datetime.now()
        return (end - self.start_time).total_seconds()
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "input": asdict(self.input),
            "output": asdict(self.output),
            "start_time": self.start_time.isoformat(),
            "end_time": self.end_time.isoformat() if self.end_time else None,
            "uptime_seconds": self.uptime_seconds,
        }


class MetricsCollector(Monitor):
    """
    Collects metrics on data flow through brain_input and brain_output.
    
    Tracks:
    - Packet counts and sizes
    - Neuron counts
    - Data rates
    - Latencies
    - Commands received
    
    Example:
        metrics = MetricsCollector()
        brain_input.attach_monitor(metrics)
        brain_output.attach_monitor(metrics)
        
        # ... run agent ...
        
        stats = metrics.get_statistics()
        print(f"Data rate: {stats.input.data_rate_mbps:.2f} MB/s")
        print(f"Avg latency: {stats.output.avg_latency_ms:.2f} ms")
        
        # Export
        metrics.export_json("metrics.json")
        metrics.export_csv("metrics.csv")
    """
    
    def __init__(
        self,
        enabled: bool = True,
        collect_interval_ms: int = 1000,
        log_level: str = "INFO"
    ):
        """
        Initialize metrics collector.
        
        Args:
            enabled: Whether collection is active
            collect_interval_ms: Metrics aggregation interval (unused, for future)
            log_level: Logging level
        """
        super().__init__(enabled=enabled, log_level=log_level)
        self.collect_interval_ms = collect_interval_ms
        
        self._input_stats = InputStatistics()
        self._output_stats = OutputStatistics()
        self._start_time = datetime.now()
    
    def on_send_start(self, data: Dict[str, Any]):
        """Record send start (no-op for metrics)."""
        pass
    
    def on_send_complete(self, data: Dict[str, Any]):
        """Record send completion."""
        if not self.enabled:
            return
        
        self._input_stats.total_packets += 1
        self._input_stats.total_bytes += data.get('packet_size_bytes', 0)
        self._input_stats.total_neurons += data.get('neuron_count', 0)
        self._input_stats.total_duration_ms += data.get('duration_ms', 0.0)
    
    def on_receive_start(self, data: Dict[str, Any]):
        """Record receive start (no-op for metrics)."""
        pass
    
    def on_receive_complete(self, data: Dict[str, Any]):
        """Record receive completion."""
        if not self.enabled:
            return
        
        self._output_stats.total_receives += 1
        self._output_stats.total_commands += data.get('command_count', 0)
        self._output_stats.total_duration_ms += data.get('duration_ms', 0.0)
    
    def get_statistics(self) -> Statistics:
        """
        Get current statistics.
        
        Returns:
            Statistics object with input and output metrics
        """
        return Statistics(
            input=self._input_stats,
            output=self._output_stats,
            start_time=self._start_time,
            end_time=None
        )
    
    def reset(self):
        """Reset all statistics."""
        self._input_stats = InputStatistics()
        self._output_stats = OutputStatistics()
        self._start_time = datetime.now()
        self._logger.info("Statistics reset")
    
    def export_json(self, output_path: str):
        """
        Export statistics to JSON file.
        
        Args:
            output_path: Path to output JSON file
        """
        stats = self.get_statistics()
        stats.end_time = datetime.now()
        
        output_file = Path(output_path)
        output_file.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_file, 'w') as f:
            json.dump(stats.to_dict(), f, indent=2)
        
        self._logger.info(f"Exported metrics to {output_path}")
    
    def export_csv(self, output_path: str):
        """
        Export statistics to CSV file.
        
        Args:
            output_path: Path to output CSV file
        """
        stats = self.get_statistics()
        stats.end_time = datetime.now()
        
        output_file = Path(output_path)
        output_file.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            
            # Header
            writer.writerow(['Metric', 'Value'])
            
            # Input metrics
            writer.writerow(['Input: Total Packets', stats.input.total_packets])
            writer.writerow(['Input: Total Bytes', stats.input.total_bytes])
            writer.writerow(['Input: Total Neurons', stats.input.total_neurons])
            writer.writerow(['Input: Avg Packet Size (bytes)', f"{stats.input.avg_packet_size:.2f}"])
            writer.writerow(['Input: Avg Neurons/Packet', f"{stats.input.avg_neurons_per_packet:.2f}"])
            writer.writerow(['Input: Data Rate (MB/s)', f"{stats.input.data_rate_mbps:.2f}"])
            writer.writerow(['Input: Packets/sec', f"{stats.input.packets_per_sec:.2f}"])
            writer.writerow(['Input: Avg Duration (ms)', f"{stats.input.avg_duration_ms:.2f}"])
            
            # Output metrics
            writer.writerow(['Output: Total Receives', stats.output.total_receives])
            writer.writerow(['Output: Total Commands', stats.output.total_commands])
            writer.writerow(['Output: Avg Commands/Receive', f"{stats.output.avg_commands_per_receive:.2f}"])
            writer.writerow(['Output: Avg Latency (ms)', f"{stats.output.avg_latency_ms:.2f}"])
            writer.writerow(['Output: Commands/sec', f"{stats.output.commands_per_sec:.2f}"])
            
            # Overall
            writer.writerow(['Uptime (seconds)', f"{stats.uptime_seconds:.2f}"])
        
        self._logger.info(f"Exported metrics to {output_path}")
    
    def print_summary(self):
        """Print a summary of collected metrics to console."""
        stats = self.get_statistics()
        
        print("\n" + "=" * 60)
        print("FEAGI Agent Metrics Summary")
        print("=" * 60)
        
        print(f"\n📊 Uptime: {stats.uptime_seconds:.2f} seconds")
        
        print(f"\n📥 Sensory Input:")
        print(f"  Total packets sent:     {stats.input.total_packets}")
        print(f"  Total bytes sent:       {stats.input.total_bytes:,}")
        print(f"  Total neurons sent:     {stats.input.total_neurons:,}")
        print(f"  Avg packet size:        {stats.input.avg_packet_size:.2f} bytes")
        print(f"  Avg neurons/packet:     {stats.input.avg_neurons_per_packet:.2f}")
        print(f"  Data rate:              {stats.input.data_rate_mbps:.2f} MB/s")
        print(f"  Packets/sec:            {stats.input.packets_per_sec:.2f}")
        print(f"  Avg send duration:      {stats.input.avg_duration_ms:.2f} ms")
        
        print(f"\n📤 Motor Output:")
        print(f"  Total receives:         {stats.output.total_receives}")
        print(f"  Total commands:         {stats.output.total_commands}")
        print(f"  Avg commands/receive:   {stats.output.avg_commands_per_receive:.2f}")
        print(f"  Avg latency:            {stats.output.avg_latency_ms:.2f} ms")
        print(f"  Commands/sec:           {stats.output.commands_per_sec:.2f}")
        
        print("\n" + "=" * 60 + "\n")

