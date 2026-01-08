"""
FEAGI PNS Observability Module

Provides monitoring, metrics, logging, and profiling tools for debugging
and optimizing FEAGI agents.

Core Components:
- Monitor: Base class for all observers
- MetricsCollector: Collect statistics on data flow
- DataLogger: Structured logging of packets
- DataInspector: Validate and detect issues
- Profiler: Performance profiling
- Dashboard: Live terminal UI

Quick Start:
    from feagi.pns.observability import enable_monitoring
    
    enable_monitoring(
        brain_input=True,
        brain_output=True,
        log_file="agent_monitor.log"
    )
"""

from feagi.pns.observability.monitor import Monitor, InputMonitor, OutputMonitor
from feagi.pns.observability.metrics import MetricsCollector, Statistics
from feagi.pns.observability.logger import DataLogger
from feagi.pns.observability.controller_logger import ControllerLogger, create_controller_logger
from feagi.pns.observability.inspector import DataInspector, ValidationReport
from feagi.pns.observability.profiler import Profiler, Profile
from feagi.pns.observability.formatters import (
    JSONFormatter,
    CSVFormatter,
    ConsoleFormatter
)
from feagi.pns.observability.utils import enable_monitoring, monitor_session

__all__ = [
    # Core classes
    "Monitor",
    "InputMonitor",
    "OutputMonitor",
    "MetricsCollector",
    "DataLogger",
    "ControllerLogger",
    "DataInspector",
    "Profiler",
    # Data classes
    "Statistics",
    "ValidationReport",
    "Profile",
    # Formatters
    "JSONFormatter",
    "CSVFormatter",
    "ConsoleFormatter",
    # Utilities
    "enable_monitoring",
    "monitor_session",
    "create_controller_logger",
]

