"""
Utility Functions

Helper functions for easy observability setup.
"""

import logging
from typing import Optional
from contextlib import contextmanager

from feagi.pns.observability.monitor import InputMonitor, OutputMonitor
from feagi.pns.observability.metrics import MetricsCollector
from feagi.pns.observability.logger import DataLogger


def enable_monitoring(
    brain_input: Optional[Any] = None,
    brain_output: Optional[Any] = None,
    collect_metrics: bool = True,
    log_file: Optional[str] = None,
    log_format: str = "jsonl",
    log_level: str = "INFO",
    show_dashboard: bool = False
) -> tuple:
    """
    Enable monitoring with a simple one-liner.
    
    Args:
        brain_input: BrainInput instance to monitor (auto-imported if None)
        brain_output: BrainOutput instance to monitor (auto-imported if None)
        collect_metrics: Whether to collect metrics
        log_file: Path to log file (None = no logging)
        log_format: Log format - "json", "jsonl", or "csv"
        log_level: Logging level
        show_dashboard: Whether to show live dashboard (not yet implemented)
    
    Returns:
        Tuple of (metrics_collector, data_logger) or (None, None)
    
    Example:
        from feagi.pns.observability import enable_monitoring
        
        metrics, logger = enable_monitoring(
            log_file="agent_monitor.log",
            log_level="INFO"
        )
        
        # ... run agent ...
        
        metrics.print_summary()
        logger.close()
    """
    # Auto-import if not provided
    if brain_input is None or brain_output is None:
        try:
            from feagi.pns import brain_input as bi, brain_output as bo
            brain_input = brain_input or bi
            brain_output = brain_output or bo
        except ImportError:
            logging.error("Failed to import brain_input/brain_output")
            return None, None
    
    metrics_collector = None
    data_logger = None
    
    # Set up metrics collection
    if collect_metrics:
        metrics_collector = MetricsCollector(log_level=log_level)
        if brain_input:
            brain_input.attach_monitor(metrics_collector)
        if brain_output:
            brain_output.attach_monitor(metrics_collector)
        logging.info("✅ Metrics collection enabled")
    
    # Set up data logging
    if log_file:
        data_logger = DataLogger(
            output_file=log_file,
            format=log_format,
            log_inputs=True,
            log_outputs=True,
            log_level=log_level
        )
        if brain_input:
            brain_input.attach_monitor(data_logger)
        if brain_output:
            brain_output.attach_monitor(data_logger)
        logging.info(f"✅ Data logging enabled: {log_file}")
    
    # Show dashboard (future feature)
    if show_dashboard:
        logging.warning("Dashboard not yet implemented")
    
    return metrics_collector, data_logger


@contextmanager
def monitor_session(
    brain_input: Optional[Any] = None,
    brain_output: Optional[Any] = None,
    collect_metrics: bool = True,
    log_file: Optional[str] = None,
    log_format: str = "jsonl",
    export_on_exit: bool = True
):
    """
    Context manager for temporary monitoring session.
    
    Automatically exports metrics and closes loggers on exit.
    
    Args:
        brain_input: BrainInput instance to monitor
        brain_output: BrainOutput instance to monitor
        collect_metrics: Whether to collect metrics
        log_file: Path to log file
        log_format: Log format
        export_on_exit: Whether to export metrics on exit
    
    Yields:
        Session object with metrics and logger
    
    Example:
        from feagi.pns.observability import monitor_session
        
        with monitor_session(log_file="debug_session.log") as session:
            # Run agent code
            for i in range(100):
                camera.set_frame(frame)
                brain_input.send()
                brain_output.receive()
            
            # Metrics automatically exported when context exits
        
        # Session data is saved
        print(f"Logged {session.packets} packets")
    """
    
    class MonitorSession:
        def __init__(self, metrics, logger):
            self.metrics = metrics
            self.logger = logger
            self.start_time = None
            self.end_time = None
        
        @property
        def packets(self) -> int:
            """Total packets processed."""
            if self.metrics:
                stats = self.metrics.get_statistics()
                return stats.input.total_packets + stats.output.total_receives
            return 0
        
        def get_statistics(self):
            """Get statistics from metrics collector."""
            if self.metrics:
                return self.metrics.get_statistics()
            return None
    
    # Set up monitoring
    metrics, logger = enable_monitoring(
        brain_input=brain_input,
        brain_output=brain_output,
        collect_metrics=collect_metrics,
        log_file=log_file,
        log_format=log_format
    )
    
    session = MonitorSession(metrics, logger)
    
    try:
        from datetime import datetime
        session.start_time = datetime.now()
        yield session
    finally:
        from datetime import datetime
        session.end_time = datetime.now()
        
        # Export metrics
        if export_on_exit and metrics:
            metrics.print_summary()
            if log_file:
                base_path = log_file.rsplit('.', 1)[0]
                metrics.export_json(f"{base_path}_metrics.json")
        
        # Close logger
        if logger:
            logger.close()
        
        logging.info(f"✅ Monitoring session complete: {session.packets} packets processed")

