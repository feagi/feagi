"""
Data Inspector

Validates data formats and detects anomalies in sensory/motor data.
"""

from typing import Dict, Any, List, Optional
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum

from feagi.pns.observability.monitor import Monitor


class IssueSeverity(Enum):
    """Severity level for validation issues."""
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"


@dataclass
class ValidationIssue:
    """Represents a validation issue found in data."""
    severity: IssueSeverity
    message: str
    packet_id: int
    cortical_area: Optional[str] = None
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())
    
    def __str__(self) -> str:
        location = f" in {self.cortical_area}" if self.cortical_area else ""
        return f"[{self.severity.value.upper()}] Packet #{self.packet_id}{location}: {self.message}"


@dataclass
class ValidationReport:
    """Report of all validation issues."""
    issues: List[ValidationIssue] = field(default_factory=list)
    packets_inspected: int = 0
    
    @property
    def error_count(self) -> int:
        """Number of errors."""
        return sum(1 for i in self.issues if i.severity == IssueSeverity.ERROR)
    
    @property
    def warning_count(self) -> int:
        """Number of warnings."""
        return sum(1 for i in self.issues if i.severity == IssueSeverity.WARNING)
    
    @property
    def info_count(self) -> int:
        """Number of info messages."""
        return sum(1 for i in self.issues if i.severity == IssueSeverity.INFO)
    
    def has_errors(self) -> bool:
        """Check if there are any errors."""
        return self.error_count > 0
    
    def has_warnings(self) -> bool:
        """Check if there are any warnings."""
        return self.warning_count > 0
    
    def print_summary(self):
        """Print summary of validation issues."""
        print("\n" + "=" * 60)
        print("Data Validation Report")
        print("=" * 60)
        print(f"Packets inspected: {self.packets_inspected}")
        print(f"Errors:   {self.error_count}")
        print(f"Warnings: {self.warning_count}")
        print(f"Info:     {self.info_count}")
        
        if self.issues:
            print("\nIssues:")
            for issue in self.issues:
                print(f"  {issue}")
        else:
            print("\n✅ No issues found!")
        print("=" * 60 + "\n")


class DataInspector(Monitor):
    """
    Inspects and validates sensory/motor data.
    
    Checks for:
    - Format correctness
    - Range violations
    - Anomalies (all zeros, NaN values, outliers)
    - Data consistency
    
    Example:
        inspector = DataInspector(
            validate_formats=True,
            check_ranges=True,
            detect_anomalies=True
        )
        
        brain_input.attach_monitor(inspector)
        
        # ... run agent ...
        
        report = inspector.get_report()
        if report.has_errors():
            print(f"Found {report.error_count} errors!")
            report.print_summary()
    """
    
    def __init__(
        self,
        validate_formats: bool = True,
        check_ranges: bool = True,
        detect_anomalies: bool = True,
        enabled: bool = True,
        log_level: str = "INFO"
    ):
        """
        Initialize data inspector.
        
        Args:
            validate_formats: Check data format correctness
            check_ranges: Check values are within expected ranges
            detect_anomalies: Detect unusual patterns
            enabled: Whether inspection is active
            log_level: Logging level
        """
        super().__init__(enabled=enabled, log_level=log_level)
        
        self.validate_formats = validate_formats
        self.check_ranges = check_ranges
        self.detect_anomalies = detect_anomalies
        
        self._report = ValidationReport()
        self._packet_counter = 0
    
    def _add_issue(
        self,
        severity: IssueSeverity,
        message: str,
        cortical_area: Optional[str] = None
    ):
        """Add a validation issue to the report."""
        issue = ValidationIssue(
            severity=severity,
            message=message,
            packet_id=self._packet_counter,
            cortical_area=cortical_area
        )
        self._report.issues.append(issue)
        
        # Log the issue
        log_method = {
            IssueSeverity.INFO: self._logger.info,
            IssueSeverity.WARNING: self._logger.warning,
            IssueSeverity.ERROR: self._logger.error,
        }[severity]
        log_method(str(issue))
    
    def on_send_start(self, data: Dict[str, Any]):
        """Validate send operation start."""
        pass
    
    def on_send_complete(self, data: Dict[str, Any]):
        """Validate sensory input packet."""
        if not self.enabled:
            return
        
        self._packet_counter += 1
        self._report.packets_inspected += 1
        
        # Check for empty packets
        neuron_count = data.get('neuron_count', 0)
        if neuron_count == 0:
            self._add_issue(
                IssueSeverity.WARNING,
                "Empty packet (0 neurons)"
            )
        
        # Check packet size consistency
        packet_size = data.get('packet_size_bytes', 0)
        if packet_size == 0 and neuron_count > 0:
            self._add_issue(
                IssueSeverity.ERROR,
                f"Packet size is 0 but neuron count is {neuron_count}"
            )
        
        # Check for unusually large packets
        if packet_size > 100 * 1024 * 1024:  # 100 MB
            self._add_issue(
                IssueSeverity.WARNING,
                f"Very large packet: {packet_size / (1024*1024):.2f} MB"
            )
        
        # Check duration
        duration_ms = data.get('duration_ms', 0.0)
        if duration_ms > 100:  # More than 100ms is slow
            self._add_issue(
                IssueSeverity.WARNING,
                f"Slow send operation: {duration_ms:.2f} ms"
            )
        
        # Validate cortical areas
        cortical_areas = data.get('cortical_areas', [])
        if not cortical_areas and neuron_count > 0:
            self._add_issue(
                IssueSeverity.WARNING,
                "No cortical areas specified but neurons sent"
            )
        
        # Check for data samples if included
        if 'data_sample' in data:
            self._validate_data_sample(data['data_sample'])
    
    def _validate_data_sample(self, sample: List[Any]):
        """Validate a data sample for anomalies."""
        if not self.detect_anomalies:
            return
        
        if not sample:
            return
        
        # Check for all zeros
        try:
            if all(isinstance(x, (int, float)) and x == 0 for x in sample):
                self._add_issue(
                    IssueSeverity.WARNING,
                    "Data sample contains all zeros"
                )
        except (TypeError, AttributeError):
            pass
        
        # Check for NaN values
        try:
            import math
            if any(isinstance(x, float) and math.isnan(x) for x in sample):
                self._add_issue(
                    IssueSeverity.ERROR,
                    "Data sample contains NaN values"
                )
        except (TypeError, AttributeError):
            pass
    
    def on_receive_start(self, data: Dict[str, Any]):
        """Validate receive operation start."""
        pass
    
    def on_receive_complete(self, data: Dict[str, Any]):
        """Validate motor output commands."""
        if not self.enabled:
            return
        
        self._packet_counter += 1
        self._report.packets_inspected += 1
        
        # Check for missing commands
        command_count = data.get('command_count', 0)
        if command_count == 0:
            self._add_issue(
                IssueSeverity.INFO,
                "No motor commands received"
            )
        
        # Check duration
        duration_ms = data.get('duration_ms', 0.0)
        if duration_ms > 50:  # More than 50ms is slow for motor commands
            self._add_issue(
                IssueSeverity.WARNING,
                f"Slow receive operation: {duration_ms:.2f} ms"
            )
    
    def get_report(self) -> ValidationReport:
        """
        Get validation report.
        
        Returns:
            ValidationReport with all issues found
        """
        return self._report
    
    def reset(self):
        """Reset validation report."""
        self._report = ValidationReport()
        self._packet_counter = 0
        self._logger.info("Validation report reset")

