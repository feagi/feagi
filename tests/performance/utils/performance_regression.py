#!/usr/bin/env python3
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
Performance Regression Testing Framework

This module provides tools for detecting performance regressions in FEAGI
by comparing current performance metrics against historical baselines.

Features:
1. Baseline Management - Store and retrieve performance baselines
2. Regression Detection - Identify performance degradations
3. Trend Analysis - Track performance trends over time
4. Alert System - Generate alerts for significant regressions
5. Reporting - Generate regression analysis reports
"""

import json
import statistics
from dataclasses import dataclass, asdict
from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


@dataclass
class PerformanceBaseline:
    """Performance baseline for regression testing."""
    test_name: str
    metric_name: str
    baseline_value: float
    baseline_timestamp: float
    sample_count: int
    std_deviation: float
    confidence_interval: Tuple[float, float]
    system_info: Dict[str, Any]


@dataclass
class RegressionAlert:
    """Performance regression alert."""
    test_name: str
    metric_name: str
    current_value: float
    baseline_value: float
    regression_percent: float
    severity: str  # 'minor', 'moderate', 'severe', 'critical'
    timestamp: float
    description: str


@dataclass
class RegressionAnalysis:
    """Complete regression analysis result."""
    test_name: str
    total_metrics_tested: int
    regressions_detected: int
    improvements_detected: int
    alerts: List[RegressionAlert]
    trend_analysis: Dict[str, Any]
    recommendations: List[str]
    timestamp: float


class PerformanceRegressionDetector:
    """Performance regression detection and analysis."""
    
    def __init__(self, baselines_dir: Optional[Path] = None):
        """Initialize regression detector."""
        self.logger = setup_logger(__name__)
        self.baselines_dir = baselines_dir or Path("tests/performance/baselines")
        self.baselines_dir.mkdir(parents=True, exist_ok=True)
        
        # Regression thresholds (percentage degradation)
        self.regression_thresholds = {
            'minor': 5.0,      # 5% degradation
            'moderate': 15.0,  # 15% degradation
            'severe': 30.0,    # 30% degradation
            'critical': 50.0   # 50% degradation
        }
        
        # Metrics that should decrease (lower is better)
        self.lower_is_better_metrics = {
            'execution_time_ms',
            'latency_p50_ms',
            'latency_p95_ms',
            'latency_p99_ms',
            'memory_usage_mb',
            'memory_peak_mb',
            'burst_processing_time_ms',
            'fcl_processing_time_ms'
        }
        
        # Metrics that should increase (higher is better)
        self.higher_is_better_metrics = {
            'throughput_neurons_per_sec',
            'burst_frequency_hz'
        }
        
        self.logger.info("Performance regression detector initialized")
    
    def create_baseline(self, test_name: str, metrics: List[Dict[str, Any]], 
                       system_info: Dict[str, Any]) -> List[PerformanceBaseline]:
        """Create performance baselines from metrics."""
        baselines = []
        
        # Group metrics by metric name
        metrics_by_name = {}
        for metric in metrics:
            for key, value in metric.items():
                if isinstance(value, (int, float)) and key != 'timestamp':
                    if key not in metrics_by_name:
                        metrics_by_name[key] = []
                    metrics_by_name[key].append(value)
        
        # Create baseline for each metric
        for metric_name, values in metrics_by_name.items():
            if len(values) < 3:  # Need at least 3 samples for meaningful baseline
                continue
            
            baseline_value = statistics.mean(values)
            std_dev = statistics.stdev(values) if len(values) > 1 else 0
            
            # Calculate 95% confidence interval
            margin = 1.96 * (std_dev / (len(values) ** 0.5))  # 95% CI
            confidence_interval = (baseline_value - margin, baseline_value + margin)
            
            baseline = PerformanceBaseline(
                test_name=test_name,
                metric_name=metric_name,
                baseline_value=baseline_value,
                baseline_timestamp=datetime.now().timestamp(),
                sample_count=len(values),
                std_deviation=std_dev,
                confidence_interval=confidence_interval,
                system_info=system_info
            )
            
            baselines.append(baseline)
        
        # Save baselines
        self._save_baselines(baselines)
        
        self.logger.info(f"Created {len(baselines)} baselines for test: {test_name}")
        return baselines
    
    def _save_baselines(self, baselines: List[PerformanceBaseline]):
        """Save baselines to disk."""
        for baseline in baselines:
            filename = f"{baseline.test_name}_{baseline.metric_name}_baseline.json"
            filepath = self.baselines_dir / filename
            
            with open(filepath, 'w') as f:
                json.dump(asdict(baseline), f, indent=2)
    
    def _load_baseline(self, test_name: str, metric_name: str) -> Optional[PerformanceBaseline]:
        """Load baseline from disk."""
        filename = f"{test_name}_{metric_name}_baseline.json"
        filepath = self.baselines_dir / filename
        
        if not filepath.exists():
            return None
        
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            return PerformanceBaseline(**data)
        except Exception as e:
            self.logger.warning(f"Failed to load baseline {filepath}: {e}")
            return None
    
    def detect_regressions(self, test_name: str, current_metrics: List[Dict[str, Any]]) -> RegressionAnalysis:
        """Detect performance regressions by comparing with baselines."""
        self.logger.info(f"Detecting regressions for test: {test_name}")
        
        alerts = []
        total_metrics = 0
        regressions = 0
        improvements = 0
        
        # Group current metrics by metric name
        current_metrics_by_name = {}
        for metric in current_metrics:
            for key, value in metric.items():
                if isinstance(value, (int, float)) and key != 'timestamp':
                    if key not in current_metrics_by_name:
                        current_metrics_by_name[key] = []
                    current_metrics_by_name[key].append(value)
        
        # Compare each metric against baseline
        for metric_name, values in current_metrics_by_name.items():
            baseline = self._load_baseline(test_name, metric_name)
            if not baseline:
                continue
            
            total_metrics += 1
            current_value = statistics.mean(values)
            baseline_value = baseline.baseline_value
            
            # Calculate percentage change
            if baseline_value != 0:
                percent_change = ((current_value - baseline_value) / baseline_value) * 100
            else:
                percent_change = 0
            
            # Determine if this is a regression or improvement
            is_regression = False
            is_improvement = False
            
            if metric_name in self.lower_is_better_metrics:
                # For metrics where lower is better
                if percent_change > 0:  # Current value is higher (worse)
                    is_regression = True
                elif percent_change < -5:  # Current value is significantly lower (better)
                    is_improvement = True
            elif metric_name in self.higher_is_better_metrics:
                # For metrics where higher is better
                if percent_change < 0:  # Current value is lower (worse)
                    is_regression = True
                    percent_change = abs(percent_change)  # Make positive for severity calculation
                elif percent_change > 5:  # Current value is significantly higher (better)
                    is_improvement = True
            
            if is_regression:
                regressions += 1
                severity = self._calculate_severity(abs(percent_change))
                
                alert = RegressionAlert(
                    test_name=test_name,
                    metric_name=metric_name,
                    current_value=current_value,
                    baseline_value=baseline_value,
                    regression_percent=abs(percent_change),
                    severity=severity,
                    timestamp=datetime.now().timestamp(),
                    description=f"{metric_name} regressed by {abs(percent_change):.1f}%"
                )
                alerts.append(alert)
            elif is_improvement:
                improvements += 1
        
        # Generate trend analysis
        trend_analysis = self._analyze_trends(test_name, current_metrics_by_name)
        
        # Generate recommendations
        recommendations = self._generate_regression_recommendations(alerts)
        
        analysis = RegressionAnalysis(
            test_name=test_name,
            total_metrics_tested=total_metrics,
            regressions_detected=regressions,
            improvements_detected=improvements,
            alerts=alerts,
            trend_analysis=trend_analysis,
            recommendations=recommendations,
            timestamp=datetime.now().timestamp()
        )
        
        self.logger.info(f"Regression analysis complete: {regressions} regressions, {improvements} improvements")
        return analysis
    
    def _calculate_severity(self, regression_percent: float) -> str:
        """Calculate regression severity based on percentage."""
        if regression_percent >= self.regression_thresholds['critical']:
            return 'critical'
        elif regression_percent >= self.regression_thresholds['severe']:
            return 'severe'
        elif regression_percent >= self.regression_thresholds['moderate']:
            return 'moderate'
        elif regression_percent >= self.regression_thresholds['minor']:
            return 'minor'
        else:
            return 'negligible'
    
    def _analyze_trends(self, test_name: str, current_metrics: Dict[str, List[float]]) -> Dict[str, Any]:
        """Analyze performance trends over time."""
        # This would analyze historical data to identify trends
        # For now, return basic analysis
        return {
            'metrics_count': len(current_metrics),
            'analysis_timestamp': datetime.now().timestamp(),
            'trend_direction': 'stable'  # Would be calculated from historical data
        }
    
    def _generate_regression_recommendations(self, alerts: List[RegressionAlert]) -> List[str]:
        """Generate recommendations based on regression alerts."""
        recommendations = []
        
        if not alerts:
            recommendations.append("No performance regressions detected - maintain current optimization level")
            return recommendations
        
        # Analyze alert patterns
        critical_alerts = [a for a in alerts if a.severity == 'critical']
        severe_alerts = [a for a in alerts if a.severity == 'severe']
        
        if critical_alerts:
            recommendations.append("URGENT: Critical performance regressions detected - immediate investigation required")
            for alert in critical_alerts:
                recommendations.append(f"  - {alert.metric_name}: {alert.regression_percent:.1f}% degradation")
        
        if severe_alerts:
            recommendations.append("Severe performance regressions detected - prioritize optimization")
            for alert in severe_alerts:
                recommendations.append(f"  - {alert.metric_name}: {alert.regression_percent:.1f}% degradation")
        
        # Metric-specific recommendations
        memory_alerts = [a for a in alerts if 'memory' in a.metric_name.lower()]
        if memory_alerts:
            recommendations.append("Memory usage regressions detected - review memory allocation patterns")
        
        latency_alerts = [a for a in alerts if 'latency' in a.metric_name.lower()]
        if latency_alerts:
            recommendations.append("Latency regressions detected - profile critical path performance")
        
        throughput_alerts = [a for a in alerts if 'throughput' in a.metric_name.lower()]
        if throughput_alerts:
            recommendations.append("Throughput regressions detected - review parallel processing efficiency")
        
        return recommendations
    
    def save_regression_analysis(self, analysis: RegressionAnalysis) -> str:
        """Save regression analysis to disk."""
        timestamp = int(analysis.timestamp)
        filename = f"regression_analysis_{analysis.test_name}_{timestamp}.json"
        filepath = self.baselines_dir.parent / "logs" / filename
        filepath.parent.mkdir(parents=True, exist_ok=True)
        
        # Convert analysis to dictionary
        analysis_dict = {
            'test_name': analysis.test_name,
            'total_metrics_tested': analysis.total_metrics_tested,
            'regressions_detected': analysis.regressions_detected,
            'improvements_detected': analysis.improvements_detected,
            'alerts': [asdict(alert) for alert in analysis.alerts],
            'trend_analysis': analysis.trend_analysis,
            'recommendations': analysis.recommendations,
            'timestamp': analysis.timestamp
        }
        
        with open(filepath, 'w') as f:
            json.dump(analysis_dict, f, indent=2)
        
        self.logger.info(f"Saved regression analysis to {filepath}")
        return str(filepath)
    
    def print_regression_report(self, analysis: RegressionAnalysis):
        """Print comprehensive regression analysis report."""
        print("\n" + "="*80)
        print("🔍 PERFORMANCE REGRESSION ANALYSIS REPORT")
        print("="*80)
        
        print(f"\n📊 Test: {analysis.test_name}")
        print(f"   Metrics Tested: {analysis.total_metrics_tested}")
        print(f"   Regressions: {analysis.regressions_detected}")
        print(f"   Improvements: {analysis.improvements_detected}")
        
        if analysis.alerts:
            print(f"\n🚨 REGRESSION ALERTS:")
            print("-" * 50)
            
            # Group alerts by severity
            alerts_by_severity = {}
            for alert in analysis.alerts:
                if alert.severity not in alerts_by_severity:
                    alerts_by_severity[alert.severity] = []
                alerts_by_severity[alert.severity].append(alert)
            
            # Print alerts by severity
            severity_order = ['critical', 'severe', 'moderate', 'minor']
            for severity in severity_order:
                if severity in alerts_by_severity:
                    severity_symbol = {
                        'critical': '🔴',
                        'severe': '🟠', 
                        'moderate': '🟡',
                        'minor': '🔵'
                    }.get(severity, '⚪')
                    
                    print(f"\n{severity_symbol} {severity.upper()} REGRESSIONS:")
                    for alert in alerts_by_severity[severity]:
                        print(f"   • {alert.metric_name}: {alert.regression_percent:.1f}% degradation")
                        print(f"     Current: {alert.current_value:.3f}, Baseline: {alert.baseline_value:.3f}")
        else:
            print(f"\n✅ No performance regressions detected!")
        
        if analysis.recommendations:
            print(f"\n💡 RECOMMENDATIONS:")
            print("-" * 30)
            for i, rec in enumerate(analysis.recommendations, 1):
                print(f"   {i}. {rec}")
        
        print("\n" + "="*80)


def create_performance_baseline(test_name: str, metrics: List[Dict[str, Any]], 
                              system_info: Dict[str, Any]) -> List[PerformanceBaseline]:
    """Convenience function to create performance baseline."""
    detector = PerformanceRegressionDetector()
    return detector.create_baseline(test_name, metrics, system_info)


def detect_performance_regressions(test_name: str, 
                                 current_metrics: List[Dict[str, Any]]) -> RegressionAnalysis:
    """Convenience function to detect performance regressions."""
    detector = PerformanceRegressionDetector()
    return detector.detect_regressions(test_name, current_metrics)
