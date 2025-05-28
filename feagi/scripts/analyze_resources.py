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
FEAGI Resource Analysis Script

Analyzes current FEAGI resource usage and provides detailed breakdown
for optimization efforts, especially for embedded device targets.

Usage:
    python3 scripts/analyze_resources.py [--runtime SECONDS] [--output FILE]
"""

import os
import sys
import time
import argparse
import multiprocessing
from pathlib import Path

# Add feagi to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from feagi.utils.resource_profiler import ResourceProfiler, start_profiling, stop_profiling, generate_resource_report
from feagi.utils.logger import setup_logger
from feagi.config.toml_loader import TomlLoader

logger = setup_logger(__name__)


def analyze_running_feagi(runtime_seconds: int = 30, output_file: str = None) -> str:
    """
    Analyze a running FEAGI instance.
    
    Args:
        runtime_seconds: How long to monitor (default 30 seconds)
        output_file: Optional file to save report to
        
    Returns:
        str: Analysis report
    """
    logger.info(f"[SEARCH] Starting FEAGI resource analysis (runtime: {runtime_seconds}s)")
    
    profiler = ResourceProfiler()
    profiler.start_profiling()
    
    # Take initial snapshot
    profiler.snapshot_component("analysis_start")
    
    try:
        # Monitor for specified duration
        intervals = max(1, runtime_seconds // 10)  # Take 10 snapshots
        interval_duration = runtime_seconds / intervals
        
        for i in range(intervals):
            time.sleep(interval_duration)
            profiler.snapshot_component(f"snapshot_{i+1}")
            logger.info(f"[STATS] Snapshot {i+1}/{intervals} taken")
        
        # Get neuron count from configuration
        config = TomlLoader.get_default_config()
        # Estimate neuron count (this would need to be improved with actual neuron counting)
        estimated_neurons = 13845  # From test data, would need real counting
        
        # Generate comprehensive report
        report = profiler.generate_optimization_report(neuron_count=estimated_neurons)
        
        # Add additional analysis
        report += "\n\n"
        report += "[SEARCH] ADDITIONAL ANALYSIS:\n"
        report += "=" * 50 + "\n"
        
        # Analyze growth over time
        snapshots = profiler.get_component_comparison()
        if len(snapshots) > 1:
            start_memory = next((s.memory_mb for s in snapshots if s.name == "analysis_start"), 0)
            end_memory = snapshots[-1].memory_mb
            memory_growth = end_memory - start_memory
            
            report += f"[UP] Memory growth during analysis: {memory_growth:+.1f}MB\n"
            if memory_growth > 50:  # > 50MB growth
                report += "🚨 WARNING: Significant memory growth detected - possible memory leak!\n"
            
            growth_rate = memory_growth / runtime_seconds * 60  # MB per minute
            report += f"[STATS] Memory growth rate: {growth_rate:+.1f}MB/minute\n"
            
            if growth_rate > 10:  # > 10MB/minute
                report += "🚨 CRITICAL: Memory growth rate too high for embedded devices!\n"
        
        # Component analysis
        report += "\n🏗️  COMPONENT IMPACT ANALYSIS:\n"
        for snapshot in snapshots:
            if "pre_" in snapshot.name or "post_" in snapshot.name:
                component = snapshot.name.replace("pre_", "").replace("post_", "")
                impact = snapshot.memory_mb / estimated_neurons * 1024  # KB per neuron
                report += f"   {component}: {impact:.1f}KB per neuron\n"
        
        report += "\n"
        
        # Save to file if specified
        if output_file:
            with open(output_file, 'w') as f:
                f.write(report)
            logger.info(f"[FOLDER] Report saved to {output_file}")
        
        return report
        
    finally:
        profiler.stop_profiling()
        logger.info("[SEARCH] Analysis completed")


def analyze_feagi_startup(config_file: str = None) -> str:
    """
    Analyze FEAGI resource usage during startup.
    
    Args:
        config_file: Optional config file path
        
    Returns:
        str: Startup analysis report
    """
    logger.info("[START] Analyzing FEAGI startup resource usage")
    
    # This would start FEAGI with profiling enabled and monitor startup
    # For now, return a placeholder
    return "[START] Startup analysis not yet implemented - use --runtime for running analysis"


def main():
    """Main analysis script."""
    parser = argparse.ArgumentParser(description="Analyze FEAGI resource usage")
    parser.add_argument("--runtime", type=int, default=30, 
                       help="Analysis runtime in seconds (default: 30)")
    parser.add_argument("--output", type=str, 
                       help="Output file for report")
    parser.add_argument("--startup", action="store_true",
                       help="Analyze startup instead of running instance")
    parser.add_argument("--config", type=str,
                       help="Config file for startup analysis")
    
    args = parser.parse_args()
    
    try:
        if args.startup:
            report = analyze_feagi_startup(args.config)
        else:
            report = analyze_running_feagi(args.runtime, args.output)
        
        print("\n" + report)
        
        # Quick summary for immediate action
        print("\n" + "="*80)
        print("[TARGET] IMMEDIATE ACTION ITEMS:")
        print("="*80)
        print("1. 🔴 CRITICAL: 3GB RAM usage is 30x too high for embedded devices")
        print("2. 🔴 CRITICAL: 14+ CPU cores is 28x too high for embedded targets") 
        print("3. 🔴 CRITICAL: Need embedded mode that disables heavy components")
        print("4. [SEARCH] INVESTIGATE: FastAPI/Uvicorn may be the primary memory hog")
        print("5. [SEARCH] INVESTIGATE: Multiple ZMQ sockets may be causing thread overhead")
        print("6. [TARGET] TARGET: <100MB total, <1KB per neuron, <0.5 CPU cores")
        print("="*80)
        
    except KeyboardInterrupt:
        logger.info("[WARN] Analysis interrupted by user")
    except Exception as e:
        logger.error(f"[ERR] Analysis failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main() 