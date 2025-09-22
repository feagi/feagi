#!/usr/bin/env python3
"""
Live Corruption Inspector for Running FEAGI Instance

Connects to a running FEAGI process to investigate cortical area isolation corruption.
This script is designed to run alongside (not replace) your existing FEAGI instance.

Usage:
    python live_corruption_inspector.py --neuron-id 22598 --area-id cS7aaa
"""

import os
import sys
import time
import requests
import json
from typing import Dict, List, Optional, Any
from dataclasses import dataclass

@dataclass
class CorruptionEvidence:
    """Evidence of cortical isolation corruption."""
    neuron_id: int
    cortical_area: str
    evidence_type: str
    details: Any
    timestamp: float
    severity: str  # 'critical', 'high', 'medium', 'low'

class LiveCorruptionInspector:
    """Inspect live FEAGI system for cortical isolation corruption."""
    
    def __init__(self, api_base="http://127.0.0.1:8000"):
        self.api_base = api_base
        self.evidence = []
        self.target_area = None
        self.target_neuron = None
        
    def set_target(self, area_id: str, neuron_id: int):
        """Set the target area and neuron to investigate."""
        self.target_area = area_id
        self.target_neuron = neuron_id
        print(f"🎯 Investigating: Area '{area_id}', Neuron {neuron_id}")
        
    def get_fcl_status(self) -> Dict[str, Any]:
        """Get current FCL status from running FEAGI."""
        try:
            response = requests.get(f"{self.api_base}/v1/burst_engine/fcl", timeout=5)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print(f"❌ Failed to get FCL status: {e}")
        return {}
        
    def monitor_activation_pattern(self, duration_seconds: int = 30) -> Dict[str, Any]:
        """Monitor activation patterns over time."""
        print(f"🔍 Monitoring FCL for {duration_seconds} seconds...")
        
        activation_history = []
        start_time = time.time()
        
        while time.time() - start_time < duration_seconds:
            fcl_data = self.get_fcl_status()
            if fcl_data:
                timestamp = time.time()
                
                # Check if our target neuron is active
                target_active = False
                if self.target_area in fcl_data.get('cortical_areas', {}):
                    area_neurons = fcl_data['cortical_areas'][self.target_area]
                    if self.target_neuron in area_neurons:
                        target_active = True
                        
                        # Record corruption evidence
                        evidence = CorruptionEvidence(
                            neuron_id=self.target_neuron,
                            cortical_area=self.target_area,
                            evidence_type="unexpected_activation",
                            details={
                                "timestep": fcl_data.get("timestep", "unknown"),
                                "co_active_areas": list(fcl_data.get('cortical_areas', {}).keys()),
                                "total_active_neurons": fcl_data.get("total_neurons", 0)
                            },
                            timestamp=timestamp,
                            severity="critical"
                        )
                        self.evidence.append(evidence)
                        
                        print(f"🚨 CORRUPTION DETECTED at timestep {fcl_data.get('timestep')}")
                        print(f"   Isolated neuron {self.target_neuron} is active in area {self.target_area}!")
                        print(f"   Co-active areas: {list(fcl_data.get('cortical_areas', {}).keys())}")
                
                # Record the data point
                activation_history.append({
                    'timestamp': timestamp,
                    'timestep': fcl_data.get('timestep'),
                    'target_active': target_active,
                    'total_active': fcl_data.get('total_neurons', 0),
                    'active_areas': list(fcl_data.get('cortical_areas', {}).keys())
                })
                
                # Progress indicator
                if len(activation_history) % 5 == 0:
                    elapsed = time.time() - start_time
                    remaining = duration_seconds - elapsed
                    print(f"⏱️  Monitoring... {elapsed:.1f}s elapsed, {remaining:.1f}s remaining")
            
            time.sleep(1)  # Sample every second
            
        return {
            'duration': duration_seconds,
            'samples': len(activation_history),
            'corruption_events': len([h for h in activation_history if h['target_active']]),
            'history': activation_history
        }
        
    def analyze_co_activation_patterns(self, monitoring_data: Dict[str, Any]) -> Dict[str, Any]:
        """Analyze what other areas are active when corruption occurs."""
        print(f"🔍 Analyzing co-activation patterns...")
        
        corruption_events = [h for h in monitoring_data['history'] if h['target_active']]
        
        if not corruption_events:
            print("✅ No corruption events detected during monitoring period")
            return {'corruption_events': 0, 'patterns': {}}
            
        # Analyze co-active areas during corruption
        co_activation_counts = {}
        for event in corruption_events:
            for area in event['active_areas']:
                if area != self.target_area:  # Exclude the target area itself
                    co_activation_counts[area] = co_activation_counts.get(area, 0) + 1
                    
        # Sort by frequency
        sorted_coactivations = sorted(co_activation_counts.items(), key=lambda x: x[1], reverse=True)
        
        print(f"📊 CO-ACTIVATION ANALYSIS:")
        print(f"   Total corruption events: {len(corruption_events)}")
        print(f"   Co-active areas during corruption:")
        for area, count in sorted_coactivations[:10]:  # Top 10
            percentage = (count / len(corruption_events)) * 100
            print(f"     - {area}: {count}/{len(corruption_events)} events ({percentage:.1f}%)")
            
        return {
            'corruption_events': len(corruption_events),
            'co_activation_patterns': dict(sorted_coactivations),
            'most_likely_source': sorted_coactivations[0] if sorted_coactivations else None
        }
        
    def investigate_timing_correlation(self, monitoring_data: Dict[str, Any]) -> Dict[str, Any]:
        """Look for timing correlations that might indicate the source of corruption."""
        print(f"🔍 Investigating timing correlations...")
        
        history = monitoring_data['history']
        corruption_events = [h for h in history if h['target_active']]
        
        if len(corruption_events) < 2:
            print("⚠️  Need at least 2 corruption events for timing analysis")
            return {'timing_patterns': 'insufficient_data'}
            
        # Analyze timing patterns
        intervals = []
        for i in range(1, len(corruption_events)):
            interval = corruption_events[i]['timestamp'] - corruption_events[i-1]['timestamp']
            intervals.append(interval)
            
        if intervals:
            avg_interval = sum(intervals) / len(intervals)
            min_interval = min(intervals)
            max_interval = max(intervals)
            
            print(f"📊 TIMING ANALYSIS:")
            print(f"   Average interval between corruption events: {avg_interval:.2f}s")
            print(f"   Minimum interval: {min_interval:.2f}s")
            print(f"   Maximum interval: {max_interval:.2f}s")
            
            # Check for regular patterns
            if max_interval - min_interval < 1.0:
                print(f"🚨 REGULAR PATTERN DETECTED - suggests systematic corruption source!")
                
        return {
            'intervals': intervals,
            'avg_interval': avg_interval if intervals else 0,
            'pattern_regularity': 'regular' if intervals and (max(intervals) - min(intervals) < 1.0) else 'irregular'
        }
        
    def generate_corruption_report(self) -> str:
        """Generate comprehensive corruption report."""
        if not self.evidence:
            return "✅ No corruption evidence found during monitoring period."
            
        report_lines = [
            "🚨 CORTICAL ISOLATION CORRUPTION REPORT",
            "=" * 50,
            f"Target Area: {self.target_area}",
            f"Target Neuron: {self.target_neuron}",
            f"Evidence Count: {len(self.evidence)}",
            "",
            "CORRUPTION EVIDENCE:",
        ]
        
        for i, evidence in enumerate(self.evidence[:10], 1):  # Show first 10 pieces of evidence
            report_lines.extend([
                f"{i}. {evidence.evidence_type.upper()}",
                f"   Severity: {evidence.severity}",
                f"   Timestep: {evidence.details.get('timestep', 'unknown')}",
                f"   Co-active areas: {evidence.details.get('co_active_areas', [])}",
                f"   Total active neurons: {evidence.details.get('total_active_neurons', 0)}",
                ""
            ])
            
        if len(self.evidence) > 10:
            report_lines.append(f"... and {len(self.evidence) - 10} more evidence items")
            
        return "\n".join(report_lines)
        
    def run_comprehensive_investigation(self, monitor_duration: int = 30):
        """Run comprehensive corruption investigation."""
        print(f"🚀 Starting comprehensive corruption investigation...")
        print(f"   Target: {self.target_area} (neuron {self.target_neuron})")
        print(f"   Duration: {monitor_duration} seconds")
        print("")
        
        # Step 1: Initial FCL check
        print("📋 STEP 1: Initial FCL Status Check")
        initial_fcl = self.get_fcl_status()
        if initial_fcl:
            if self.target_area in initial_fcl.get('cortical_areas', {}):
                area_neurons = initial_fcl['cortical_areas'][self.target_area]
                if self.target_neuron in area_neurons:
                    print(f"🚨 IMMEDIATE CORRUPTION CONFIRMED!")
                    print(f"   Neuron {self.target_neuron} is currently active in isolated area {self.target_area}")
                else:
                    print(f"✅ Target neuron not currently active (may be intermittent)")
            else:
                print(f"❌ Target area {self.target_area} not found in current FCL")
        print("")
        
        # Step 2: Monitor activation patterns
        print("📋 STEP 2: Activation Pattern Monitoring")
        monitoring_data = self.monitor_activation_pattern(monitor_duration)
        print(f"   Completed: {monitoring_data['samples']} samples, {monitoring_data['corruption_events']} corruption events")
        print("")
        
        # Step 3: Analyze co-activation patterns
        print("📋 STEP 3: Co-activation Pattern Analysis")
        coactivation_analysis = self.analyze_co_activation_patterns(monitoring_data)
        print("")
        
        # Step 4: Timing correlation analysis
        print("📋 STEP 4: Timing Correlation Analysis") 
        timing_analysis = self.investigate_timing_correlation(monitoring_data)
        print("")
        
        # Step 5: Generate report
        print("📋 STEP 5: Final Report Generation")
        report = self.generate_corruption_report()
        print(report)
        
        # Recommendations
        print("\n🎯 RECOMMENDATIONS:")
        
        if coactivation_analysis['corruption_events'] > 0:
            most_likely_source = coactivation_analysis.get('most_likely_source')
            if most_likely_source:
                area, count = most_likely_source
                print(f"1. Investigate area '{area}' - co-active in {count} corruption events")
                print(f"2. Check for phantom synapses from '{area}' to '{self.target_area}'")
                
            if timing_analysis.get('pattern_regularity') == 'regular':
                print(f"3. SYSTEMATIC CORRUPTION - check burst processing logic")
                print(f"4. Investigate FCL injection during synaptic propagation")
            else:
                print(f"3. Intermittent corruption - check external input sources")
                
        else:
            print("1. Corruption may be very rare - run longer monitoring period")
            print("2. Check if area is truly isolated (no incoming connections)")
            
        return {
            'initial_status': initial_fcl,
            'monitoring_data': monitoring_data,
            'coactivation_analysis': coactivation_analysis,
            'timing_analysis': timing_analysis,
            'evidence_count': len(self.evidence)
        }

def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Live corruption inspector for running FEAGI')
    parser.add_argument('--area-id', default='cS7aaa', help='Cortical area ID')
    parser.add_argument('--neuron-id', type=int, default=22598, help='Neuron ID to monitor')
    parser.add_argument('--duration', type=int, default=30, help='Monitoring duration in seconds')
    parser.add_argument('--api-base', default='http://127.0.0.1:8000', help='FEAGI API base URL')
    
    args = parser.parse_args()
    
    inspector = LiveCorruptionInspector(args.api_base)
    inspector.set_target(args.area_id, args.neuron_id)
    
    try:
        results = inspector.run_comprehensive_investigation(args.duration)
        
        print(f"\n🏁 Investigation completed!")
        print(f"Evidence collected: {results['evidence_count']} items")
        
        if results['evidence_count'] > 0:
            sys.exit(2)  # Corruption confirmed
        else:
            sys.exit(0)  # No corruption detected
            
    except KeyboardInterrupt:
        print(f"\n⚠️  Investigation interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Investigation failed: {e}")
        sys.exit(3)

if __name__ == "__main__":
    main()
