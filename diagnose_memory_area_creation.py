#!/usr/bin/env python3
"""
Memory Area Creation Diagnostic Script

This script monitors FEAGI logs in real-time while creating a memory area
to identify exactly where the creation process is failing.

Usage:
1. Start FEAGI: python3 feagi/main.py --debug-npu --log-level DEBUG
2. In another terminal: python3 diagnose_memory_area_creation.py
"""

import sys
import time
import subprocess
import threading
import requests
import json
from datetime import datetime

BASE_URL = "http://127.0.0.1:8000"

class LogMonitor:
    def __init__(self):
        self.logs_captured = []
        self.monitoring = False

    def start_monitoring(self):
        """Start monitoring logs in a separate thread."""
        self.monitoring = True
        self.logs_captured = []
        
        def monitor_logs():
            try:
                # Monitor the latest log files
                cmd = "tail -f feagi/logs/latest_run/*.log 2>/dev/null"
                process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, 
                                         stderr=subprocess.PIPE, text=True, bufsize=1)
                
                while self.monitoring:
                    line = process.stdout.readline()
                    if line:
                        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        self.logs_captured.append(f"[{timestamp}] {line.strip()}")
                    elif process.poll() is not None:
                        break
                        
                process.terminate()
            except Exception as e:
                print(f"Log monitoring error: {e}")
        
        self.monitor_thread = threading.Thread(target=monitor_logs, daemon=True)
        self.monitor_thread.start()
        time.sleep(0.5)  # Give monitor time to start
    
    def stop_monitoring(self):
        """Stop monitoring logs."""
        self.monitoring = False
        if hasattr(self, 'monitor_thread'):
            self.monitor_thread.join(timeout=1)
    
    def get_relevant_logs(self, keywords=None):
        """Get logs containing specific keywords."""
        if not keywords:
            keywords = ["Creating memory", "✅ Created cortical", "✅ Registered memory", 
                       "Error creating", "rollback", "Exception", "Failed"]
        
        relevant = []
        for log in self.logs_captured:
            if any(keyword.lower() in log.lower() for keyword in keywords):
                relevant.append(log)
        return relevant

def create_memory_area_with_monitoring():
    """Create a memory area while monitoring logs."""
    
    print("🔍 Starting Memory Area Creation Diagnostic")
    print("=" * 60)
    
    # Start log monitoring
    monitor = LogMonitor()
    print("📋 Starting log monitoring...")
    monitor.start_monitoring()
    
    try:
        # Wait a moment for log monitoring to start
        time.sleep(1)
        
        print("🧠 Creating memory cortical area...")
        
        # Create memory area payload
        payload = {
            "brain_region_id": "root",
            "coordinates_2d": [70, 70],
            "coordinates_3d": [70, 70, 0],
            "cortical_dimensions": [2, 2, 1],  # This will be overridden to [1,1,1]
            "cortical_group": "CUSTOM",
            "cortical_name": "DiagnosticMemory",
            "cortical_sub_group": "MEMORY",
            "sub_group_id": "MEMORY",  # This makes it a memory area
            "temporal_depth": 2,
            "init_lifespan": 15,
            "lifespan_growth_rate": 1.1,
            "longterm_mem_threshold": 50,
            "per_voxel_neuron_cnt": 1,
            "visualization": True
        }
        
        # Make API call
        response = requests.post(f"{BASE_URL}/v1/cortical_area/custom_cortical_area", json=payload)
        
        print(f"📡 API Response: {response.status_code}")
        if response.status_code == 200:
            result = response.json()
            cortical_id = result.get("cortical_id")
            print(f"✅ API returned cortical_id: {cortical_id}")
        else:
            print(f"❌ API Error: {response.text}")
            return None
        
        # Wait for creation process to complete
        print("⏳ Waiting for creation process logs...")
        time.sleep(3)
        
        # Check if area exists
        print(f"🔍 Checking if {cortical_id} exists...")
        check_response = requests.get(f"{BASE_URL}/v1/cortical_area/{cortical_id}/properties")
        
        if check_response.status_code == 200:
            print(f"✅ Memory area {cortical_id} exists and accessible")
        else:
            print(f"❌ Memory area {cortical_id} disappeared (404)")
        
        return cortical_id
        
    finally:
        # Stop monitoring and analyze logs
        print("📋 Stopping log monitoring...")
        monitor.stop_monitoring()
        
        print("\n" + "=" * 60)
        print("📋 RELEVANT LOGS CAPTURED:")
        print("=" * 60)
        
        relevant_logs = monitor.get_relevant_logs()
        if relevant_logs:
            for log in relevant_logs:
                print(log)
        else:
            print("⚠️  No relevant logs captured - this indicates a deeper issue")
        
        print("\n" + "=" * 60)
        print("🔍 DIAGNOSTIC SUMMARY:")
        print("=" * 60)
        
        # Analyze the logs
        has_creation_start = any("Creating memory" in log for log in relevant_logs)
        has_creation_success = any("✅ Created cortical" in log for log in relevant_logs)
        has_memory_registration = any("✅ Registered memory" in log for log in relevant_logs)
        has_errors = any(keyword in log.lower() for log in relevant_logs 
                        for keyword in ["error", "exception", "failed", "rollback"])
        
        print(f"Memory creation started: {has_creation_start}")
        print(f"Cortical area created: {has_creation_success}")
        print(f"Memory registration: {has_memory_registration}")
        print(f"Errors detected: {has_errors}")
        
        if has_errors:
            print("\n❌ ERRORS FOUND - Memory area creation is failing")
            print("Check the logs above for specific error messages")
        elif has_creation_start and has_creation_success and has_memory_registration:
            print("\n✅ CREATION SUCCESSFUL - But area disappeared after")
            print("This suggests a post-creation deletion or validation issue")
        elif has_creation_start and not has_creation_success:
            print("\n❌ CREATION FAILED - Error during cortical area creation")
        else:
            print("\n❓ INCONCLUSIVE - No clear creation logs found")

def main():
    """Main diagnostic workflow."""
    
    # Check if FEAGI is running
    try:
        health_response = requests.get(f"{BASE_URL}/v1/system/health_check", timeout=2)
        if health_response.status_code != 200:
            print("❌ FEAGI health check failed")
            return False
    except Exception:
        print("❌ FEAGI is not running or not accessible")
        print("Please start FEAGI first: python3 feagi/main.py --debug-npu --log-level DEBUG")
        return False
    
    print("✅ FEAGI is running")
    
    # Run the diagnostic
    cortical_id = create_memory_area_with_monitoring()
    
    print(f"\n🎯 CONCLUSION:")
    if cortical_id:
        print(f"Created area {cortical_id} but need to check logs to see why it disappeared")
    else:
        print("Failed to create memory area - check logs for specific errors")
    
    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1) 