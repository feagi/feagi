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
FEAGI Profiling Tests

Tests FEAGI performance with real genome data and records detailed
resource usage statistics for embedded device optimization.
"""

import pytest
import os
import sys
import json
import time
import subprocess
import psutil
from pathlib import Path
from datetime import datetime
import threading
from typing import Dict, Any, List

# Add feagi to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from feagi.utils.resource_profiler import ResourceProfiler, start_profiling, stop_profiling
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class FeagiProfileRunner:
    """Runs FEAGI with profiling and collects resource statistics."""
    
    def __init__(self, genome_path: str, runtime_seconds: int = 60):
        self.genome_path = genome_path
        self.runtime_seconds = runtime_seconds
        self.process = None
        self.profiler = ResourceProfiler()
        self.resource_snapshots = []
        self.monitoring_thread = None
        self.monitoring_active = False
        
    def start_feagi(self) -> bool:
        """Start FEAGI process with profiling enabled."""
        try:
            # Prepare environment
            env = os.environ.copy()
            env["FEAGI_SKIP_VERSION_CHECK"] = "1"
            
            # FEAGI command with profile mode and genome
            cmd = [
                sys.executable,
                "feagi/main.py",
                "--profile",
                "--genome", self.genome_path,
                "--test-duration", str(self.runtime_seconds)
            ]
            
            # Start FEAGI process
            self.process = subprocess.Popen(
                cmd,
                cwd=Path(__file__).parent.parent.parent,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            logger.info(f"[START] Started FEAGI process with PID {self.process.pid}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start FEAGI: {e}")
            return False
    
    def start_monitoring(self):
        """Start resource monitoring in a separate thread."""
        self.monitoring_active = True
        
        def monitor():
            start_time = time.time()
            snapshot_interval = 5.0  # Take snapshot every 5 seconds
            
            while self.monitoring_active and (time.time() - start_time) < self.runtime_seconds:
                try:
                    if self.process and self.process.poll() is None:
                        # Get process stats
                        proc = psutil.Process(self.process.pid)
                        
                        snapshot = {
                            'timestamp': datetime.now().isoformat(),
                            'elapsed_seconds': time.time() - start_time,
                            'memory_mb': proc.memory_info().rss / (1024 * 1024),
                            'memory_vms_mb': proc.memory_info().vms / (1024 * 1024),
                            'cpu_percent': proc.cpu_percent(interval=1),
                            'thread_count': proc.num_threads(),
                            'fd_count': proc.num_fds() if hasattr(proc, 'num_fds') else 0,
                            'connections': len(proc.connections()) if hasattr(proc, 'connections') else 0
                        }
                        
                        self.resource_snapshots.append(snapshot)
                        logger.info(f"[STATS] Snapshot {len(self.resource_snapshots)}: "
                                  f"{snapshot['memory_mb']:.1f}MB, {snapshot['cpu_percent']:.1f}% CPU")
                    
                    time.sleep(snapshot_interval)
                    
                except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
                    logger.warning(f"Process monitoring error: {e}")
                    break
                except Exception as e:
                    logger.error(f"Monitoring error: {e}")
                    break
        
        self.monitoring_thread = threading.Thread(target=monitor, daemon=True)
        self.monitoring_thread.start()
        logger.info("[STATS] Started resource monitoring")
    
    def stop_monitoring(self):
        """Stop resource monitoring."""
        self.monitoring_active = False
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=5)
        logger.info("[STATS] Stopped resource monitoring")
    
    def wait_for_completion(self) -> tuple[bool, str, str]:
        """Wait for FEAGI to complete and capture output."""
        try:
            stdout, stderr = self.process.communicate(timeout=self.runtime_seconds + 30)
            return_code = self.process.returncode
            
            success = return_code == 0
            logger.info(f"🏁 FEAGI completed with return code: {return_code}")
            
            return success, stdout, stderr
            
        except subprocess.TimeoutExpired:
            logger.warning("⏰ FEAGI process timed out, terminating...")
            self.process.terminate()
            try:
                stdout, stderr = self.process.communicate(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                stdout, stderr = self.process.communicate()
            
            return False, stdout, stderr
        except Exception as e:
            logger.error(f"Error waiting for completion: {e}")
            return False, "", str(e)
    
    def generate_report(self, genome_stats: Dict[str, Any]) -> Dict[str, Any]:
        """Generate comprehensive profiling report."""
        if not self.resource_snapshots:
            return {"error": "No resource snapshots collected"}
        
        # Calculate statistics
        memory_values = [s['memory_mb'] for s in self.resource_snapshots]
        cpu_values = [s['cpu_percent'] for s in self.resource_snapshots]
        thread_values = [s['thread_count'] for s in self.resource_snapshots]
        
        # Neuron count from genome
        neuron_count = genome_stats.get('innate_neuron_count', 0)
        
        # Memory per neuron calculations
        avg_memory = sum(memory_values) / len(memory_values)
        max_memory = max(memory_values)
        memory_per_neuron_kb = (avg_memory * 1024) / neuron_count if neuron_count > 0 else 0
        
        report = {
            'test_info': {
                'genome_file': os.path.basename(self.genome_path),
                'runtime_seconds': self.runtime_seconds,
                'snapshot_count': len(self.resource_snapshots),
                'test_timestamp': datetime.now().isoformat()
            },
            'genome_stats': genome_stats,
            'resource_usage': {
                'memory': {
                    'avg_mb': avg_memory,
                    'max_mb': max_memory,
                    'min_mb': min(memory_values),
                    'memory_per_neuron_kb': memory_per_neuron_kb
                },
                'cpu': {
                    'avg_percent': sum(cpu_values) / len(cpu_values),
                    'max_percent': max(cpu_values),
                    'min_percent': min(cpu_values)
                },
                'threads': {
                    'avg_count': sum(thread_values) / len(thread_values),
                    'max_count': max(thread_values),
                    'min_count': min(thread_values)
                }
            },
            'embedded_device_analysis': {
                'memory_over_target': avg_memory / 100,  # Target: 100MB
                'memory_per_neuron_over_target': memory_per_neuron_kb / 1,  # Target: 1KB/neuron
                'thread_over_target': (sum(thread_values) / len(thread_values)) / 5,  # Target: 5 threads
                'recommendations': self._generate_recommendations(avg_memory, memory_per_neuron_kb, 
                                                                sum(thread_values) / len(thread_values))
            },
            'raw_snapshots': self.resource_snapshots
        }
        
        return report
    
    def _generate_recommendations(self, avg_memory: float, memory_per_neuron: float, 
                                avg_threads: float) -> List[str]:
        """Generate optimization recommendations."""
        recommendations = []
        
        if avg_memory > 100:
            recommendations.append(f"🔴 Memory usage ({avg_memory:.1f}MB) exceeds embedded target (100MB)")
            
        if memory_per_neuron > 1:
            recommendations.append(f"🔴 Memory per neuron ({memory_per_neuron:.1f}KB) exceeds embedded target (1KB)")
            
        if avg_threads > 5:
            recommendations.append(f"🔴 Thread count ({avg_threads:.1f}) exceeds embedded target (5)")
            
        if avg_memory > 200:
            recommendations.append("🛠️ Consider implementing --embedded mode")
            
        if memory_per_neuron > 10:
            recommendations.append("🛠️ Implement memory-mapped neural data structures")
            
        if avg_threads > 10:
            recommendations.append("🛠️ Replace multi-threading with async/await patterns")
            
        return recommendations


@pytest.fixture
def genome_path():
    """Get path to the profile genome file."""
    return str(Path(__file__).parent / "profile_genome.json")


@pytest.fixture
def logs_dir():
    """Ensure logs directory exists."""
    logs_path = Path(__file__).parent / "logs"
    logs_path.mkdir(exist_ok=True)
    return logs_path


def test_profile_genome_performance(genome_path, logs_dir):
    """Test FEAGI performance with the profile genome for 1 minute."""
    
    # Verify genome file exists
    assert os.path.exists(genome_path), f"Genome file not found: {genome_path}"
    
    # Load genome stats
    with open(genome_path, 'r') as f:
        genome_data = json.load(f)
    
    genome_stats = genome_data.get('stats', {})
    logger.info(f"[STATS] Testing with genome: {genome_data.get('genome_title', 'Unknown')}")
    logger.info(f"[STATS] Neurons: {genome_stats.get('innate_neuron_count', 0)}")
    logger.info(f"[STATS] Cortical Areas: {genome_stats.get('innate_cortical_area_count', 0)}")
    
    # Create profiler runner
    runner = FeagiProfileRunner(genome_path, runtime_seconds=60)
    
    try:
        # Start FEAGI
        assert runner.start_feagi(), "Failed to start FEAGI process"
        
        # Start monitoring
        runner.start_monitoring()
        
        # Wait for completion
        success, stdout, stderr = runner.wait_for_completion()
        
        # Stop monitoring
        runner.stop_monitoring()
        
        # Generate report
        report = runner.generate_report(genome_stats)
        
        # Save results to logs
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_file = logs_dir / f"profile_report_{timestamp}.json"
        
        with open(report_file, 'w') as f:
            json.dump(report, indent=2, fp=f)
        
        logger.info(f"[FOLDER] Report saved to {report_file}")
        
        # Save stdout/stderr if they exist
        if stdout:
            stdout_file = logs_dir / f"feagi_stdout_{timestamp}.log"
            with open(stdout_file, 'w') as f:
                f.write(stdout)
        
        if stderr:
            stderr_file = logs_dir / f"feagi_stderr_{timestamp}.log"
            with open(stderr_file, 'w') as f:
                f.write(stderr)
        
        # Print summary
        if 'resource_usage' in report:
            memory_avg = report['resource_usage']['memory']['avg_mb']
            memory_per_neuron = report['resource_usage']['memory']['memory_per_neuron_kb']
            cpu_avg = report['resource_usage']['cpu']['avg_percent']
            
            logger.info(f"[STATS] PERFORMANCE SUMMARY:")
            logger.info(f"   Average Memory: {memory_avg:.1f}MB")
            logger.info(f"   Memory per Neuron: {memory_per_neuron:.1f}KB")
            logger.info(f"   Average CPU: {cpu_avg:.1f}%")
            
            # Assertions for basic sanity checks
            assert memory_avg > 0, "Memory usage should be greater than 0"
            assert memory_avg < 5000, f"Memory usage ({memory_avg:.1f}MB) is suspiciously high"
            assert memory_per_neuron > 0, "Memory per neuron should be greater than 0"
        
        # Process should have run successfully (or at least not crashed immediately)
        assert len(runner.resource_snapshots) > 0, "No resource snapshots were collected"
        
        logger.info("[OK] Profiling test completed successfully")
        
    except Exception as e:
        logger.error(f"[ERR] Profiling test failed: {e}")
        raise
    finally:
        # Cleanup
        if runner.process and runner.process.poll() is None:
            runner.process.terminate()
            runner.process.wait(timeout=10)


if __name__ == "__main__":
    # Allow running this test directly
    pytest.main([__file__, "-v", "-s"]) 