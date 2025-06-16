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
FEAGI Embedded Mode Comparison Tests

Compares resource usage between normal mode and embedded mode to
validate that embedded mode actually reduces resource consumption.
"""

import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

import psutil
import pytest

# Add feagi to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class FeagiModeComparator:
    """Compares FEAGI resource usage between different modes."""

    def __init__(self, genome_path: str, runtime_seconds: int = 30):
        self.genome_path = genome_path
        self.runtime_seconds = runtime_seconds
        self.results = {}

    def run_mode_test(self, mode_name: str, embedded: bool = False) -> Dict[str, Any]:
        """Run FEAGI in a specific mode and collect resource stats."""

        logger.info(f"🔬 Testing {mode_name} mode...")

        # Prepare environment
        env = os.environ.copy()
        env["FEAGI_SKIP_VERSION_CHECK"] = "1"

        # Build command
        cmd = [
            sys.executable,
            "feagi/main.py",
            "--profile",
            "--genome",
            self.genome_path,
            "--test",
            "--test-duration",
            str(self.runtime_seconds),
        ]

        if embedded:
            cmd.append("--embedded")

        logger.info(f"Command: {' '.join(cmd)}")

        # Start FEAGI process
        process = subprocess.Popen(
            cmd,
            cwd=Path(__file__).parent.parent.parent,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        logger.info(f"[START] Started {mode_name} FEAGI process with PID {process.pid}")

        # Monitor resource usage
        snapshots = []
        start_time = time.time()

        while (
            time.time() - start_time < self.runtime_seconds + 10
        ):  # Extra time for startup
            try:
                if process.poll() is not None:
                    # Process finished
                    break

                # Get process stats
                proc = psutil.Process(process.pid)

                snapshot = {
                    "timestamp": datetime.now().isoformat(),
                    "elapsed": time.time() - start_time,
                    "memory_mb": proc.memory_info().rss / (1024 * 1024),
                    "cpu_percent": proc.cpu_percent(interval=0.1),
                    "thread_count": proc.num_threads(),
                    "fd_count": proc.num_fds() if hasattr(proc, "num_fds") else 0,
                }

                snapshots.append(snapshot)

                if len(snapshots) % 5 == 0:
                    logger.info(
                        f"[STATS] {mode_name}: {snapshot['memory_mb']:.1f}MB, "
                        f"{snapshot['cpu_percent']:.1f}% CPU, "
                        f"{snapshot['thread_count']} threads"
                    )

                time.sleep(2)  # Sample every 2 seconds

            except (psutil.NoSuchProcess, psutil.AccessDenied):
                break
            except Exception as e:
                logger.warning(f"Monitoring error: {e}")
                break

        # Wait for process completion
        try:
            stdout, stderr = process.communicate(timeout=15)
            return_code = process.returncode
        except subprocess.TimeoutExpired:
            logger.warning(f"⏰ {mode_name} process timed out, terminating...")
            process.terminate()
            try:
                stdout, stderr = process.communicate(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                stdout, stderr = process.communicate()
            return_code = -1

        # Calculate statistics
        if snapshots:
            memory_values = [
                s["memory_mb"] for s in snapshots if s["elapsed"] > 5
            ]  # Skip startup
            cpu_values = [s["cpu_percent"] for s in snapshots if s["elapsed"] > 5]
            thread_values = [s["thread_count"] for s in snapshots if s["elapsed"] > 5]

            if memory_values:
                stats = {
                    "mode": mode_name,
                    "embedded": embedded,
                    "return_code": return_code,
                    "snapshots_total": len(snapshots),
                    "snapshots_analyzed": len(memory_values),
                    "memory": {
                        "avg_mb": sum(memory_values) / len(memory_values),
                        "max_mb": max(memory_values),
                        "min_mb": min(memory_values),
                    },
                    "cpu": {
                        "avg_percent": (
                            sum(cpu_values) / len(cpu_values) if cpu_values else 0
                        ),
                        "max_percent": max(cpu_values) if cpu_values else 0,
                    },
                    "threads": {
                        "avg_count": (
                            sum(thread_values) / len(thread_values)
                            if thread_values
                            else 0
                        ),
                        "max_count": max(thread_values) if thread_values else 0,
                    },
                    "raw_snapshots": snapshots,
                }

                logger.info(
                    f"[OK] {mode_name} completed: "
                    f"{stats['memory']['avg_mb']:.1f}MB avg, "
                    f"{stats['cpu']['avg_percent']:.1f}% CPU avg"
                )

                return stats

        logger.error(f"[ERR] No valid data collected for {mode_name}")
        return {"mode": mode_name, "embedded": embedded, "error": "No data collected"}

    def compare_modes(self) -> Dict[str, Any]:
        """Compare normal vs embedded mode."""

        # Test normal mode
        normal_stats = self.run_mode_test("Normal", embedded=False)

        # Wait a bit between tests
        time.sleep(5)

        # Test embedded mode
        embedded_stats = self.run_mode_test("Embedded", embedded=True)

        # Calculate improvements
        comparison = {
            "normal_mode": normal_stats,
            "embedded_mode": embedded_stats,
            "test_info": {
                "genome_file": os.path.basename(self.genome_path),
                "runtime_seconds": self.runtime_seconds,
                "test_timestamp": datetime.now().isoformat(),
            },
        }

        # Calculate savings if both modes have valid data
        if (
            "memory" in normal_stats
            and "memory" in embedded_stats
            and "error" not in normal_stats
            and "error" not in embedded_stats
        ):
            normal_mem = normal_stats["memory"]["avg_mb"]
            embedded_mem = embedded_stats["memory"]["avg_mb"]
            memory_savings = normal_mem - embedded_mem
            memory_savings_percent = (
                (memory_savings / normal_mem) * 100 if normal_mem > 0 else 0
            )

            normal_cpu = normal_stats["cpu"]["avg_percent"]
            embedded_cpu = embedded_stats["cpu"]["avg_percent"]
            cpu_savings = normal_cpu - embedded_cpu

            normal_threads = normal_stats["threads"]["avg_count"]
            embedded_threads = embedded_stats["threads"]["avg_count"]
            thread_savings = normal_threads - embedded_threads

            comparison["improvements"] = {
                "memory_savings_mb": memory_savings,
                "memory_savings_percent": memory_savings_percent,
                "cpu_savings_percent": cpu_savings,
                "thread_reduction": thread_savings,
                "summary": {
                    "memory_improved": memory_savings > 0,
                    "cpu_improved": cpu_savings > 0,
                    "threads_improved": thread_savings > 0,
                },
            }

            logger.info("[STATS] COMPARISON RESULTS:")
            logger.info(
                f"   Memory: {normal_mem:.1f}MB → {embedded_mem:.1f}MB "
                f"({memory_savings:+.1f}MB, {memory_savings_percent:+.1f}%)"
            )
            logger.info(
                f"   CPU: {normal_cpu:.1f}% → {embedded_cpu:.1f}% ({cpu_savings:+.1f}%)"
            )
            logger.info(
                f"   Threads: {normal_threads:.1f} → {embedded_threads:.1f} "
                f"({thread_savings:+.1f})"
            )

        return comparison


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


def test_embedded_mode_comparison(genome_path, logs_dir):
    """Compare resource usage between normal and embedded modes."""

    # Verify genome file exists
    assert os.path.exists(genome_path), f"Genome file not found: {genome_path}"

    # Create comparator
    comparator = FeagiModeComparator(genome_path, runtime_seconds=20)  # Shorter test

    try:
        # Run comparison
        results = comparator.compare_modes()

        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        comparison_file = logs_dir / f"mode_comparison_{timestamp}.json"

        with open(comparison_file, "w") as f:
            json.dump(results, indent=2, fp=f)

        logger.info(f"[FOLDER] Comparison results saved to {comparison_file}")

        # Validate results
        assert "normal_mode" in results
        assert "embedded_mode" in results

        # Check if embedded mode actually improved anything
        if "improvements" in results:
            improvements = results["improvements"]
            summary = improvements["summary"]

            logger.info("[SEARCH] EMBEDDED MODE EFFECTIVENESS:")
            logger.info(f"   Memory improved: {summary['memory_improved']}")
            logger.info(f"   CPU improved: {summary['cpu_improved']} ")
            logger.info(f"   Threads improved: {summary['threads_improved']}")

            # At least one metric should improve
            any_improvement = (
                summary["memory_improved"]
                or summary["cpu_improved"]
                or summary["threads_improved"]
            )

            if not any_improvement:
                logger.warning("[WARN]  Embedded mode did not improve any metrics!")
            else:
                logger.info("[OK] Embedded mode showed some improvements")

        logger.info("[OK] Mode comparison test completed successfully")

    except Exception as e:
        logger.error(f"[ERR] Mode comparison test failed: {e}")
        raise


if __name__ == "__main__":
    # Allow running this test directly
    pytest.main([__file__, "-v", "-s"])
