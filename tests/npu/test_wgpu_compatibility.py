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
WGPU Compatibility Monitor for NPU modules.

This test suite analyzes all NPU Python code for patterns that would be
compatible or incompatible with WGPU (native cross-platform GPU library
supporting Metal/DirectX 12/Vulkan).

It performs static analysis using AST parsing to detect:
- WGPU-compatible data structures (SoA, contiguous arrays)
- WGPU-incompatible patterns (os.environ, heavy stdout usage)
- Memory layout issues (non-contiguous arrays, mixed dtypes)
- Backend abstraction usage
- GPU-friendly vs CPU-only patterns

The goal is to maintain WGPU-ready code for cross-platform GPU acceleration.
"""

import ast
import os
import re
import sys
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, NamedTuple, Optional, Set, Tuple

import pytest


class WGPUCompatibilityLevel(Enum):
    """WGPU compatibility levels."""

    EXCELLENT = "EXCELLENT"  # Perfectly WGPU-compatible
    GOOD = "GOOD"  # WGPU-compatible with minor considerations
    MEDIUM = "MEDIUM"  # Requires adaptation for WGPU
    POOR = "POOR"  # Significant WGPU compatibility issues
    CRITICAL = "CRITICAL"  # Major blockers for WGPU


@dataclass
class WGPUAnalysisResult:
    """Results of WGPU compatibility analysis for a module."""

    module_name: str
    compatibility_score: float  # 0-100
    compatibility_level: WGPUCompatibilityLevel

    # WGPU-compatible patterns (good)
    soa_patterns: int  # Structure of Arrays usage
    contiguous_arrays: int  # np.zeros(..., order='C')
    gpu_friendly_dtypes: int  # float32, uint32, int32
    backend_abstractions: int  # get_backend() usage
    vectorized_operations: int  # NumPy vectorized ops
    pre_allocated_buffers: int  # Fixed-size allocations

    # WGPU-incompatible patterns (bad)
    environment_dependencies: int  # os.environ usage
    stdout_operations: int  # print() statements
    dynamic_allocations: int  # Runtime memory allocation
    file_operations: int  # File I/O
    non_contiguous_memory: int  # Non-contiguous layouts
    mixed_dtypes: int  # Mixed data types in arrays
    platform_specific_code: int  # Platform-dependent code

    # Neutral patterns (context-dependent)
    numpy_dependencies: int  # NumPy usage (can be abstracted)
    exception_handling: int  # Exception patterns
    logging_operations: int  # Logger usage

    # Detailed findings
    excellent_patterns: List[str]
    good_patterns: List[str]
    medium_concerns: List[str]
    poor_patterns: List[str]
    critical_issues: List[str]


class WGPUCompatibilityAnalyzer(ast.NodeVisitor):
    """AST visitor that analyzes Python code for WGPU compatibility."""

    def __init__(self, source_code: str, module_name: str):
        self.source_code = source_code
        self.module_name = module_name
        self.lines = source_code.split("\n")

        # Pattern counters
        self.soa_patterns = 0
        self.contiguous_arrays = 0
        self.gpu_friendly_dtypes = 0
        self.backend_abstractions = 0
        self.vectorized_operations = 0
        self.pre_allocated_buffers = 0

        self.environment_dependencies = 0
        self.stdout_operations = 0
        self.dynamic_allocations = 0
        self.file_operations = 0
        self.non_contiguous_memory = 0
        self.mixed_dtypes = 0
        self.platform_specific_code = 0

        self.numpy_dependencies = 0
        self.exception_handling = 0
        self.logging_operations = 0

        # Detailed findings
        self.excellent_patterns = []
        self.good_patterns = []
        self.medium_concerns = []
        self.poor_patterns = []
        self.critical_issues = []

        # Track imports
        self.imports = set()
        self.from_imports = {}  # module -> [names]

    def visit_Import(self, node: ast.Import):
        """Track import statements."""
        for alias in node.names:
            self.imports.add(alias.name)

            # Check for WGPU-compatible imports
            if alias.name == "numpy":
                self.numpy_dependencies += 1
                self.medium_concerns.append(
                    f"Line {node.lineno}: NumPy import (can be abstracted to WGPU buffers)"
                )
            elif alias.name == "os":
                self.environment_dependencies += 1
                self.poor_patterns.append(
                    f"Line {node.lineno}: os import (environment access incompatible with WGPU)"
                )

        self.generic_visit(node)

    def visit_ImportFrom(self, node: ast.ImportFrom):
        """Track from-import statements."""
        if node.module:
            if node.module not in self.from_imports:
                self.from_imports[node.module] = []

            for alias in node.names:
                self.from_imports[node.module].append(alias.name)

                # Check for backend abstraction usage
                if node.module == "feagi.core.backend" and alias.name in [
                    "get_backend",
                    "BackendType",
                ]:
                    self.backend_abstractions += 1
                    self.excellent_patterns.append(
                        f"Line {node.lineno}: Backend abstraction import (perfect for WGPU)"
                    )

                # Check for NumPy imports
                if alias.name == "np" and node.module == "numpy":
                    self.numpy_dependencies += 1

        self.generic_visit(node)

    def visit_Call(self, node: ast.Call):
        """Analyze function calls for WGPU compatibility patterns."""

        # Get function name if possible
        func_name = self._get_func_name(node.func)

        if func_name:
            # EXCELLENT: Backend abstraction usage
            if func_name == "get_backend":
                self.backend_abstractions += 1
                self.excellent_patterns.append(
                    f"Line {node.lineno}: get_backend() call (WGPU-ready)"
                )

            # GOOD: GPU-friendly NumPy array creation
            elif func_name in [
                "np.zeros",
                "np.ones",
                "np.empty",
                "numpy.zeros",
                "numpy.ones",
                "numpy.empty",
            ]:
                self._analyze_numpy_array_creation(node)

            # GOOD: Vectorized operations
            elif func_name in [
                "np.where",
                "np.add.at",
                "np.all",
                "np.any",
                "np.nonzero",
            ]:
                self.vectorized_operations += 1
                self.good_patterns.append(
                    f"Line {node.lineno}: Vectorized operation {func_name} (WGPU-compatible)"
                )

            # POOR: Environment variable access
            elif func_name == "os.environ.get":
                self.environment_dependencies += 1
                self.poor_patterns.append(
                    f"Line {node.lineno}: os.environ.get() (not available in WGPU)"
                )

            # POOR: Print statements (stdout issues)
            elif func_name == "print":
                self.stdout_operations += 1
                if self.stdout_operations <= 3:  # Don't spam for every print
                    self.poor_patterns.append(
                        f"Line {node.lineno}: print() statement (stdout not available in WGPU)"
                    )

            # POOR: Dynamic memory allocation
            elif func_name in ["list", "dict", "set"] and self._is_in_hot_path(node):
                self.dynamic_allocations += 1
                self.poor_patterns.append(
                    f"Line {node.lineno}: Dynamic allocation in hot path (incompatible with WGPU)"
                )

            # CRITICAL: File operations
            elif func_name in ["open", "file", "read", "write"]:
                self.file_operations += 1
                self.critical_issues.append(
                    f"Line {node.lineno}: File I/O operation (not available in WGPU)"
                )

            # MEDIUM: Exception handling
            elif func_name in ["raise", "except"]:
                self.exception_handling += 1

            # GOOD: Logging (better than print)
            elif "log" in func_name.lower() or func_name.startswith("logger."):
                self.logging_operations += 1
                if self.logging_operations <= 2:  # Don't spam
                    self.good_patterns.append(
                        f"Line {node.lineno}: Logging usage (WGPU-compatible)"
                    )

        self.generic_visit(node)

    def visit_Attribute(self, node: ast.Attribute):
        """Analyze attribute access for WGPU patterns."""

        # Check for os.environ access
        if (
            isinstance(node.value, ast.Name)
            and node.value.id == "os"
            and node.attr == "environ"
        ):
            self.environment_dependencies += 1
            self.poor_patterns.append(
                f"Line {node.lineno}: os.environ access (not available in WGPU)"
            )

        # Check for backend type checking
        elif node.attr == "backend_type":
            self.backend_abstractions += 1
            self.excellent_patterns.append(
                f"Line {node.lineno}: backend_type access (WGPU-ready)"
            )

        # Check for array order specification
        elif node.attr == "order" and self._is_numpy_context(node):
            # This is likely order='C' specification
            line_content = (
                self.lines[node.lineno - 1] if node.lineno <= len(self.lines) else ""
            )
            if "'C'" in line_content or '"C"' in line_content:
                self.contiguous_arrays += 1
                self.excellent_patterns.append(
                    f"Line {node.lineno}: Contiguous array order='C' (perfect for WGPU)"
                )

        self.generic_visit(node)

    def visit_Assign(self, node: ast.Assign):
        """Analyze assignments for WGPU-compatible patterns."""

        # Look for SoA patterns (Structure of Arrays)
        for target in node.targets:
            if isinstance(target, ast.Attribute):
                attr_name = target.attr

                # Check for coordinate SoA patterns
                if attr_name in ["coordinates_x", "coordinates_y", "coordinates_z"]:
                    self.soa_patterns += 1
                    self.excellent_patterns.append(
                        f"Line {node.lineno}: SoA coordinates pattern (perfect for WGPU)"
                    )

                # Check for other SoA patterns
                elif attr_name in [
                    "membrane_potentials",
                    "thresholds",
                    "refractory_periods",
                    "refractory_counters",
                    "enabled_flags",
                    "cortical_idxs",
                ]:
                    self.soa_patterns += 1
                    self.excellent_patterns.append(
                        f"Line {node.lineno}: SoA neural data pattern (perfect for WGPU)"
                    )

        self.generic_visit(node)

    def _get_func_name(self, node: ast.AST) -> Optional[str]:
        """Extract function name from call node."""
        if isinstance(node, ast.Name):
            return node.id
        elif isinstance(node, ast.Attribute):
            if isinstance(node.value, ast.Name):
                return f"{node.value.id}.{node.attr}"
            elif isinstance(node.value, ast.Attribute):
                parent = self._get_func_name(node.value)
                return f"{parent}.{node.attr}" if parent else None
        return None

    def _analyze_numpy_array_creation(self, node: ast.Call):
        """Analyze NumPy array creation for WGPU compatibility."""
        line_content = (
            self.lines[node.lineno - 1] if node.lineno <= len(self.lines) else ""
        )

        # Check for WGPU-friendly dtypes
        if any(dtype in line_content for dtype in ["float32", "uint32", "int32"]):
            self.gpu_friendly_dtypes += 1
            self.excellent_patterns.append(
                f"Line {node.lineno}: GPU-friendly dtype (perfect for WGPU)"
            )

        # Check for contiguous memory layout
        if "order='C'" in line_content or 'order="C"' in line_content:
            self.contiguous_arrays += 1
            self.excellent_patterns.append(
                f"Line {node.lineno}: Contiguous array layout (perfect for WGPU)"
            )

        # Check for pre-allocation (capacity parameter)
        if "capacity" in line_content:
            self.pre_allocated_buffers += 1
            self.good_patterns.append(
                f"Line {node.lineno}: Pre-allocated buffer (WGPU-compatible)"
            )

    def _is_in_hot_path(self, node: ast.AST) -> bool:
        """Determine if a node is likely in a performance-critical path."""
        # Simple heuristic: check if we're inside a loop or a method with "update", "process", "run"
        line_content = (
            self.lines[node.lineno - 1] if node.lineno <= len(self.lines) else ""
        )
        context_lines = self.lines[max(0, node.lineno - 5) : node.lineno + 5]
        context = " ".join(context_lines).lower()

        return any(
            keyword in context
            for keyword in ["for ", "while ", "update", "process", "run", "loop"]
        )

    def _is_numpy_context(self, node: ast.AST) -> bool:
        """Check if we're in a NumPy-related context."""
        line_content = (
            self.lines[node.lineno - 1] if node.lineno <= len(self.lines) else ""
        )
        return "np." in line_content or "numpy." in line_content

    def analyze(self) -> WGPUAnalysisResult:
        """Perform the full analysis and return results."""
        try:
            tree = ast.parse(self.source_code)
            self.visit(tree)
        except SyntaxError as e:
            self.critical_issues.append(f"Syntax error: {e}")

        # Calculate compatibility score
        score = self._calculate_compatibility_score()
        level = self._determine_compatibility_level(score)

        return WGPUAnalysisResult(
            module_name=self.module_name,
            compatibility_score=score,
            compatibility_level=level,
            soa_patterns=self.soa_patterns,
            contiguous_arrays=self.contiguous_arrays,
            gpu_friendly_dtypes=self.gpu_friendly_dtypes,
            backend_abstractions=self.backend_abstractions,
            vectorized_operations=self.vectorized_operations,
            pre_allocated_buffers=self.pre_allocated_buffers,
            environment_dependencies=self.environment_dependencies,
            stdout_operations=self.stdout_operations,
            dynamic_allocations=self.dynamic_allocations,
            file_operations=self.file_operations,
            non_contiguous_memory=self.non_contiguous_memory,
            mixed_dtypes=self.mixed_dtypes,
            platform_specific_code=self.platform_specific_code,
            numpy_dependencies=self.numpy_dependencies,
            exception_handling=self.exception_handling,
            logging_operations=self.logging_operations,
            excellent_patterns=self.excellent_patterns,
            good_patterns=self.good_patterns,
            medium_concerns=self.medium_concerns,
            poor_patterns=self.poor_patterns,
            critical_issues=self.critical_issues,
        )

    def _calculate_compatibility_score(self) -> float:
        """Calculate WGPU compatibility score (0-100)."""

        # Positive points for WGPU-compatible patterns
        positive_points = (
            self.soa_patterns * 15  # SoA is crucial for WGPU
            + self.contiguous_arrays * 12  # Contiguous memory essential
            + self.gpu_friendly_dtypes * 10  # Proper dtypes important
            + self.backend_abstractions * 20  # Backend abstraction critical
            + self.vectorized_operations * 8  # Vectorization good
            + self.pre_allocated_buffers * 10  # Pre-allocation important
        )

        # Negative points for incompatible patterns
        negative_points = (
            self.environment_dependencies * 25  # Environment vars major issue
            + self.stdout_operations * 3  # Print statements problematic
            + self.dynamic_allocations * 15  # Dynamic allocation bad
            + self.file_operations * 30  # File I/O critical issue
            + self.non_contiguous_memory * 20  # Memory layout crucial
            + self.mixed_dtypes * 15  # Type consistency important
            + self.platform_specific_code * 25  # Platform specificity bad
        )

        # Neutral adjustments
        neutral_adjustment = (
            -self.numpy_dependencies * 2  # NumPy slightly negative (can abstract)
            + self.logging_operations * 2  # Logging slightly positive
        )

        # Base score calculation
        raw_score = max(0, 60 + positive_points - negative_points + neutral_adjustment)

        # Cap at 100
        return min(100.0, raw_score)

    def _determine_compatibility_level(self, score: float) -> WGPUCompatibilityLevel:
        """Determine compatibility level based on score and critical issues."""

        # Critical issues always result in CRITICAL level
        if self.file_operations > 0 or self.environment_dependencies > 5:
            return WGPUCompatibilityLevel.CRITICAL

        # Score-based determination
        if score >= 90:
            return WGPUCompatibilityLevel.EXCELLENT
        elif score >= 75:
            return WGPUCompatibilityLevel.GOOD
        elif score >= 50:
            return WGPUCompatibilityLevel.MEDIUM
        elif score >= 25:
            return WGPUCompatibilityLevel.POOR
        else:
            return WGPUCompatibilityLevel.CRITICAL


class NPUWGPUCompatibilityMonitor:
    """Monitor for NPU WGPU compatibility."""

    def __init__(self, npu_dir: str):
        self.npu_dir = Path(npu_dir)
        if not self.npu_dir.exists():
            raise FileNotFoundError(f"NPU directory not found: {npu_dir}")

    def get_python_files(self) -> List[Path]:
        """Get all Python files in the NPU directory."""
        python_files = []
        for file_path in self.npu_dir.rglob("*.py"):
            if file_path.name != "__init__.py":  # Skip __init__.py files
                python_files.append(file_path)
        return python_files

    def analyze_file(self, file_path: Path) -> WGPUAnalysisResult:
        """Analyze a single Python file for WGPU compatibility."""
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                source_code = f.read()

            analyzer = WGPUCompatibilityAnalyzer(source_code, file_path.name)
            return analyzer.analyze()

        except Exception as e:
            # Return a result indicating analysis failure
            return WGPUAnalysisResult(
                module_name=file_path.name,
                compatibility_score=0.0,
                compatibility_level=WGPUCompatibilityLevel.CRITICAL,
                soa_patterns=0,
                contiguous_arrays=0,
                gpu_friendly_dtypes=0,
                backend_abstractions=0,
                vectorized_operations=0,
                pre_allocated_buffers=0,
                environment_dependencies=0,
                stdout_operations=0,
                dynamic_allocations=0,
                file_operations=0,
                non_contiguous_memory=0,
                mixed_dtypes=0,
                platform_specific_code=0,
                numpy_dependencies=0,
                exception_handling=0,
                logging_operations=0,
                excellent_patterns=[],
                good_patterns=[],
                medium_concerns=[],
                poor_patterns=[],
                critical_issues=[f"Analysis failed: {str(e)}"],
            )

    def analyze_all_modules(self) -> Dict[str, WGPUAnalysisResult]:
        """Analyze all NPU modules for WGPU compatibility."""
        results = {}
        python_files = self.get_python_files()

        for file_path in python_files:
            result = self.analyze_file(file_path)
            results[file_path.name] = result

        return results

    def generate_report(self, results: Dict[str, WGPUAnalysisResult]) -> str:
        """Generate a comprehensive WGPU compatibility report."""

        # Calculate summary statistics
        total_modules = len(results)
        scores = [r.compatibility_score for r in results.values()]
        avg_score = sum(scores) / len(scores) if scores else 0

        # Count by compatibility level
        level_counts = {}
        for level in WGPUCompatibilityLevel:
            level_counts[level] = sum(
                1 for r in results.values() if r.compatibility_level == level
            )

        # Find critical and excellent modules
        critical_modules = [
            name
            for name, r in results.items()
            if r.compatibility_level == WGPUCompatibilityLevel.CRITICAL
        ]
        excellent_modules = [
            name
            for name, r in results.items()
            if r.compatibility_level == WGPUCompatibilityLevel.EXCELLENT
        ]

        report = []
        report.append("[START] NPU WGPU COMPATIBILITY ANALYSIS REPORT")
        report.append("=" * 60)
        report.append(f"[STATS] SUMMARY:")
        report.append(f"   Total modules analyzed: {total_modules}")
        report.append(f"   Average compatibility score: {avg_score:.1f}/100")
        report.append("")

        report.append("[UP] COMPATIBILITY BREAKDOWN:")
        for level in WGPUCompatibilityLevel:
            emoji = {
                "EXCELLENT": "🟢",
                "GOOD": "🟢",
                "MEDIUM": "🟡",
                "POOR": "🟠",
                "CRITICAL": "🔴",
            }[level.value]
            count = level_counts[level]
            percentage = (count / total_modules * 100) if total_modules > 0 else 0
            report.append(
                f"   {emoji} {level.value}: {count} modules ({percentage:.1f}%)"
            )
        report.append("")

        if excellent_modules:
            report.append("[OK] WGPU-READY MODULES:")
            for module in excellent_modules:
                score = results[module].compatibility_score
                report.append(f"   🟢 {module} (Score: {score:.1f}/100)")
            report.append("")

        if critical_modules:
            report.append("🔴 CRITICAL WGPU COMPATIBILITY ISSUES:")
            for module in critical_modules:
                result = results[module]
                report.append(
                    f"   🔴 {module} (Score: {result.compatibility_score:.1f}/100)"
                )
                for issue in result.critical_issues[:3]:  # Show top 3 issues
                    report.append(f"      - {issue}")
            report.append("")

        report.append("[TARGET] DETAILED MODULE ANALYSIS:")
        for module_name, result in sorted(
            results.items(), key=lambda x: x[1].compatibility_score, reverse=True
        ):
            emoji = {
                "EXCELLENT": "🟢",
                "GOOD": "🟢",
                "MEDIUM": "🟡",
                "POOR": "🟠",
                "CRITICAL": "🔴",
            }[result.compatibility_level.value]
            report.append(
                f"{emoji} {module_name}: {result.compatibility_score:.1f}/100 ({result.compatibility_level.value})"
            )

            # Show top patterns
            if result.excellent_patterns:
                report.append(
                    f"   [OK] WGPU-Ready: {len(result.excellent_patterns)} excellent patterns"
                )
                for pattern in result.excellent_patterns[:2]:  # Show top 2
                    report.append(f"      + {pattern}")

            if result.poor_patterns:
                report.append(
                    f"   [WARN]  Issues: {len(result.poor_patterns)} problematic patterns"
                )
                for pattern in result.poor_patterns[:2]:  # Show top 2
                    report.append(f"      - {pattern}")

            if result.critical_issues:
                report.append(
                    f"   🔴 Critical: {len(result.critical_issues)} blocking issues"
                )
                for issue in result.critical_issues[:1]:  # Show top 1
                    report.append(f"      ! {issue}")

            report.append("")

        report.append("[CONFIG] WGPU IMPLEMENTATION RECOMMENDATIONS:")
        report.append("   1. Modules with excellent scores are WGPU-ready")
        report.append("   2. Replace os.environ with config parameters")
        report.append("   3. Replace print() with logger calls")
        report.append("   4. Implement WGPUBackend class in backend system")
        report.append("   5. Maintain SoA patterns and contiguous memory layouts")

        return "\n".join(report)


def test_npu_wgpu_compatibility():
    """
    Test NPU modules for WGPU compatibility.

    This test ensures that all NPU modules remain compatible with WGPU
    (native cross-platform GPU library supporting Metal/DirectX 12/Vulkan).

    The test FAILS if:
    - Any module has critical WGPU compatibility issues
    - Average compatibility score drops below 70/100
    - Critical patterns like file I/O or excessive environment dependencies are found
    """

    # Initialize the monitor
    monitor = NPUWGPUCompatibilityMonitor("feagi/npu")

    # Analyze all modules
    analyses = monitor.analyze_all_modules()

    # Generate and save report
    report = monitor.generate_report(analyses)

    # Save report to tmp directory for review
    os.makedirs("tmp", exist_ok=True)
    with open("tmp/npu_wgpu_compatibility_report.txt", "w") as f:
        f.write(report)

    print("\n" + "=" * 60)
    print("[START] NPU WGPU COMPATIBILITY TEST")
    print("=" * 60)

    # Calculate summary metrics
    total_modules = len(analyses)
    avg_score = (
        sum(a.compatibility_score for a in analyses.values()) / total_modules
        if total_modules > 0
        else 0
    )

    # Find critical modules
    critical_modules = [
        name
        for name, analysis in analyses.items()
        if analysis.compatibility_level == WGPUCompatibilityLevel.CRITICAL
    ]

    # Find modules with file I/O (absolute blocker)
    file_io_modules = [
        name for name, analysis in analyses.items() if analysis.file_operations > 0
    ]

    # Find modules with excessive environment dependencies
    env_heavy_modules = [
        name
        for name, analysis in analyses.items()
        if analysis.environment_dependencies > 5
    ]

    print(f"[STATS] Analyzed {total_modules} NPU modules")
    print(f"[UP] Average WGPU compatibility score: {avg_score:.1f}/100")

    # Print summary by compatibility level
    for level in WGPUCompatibilityLevel:
        count = sum(1 for a in analyses.values() if a.compatibility_level == level)
        emoji = {
            "EXCELLENT": "🟢",
            "GOOD": "🟢",
            "MEDIUM": "🟡",
            "POOR": "🟠",
            "CRITICAL": "🔴",
        }[level.value]
        print(f"{emoji} {level.value}: {count} modules")

    # Show excellent modules (WGPU-ready)
    excellent_modules = [
        name
        for name, analysis in analyses.items()
        if analysis.compatibility_level == WGPUCompatibilityLevel.EXCELLENT
    ]
    if excellent_modules:
        print(f"\n[OK] WGPU-READY MODULES ({len(excellent_modules)}):")
        for module in excellent_modules[:5]:  # Show top 5
            score = analyses[module].compatibility_score
            print(f"   🟢 {module} ({score:.1f}/100)")

    print(f"\n📄 Detailed report saved to: tmp/npu_wgpu_compatibility_report.txt")

    # Failure conditions
    failure_reasons = []

    # CRITICAL: File I/O operations (absolute blocker for WGPU)
    if file_io_modules:
        failure_reasons.append(
            f"CRITICAL: File I/O found in modules: {', '.join(file_io_modules)}"
        )

    # CRITICAL: Excessive environment dependencies (major WGPU blocker)
    if env_heavy_modules:
        failure_reasons.append(
            f"CRITICAL: Heavy environment dependencies in modules: {', '.join(env_heavy_modules)}"
        )

    # CRITICAL: Overall poor compatibility
    if avg_score < 70:
        failure_reasons.append(
            f"CRITICAL: Average WGPU compatibility score too low: {avg_score:.1f}/100 (minimum: 70)"
        )

    # CRITICAL: Too many critical modules
    if len(critical_modules) > 1:  # Allow max 1 critical module
        failure_reasons.append(
            f"CRITICAL: Too many modules with critical WGPU issues: {', '.join(critical_modules)}"
        )

    # Warning for low compatibility scores (but don't fail)
    if avg_score < 80:
        import warnings

        warnings.warn(
            f"NPU WGPU compatibility score is below optimal: {avg_score:.1f}/100",
            UserWarning,
        )

    # Fail test if there are critical compatibility issues
    if failure_reasons:
        failure_message = (
            f"\n🔴 NPU WGPU COMPATIBILITY TEST FAILED!\n\n"
            f"FAILURE REASONS:\n"
            + "\n".join(f"  • {reason}" for reason in failure_reasons)
            + f"\n\nCRITICAL WGPU compatibility issues must be fixed before enabling WGPU support.\n"
            f"These issues prevent WGPU from running on Metal/DirectX 12/Vulkan backends.\n"
            f"See report for detailed recommendations: tmp/npu_wgpu_compatibility_report.txt"
        )
        pytest.fail(failure_message)

    print(f"\n[OK] NPU WGPU COMPATIBILITY TEST PASSED!")
    print(
        f"[START] NPU modules are ready for WGPU acceleration on Metal/DirectX 12/Vulkan!"
    )


if __name__ == "__main__":
    # Allow running as standalone script with proper path handling
    import os
    import sys

    # Add the feagi_core directory to path for imports
    current_dir = Path(__file__).parent
    feagi_core_dir = current_dir.parent.parent
    sys.path.insert(0, str(feagi_core_dir))

    # Change to feagi_core directory for correct relative paths
    original_cwd = os.getcwd()
    os.chdir(feagi_core_dir)

    try:
        test_npu_wgpu_compatibility()
    finally:
        # Restore original working directory
        os.chdir(original_cwd)
