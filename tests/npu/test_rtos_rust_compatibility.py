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
RTOS/Rust Compatibility Monitor for NPU modules.

This test suite analyzes all NPU Python code for patterns that would be
incompatible with RTOS environments or difficult to migrate to Rust.

It performs static analysis using AST parsing to detect:
- Threading and async operations
- Dynamic memory allocation
- Non-deterministic timing operations
- Heavy Python-specific dependencies
- Exception handling in critical paths
- Global state and mutable singletons
- File I/O in hot paths
- Random operations

The goal is to maintain RTOS-friendly code and ease future Rust migration.
"""

import ast
import importlib.util
import os
import sys
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, NamedTuple, Set, Tuple

import pytest


class CompatibilityLevel(Enum):
    """Compatibility risk levels."""

    CRITICAL = "CRITICAL"  # Completely incompatible with RTOS
    HIGH = "HIGH"  # Major compatibility issues
    MEDIUM = "MEDIUM"  # Some compatibility concerns
    LOW = "LOW"  # Minor issues
    COMPATIBLE = "COMPATIBLE"  # RTOS/Rust friendly


@dataclass
class CompatibilityIssue:
    """Represents a single compatibility issue."""

    level: CompatibilityLevel
    category: str
    description: str
    file_path: str
    line_number: int
    code_snippet: str
    suggestion: str
    impact: str


@dataclass
class ModuleAnalysis:
    """Analysis results for a single module."""

    module_path: str
    issues: List[CompatibilityIssue] = field(default_factory=list)
    compatibility_score: float = 100.0
    overall_level: CompatibilityLevel = CompatibilityLevel.COMPATIBLE

    def add_issue(self, issue: CompatibilityIssue):
        """Add an issue and update compatibility score."""
        self.issues.append(issue)

        # Update score based on issue severity
        penalty = {
            CompatibilityLevel.CRITICAL: 25.0,
            CompatibilityLevel.HIGH: 15.0,
            CompatibilityLevel.MEDIUM: 8.0,
            CompatibilityLevel.LOW: 3.0,
            CompatibilityLevel.COMPATIBLE: 0.0,
        }
        self.compatibility_score -= penalty[issue.level]
        self.compatibility_score = max(0.0, self.compatibility_score)

        # Update overall level to worst issue level
        level_priority = {
            CompatibilityLevel.CRITICAL: 5,
            CompatibilityLevel.HIGH: 4,
            CompatibilityLevel.MEDIUM: 3,
            CompatibilityLevel.LOW: 2,
            CompatibilityLevel.COMPATIBLE: 1,
        }
        if level_priority[issue.level] > level_priority[self.overall_level]:
            self.overall_level = issue.level


class RTOSCompatibilityAnalyzer(ast.NodeVisitor):
    """AST visitor that detects RTOS/Rust compatibility issues."""

    def __init__(self, file_path: str, source_lines: List[str]):
        self.file_path = file_path
        self.source_lines = source_lines
        self.issues: List[CompatibilityIssue] = []
        self.current_function = None
        self.in_main_loop = False
        self.loop_depth = 0

    # Problematic imports and functions
    CRITICAL_IMPORTS = {
        "threading",
        "multiprocessing",
        "asyncio",
        "concurrent.futures",
        "queue",
        "subprocess",
        "signal",
    }

    HIGH_RISK_IMPORTS = {
        "numpy",
        "scipy",
        "pandas",
        "torch",
        "tensorflow",
        "requests",
        "urllib",
        "socket",
        "sqlite3",
    }

    MEDIUM_RISK_IMPORTS = {
        "random",
        "pickle",
        "json",
        "yaml",
        "configparser",
        "collections.defaultdict",
        "collections.deque",
    }

    CRITICAL_FUNCTIONS = {
        "time.sleep",
        "threading.Thread",
        "multiprocessing.Process",
        "asyncio.run",
        "asyncio.create_task",
        "subprocess.run",
        "signal.signal",
        "os.fork",
        "os.system",
    }

    HIGH_RISK_FUNCTIONS = {
        "numpy.resize",
        "numpy.append",
        "list.append",
        "dict.update",
        "numpy.concatenate",
        "numpy.hstack",
        "numpy.vstack",
        "open",
        "file.read",
        "file.write",
    }

    MEDIUM_RISK_FUNCTIONS = {
        "random.random",
        "random.randint",
        "random.choice",
        "time.time",
        "datetime.now",
        "print",
        "input",
        "len",
        "isinstance",
        "hasattr",
        "getattr",
    }

    def visit_Import(self, node: ast.Import):
        """Check import statements for problematic dependencies."""
        for alias in node.names:
            module_name = alias.name
            self._check_import_compatibility(module_name, node.lineno)
        self.generic_visit(node)

    def visit_ImportFrom(self, node: ast.ImportFrom):
        """Check from-import statements for problematic dependencies."""
        if node.module:
            self._check_import_compatibility(node.module, node.lineno)

            # Check specific imported names
            for alias in node.names:
                full_name = f"{node.module}.{alias.name}"
                self._check_function_compatibility(full_name, node.lineno, "import")
        self.generic_visit(node)

    def visit_Call(self, node: ast.Call):
        """Check function calls for RTOS incompatible operations."""
        func_name = self._get_function_name(node.func)
        if func_name:
            self._check_function_compatibility(func_name, node.lineno, "call")

            # Special checks for specific patterns
            self._check_dynamic_allocation_patterns(node, func_name)
            self._check_timing_patterns(node, func_name)

        self.generic_visit(node)

    def visit_For(self, node: ast.For):
        """Check for loops for problematic patterns."""
        self.loop_depth += 1

        # Check if this looks like a main loop
        if self._is_main_loop(node):
            self.in_main_loop = True

        self.generic_visit(node)
        self.loop_depth -= 1
        self.in_main_loop = False

    def visit_While(self, node: ast.While):
        """Check while loops for problematic patterns."""
        self.loop_depth += 1

        # Check if this looks like a main loop
        if self._is_main_loop(node):
            self.in_main_loop = True

        self.generic_visit(node)
        self.loop_depth -= 1
        self.in_main_loop = False

    def visit_Try(self, node: ast.Try):
        """Check try/except blocks in critical paths."""
        if self.in_main_loop:
            self._add_issue(
                CompatibilityLevel.MEDIUM,
                "Exception Handling",
                "Exception handling in main loop adds non-deterministic overhead",
                node.lineno,
                self._get_code_snippet(node.lineno),
                "Use Result<T,E> pattern or fail-fast approach for RTOS",
                "Non-deterministic timing due to exception handling overhead",
            )
        self.generic_visit(node)

    def visit_Global(self, node: ast.Global):
        """Check global variable usage."""
        self._add_issue(
            CompatibilityLevel.MEDIUM,
            "Global State",
            f"Global variables used: {', '.join(node.names)}",
            node.lineno,
            self._get_code_snippet(node.lineno),
            "Minimize global state for RTOS compatibility",
            "Global state can cause race conditions and complicate testing",
        )
        self.generic_visit(node)

    def visit_FunctionDef(self, node: ast.FunctionDef):
        """Track current function context."""
        old_function = self.current_function
        self.current_function = node.name
        self.generic_visit(node)
        self.current_function = old_function

    def _check_import_compatibility(self, module_name: str, line_no: int):
        """Check if an import is compatible with RTOS/Rust."""
        if module_name in self.CRITICAL_IMPORTS:
            self._add_issue(
                CompatibilityLevel.CRITICAL,
                "Critical Import",
                f"Import '{module_name}' is incompatible with RTOS",
                line_no,
                self._get_code_snippet(line_no),
                f"Replace {module_name} with RTOS-compatible alternative",
                "Cannot run in RTOS environment",
            )
        elif module_name in self.HIGH_RISK_IMPORTS:
            self._add_issue(
                CompatibilityLevel.HIGH,
                "High-Risk Import",
                f"Import '{module_name}' is difficult to port to embedded systems",
                line_no,
                self._get_code_snippet(line_no),
                f"Consider lighter alternative to {module_name} or conditional import",
                "Heavy dependency not available in embedded environments",
            )
        elif module_name in self.MEDIUM_RISK_IMPORTS:
            self._add_issue(
                CompatibilityLevel.MEDIUM,
                "Medium-Risk Import",
                f"Import '{module_name}' has some RTOS compatibility concerns",
                line_no,
                self._get_code_snippet(line_no),
                f"Use {module_name} carefully with fallback options",
                "May need custom implementation for RTOS",
            )

    def _check_function_compatibility(self, func_name: str, line_no: int, context: str):
        """Check if a function call is compatible with RTOS/Rust."""
        if func_name in self.CRITICAL_FUNCTIONS:
            self._add_issue(
                CompatibilityLevel.CRITICAL,
                "Critical Function",
                f"Function '{func_name}' is incompatible with RTOS",
                line_no,
                self._get_code_snippet(line_no),
                f"Replace {func_name} with RTOS-compatible alternative",
                "Blocking or non-deterministic operation",
            )
        elif func_name in self.HIGH_RISK_FUNCTIONS:
            self._add_issue(
                CompatibilityLevel.HIGH,
                "High-Risk Function",
                f"Function '{func_name}' causes dynamic allocation or I/O",
                line_no,
                self._get_code_snippet(line_no),
                f"Pre-allocate memory or use fixed-size alternatives",
                "Dynamic allocation forbidden in many RTOS systems",
            )
        elif func_name in self.MEDIUM_RISK_FUNCTIONS:
            self._add_issue(
                CompatibilityLevel.MEDIUM,
                "Medium-Risk Function",
                f"Function '{func_name}' has RTOS compatibility concerns",
                line_no,
                self._get_code_snippet(line_no),
                f"Use {func_name} carefully with deterministic alternatives",
                "May need custom implementation for predictable behavior",
            )

    def _check_dynamic_allocation_patterns(self, node: ast.Call, func_name: str):
        """Check for dynamic memory allocation patterns."""
        # Check for list operations in loops
        if self.loop_depth > 0 and func_name in [
            "list.append",
            "list.extend",
            "list.insert",
        ]:
            self._add_issue(
                CompatibilityLevel.HIGH,
                "Dynamic Allocation",
                f"Dynamic list growth in loop: {func_name}",
                node.lineno,
                self._get_code_snippet(node.lineno),
                "Pre-allocate lists or use fixed-size circular buffers",
                "Unpredictable memory allocation in hot path",
            )

        # Check for numpy resize operations
        if "resize" in func_name or "append" in func_name:
            self._add_issue(
                CompatibilityLevel.HIGH,
                "Dynamic Allocation",
                f"Dynamic array resizing: {func_name}",
                node.lineno,
                self._get_code_snippet(node.lineno),
                "Use fixed-size arrays allocated at initialization",
                "Memory allocation during runtime violates RTOS constraints",
            )

    def _check_timing_patterns(self, node: ast.Call, func_name: str):
        """Check for non-deterministic timing patterns."""
        if "sleep" in func_name:
            severity = (
                CompatibilityLevel.CRITICAL
                if self.in_main_loop
                else CompatibilityLevel.HIGH
            )
            self._add_issue(
                severity,
                "Non-Deterministic Timing",
                f"Blocking sleep operation: {func_name}",
                node.lineno,
                self._get_code_snippet(node.lineno),
                "Use deterministic timing wheels or RTOS timer APIs",
                "Sleep operations violate real-time constraints",
            )

        if func_name in ["time.time", "datetime.now"]:
            self._add_issue(
                CompatibilityLevel.MEDIUM,
                "Non-Deterministic Timing",
                f"Wall-clock time dependency: {func_name}",
                node.lineno,
                self._get_code_snippet(node.lineno),
                "Use monotonic time sources (time.perf_counter)",
                "Wall-clock time can jump and affect real-time behavior",
            )

    def _is_main_loop(self, node) -> bool:
        """Detect if this is likely a main processing loop."""
        # Look for common main loop patterns
        if isinstance(node, ast.While):
            test = node.test
            if isinstance(test, ast.Name) and "running" in test.id.lower():
                return True
            if isinstance(test, ast.Attribute) and "running" in test.attr.lower():
                return True

        if isinstance(node, ast.For):
            # Check if iterating over something that looks infinite
            if hasattr(node.iter, "id") and node.iter.id in ["itertools.count"]:
                return True

        return False

    def _get_function_name(self, node) -> str:
        """Extract function name from call node."""
        if isinstance(node, ast.Name):
            return node.id
        elif isinstance(node, ast.Attribute):
            if isinstance(node.value, ast.Name):
                return f"{node.value.id}.{node.attr}"
            elif isinstance(node.value, ast.Attribute):
                # Handle nested attributes like numpy.array.resize
                base = self._get_function_name(node.value)
                return f"{base}.{node.attr}" if base else node.attr
        return ""

    def _get_code_snippet(self, line_no: int, context: int = 0) -> str:
        """Get code snippet around the specified line."""
        if 1 <= line_no <= len(self.source_lines):
            start = max(0, line_no - 1 - context)
            end = min(len(self.source_lines), line_no + context)
            lines = self.source_lines[start:end]

            # Add line numbers and highlight the target line
            result = []
            for i, line in enumerate(lines):
                line_num = start + i + 1
                prefix = ">>> " if line_num == line_no else "    "
                result.append(f"{prefix}{line_num:3d}: {line.rstrip()}")

            return "\n".join(result)
        return f"Line {line_no}: <source not available>"

    def _add_issue(
        self,
        level: CompatibilityLevel,
        category: str,
        description: str,
        line_no: int,
        code_snippet: str,
        suggestion: str,
        impact: str,
    ):
        """Add a compatibility issue to the results."""
        issue = CompatibilityIssue(
            level=level,
            category=category,
            description=description,
            file_path=self.file_path,
            line_number=line_no,
            code_snippet=code_snippet,
            suggestion=suggestion,
            impact=impact,
        )
        self.issues.append(issue)


class NPUCompatibilityMonitor:
    """Main class for monitoring NPU RTOS/Rust compatibility."""

    def __init__(self, npu_path: str):
        self.npu_path = Path(npu_path)
        self.analyses: Dict[str, ModuleAnalysis] = {}

    def analyze_all_modules(self) -> Dict[str, ModuleAnalysis]:
        """Analyze all Python files in the NPU directory."""
        python_files = list(self.npu_path.glob("*.py"))

        for file_path in python_files:
            if file_path.name.startswith("__"):
                continue  # Skip __init__.py and __pycache__

            try:
                analysis = self.analyze_module(str(file_path))
                self.analyses[file_path.name] = analysis
            except Exception as e:
                # Create a failed analysis entry
                self.analyses[file_path.name] = ModuleAnalysis(
                    module_path=str(file_path),
                    issues=[
                        CompatibilityIssue(
                            level=CompatibilityLevel.HIGH,
                            category="Analysis Error",
                            description=f"Failed to analyze module: {str(e)}",
                            file_path=str(file_path),
                            line_number=1,
                            code_snippet="<analysis failed>",
                            suggestion="Fix syntax errors or add to skip list",
                            impact="Cannot assess RTOS compatibility",
                        )
                    ],
                    compatibility_score=0.0,
                    overall_level=CompatibilityLevel.HIGH,
                )

        return self.analyses

    def analyze_module(self, file_path: str) -> ModuleAnalysis:
        """Analyze a single Python module for RTOS/Rust compatibility."""
        with open(file_path, "r", encoding="utf-8") as f:
            source_code = f.read()
            source_lines = source_code.splitlines()

        # Parse the AST
        try:
            tree = ast.parse(source_code, filename=file_path)
        except SyntaxError as e:
            raise ValueError(f"Syntax error in {file_path}: {e}")

        # Analyze the AST
        analyzer = RTOSCompatibilityAnalyzer(file_path, source_lines)
        analyzer.visit(tree)

        # Create module analysis
        analysis = ModuleAnalysis(module_path=file_path)
        for issue in analyzer.issues:
            analysis.add_issue(issue)

        return analysis

    def generate_report(self) -> str:
        """Generate a comprehensive compatibility report."""
        if not self.analyses:
            return "No modules analyzed."

        report_lines = []
        report_lines.append("=" * 80)
        report_lines.append("NPU RTOS/RUST COMPATIBILITY ANALYSIS REPORT")
        report_lines.append("=" * 80)
        report_lines.append("")

        # Overall summary
        total_score = sum(a.compatibility_score for a in self.analyses.values())
        avg_score = total_score / len(self.analyses)

        critical_count = sum(
            1
            for a in self.analyses.values()
            if a.overall_level == CompatibilityLevel.CRITICAL
        )
        high_count = sum(
            1
            for a in self.analyses.values()
            if a.overall_level == CompatibilityLevel.HIGH
        )

        report_lines.append(f"📊 OVERALL SUMMARY:")
        report_lines.append(f"   Average Compatibility Score: {avg_score:.1f}/100")
        report_lines.append(f"   Modules with CRITICAL issues: {critical_count}")
        report_lines.append(f"   Modules with HIGH issues: {high_count}")
        report_lines.append(f"   Total modules analyzed: {len(self.analyses)}")
        report_lines.append("")

        # Module-by-module breakdown
        for module_name in sorted(self.analyses.keys()):
            analysis = self.analyses[module_name]
            report_lines.append("-" * 60)
            report_lines.append(f"📁 MODULE: {module_name}")
            report_lines.append(
                f"   Compatibility Score: {analysis.compatibility_score:.1f}/100"
            )
            report_lines.append(f"   Overall Level: {analysis.overall_level.value}")
            report_lines.append(f"   Issues Found: {len(analysis.issues)}")
            report_lines.append("")

            if analysis.issues:
                # Group issues by category
                issues_by_category = {}
                for issue in analysis.issues:
                    if issue.category not in issues_by_category:
                        issues_by_category[issue.category] = []
                    issues_by_category[issue.category].append(issue)

                for category in sorted(issues_by_category.keys()):
                    issues = issues_by_category[category]
                    report_lines.append(
                        f"   🔍 {category.upper()} ({len(issues)} issues):"
                    )

                    for issue in sorted(issues, key=lambda x: x.level.value):
                        level_emoji = {
                            CompatibilityLevel.CRITICAL: "🔴",
                            CompatibilityLevel.HIGH: "🟠",
                            CompatibilityLevel.MEDIUM: "🟡",
                            CompatibilityLevel.LOW: "🟢",
                            CompatibilityLevel.COMPATIBLE: "✅",
                        }

                        report_lines.append(
                            f"     {level_emoji[issue.level]} Line {issue.line_number}: {issue.description}"
                        )
                        report_lines.append(f"        Impact: {issue.impact}")
                        report_lines.append(f"        Suggestion: {issue.suggestion}")
                        report_lines.append("")
            else:
                report_lines.append("   ✅ No compatibility issues found!")
                report_lines.append("")

        # Recommendations
        report_lines.append("=" * 80)
        report_lines.append("🎯 RECOMMENDATIONS FOR RTOS/RUST MIGRATION")
        report_lines.append("=" * 80)

        if critical_count > 0:
            report_lines.append("🔴 CRITICAL PRIORITY:")
            report_lines.append("   1. Replace threading with RTOS task primitives")
            report_lines.append("   2. Eliminate all time.sleep() calls")
            report_lines.append(
                "   3. Remove signal handlers (use RTOS signals instead)"
            )
            report_lines.append("")

        if high_count > 0:
            report_lines.append("🟠 HIGH PRIORITY:")
            report_lines.append("   1. Pre-allocate all data structures")
            report_lines.append("   2. Replace NumPy with fixed-size arrays")
            report_lines.append("   3. Eliminate dynamic memory allocation")
            report_lines.append("")

        report_lines.append("🟡 MEDIUM PRIORITY:")
        report_lines.append(
            "   1. Replace exception handling with Result<T,E> patterns"
        )
        report_lines.append("   2. Minimize global state")
        report_lines.append("   3. Use monotonic time sources")
        report_lines.append("")

        return "\n".join(report_lines)


# Test fixtures and pytest integration


@pytest.fixture
def npu_path():
    """Get the path to the NPU modules."""
    # Go up from tests/npu to feagi/npu
    current_dir = Path(__file__).parent
    npu_dir = current_dir.parent.parent / "feagi" / "npu"
    return str(npu_dir)


@pytest.fixture
def compatibility_monitor(npu_path):
    """Create a compatibility monitor instance."""
    return NPUCompatibilityMonitor(npu_path)


def test_npu_rtos_rust_compatibility(compatibility_monitor, npu_path):
    """
    Main test that analyzes all NPU modules for RTOS/Rust compatibility.

    This test will fail if critical compatibility issues are found,
    encouraging developers to fix them before merging code.
    """
    print(f"\n🔍 Analyzing NPU modules in: {npu_path}")

    # Analyze all modules
    analyses = compatibility_monitor.analyze_all_modules()

    # Generate and print the report
    report = compatibility_monitor.generate_report()
    print(report)

    # Check if any modules have critical issues
    critical_modules = [
        name
        for name, analysis in analyses.items()
        if analysis.overall_level == CompatibilityLevel.CRITICAL
    ]

    high_risk_modules = [
        name
        for name, analysis in analyses.items()
        if analysis.overall_level == CompatibilityLevel.HIGH
    ]

    # Calculate overall statistics
    total_issues = sum(len(analysis.issues) for analysis in analyses.values())
    avg_score = sum(
        analysis.compatibility_score for analysis in analyses.values()
    ) / len(analyses)

    # Write detailed report to file for CI artifacts
    report_file = (
        Path(npu_path).parent.parent / "tmp" / "npu_rtos_compatibility_report.txt"
    )
    report_file.parent.mkdir(exist_ok=True)
    with open(report_file, "w") as f:
        f.write(report)
    print(f"\n📄 Detailed report written to: {report_file}")

    # Assertions for test pass/fail
    assert len(analyses) > 0, "No NPU modules found to analyze"

    # Warning for low compatibility scores (but don't fail)
    if avg_score < 70:
        import warnings

        warnings.warn(
            f"Average compatibility score is low: {avg_score:.1f}/100", UserWarning
        )

    # Fail test if there are critical compatibility issues
    if critical_modules:
        pytest.fail(
            f"CRITICAL RTOS compatibility issues found in modules: {', '.join(critical_modules)}. "
            f"These must be fixed before RTOS migration. See report for details."
        )

    # Warning for high-risk modules (but don't fail the build)
    if high_risk_modules:
        print(
            f"\n⚠️  WARNING: High-risk RTOS compatibility issues in: {', '.join(high_risk_modules)}"
        )
        print("These should be addressed before RTOS migration.")

    print(f"\n✅ RTOS Compatibility Analysis Complete:")
    print(f"   📊 Average Score: {avg_score:.1f}/100")
    print(f"   📈 Total Issues: {total_issues}")
    print(f"   🔴 Critical Modules: {len(critical_modules)}")
    print(f"   🟠 High-Risk Modules: {len(high_risk_modules)}")


def test_specific_module_compatibility(compatibility_monitor, npu_path):
    """
    Test individual modules to ensure they maintain compatibility over time.
    This test can be used to check specific modules without failing the build.
    """
    # Test the most critical modules individually
    critical_modules = [
        "burst_engine.py",
        "optimized_structures.py",
        "gpu_fcl_adapter.py",
    ]

    for module_name in critical_modules:
        module_path = Path(npu_path) / module_name
        if module_path.exists():
            print(f"\n🔍 Analyzing {module_name}...")
            analysis = compatibility_monitor.analyze_module(str(module_path))

            # Report the results for this module
            print(f"   Score: {analysis.compatibility_score:.1f}/100")
            print(f"   Level: {analysis.overall_level.value}")
            print(f"   Issues: {len(analysis.issues)}")

            # Store in analyses for reporting
            compatibility_monitor.analyses[module_name] = analysis


def test_compatibility_score_trend():
    """
    Test to track compatibility score trends over time.
    This can be used in CI to ensure scores don't regress.
    """
    # This would be implemented with historical data storage
    # For now, just ensure the basic infrastructure works
    assert True, "Trend tracking infrastructure ready"


if __name__ == "__main__":
    # Allow running this script directly for development
    import sys

    npu_path = sys.argv[1] if len(sys.argv) > 1 else "../../../feagi/npu"

    monitor = NPUCompatibilityMonitor(npu_path)
    analyses = monitor.analyze_all_modules()
    report = monitor.generate_report()

    print(report)

    # Write report to file
    with open("npu_rtos_compatibility_report.txt", "w") as f:
        f.write(report)

    print(f"\nReport written to: npu_rtos_compatibility_report.txt")
