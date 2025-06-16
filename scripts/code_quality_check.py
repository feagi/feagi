#!/usr/bin/env python3
"""
FEAGI Code Quality Checker
Detects code quality issues including duplicate methods, large files, and more.
"""

import argparse
import ast
import re
import sys
from collections import Counter
from pathlib import Path
from typing import Dict, List


class CodeQualityChecker:
    """Comprehensive code quality checker for FEAGI codebase."""

    def __init__(self, max_method_lines: int = 50, max_file_lines: int = 1000):
        self.max_method_lines = max_method_lines
        self.max_file_lines = max_file_lines
        self.issues = []

    def check_file(self, file_path: Path) -> Dict[str, any]:
        """Check a single Python file for quality issues."""
        if not file_path.suffix == ".py":
            return {"skipped": True, "reason": "Not a Python file"}

        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()
                lines = content.splitlines()

            results = {
                "file_path": str(file_path),
                "line_count": len(lines),
                "duplicate_methods": self._check_duplicate_methods(content),
                "large_methods": self._check_large_methods(content),
                "syntax_errors": self._check_syntax(content, file_path),
                "complexity_issues": self._check_complexity(content),
                "file_too_large": len(lines) > self.max_file_lines,
            }

            return results

        except Exception as e:
            return {"error": str(e), "file_path": str(file_path)}

    def _check_duplicate_methods(self, content: str) -> List[Dict[str, any]]:
        """Find duplicate method definitions."""
        method_pattern = r"^[ \t]*def (\w+)\(.*?\):"
        methods_with_lines = []

        for line_num, line in enumerate(content.splitlines(), 1):
            match = re.match(method_pattern, line)
            if match:
                method_name = match.group(1)
                methods_with_lines.append((method_name, line_num))

        # Count duplicates
        method_names = [method for method, _ in methods_with_lines]
        method_counts = Counter(method_names)

        duplicates = []
        for method, count in method_counts.items():
            if count > 1:
                lines = [
                    line_num
                    for method_name, line_num in methods_with_lines
                    if method_name == method
                ]
                duplicates.append(
                    {"method_name": method, "count": count, "line_numbers": lines}
                )

        return duplicates

    def _check_large_methods(self, content: str) -> List[Dict[str, any]]:
        """Find methods that are too large."""
        lines = content.splitlines()
        method_pattern = r"^[ \t]*def (\w+)\(.*?\):"
        large_methods = []

        current_method = None
        method_start = 0
        base_indent = 0

        for line_num, line in enumerate(lines, 1):
            method_match = re.match(method_pattern, line)

            if method_match:
                # Save previous method if it was large
                if current_method and (line_num - method_start) > self.max_method_lines:
                    large_methods.append(
                        {
                            "method_name": current_method,
                            "start_line": method_start,
                            "line_count": line_num - method_start,
                            "severity": (
                                "warning"
                                if (line_num - method_start)
                                < self.max_method_lines * 1.5
                                else "error"
                            ),
                        }
                    )

                # Start tracking new method
                current_method = method_match.group(1)
                method_start = line_num
                base_indent = len(line) - len(line.lstrip())

            elif current_method and line.strip():
                # Check if we've left the method
                current_indent = len(line) - len(line.lstrip())
                if current_indent <= base_indent and not line.startswith(
                    " " * (base_indent + 1)
                ):
                    # Method ended
                    if (line_num - method_start) > self.max_method_lines:
                        large_methods.append(
                            {
                                "method_name": current_method,
                                "start_line": method_start,
                                "line_count": line_num - method_start,
                                "severity": (
                                    "warning"
                                    if (line_num - method_start)
                                    < self.max_method_lines * 1.5
                                    else "error"
                                ),
                            }
                        )
                    current_method = None

        # Check final method
        if current_method:
            method_length = len(lines) - method_start + 1
            if method_length > self.max_method_lines:
                large_methods.append(
                    {
                        "method_name": current_method,
                        "start_line": method_start,
                        "line_count": method_length,
                        "severity": (
                            "warning"
                            if method_length < self.max_method_lines * 1.5
                            else "error"
                        ),
                    }
                )

        return large_methods

    def _check_syntax(self, content: str, file_path: Path) -> List[str]:
        """Check for Python syntax errors."""
        try:
            ast.parse(content)
            return []
        except SyntaxError as e:
            return [f"Syntax error at line {e.lineno}: {e.msg}"]
        except Exception as e:
            return [f"Parse error: {str(e)}"]

    def _check_complexity(self, content: str) -> List[Dict[str, any]]:
        """Check for complexity issues like deeply nested code."""
        lines = content.splitlines()
        complexity_issues = []

        for line_num, line in enumerate(lines, 1):
            # Check indentation depth (proxy for nesting complexity)
            if line.strip():
                indent_level = (len(line) - len(line.lstrip())) // 4
                if indent_level > 6:  # More than 6 levels of nesting
                    complexity_issues.append(
                        {
                            "type": "deep_nesting",
                            "line": line_num,
                            "indent_level": indent_level,
                            "severity": "warning" if indent_level < 8 else "error",
                        }
                    )

        return complexity_issues

    def check_directory(
        self, directory: Path, exclude_patterns: List[str] = None
    ) -> Dict[str, any]:
        """Check all Python files in a directory."""
        if exclude_patterns is None:
            exclude_patterns = ["__pycache__", ".git", "venv", ".venv", "build", "dist"]

        results = {
            "total_files": 0,
            "files_with_issues": 0,
            "duplicate_methods_found": 0,
            "large_methods_found": 0,
            "syntax_errors_found": 0,
            "files": [],
        }

        python_files = list(directory.rglob("*.py"))

        for file_path in python_files:
            # Skip excluded directories
            if any(pattern in str(file_path) for pattern in exclude_patterns):
                continue

            file_result = self.check_file(file_path)

            if "error" in file_result:
                continue

            results["total_files"] += 1

            has_issues = (
                file_result.get("duplicate_methods", [])
                or file_result.get("large_methods", [])
                or file_result.get("syntax_errors", [])
                or file_result.get("file_too_large", False)
            )

            if has_issues:
                results["files_with_issues"] += 1
                results["files"].append(file_result)

                results["duplicate_methods_found"] += len(
                    file_result.get("duplicate_methods", [])
                )
                results["large_methods_found"] += len(
                    file_result.get("large_methods", [])
                )
                results["syntax_errors_found"] += len(
                    file_result.get("syntax_errors", [])
                )

        return results

    def print_report(self, results: Dict[str, any], verbose: bool = False) -> bool:
        """Print a formatted report of quality issues."""
        print("=" * 80)
        print("FEAGI CODE QUALITY REPORT")
        print("=" * 80)

        print(f"📊 Files scanned: {results['total_files']}")
        print(f"⚠️  Files with issues: {results['files_with_issues']}")
        print(f"🔄 Duplicate methods: {results['duplicate_methods_found']}")
        print(f"📏 Large methods: {results['large_methods_found']}")
        print(f"❌ Syntax errors: {results['syntax_errors_found']}")

        has_critical_issues = False

        for file_result in results["files"]:
            print(f"\n📁 {file_result['file_path']}")
            print(f"   Lines: {file_result['line_count']}")

            # Duplicate methods (CRITICAL)
            if file_result.get("duplicate_methods"):
                has_critical_issues = True
                print("   🚨 CRITICAL: Duplicate methods found!")
                for dup in file_result["duplicate_methods"]:
                    print(
                        f"      ❌ {dup['method_name']}: {dup['count']} times at lines {dup['line_numbers']}"
                    )

            # Large methods (WARNING)
            if file_result.get("large_methods"):
                print("   📏 Large methods:")
                for large in file_result["large_methods"]:
                    severity_icon = "🚨" if large["severity"] == "error" else "⚠️"
                    print(
                        f"      {severity_icon} {large['method_name']}: {large['line_count']} lines (starts at {large['start_line']})"
                    )
                    if large["severity"] == "error":
                        has_critical_issues = True

            # Syntax errors (CRITICAL)
            if file_result.get("syntax_errors"):
                has_critical_issues = True
                print("   🚨 CRITICAL: Syntax errors!")
                for error in file_result["syntax_errors"]:
                    print(f"      ❌ {error}")

            # File too large (WARNING)
            if file_result.get("file_too_large"):
                print(
                    f"   ⚠️  File is large ({file_result['line_count']} lines > {self.max_file_lines})"
                )

        print("\n" + "=" * 80)
        if has_critical_issues:
            print("🚨 CRITICAL ISSUES FOUND - Fix before committing!")
            print("=" * 80)
            return False
        elif results["files_with_issues"] > 0:
            print("⚠️  Warnings found - Consider addressing these issues")
            print("=" * 80)
            return True
        else:
            print("✅ No code quality issues found")
            print("=" * 80)
            return True


def main():
    parser = argparse.ArgumentParser(description="FEAGI Code Quality Checker")
    parser.add_argument(
        "--directory",
        "-d",
        type=Path,
        default=Path("feagi"),
        help="Directory to scan (default: feagi)",
    )
    parser.add_argument(
        "--max-method-lines",
        type=int,
        default=50,
        help="Maximum lines per method (default: 50)",
    )
    parser.add_argument(
        "--max-file-lines",
        type=int,
        default=1000,
        help="Maximum lines per file (default: 1000)",
    )
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    parser.add_argument(
        "--fail-on-warnings",
        action="store_true",
        help="Exit with error code on warnings",
    )

    args = parser.parse_args()

    if not args.directory.exists():
        print(f"❌ Directory not found: {args.directory}")
        sys.exit(1)

    checker = CodeQualityChecker(
        max_method_lines=args.max_method_lines, max_file_lines=args.max_file_lines
    )

    results = checker.check_directory(args.directory)
    success = checker.print_report(results, args.verbose)

    if not success:
        sys.exit(1)  # Critical issues found
    elif args.fail_on_warnings and results["files_with_issues"] > 0:
        sys.exit(1)  # Warnings found and fail-on-warnings enabled
    else:
        sys.exit(0)  # Success


if __name__ == "__main__":
    main()
