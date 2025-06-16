#!/usr/bin/env python3
"""
Duplicate Function Detection Script for FEAGI Pre-commit Hooks

This script detects duplicate function implementations across the codebase
to prevent architectural violations and maintain code quality.

Usage:
    python scripts/check_duplicates.py [--fix] [--verbose] [paths...]

Exit codes:
    0: No duplicates found
    1: Duplicates found (or other errors)
    2: Critical duplicates found (must be fixed)
"""

import argparse
import ast
import hashlib
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple


class FunctionInfo:
    """Information about a function definition."""

    def __init__(
        self,
        name: str,
        file_path: str,
        line_number: int,
        signature: str,
        body_hash: str,
        is_method: bool = False,
    ):
        self.name = name
        self.file_path = file_path
        self.line_number = line_number
        self.signature = signature
        self.body_hash = body_hash
        self.is_method = is_method

    def __repr__(self):
        return f"FunctionInfo({self.name} @ {self.file_path}:{self.line_number})"


class DuplicateDetector:
    """Detects duplicate function implementations."""

    # Critical functions that should never be duplicated
    CRITICAL_FUNCTIONS = {
        "_is_debug_bdu_enabled",
        "linearize_position",
        "delinearize_position",
        "is_debug_enabled",
        "get_state_manager",
        "setup_logger",
    }

    # Known utility modules where functions should be centralized
    UTILITY_MODULES = {
        "feagi/bdu/utils/",
        "feagi/utils/",
        "feagi/core/",
    }

    # Functions to ignore (constructors, common patterns, etc.)
    IGNORE_FUNCTIONS = {
        "__init__",
        "__str__",
        "__repr__",
        "__eq__",
        "__hash__",
        "setUp",
        "tearDown",
        "test_",
        "mock_",
        "_test_",
        "get",
        "set",
        "add",
        "remove",
        "create",
        "delete",
        "load",
        "save",
        "reset",
        "clear",
        "update",
    }

    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.functions: Dict[str, List[FunctionInfo]] = defaultdict(list)
        self.duplicates: Dict[str, List[FunctionInfo]] = {}
        self.critical_duplicates: Dict[str, List[FunctionInfo]] = {}

    def should_ignore_function(self, func_name: str, file_path: str) -> bool:
        """Check if a function should be ignored."""
        # Ignore test functions
        if "/test" in file_path or file_path.endswith("_test.py"):
            return True

        # Ignore common patterns
        for pattern in self.IGNORE_FUNCTIONS:
            if func_name.startswith(pattern):
                return True

        # Ignore very short functions (likely trivial)
        if len(func_name) <= 2:
            return True

        return False

    def extract_function_signature(self, node: ast.FunctionDef) -> str:
        """Extract a normalized function signature."""
        args = []

        # Regular arguments
        for arg in node.args.args:
            args.append(arg.arg)

        # *args
        if node.args.vararg:
            args.append(f"*{node.args.vararg.arg}")

        # **kwargs
        if node.args.kwarg:
            args.append(f"**{node.args.kwarg.arg}")

        return f"{node.name}({', '.join(args)})"

    def extract_function_body_hash(
        self, node: ast.FunctionDef, source_lines: List[str]
    ) -> str:
        """Extract and hash the function body for similarity comparison."""
        try:
            # Get the function body lines
            start_line = node.lineno - 1  # ast uses 1-based indexing
            end_line = node.end_lineno if node.end_lineno else start_line + 1

            body_lines = source_lines[start_line:end_line]

            # Normalize the body (remove comments, extra whitespace)
            normalized_body = []
            for line in body_lines:
                # Remove comments
                line = re.sub(r"#.*$", "", line)
                # Normalize whitespace
                line = re.sub(r"\s+", " ", line.strip())
                if line:
                    normalized_body.append(line)

            # Create hash of normalized body
            body_text = "\n".join(normalized_body)
            return hashlib.md5(body_text.encode()).hexdigest()

        except Exception:
            # Fallback to simple hash if extraction fails
            return hashlib.md5(f"{node.name}_{node.lineno}".encode()).hexdigest()

    def analyze_file(self, file_path: Path) -> None:
        """Analyze a Python file for function definitions."""
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                source = f.read()
                source_lines = source.splitlines()

            tree = ast.parse(source)

            for node in ast.walk(tree):
                if isinstance(node, ast.FunctionDef):
                    func_name = node.name
                    file_str = str(file_path)

                    if self.should_ignore_function(func_name, file_str):
                        continue

                    signature = self.extract_function_signature(node)
                    body_hash = self.extract_function_body_hash(node, source_lines)
                    is_method = self.is_method(node, tree)

                    func_info = FunctionInfo(
                        name=func_name,
                        file_path=file_str,
                        line_number=node.lineno,
                        signature=signature,
                        body_hash=body_hash,
                        is_method=is_method,
                    )

                    self.functions[func_name].append(func_info)

                    if self.verbose:
                        print(
                            f"Found function: {func_name} in {file_str}:{node.lineno}"
                        )

        except Exception as e:
            if self.verbose:
                print(f"Error analyzing {file_path}: {e}")

    def is_method(self, func_node: ast.FunctionDef, tree: ast.AST) -> bool:
        """Check if a function is a method (inside a class)."""
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef):
                for item in node.body:
                    if item == func_node:
                        return True
        return False

    def find_duplicates(self) -> None:
        """Find duplicate function implementations."""
        for func_name, func_list in self.functions.items():
            if len(func_list) <= 1:
                continue

            # Group by signature and body hash to find true duplicates
            signature_groups = defaultdict(list)
            for func_info in func_list:
                key = f"{func_info.signature}_{func_info.body_hash}"
                signature_groups[key].append(func_info)

            # Check for duplicates within each signature group
            for group in signature_groups.values():
                if len(group) > 1:
                    self.duplicates[func_name] = group

                    # Check if this is a critical duplicate
                    if func_name in self.CRITICAL_FUNCTIONS:
                        self.critical_duplicates[func_name] = group

    def analyze_paths(self, paths: List[str]) -> None:
        """Analyze the given paths for duplicates."""
        python_files = []

        for path_str in paths:
            path = Path(path_str)
            if path.is_file() and path.suffix == ".py":
                python_files.append(path)
            elif path.is_dir():
                python_files.extend(path.rglob("*.py"))

        if self.verbose:
            print(f"Analyzing {len(python_files)} Python files...")

        for file_path in python_files:
            self.analyze_file(file_path)

        self.find_duplicates()

    def report_duplicates(self) -> bool:
        """Report found duplicates. Returns True if duplicates found."""
        has_duplicates = bool(self.duplicates)
        has_critical = bool(self.critical_duplicates)

        if has_critical:
            print("🚨 CRITICAL DUPLICATES FOUND:")
            print("=" * 50)
            for func_name, func_list in self.critical_duplicates.items():
                print(f"\n❌ CRITICAL: {func_name} ({len(func_list)} copies)")
                for func_info in func_list:
                    print(f"   📍 {func_info.file_path}:{func_info.line_number}")
                    print(f"      Signature: {func_info.signature}")

                # Suggest consolidation
                utility_files = [
                    f
                    for f in func_list
                    if any(um in f.file_path for um in self.UTILITY_MODULES)
                ]
                if utility_files:
                    canonical = utility_files[0]
                    print(
                        f"   💡 SUGGESTION: Use {canonical.file_path} as canonical implementation"
                    )
                else:
                    print(f"   💡 SUGGESTION: Move to feagi/bdu/utils/ or feagi/utils/")

        if self.duplicates and not has_critical:
            print("⚠️  DUPLICATES FOUND:")
            print("=" * 30)
            for func_name, func_list in self.duplicates.items():
                if func_name not in self.critical_duplicates:
                    print(f"\n⚠️  {func_name} ({len(func_list)} copies)")
                    for func_info in func_list:
                        print(f"   📍 {func_info.file_path}:{func_info.line_number}")

        if has_duplicates:
            print(f"\n📊 SUMMARY:")
            print(f"   Total duplicate functions: {len(self.duplicates)}")
            print(f"   Critical duplicates: {len(self.critical_duplicates)}")
            print(
                f"   Total duplicate instances: {sum(len(funcs) for funcs in self.duplicates.values())}"
            )

            if has_critical:
                print(
                    f"\n🚨 CRITICAL: This commit introduces or maintains critical duplicates!"
                )
                print(f"   These functions MUST be consolidated before merging.")
                return True
            else:
                print(f"\n⚠️  WARNING: Non-critical duplicates found.")
                print(
                    f"   Consider consolidating these functions to improve maintainability."
                )
        else:
            print("✅ No duplicate functions found!")

        return has_duplicates

    def suggest_fixes(self) -> None:
        """Suggest how to fix the duplicates."""
        if not self.duplicates:
            return

        print(f"\n🔧 SUGGESTED FIXES:")
        print("=" * 20)

        for func_name, func_list in self.duplicates.items():
            print(f"\n🔧 {func_name}:")

            # Find the best canonical location
            utility_files = [
                f
                for f in func_list
                if any(um in f.file_path for um in self.UTILITY_MODULES)
            ]

            if utility_files:
                canonical = utility_files[0]
                print(f"   1. Keep: {canonical.file_path}:{canonical.line_number}")

                others = [f for f in func_list if f != canonical]
                for other in others:
                    print(f"   2. Remove from: {other.file_path}:{other.line_number}")
                    print(
                        f"      Add import: from {self.path_to_import(canonical.file_path)} import {func_name}"
                    )
            else:
                print(f"   1. Choose one implementation as canonical")
                print(f"   2. Move to appropriate utils module")
                print(f"   3. Update imports in other locations")

    def path_to_import(self, file_path: str) -> str:
        """Convert file path to Python import path."""
        # Remove .py extension and convert slashes to dots
        import_path = file_path.replace(".py", "").replace("/", ".")

        # Remove leading feagi. if present
        if import_path.startswith("feagi."):
            import_path = import_path[6:]

        return f"feagi.{import_path}"


def main():
    parser = argparse.ArgumentParser(
        description="Detect duplicate functions in FEAGI codebase"
    )
    parser.add_argument(
        "paths",
        nargs="*",
        default=["."],
        help="Paths to analyze (default: current directory)",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Enable verbose output"
    )
    parser.add_argument(
        "--critical-only",
        action="store_true",
        help="Only check for critical duplicates",
    )
    parser.add_argument(
        "--suggest-fixes", action="store_true", help="Suggest how to fix duplicates"
    )

    args = parser.parse_args()

    detector = DuplicateDetector(verbose=args.verbose)
    detector.analyze_paths(args.paths)

    has_duplicates = detector.report_duplicates()

    if args.suggest_fixes and has_duplicates:
        detector.suggest_fixes()

    # Exit with appropriate code
    if detector.critical_duplicates:
        sys.exit(2)  # Critical duplicates found
    elif has_duplicates:
        sys.exit(1)  # Non-critical duplicates found
    else:
        sys.exit(0)  # No duplicates


if __name__ == "__main__":
    main()  # Test comment
