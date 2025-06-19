#!/usr/bin/env python3
"""
Utility Function Search Script for FEAGI

This script helps developers find existing utility functions before
implementing new ones, preventing duplication.

Usage:
    python scripts/find_utils.py <search_term>
    python scripts/find_utils.py --list-all
    python scripts/find_utils.py --by-module

Examples:
    python scripts/find_utils.py linearize
    python scripts/find_utils.py debug
    python scripts/find_utils.py position
"""

import argparse
import ast
import sys
from pathlib import Path
from typing import Dict, List


class UtilityFinder:
    """Finds utility functions in the codebase."""

    # Known utility directories
    UTILITY_DIRS = [
        "feagi/bdu/utils/",
        "feagi/utils/",
        "feagi/core/",
        "feagi/api/utils/",
    ]

    def __init__(self):
        self.functions: Dict[str, List[Dict]] = {}
        self.modules: Dict[str, List[str]] = {}

    def scan_utilities(self) -> None:
        """Scan all utility directories for functions."""
        for util_dir in self.UTILITY_DIRS:
            util_path = Path(util_dir)
            if util_path.exists():
                for py_file in util_path.rglob("*.py"):
                    if py_file.name != "__init__.py":
                        self._scan_file(py_file)

    def _scan_file(self, file_path: Path) -> None:
        """Scan a single file for function definitions."""
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                source = f.read()

            tree = ast.parse(source)
            module_name = str(file_path).replace(".py", "").replace("/", ".")

            functions_in_module = []

            for node in ast.walk(tree):
                if isinstance(node, ast.FunctionDef):
                    # Skip private functions and methods
                    if node.name.startswith("_"):
                        continue

                    func_info = {
                        "name": node.name,
                        "file": str(file_path),
                        "line": node.lineno,
                        "module": module_name,
                        "signature": self._get_signature(node),
                        "docstring": ast.get_docstring(node) or "No documentation",
                    }

                    if node.name not in self.functions:
                        self.functions[node.name] = []
                    self.functions[node.name].append(func_info)
                    functions_in_module.append(node.name)

            if functions_in_module:
                self.modules[module_name] = functions_in_module

        except Exception as e:
            print(f"Error scanning {file_path}: {e}", file=sys.stderr)

    def _get_signature(self, node: ast.FunctionDef) -> str:
        """Extract function signature."""
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

    def search(self, term: str) -> List[Dict]:
        """Search for functions matching the term."""
        results = []
        term_lower = term.lower()

        for func_name, func_list in self.functions.items():
            # Check if term matches function name
            if term_lower in func_name.lower():
                results.extend(func_list)
            else:
                # Check docstrings
                for func_info in func_list:
                    if term_lower in func_info["docstring"].lower():
                        results.append(func_info)

        return results

    def list_all(self) -> None:
        """List all utility functions organized by module."""
        print("🔧 FEAGI Utility Functions")
        print("=" * 50)

        for module, functions in sorted(self.modules.items()):
            print(f"\n📦 {module}")
            print("-" * len(module))
            for func_name in sorted(functions):
                func_info = next(
                    f for f in self.functions[func_name] if f["module"] == module
                )
                print(f"  • {func_info['signature']}")
                # Show first line of docstring
                first_line = func_info["docstring"].split("\n")[0]
                if first_line and first_line != "No documentation":
                    print(f"    {first_line}")

    def list_by_category(self) -> None:
        """List functions grouped by category."""
        categories = {
            "Position & Coordinates": [
                "position",
                "coordinate",
                "linearize",
                "delinearize",
            ],
            "Debug & Logging": ["debug", "log", "logger"],
            "Metrics & Performance": ["metric", "performance", "timing", "profil"],
            "Array & Math": ["array", "math", "matrix", "vector"],
            "Validation": ["validate", "check", "verify"],
            "Conversion": ["convert", "transform", "serialize"],
        }

        print("🗂️  FEAGI Utilities by Category")
        print("=" * 50)

        for category, keywords in categories.items():
            print(f"\n📂 {category}")
            print("-" * len(category))

            found_functions = set()
            for keyword in keywords:
                for func_name, func_list in self.functions.items():
                    if keyword in func_name.lower():
                        for func_info in func_list:
                            if func_info["name"] not in found_functions:
                                print(f"  • {func_info['signature']}")
                                print(f"    📍 {func_info['file']}:{func_info['line']}")
                                found_functions.add(func_info["name"])

    def suggest_import(self, func_name: str) -> None:
        """Suggest how to import a function."""
        if func_name in self.functions:
            for func_info in self.functions[func_name]:
                module_path = func_info["module"]
                print(f"💡 To use {func_name}:")
                print(f"   from {module_path} import {func_name}")
                print(f"   # Located at: {func_info['file']}:{func_info['line']}")
                print(f"   # Signature: {func_info['signature']}")
                print()


def main():
    parser = argparse.ArgumentParser(
        description="Find utility functions in FEAGI codebase"
    )
    parser.add_argument(
        "search_term", nargs="?", help="Function name or keyword to search for"
    )
    parser.add_argument(
        "--list-all", action="store_true", help="List all utility functions"
    )
    parser.add_argument(
        "--by-category", action="store_true", help="List functions by category"
    )
    parser.add_argument(
        "--suggest-import", help="Show import statement for a specific function"
    )

    args = parser.parse_args()

    finder = UtilityFinder()
    finder.scan_utilities()

    if args.list_all:
        finder.list_all()
    elif args.by_category:
        finder.list_by_category()
    elif args.suggest_import:
        finder.suggest_import(args.suggest_import)
    elif args.search_term:
        results = finder.search(args.search_term)

        if results:
            print(
                f"🔍 Found {len(results)} utility function(s) matching '{args.search_term}':"
            )
            print("=" * 60)

            for func_info in results:
                print(f"\n✅ {func_info['signature']}")
                print(f"   📍 {func_info['file']}:{func_info['line']}")
                print(
                    f"   📦 Import: from {func_info['module']} import {func_info['name']}"
                )

                # Show docstring summary
                doc_lines = func_info["docstring"].split("\n")
                if doc_lines[0] and doc_lines[0] != "No documentation":
                    print(f"   📝 {doc_lines[0]}")
        else:
            print(f"❌ No utility functions found matching '{args.search_term}'")
            print("\n💡 Consider:")
            print("   1. Checking if you need to implement this function")
            print("   2. Running with --list-all to see all available utilities")
            print("   3. Running with --by-category to browse by function type")
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
