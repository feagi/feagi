#!/usr/bin/env python3
"""
Script to systematically fix ruff issues in the BDU folder.
This script addresses the most common issues: E402 (imports) and E501 (line length).
"""

import os
import re
import subprocess
from pathlib import Path


def fix_imports_in_file(file_path):
    """Fix E402 issues by moving imports to the top of the file."""
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    lines = content.split("\n")

    # Find the end of the copyright header
    copyright_end = 0
    in_copyright = False
    for i, line in enumerate(lines):
        if line.strip().startswith('"""') and not in_copyright:
            in_copyright = True
        elif line.strip().endswith('"""') and in_copyright:
            copyright_end = i + 1
            break

    # Extract imports and non-imports
    imports = []
    other_lines = []
    docstring_lines = []

    in_docstring = False
    docstring_start = -1

    for i, line in enumerate(lines[copyright_end:], copyright_end):
        stripped = line.strip()

        # Handle module docstring
        if stripped.startswith('"""') and not in_docstring and i > copyright_end:
            in_docstring = True
            docstring_start = i
            docstring_lines.append(line)
        elif stripped.endswith('"""') and in_docstring:
            in_docstring = False
            docstring_lines.append(line)
        elif in_docstring:
            docstring_lines.append(line)
        # Handle imports
        elif (
            stripped.startswith("import ")
            or stripped.startswith("from ")
            or (stripped == "" and len(imports) > 0 and len(other_lines) == 0)
        ):
            imports.append(line)
        else:
            other_lines.append(line)

    # Reconstruct the file
    new_lines = lines[:copyright_end]

    # Add empty line after copyright
    if new_lines and new_lines[-1].strip():
        new_lines.append("")

    # Add imports
    if imports:
        new_lines.extend(imports)
        new_lines.append("")

    # Add module docstring
    if docstring_lines:
        new_lines.extend(docstring_lines)
        new_lines.append("")

    # Add other content
    new_lines.extend(other_lines)

    # Write back to file
    with open(file_path, "w", encoding="utf-8") as f:
        f.write("\n".join(new_lines))


def fix_long_lines_in_file(file_path):
    """Fix E501 issues by breaking long lines."""
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    lines = content.split("\n")
    new_lines = []

    for line in lines:
        if len(line) > 88:
            # Try to break long lines at logical points
            if " = " in line and len(line) > 88:
                # Break assignment lines
                parts = line.split(" = ", 1)
                if len(parts) == 2:
                    indent = len(line) - len(line.lstrip())
                    new_lines.append(parts[0] + " = (")
                    new_lines.append(" " * (indent + 4) + parts[1])
                    new_lines.append(" " * indent + ")")
                    continue

            # Break long string literals
            if 'f"' in line and len(line) > 88:
                # Try to break f-strings
                match = re.search(r'f"([^"]*)"', line)
                if match and len(match.group(0)) > 50:
                    # This is a complex case, just add the line as-is for now
                    pass

            # Break long comments
            if line.strip().startswith("#") and len(line) > 88:
                words = line.split()
                if len(words) > 1:
                    indent = len(line) - len(line.lstrip())
                    current_line = words[0]
                    for word in words[1:]:
                        if len(current_line + " " + word) <= 88:
                            current_line += " " + word
                        else:
                            new_lines.append(current_line)
                            current_line = " " * indent + "# " + word
                    new_lines.append(current_line)
                    continue

        new_lines.append(line)

    # Write back to file
    with open(file_path, "w", encoding="utf-8") as f:
        f.write("\n".join(new_lines))


def fix_b904_issues_in_file(file_path):
    """Fix B904 issues by adding 'from e' to raise statements."""
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    # Fix raise statements in except blocks
    pattern = (
        r"(\s+)except\s+\w+\s+as\s+(\w+):\s*\n(.*?\n)*?\s*raise\s+(\w+Error\([^)]*\))"
    )

    def replace_raise(match):
        indent = match.group(1)
        exception_var = match.group(2)
        middle_content = match.group(3) or ""
        raise_statement = match.group(4)

        return f"{indent}except Exception as {exception_var}:\n{middle_content}{indent}raise {raise_statement} from {exception_var}"

    content = re.sub(pattern, replace_raise, content, flags=re.MULTILINE | re.DOTALL)

    with open(file_path, "w", encoding="utf-8") as f:
        f.write(content)


def main():
    """Main function to fix ruff issues."""
    bdu_path = Path("feagi/bdu")

    if not bdu_path.exists():
        print(f"BDU path {bdu_path} does not exist")
        return

    # Find all Python files
    python_files = list(bdu_path.rglob("*.py"))

    print(f"Found {len(python_files)} Python files in BDU folder")

    for file_path in python_files:
        print(f"Processing {file_path}")

        try:
            # Fix imports (E402)
            fix_imports_in_file(file_path)

            # Fix long lines (E501) - basic cases
            fix_long_lines_in_file(file_path)

            # Fix B904 issues
            fix_b904_issues_in_file(file_path)

        except Exception as e:
            print(f"Error processing {file_path}: {e}")

    print("Finished processing files")

    # Run ruff check to see remaining issues
    try:
        result = subprocess.run(
            ["ruff", "check", "feagi/bdu/", "--no-fix"], capture_output=True, text=True
        )
        issue_count = (
            len(result.stdout.strip().split("\n")) if result.stdout.strip() else 0
        )
        print(f"Remaining ruff issues: {issue_count}")
    except Exception as e:
        print(f"Could not run ruff check: {e}")


if __name__ == "__main__":
    main()
