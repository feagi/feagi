#!/usr/bin/env python
"""
This script helps identify and fix HTML formatting issues in markdown files,
particularly focusing on tables with HTML formatting issues.
"""

import os
import re
import argparse
from pathlib import Path


def find_markdown_files(directory):
    """Find all markdown files in the given directory and its subdirectories."""
    return list(Path(directory).glob("**/*.md"))


def check_html_tables(file_path):
    """Check for HTML tables that might have formatting issues."""
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()
        
    # Check for HTML tables (starting with <table>)
    html_tables = re.findall(r'<table>.*?</table>', content, re.DOTALL)
    
    if html_tables:
        print(f"\nHTML tables found in {file_path}:")
        for i, table in enumerate(html_tables, 1):
            # Look for common issues
            issues = []
            
            # Check for missing or malformed tags
            if table.count('<tr>') != table.count('</tr>'):
                issues.append("Mismatched <tr> tags")
            
            if table.count('<td>') != table.count('</td>'):
                issues.append("Mismatched <td> tags")
                
            if table.count('<th>') != table.count('</th>'):
                issues.append("Mismatched <th> tags")
            
            # Check for rows with different column counts
            rows = re.findall(r'<tr>.*?</tr>', table, re.DOTALL)
            if rows:
                col_counts = [len(re.findall(r'<t[dh]>', row)) for row in rows]
                if len(set(col_counts)) > 1:
                    issues.append(f"Inconsistent column counts: {col_counts}")
            
            if issues:
                print(f"  Table {i} has issues: {', '.join(issues)}")
                print(f"  Table content: {table[:200]}..." if len(table) > 200 else f"  Table content: {table}")
            else:
                print(f"  Table {i} appears to be well-formed")
        
        return html_tables
    
    return []


def check_markdown_tables(file_path):
    """Check for markdown tables that might have formatting issues."""
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()
    
    # Find markdown tables (lines with | characters)
    lines = content.split('\n')
    table_start_indices = []
    
    for i, line in enumerate(lines):
        if re.match(r'\s*\|.*\|\s*$', line):
            if i == 0 or not re.match(r'\s*\|.*\|\s*$', lines[i-1]):
                table_start_indices.append(i)
    
    tables = []
    for start_idx in table_start_indices:
        end_idx = start_idx
        while end_idx < len(lines) and re.match(r'\s*\|.*\|\s*$', lines[end_idx]):
            end_idx += 1
        
        tables.append((start_idx, end_idx, '\n'.join(lines[start_idx:end_idx])))
    
    if tables:
        print(f"\nMarkdown tables found in {file_path}:")
        for i, (start, end, table) in enumerate(tables, 1):
            # Look for common formatting issues
            table_lines = table.split('\n')
            
            issues = []
            
            # Check if it has a header separator row
            if len(table_lines) > 1:
                header_sep = table_lines[1]
                if not re.match(r'\s*\|[-\s|:]*\|\s*$', header_sep):
                    issues.append("Missing or malformed header separator")
            
            # Check for consistent cell count
            cell_counts = [line.count('|') for line in table_lines]
            if len(set(cell_counts)) > 1:
                issues.append(f"Inconsistent column counts: {cell_counts}")
            
            if issues:
                print(f"  Table at lines {start+1}-{end} has issues: {', '.join(issues)}")
                print(f"  Table content:\n{table}")
            else:
                print(f"  Table at lines {start+1}-{end} appears to be well-formed")
    
    return tables


def main():
    parser = argparse.ArgumentParser(description="Check for HTML and Markdown table issues in documentation files")
    parser.add_argument("directory", help="Directory to scan for markdown files")
    args = parser.parse_args()
    
    if not os.path.isdir(args.directory):
        print(f"Error: {args.directory} is not a valid directory")
        return
    
    markdown_files = find_markdown_files(args.directory)
    print(f"Found {len(markdown_files)} markdown files in {args.directory}")
    
    issue_count = 0
    for file_path in markdown_files:
        html_tables = check_html_tables(file_path)
        markdown_tables = check_markdown_tables(file_path)
        
        if html_tables or markdown_tables:
            issue_count += 1
    
    print(f"\nSummary: Checked {len(markdown_files)} files, found potential issues in {issue_count} files")
    
    if issue_count > 0:
        print("\nTo fix HTML tables, convert them to markdown format:")
        print("1. Use the markdown table format with | characters")
        print("2. Ensure the header row has a separator row with dashes")
        print("3. Make sure each row has the same number of cells")
        print("\nExample markdown table:")
        print("| Header 1 | Header 2 | Header 3 |")
        print("|----------|----------|----------|")
        print("| Value 1  | Value 2  | Value 3  |")
        print("| Value 4  | Value 5  | Value 6  |")


if __name__ == "__main__":
    main() 