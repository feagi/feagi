#!/usr/bin/env python
"""
This script converts HTML tables in markdown files to proper markdown tables.
It's specifically designed to fix HTML formatting issues in documentation.
"""

import os
import re
import argparse
from pathlib import Path
from bs4 import BeautifulSoup


def find_markdown_files(directory):
    """Find all markdown files in the given directory and its subdirectories."""
    return list(Path(directory).glob("**/*.md"))


def extract_html_tables(content):
    """Extract HTML tables from markdown content."""
    # Find all HTML tables using a regex pattern
    table_pattern = re.compile(r'<table>.*?</table>', re.DOTALL)
    tables = table_pattern.findall(content)
    return tables


def html_table_to_markdown(html_table):
    """Convert an HTML table to a markdown table."""
    try:
        # Parse the HTML table
        soup = BeautifulSoup(html_table, 'html.parser')
        table = soup.find('table')
        
        if not table:
            return None
        
        # Extract headers
        headers = []
        header_row = table.find('tr')
        if header_row:
            headers = [th.get_text().strip() for th in header_row.find_all(['th', 'td'])]
        
        # If no headers were found, try to use the first row as headers
        if not headers and table.find_all('tr'):
            first_row = table.find_all('tr')[0]
            headers = [td.get_text().strip() for td in first_row.find_all('td')]
        
        if not headers:
            # Can't create a markdown table without headers
            return None
        
        # Start building the markdown table
        markdown_table = []
        
        # Add header row
        header_line = '| ' + ' | '.join(headers) + ' |'
        markdown_table.append(header_line)
        
        # Add separator row
        separator_line = '| ' + ' | '.join(['---'] * len(headers)) + ' |'
        markdown_table.append(separator_line)
        
        # Add data rows (skip the header row if we used it for headers)
        rows = table.find_all('tr')
        start_row = 1 if headers == [td.get_text().strip() for td in rows[0].find_all('td')] else 0
        
        for row in rows[start_row:]:
            cells = row.find_all(['td', 'th'])
            if cells:
                cell_values = [cell.get_text().strip() for cell in cells]
                # Ensure we have the right number of cells
                while len(cell_values) < len(headers):
                    cell_values.append('')
                
                row_line = '| ' + ' | '.join(cell_values) + ' |'
                markdown_table.append(row_line)
        
        return '\n'.join(markdown_table)
    
    except Exception as e:
        print(f"Error converting HTML table: {e}")
        return None


def convert_tables_in_file(file_path, dry_run=False):
    """Convert all HTML tables in a file to markdown tables."""
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()
    
    # Extract HTML tables
    html_tables = extract_html_tables(content)
    
    if not html_tables:
        print(f"No HTML tables found in {file_path}")
        return False
    
    print(f"Found {len(html_tables)} HTML tables in {file_path}")
    
    # Replace each HTML table with a markdown table
    new_content = content
    for html_table in html_tables:
        markdown_table = html_table_to_markdown(html_table)
        if markdown_table:
            print(f"Successfully converted table in {file_path}")
            print(f"Original HTML table:\n{html_table[:200]}...")
            print(f"Converted to markdown table:\n{markdown_table}")
            
            if not dry_run:
                new_content = new_content.replace(html_table, markdown_table)
        else:
            print(f"Failed to convert table in {file_path}")
    
    # Write the new content back to the file
    if new_content != content and not dry_run:
        with open(file_path, 'w', encoding='utf-8') as file:
            file.write(new_content)
        print(f"Updated {file_path}")
        return True
    
    return False


def main():
    parser = argparse.ArgumentParser(description="Convert HTML tables in markdown files to markdown tables")
    parser.add_argument("directory", help="Directory to scan for markdown files")
    parser.add_argument("--dry-run", action="store_true", help="Don't modify files, just show what would be done")
    args = parser.parse_args()
    
    if not os.path.isdir(args.directory):
        print(f"Error: {args.directory} is not a valid directory")
        return
    
    markdown_files = find_markdown_files(args.directory)
    print(f"Found {len(markdown_files)} markdown files in {args.directory}")
    
    converted_count = 0
    for file_path in markdown_files:
        if convert_tables_in_file(file_path, args.dry_run):
            converted_count += 1
    
    print(f"\nSummary: Converted tables in {converted_count} files")


if __name__ == "__main__":
    main() 