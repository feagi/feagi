#!/usr/bin/env python
"""
This script helps identify and fix broken links in markdown files,
particularly focusing on cross-module links in documentation.
"""

import os
import re
import argparse
from pathlib import Path
from collections import defaultdict


def find_markdown_files(directory):
    """Find all markdown files in the given directory and its subdirectories."""
    return list(Path(directory).glob("**/*.md"))


def extract_links(file_path):
    """Extract all markdown links from a file."""
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()
    
    # Regular expression to match Markdown links: [text](url)
    # Also match image links: ![alt](url)
    links = re.findall(r'(!?\[.*?\])\((.*?)\)', content)
    
    return links


def categorize_links(links):
    """Categorize links as internal, external, anchor, etc."""
    categorized = {
        'external': [],  # http://, https://
        'relative': [],  # path/to/file.md
        'anchor': [],    # #section
        'image': [],     # *.png, *.jpg, etc.
        'other': []      # anything else
    }
    
    for link_text, link_url in links:
        if link_url.startswith(('http://', 'https://')):
            categorized['external'].append((link_text, link_url))
        elif link_url.startswith('#'):
            categorized['anchor'].append((link_text, link_url))
        elif any(link_url.lower().endswith(ext) for ext in ('.png', '.jpg', '.jpeg', '.gif', '.svg')):
            categorized['image'].append((link_text, link_url))
        elif link_url.endswith(('.md', '.mdx')) or '/' in link_url:
            categorized['relative'].append((link_text, link_url))
        else:
            categorized['other'].append((link_text, link_url))
    
    return categorized


def check_links(directory, root_dir=None):
    """Check for broken links in markdown files."""
    if root_dir is None:
        root_dir = directory
    
    root_path = Path(root_dir)
    all_files = find_markdown_files(directory)
    
    # Create a mapping of all available markdown files
    available_files = set()
    for file_path in all_files:
        # Get the path relative to the root directory
        rel_path = file_path.relative_to(root_path)
        available_files.add(str(rel_path))
        # Also add without .md extension (for links that omit the extension)
        if rel_path.suffix == '.md':
            available_files.add(str(rel_path.with_suffix('')))
    
    # Check links in each file
    broken_links = defaultdict(list)
    for file_path in all_files:
        links = extract_links(file_path)
        categorized = categorize_links(links)
        
        # Check relative links
        for link_text, link_url in categorized['relative']:
            # Remove fragment if present
            url_without_fragment = link_url.split('#')[0]
            
            # Skip empty URLs
            if not url_without_fragment:
                continue
            
            # For URLs pointing to markdown files, check if they exist
            if url_without_fragment.endswith(('.md', '.mdx')) or '/' in url_without_fragment:
                # Handle relative paths
                source_dir = file_path.parent.relative_to(root_path)
                target_path = (source_dir / url_without_fragment).resolve().relative_to(root_path)
                
                # Check if the target file exists
                if str(target_path) not in available_files and str(target_path) + '.md' not in available_files:
                    broken_links[file_path].append((link_text, link_url, 'File not found'))
    
    return broken_links


def suggest_fix(broken_link, available_files, source_file, root_dir):
    """Suggest possible fixes for a broken link."""
    link_text, link_url, _ = broken_link
    
    # Remove fragment if present
    url_without_fragment = link_url.split('#')[0]
    fragment = link_url[len(url_without_fragment):] if len(link_url) > len(url_without_fragment) else ''
    
    # Get the filename without path
    filename = os.path.basename(url_without_fragment)
    
    # Look for files with similar names
    similar_files = []
    for file in available_files:
        if os.path.basename(file) == filename:
            similar_files.append(file)
    
    if similar_files:
        source_rel_path = os.path.relpath(source_file, root_dir)
        source_dir = os.path.dirname(source_rel_path)
        
        suggestions = []
        for similar_file in similar_files:
            # Calculate relative path from source file to the similar file
            rel_path = os.path.relpath(similar_file, source_dir)
            suggestions.append(rel_path + fragment)
        
        return suggestions
    
    return []


def main():
    parser = argparse.ArgumentParser(description="Check for broken links in markdown files")
    parser.add_argument("directory", help="Directory to scan for markdown files")
    parser.add_argument("--root", help="Root directory for resolving relative links (defaults to directory)")
    args = parser.parse_args()
    
    if not os.path.isdir(args.directory):
        print(f"Error: {args.directory} is not a valid directory")
        return
    
    root_dir = args.root if args.root else args.directory
    if not os.path.isdir(root_dir):
        print(f"Error: Root directory {root_dir} is not valid")
        return
    
    broken_links = check_links(args.directory, root_dir)
    
    if broken_links:
        print(f"Found broken links in {len(broken_links)} files:")
        for file_path, links in broken_links.items():
            print(f"\n{file_path}:")
            for link_text, link_url, error in links:
                print(f"  - {link_text} -> {link_url} ({error})")
    else:
        print("No broken links found!")
    
    print(f"\nScanned {len(find_markdown_files(args.directory))} files in {args.directory}")


if __name__ == "__main__":
    main() 