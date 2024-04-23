#!/bin/bash

# Input file and line number to replace
file_path="version.py"
line_number="1"

# Get current git tag from release in the repo
git fetch
current_tag_version=$(git tag | sort -V | tail -n 1)
if [ -z "$current_tag_version" ]; then
    current_tag_version="---"
fi
full_line="__version__ = '$current_tag_version'"

sed -i "${line_number}s/.*/$full_line/" "$file_path"
