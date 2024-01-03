#!/bin/bash

# Input file and line number to replace
file_path="version.py"
line_number="1"

# Get current UNIX timestamp
timestamp=$(date +%s)
full_line="__version__ = '$timestamp'"

sed -i "${line_number}s/.*/$full_line/" "$file_path"
