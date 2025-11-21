#!/usr/bin/env python3
"""
Version Bumping Helper for FEAGI SDK

Automatically bumps version in pyproject.toml

Usage:
    # Bump patch version (3.0.0 → 3.0.1)
    python scripts/bump_version.py patch
    
    # Bump minor version (3.0.1 → 3.1.0)
    python scripts/bump_version.py minor
    
    # Bump major version (3.1.0 → 4.0.0)
    python scripts/bump_version.py major
    
    # Set specific version
    python scripts/bump_version.py set 3.2.0
    
    # Add dev suffix (3.0.0 → 3.0.0.dev1)
    python scripts/bump_version.py dev
"""

import re
import sys
from pathlib import Path


def get_current_version():
    """Read current version from pyproject.toml"""
    pyproject_path = Path(__file__).parent.parent / "pyproject.toml"
    
    with open(pyproject_path, 'r') as f:
        content = f.read()
    
    match = re.search(r'version = "([^"]+)"', content)
    if not match:
        raise ValueError("Could not find version in pyproject.toml")
    
    return match.group(1), content


def set_version(new_version):
    """Update version in pyproject.toml"""
    pyproject_path = Path(__file__).parent.parent / "pyproject.toml"
    
    with open(pyproject_path, 'r') as f:
        content = f.read()
    
    # Replace version
    new_content = re.sub(
        r'version = "[^"]+"',
        f'version = "{new_version}"',
        content,
        count=1
    )
    
    with open(pyproject_path, 'w') as f:
        f.write(new_content)
    
    print(f"✅ Version updated to: {new_version}")
    print(f"📝 Updated: {pyproject_path}")


def bump_version(bump_type):
    """Bump version based on type"""
    current_version, _ = get_current_version()
    
    # Remove dev suffix if present
    base_version = current_version.split('.dev')[0]
    
    # Parse version
    parts = base_version.split('.')
    if len(parts) != 3:
        raise ValueError(f"Invalid version format: {current_version}")
    
    major, minor, patch = map(int, parts)
    
    if bump_type == 'major':
        major += 1
        minor = 0
        patch = 0
    elif bump_type == 'minor':
        minor += 1
        patch = 0
    elif bump_type == 'patch':
        patch += 1
    elif bump_type == 'dev':
        # Add or increment dev suffix
        if '.dev' in current_version:
            dev_match = re.search(r'\.dev(\d+)', current_version)
            if dev_match:
                dev_num = int(dev_match.group(1)) + 1
            else:
                dev_num = 1
        else:
            dev_num = 1
        
        new_version = f"{base_version}.dev{dev_num}"
        set_version(new_version)
        return
    else:
        raise ValueError(f"Invalid bump type: {bump_type}")
    
    new_version = f"{major}.{minor}.{patch}"
    set_version(new_version)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    
    command = sys.argv[1].lower()
    
    try:
        current_version, _ = get_current_version()
        print(f"Current version: {current_version}")
        print()
        
        if command == 'set':
            if len(sys.argv) < 3:
                print("❌ Error: 'set' requires a version number")
                print("   Example: python scripts/bump_version.py set 3.2.0")
                sys.exit(1)
            new_version = sys.argv[2]
            set_version(new_version)
        
        elif command in ['major', 'minor', 'patch', 'dev']:
            bump_version(command)
        
        else:
            print(f"❌ Error: Unknown command '{command}'")
            print()
            print(__doc__)
            sys.exit(1)
        
        # Show git diff
        print()
        print("📋 Changes made:")
        import subprocess
        try:
            result = subprocess.run(
                ['git', 'diff', 'pyproject.toml'],
                capture_output=True,
                text=True
            )
            if result.stdout:
                print(result.stdout)
        except:
            pass
        
        print()
        print("Next steps:")
        print("  1. Review the change: git diff pyproject.toml")
        print("  2. Commit: git add pyproject.toml && git commit -m 'Bump version to X.Y.Z'")
        print("  3. Push to staging: git push origin staging")
        print("  4. GitHub Action will automatically publish to test.pypi.org")
    
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()






