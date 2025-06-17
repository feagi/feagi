#!/usr/bin/env python3
"""
Install FEAGI State Manager Pre-commit Hooks

This script installs pre-commit hooks to automatically check for state management
violations before commits are allowed.

Usage:
    python scripts/install_state_manager_hooks.py [--force]
"""

import os
import sys
import stat
import argparse
from pathlib import Path

def create_pre_commit_hook(git_hooks_dir: Path, force: bool = False) -> bool:
    """Create the pre-commit hook script."""
    
    hook_path = git_hooks_dir / "pre-commit"
    
    # Check if hook already exists
    if hook_path.exists() and not force:
        print(f"⚠️  Pre-commit hook already exists: {hook_path}")
        print("Use --force to overwrite existing hook")
        return False
    
    # Pre-commit hook content
    hook_content = '''#!/bin/bash
# FEAGI State Manager Pre-commit Hook
# This hook runs the state manager audit before allowing commits

echo "🔍 Running FEAGI State Manager Audit..."

# Change to feagi_core directory
cd "$(git rev-parse --show-toplevel)/feagi_core" || {
    echo "❌ Could not find feagi_core directory"
    exit 1
}

# Run the audit script (excluding tests to focus on production code)
python3 scripts/state_manager_audit.py --exclude-tests

audit_result=$?

if [ $audit_result -eq 0 ]; then
    echo "✅ State management audit passed!"
    echo ""
elif [ $audit_result -eq 1 ]; then
    echo ""
    echo "❌ STATE MANAGEMENT VIOLATIONS DETECTED!"
    echo ""
    echo "🚨 Critical state management issues found that could cause:"
    echo "   • FQ samplers initializing before prerequisites are met"
    echo "   • Brain readiness set inconsistently"
    echo "   • Genome state bypassing validation"
    echo "   • Agent registration chaos"
    echo ""
    echo "🔧 To fix these issues:"
    echo "   1. Run: python3 scripts/state_manager_audit.py --verbose"
    echo "   2. Replace direct assignments with proper setter methods"
    echo "   3. Use state_manager.set_*() methods for all state changes"
    echo "   4. Re-run this audit to verify fixes"
    echo ""
    echo "📚 See docs/state_management_audit_analysis.md for detailed guidance"
    echo ""
    echo "💡 To temporarily bypass this check (NOT RECOMMENDED):"
    echo "   git commit --no-verify"
    echo ""
    exit 1
else
    echo "❌ State manager audit script failed to run"
    echo "Please check the script and try again"
    exit 1
fi

echo "🎉 All state management checks passed - proceeding with commit"
'''
    
    # Write the hook
    try:
        with open(hook_path, 'w') as f:
            f.write(hook_content)
        
        # Make it executable
        os.chmod(hook_path, stat.S_IRWXU | stat.S_IRGRP | stat.S_IROTH)
        
        print(f"✅ Pre-commit hook installed: {hook_path}")
        return True
        
    except Exception as e:
        print(f"❌ Failed to create pre-commit hook: {e}")
        return False

def create_commit_msg_hook(git_hooks_dir: Path, force: bool = False) -> bool:
    """Create a commit message hook to add state management context."""
    
    hook_path = git_hooks_dir / "commit-msg"
    
    if hook_path.exists() and not force:
        print(f"⚠️  Commit-msg hook already exists: {hook_path}")
        print("Use --force to overwrite existing hook")
        return False
    
    hook_content = '''#!/bin/bash
# FEAGI State Manager Commit Message Hook
# Adds context about state management changes

commit_msg_file="$1"

# Check if this commit includes state management changes
if git diff --cached --name-only | grep -E "(state_manager|genome_service|brain_service|agents_service)" > /dev/null; then
    echo ""
    echo "🔍 This commit includes state management changes"
    echo "📋 Please ensure your commit message includes:"
    echo "   • What state management issue was fixed"
    echo "   • Which services were affected"
    echo "   • Whether state transitions were validated"
    echo ""
fi
'''
    
    try:
        with open(hook_path, 'w') as f:
            f.write(hook_content)
        
        # Make it executable
        os.chmod(hook_path, stat.S_IRWXU | stat.S_IRGRP | stat.S_IROTH)
        
        print(f"✅ Commit-msg hook installed: {hook_path}")
        return True
        
    except Exception as e:
        print(f"❌ Failed to create commit-msg hook: {e}")
        return False

def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Install FEAGI State Manager Pre-commit Hooks"
    )
    parser.add_argument(
        "--force", 
        action="store_true",
        help="Overwrite existing hooks"
    )
    
    args = parser.parse_args()
    
    # Find git repository root
    current_dir = Path(__file__).parent.parent
    git_dir = None
    
    # Search up the directory tree for .git
    search_dir = current_dir
    for _ in range(10):  # Limit search depth
        if (search_dir / ".git").exists():
            git_dir = search_dir / ".git"
            break
        search_dir = search_dir.parent
    
    if not git_dir:
        print("❌ Could not find .git directory")
        print("Please run this script from within a git repository")
        return 1
    
    # Create hooks directory if it doesn't exist
    hooks_dir = git_dir / "hooks"
    hooks_dir.mkdir(exist_ok=True)
    
    print(f"📁 Git hooks directory: {hooks_dir}")
    print("")
    
    success = True
    
    # Install pre-commit hook
    if not create_pre_commit_hook(hooks_dir, args.force):
        success = False
    
    # Install commit-msg hook
    if not create_commit_msg_hook(hooks_dir, args.force):
        success = False
    
    if success:
        print("")
        print("🎉 FEAGI State Manager hooks installed successfully!")
        print("")
        print("📋 What happens now:")
        print("   • Every commit will be checked for state management violations")
        print("   • Commits with violations will be blocked")
        print("   • You'll get helpful guidance on how to fix issues")
        print("")
        print("🧪 To test the hook:")
        print("   1. Make a small change to a file")
        print("   2. Run: git add . && git commit -m 'test commit'")
        print("   3. The audit should run automatically")
        print("")
        print("⚙️  To temporarily bypass (emergency only):")
        print("   git commit --no-verify")
        print("")
    else:
        print("❌ Some hooks failed to install")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 