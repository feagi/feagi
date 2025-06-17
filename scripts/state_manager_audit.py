#!/usr/bin/env python3
"""
FEAGI State Manager Audit Script

This script detects potential state management bypasses throughout the FEAGI codebase.
It ensures architectural integrity by identifying direct state changes that don't go 
through the centralized state manager.

Usage:
    python scripts/state_manager_audit.py [--fix] [--verbose] [--exclude-tests]
    
Exit codes:
    0: No violations found
    1: Violations found
    2: Script error
"""

import os
import re
import sys
import argparse
from pathlib import Path
from typing import List, Dict, Set, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

class ViolationType(Enum):
    """Types of state management violations."""
    DIRECT_ATTRIBUTE_ASSIGNMENT = "direct_attribute_assignment"
    BYPASS_SETTER_METHOD = "bypass_setter_method"
    DIRECT_STATE_PTR_ACCESS = "direct_state_ptr_access"
    MISSING_STATE_MANAGER_USAGE = "missing_state_manager_usage"
    INCONSISTENT_STATE_CHANGE = "inconsistent_state_change"
    UNAUTHORIZED_STATE_ACCESS = "unauthorized_state_access"

@dataclass
class Violation:
    """Represents a state management violation."""
    file_path: str
    line_number: int
    line_content: str
    violation_type: ViolationType
    severity: str  # "CRITICAL", "HIGH", "MEDIUM", "LOW"
    description: str
    suggested_fix: Optional[str] = None
    context_lines: List[str] = None

class StateManagerAuditor:
    """Audits FEAGI codebase for state management violations."""
    
    def __init__(self, feagi_root: Path, exclude_tests: bool = False, verbose: bool = False):
        self.feagi_root = feagi_root
        self.exclude_tests = exclude_tests
        self.verbose = verbose
        self.violations: List[Violation] = []
        
        # Critical state attributes that must go through state manager
        self.critical_state_attributes = {
            'genome_state', 'brain_readiness', 'burst_engine_state', 'fq_sampler_state',
            'connectome_state', 'api_state', 'zmq_state', 'simulation_state',
            'exit_condition', 'genome_validity', 'brain_stats', 'cortical_list',
            'connected_agents', 'agent_count', 'burst_frequency', 'genome_timestamp'
        }
        
        # State manager setter methods (authorized ways to change state)
        self.authorized_setters = {
            'set_genome_state', 'set_brain_readiness', 'set_burst_engine_state',
            'set_fq_sampler_state', 'set_connectome_state', 'set_api_state',
            'set_zmq_state', 'set_simulation_state', 'set_burst_frequency',
            'set_genome_timestamp', 'set_agent_count', 'register_agent',
            'deregister_agent', 'update_agent_registry'
        }
        
        # Files that are allowed to directly modify state (state manager itself)
        self.authorized_files = {
            'feagi/core/state_manager.py',
            'tests/'  # Test files have more flexibility
        }
        
        # Patterns for detecting violations
        self.violation_patterns = self._compile_patterns()
    
    def _compile_patterns(self) -> Dict[ViolationType, List[re.Pattern]]:
        """Compile regex patterns for detecting violations."""
        patterns = {
            ViolationType.DIRECT_ATTRIBUTE_ASSIGNMENT: [
                # Direct assignment to state manager attributes
                re.compile(r'(?:self\.)?state_manager\.(' + '|'.join(self.critical_state_attributes) + r')\s*=', re.IGNORECASE),
                # Direct assignment to state_ptr contents
                re.compile(r'\.state_ptr\.contents\.(' + '|'.join(self.critical_state_attributes) + r')\s*=', re.IGNORECASE),
            ],
            ViolationType.BYPASS_SETTER_METHOD: [
                # Using attribute access instead of setter methods
                re.compile(r'(?:self\.)?state_manager\.(' + '|'.join(self.critical_state_attributes) + r')\s*=(?!\s*(?:self\.)?state_manager\.get_)', re.IGNORECASE),
            ],
            ViolationType.DIRECT_STATE_PTR_ACCESS: [
                # Direct state_ptr manipulation outside state manager
                re.compile(r'\.state_ptr\.contents\.[a-zA-Z_]+\s*=', re.IGNORECASE),
            ],
            ViolationType.UNAUTHORIZED_STATE_ACCESS: [
                # State changes in unauthorized contexts
                re.compile(r'(?:genome_state|brain_readiness|burst_engine_state)\s*=\s*(?:True|False|[A-Z_]+)', re.IGNORECASE),
            ]
        }
        return patterns
    
    def audit_codebase(self) -> List[Violation]:
        """Audit the entire FEAGI codebase for state management violations."""
        print("🔍 Starting FEAGI State Manager Audit...")
        
        # Find all Python files
        python_files = self._find_python_files()
        
        print(f"📁 Found {len(python_files)} Python files to audit")
        
        # Audit each file
        for file_path in python_files:
            try:
                self._audit_file(file_path)
            except Exception as e:
                print(f"❌ Error auditing {file_path}: {e}")
                if self.verbose:
                    import traceback
                    traceback.print_exc()
        
        # Sort violations by severity and file
        self.violations.sort(key=lambda v: (v.severity, v.file_path, v.line_number))
        
        return self.violations
    
    def _find_python_files(self) -> List[Path]:
        """Find all Python files in the FEAGI codebase."""
        python_files = []
        
        # Search patterns
        search_dirs = [
            self.feagi_root / "feagi",
            self.feagi_root / "scripts",
        ]
        
        if not self.exclude_tests:
            search_dirs.append(self.feagi_root / "tests")
        
        for search_dir in search_dirs:
            if search_dir.exists():
                for py_file in search_dir.rglob("*.py"):
                    # Skip __pycache__ and other generated files
                    if "__pycache__" not in str(py_file) and ".egg-info" not in str(py_file):
                        python_files.append(py_file)
        
        return python_files
    
    def _audit_file(self, file_path: Path):
        """Audit a single Python file for state management violations."""
        if self.verbose:
            print(f"🔍 Auditing: {file_path}")
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
        except (UnicodeDecodeError, IOError) as e:
            if self.verbose:
                print(f"⚠️  Skipping {file_path}: {e}")
            return
        
        # Check if this file is authorized to modify state directly
        relative_path = str(file_path.relative_to(self.feagi_root))
        is_authorized = any(auth_path in relative_path for auth_path in self.authorized_files)
        
        # Audit each line
        for line_num, line in enumerate(lines, 1):
            line_stripped = line.strip()
            if not line_stripped or line_stripped.startswith('#'):
                continue
            
            # Check for violations
            self._check_line_for_violations(
                file_path, line_num, line, line_stripped, is_authorized, lines
            )
    
    def _check_line_for_violations(
        self, 
        file_path: Path, 
        line_num: int, 
        line: str, 
        line_stripped: str, 
        is_authorized: bool,
        all_lines: List[str]
    ):
        """Check a single line for state management violations."""
        
        # Skip test files if they're doing mock setup
        if "test" in str(file_path).lower() and ("mock" in line_stripped.lower() or "Mock(" in line):
            return
        
        # Check each violation type
        for violation_type, patterns in self.violation_patterns.items():
            for pattern in patterns:
                match = pattern.search(line_stripped)
                if match:
                    # Determine severity and create violation
                    severity = self._determine_severity(violation_type, line_stripped, is_authorized)
                    
                    # Skip if it's authorized and low severity
                    if is_authorized and severity == "LOW":
                        continue
                    
                    # Get context lines
                    context_lines = self._get_context_lines(all_lines, line_num)
                    
                    # Create violation
                    violation = Violation(
                        file_path=str(file_path.relative_to(self.feagi_root)),
                        line_number=line_num,
                        line_content=line_stripped,
                        violation_type=violation_type,
                        severity=severity,
                        description=self._get_violation_description(violation_type, match, line_stripped),
                        suggested_fix=self._get_suggested_fix(violation_type, match, line_stripped),
                        context_lines=context_lines
                    )
                    
                    self.violations.append(violation)
    
    def _determine_severity(self, violation_type: ViolationType, line: str, is_authorized: bool) -> str:
        """Determine the severity of a violation."""
        
        # Critical violations that break architecture
        if violation_type == ViolationType.DIRECT_STATE_PTR_ACCESS and not is_authorized:
            return "CRITICAL"
        
        # High severity for core state bypasses
        if violation_type == ViolationType.DIRECT_ATTRIBUTE_ASSIGNMENT:
            if any(attr in line for attr in ['brain_readiness', 'genome_state', 'burst_engine_state']):
                return "CRITICAL" if not is_authorized else "HIGH"
            return "HIGH"
        
        # Medium severity for setter bypasses
        if violation_type == ViolationType.BYPASS_SETTER_METHOD:
            return "MEDIUM"
        
        # Default to medium
        return "MEDIUM"
    
    def _get_context_lines(self, all_lines: List[str], line_num: int, context: int = 3) -> List[str]:
        """Get context lines around a violation."""
        start = max(0, line_num - context - 1)
        end = min(len(all_lines), line_num + context)
        return [f"{i+1:4d}: {all_lines[i].rstrip()}" for i in range(start, end)]
    
    def _get_violation_description(self, violation_type: ViolationType, match: re.Match, line: str) -> str:
        """Get a description of the violation."""
        try:
            if violation_type == ViolationType.DIRECT_ATTRIBUTE_ASSIGNMENT:
                attr_name = match.group(1) if match.groups() else "unknown"
                return f"Direct assignment to state attribute '{attr_name}' bypasses state manager"
            elif violation_type == ViolationType.BYPASS_SETTER_METHOD:
                attr_name = match.group(1) if match.groups() else "unknown"
                return f"State change bypasses authorized setter method for '{attr_name}'"
            elif violation_type == ViolationType.DIRECT_STATE_PTR_ACCESS:
                return "Direct manipulation of state_ptr.contents bypasses state manager"
            elif violation_type == ViolationType.UNAUTHORIZED_STATE_ACCESS:
                return "State change in unauthorized context"
            else:
                return "Unknown violation type"
        except IndexError:
            return f"State management violation in line: {line[:50]}..."
    
    def _get_suggested_fix(self, violation_type: ViolationType, match: re.Match, line: str) -> Optional[str]:
        """Get a suggested fix for the violation."""
        
        if violation_type == ViolationType.DIRECT_ATTRIBUTE_ASSIGNMENT:
            attr_name = match.group(1)
            setter_mapping = {
                'genome_state': 'set_genome_state',
                'brain_readiness': 'set_brain_readiness',
                'burst_engine_state': 'set_burst_engine_state',
                'fq_sampler_state': 'set_fq_sampler_state',
                'connectome_state': 'set_connectome_state',
                'api_state': 'set_api_state',
                'zmq_state': 'set_zmq_state',
                'simulation_state': 'set_simulation_state',
                'burst_frequency': 'set_burst_frequency',
                'genome_timestamp': 'set_genome_timestamp',
                'agent_count': 'set_agent_count',
            }
            
            if attr_name in setter_mapping:
                return f"Use state_manager.{setter_mapping[attr_name]}(value) instead"
        
        return None
    
    def print_report(self):
        """Print a comprehensive audit report."""
        print("\n" + "="*80)
        print("🔍 FEAGI STATE MANAGER AUDIT REPORT")
        print("="*80)
        
        if not self.violations:
            print("✅ No state management violations found!")
            print("🎉 Architecture integrity maintained!")
            return
        
        # Summary by severity
        severity_counts = {}
        for violation in self.violations:
            severity_counts[violation.severity] = severity_counts.get(violation.severity, 0) + 1
        
        print(f"\n📊 SUMMARY:")
        print(f"   Total violations: {len(self.violations)}")
        for severity in ["CRITICAL", "HIGH", "MEDIUM", "LOW"]:
            count = severity_counts.get(severity, 0)
            if count > 0:
                emoji = {"CRITICAL": "🚨", "HIGH": "⚠️", "MEDIUM": "⚡", "LOW": "💡"}[severity]
                print(f"   {emoji} {severity}: {count}")
        
        # Group by violation type
        type_counts = {}
        for violation in self.violations:
            type_counts[violation.violation_type] = type_counts.get(violation.violation_type, 0) + 1
        
        print(f"\n🏷️  VIOLATION TYPES:")
        for vtype, count in type_counts.items():
            print(f"   • {vtype.value.replace('_', ' ').title()}: {count}")
        
        # Detailed violations
        print(f"\n📋 DETAILED VIOLATIONS:")
        print("-" * 80)
        
        current_file = None
        for violation in self.violations:
            if violation.file_path != current_file:
                current_file = violation.file_path
                print(f"\n📁 {current_file}")
            
            # Severity emoji
            severity_emoji = {
                "CRITICAL": "🚨",
                "HIGH": "⚠️", 
                "MEDIUM": "⚡",
                "LOW": "💡"
            }[violation.severity]
            
            print(f"   {severity_emoji} Line {violation.line_number}: {violation.description}")
            print(f"      Code: {violation.line_content}")
            
            if violation.suggested_fix:
                print(f"      Fix:  {violation.suggested_fix}")
            
            if self.verbose and violation.context_lines:
                print("      Context:")
                for context_line in violation.context_lines:
                    marker = ">>>" if str(violation.line_number) in context_line[:4] else "   "
                    print(f"      {marker} {context_line}")
            print()
    
    def get_exit_code(self) -> int:
        """Get appropriate exit code based on violations found."""
        if not self.violations:
            return 0
        
        # Check for critical violations
        critical_violations = [v for v in self.violations if v.severity == "CRITICAL"]
        if critical_violations:
            return 1
        
        # Check for high severity violations
        high_violations = [v for v in self.violations if v.severity == "HIGH"]
        if high_violations:
            return 1
        
        # Medium/Low violations are warnings
        return 0

def main():
    """Main entry point for the audit script."""
    parser = argparse.ArgumentParser(
        description="Audit FEAGI codebase for state management violations"
    )
    parser.add_argument(
        "--verbose", "-v", 
        action="store_true", 
        help="Enable verbose output with context lines"
    )
    parser.add_argument(
        "--exclude-tests", 
        action="store_true", 
        help="Exclude test files from audit"
    )
    parser.add_argument(
        "--feagi-root", 
        type=Path, 
        default=Path(__file__).parent.parent,
        help="Path to FEAGI root directory"
    )
    
    args = parser.parse_args()
    
    # Validate FEAGI root
    if not args.feagi_root.exists():
        print(f"❌ FEAGI root directory not found: {args.feagi_root}")
        return 2
    
    if not (args.feagi_root / "feagi").exists():
        print(f"❌ Invalid FEAGI root (no 'feagi' directory): {args.feagi_root}")
        return 2
    
    try:
        # Create auditor and run audit
        auditor = StateManagerAuditor(
            feagi_root=args.feagi_root,
            exclude_tests=args.exclude_tests,
            verbose=args.verbose
        )
        
        # Run audit
        violations = auditor.audit_codebase()
        
        # Print report
        auditor.print_report()
        
        # Additional recommendations
        if violations:
            print("\n💡 RECOMMENDATIONS:")
            print("   1. Use state_manager.set_*() methods for all state changes")
            print("   2. Never directly assign to state_manager attributes")
            print("   3. Avoid direct state_ptr.contents manipulation")
            print("   4. Ensure all state changes go through centralized state manager")
            print("   5. Use state manager getters for reading state")
            
            print(f"\n🔧 To fix these issues:")
            print(f"   1. Review each violation above")
            print(f"   2. Replace direct assignments with proper setter methods")
            print(f"   3. Re-run this script to verify fixes")
            print(f"   4. Consider adding this script to pre-commit hooks")
        
        return auditor.get_exit_code()
        
    except Exception as e:
        print(f"❌ Audit failed: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 2

if __name__ == "__main__":
    sys.exit(main()) 