#!/usr/bin/env python3
"""
Capabilities JSON Migration Script - Old Format → New Format

Converts old flat capabilities.json format to new hierarchical format with
channel arrays and pipeline stage support.

This is an OFFLINE tool - NOT integrated with runtime code.

Usage:
    python migrate_capabilities_json.py --input old_caps.json --output new_caps.json
    python migrate_capabilities_json.py --input old_caps.json --output new_caps.json --verbose
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List


def migrate_sensor_config(old_config: Dict[str, Any]) -> Dict[str, Any]:
    """
    Convert old sensor configuration to new format
    
    Old format:
        {
          "custom_name": "accelerometer",
          "disabled": false,
          "feagi_index": 0,
          "max_value": [1023, 1023, 1023],
          "min_value": [-1023, -1023, -1023]
        }
    
    New format:
        {
          "friendly_name": "accelerometer",
          "channels": [
            {
              "friendly_name": "Channel 0",
              "channel_index_override": null,
              "pipeline_stages": []
            }
          ]
        }
    """
    # Extract friendly name
    friendly_name = old_config.get('custom_name', old_config.get('name', 'Unknown Device'))
    
    # Determine number of channels (usually 1 for old format)
    # Could be inferred from feagi_index or assume 1
    feagi_index = old_config.get('feagi_index', 0)
    
    # Create single channel (old format typically had 1 channel per device)
    channels = [
        {
            "friendly_name": f"Channel {feagi_index}",
            "channel_index_override": feagi_index if feagi_index != 0 else None,
            "pipeline_stages": []  # No pipelines in old format
        }
    ]
    
    new_config = {
        "friendly_name": friendly_name,
        "channels": channels
    }
    
    return new_config


def migrate_motor_config(old_config: Dict[str, Any]) -> Dict[str, Any]:
    """
    Convert old motor configuration to new format
    
    Same structure as sensors
    """
    return migrate_sensor_config(old_config)


def migrate_capabilities(old_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Migrate entire capabilities structure
    
    Args:
        old_data: Old format capabilities JSON
        
    Returns:
        New format capabilities JSON
    """
    if 'capabilities' not in old_data:
        raise ValueError("Invalid JSON: missing 'capabilities' key")
    
    old_caps = old_data['capabilities']
    new_caps = {
        "input": {},
        "output": {}
    }
    
    # Migrate input (sensors)
    if 'input' in old_caps:
        for sensor_type, groups in old_caps['input'].items():
            new_caps['input'][sensor_type] = {}
            
            for group_id, old_config in groups.items():
                new_caps['input'][sensor_type][group_id] = migrate_sensor_config(old_config)
    
    # Migrate output (motors)
    if 'output' in old_caps:
        for motor_type, groups in old_caps['output'].items():
            new_caps['output'][motor_type] = {}
            
            for group_id, old_config in groups.items():
                new_caps['output'][motor_type][group_id] = migrate_motor_config(old_config)
    
    return {"capabilities": new_caps}


def detect_format(data: Dict[str, Any]) -> str:
    """
    Detect if JSON is old or new format
    
    Returns:
        'old', 'new', or 'unknown'
    """
    try:
        caps = data.get('capabilities', {})
        input_data = caps.get('input', {})
        
        for sensor_type, groups in input_data.items():
            for group_id, config in groups.items():
                # Old format indicators
                if 'feagi_index' in config:
                    return 'old'
                if 'custom_name' in config and 'channels' not in config:
                    return 'old'
                
                # New format indicators
                if 'channels' in config and isinstance(config['channels'], list):
                    return 'new'
                    
        return 'unknown'
    except:
        return 'unknown'


def main():
    parser = argparse.ArgumentParser(
        description='Migrate capabilities.json from old format to new format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic migration
  python migrate_capabilities_json.py --input old.json --output new.json
  
  # With verbose output
  python migrate_capabilities_json.py --input old.json --output new.json --verbose
  
  # Check format without converting
  python migrate_capabilities_json.py --input caps.json --check-only
        """
    )
    
    parser.add_argument('--input', '-i', required=True, type=Path,
                        help='Input capabilities.json file (old format)')
    parser.add_argument('--output', '-o', type=Path,
                        help='Output capabilities.json file (new format)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Print detailed migration information')
    parser.add_argument('--check-only', action='store_true',
                        help='Only check format, do not convert')
    
    args = parser.parse_args()
    
    # Validate input file exists
    if not args.input.exists():
        print(f"Error: Input file not found: {args.input}", file=sys.stderr)
        return 1
    
    # Load input JSON
    try:
        with open(args.input, 'r') as f:
            old_data = json.load(f)
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON in input file: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Error reading input file: {e}", file=sys.stderr)
        return 1
    
    # Detect format
    format_type = detect_format(old_data)
    
    if args.verbose or args.check_only:
        print(f"Input file: {args.input}")
        print(f"Detected format: {format_type}")
    
    if format_type == 'new':
        print("File is already in new format - no migration needed!")
        return 0
    
    if format_type == 'unknown':
        print("Warning: Could not determine format. Attempting migration anyway...")
    
    if args.check_only:
        if format_type == 'old':
            print("File needs migration to new format")
            return 2  # Exit code 2 = needs migration
        return 0
    
    # Require output file for actual migration
    if not args.output:
        print("Error: --output required for migration", file=sys.stderr)
        return 1
    
    # Perform migration
    try:
        if args.verbose:
            print("Migrating...")
        
        new_data = migrate_capabilities(old_data)
        
        if args.verbose:
            # Count devices
            input_count = sum(len(groups) for groups in new_data['capabilities']['input'].values())
            output_count = sum(len(groups) for groups in new_data['capabilities']['output'].values())
            print(f"   Migrated {input_count} input devices")
            print(f"   Migrated {output_count} output devices")
        
    except Exception as e:
        print(f"Error during migration: {e}", file=sys.stderr)
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1
    
    # Write output JSON
    try:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with open(args.output, 'w') as f:
            json.dump(new_data, f, indent=2)
        
        print(f"Migration successful!")
        print(f"   Output written to: {args.output}")
        
        if args.verbose:
            print(f"\nManual steps required:")
            print(f"   1. Review the migrated JSON")
            print(f"   2. Add pipeline_stages if needed")
            print(f"   3. Update channel friendly_names")
            print(f"   4. Test with your agent")
        
    except Exception as e:
        print(f"Error writing output file: {e}", file=sys.stderr)
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

