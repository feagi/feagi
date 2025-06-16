#!/usr/bin/env python3
"""
Debug script to check FCL manager methods.
"""

from feagi.npu.fcl_manager import FCLManager


def main():
    print("=== FCL Manager Methods Debug ===")

    fcl_manager = FCLManager()
    print(f"FCL Manager type: {type(fcl_manager)}")
    print(f"FCL Manager class: {fcl_manager.__class__}")

    print("\nMethods available on FCL manager:")
    methods = [method for method in dir(fcl_manager) if not method.startswith("_")]
    for method in sorted(methods):
        print(f"  - {method}")

    print(f"\nHas update_fcl: {hasattr(fcl_manager, 'update_fcl')}")
    if hasattr(fcl_manager, "update_fcl"):
        print(f"update_fcl callable: {callable(fcl_manager.update_fcl)}")
        print(f"update_fcl type: {type(fcl_manager.update_fcl)}")

    print(f"Has add_to_current_fcl: {hasattr(fcl_manager, 'add_to_current_fcl')}")


if __name__ == "__main__":
    main()
