#!/usr/bin/env python3
"""
Manual Debug Trigger for Refractory Period Bug

This script will help enable debug logging and trigger the bug for observation.
Run this while FEAGI is running to see the debug output.
"""

import sys
import os

# Add the current directory to Python path so we can import FEAGI modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def enable_debug_and_test():
    """Enable debug logging and test refractory behavior."""
    print("🔬 Manual Refractory Period Debug Trigger")
    print("=" * 50)
    
    try:
        # Try to import and access the FEAGI components
        from feagi.core.state_manager import get_state_manager
        
        # Get the state manager instance
        state_manager = get_state_manager()
        if not state_manager:
            print("❌ Could not get FEAGI state manager")
            print("💡 Make sure FEAGI is running first")
            return
        
        print("✅ Connected to FEAGI state manager")
        
        # Try to get the connectome manager
        # This might be available through the state manager or burst engine
        if hasattr(state_manager, 'connectome_manager'):
            connectome_manager = state_manager.connectome_manager
        else:
            print("⚠️  Connectome manager not directly accessible via state manager")
            print("💡 Trying alternative access method...")
            
            # Try to get it through the burst engine
            if hasattr(state_manager, 'burst_engine'):
                burst_engine = state_manager.burst_engine
                if hasattr(burst_engine, 'connectome_manager'):
                    connectome_manager = burst_engine.connectome_manager
                else:
                    print("❌ Could not access connectome manager")
                    return
            else:
                print("❌ Could not access burst engine")
                return
        
        print("✅ Got connectome manager")
        
        # Enable debug logging
        print("\n🔬 Enabling refractory period debug logging...")
        connectome_manager.enable_refractory_debug_logging()
        
        print("\n🎯 Debug logging is now ENABLED!")
        print("=" * 50)
        print("📊 NEXT STEPS:")
        print("1. Use your visualization tool to stimulate neurons")
        print("2. Watch the FEAGI console output for debug messages")
        print("3. Look for '🔥 NEURON DEBUG:' messages when neurons fire")
        print("4. Look for '🚨 BUG DETECTED:' if area-wide suppression occurs")
        print("\n💡 WHAT TO STIMULATE:")
        print("- Power area (___pwr) - but it only has 1 neuron")
        print("- Any cortical area with multiple neurons")
        print("- Motor areas (m__rig, m__lef, m__for, m__bac)")
        print("- Visual areas (iv00BR, iv00_C, etc.)")
        
        print("\n⚠️  TO DISABLE DEBUG: Run this script again or restart FEAGI")
        
    except ImportError as e:
        print(f"❌ Could not import FEAGI modules: {e}")
        print("💡 Make sure you're running this from the FEAGI directory")
        print("💡 And make sure FEAGI is installed and running")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        print("💡 This might mean FEAGI is not running or not initialized")

def main():
    """Main function."""
    enable_debug_and_test()

if __name__ == "__main__":
    main() 