#!/usr/bin/env python3
"""
Test script for the Neuroembryogenesis module.

This script demonstrates how to use the Neuroembryogenesis module to develop
a brain from a genome file.
"""

import os
import sys
from feagi.utils.logger import setup_logger
logger = setup_logger()
import datetime
from pathlib import Path
import traceback

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

# Add the project root to the path if needed
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.insert(0, project_root)

# Try importing our dependencies, with helpful error messages
try:
    from feagi.bdu.connectome_manager import ConnectomeManager
    from feagi.bdu.neuroembryogenesis import Neuroembryogenesis, DevelopmentStage
    from feagi.utils.config import FeagiConfig
except ImportError as e:
    print(f"Error importing dependencies: {e}")
    print("Make sure you're running this script from the FEAGI project root or that PYTHONPATH is set correctly.")
    traceback.print_exc()
    sys.exit(1)


def progress_callback(stage: DevelopmentStage, progress: float, message: str):
    """Simple callback to display development progress."""
    stage_name = stage.value.upper()
    print(f"[{stage_name}] {progress:.1f}% - {message}")


def find_genome_file():
    """Find the essential_genome.json file in various potential locations."""
    possible_paths = [
        # Direct path from current directory
        Path("feagi/evo/defaults/genome/essential_genome.json"),
        
        # From project root
        Path(project_root) / "feagi/evo/defaults/genome/essential_genome.json",
        
        # From one directory up
        Path("../feagi/evo/defaults/genome/essential_genome.json"),
        
        # Other possible locations
        Path(project_root) / "evo/defaults/genome/essential_genome.json",
        Path("./evo/defaults/genome/essential_genome.json")
    ]
    
    for path in possible_paths:
        if path.exists():
            return path
            
    return None


def main():
    """Main entry point for the test script."""
    # Create output directory for connectome data
    output_dir = Path("./output/connectome")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Find the genome file
    genome_path = find_genome_file()
    
    if genome_path is None:
        print("Error: Could not find essential_genome.json file. Please specify the path manually.")
        return 1
        
    print(f"Using genome: {genome_path}")
    
    # Configure FEAGI
    config_dict = {
        "connectome_path": str(output_dir),
        "skip_memory_neurogenesis": False,  # Include memory areas in initial development
    }
    
    try:
        # Try different ways to initialize FeagiConfig depending on its implementation
        try:
            config = FeagiConfig(**config_dict)
        except TypeError:
            try:
                config = FeagiConfig(config_dict)
            except TypeError:
                config = FeagiConfig()
                for key, value in config_dict.items():
                    setattr(config, key, value)
    except Exception as e:
        print(f"Warning: Error creating FeagiConfig: {e}")
        print("Using default configuration.")
        config = None
    
    # Create the connectome manager
    print("Creating connectome manager...")
    try:
        connectome_manager = ConnectomeManager(config)
    except Exception as e:
        print(f"Error creating ConnectomeManager: {e}")
        traceback.print_exc()
        return 1
    
    # Create the neuroembryogenesis instance
    print("Initializing neuroembryogenesis...")
    try:
        embryo = Neuroembryogenesis(
            connectome_manager=connectome_manager,
            config=config,
            progress_callback=progress_callback
        )
    except Exception as e:
        print(f"Error creating Neuroembryogenesis: {e}")
        traceback.print_exc()
        return 1
    
    # Start timer
    start_time = datetime.datetime.now()
    print(f"Starting brain development at {start_time}")
    
    # Develop the brain
    try:
        success = embryo.develop_brain(genome_path)
    except Exception as e:
        print(f"Unhandled error during brain development: {e}")
        traceback.print_exc()
        return 1
    
    # End timer
    end_time = datetime.datetime.now()
    duration = end_time - start_time
    
    # Report results
    if success:
        stats = embryo.get_development_statistics()
        print("\nBrain development completed successfully!")
        print(f"Development took {duration}")
        print(f"Created {stats['cortical_areas']} cortical areas")
        print(f"Created {stats['neurons']} neurons")
        print(f"Created {stats['synapses']} synapses")
        
        # Save the connectome state
        save_path = output_dir / "brain_state.npz"
        print(f"Saving brain state to {save_path}")
        try:
            connectome_manager.serialize_brain_state(str(save_path))
            print(f"Brain state saved to {save_path}")
        except Exception as e:
            print(f"Warning: Could not save brain state: {e}")
            traceback.print_exc()
        
        return 0
    else:
        print(f"\nBrain development failed: {embryo.error}")
        return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"Unhandled exception: {e}")
        traceback.print_exc()
        sys.exit(1) 