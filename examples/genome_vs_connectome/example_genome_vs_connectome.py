"""
FEAGI Engine: Genome vs Connectome

This example demonstrates the difference between loading a genome
and loading a connectome.

Genome:
    - Initial neural structure (blueprint)
    - Start fresh with untrained brain
    - Use for new experiments

Connectome:
    - Trained neural state (saved brain)
    - Resume from saved state
    - Use to continue training or deploy trained models

Usage:
    python example_genome_vs_connectome.py
"""

import os
from pathlib import Path

from feagi.engine import FeagiEngine


def _config_path() -> Path:
    env_path = os.environ.get("FEAGI_CONFIG_PATH")
    if env_path:
        return Path(env_path)
    here = Path(__file__).resolve().parent
    local = here / "feagi_configuration.toml"
    if local.is_file():
        return local
    return here.parent / "feagi_configuration.toml"


CONFIG_PATH = _config_path()

print("=" * 70)
print("FEAGI Engine: Genome vs Connectome")
print("=" * 70)

# ==============================================================================
# Option 1: Start with a Genome (fresh brain)
# ==============================================================================
print("\n" + "=" * 70)
print("Option 1: Load Genome (fresh neural structure)")
print("=" * 70)

engine = FeagiEngine()
engine.load_config(str(CONFIG_PATH))
engine.load_genome("my_genome.json")  # Start fresh!

print("\nGenome loaded")
print("   - Fresh neural structure")
print("   - No training/learning yet")
print("   - Ready for new experiments")

# Start FEAGI
# engine.start()

# Do training/learning...

# Save trained state to connectome
# engine.save_connectome("trained_brain.connectome")

# Stop FEAGI
# engine.stop()

print("\nSkipping actual start for demonstration...")


# ==============================================================================
# Option 2: Load Connectome (trained brain)
# ==============================================================================
print("\n" + "=" * 70)
print("Option 2: Load Connectome (trained neural state)")
print("=" * 70)

engine2 = FeagiEngine()
engine2.load_config(str(CONFIG_PATH))
engine2.load_connectome("trained_brain.connectome")  # Resume from saved state!

print("\nConnectome loaded")
print("   - Trained neural state")
print("   - All learned connections preserved")
print("   - Ready to continue or deploy")

# Start FEAGI
# engine2.start()

# Continue training or use for inference...

# Stop FEAGI
# engine2.stop()

print("\nSkipping actual start for demonstration...")


# ==============================================================================
# Mutually Exclusive: Genome OR Connectome (not both)
# ==============================================================================
print("\n" + "=" * 70)
print("Note: Genome and Connectome are Mutually Exclusive")
print("=" * 70)

engine3 = FeagiEngine()
engine3.load_config(str(CONFIG_PATH))
engine3.load_genome("genome1.json")      # Load genome first
print("\nGenome loaded")

engine3.load_connectome("brain.connectome")  # This will replace genome!
print("Connectome loaded - genome was cleared (mutually exclusive)")

# Only connectome will be used when starting
print("\nFinal state: Connectome will be used (genome was cleared)")


# ==============================================================================
# Use Case Examples
# ==============================================================================
print("\n" + "=" * 70)
print("Use Case Examples")
print("=" * 70)

print("""
1. New Experiment (Use Genome):
   engine.load_genome("experiment_v1.json")
   engine.start()
   # Train the neural network
   # Save result as connectome

2. Continue Training (Use Connectome):
   engine.load_connectome("experiment_v1_checkpoint.connectome")
   engine.start()
   # Continue training from saved state
   # Save updated connectome

3. Deploy Trained Model (Use Connectome):
   engine.load_connectome("production_model.connectome")
   engine.start()
   # Use trained brain for inference
   # No need to retrain

4. A/B Testing:
   # Test A: Fresh genome
   engine_a.load_genome("base_genome.json")

   # Test B: Trained connectome
   engine_b.load_connectome("trained_v2.connectome")
""")

print("=" * 70)
print("Summary")
print("=" * 70)
print("""
Genome (Blueprint):
  Use for: New experiments, fresh starts
  Contains: Neural structure definition
  State: Untrained

Connectome (Trained Brain):
  Use for: Resume training, deploy models
  Contains: Neural state + learned connections
  State: Trained/learned

Method Chaining:
  engine.load_config("config.toml").load_genome("genome.json").start()
  engine.load_config("config.toml").load_connectome("brain.connectome").start()
""")

print("=" * 70)
