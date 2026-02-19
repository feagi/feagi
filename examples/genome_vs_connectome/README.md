# Genome vs Connectome Example

Demonstrates the difference between loading a **genome** (initial neural blueprint) and a **connectome** (trained neural state) with `FeagiEngine`.

- **Genome**: fresh structure, new experiments.
- **Connectome**: saved state, resume training or deploy.

## Requirements

- Python 3.10+
- See `requirements.txt` in this folder.

## Configuration

Uses `feagi_configuration.toml` from this folder or parent `examples/`, or set `FEAGI_CONFIG_PATH`. Example uses placeholder paths (`my_genome.json`, `trained_brain.connectome`); replace with your files to run for real.

## Run

From this folder:

```bash
python example_genome_vs_connectome.py
```

Engine start/stop are commented out for demonstration; uncomment and set genome/connectome paths to run FEAGI.
