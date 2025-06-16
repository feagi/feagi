# Neuroembryogenesis Module

This module is responsible for reading instructions from the genome (genotype) and translating them into a functional connectome (phenotype). The process is biologically inspired by neuroembryogenesis, where genetic instructions guide brain development from the embryonic neural tube.

## Key Components

1. **Corticogenesis** - Creation of cortical area definitions
2. **Voxelogenesis** - Establishing the 3D spatial framework for neuron placement
3. **Neurogenesis** - Generation of neurons within cortical areas
4. **Synaptogenesis** - Formation of synaptic connections between neurons

## Usage

```python
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis
from feagi.bdu.connectome_manager import ConnectomeManager

# Create connectome manager
connectome_manager = ConnectomeManager()

# Create embryogenesis system
embryo = NeuroEmbryogenesis(connectome_manager)

# Develop brain from genome
success = embryo.develop_brain("path/to/genome.json")
```

## Logging Configuration

The embryogenesis process can generate verbose logging during development, particularly "No mappings found" messages during synaptogenesis. You can control this verbosity in several ways:

### Environment Variables

```bash
# Suppress "No mappings found" messages
export FEAGI_EMBRYOGENESIS_QUIET=true

# Disable all embryogenesis verbose logging
export FEAGI_EMBRYOGENESIS_VERBOSE=false

# Run your application
python your_app.py
```

### Configuration Object

```python
from feagi.utils.config import FeagiConfig

# Create config with embryogenesis settings
config = FeagiConfig()
config.embryogenesis_verbose_logging = False
config.embryogenesis_suppress_no_mappings_logs = True

# Pass to embryogenesis
embryo = NeuroEmbryogenesis(connectome_manager, config=config)
```

### For Testing

When running tests, you can suppress verbose embryogenesis logs:

```bash
# Suppress during testing
FEAGI_EMBRYOGENESIS_QUIET=true python -m pytest tests/

# Or for specific tests
FEAGI_EMBRYOGENESIS_QUIET=true python -m pytest tests/npu/test_burst_engine_complete.py -v
```

## Naming Convention

* **cortical_id**: 6-character unique identifier from the genome (e.g., "iv00_C")
* **cortical_idx**: Auto-incremented integer ID used internally for efficient indexing

The module maintains mappings between these two ID types:
1. `cortical_id_map[cortical_idx] = cortical_id`
2. `reverse_cortical_id_map[cortical_id] = cortical_idx`

## RTOS/Rust Compatibility

This module has been designed with future RTOS and Rust migration in mind:

- Uses static typing throughout
- Minimizes dynamic behavior
- Avoids dependencies incompatible with embedded systems
- Implements fail-safe defaults and meaningful error messages

## Performance Considerations

- The module uses the ConnectomeManager API for efficient neuron and synapse management
- Focuses on memory efficiency and thread-safety
- Supports batch operations for large-scale brain development
- Includes development statistics tracking for performance monitoring
