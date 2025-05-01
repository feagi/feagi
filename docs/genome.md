# FEAGI Genome System Documentation

## Overview

The FEAGI Genome is a structured representation of a brain's architecture, defining all cortical areas, connectivity patterns, and neuron properties. It serves as both a blueprint for brain initialization and a serializable record that can be saved, loaded, and evolved over time. 

The genome system consists of several components that work together to define, validate, process, and modify brain architectures through a structured genetic representation.

## Core Components

### 1. Genome Structure

A FEAGI genome is a hierarchical JSON structure with the following key sections:

- **blueprint**: Defines the cortical areas and their properties
- **neuron_morphologies**: Defines connection patterns between neurons
- **physiology**: Contains global parameters affecting neural dynamics
- **stats**: Tracks statistics about the brain architecture
- **signatures**: Contains checksums for different genome parts
- **version**: Specifies the genome format version (current: "2.0")

### 2. Gene Encoding Format

Genes in the FEAGI genome follow a structured naming convention with 5 segments separated by hyphens:

```
_____10c-cortex_id-cx-property-t
```

Where:
- Segment 1: Expression parameters
- Segment 2: Cortical area ID (6 characters)
- Segment 3: Gene classifier (nx = neuron property, cx = cortical property)
- Segment 4: Encoding ID (specific property identifier)
- Segment 5: Value type (b = boolean, i = integer, f = float, d = dictionary, t = text)

### 3. Cortical Areas

Each cortical area is defined with properties such as:
- Boundaries and position (relative_coordinate, block_boundaries)
- Neuron properties (firing_threshold, postsynaptic_current, etc.)
- Connectivity mappings to other areas (cortical_mapping_dst)
- Special properties (memory areas, visualization settings)

### 4. Neuron Morphologies

Morphologies define patterns for establishing connections between neurons in different cortical areas. Types include:

- **Vectors**: Algebraic vector-based connections
- **Patterns**: Pattern-based connections using special characters (* and ?)
- **Functions**: Special functions like expanders, reducers, and randomizers
- **Composite**: Combinations of other morphology types

## Key Modules

### 1. Genome Processor (`genome_processor.py`)

The primary module for handling genome operations, including:

- Converting between different genome versions
- Updating and standardizing genome components
- Processing morphology definitions
- Handling conversions between flat and hierarchical representations

Key functions:
- `genome_ver_check`: Validates and upgrades genome versions
- `genome_2_1_convertor`: Converts between genome formats
- `morphology_convertor`: Standardizes morphology definitions
- `genome_physiology_updator`: Updates physiological parameters

### 2. Genome Validator (`genome_validator.py`)

Ensures the genome follows the correct structure and references valid components:

- Validates morphology definitions
- Checks gene structure
- Validates connectivity rules and references
- Reports validation status with detailed error messages

Key functions:
- `morphology_validator`: Validates all morphology definitions
- `blueprint_validator`: Validates the blueprint structure
- `genome_validator`: Main entry point for validation

### 3. Genome Editor (`genome_editor.py`)

Provides utilities for modifying and saving genomes:

- Adding or modifying genes
- Generating cryptographic signatures
- Serializing and saving genome to files

Key functions:
- `save_genome`: Serializes and saves genome to a file
- `generate_hash`: Creates cryptographic signatures
- `add_gene`: Utility to add new genes to the genome

### 4. Genome Properties (`genome_properties.py`)

Defines constants and structural properties of the genome system:

- Segment formatting and separators
- Position indices for different parts of a gene
- Valid value types and classifiers

## Genome Evolution and Processing

The genome system supports evolution through:

1. **Version Conversion**: The system can convert between different genome formats
2. **Validation**: Ensures changes to the genome maintain structural integrity
3. **Morphology Management**: Standardizes and validates connection patterns
4. **Serialization**: Supports saving and loading complete brain architectures

## Special Features

### Multiple Neurons Per Voxel
The `per_voxel_neuron_cnt` property allows multiple neurons to exist in the same voxel, enabling higher neuron density in specific areas.

### Memory-Type Cortical Areas
Areas with the `is_mem_type` flag have special temporal properties, including:
- Extended temporal depth for memory storage
- Lifespan parameters for memory persistence
- Thresholds for long-term memory formation

### Cortical Area Types
The system supports different functional area types including:
- Input Processing Units (IPU)
- Output Processing Units (OPU)
- Core areas for processing
- Specialized memory areas

## Usage Examples

### Defining a New Cortical Area
In the genome blueprint, a cortical area is defined with properties like:

```json
"my_area": {
  "cortical_name": "My Processing Area",
  "block_boundaries": [10, 10, 5],
  "relative_coordinate": [0, 0, 0],
  "visualization": true,
  "firing_threshold": 1.0,
  "postsynaptic_current": 1.0,
  "refractory_period": 0,
}
```

### Defining Connectivity Between Areas
Connectivity uses morphologies to define connection patterns:

```json
"cortical_mapping_dst": {
  "target_area": [
    {
      "morphology_id": "one_to_one",
      "morphology_scalar": 1.0,
      "postSynapticCurrent_multiplier": 1.0,
      "plasticity_flag": false
    }
  ]
}
```

### Creating a Morphology
A vector-based morphology example:

```json
"one_to_one": {
  "type": "vectors",
  "class": "custom",
  "parameters": {
    "vectors": [[0, 0, 0]]
  }
}
```

## Implementation Considerations

1. **Performance**: The genome system is designed to efficiently represent complex neural architectures
2. **Extensibility**: New properties can be added to the genome without breaking existing functionality
3. **Validation**: Strict validation ensures genome integrity across operations
4. **Portability**: The JSON-based format allows for easy storage, sharing, and modification

## Future Directions

1. **Enhanced morphologies**: Support for more complex connection patterns
2. **Learning parameters**: Additional properties for controlling neural plasticity
3. **Template systems**: Pre-defined cortical area templates for common structures
4. **Hierarchical organization**: Support for brain region groupings above cortical areas 