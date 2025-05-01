# Connectivity Rule Module Documentation

Connectivity rules (also known as synaptogenesis rules) define how neurons in FEAGI establish connections between source and destination cortical areas. These rules determine the patterns of synaptic formation and are essential for creating the connectome of the brain.

## Overview

The `synaptogenesis_rules.py` module provides mechanisms for:
1. Finding target neurons for synaptic connections
2. Defining patterns of connectivity between cortical areas
3. Supporting various connectivity paradigms (e.g., one-to-one, many-to-one, one-to-many mappings)

## Technical Architecture

The synaptogenesis process relies on several key components:

1. **Neuron Morphology**: Defined in the genome as structures that determine connection patterns
2. **Rule Selection**: Based on neuron's morphology type (vectors, patterns, or functions)
3. **Coordinate Transformation**: Converting between source and destination coordinate spaces
4. **Post-Synaptic Current Assignment**: Determining the strength of connections based on morphology properties

The main entry point is the `neighbor_finder` function, which takes:
- `cortical_area_src`: Source cortical area ID
- `cortical_area_dst`: Destination cortical area ID
- `src_neuron_id`: ID of the source neuron
- `morphology_`: Dictionary containing morphology information
- `src_subregion`: Tuple of ((min_x, min_y, min_z), (max_x, max_y, max_z))
- `morphology_id_overwrite`: Optional override for morphology ID

## Connectivity Rule Types

FEAGI supports three main types of connectivity rules:

### 1. Vectors

Vector rules are defined by one or many vectors in the form of [x, y, z] which defines a 3D vector in the cartesian 
coordinate that translates the current neuron position to its destination. 

e.g., if a neuron is located at position [2, 4, 1] within a cortical area, the connectivity rule as [0, 1, 0] would 
target the voxel with address [2 + 0, 4 + 1, 1 + 0] as the destination for synaptic mapping.

#### Technical Implementation

The `match_vectors` function handles vector-based connectivity with these steps:
1. Processes the source voxel coordinates from the neuron's position
2. Applies morphology_scalar to determine the range of destination coordinates to consider
3. For each possible destination coordinate:
   - Evaluates algebraic expressions using sympy (with x, y, z variables)
   - Scales the translation vector based on the morphology vector
   - Computes candidate vectors by adding to the source position
   - Filters candidates that are within the destination cortical area dimensions

```python
def match_vectors(src_voxel, cortical_area_dst, vector, morphology_scalar, src_subregion):
    # Preprocessing of expressions
    # Determine coordinate ranges based on morphology_scalar
    # For each coordinate in ranges:
    #   - Evaluate expressions with sympy
    #   - Scale vector by evaluated expressions
    #   - Add to source position
    #   - Check validity of resulting position
    # Return valid candidate positions
```

Multiple vector mapping can come together as one connectivity rule which will be captured within genome as shown below:

```json
"ori_135_deg": {
  "parameters": {
    "vectors": [
      [0, 0, 0],
      [1, -1, 0],
      [2, -2, 0],
      [3, -3, 0],
      [4, -4, 0],
      [5, -5, 0]
    ]
  },
  "type": "vectors",
  "class": "custom"
}
```

Vector rules can also use algebraic expressions with `x`, `y`, and `z` as variables, allowing for complex connectivity patterns that are dependent on the position of the neuron. These expressions are evaluated using the `sympy` library.

#### Expression Processing
1. The preprocessing phase handles implicit multiplication (e.g., `2x → 2*x`) using regex
2. Expressions are parsed and evaluated by `sympy.sympify` with variable substitution
3. Results are converted to integers and validity is checked before application

The rules can be scaled using a `morphology_scalar` parameter to modify the range or scope of connections.

### 2. Patterns

Pattern rules define explicit source-to-destination mappings using a special notation system. Patterns use special characters to define flexible matching rules:

- `*`: Match any position along that axis
- `?`: Match the corresponding source coordinate
- `!`: Match any coordinate except the corresponding source coordinate
- `int`: Match a specific position on that axis

#### Technical Implementation

Pattern handling is implemented in two main functions:
1. `find_source_coordinates`: Generates all source coordinates matching a pattern
2. `find_destination_coordinates`: Maps source coordinates to destination coordinates

The destination coordinate generation follows these rules:
```python
# For each axis:
x_range = (
    range(dst_cortical_boundary[0]) if dst_pattern[0] == "*"
    else [src_coordinate[0]] if (dst_pattern[0] == "?" and valid_source_condition)
    else [i for i in range(dst_cortical_boundary[0]) if i != src_coordinate[0]] if dst_pattern[0] == "!"
    else [dst_pattern[0]] if (is_integer and valid_pattern_condition) else []
)
# Similar logic for y_range and z_range
```

Patterns are defined as pairs of source and destination patterns:

```json
"pattern_example": {
  "parameters": {
    "patterns": [
      [["*", "*", "*"], ["?", "?", "*"]]
    ]
  },
  "type": "patterns",
  "class": "custom"
}
```

This allows for complex mapping strategies, such as:
- Creating one-to-one mappings between areas with different dimensions
- Mapping specific regions to others while preserving relative positions
- Creating branching or converging connectivity patterns

### 3. Functions

Function-based rules implement specialized algorithms for neuronal connectivity. These include:

#### Expander/Reducer (x-dimension)
- `expander_x`: Maps combinations of source neurons to unique configurations in the destination area
  ```python
  def syn_expander_x(src_cortical_area, dst_cortical_area, src_neuron_id, src_subregion, dst_y_index=0, dst_z_index=0):
      # Determine dimensions of source and destination areas
      # Convert destination x index to binary
      # Check specific bit position corresponding to source neuron's x position
      # Return matching voxel coordinates if bit is set
  ```
- `reducer_x`: Reverses the expander, mapping combinations back to individual components
  ```python
  def syn_reducer_x(src_cortical_area, dst_cortical_area, src_neuron_id, src_subregion, dst_y_index=0, dst_z_index=0):
      # Convert source neuron's x position to binary representation
      # For each bit position, check if bit is set
      # Return corresponding destination voxels for set bits
  ```

#### Projector Functions
- `projector`: Maps neurons from source to destination areas while maintaining topological relationships
  ```python
  def syn_projector(src_cortical_area, dst_cortical_area, src_neuron_id, src_subregion, transpose=None, project_last_layer_of=None):
      # Calculate source and destination shapes
      # For each dimension:
      #   If src_dim > dst_dim: Scale down by ratio
      #   If src_dim < dst_dim: Scale up by ratio
      #   If src_dim == dst_dim: Maintain position
      # Return all matching destination voxels
  ```
- `projector_xy`, `projector_xz`, `projector_yz`: Call `syn_projector` with appropriate transpose parameter
- `project_from_end_x`, `project_from_end_y`, `project_from_end_z`: Projects only from the last layer of the specified axis, using special logic in the projector

#### Special-Purpose Functions
- `randomizer`: Creates random connections to neurons in the destination area using Python's `randrange`
- `lateral_pairs_x`: Creates lateral connections between neighboring neurons on the x-axis (0→1, 2→3, etc.)
  ```python
  def syn_lateral_pairs_x(neuron_id, cortical_area, src_subregion):
      # Check if neuron's x position is even or odd
      # If even: connect to the neuron at x+1 (if in range)
      # If odd: connect to the neuron at x-1 (if in range)
  ```
- `block_connection`: Maps blocks of neurons from source to destination areas based on a scaling factor
  ```python
  def syn_block_connection(src_cortical_area, dst_cortical_area, src_neuron_id, src_subregion, s=10):
      # Divide neuron's x position by scaling factor s
      # Return corresponding destination voxel
  ```
- `memory`: Registers source-destination area relationships in a memory registry
  ```python
  def syn_memory(src_cortical_area, dst_cortical_area):
      # Register relationship in runtime_data.memory_register
  ```
- `last_to_first`: Connects the last neuron of an area to the first neuron

## Implementation Details

### Subregion Definition

The `define_subregions` function creates patterns of connectivity based on "seed" and "pattern" parameters, allowing for complex geometric arrangements of connections.

```python
def define_subregions(cortical_area, parameters):
    # Extract seed and pattern parameters
    # seed: A 3D vector defining a unit cube [seed_x, seed_y, seed_z]
    # pattern: Format [[choose_x, skip_x], [choose_y, skip_y], [choose_z, skip_z]]
    
    # Generate a set of subregions based on pattern
    # Each subregion is defined as a tuple of ((min_x, min_y, min_z), (max_x, max_y, max_z))
    # Return the set of all subregions
```

Example input pattern: `[[2, 1], [3, 0], [2, 1]]` and seed: `[10, 5, 8]` would:
1. Choose 2 segments in x-direction, skip 1
2. Choose 3 segments in y-direction, skip 0
3. Choose 2 segments in z-direction, skip 1

### Position Matching

The module provides specialized functions for matching source and destination coordinates:
- `find_source_coordinates`: Generates coordinates within cortical boundaries that match source patterns
  ```python
  def find_source_coordinates(src_pattern, src_cortical_boundary):
      # For each axis:
      #   If pattern is "*": Consider all positions
      #   Else: Consider only the specified position
      # Yield all combinations of valid positions
  ```

- `find_destination_coordinates`: Maps source coordinates to appropriate destination coordinates based on pattern rules
  ```python
  def find_destination_coordinates(dst_cortical_boundary, src_coordinate, src_pattern, dst_pattern):
      # For each axis in the destination pattern:
      #   "*": All positions on that axis
      #   "?": Only the corresponding source position
      #   "!": All positions except the corresponding source position
      #   int: Only that specific position
      # Yield all valid destination coordinates
  ```

### Pattern Validation

Functions like `check_pattern_validity` ensure that connectivity patterns adhere to the expected format and valid character sets.

```python
def check_pattern_validity(pattern):
    # Check each element in the pattern:
    #   If special character ("*", "?", "!"): Valid
    #   If integer >= 0: Valid
    #   Else: Invalid
    # Return True if all elements are valid
```

## Usage in Synaptogenesis

During synaptogenesis, the `neighbor_finder` function uses the connectivity rules to determine the appropriate target neurons for a source neuron. It:

1. Identifies the appropriate rule type for the neuron's morphology
2. Applies the rule to find candidate target voxels
3. Converts the voxel coordinates to neuron IDs
4. Sets the appropriate post-synaptic current value for each connection

```python
def neighbor_finder(cortical_area_src, cortical_area_dst, src_neuron_id, morphology_, src_subregion, 
                    morphology_id_overwrite=None):
    # Initialize candidate lists
    # Determine morphology type and parameters
    # Calculate post-synaptic current from base value and multiplier
    # Apply appropriate rule based on morphology type:
    #   - For vectors: Match vector patterns
    #   - For patterns: Find matching coordinates
    #   - For functions: Call appropriate function
    # Convert voxel coordinates to neuron IDs
    # Return list of (neuron_id, post_synaptic_current) pairs
```

The function returns a list of tuples, each containing:
- A target neuron ID
- The post-synaptic current (PSC) value for the connection

## Advanced Features

### Algebraic Expressions

The vector-based rules support algebraic expressions, which are preprocessed and evaluated using the `sympy` library. This allows for sophisticated connectivity patterns defined mathematically.

Example expression processing:
```python
def preprocess_expression(expr):
    # Add * for implicit multiplication (e.g., 2x -> 2*x)
    expr = re.sub(r'(\d)([a-zA-Z])', r'\1*\2', expr)
    # Replace ^ with ** for exponentiation
    expr = expr.replace('^', '**')
    return expr

# Evaluation
evaluated_value = int(sympify(preprocess_expression(expr)).subs({"x": x_val, "y": y_val, "z": z_val}))
```

### Morphology Scaling

Morphologies can be scaled using parameters that control the extent and pattern of connections. This scaling can be applied globally or per-axis, providing fine-grained control over neuronal growth.

The `morphology_scalar` parameter can include:
- Fixed values: Direct scaling factors
- Variables (x, y, z): Position-dependent scaling
- Expressions: Complex position-dependent scaling

### Subregion Definition

Complex patterns of connectivity can be defined by creating subregions within cortical areas and applying different rules to each subregion. 

## Interconnection with Other Modules

The synaptogenesis module interacts with:

1. **Genome**: Retrieves cortical area definitions, morphology parameters, and base PSC values
2. **Voxel Management**: Uses `voxels.voxel_list_to_neuron_list()` to convert voxel coordinates to neuron IDs
3. **Runtime Data**: Accesses neuron positions and block boundaries from `runtime_data.brain` and `runtime_data.genome`
4. **Memory Management**: For certain morphologies, updates `runtime_data.memory_register` 