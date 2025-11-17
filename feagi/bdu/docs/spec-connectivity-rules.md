# Connectivity Rule Module Documentation

Connectivity rules (also known as synaptogenesis rules) define how neurons in FEAGI establish connections between source and destination cortical areas. These rules determine the patterns of synaptic formation and are essential for creating the connectome of the brain.

## Overview

The `synaptogenesis_rules.py` module provides mechanisms for:
1. Finding target neurons for synaptic connections
2. Defining patterns of connectivity between cortical areas
3. Supporting various connectivity paradigms (e.g., one-to-one, many-to-one, one-to-many mappings)
4. Efficient bitmap-based operations for handling multiple neurons per voxel

## Technical Architecture

The synaptogenesis process relies on several key components:

1. **Neuron Morphology**: Defined in the genome as structures that determine connection patterns
2. **Rule Selection**: Based on neuron's morphology type (vectors, patterns, or functions)
3. **Coordinate Transformation**: Converting between source and destination coordinate spaces
4. **Post-Synaptic Current Assignment**: Determining the strength of connections based on morphology properties
5. **ConnectomeManager**: Central interface for accessing neuron and cortical area information
6. **BitMap Optimization**: Efficient set operations for handling large numbers of neurons

The module uses strong type annotations for Rust compatibility:
```python
# Type aliases for improved code readability and Rust compatibility
AreaId = int
NeuronId = int
Position = Tuple[int, int, int]
VoxelIndex = Tuple[int, int, int, int]  # area_id, x, y, z
LinearPosition = int
BoundingBox = Tuple[Tuple[int, int, int], Tuple[int, int, int]]  # ((min_x, min_y, min_z), (max_x, max_y, max_z))
MorphologyParams = Dict[str, Any]
```

The main entry point is the `neighbor_finder` function, which takes:
```python
def neighbor_finder(
    src_area_id: AreaId,
    dst_area_id: AreaId,
    src_neuron_id: NeuronId,
    morphology: MorphologyParams,
    src_subregion: BoundingBox,
    connectome_manager,
    memory_register: Dict[AreaId, Set[AreaId]],
    morphology_id_overwrite: Optional[str] = None
) -> List[Tuple[NeuronId, float]]:
    """Find candidate neurons in the destination area for synaptic connections."""
```

## Connectivity Rule Types

FEAGI supports three main types of connectivity rules, defined as an enum:

```python
class RuleType(Enum):
    """Types of synaptogenesis rules."""
    VECTORS = "vectors"
    PATTERNS = "patterns"
    FUNCTIONS = "functions"
    PLACEHOLDER = "placeholder"
```

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
def match_vectors(
    src_voxel: Position,
    dst_area_id: AreaId,
    vector: List[int],
    morphology_scalar: List[Union[int, str]],
    src_subregion: BoundingBox,
    connectome_manager
) -> List[Position]:
    """Match vectors between source and destination areas."""
    # Create a string representation to check which dimensions need full range
    morphology_scalar_string = ''.join(str(s) for s in morphology_scalar)

    # Determine ranges based on morphology_scalar
    x_range = range(dst_area_dims[0]) if "x" in morphology_scalar_string else [src_voxel[0]]
    y_range = range(dst_area_dims[1]) if "y" in morphology_scalar_string else [src_voxel[1]]

    # Special handling for z-dimension
    z_range = range(dst_area_dims[2]) if "z" in morphology_scalar_string else [src_voxel[2]]

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

```python
def preprocess_expression(expr: str) -> str:
    """Preprocess algebraic expressions for evaluation."""
    # Add * for implicit multiplication (e.g., 2x -> 2*x)
    expr = re.sub(r'(\d)([a-zA-Z])', r'\1*\2', expr)
    # Replace ^ with ** for exponentiation
    expr = expr.replace('^', '**')
    return expr

def evaluate_expression(expr: Union[str, int], x: int, y: int, z: int) -> int:
    """Evaluate an algebraic expression with the given x, y, z values."""
    if isinstance(expr, (int, float)):
        return int(expr)

    try:
        result = sympify(preprocess_expression(expr)).subs({"x": x, "y": y, "z": z})
        return int(result)
    except Exception as e:
        logger.error(f"Error evaluating expression '{expr}': {e}")
        return 0
```

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
   ```python
   def find_source_coordinates(
       src_pattern: List[Any],
       src_cortical_boundary: Position
   ) -> Generator[Position, None, None]:
       """Generate coordinates within the cortical boundary that match the given pattern."""
       # Generate ranges based on pattern and boundary
       x_range = range(src_cortical_boundary[0]) if src_pattern[0] == "*" else [src_pattern[0]]
       y_range = range(src_cortical_boundary[1]) if src_pattern[1] == "*" else [src_pattern[1]]
       z_range = range(src_cortical_boundary[2]) if src_pattern[2] == "*" else [src_pattern[2]]

       # Yield matching coordinates using a generator for memory efficiency
       for x in x_range:
           for y in y_range:
               for z in z_range:
                   yield (x, y, z)
   ```

2. `find_destination_coordinates`: Maps source coordinates to destination coordinates
   ```python
   def find_destination_coordinates(
       dst_cortical_boundary: Position,
       src_coordinate: Position,
       src_pattern: List[Any],
       dst_pattern: List[Any]
   ) -> Generator[Position, None, None]:
       """Generate destination coordinates that match the given patterns."""
       # For each axis in the destination pattern:
       #   "*": All positions on that axis
       #   "?": Only the corresponding source position
       #   "!": All positions except the corresponding source position
       #   int: Only that specific position
       # Yield all valid destination coordinates
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

Function-based rules implement specialized algorithms for neuronal connectivity. These are defined in an enum:

```python
class MorphologyFunction(Enum):
    """Available function-based morphologies."""
    EXPANDER_X = "expander_x"
    REDUCER_X = "reducer_x"
    RANDOMIZER = "randomizer"
    LATERAL_PAIRS_X = "lateral_pairs_x"
    BLOCK_CONNECTION = "block_connection"
    LAST_TO_FIRST = "last_to_first"
    PROJECTOR = "projector"
    PROJECTOR_XY = "projector_xy"
    PROJECTOR_XZ = "projector_xz"
    PROJECTOR_YZ = "projector_yz"
    PROJECT_FROM_END_X = "project_from_end_x"
    PROJECT_FROM_END_Y = "project_from_end_y"
    PROJECT_FROM_END_Z = "project_from_end_z"
    MEMORY = "memory"
```

These include:

#### Expander/Reducer (x-dimension)
- `expander_x`: Maps source neurons to destination neurons where the binary representation has a set bit at the source neuron's x-position index or higher
  ```python
  def syn_expander_x(
      src_area_id: AreaId,
      dst_area_id: AreaId,
      src_neuron_id: NeuronId,
      src_subregion: BoundingBox,
      connectome_manager,
      dst_y_index: int = 0,
      dst_z_index: int = 0
  ) -> List[Position]:
      """Implement the expander rule for x-dimension."""
      # For each destination x-position:
      #   Convert to binary and pad
      #   Check if the bit at the source neuron's x-position is set
      #   Return matching voxel coordinates
  ```

- `reducer_x`: Maps source neurons to destination neurons corresponding to the position of the highest set bit in the binary representation of the source x-position
  ```python
  def syn_reducer_x(
      src_area_id: AreaId,
      dst_area_id: AreaId,
      src_neuron_id: NeuronId,
      src_subregion: BoundingBox,
      connectome_manager,
      dst_y_index: int = 0,
      dst_z_index: int = 0
  ) -> List[Position]:
      """Implement the reducer rule for x-dimension."""
      # Convert source neuron's x position to binary
      # Find the positions of set bits
      # Return corresponding destination voxels for those bit positions
  ```

#### Projector Functions
- `projector`: Maps neurons from source to destination areas while maintaining topological relationships
  ```python
  def syn_projector(
      src_area_id: AreaId,
      dst_area_id: AreaId,
      src_neuron_id: NeuronId,
      src_subregion: BoundingBox,
      connectome_manager,
      transpose: Optional[Tuple[str, str, str]] = None,
      project_last_layer_of: Optional[str] = None
  ) -> List[Position]:
      """Project neurons between areas of different dimensions."""
      # Calculate source and destination shapes
      # For each dimension:
      #   If src_dim > dst_dim: Scale down by ratio
      #   If src_dim < dst_dim: Scale up by ratio
      #   If src_dim == dst_dim: Maintain position
      # Handle transpose if specified
      # Handle last layer projection if specified
      # Return all matching destination voxels
  ```

- Additional projector variants (`PROJECTOR_XY`, `PROJECTOR_XZ`, `PROJECTOR_YZ`) call `syn_projector` with different transpose parameters
- Project-from-end variants (`PROJECT_FROM_END_X`, `PROJECT_FROM_END_Y`, `PROJECT_FROM_END_Z`) apply projections only to neurons in the last layer of the specified axis

#### Special-Purpose Functions
- `randomizer`: Creates random connections to neurons in the destination area
  ```python
  def syn_randomizer(dst_area_id: AreaId, connectome_manager) -> Position:
      """Select a random position in the destination area."""
      # Get destination area dimensions
      # Return a random position within those dimensions
  ```

- `lateral_pairs_x`: Creates lateral connections between neighboring neurons on the x-axis (0→1, 2→3, etc.)
  ```python
  def syn_lateral_pairs_x(
      neuron_id: NeuronId,
      area_id: AreaId,
      src_subregion: BoundingBox,
      connectome_manager
  ) -> Optional[Position]:
      """Create lateral connections between neighboring neurons on the x-axis."""
      # If even x position: connect to x+1
      # If odd x position: connect to x-1
      # Return the target position if in range
  ```

- `block_connection`: Maps blocks of neurons from source to destination areas based on a scaling factor
  ```python
  def syn_block_connection(
      src_area_id: AreaId,
      dst_area_id: AreaId,
      src_neuron_id: NeuronId,
      src_subregion: BoundingBox,
      connectome_manager,
      scaling_factor: int = 10
  ) -> Position:
      """Map blocks of neurons between areas based on scaling factor."""
      # Divide neuron's position by scaling factor
      # Return corresponding destination voxel
  ```

- `memory`: Registers source-destination area relationships in a memory registry
  ```python
  def syn_memory(
      src_area_id: AreaId,
      dst_area_id: AreaId,
      memory_register: Dict[AreaId, Set[AreaId]]
  ) -> None:
      """Register source-destination area relationships in memory registry."""
      # Create dst_area entry if it doesn't exist
      # Add src_area to the set of areas connected to dst_area
  ```

- `last_to_first`: Connects the last neuron of an area to the first neuron
  ```python
  def last_to_first(src_area_id: AreaId, connectome_manager) -> List[Position]:
      """Return the position of the first voxel [0, 0, 0]."""
      # Always returns [(0, 0, 0)]
  ```

## Position Handling and Optimization

### Position Linearization

For efficient bitmap operations, the module provides functions to convert between 3D positions and linearized indices:

```python
def linearize_position(position: Position, dimensions: Position) -> LinearPosition:
    """Convert a 3D position to a linearized 1D index."""
    x, y, z = position
    width, height, depth = dimensions
    return x + (y * width) + (z * width * height)

def delinearize_position(linear_pos: LinearPosition, dimensions: Position) -> Position:
    """Convert a linearized 1D index back to a 3D position."""
    width, height, depth = dimensions
    z = linear_pos // (width * height)
    remainder = linear_pos % (width * height)
    y = remainder // width
    x = remainder % width
    return (x, y, z)
```

### BitMap Optimization

The module uses a BitMap implementation for efficient set operations:

```python
try:
    from feagi.npu.fcl_manager import BitMap
except ImportError:
    # Fallback implementation if the main one is not available
    class BitMap(set):
        """Simple set-based fallback for environments without proper BitMap."""
        def __init__(self, elements=None):
            super().__init__(elements or [])

        def add(self, element: int) -> None:
            super().add(element)

        def copy(self) -> 'BitMap':
            return BitMap(self)

        def is_empty(self) -> bool:
            return len(self) == 0
```

## Implementation Details

### Subregion Definition

The `define_subregions` function creates patterns of connectivity based on "seed" and "pattern" parameters, allowing for complex geometric arrangements of connections.

```python
def define_subregions(
    area_id: AreaId,
    parameters: Dict[str, Any],
    cortical_dimensions: Position
) -> Set[BoundingBox]:
    """Define subregions within a cortical area for targeted synaptogenesis."""
    # Extract seed and pattern parameters
    # seed: A 3D vector defining a unit cube [seed_x, seed_y, seed_z]
    # pattern: Format [[choose_x, skip_x], [choose_y, skip_y], [choose_z, skip_z]]

    # Generate all possible subregions based on the pattern
    # Each subregion is defined as a tuple of ((min_x, min_y, min_z), (max_x, max_y, max_z))
    # Return the set of all subregions
```

Example input pattern: `[[2, 1], [3, 0], [2, 1]]` and seed: `[10, 5, 8]` would:
1. Choose 2 segments in x-direction, skip 1
2. Choose 3 segments in y-direction, skip 0
3. Choose 2 segments in z-direction, skip 1

### Pattern Validation

Functions like `check_pattern_validity` ensure that connectivity patterns adhere to the expected format and valid character sets.

```python
def check_pattern_validity(pattern: List[Any]) -> bool:
    """Check if a pattern contains valid elements."""
    valid_patterns = {"*", "?", "!"}
    for element in pattern:
        if element not in valid_patterns:
            try:
                value = int(element)
                if value < 0:
                    return False
            except (ValueError, TypeError):
                return False
    return True
```

## Usage in Synaptogenesis

During synaptogenesis, the `neighbor_finder` function uses the connectivity rules to determine the appropriate target neurons for a source neuron. It:

1. Identifies the appropriate rule type for the neuron's morphology
2. Applies the rule to find candidate target voxels
3. Converts the voxel coordinates to neuron IDs
4. Sets the appropriate post-synaptic current value for each connection

```python
def neighbor_finder(
    src_area_id: AreaId,
    dst_area_id: AreaId,
    src_neuron_id: NeuronId,
    morphology: MorphologyParams,
    src_subregion: BoundingBox,
    connectome_manager,
    memory_register: Dict[AreaId, Set[AreaId]],
    morphology_id_overwrite: Optional[str] = None
) -> List[Tuple[NeuronId, float]]:
    """Find candidate neurons in the destination area for synaptic connections."""
    # Get source neuron position
    # Determine morphology type and parameters
    # Calculate post-synaptic current from base value and multiplier
    # Apply appropriate rule based on morphology type:
    #   - For vectors: Match vector patterns
    #   - For patterns: Find matching coordinates
    #   - For functions: Call appropriate function
    # Convert voxel coordinates to neuron IDs, handling multiple neurons per voxel
    # Return list of (neuron_id, post_synaptic_current) pairs
```

The function returns a list of tuples, each containing:
- A target neuron ID
- The post-synaptic current (PSC) value for the connection

## Error Handling and Logging

The module includes extensive error handling and logging to assist with debugging:

```python
try:
    # Rule application logic
except Exception as e:
    logger.error(f"Error during synaptogenesis of {src_area_id} and {dst_area_id}: {e}")
    logger.error(traceback.format_exc())
```

## Multiple Neurons Per Voxel

The implementation supports multiple neurons per voxel through the ConnectomeManager interface:

```python
# Convert positions to neurons
for position in raw_candidate_positions:
    # Find all neurons at this position
    dst_neurons = connectome_manager.get_neurons_at_position(
        area_id=dst_area_id,
        position=position
    )

    # Add each neuron with the appropriate weight (PSC)
    for neuron_id in dst_neurons:
        candidate_neuron_list.append((neuron_id, post_synaptic_current))
```

## Interconnection with Other Modules

The synaptogenesis module interacts with:

1. **ConnectomeManager**: Accesses cortical area and neuron information instead of directly using runtime_data
2. **NeuronBitMap**: Uses efficient bitmap operations for handling large sets of neurons (with fallback implementation)
3. **Sympy Library**: For evaluating algebraic expressions in vector rules
4. **Memory Registry**: For morphologies that need to track relationships between areas
