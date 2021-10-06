# Genome Development

The genome is a data structure comprised of key-value pairs that provides user-configurable anatomical and physiological parameters to FEAGI for artificial brain development. The neuroembryogenesis unit (`src/evo/neuroembryogenesis.py`) processes genome data and generates virtual anatomical features within the artificial brain, such as cortical areas, cortical layers, neurons and synapses. Users may generate an artificial brain capable of processing various input data types (via the Input Processing Unit - `src/ipu/source/`) solely through modification of the seed genome.

## **Genome structure**    

![genome1](../../docs/_static/genome1.png)   
**Fig. 1: A section of cortical area data in the genome data structure showing various parameters.**    

The existing genome contains cortical areas for processing image and LIDAR input data, which can be found under the `"blueprint"` key in `src/evo/static_genome.py` and are useful examples for developing new cortical areas. Other keys at the `"blueprint"` level (e.g. `"firing_patterns"`, `"neighbor_locator_rule"`) are used for further defining neurophysiologic and synaptogenic properties within a cortical area. The figure above shows a layer of the vision cortical area. Note the hierarchical nature of the data; properties such as `"growth_path"` are nested under the `"vision_v1-1"` key. Each cortical area belongs to a specific group (`"group_id"`) and subgroup (`"sub_group_id"`), possesses a user-defined cortical neuron count (among other parameters) and is mapped to another region in the artificial brain (`"cortical_mapping_dst"`).

![genome2](../../docs/_static/genome2.png)    
**Fig. 2: More data from the same cortical area showing geometric and neurophysiologic parameters.**

Cortical areas in the genome have virtual dimensions for accommodating the proliferation of neurons and formation of synapses within the defined area. In **Fig. 2**, note the `"geometric_boundaries"` listed for the _x_, _y_ and _z_ dimensions. Each neuron cell body created via neurogenesis is associated with a point _(x, y, z)_ existing within these boundaries. Users must define boundaries according to the nature of the input data to ensure its appropriate translation to neuronal activity. Users can subdivide cortical areas by setting block boundaries (see the `"block_boundaries"` key in **Fig. 2**), which define the dimensions of a block. Blocks are cortical area subregions that facilitate the localization of neurons for activation following translation of brain input data. To better illustrate the concept, if a user defined a `100x100x100` (_x_, _y_, _z_) cortical area with `10x10x10` block boundaries, the cortical area will be divided into `10` blocks, each containing unique neurons. FEAGI will then create references to these blocks, allowing for faster and more refined stimulation of neurons in the cortical area.

## **Editing the genome file**

- Open the genome file (`src/evo/static_genome.py`) using a text editor or IDE.

- Navigate to the end of the data present under the `"blueprint"` key, which contains all of the existing cortical area definitions, and enter the data defining the new cortical area (it will likely be easier to copy a single existing cortical area definition in the genome, paste it at the end of the file and update the values accordingly).
  - Give the cortical area a unique name.
  - Enter a `"group_id"` (typically `"IPU"` if defining an area for processing input data and `"Memory"` when defining a memory area) and `sub_group_id`.
  - Specify a number (integer) of neurons that will populate the cortical area (`"cortical_neuron_count"`). Note that sparsely populated cortical areas will likely result in diminished neuronal activity.
  - If the cortical area being created is intended for processing input data, ensure that it is mapped (`"cortical_mapping_dst"`) to a destination area (typically a corresponding memory area - see the `"proximity"` and `"proximity_memory"` areas for an example of this mapping).
  - Specify `"geometric_boundaries"` and `"block_boundaries"` for the cortical area based on the type/structure of input data.
  - The other keys in the cortical area (not mentioned above) can be left as-is with existing values (assuming the user copied an existing cortical area to use as a template).

- Save the genome file.

## **Troubleshooting issues**

After adding the desired data to the genome file, users should confirm that FEAGI is able to create the new cortical area(s). Navigate to the `src/` directory in `feagi-core/` and run `python3 main.py` to begin FEAGI execution. During FEAGI initialization, a list of cortical areas loaded from the genome are displayed in the terminal output (see **Fig. 3**). Ensure that the newly-added cortical areas are present in this list.  

![feagi_output](../../docs/_static/feagi_output.png)    
**Fig. 3: Output of FEAGI execution showing some of the cortical areas created from the genome file.**    

If newly-created cortical areas are not present in the output of FEAGI initialization or users encounter other runtime errors following genome modification, consider the following:    

- **Genome file**
  - Were changes made to the correct genome file? Users should add new cortical areas to `src/evo/static_genome.py`.

- **Genome file syntax**
  - Did the changes made to the genome file introduce syntax errors? Ensure that all new cortical areas are added under the `"blueprint"` key and indented to the appropriate levels. All opening brackets ( `{`, `(`, `[` ) should have a corresponding closing bracket ( `}`, `)`, `]` ), commas ( `,` ) must separate each key-value pair and no values should be empty/blank.
