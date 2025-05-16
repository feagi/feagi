# 10 - Neuron Potential Data (Flat)

**Current Version: 1**

**Size: Varies (depends on neuron count)**

**ID: 10**

This structure is used to define the membrane potential of any given neurons of any given cortical areas. The primary use is between the bridge and FEAGI itself, for FEAGI to output neuron states of every burst and for bridge to pass in inputs, or in the case of embedded machines to have controllers talk directly with FEAGI. It may be used in other cases as a generic data-structure for FEAGI interaction in which no specific structure already exists.

*It is important to note that this structure is focused on processing speed rather than size compression or cortical area catagorization, so be wary about its use.*

(Example below assumes N neurons are transmitted)

#### General Structure

<table border="1" id="bkmrk-section-data-descrip" style="border-collapse: collapse; width: 100%; height: 89.4px;"><colgroup><col style="width: 50.0407%;"></col><col style="width: 25.0204%;"></col><col style="width: 25.0204%;"></col></colgroup><tbody><tr style="height: 29.8px;"><td style="height: 29.8px;">Section Data Description  
</td><td style="height: 29.8px;">Number of bytes  
</td><td style="height: 29.8px;">Data Representation  
</td></tr><tr style="height: 29.8px;"><td style="height: 29.8px;">Global Header ID  
</td><td style="height: 29.8px;">1  
</td><td style="height: 29.8px;">INT8  
</td></tr><tr style="height: 29.8px;"><td style="height: 29.8px;">Global Header Version  
</td><td style="height: 29.8px;">1  
</td><td style="height: 29.8px;">INT8</td></tr></tbody></table>

#### Neuron Data (each section in sequence)

<table border="1" id="bkmrk-section-data-descrip-2" style="border-collapse: collapse; width: 100%; height: 178.8px;"><colgroup><col style="width: 50.0586%;"></col><col style="width: 25.0293%;"></col><col style="width: 25.0293%;"></col></colgroup><tbody><tr style="height: 29.8px;"><td style="height: 29.8px;">Section Data Description  
</td><td style="height: 29.8px;">Number of bytes  
</td><td style="height: 29.8px;">Data Representation  
</td></tr><tr style="height: 29.8px;"><td style="height: 29.8px;">Cortical IDs as ASCII, per neuron</td><td style="height: 29.8px;">6 * N</td><td style="height: 29.8px;">ASCII</td></tr><tr style="height: 29.8px;"><td style="height: 29.8px;">X coordinates, per neuron</td><td style="height: 29.8px;">4 * N</td><td style="height: 29.8px;">INT32</td></tr><tr style="height: 29.8px;"><td style="height: 29.8px;">Y coordinates, per neuron</td><td style="height: 29.8px;">4 * N</td><td style="height: 29.8px;">INT32</td></tr><tr style="height: 29.8px;"><td style="height: 29.8px;">Z coordinates, per neuron</td><td style="height: 29.8px;">4 * N</td><td style="height: 29.8px;">INT32</td></tr><tr style="height: 29.8px;"><td style="height: 29.8px;">Neuron Potential, per neuron</td><td style="height: 29.8px;">4 * N</td><td style="height: 29.8px;">FLOAT</td></tr></tbody></table>

Ergo, size for the structure in number of bytes will be: 2 (global header) + (22 * N).
