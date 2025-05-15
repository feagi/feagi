# 11 - Neuron Potential Data (Categories, XYZ)

**Current Version: 1**

**Size: Varies (depends on neuron count)**

**ID: 11**

This structure is used to define the membrane potential of any given neurons of any given cortical areas. The primary use is when it is important to save on bandwidth (in comparison to the flat variant of the Neuron Potential Data) and more importantly, to be able to rapidly read neuron data from specific cortical areas without having to transverse the entire structure.

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

#### Initial Section Header

<table border="1" id="bkmrk-section-data-descrip-1" style="border-collapse: collapse; width: 100%; height: 76.4px;"><colgroup><col style="width: 50.0407%;"></col><col style="width: 25.0204%;"></col><col style="width: 25.0204%;"></col></colgroup><tbody><tr style="height: 29.8px;"><td style="height: 29.8px;">Section Data Description  
</td><td style="height: 29.8px;">Number of bytes  
</td><td style="height: 29.8px;">Data Representation  
</td></tr><tr style="height: 46.6px;"><td style="height: 46.6px;">Cortical Area Count (number cortical areas this struct contains)

</td><td style="height: 46.6px;">2</td><td style="height: 46.6px;">INT16

</td></tr></tbody></table>

#### Secondary Header (One copy exists per cortical area, in sequence)

<table border="1" id="bkmrk-section-data-descrip-2" style="border-collapse: collapse; width: 100%; height: 106.033px;"><colgroup><col style="width: 50.0407%;"></col><col style="width: 25.0204%;"></col><col style="width: 25.0204%;"></col></colgroup><tbody><tr style="height: 29.8px;"><td style="height: 29.8px;">Section Data Description  
</td><td style="height: 29.8px;">Number of bytes  
</td><td style="height: 29.8px;">Data Representation  
</td></tr><tr style="height: 46.6px;"><td style="height: 46.6px;">Cortical ID as ASCII

</td><td style="height: 46.6px;">6</td><td style="height: 46.6px;">6 * ASCII</td></tr><tr style="height: 29.6333px;"><td style="height: 29.6333px;">Reading Start Index (relative to this whole struct)

</td><td style="height: 29.6333px;">4</td><td style="height: 29.6333px;">UINT32

</td></tr><tr><td>Number of Neurons

</td><td>4</td><td>UINT32

</td></tr></tbody></table>

#### Neuron Data (In sequence of cortical areas, each neuron is split)

<table border="1" id="bkmrk-section-data-descrip-3" style="border-collapse: collapse; width: 100%; height: 149px;"><colgroup><col style="width: 50.0586%;"></col><col style="width: 25.0293%;"></col><col style="width: 25.0293%;"></col></colgroup><tbody><tr style="height: 29.8px;"><td style="height: 29.8px;">Section Data Description  
</td><td style="height: 29.8px;">Number of bytes  
</td><td style="height: 29.8px;">Data Representation  
</td></tr><tr style="height: 29.8px;"><td style="height: 29.8px;">X coordinates, per neuron</td><td style="height: 29.8px;">4 * N</td><td style="height: 29.8px;">UINT32</td></tr><tr style="height: 29.8px;"><td style="height: 29.8px;">Y coordinates, per neuron</td><td style="height: 29.8px;">4 * N</td><td style="height: 29.8px;">UINT32</td></tr><tr style="height: 29.8px;"><td style="height: 29.8px;">Z coordinates, per neuron</td><td style="height: 29.8px;">4 * N</td><td style="height: 29.8px;">UINT32</td></tr><tr style="height: 29.8px;"><td style="height: 29.8px;">Neuron Potential, per neuron</td><td style="height: 29.8px;">4 * N</td><td style="height: 29.8px;">FLOAT</td></tr></tbody></table>

Ergo, size for the structure in number of bytes will be: 2 (global header) + 2 (section header) + (14 \* C) (Cortical Areas) + (16 \* N) (Neurons).
