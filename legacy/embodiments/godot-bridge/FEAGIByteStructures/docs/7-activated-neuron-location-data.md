# 7 - Activated Neuron Location Data

**Current Version: 1**

**Size: Varies**

This structure encapsulates the 3D location of activated neurons from FEAGI. *This format is intended for transmission only due to changing structure sizes per* burst. **This structure is intended for temporary use as we transition to more efficient methods to send activation state data.**

#### General Structure

<table border="1" id="bkmrk-section-data-descrip" style="border-collapse: collapse; width: 100%; height: 118.067px;"><colgroup><col style="width: 50.0407%;"></col><col style="width: 25.0204%;"></col><col style="width: 25.0204%;"></col></colgroup><tbody><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Section Data Description  
</td><td style="height: 29.5167px;">Number of bytes  
</td><td style="height: 29.5167px;">Data Representation  
</td></tr><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Global Header ID  
</td><td style="height: 29.5167px;">1  
</td><td style="height: 29.5167px;">INT8  
</td></tr><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Global Header Version  
</td><td style="height: 29.5167px;">1  
</td><td style="height: 29.5167px;">INT8  
</td></tr></tbody></table>

(Per Activated Neuron, repeated in sequence as needed for each neuron)

<table border="1" id="bkmrk-x-coordinate-2-sint1" style="border-collapse: collapse; width: 100%; height: 88.5501px;"><colgroup><col style="width: 50.1609%;"></col><col style="width: 24.8948%;"></col><col style="width: 25.0278%;"></col></colgroup><tbody><tr style="height: 29.5167px;"><td style="height: 29.5167px;">X coordinate  
</td><td style="height: 29.5167px;">2  
</td><td style="height: 29.5167px;">SINT16  
</td></tr><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Y coordinate  
</td><td style="height: 29.5167px;">2  
</td><td style="height: 29.5167px;">SINT16</td></tr><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Z coordinate  
</td><td style="height: 29.5167px;">2  
</td><td style="height: 29.5167px;">SINT16</td></tr></tbody></table>

Essentially, structure works as the following

- For EACH activated neuron (in sequence)  
    
    - We store the X Y Z coordinate sequentially

#### Example

TODO