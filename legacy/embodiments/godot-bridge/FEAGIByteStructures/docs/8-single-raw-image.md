# 8 - Single Raw Image

**Current Version: 1**

**Size: Varies but only changes when image resolution changes**

This structure encodes a 2D image in RAW BGR (due to Godot) Int8 format. It is intended to be used to transfer images in a fully uncompressed format while also encoding its size information. This implementation is generic intentionally. *Size generally doesn't change size unless resolution of the image changes.*

#### General Structure

<table border="1" id="bkmrk-section-data-descrip" style="border-collapse: collapse; width: 100%; height: 118.067px;"><colgroup><col style="width: 50.0407%;"></col><col style="width: 25.0204%;"></col><col style="width: 25.0204%;"></col></colgroup><tbody><tr style="height: 29.5167px;"><td style="height: 29.5167px;">**Section Data Description**  
</td><td style="height: 29.5167px;">**Number of bytes**  
</td><td style="height: 29.5167px;">**Data Representation**  
</td></tr><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Global Header ID  
</td><td style="height: 29.5167px;">1  
</td><td style="height: 29.5167px;">INT8  
</td></tr><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Global Header Version  
</td><td style="height: 29.5167px;">1  
</td><td style="height: 29.5167px;">INT8  
</td></tr></tbody></table>

**Sub-Header**

<table border="1" id="bkmrk-x-coordinate-2-sint1" style="border-collapse: collapse; width: 100%; height: 79.5501px;"><colgroup><col style="width: 50.1609%;"></col><col style="width: 24.8948%;"></col><col style="width: 25.0278%;"></col></colgroup><tbody><tr style="height: 29.5167px;"><td style="height: 29.5167px;">X resolution  
</td><td style="height: 29.5167px;">2  
</td><td style="height: 29.5167px;">SINT16  
</td></tr><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Y resolution  
</td><td style="height: 29.5167px;">2  
</td><td style="height: 29.5167px;">SINT16</td></tr></tbody></table>

**Data (repeated for every pixel, of which there is Xres\*Yres of)**

<table border="1" id="bkmrk-r-pixel-1-uint8-g-pi" style="border-collapse: collapse; width: 100%; height: 88.5501px;"><colgroup><col style="width: 50.0417%;"></col><col style="width: 25.0139%;"></col><col style="width: 25.0278%;"></col></colgroup><tbody><tr style="height: 29.5167px;"><td style="height: 29.5167px;">B pixel  
</td><td style="height: 29.5167px;">1  
</td><td style="height: 29.5167px;">UINT8  
</td></tr><tr style="height: 29.5167px;"><td style="height: 29.5167px;">G pixel  
</td><td style="height: 29.5167px;">1  
</td><td style="height: 29.5167px;">UINT8  
</td></tr><tr style="height: 29.5167px;"><td style="height: 29.5167px;">R pixel  
</td><td style="height: 29.5167px;">1  
</td><td style="height: 29.5167px;">UINT8</td></tr></tbody></table>

Essentially, structure works as the following

\- Sub-header defines resolution of the image, and the following data is the raw byte data for the image

#### Example

TODO