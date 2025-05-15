# 9 - Multi bytestruct holder

**Current Version: 1**

**Size: Varies**

**ID: 9**

This structure can hold multiple other byte-structures. This is useful if you wish to send multiple types of data in a single package. *Size can vary as per the contained structures.*

#### General Structure

<table border="1" id="bkmrk-section-data-descrip" style="border-collapse: collapse; width: 100%; height: 118.067px;"><colgroup><col style="width: 50.0407%;"></col><col style="width: 25.0204%;"></col><col style="width: 25.0204%;"></col></colgroup><tbody><tr style="height: 29.5167px;"><td style="height: 29.5167px;">**Section Data Description**  
</td><td style="height: 29.5167px;">**Number of bytes**  
</td><td style="height: 29.5167px;">**Data Representation**  
</td></tr><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Global Header ID  
</td><td style="height: 29.5167px;">1  
</td><td style="height: 29.5167px;">UINT8  
</td></tr><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Global Header Version  
</td><td style="height: 29.5167px;">1  
</td><td style="height: 29.5167px;">UINT8  
</td></tr></tbody></table>

**Sub-Header 1(Only once)**

<table border="1" id="bkmrk-x-coordinate-2-sint1" style="border-collapse: collapse; width: 100%; height: 59.0334px;"><colgroup><col style="width: 50.1609%;"></col><col style="width: 24.8948%;"></col><col style="width: 25.0278%;"></col></colgroup><tbody><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Number of contained bytestructs  
</td><td style="height: 29.5167px;">1</td><td style="height: 29.5167px;">UINT8  
</td></tr></tbody></table>

**Sub-Header 2 (repeated for each contained byte structure, ordered right after each other)**

<table border="1" id="bkmrk-r-pixel-1-uint8-g-pi" style="border-collapse: collapse; width: 100%; height: 88.5501px;"><colgroup><col style="width: 50.0417%;"></col><col style="width: 25.0139%;"></col><col style="width: 25.0278%;"></col></colgroup><tbody><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Index of starting byte inclusive (relative to this whole structure)  
</td><td style="height: 29.5167px;">4  
</td><td style="height: 29.5167px;">UINT32  
</td></tr><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Number of bytes of the substructure  
</td><td style="height: 29.5167px;">4  
</td><td style="height: 29.5167px;">UINT32  
</td></tr></tbody></table>

**Data (repeated for each contained byte structure, ordered right after each other)**

<table border="1" id="bkmrk-contained-byte-struc" style="border-collapse: collapse; width: 100%; height: 88.5501px;"><colgroup><col style="width: 50.0417%;"></col><col style="width: 25.0139%;"></col><col style="width: 25.0278%;"></col></colgroup><tbody><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Contained byte structure</td><td style="height: 29.5167px;">?</td><td style="height: 29.5167px;">Byte-Structure</td></tr></tbody></table>

Essentially, structure works as the following

- First part of the sub header defines the number of internal structures. Limited up to 255 (if you have more than this, what are you doing???)
- The second part of the sub-header has sequential (number of sections determined in the first part) ranges defining where each section starts and how far to read. From here, you can copy / read directly the data 
    - Beware that some bytestructs themselves use array indexes, and those indexes are going to be relative to the substructure itself, so if you read directly from this multi-byte struct, be wary and apply appropriate offsets.
- The third part of the structure is actually an array of contained byte structures, one after the other.
- Yes, you can technically nest multiple multi-byte-structure holders inside each other. Note sure why you would
