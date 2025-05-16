# General Information

Communication between components of the FEAGI ecosystem are done with arrays of Bytes. While historically we heavily relied on JSON, for variety of reasons of performance we will be encoding various data structures into more compute friendly layouts. However, as we have different needs between different components, we need a way to efficiently differentiate between them.

Note: This specific documentation is specifically about the byte structure itself. To see implementation of this data in a given project, check the relevant library / module itself!

## Universal FEAGI-Byte-Structure Header

<table border="1" id="bkmrk-1-byte---uint---data" style="border-collapse: collapse; width: 100%;"><colgroup><col style="width: 23.9566%;"></col><col style="width: 26.9346%;"></col><col style="width: 49.1068%;"></col></colgroup><tbody><tr><td>1 Byte - UINT8 - Data Type  
</td><td>1 Byte - UINT8 - Version counter  
</td><td> ? Bytes - Actual Data (depends on specific structure)
</td></tr></tbody></table>

Essentially, all structures will have an initial 2 byte header composed of 2 8-bit unsigned integers

- **Data Type**: This describes what the actual data type will be. Following pages will go into more detail of each type.
- **Version** **Counter**: Over time we will have revisions to our standards. If we make changes that are not backward compatible, we increment this number such that applications can differentiate between versions and act appropriately

**Note** this data may be sent over ZMG, WebSocket, or other means

## Note on Compression

We use the [Deflate](https://en.wikipedia.org/wiki/Deflate) compression before sending / reading this data over the network to cut down bandwidth costs, and because this is a fast and built in method for many languages and interfaces. Keep this in mind when using network analysis tools to debug network traffic!
