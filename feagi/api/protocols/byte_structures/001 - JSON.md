# 1 - JSON

**Current Version: 1**

**Size: Varies**

**ID: 1**

There are some use-cases (IE user requesting a change) where it is practical to use JSON, so we don't have to make 500 different protocols for trivial actions.

**Keep in mind JSON is slow in contrast to raw bytes due to parsing overhead, so do NOT use this for any sort of real time data streaming!**

#### Global Header

<table border="1" id="bkmrk-section-data-descrip" style="border-collapse: collapse; width: 100%; height: 118.067px;"><colgroup><col style="width: 50.0407%;"></col><col style="width: 25.0204%;"></col><col style="width: 25.0204%;"></col></colgroup><tbody><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Section Data Description  
</td><td style="height: 29.5167px;">Number of bytes  
</td><td style="height: 29.5167px;">Data Representation  
</td></tr><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Global Header ID  
</td><td style="height: 29.5167px;">1  
</td><td style="height: 29.5167px;">INT8  
</td></tr><tr style="height: 29.5167px;"><td style="height: 29.5167px;">Global Header Version  
</td><td style="height: 29.5167px;">1  
</td><td style="height: 29.5167px;">INT8</td></tr></tbody></table>

#### Data

<table border="1" id="bkmrk-r-pixel-1-uint8-g-pi" style="border-collapse: collapse; width: 100%; height: 88.5501px;"><tbody><tr style="height: 29.5167px;"><td style="height: 29.5167px; width: 33.3664%;">Section Data Description  
</td><td style="height: 29.5167px; width: 33.3664%;">Number of bytes  
</td><td style="height: 29.5167px; width: 33.3664%;">Data Representation  
</td></tr><tr><td style="width: 33.3664%;">JSON as UTF-8 string</td><td style="width: 33.3664%;">?</td><td style="width: 33.3664%;">UTF-8</td></tr></tbody></table>

### Request / Response (Direction Irrelevant)

Methodology is trivial. simply encode the JSON in UTF-8 byte encoding, and append it to the appropriate JSON header

Ergo, the following JSON

```json
{"Jensen": "I never asked for this"}
```

would be encoded as such (including the header)

```
\x01\x01\x7B\x22\x4A\x65\x6E\x73\x65\x6E\x22\x3A\x20\x22\x49\x20\x6E\x65\x76\x65\x72\x20\x61\x73\x6B\x65\x64\x20\x66\x6F\x72\x20\x74\x68\x69\x73\x22\x7D
```

Ergo when decoding, make sure you remove the first 2 bytes from the rest of the bytes before using your languages built in UTF-8 bytes to string conversion method (or simply use the library's built in tool to dump this back to a string).

Use the following tool to experiment for yourself (note the first 2 byte header, being "\\x01\\x01"

[Tool](https://mothereff.in/utf-8)
