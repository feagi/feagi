# Spiking Neural Network Data Extraction and Transmission - High Performance Design

## Overview

This document outlines a high-performance data handling strategy for a spiking neural network (SNN) simulator involving large-scale neurons distributed across 3D cortical areas. The goal is to efficiently extract coordinate and membrane potential data of firing neurons and transmit this data over the network with minimal latency and maximal throughput.

---

## System Summary and Requirements

1. **Network structure:** Neurons exist in large 3D cortical areas. Each cortical area has a unique integer ID internally, but must be represented as a fixed 6-character ASCII string ID for transmission.

2. **Sparse firing:** At each timestep, only a sparse subset of neurons fire in each cortical area. These are stored as roaring bitmaps keyed by cortical area IDs.

3. **Data to transmit:** For each firing neuron, extract:
   - Coordinates (x, y, z) as int32 arrays
   - Membrane potentials (p) as float32 arrays

4. **Performance focus:** 
   - No concern for human readability.
   - Maximum memory and transmission efficiency.
   - Avoid costly serialization formats like JSON.
   - Minimize CPU/GPU data copying.
   - Exploit SIMD/SoA layouts.

---

## Data Structures

### Internal Representation (SoA)

- Store neuron properties in Structure of Arrays (SoA) form to optimize vectorized access and GPU usage:
  - `x: np.ndarray[int32]` shape=(capacity,)
  - `y: np.ndarray[int32]` shape=(capacity,)
  - `z: np.ndarray[int32]` shape=(capacity,)
  - `p: np.ndarray[float32]` shape=(capacity,)
- Keep membrane potentials and coordinates in separate contiguous arrays to benefit from SIMD and GPU-friendly memory layout.

### Firing Neurons Representation

- Use **Roaring Bitmaps** per cortical area to represent sparse sets of firing neuron indices efficiently.
- Example:
  ```python
  {
    100: roaring_bitmap_of_firing_neuron_ids_in_area_100,
    200: roaring_bitmap_of_firing_neuron_ids_in_area_200,
    ...
  }
````

---

## Data Extraction Steps (All in One)

1. **Iterate over cortical areas with firing neurons:**

   * For each cortical area integer ID, obtain the corresponding roaring bitmap of firing neurons.

2. **Convert cortical area integer ID to 6-character ASCII string ID:**

   * Use a fixed, lossless mapping (e.g., base-36 or custom encoding) to convert int → 6-letter string.
   * This 6-byte string will be included in the data packet header.

3. **Extract neuron data efficiently:**

   * Convert roaring bitmap to a NumPy array of neuron indices.
   * Use these indices to slice the SoA arrays (`x`, `y`, `z`, `p`) **directly** — no loops.
   * This yields four NumPy arrays per cortical area, each holding data only for firing neurons.

---

## Data Transmission Format (Raw Bytes)

### Motivation:

* Avoid text-based formats like JSON to eliminate serialization overhead.
* Transmit compact, binary data streams for lowest latency and bandwidth use.
* Allow direct memory-mapped reception on the other side without parsing text.

### Packet Structure per Cortical Area:

| Field               | Size (bytes) | Description                                    |
| ------------------- | ------------ | ---------------------------------------------- |
| Cortical Area ID    | 6            | ASCII string representing cortical area        |
| Number of Neurons   | 4            | uint32 number of firing neurons in this packet |
| x-coordinates       | 4 \* N       | int32 array of x coordinates                   |
| y-coordinates       | 4 \* N       | int32 array of y coordinates                   |
| z-coordinates       | 4 \* N       | int32 array of z coordinates                   |
| membrane potentials | 4 \* N       | float32 array of membrane potentials           |

Where N is the number of firing neurons in that cortical area.

---

## Implementation Notes

* Use `numpy.ndarray.tobytes()` for zero-copy conversion of arrays to raw bytes.
* Prepend the 6-byte ASCII cortical area ID and 4-byte neuron count as a fixed-length header.
* Concatenate header + raw byte arrays into a single byte buffer per cortical area.
* Send packets sequentially or in parallel over the network.
* On the receiver side:

  * Parse header (6 bytes + 4 bytes).
  * Read exact sizes for `x, y, z, p` arrays.
  * Reconstruct NumPy arrays using `np.frombuffer()` without copying.

---

## Justification of Choices

* **SoA layout:** Enables fast vectorized extraction of coordinates and membrane potentials.
* **Roaring bitmaps:** Efficient sparse representation of firing neurons; fast set operations and iteration.
* **6-character ASCII cortical IDs:** Compact, fixed length, easy to parse, human-readable if needed.
* **Raw bytes transmission:** Minimizes serialization overhead, maximizes throughput, enables zero-copy reception.
* **Fixed-length headers:** Allows easy framing and parsing on network stream.
* **Avoid JSON or text formats:** Eliminates CPU cycles wasted on string formatting/parsing and reduces data size.

---

## Summary

This approach maximizes efficiency for large-scale, sparse, real-time neuron data extraction and network transmission by combining:

* SIMD/GPU-friendly SoA memory layouts
* Roaring bitmap sparse sets
* Fixed-length ASCII cortical IDs
* Raw binary streaming with minimal headers

This design ensures your SNN simulator can scale to millions of neurons firing asynchronously across many cortical areas with minimal bottlenecks.

---

## Implementation Review and Refactoring Tracker

The following table tracks identified deviations between the architecture specification and the current implementation, along with refactoring needs:

| ID | Component | Current Implementation | Specification | Impact | Refactoring Needed | Status |
|----|-----------|------------------------|---------------|--------|-------------------|--------|
| 1 | Coordinate Storage | In Python, using a single structured array:<br>`self.coordinates = np.zeros(capacity, dtype=[('x', np.int32), ('y', np.int32), ('z', np.int32)])` | Separate arrays for x, y, z coordinates | Reduced SIMD/GPU efficiency | Refactor Python implementation to use separate arrays for x, y, z | Completed |
| 2 | Coordinate Data Type | Rust implementation uses u16:<br>`pub coordinates_x: Vec<u16>` | int32 for all coordinates | Potential range limitations and compatibility issues | Standardize coordinate types between Rust and Python (either stick with u16 and update spec, or implement int32) | Completed - Both implementations now use u32/uint32 |
| 3 | Binary Format | Additional metadata included (structure ID, version) beyond specification | Minimal header with ID and neuron count | Slightly increased packet size | Review need for extra metadata; document or update spec if needed | Pending |
| 4 | Data Extraction | Some code paths use loops for data extraction:<br>`for neuron in neurons: result += struct.pack("!I", neuron.get("x", 0))` | Vectorized operations with NumPy | Lower performance for data extraction | Replace loops with vectorized NumPy operations | Pending |
| 5 | Data Conversion | Inconsistent use of direct memory access methods | Use `numpy.ndarray.tobytes()` and `np.frombuffer()` for zero-copy operations | Memory copying overhead | Consistently implement zero-copy approaches | Pending |
| 6 | JSON Fallback | JSON encoding used in edge cases:<br>`return self.encoder.encode_json({"message_type": "neuron_data", "data": {}})` | Binary format only | Performance degradation in edge cases | Eliminate JSON fallbacks in favor of consistent binary encoding | Pending |
| 7 | Packet Structure | Some implementation variations in header structure | Fixed structure as defined in spec | Potential compatibility issues | Standardize packet structure across codebase | Pending |

### Additional Notes

1. The Roaring Bitmap implementation appears largely compliant with the architecture.
2. The cortical area ID encoding/decoding follows the 6-character ASCII specification.
3. The binary transmission approach generally follows the design but with some inefficiencies.

This tracker should be updated as refactoring tasks are completed to ensure alignment with the architecture specification.

