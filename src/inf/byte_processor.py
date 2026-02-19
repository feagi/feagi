
import struct


def feagi_data_to_bytes(feagi_data):
    """
    Serializes FEAGI data into a compact binary format.

    Format for each cortical area block:
    - 6 bytes: Cortical area ID (ASCII, padded with nulls if shorter than 6)
    - 2 bytes: Number of data points (uint16, little endian)
    - For each data point:
        - 4 bytes: x coordinate (uint32, little endian)
        - 4 bytes: y coordinate (uint32, little endian)
        - 4 bytes: z coordinate (uint32, little endian)
        - 4 bytes: value (float32, little endian)

    Total per data point: 16 bytes

    Args:
        feagi_data (dict): A dictionary where keys are 6-character cortical area IDs,
                           and values are dictionaries mapping (x, y, z) tuples to float values.

    Returns:
        bytearray: The serialized binary representation of the FEAGI data.
    """
    result = bytearray()

    for cortical_id, coords in feagi_data.items():
        # Ensure cortical_id is 6 bytes, padded or trimmed
        cid = cortical_id.encode('ascii')[:6].ljust(6, b'\x00')
        result += cid

        # Write length (number of (x, y, z, v) sets)
        length = len(coords)
        result += struct.pack('<H', length)  # uint16, little endian

        # Add each (x, y, z, v)
        for (x, y, z), v in coords.items():
            result += struct.pack('<III f', x, y, z, v)  # 3x uint32, 1x float32

    return result


def bytes_to_feagi_data(data: bytes):
    """
    Deserializes binary FEAGI data into a structured dictionary.

    Expects the binary format to be:
    - 6 bytes: Cortical area ID (ASCII, padded with nulls if shorter than 6)
    - 2 bytes: Number of data points (uint16, little endian)
    - For each data point:
        - 4 bytes: x coordinate (uint32, little endian)
        - 4 bytes: y coordinate (uint32, little endian)
        - 4 bytes: z coordinate (uint32, little endian)
        - 4 bytes: value (float32, little endian)

    Total per data point: 16 bytes

    Args:
        data (bytes): Binary input data representing one or more cortical area blocks.

    Returns:
        dict: A dictionary where keys are cortical area IDs (str),
              and values are dictionaries mapping (x, y, z) tuples to float values.
    """
    feagi_data = {}
    offset = 0

    while offset < len(data):
        # Read 6-byte cortical area ID
        cortical_id = data[offset:offset + 6].rstrip(b'\x00').decode('ascii')
        offset += 6

        # Read length (number of (x, y, z, v) sets)
        length = struct.unpack_from('<H', data, offset)[0]
        offset += 2

        coords = {}
        for _ in range(length):
            x, y, z, v = struct.unpack_from('<III f', data, offset)
            coords[(x, y, z)] = v
            offset += 16  # 4 + 4 + 4 + 4 bytes

        feagi_data[cortical_id] = coords

    return feagi_data

