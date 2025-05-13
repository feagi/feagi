
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

def parse_rust_byte_data(data: bytes):
    result = {}

    # Constants
    BYTE_STRUCT_ID = 11
    BYTE_STRUCT_VERSION = 1
    GLOBAL_HEADER_SIZE = 2
    CORTICAL_COUNT_HEADER_SIZE = 2
    PER_CORTICAL_HEADER_DESCRIPTOR_SIZE = 14
    PER_NEURON_XYZP_SIZE = 16

    # Validate header
    if data[0] != BYTE_STRUCT_ID or data[1] != BYTE_STRUCT_VERSION:
        raise ValueError("Invalid byte structure ID or version")

    cortical_count = int.from_bytes(data[2:4], 'little')

    headers = []
    for i in range(cortical_count):
        start = GLOBAL_HEADER_SIZE + CORTICAL_COUNT_HEADER_SIZE + i * PER_CORTICAL_HEADER_DESCRIPTOR_SIZE
        end = start + PER_CORTICAL_HEADER_DESCRIPTOR_SIZE
        header = data[start:end]
        cortical_id = header[0:6].decode('ascii')
        offset = int.from_bytes(header[6:10], 'little')
        length = int.from_bytes(header[10:14], 'little')
        headers.append((cortical_id, offset, length))

    for cortical_id, offset, length in headers:
        x_vals, y_vals, z_vals, p_vals = [], [], [], []
        for i in range(offset, offset + length, PER_NEURON_XYZP_SIZE):
            x, y, z, p = struct.unpack_from('<ffff', data, i)
            x_vals.append(x)
            y_vals.append(y)
            z_vals.append(z)
            p_vals.append(p)
        result[cortical_id] = [x_vals, y_vals, z_vals, p_vals]

    return result


def parse_byte_structure_xyz_to_dict_with_tuples(data: bytes) -> dict:
    result = {}

    # Constants
    GLOBAL_HEADER_SIZE = 2
    CORTICAL_COUNT_HEADER_SIZE = 2
    HEADER_ENTRY_SIZE = 14
    HEADER_START = GLOBAL_HEADER_SIZE + CORTICAL_COUNT_HEADER_SIZE

    # Read cortical area count (u16 LE)
    cortical_count = struct.unpack_from('<H', data, 2)[0]

    for i in range(cortical_count):
        offset = HEADER_START + i * HEADER_ENTRY_SIZE

        cortical_id = data[offset:offset+6].decode('ascii')
        data_start = struct.unpack_from('<I', data, offset + 6)[0]
        data_len = struct.unpack_from('<I', data, offset + 10)[0]

        segment = data[data_start:data_start + data_len]
        num_neurons = data_len // 16

        neurons = {}
        for j in range(num_neurons):
            x, y, z = struct.unpack_from('<III', segment, j * 16)
            v = struct.unpack_from('<f', segment, j * 16 + 12)[0]
            neurons[(x, y, z)] = v

        result[cortical_id] = neurons

    return result