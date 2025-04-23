
import struct


def feagi_data_to_bytes(feagi_data):
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
            result += struct.pack('<HHHf', x, y, z, v)  # 3x uint16, 1x float32

    return result


def bytes_to_feagi_data(data: bytes):
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
            x, y, z, v = struct.unpack_from('<HHHf', data, offset)
            coords[(x, y, z)] = v
            offset += 10  # 2 + 2 + 2 + 4 bytes

        feagi_data[cortical_id] = coords

    return feagi_data

