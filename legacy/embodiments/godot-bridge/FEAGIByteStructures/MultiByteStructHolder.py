from .AbstractByteStructure import AbstractByteStructure
from .ActivatedNeuronLocation import ActivatedNeuronLocation
from .JSONByteStructure import JSONByteStructure
from .SingleRawImage import SingleRawImage
import struct

class MultiByteStructHolder(AbstractByteStructure):
    """
    Essentially a list of other ByteStuctures
    """

    structure_id = 9


    def __init__(self, byte_structures: list[AbstractByteStructure]):
        self.byte_structures: list[AbstractByteStructure] = byte_structures


    @staticmethod
    def create_from_bytes(byte_array: bytes) -> 'MultiByteStructHolder':

        def attempt_to_create_byte_structure_from_bytes(byte_array: bytes) -> AbstractByteStructure:
            """
            Given a sequence of bytes, attempts to create to correct Byte Structure from it
            """
            if len(byte_array) < 3:
                raise Exception("Byte structure is missing initial header!")
            structure_type: int = byte_array[0]
            match structure_type:
                case 1:
                    return JSONByteStructure.create_from_bytes(byte_array)
                case 7:
                    return ActivatedNeuronLocation.create_from_bytes(byte_array)
                case 8:
                    return SingleRawImage.create_from_bytes(byte_array)
                case 9:
                    return MultiByteStructHolder.create_from_bytes(byte_array)
            raise Exception(f"Unable to generate unsupported byte structure of type {structure_type}!")

        MultiByteStructHolder.confirm_header(byte_array)  # throws an exception if something is wrong
        if len(byte_array) < 3:
            raise Exception("Missing data fro initial sub-header for 'MultiByteStructHolder'!")
        number_sub_byte_structs: int = byte_array[3]
        if len(byte_array) < 3 + (8 * number_sub_byte_structs):
            raise Exception("Sub-header size for 'MultiByteStructHolder' is smaller than expected!")
        parsed_byte_structures: list[AbstractByteStructure] = []
        header_offset: int = 3
        for byte_structure_index in range(number_sub_byte_structs):
            sub_struct_start_index, sub_struct_length = struct.unpack('<II', header_offset)
            sub_struct_bytes: bytes = byte_array[sub_struct_start_index:sub_struct_start_index + sub_struct_length]
            try:
                parsed_byte_structures.append(attempt_to_create_byte_structure_from_bytes(sub_struct_bytes))
            except Exception as e:
                print(f"Unable to parse one of the byte structures as a AbstractByteStructure member: {e}")
            header_offset += 8
        return MultiByteStructHolder(parsed_byte_structures)

    @staticmethod
    def create_from_list_of_bytestructs(byte_structures: list[AbstractByteStructure]) -> 'MultiByteStructHolder':
        return MultiByteStructHolder(byte_structures)

    def to_bytes(self) -> bytes:

        output: bytearray = bytearray(self._create_header() + bytes([len(self.byte_structures)]) + bytes([0] * 8 * len(self.byte_structures))) ## allocate space for header

        # process subheader indexes for contained byte structure locations, and append data
        sub_header_index_offset: int = 3  # skipping global header, and the initial sub_struct count
        for byte_structure in self.byte_structures:
            data_start_offset: int = len(output)  # where we start reading data for this byte structure
            data: bytes = byte_structure.to_bytes()
            output[sub_header_index_offset: sub_header_index_offset + 8] = struct.pack('<II',data_start_offset, len(data))
            output += data
            sub_header_index_offset += 8
        return bytes(output)

    def get_number_of_contained_structures(self) -> int:
        """
        Returns the number of contained structures
        """
        return len(self.byte_structures)

    def is_empty(self) -> bool:
        """
        Returns True if this structure contains no structures within itself
        """
        return len(self.byte_structures) == 0