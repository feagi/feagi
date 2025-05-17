from .AbstractByteStructure import AbstractByteStructure
import struct


# TODO: We really should be using np arrays... however for now to ensure interoperability, use lists of tuples instead

class ActivatedNeuronLocation(AbstractByteStructure):
    """
    Essentially a list of coordinates of locations of the neurons that are activated
    """

    structure_id = 7


    def __init__(self, coordinates: list[tuple[int, int, int]]):
        self.activated_neuron_coordinates: list[tuple[int, int, int]] = coordinates

    @staticmethod
    def create_from_bytes(byte_array: bytes) -> 'ActivatedNeuronLocation':
        ActivatedNeuronLocation.confirm_header(byte_array) # throws an exception if something is wrong
        if (len(byte_array) - 2)  % 6 != 0:
            raise Exception("Invalid number of bytes for Activated Neuron location structure!")
        output: list[tuple[int, int, int]] = []
        index: int = 2
        while index < len(byte_array) - 2:
            appending: tuple[int, int, int] = (
                struct.unpack('<h', byte_array[index: index + 2])[0],
                struct.unpack('<h', byte_array[index + 2: index + 4])[0],
                struct.unpack('<h', byte_array[index + 4: index + 6])[0])
            output.append(appending)
            index += 6
        return ActivatedNeuronLocation(output)

    @staticmethod
    def create_from_list_of_tuples(coordinates: list[tuple[int, int, int]]) -> 'ActivatedNeuronLocation':
        return ActivatedNeuronLocation(coordinates)

    def to_bytes(self) -> bytes:
        output: bytearray = bytearray([0] * (len(self.activated_neuron_coordinates) // 6))
        offset: int = 0
        for activated_neuron_coordinate in self.activated_neuron_coordinates:
            output[offset: offset + 6] = struct.pack('<hhh', activated_neuron_coordinate[0], activated_neuron_coordinate[1], activated_neuron_coordinate[2])
            offset += 6
        return bytes (self._create_header() + output)

