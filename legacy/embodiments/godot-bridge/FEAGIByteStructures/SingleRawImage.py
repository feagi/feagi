from .AbstractByteStructure import AbstractByteStructure
import struct
import numpy as np
import numpy.typing as npt # PROPER TYPE SAFETY IS SUCH A GREAT EXPERIENCE IN PYTHON! /s

class SingleRawImage(AbstractByteStructure):
    """
    Encodes a raw image in BGR format with resolution information
    """

    structure_id = 8

    def __init__(self, resolution: tuple[int, int], raw_data_BGR: npt.NDArray[np.uint8]):
        self.resolution: tuple[int, int] = resolution
        self.raw_data_BGR: npt.NDArray[np.uint8] = raw_data_BGR

    @staticmethod
    def create_from_bytes(byte_array: bytes, flip_BR_color_channels: bool = False) -> 'SingleRawImage':
        SingleRawImage.confirm_header(byte_array) # throws an exception if something is wrong
        if len(byte_array) < 4:
            raise Exception("Missing sub-header for 'SingleRawImage'!")

        if (len(byte_array) - 4) % 3 != 0: # global header + sub header
            raise Exception("Invalid number of bytes for 'SingleRawImage'!")

        resolution: tuple[int, int] = struct.unpack('<hh', byte_array[2:4]) # TODO how can I define types for this properly?

        if len(byte_array) - 4 != resolution[0] * resolution[1] * 3:
            raise Exception(f"number of data bytes '{len(byte_array) - 4}' does not match expected given resolution '{resolution[0]}' and '{resolution[1]}' for 'SingleRawImage'!")
        array: npt.NDArray[np.uint8] = np.array(byte_array[4:], dtype=np.uint8)
        if flip_BR_color_channels:
            array[0::3], array[2::3] = array[2::3], array[0::3].copy() # TODO currently optimizing for speed here by keeping this entirely within numpy, but perhaps we can also reduce memory usage a bit if we find a method that doesn't need to copy a third of the array?
        return SingleRawImage(resolution, array)


    @staticmethod
    def create_from_FEAGI_delta_dict(resolution: tuple[int, int], feagi_RGB_dict: dict[tuple[int,int,int]: int]) -> 'SingleRawImage':
        """
        Given a feagi dictionary of what pixels (and their coordinates) have changed, update the data
        """
        # Where the FEAGI dict is of structure {(x_coord,y_coord,rgb_channel_index): pixel_value}
        array: npt.NDArray[np.uint8] = np.zeros((resolution[0], resolution[1], 3), dtype=np.uint8) # start all black
        if len(feagi_RGB_dict) != 0: # no need to do all this if nothing changed
            try:
                for xycoord_rgbindex_tuple in feagi_RGB_dict:
                    array[xycoord_rgbindex_tuple] = feagi_RGB_dict[xycoord_rgbindex_tuple]  # set the correct X Y color_index coordinate to the correct pixel and channel value
                array = np.rot90(array, k=1, axes=(0, 1)) # rotate the image
                array[:, :, [0, 2]] = array[:, :, [2, 0]] # Swap R and B channels to convert from RGB to BGR
            except:
                # FEAGI sent something invalid. # TODO we should have some sort of logging
                pass

        array = array.flatten()
        return SingleRawImage(resolution, array)

    def to_bytes(self) -> bytes:
        return self._create_header() + struct.pack('<hh', self.resolution[0], self.resolution[1]) + bytes(self.raw_data_BGR)
