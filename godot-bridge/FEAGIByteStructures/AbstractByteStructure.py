from abc import ABC, abstractmethod
from typing import Type
import struct

class AbstractByteStructure(ABC):
    """
    Root parent class for all possible byte structures that are used within FEAGI
    """

    structure_id: int = -1 # every child class must override this
    structure_version: int = 1 # In most cases this will be 1

    @staticmethod
    @abstractmethod
    def create_from_bytes(byte_array: bytes) -> Type['AbstractByteStructure']:
        pass

    @abstractmethod
    def to_bytes(self) -> bytes:
        """
        Returns this object as a properly formatted byte array with the correct wrapping
        """
        pass

    @classmethod
    def confirm_header(cls, byte_array: bytes) -> bool:
        """
        Returns true if the byte header is expected of this class. else throws an exception
        """
        if len(byte_array) < 3:
            raise Exception("Byte structure is missing initial header!")
        if int(byte_array[0]) != cls.structure_id:
            raise Exception(f"Byte structure ID is {int(byte_array[0])} when expected {cls.structure_id}!")
        if int(byte_array[1]) != cls.structure_version:
            raise Exception(f"Byte structure ID is {int(byte_array[1])} when expected {cls.structure_version}!")
        return True

    def _create_header(self) -> bytes:
        return bytes(struct.pack('bb', self.structure_id, self.structure_version))
