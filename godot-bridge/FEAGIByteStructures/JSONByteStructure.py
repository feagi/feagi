from .AbstractByteStructure import AbstractByteStructure

class JSONByteStructure(AbstractByteStructure):

    structure_id: int = 1

    def __init__(self, json_string: str):
        self.json_string: str = json_string

    @staticmethod
    def create_from_bytes(byte_array: bytes) -> 'JSONByteStructure':
        JSONByteStructure.confirm_header(byte_array) # throws an exception if something is wrong
        return JSONByteStructure(byte_array[2:].decode('utf-8'))


    @staticmethod
    def create_from_json_string(json_string: str) -> 'JSONByteStructure':
        return JSONByteStructure(json_string)

    def to_bytes(self) -> bytes:
        return bytes(self._create_header() + self.json_string.encode('utf-8'))


