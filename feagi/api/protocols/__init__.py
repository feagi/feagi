#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""
FEAGI Protocol Package - Byte Structures Implementation

This module provides protocol definitions for binary communication between FEAGI
and its clients using specialized byte structures optimized for neural data.
"""

# Import from the PyPI feagi-data-processing package
import feagi_data_processing as fdp

# Import protocol definitions and constants
from feagi.api.protocols.constants import ByteStructureID, ProtocolID


# Create compatibility classes to maintain the existing API
class ByteStructureEncoder:
    """Compatibility wrapper for feagi_data_processing encoding"""

    def __init__(self):
        self.fdp = fdp

    def encode_json(self, data: dict) -> bytes:
        """Encode JSON data to FeagiByteStructure format"""
        import json

        json_bytes = json.dumps(data).encode("utf-8")
        # For JSON, we'll use a simple approach for now
        return json_bytes

    def encode_neuron_data(self, neuron_data: list) -> bytes:
        """Encode neuron data to FeagiByteStructure format using high-performance NumPy arrays"""
        try:
            import numpy as np

            if not neuron_data:
                return b""

            # Create the main mapped neuron data container
            generated_mapped_neuron_data = (
                self.fdp.neuron_data.neuron_mappings.CorticalMappedXYZPNeuronData()
            )

            # Organize data by cortical area for high-performance encoding
            cortical_areas = {}

            for neuron in neuron_data:
                if isinstance(neuron, dict):
                    cortical_id = str(neuron.get("cortical_id", "0"))
                    if cortical_id not in cortical_areas:
                        cortical_areas[cortical_id] = {
                            "x": [],
                            "y": [],
                            "z": [],
                            "p": [],
                        }

                    cortical_areas[cortical_id]["x"].append(neuron.get("x", 0))
                    cortical_areas[cortical_id]["y"].append(neuron.get("y", 0))
                    cortical_areas[cortical_id]["z"].append(neuron.get("z", 0))
                    cortical_areas[cortical_id]["p"].append(neuron.get("p", 0))

            # Use high-performance approach for each cortical area
            for cortical_id, coords in cortical_areas.items():
                if not coords["x"]:  # Skip empty areas
                    continue

                # Create NumPy arrays with proper dtypes
                neurons_x = np.asarray(coords["x"], dtype=np.uint32)
                neurons_y = np.asarray(coords["y"], dtype=np.uint32)
                neurons_z = np.asarray(coords["z"], dtype=np.uint32)
                neurons_p = np.asarray(coords["p"], dtype=np.float32)

                # Create cortical ID
                cortical_id_obj = self.fdp.cortical_data.CorticalID(cortical_id)

                # Use high-performance NumPy approach
                neurons_array = (
                    self.fdp.neuron_data.neuron_arrays.NeuronXYZPArrays.new_from_numpy(
                        neurons_x, neurons_y, neurons_z, neurons_p
                    )
                )

                # Insert the neuron array into the mapped data with its cortical ID
                generated_mapped_neuron_data.insert(cortical_id_obj, neurons_array)

            # Create the final byte structure from the mapped data
            byte_structure = generated_mapped_neuron_data.as_new_feagi_byte_structure()
            return byte_structure.get_data_as_bytes()

        except Exception:
            # Fallback to simple JSON encoding
            import json

            return json.dumps(neuron_data).encode("utf-8")


class ByteStructureDecoder:
    """Compatibility wrapper for feagi_data_processing decoding"""

    def __init__(self):
        self.fdp = fdp

    def decode_message(self, data: bytes) -> dict:
        """Decode FeagiByteStructure format to dictionary"""
        # Try to create a FeagiByteStructure from the bytes
        try:
            byte_structure = self.fdp.byte_structures.FeagiByteStructure(data)
            structure_type = byte_structure.try_get_structure_type()

            if structure_type == 11:  # NeuronCategoricalXYZP
                # Create CorticalMappedXYZPNeuronData from the byte structure
                cortical_mapped = (
                    self.fdp.neuron_data.neuron_mappings.CorticalMappedXYZPNeuronData()
                )
                cortical_mapped.from_feagi_byte_structure(byte_structure)

                # Extract data (this is a simplified extraction)
                return {"type": "neuron_data", "structure_type": structure_type}
            else:
                return {"type": "unknown", "structure_type": structure_type}

        except Exception:
            # Fallback to JSON parsing
            try:
                import json

                return json.loads(data.decode("utf-8"))
            except Exception as e:
                return {"error": f"Could not decode message: {e}"}


class ByteStructureTranslator:
    """Compatibility wrapper providing translation methods"""

    def __init__(self):
        self.encoder = ByteStructureEncoder()
        self.decoder = ByteStructureDecoder()
        self.fdp = fdp

    def create_message(self, data: dict) -> bytes:
        """Create a binary message from dictionary data"""
        return self.encoder.encode_json(data)

    def decode_message(self, data: bytes) -> dict:
        """Decode a binary message to dictionary"""
        return self.decoder.decode_message(data)


# Re-export for backward compatibility
from feagi.api.protocols.translator import default_translator

__all__ = [
    "ProtocolID",
    "ByteStructureID",
    "ByteStructureEncoder",
    "ByteStructureDecoder",
    "ByteStructureTranslator",
    "default_translator",
]
