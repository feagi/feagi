"""
Protocol definitions for FEAGI communication.
"""

from feagi_connector.protocols.constants import ProtocolID
from feagi_connector.protocols.neural import (
    NeuralProtocolID,
    NeuralDataHeader,
    CompressionType,
    NeuralPrecision,
    create_header,
    parse_header,
    encode_neuron_flat_data,
    create_neuron_flat_message,
    encode_cortical_area_to_id,
    NEURAL_HEADER_SIZE,
    NEURAL_MAGIC,
)

__all__ = [
    "NeuralProtocolID",
    "NeuralDataHeader", 
    "CompressionType",
    "NeuralPrecision",
    "create_header",
    "parse_header",
    "encode_neuron_flat_data",
    "create_neuron_flat_message",
    "encode_cortical_area_to_id",
    "NEURAL_HEADER_SIZE",
    "NEURAL_MAGIC",
] 