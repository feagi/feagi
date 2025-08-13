"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

genome_properties = {
    "structure": {
        "segment_guide": "________-______-__-______-_",
        "segment_count": 5,
        "segment_seperator": "-",
        "cortical_id_length": 6,
    },
    "position": {
        "gene": [0, 0, 26],
        "expression_parameters": [1, 0, 7],
        "cortical_id": [2, 9, 14],
        "gene_classifier": [3, 16, 17],
        "encoding_id": [4, 19, 24],
        "value_type": [5, 26, 26],
    },
    "value": {"gene_classifier": ["nx", "cx"], "value_type": ["b", "i", "f", "d", "t"]},
}
