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
# ======================================================================

"""
Modern brain region management for FEAGI (inspired by legacy region.py).
Uses FeagiStateManager for state and is compatible with the new connectome architecture.
"""

import random
import string
from datetime import datetime
from time import time

from feagi.core.state_manager import FeagiStateManager

# To break circular imports, don't import these at the module level
# Instead, import them when needed in the functions that use them
# from feagi.evo.genome_processor import genome_v1_v2_converter
# from feagi.evo.genome_editor import generate_hash

state = FeagiStateManager.instance()


def region_id_2_title(region_id):
    """Return the title of a brain region by its ID."""
    brain_regions = state.genome.get("brain_regions", {})
    if region_id in brain_regions:
        return brain_regions[region_id].get("title")
    return None


def construct_genome_from_region(region_id):
    """Construct a genome payload for a given brain region ID."""
    from feagi.evo.genome_processor import genome_v1_v2_converter

    # Use our local implementation instead of importing
    # from feagi.evo.genome_editor import generate_hash

    genome = state.genome
    brain_regions = genome.get("brain_regions", {})
    if region_id not in brain_regions:
        raise ValueError(f"Region ID {region_id} not found in genome.")

    region = brain_regions[region_id]

    # Gather all areas and subregions recursively
    def direct_region_cortical_areas(region_id):
        return brain_regions[region_id].get("areas", [])

    def recursive_sub_regions(region_id):
        region_list = set(brain_regions[region_id].get("regions", []))
        return region_list

    def recursive_region_cortical_areas(region_id):
        recursive_region_list = recursive_sub_regions(region_id)
        area_list = set(direct_region_cortical_areas(region_id))
        for sub_region_id in recursive_region_list:
            area_list.update(set(direct_region_cortical_areas(sub_region_id)))
        return area_list

    # region_cortical_list = direct_region_cortical_areas(region_id)
    # Unused variable removed
    comprehensive_subregion_list = recursive_sub_regions(region_id)
    comprehensive_area_list = recursive_region_cortical_areas(region_id)

    # Build the genome payload
    genome_from_region = {
        "genome_title": region.get("title", "No Title"),
        "genome_description": region.get("description", "No Description"),
        "timestamp": time(),
        "genome_id": f"{datetime.now().strftime('%Y%m%d%H%M%S%f')[2:]}_{''.join(random.choices(string.ascii_uppercase + string.digits, k=6))}_R",
        "version": genome.get("version", "2.0"),
        "signatures": {},
        "stats": {},
        "hosts": {},
        "physiology": genome.get("physiology", {}),
        "neuron_morphologies": {},
        "blueprint": {},
        "brain_regions": {
            "root": {
                "title": region.get("title", "No Title"),
                "parent_region_id": None,
                "coordinate_2d": [0, 0],
                "coordinate_3d": [0, 0, 0],
                "areas": region.get("areas", []),
                "regions": region.get("regions", []),
                "inputs": [],
                "outputs": [],
            }
        },
    }

    # Add blueprint for all areas in the region
    for cortical_area in comprehensive_area_list:
        if cortical_area in genome["blueprint"]:
            genome_from_region["blueprint"][cortical_area] = genome["blueprint"][
                cortical_area
            ].copy()

    # Add subregions
    genome_from_region["brain_regions"]["root"] = region.copy()
    genome_from_region["brain_regions"]["root"]["parent_region_id"] = None
    genome_from_region["brain_regions"]["root"]["coordinate_2d"] = [0, 0]
    genome_from_region["brain_regions"]["root"]["coordinate_3d"] = [0, 0, 0]
    for subregion in comprehensive_subregion_list:
        genome_from_region["brain_regions"][subregion] = brain_regions[subregion].copy()

    # Set signatures
    genome_from_region["signatures"]["genome"] = generate_hash(genome_from_region)
    genome_from_region["signatures"]["blueprint"] = generate_hash(
        genome_from_region["blueprint"]
    )
    genome_from_region["signatures"]["physiology"] = generate_hash(
        genome_from_region["physiology"]
    )

    # Convert Genome to 2.0
    genome_from_region["blueprint"] = genome_v1_v2_converter(genome_from_region)[
        "blueprint"
    ]

    return genome_from_region


def region_id_gen(
    size: int = 6, chars: str = string.ascii_uppercase + string.digits
) -> str:
    """
    Generate a unique region ID using timestamp and random characters.
    """
    now = datetime.now()
    return f"{now.strftime('%Y%m%d%H%M%S%f')[2:]}_{''.join(random.choice(chars) for _ in range(size))}_R"


def change_cortical_area_parent(cortical_area_id: str, new_parent_id: str) -> None:
    """
    Change the parent region of a cortical area.
    Updates both the association and the region membership lists.
    """
    try:
        current_parent_id = state.cortical_area_region_association.get(cortical_area_id)
        if current_parent_id is None:
            raise ValueError(
                f"Cortical area {cortical_area_id} has no current parent region."
            )
        state.cortical_area_region_association[cortical_area_id] = new_parent_id
        # Remove from old region
        if (
            cortical_area_id
            in state.genome["brain_regions"][current_parent_id]["areas"]
        ):
            state.genome["brain_regions"][current_parent_id]["areas"].remove(
                cortical_area_id
            )
        # Add to new region
        if (
            cortical_area_id
            not in state.genome["brain_regions"][new_parent_id]["areas"]
        ):
            state.genome["brain_regions"][new_parent_id]["areas"].append(
                cortical_area_id
            )
    except Exception as e:
        raise RuntimeError(f"Failed to change cortical area parent: {e}") from e


def change_brain_region_parent(region_id: str, new_parent_id: str) -> None:
    """
    Change the parent region of a brain region.
    Updates the parent and membership lists.
    """
    brain_regions = state.genome["brain_regions"]
    current_parent_id = brain_regions[region_id]["parent_region_id"]
    brain_regions[region_id]["parent_region_id"] = new_parent_id
    if region_id in brain_regions[current_parent_id]["regions"]:
        brain_regions[current_parent_id]["regions"].remove(region_id)
    if region_id not in brain_regions[new_parent_id]["regions"]:
        brain_regions[new_parent_id]["regions"].append(region_id)


def create_region(region_data) -> str:
    """
    Create a new brain region and update the genome/state.
    region_data: should have title, region_description, parent_region_id, coordinates_2d, coordinates_3d, areas, regions
    Returns the new region_id.
    """
    region_id = region_id_gen()
    state.genome["brain_regions"][region_id] = {
        "title": region_data.title,
        "description": getattr(region_data, "region_description", ""),
        "parent_region_id": region_data.parent_region_id,
        "coordinate_2d": getattr(region_data, "coordinates_2d", [0, 0]),
        "coordinate_3d": getattr(region_data, "coordinates_3d", [0, 0, 0]),
        "areas": [],
        "regions": [],
        "inputs": [],
        "outputs": [],
        "signature": "",
    }
    # Add to parent's regions list
    state.genome["brain_regions"][region_data.parent_region_id]["regions"].append(
        region_id
    )
    # Associate areas
    if hasattr(region_data, "areas") and region_data.areas:
        for associated_area in region_data.areas:
            if associated_area in state.cortical_list:
                change_cortical_area_parent(
                    cortical_area_id=associated_area, new_parent_id=region_id
                )
    # Associate subregions
    if hasattr(region_data, "regions") and region_data.regions:
        for associated_region in region_data.regions:
            if associated_region in state.genome["brain_regions"]:
                state.genome["brain_regions"][region_id]["regions"].append(
                    associated_region
                )
    return region_id


def update_region(region_data: dict) -> None:
    """
    Update properties of a brain region.
    region_data must include 'region_id'.
    """
    region_id = region_data["region_id"]
    region_data = dict(region_data)  # Copy to avoid mutating input
    region_data.pop("region_id")
    for update in region_data:
        if update not in ["area", "region"]:
            if update == "parent_region_id":
                change_brain_region_parent(
                    region_id=region_id, new_parent_id=region_data["parent_region_id"]
                )
            else:
                state.genome["brain_regions"][region_id][update] = region_data[update]
        else:
            raise ValueError(f"{update} cannot be updated using this endpoint")


def delete_region_with_members(region_id: str) -> None:
    """
    Delete a region and reassign its areas and subregions to its parent.
    """
    if region_id in state.genome["brain_regions"]:
        parent_region = state.genome["brain_regions"][region_id]["parent_region_id"]
        # Move areas to parent
        for area in state.genome["brain_regions"][region_id]["areas"]:
            change_cortical_area_parent(
                cortical_area_id=area, new_parent_id=parent_region
            )
        # Move subregions to parent
        for subregion in state.genome["brain_regions"][region_id]["regions"]:
            change_brain_region_parent(region_id=subregion, new_parent_id=parent_region)
        # Remove from parent's regions list
        if region_id in state.genome["brain_regions"][parent_region]["regions"]:
            state.genome["brain_regions"][parent_region]["regions"].remove(region_id)
        # Delete the region
        state.genome["brain_regions"].pop(region_id)


def relocate_region_members(relocation_data: dict) -> None:
    """
    Relocate areas or regions (update coordinates and/or parent region).
    relocation_data: dict of object_id -> {coordinate_2d, parent_region_id}
    """
    for object_id in relocation_data:
        if object_id in state.genome["blueprint"]:
            if "coordinate_2d" in relocation_data[object_id]:
                state.genome["blueprint"][object_id]["2d_coordinate"][0] = (
                    relocation_data[object_id]["coordinate_2d"][0]
                )
                state.genome["blueprint"][object_id]["2d_coordinate"][1] = (
                    relocation_data[object_id]["coordinate_2d"][1]
                )
            if "parent_region_id" in relocation_data[object_id]:
                change_cortical_area_parent(
                    cortical_area_id=object_id,
                    new_parent_id=relocation_data[object_id]["parent_region_id"],
                )
        elif object_id in state.genome["brain_regions"]:
            if "coordinate_2d" in relocation_data[object_id]:
                state.genome["brain_regions"][object_id]["coordinate_2d"][0] = (
                    relocation_data[object_id]["coordinate_2d"][0]
                )
                state.genome["brain_regions"][object_id]["coordinate_2d"][1] = (
                    relocation_data[object_id]["coordinate_2d"][1]
                )
            if "parent_region_id" in relocation_data[object_id]:
                if (
                    relocation_data[object_id]["parent_region_id"]
                    in state.genome["brain_regions"]
                ):
                    change_brain_region_parent(
                        region_id=object_id,
                        new_parent_id=relocation_data[object_id]["parent_region_id"],
                    )
                else:
                    raise ValueError(
                        f"{relocation_data[object_id]['parent_region_id']} "
                        f"is not a valid region id"
                    )
        else:
            raise ValueError(f"{object_id} is not a valid region nor cortical id")
    # Optionally, update cached dimensions or other state as needed


class BrainRegion:
    """
    Represents a brain region containing multiple cortical areas.

    A brain region is a logical grouping of cortical areas that are
    functionally related.
    """

    def __init__(
        self,
        region_id: str,
        name: str,
        region_type: str = "custom",
        properties: dict = None,
    ):
        """
        Initialize a brain region.

        Args:
            region_id: Unique identifier for the region
            name: Human-readable name of the region
            region_type: Type of region (e.g., "sensory", "motor", "custom")
            properties: Additional properties as key-value pairs
        """
        self.id = region_id
        self.name = name
        self.region_type = region_type
        self.properties = properties or {}
        self.cortical_areas = set()  # Set of cortical area IDs

    def add_area(self, area_id: str) -> None:
        """
        Add a cortical area to this region.

        Args:
            area_id: ID of the cortical area to add
        """
        self.cortical_areas.add(area_id)

    def remove_area(self, area_id: str) -> bool:
        """
        Remove a cortical area from this region.

        Args:
            area_id: ID of the cortical area to remove

        Returns:
            True if area was removed, False if area was not in the region
        """
        if area_id in self.cortical_areas:
            self.cortical_areas.remove(area_id)
            return True
        return False

    def contains_area(self, area_id: str) -> bool:
        """
        Check if the region contains a specific cortical area.

        Args:
            area_id: ID of the cortical area to check

        Returns:
            True if the area is in this region, False otherwise
        """
        return area_id in self.cortical_areas

    def get_all_areas(self) -> list:
        """
        Get a list of all cortical area IDs in this region.

        Returns:
            List of cortical area IDs
        """
        return list(self.cortical_areas)

    def to_dict(self) -> dict:
        """
        Convert the brain region to a dictionary representation.

        Returns:
            Dictionary representation of the brain region
        """
        return {
            "id": self.id,
            "name": self.name,
            "region_type": self.region_type,
            "cortical_areas": list(self.cortical_areas),
            "properties": self.properties,
        }

    def update(self, updates: dict) -> None:
        """
        Update brain region properties.

        Args:
            updates: Dictionary of properties to update

        Raises:
            KeyError: If an invalid property is specified
        """
        valid_properties = {"name", "region_type", "properties"}

        for key, value in updates.items():
            if key not in valid_properties:
                raise KeyError(f"Invalid property: {key}")

            if key == "properties":
                self.properties.update(value)
            else:
                setattr(self, key, value)

    def clear_areas(self) -> None:
        """
        Remove all cortical areas from this region.
        """
        self.cortical_areas.clear()


def generate_hash(data):
    """Simple placeholder for the hash generation function.
    This allows us to avoid importing the actual function and breaking the
    circular imports.
    """
    import hashlib
    import json

    # Convert the data to a string representation
    if isinstance(data, (dict, list)):
        data_str = json.dumps(data, sort_keys=True)
    else:
        data_str = str(data)

    # Generate a hash
    return hashlib.md5(data_str.encode()).hexdigest()
