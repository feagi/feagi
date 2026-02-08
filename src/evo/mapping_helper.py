from math import floor
from random import randrange

from src.inf import runtime_data
from src.evo.templates import cortical_types
from src.evo.x_genesis import update_cortical_mappings, add_core_cortical_area


def generate_detailed_cortical_map():
    cortical_map = dict()
    for cortical_area in runtime_data.genome["blueprint"]:
        cortical_map[cortical_area] = dict()
        for dst in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"]:
            cortical_map[cortical_area][dst] = list()
            for mapping in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"][dst]:
                cortical_map[cortical_area][dst].append(mapping)
    return cortical_map


def build_power_connections(target_area_id: str, cortical_type: str,  mapping_dict: dict):
    power_area = "___pwr"
    cortical_template = cortical_types[cortical_type]["supported_devices"][target_area_id].copy()

    if target_area_id not in runtime_data.genome["blueprint"]:
        add_core_cortical_area(cortical_properties={
            "cortical_id": target_area_id,
            "cortical_type": cortical_type,
            "cortical_name": cortical_template["cortical_name"],
            "coordinates_3d": cortical_template["coordinate_3d"],
            "dev_count": 1,
            "coordinates_2d": [randrange(0, 10), randrange(0, 10)]
        })

    target_area_width = runtime_data.genome["blueprint"][target_area_id]["block_boundaries"][0]
    target_area_depth = runtime_data.genome["blueprint"][target_area_id]["block_boundaries"][2]

    morphology_template = {
        "parameters": {
            "patterns": []
        },
        "type": "patterns",
        "class": "custom"
    }

    for entry in mapping_dict:
        if mapping_dict[entry] or mapping_dict[entry] == 0:
            if int(entry) < target_area_width and 0 <= mapping_dict[entry] <= 1:
                target_voxel = floor((target_area_depth - 1) * mapping_dict[entry])
                morphology_template["parameters"]["patterns"].append([[0, 0, 0], [int(entry), 0, target_voxel]])

    # check if prior morphology exists
    power_mappings = runtime_data.genome["blueprint"][power_area].get("cortical_mapping_dst")
    if power_mappings:
        existing_power_to_target_area = power_mappings.get(target_area_id)
        if existing_power_to_target_area:
            # remove existing mapping
            del runtime_data.genome["blueprint"][power_area]["cortical_mapping_dst"][target_area_id]

    morphology_name = "system-" + power_area + "-" + target_area_id

    # Create new morphology
    runtime_data.genome["neuron_morphologies"][morphology_name] = morphology_template

    mapping_data = [{
        'morphology_id': morphology_name,
        'morphology_scalar': [1, 1, 1],
        'plasticity_flag': False,
        'postSynapticCurrent_multiplier': 1}]

    # create new mapping
    update_cortical_mappings({
        "src_cortical_area": power_area,
        "dst_cortical_area": target_area_id,
        "mapping_data": mapping_data
    })


def remove_power_connection(target_area_id: str, x_index, delete_morphology):
    morphology_list = set()
    # Fetch existing incoming cortical mappings
    existing_power_mappings = generate_detailed_cortical_map().get("___pwr")
    if existing_power_mappings:
        power_mappings_to_target_area = existing_power_mappings.get(target_area_id)

        if power_mappings_to_target_area:
            for mapping in power_mappings_to_target_area:
                morphology_list.add(mapping["morphology_id"])

    # Identify matching mapping


    # Remove the mapping

    # Delete associated morphology (optional)




