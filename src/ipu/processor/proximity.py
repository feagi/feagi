from math import sin, cos
from evo.neuron import neuron_finder
from inf import runtime_data


def detections_to_coords(proximity_data, proximity_type='LIDAR'):
    """ Converts coordinates from LIDAR detections to modified 
    Cartesian plane.

    :param proximity_data:
    :return:
    """
    detection_locations = []
    for sweep in proximity_data:
        for point in proximity_data[sweep][proximity_type]:
            h_angle = point[0]
            v_angle = point[1]
            distance = point[2]

            x = distance * sin(v_angle) * cos(h_angle)
            y = distance * sin(v_angle) * sin(h_angle)
            z = distance * cos(v_angle)

            detection_locations.append((x, y, z))
    
    return detection_locations


def locations_to_neuron_ids(detection_locations, cortical_area):
    """ Converts LIDAR detection locations (x, y, z) to neuron IDs in
    the corresponding cortical area.

    :param detection_locations:
    :param cortical_area:
    :return:
    """
    neuron_ids = []
    for i in range(len(detection_locations)):
        # found_neurons = neuron_finder(
        #     cortical_area, 
        #     detection_locations[i], 
        #     runtime_data.genome["location_tolerance"]
        # )    
        # if found_neurons:
        #     for neuron in found_neurons:
        #         neuron_ids.append(neuron)
        block_ref = block_ref_builder(detection_locations[i])
        if block_ref in runtime_data.block_dic[cortical_area]:
            block_neurons = runtime_data.block_dic[cortical_area][block_ref]
            for neuron in block_neurons:
                if neuron is not None and neuron not in neuron_ids:
                    neuron_ids.append(neuron)

    return neuron_ids


def block_ref_builder(locations):
    block_ref = ''
    for loc in locations:
        block_ref += str(round(loc))
    return '-'.join(block_ref)
