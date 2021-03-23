from math import sin, cos, sqrt, inf
from evo.neuron import block_reference_builder
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


def coords_to_neuron_ids(detection_locations, cortical_area):
    """ Converts LIDAR detection locations (x, y, z) to neuron IDs in
    the corresponding cortical area.

    :param detection_locations:
    :param cortical_area:
    :return:
    """
    neuron_ids = []
    for i in range(len(detection_locations)):
        block_ref = coords_to_block_ref(detection_locations[i], cortical_area)
        if block_ref in runtime_data.block_dic[cortical_area]:
            block_neurons = runtime_data.block_dic[cortical_area][block_ref]
            for neuron in block_neurons:
                if neuron is not None and neuron not in neuron_ids:
                    neuron_ids.append(neuron)
    return neuron_ids


def coords_to_block_ref(location, cortical_area):
    """ Finds neuron closest to provided location and returns neuron's
    block reference.

    :param location:
    :param cortical_area:
    :return:
    """
    # could instead sort neurons by soma location
    # then perform binary search
    brain = runtime_data.brain
    closest_neuron = None
    min_distance = inf
    for neuron in brain[cortical_area]:
        soma_loc = brain[cortical_area][neuron]['soma_location'][0]
        soma_diff = distance_3d(soma_loc, location)
        if soma_diff < min_distance:
            closest_neuron = neuron
            min_distance = soma_diff
    try:
        closest_block_ref = block_reference_builder(
            brain[cortical_area][closest_neuron]['soma_location'][1]
        )
        return closest_block_ref
    except KeyError:
        return None


def distance_3d(p1, p2):
    """ Calculates distance between two points ([x, y, z]) in 3D space.

    :param p1:
    :param p2:
    :return:
    """
    return sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 + (p2[2]-p1[2])**2)
