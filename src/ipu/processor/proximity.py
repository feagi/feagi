from math import sqrt, inf
from evo.neuron import block_reference_builder
from inf import runtime_data


# TODO: make detection threshold part of config
def detections_to_coords(proximity_data, threshold=5):
    """ Converts turtlebot3 LIDAR data to coordinates in 
    the proximity cortical area.

    :param proximity_data: float array of detection distances
    :param threshold: int threshold for detection distance
    :return: list of tuple detection locations (x, y, z)
    """
    # turtlebot3 specs/documentation
    DETECT_MIN = 0.12
    DETECT_MAX = 3.5

    Z_MAX = runtime_data.genome['blueprint'] \
                               ['proximity'] \
                               ['neuron_params'] \
                               ['block_boundaries'][-1]

    detection_locations = []
    for idx, dist in enumerate(proximity_data):
        if dist != inf:
            dist_map = map_value(dist, DETECT_MIN, DETECT_MAX, 0, Z_MAX)
            if dist_map <= threshold:
                x = idx
                y = 90
                z = dist_map
                detection_locations.append((x, y, int(z)))
    return detection_locations


def coords_to_neuron_ids(detection_locations, cortical_area):
    """ Converts LIDAR detection locations to neuron IDs in
    the corresponding cortical area block.

    :param detection_locations: list of tuple (x, y, z) detections
    :param cortical_area: name of cortical area (str)
    :return: list of neuron IDs (str)
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

    :param location: iterable containing x, y, z coordinate values
    :param cortical_area: name of cortical area (str)
    :return: block reference (str) of block closest to location
    """
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


def map_value(val, min1, max1, min2, max2):
    """ Performs linear transformation to map value from
    range 1 [min1, max1] to a value in range 2 [min2, max2].

    :param val: value (int/float) being mapped
    :param min1: min (int/float) of range 1
    :param max1: max (int/float) of range 1
    :param min2: min (int/float) of range 2
    :param max2: max (int/float) of range 2
    :return: value (int/float) mapped from range 1 to range 2 
    """
    return (val-min1) * ((max2-min2) / (max1-min1)) + min2


def distance_3d(p1, p2):
    """ Calculates distance between two points in 3D space.

    :param p1: iterable cointaining point1 x, y, z values
    :param p2: iterable cointaining point2 x, y, z values
    :return: distance between 2 points (float)
    """
    return sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 + (p2[2]-p1[2])**2)


# def detections_to_coords(proximity_data, proximity_type='LIDAR'):
#     """ Converts coordinates from LIDAR detections to modified 
#     Cartesian plane.

#     :param proximity_data:
#     :return:
#     """
#     detection_locations = []
#     for sweep in proximity_data:
#         for point in proximity_data[sweep][proximity_type]:
#             h_angle = point[0]
#             v_angle = point[1]
#             distance = point[2]

#             x = distance * sin(v_angle) * cos(h_angle)
#             y = distance * sin(v_angle) * sin(h_angle)
#             z = distance * cos(v_angle)

#             detection_locations.append((x, y, z))
    
#     return detection_locations
