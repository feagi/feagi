from math import sin, cos, sqrt, inf
from evo.neuron import block_reference_builder
# from evo.stats import cortical_xyz_range
from inf import runtime_data


# TODO: make detection threshold part of config
def detections_to_coords(proximity_data, threshold=5):
    """ Converts turtlebot3 (burger) LIDAR data to
    coordinates in the proximity cortical area.

    :param proximity_data:
    :param threshold:
    :return:
    """
    # turtlebot3 specs/documentation
    DETECT_MIN = 0.12
    DETECT_MAX = 3.5

    # need a way to find cortical area z-block range
    # brain_xyz_max = cortical_xyz_range()
    # proximity_z_max = brain_xyz_max['proximity'][-1]

    detection_locations = []
    for i, distance in enumerate(proximity_data):
        # if distance not in range(proximity_z_max + 1):
            # distance = map_value(distance, 0, proximity_z_max, 0, 100)
        distance_map = map_value(distance, DETECT_MIN, DETECT_MAX, 0, 20)
        if distance_map < threshold:
            x = i
            y = 90
            z = distance_map
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
    range [min1, max1] to a value in range [min2, max2].

    :param val:
    :param min1:
    :param max1:
    :param min2:
    :param max2:
    :return:
    """
    return (val-min1) * ((max2-min2) / (max1-min1)) + min2


def distance_3d(p1, p2):
    """ Calculates distance between two points ([x, y, z]) in 3D space.

    :param p1:
    :param p2:
    :return:
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
