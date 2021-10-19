"""
This module processes LIDAR data from a message queue and converts it to neuronal activity
within the proximity cortical area.
"""
from math import inf
from evo.neuron import block_reference_builder
from evo.blocks import neurons_in_the_block
from inf import runtime_data

# TODO: Generalize this module to process any data with a linear range e.g. lidar, battery, sonar, temperature, etc.


def range_to_coords(cortical_area, range_data, range_min, range_max, threshold):
    """
    This multi purpose function converts data in a linear range format to coordinates in target cortical area.

    :param range_data: float array of detection distances
    :param threshold: threshold for detection distance (int)
    :return: list of tuple detection locations (x, y, z)
    """
    y_max = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries'][1]
    z_max = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries'][2]

    detection_locations = list
    for idx, dist in enumerate(range_data):
        if dist != inf:
            dist_map = map_value(dist, range_min, range_max, 0, z_max)
            if dist_map <= threshold:
                x = idx
                y = y_max // 2
                z = dist_map
                detection_locations.append((x, y, int(z)))
    return detection_locations


# TODO: make detection threshold part of config
def lidar_to_coords(lidar_data, threshold=5):
    """ Converts turtlebot LIDAR data to coordinates in 
    the proximity cortical area.

    :param lidar_data: float array of detection distances
    :param threshold: threshold for detection distance (int)
    :return: list of tuple detection locations (x, y, z)
    """
    # turtlebot3 specs/documentation (in meters)
    LIDAR_MIN = 0.12
    LIDAR_MAX = 3.5

    Y_MAX = runtime_data.genome['blueprint'] \
                               ['proximity_ipu'] \
                               ['neuron_params'] \
                               ['block_boundaries'][1]
    
    Z_MAX = runtime_data.genome['blueprint'] \
                               ['proximity_ipu'] \
                               ['neuron_params'] \
                               ['block_boundaries'][2]

    detection_locations = []
    for idx, dist in enumerate(lidar_data):
        if dist != inf:
            dist_map = map_value(dist, LIDAR_MIN, LIDAR_MAX, 0, Z_MAX)
            if dist_map <= threshold:
                x = idx
                y = Y_MAX // 2
                z = dist_map
                detection_locations.append((x, y, int(z)))
    return detection_locations


def sonar_to_coords(sonar_data, threshold=10):
    """ Converts SONAR data from sensor to coordinates in 
    the proximity cortical area.

    :param sonar_data: detection distance (int)
    :param threshold: threshold for detection distance (int)
    :return: list containing single tuple detection location (x, y, z)
    """
    # HC-SR04 datasheet specs (in cm)
    # TODO: parameterize min max vals for various sensors/scenarios
    # updated these min/max values to reflect those in gazebo simulation
    SONAR_MIN = 0
    SONAR_MAX = 4

    X_MAX = runtime_data.genome['blueprint'] \
                               ['proximity_ipu'] \
                               ['neuron_params'] \
                               ['block_boundaries'][0]

    Y_MAX = runtime_data.genome['blueprint'] \
                               ['proximity_ipu'] \
                               ['neuron_params'] \
                               ['block_boundaries'][1]

    Z_MAX = runtime_data.genome['blueprint'] \
                               ['proximity_ipu'] \
                               ['neuron_params'] \
                               ['block_boundaries'][2]

    try:
        dist_map = round(map_value(sonar_data, SONAR_MIN, SONAR_MAX, 0, Z_MAX))
    except TypeError:
        dist_map = 0
        print("Type Error in sonar_to_coords...")
    
    if dist_map != 0:
        x = X_MAX // 2
        y = Y_MAX // 2
        z = dist_map
        return [(x, y, z)]
    

def coords_to_neuron_ids(detection_locations, cortical_area):
    """ Converts proximity detection locations to neuron IDs in
    the corresponding cortical area block.

    :param detection_locations: list of tuple (x, y, z) detections
    :param cortical_area: name of cortical area (str)
    :return: list of neuron IDs (str)
    """
    neuron_ids = []
    if detection_locations is not None:
        for i in range(len(detection_locations)):
            block_ref = block_reference_builder(detection_locations[i])
            block_neurons = neurons_in_the_block(cortical_area, block_ref)
            for neuron in block_neurons:
                if neuron is not None and neuron not in neuron_ids:
                    neuron_ids.append(neuron)
    return neuron_ids


# todo: Move map value function out of proximity and to a more generic location
def map_value(val, min1, max1, min2, max2):
    """ Performs linear transformation to map value from
    range 1 [min1, max1] to a value in range 2 [min2, max2].

    :param val: value (int/float) being mapped
    :param min1: min of range 1
    :param max1: max of range 1
    :param min2: min of range 2
    :param max2: max of range 2
    :return: value mapped from range 1 to range 2 
    """
    if val < min1:
        return min2
    if val > max1:
        return max2

    mapped_value = (val - min1) * ((max2 - min2) / (max1 - min1)) + min2

    if max2 >= mapped_value >= min2:
        return mapped_value
