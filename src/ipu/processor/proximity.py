from math import sin, cos
from evo.neuron import neuron_finder
from inf import runtime_data


def detections_to_coords(proximity_data, proximity_type):
    """ Converts coordinates from various types of proximity detections 
    (LIDAR, SONAR, etc.) to modified Cartesian plane.

    :param proximity_data:
    :param proximity_type:
    :return:
    """
    detection_locations = []
    # can become expensive (O(n^2)) if high-res LIDAR (?)
    # convert to list comprehension for modest gains? <-- could be ugly
    # cythonize this?
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
    """ Converts proximity detection locations (x, y, z) to neuron IDs in
    the cortical area.

    :param detection_locations:
    :param cortical_area:
    :return:
    """
    neuron_ids = []
    for i in range(len(point_locations)):
        found_neurons = neuron_finder(
            cortical_area, 
            point_locations[i], 
            runtime_data.genome["location_tolerance"]
        )    
        if found_neurons:
            for neuron in found_neurons:
                neuron_ids.append(neuron)
    
    return neuron_ids
