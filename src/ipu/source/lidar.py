"""
This module reads LIDAR data from a message queue and makes them available to the proximity processor.

# todo: have this module read settings from feagi_configuration.ini and automatically subscribe to Lidar channel(s)

"""
import time

from ipu.processor import proximity
from inf import runtime_data


def translate(proximity_data, type=None):
    """
    Translate the lidar messages based on its type.

    todo: add details here about the message format and expectations


    Type is not needed at this point given the lidar vs sonar data is automatically differentiated within the func.
    """

    if proximity_data is not None:
        # print("SLOT_TYPES", message.SLOT_TYPES)
        # print("angle_increment:", message.angle_increment)
        # print("angle_max:", message.angle_max)
        # print("angle_min:", message.angle_min)
        # print("get_fields_and_field_types:", message.get_fields_and_field_types)
        # print("header:", message.header)
        # print("intensities:", message.intensities)
        # print("range_max:", message.range_max)
        # print("range_min:", message.range_min)
        # print("ranges:", message.ranges)
        # print("scan_time:", message.scan_time)
        # print("time_increment:", message.time_increment)
        # print("-----")

        for sensor in proximity_data:
            # differentiate between LIDAR/SONAR data
            if hasattr(proximity_data[sensor], '__iter__'):
                detections = proximity.lidar_to_coords(proximity_data[sensor])
                print("** ^^^ **")
            else:
                detections = proximity.sonar_to_coords(proximity_data[sensor])

            neurons = proximity.coords_to_neuron_ids(
                detections, cortical_area='proximity_ipu'
            )
            runtime_data.fcl_queue.put({'proximity_ipu': set(neurons)})
