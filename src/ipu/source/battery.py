"""
This module will provide the methods to receive information about embodiment battery level and have it passed along to
the artificial brain.

Battery level is broken down into 10% increments and be represented in the form of a range in a single cortical block
with the dimensions of 1x1x10 where x represents the battery index, y is unused, and z reflects the range. In the event
that the embodiment consists of multiple battery backs the x axis will be used to capture it e.g. 4x2x5 for the case of
four battery packs.
"""

from ipu.processor import range
from inf import runtime_data


def translate(sensor_data):
    """
    Translates battery related data to neuronal activity

    todo: place the battery data format here

    sensor_data = {



    }

    """

    print("Translating Battery data...")

    cortical_area = 'battery_ipu'
    if sensor_data is not None:
        for sensor in sensor_data:
            print("----------------->>>> Battery data:", sensor_data[sensor])
            detections = range.range_to_coords(cortical_area=cortical_area,
                                               range_data=int(float(sensor_data[sensor])*100),
                                               range_min=0,
                                               range_max=100,
                                               threshold=10)

            neurons = range.coords_to_neuron_ids(detections,
                                                 cortical_area=cortical_area)
            # TODO: Add proximity feeder function in fcl_injector
            runtime_data.fcl_queue.put({'battery_ipu': set(neurons)})
