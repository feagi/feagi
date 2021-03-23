"""
Copyright (c) 2019 Mohammad Nadji-Tehrani <m.nadji.tehrani@gmail.com>

This module manages all IPU related modules

todo: figure how the exposure counter can work in synchrony with the burst engine
todo: convert IPU library to a plug-in based architecture
"""
import time
import traceback
from queue import Queue
from threading import Thread
from inf import runtime_data
from ipu.source import folder_monitor
from ipu.source.mnist import MNIST, print_mnist_img_raw
from ipu.processor.image import Image
from ipu.processor.proximity import detections_to_coords, coords_to_neuron_ids
from evo.neuroembryogenesis import cortical_sub_group_members


def initialize():
    """
    This function will monitor the IPU folder and other possible input devices such as a camera or mic for data and
    pass them along to the IPU module to have them converted to neuronal activity that in turn can be passed to cortical
    areas via FCL injection.
    """
    print("\n\n\n\n\n**** *** **  Initializing the IPU Controller  **** * * **** ** ** * * *** ** *** *\n\n\n\n ")
    # todo: figure it its best to enable devices using the following if statements or using class instantiation within
    #           ipu_controller function
    # Initialize IPU devices
    if runtime_data.parameters['IPU']['folder_monitor']:
        folder_monitor.initialize()

    # IPU feeder thread processes input stimuli and pass it along to the corresponding IPU module.
    if runtime_data.parameters['IPU']['mnist']:
        ipu_controller_thread = Thread(
            target=ipu_controller, 
            args=(
                runtime_data.watchdog_queue, 
                runtime_data.fcl_queue, 
            ), 
            name="IPU_Controller", 
            daemon=True
        )
        ipu_controller_thread.start()
        print(">> >> IPU Controller thread has started.")

    if runtime_data.parameters['IPU']['proximity']:
        proximity_controller_thread = Thread(
            target=proximity_controller,
            args=(
                runtime_data.proximity_queue,
                runtime_data.fcl_queue,
            ),
            name="Proximity_Controller",
            daemon=True
        )
        proximity_controller_thread.start()
        print(">> >> Proximity Controller thread has started.")


def mnist_load_queue(target_queue):
    # todo: The following is experimental and needs to be rebuilt
    mnist = MNIST()
    mnist_img = mnist.mnist_array['training'][5][1]
    print_mnist_img_raw(mnist_img)
    # Building the list of visual corticothalamic layers associated with vision
    visual_cortical_layers = cortical_sub_group_members('t_vision')
    fcl_entry = {}
    for cortical_layer in visual_cortical_layers:
        neuron_list = Image.convert_image_locations_to_neuron_ids(image_locations=mnist_img, 
                                                                  cortical_area=cortical_layer)
        fcl_entry[cortical_layer] = neuron_list
    target_queue.put(fcl_entry)
    # todo: exiting immediately for test purposes
    runtime_data.exit_condition = True
    runtime_data.parameters["Switches"]["ready_to_exit_burst"] = True


def proximity_load_queue(source_queue, target_queue):
    """ Gets data from source queue, modifies it and places
    it in the target queue.
    """
    proximity_data = source_queue.get()
    coordinates = detections_to_coords(proximity_data)
    
    prox_cortical_area = cortical_sub_group_members('IPU_proximity')[0]
    neuron_list = coords_to_neuron_ids(coordinates, prox_cortical_area)

    target_queue.put(neuron_list)


# todo: most likely this function needs to run on its own thread and not block other operations...maybe!
def ipu_controller(watchdoq_queue, fcl_queue):
    print("<> <> <> <> <> <> <> <> <>        <> <> <> <> <> <>      <> <> <> <> <> <> <>")
    while not runtime_data.exit_condition:
        try:
            mnist_load_queue(fcl_queue)
        except Exception as e:
            traceback.print_exc()
    time.sleep(2)


def proximity_controller(proximity_queue, fcl_queue):
    """ 

    """
    while not runtime_data.exit_condition:
        try:
            proximity_load_queue(proximity_queue, fcl_queue)
        except Exception as e:
            traceback.print_exc()
