"""
Copyright (c) 2019 Mohammad Nadji-Tehrani <m.nadji.tehrani@gmail.com>

This module manages all IPU related modules

todo: figure how the exposure counter can work in synchrony with the burst engine
todo: convert IPU library to a plug-in based architecture
"""
import time
from queue import Queue
from threading import Thread
from inf import runtime_data
from ipu.device import folder_monitor
from ipu.device.mnist import MNIST
from ipu.processor import visual
from evo.neuroembryogenesis import cortical_sub_group_members


def initialize():
    """
    This function will monitor the IPU folder and other possible input devices such as a camera or mic for data and
    pass them along to the IPU module to have them converted to neuronal activity that in turn can be passed to cortical
    areas via FCL injection.
    """

    fcl_queue = Queue()

    # todo: figure it its best to enable devices using the following if statements or using class instantiation within
    #           ipu_controller function
    # Initialize IPU devices
    if runtime_data.parameters['IPU']['folder_monitor']:
        folder_monitor.initialize()
    # if runtime_data.parameters['IPU']['MNIST']:
    #     mnist.
    # if runtime_data.parameters['IPU']['camera']:
    #     mnist.
    # if runtime_data.parameters['IPU']['microphone']:
    #     mnist.
    # if runtime_data.parameters['IPU']['keyboard']:
    #     mnist.
    # if runtime_data.parameters['IPU']['mouse']:
    #     mnist.

    # IPU feeder thread processes input stimuli and pass it along to the corresponding IPU module.
    ipu_controller_thread = Thread(target=ipu_controller, args=(runtime_data.watchdog_queue,),
                                   name="IPU_feeder", daemon=True)
    ipu_controller_thread.start()


# todo: most likely this function needs to run on its own thread and not block other operations...maybe!
def ipu_controller():
    mnist = MNIST()
    while not runtime_data.exit_condition:
        # todo: The following is experimental and needs to be rebuilt
        mnist_value = mnist.mnist_array['training'][5]
        # Building the list of visual corticothalamic layers associated with vision
        visual_cortical_layers = cortical_sub_group_members('t_vision')
        for cortical_layer in visual_cortical_layers:
            visual.Image.convert_image_locations_to_neuron_ids(image_locations=mnist_value,
                                                               cortical_area=cortical_layer)
        time.sleep(2)
