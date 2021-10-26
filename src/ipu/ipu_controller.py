"""
Copyright (c) 2019 Mohammad Nadji-Tehrani <m.nadji.tehrani@gmail.com>

This module manages all IPU related modules

todo: figure how the exposure counter can work in synchrony with the burst engine
todo: convert IPU library to a plug-in based architecture
"""
import time
import traceback
from datetime import datetime
from threading import Thread
from inf import runtime_data
from ipu.source import lidar, ir, battery, folder_monitor
from ipu.source.mnist import MNIST, print_mnist_img_raw
from ipu.processor.image import Image
from evo.neuroembryogenesis import cortical_sub_group_members


def initialize():
    """
    This function will monitor the IPU folder and other possible input devices such as a camera or mic for data and
    pass them along to the IPU module to have them converted to neuronal activity that in turn can be passed to cortical
    areas via FCL injection.
    """
    print("\n\n\n\n\n**** *** **  Initializing the IPU Controller  **** * * **** ** ** * * *** ** *** *\n\n\n\n ")
    # todo: figure it its best to enable devices using the following if statements or using class instantiation within...
    # ...ipu_controller function
    # Initialize IPU devices
    if runtime_data.parameters['IPU']['folder_monitor']:
        folder_monitor.initialize()

    # IPU feeder thread processes input stimuli and pass it along to the corresponding IPU module.
    if runtime_data.parameters['IPU']['mnist']:
        mnist_controller_thread = Thread(
            target=mnist_controller, 
            args=(
                runtime_data.watchdog_queue, 
                runtime_data.fcl_queue, 
            ), 
            name="MNIST_Controller", 
            daemon=True
        )
        mnist_controller_thread.start()
        print(">> >> MNIST Controller thread has started.")

    # if runtime_data.parameters['IPU']['proximity']:
    #     proximity_controller_thread = Thread(
    #         target=proximity_controller,
    #         name="Proximity_Controller",
    #         daemon=True
    #     )
    #     proximity_controller_thread.start()
    #     print(">> >> Proximity Controller thread has started.")

    # if runtime_data.parameters['IPU']['ir']:
    #
    #     def ir_controller():
    #         while not runtime_data.exit_condition:
    #             try:
    #                 ir.convert_ir_to_fire_list(ir_data=ir_data)
    #                 time.sleep(0.1)
    #             except Exception as e:
    #                 traceback.print_exc()
    #             finally:
    #                 runtime_data.last_ipu_activity = datetime.now()
    #
    #     ir_controller_thread = Thread(
    #         target=ir_controller,
    #         name="IR_Controller",
    #         daemon=True
    #     )
    #     ir_controller_thread.start()
    #     print(">> >> IR Controller thread has started.")

    runtime_data.last_ipu_activity = datetime.now()


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


def mnist_controller(watchdoq_queue, fcl_queue):
    print("<> <> <> <> <> <> <> <> <>        <> <> <> <> <> <>      <> <> <> <> <> <> <>")
    while not runtime_data.exit_condition:
        try:
            mnist_load_queue(fcl_queue)
        except Exception as e:
            traceback.print_exc()
        finally:
            runtime_data.last_ipu_activity = datetime.now()
    time.sleep(2)


def proximity_controller():
    while not runtime_data.exit_condition:
        try:
            lidar.get_and_translate()
        except Exception as e:
            traceback.print_exc()
        finally:
            runtime_data.last_ipu_activity = datetime.now()


def ipu_handler(ipu_data):
        """
        Decodes the message received from the ipu router and distribute the sub-messages to corresponding IPU modules

        expected ipu_data structure:

        ipu_data = {
            sensor_type: {
                sensor_name: sensor_data,
                sensor_name: sensor_data,
                ...
                },
            sensor_type: {
                sensor_name: sensor_data,
                sensor_name: sensor_data,
                ...
                },
            ...
            }
        }
        """
        if type(ipu_data) == dict:
            for sensor_type in ipu_data:
                # Ultrasonic / Lidar Handler
                # todo: need a more consistent naming convention when it comes to lidar vs ultrasonic vs proximity
                # todo: find a way to generalize the handling of all IPU data instead of using all the if statements
                if 'ultrasonic' in sensor_type and runtime_data.parameters['IPU']['proximity']:
                    try:
                        lidar.translate(proximity_data=ipu_data[sensor_type])

                    except:
                        print("ERROR while processing lidar function")

                # Infrared Handler
                elif 'ir' in sensor_type and runtime_data.parameters['IPU']['ir']:
                    try:
                        # print("+_+_+ipu_data[sensor_type]: ", ipu_data[sensor_type])
                        ir.convert_ir_to_fire_list(ir_data=ipu_data[sensor_type])
                    except:
                        print("ERROR while processing Infrared IPU")

                elif 'battery' in sensor_type and runtime_data.parameters['IPU']['battery']:
                    try:
                        battery.translate(sensor_data=ipu_data[sensor_type])
                    except:
                        print("ERROR while processing Battery IPU")

        else:
            print("ERROR: IPU handler encountered non-compliant data")
