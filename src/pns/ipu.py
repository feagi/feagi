
# Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

import os
import time
import struct
import numpy as np
from inf.disk_ops import save_processed_mnist_to_disk
from datetime import datetime
from evo.blocks import neurons_in_the_block, block_reference_builder, block_ref_2_id
from queue import Queue
from inf import runtime_data, settings
import time
import traceback
from datetime import datetime
from threading import Thread
from inf import runtime_data
# from ipu.source import lidar, ir, battery, folder_monitor, stimulation
# from ipu.source.mnist import MNIST, print_mnist_img_raw
# from ipu.processor.image import Image
from evo.neuroembryogenesis import cortical_sub_group_members


class IPU:
    def __init__(self):
        print("IPU class initialized...")

    # class Injector:
    #     """
    #     This module plays the role of the information gateway to the cortical neurons. After physical stimuli is processed by
    #     the Input Processing Unit (IPU), it will be translated to neuronal activation and fed to Fire Candidate List (FCL).
    #
    #     Expectations:
    #     - To receive the list of activated IPU neurons from the corresponding IPU module and inject them into FCL
    #     - To monitor flow of information and act as needed
    #
    #     Q) Why the need for seperate module? Can this be merged with the burst-engine? or, each IPU can directly inject to FCL!
    #     """
    #
    #     def __init__(self):
    #         self.something = 'something'

        # def utf8_feeder(self):
        #     # inject label to FCL
        #     runtime_data.training_neuron_list_utf = set()
        #
        #     if self.injector_injection_mode == 'c':
        #         runtime_data.training_neuron_list_utf = \
        #             convert_char_to_fire_list(self.injector_utf_to_inject)
        #     else:
        #         runtime_data.training_neuron_list_utf = \
        #             convert_char_to_fire_list(str(runtime_data.labeled_image[1]))
        #         print("!!! Image label: ", runtime_data.labeled_image[1])
        #
        #     runtime_data.fire_candidate_list['utf8_ipu'].update(runtime_data.training_neuron_list_utf)

        # @staticmethod
        # def img_neuron_list_feeder():
        #     # inject neuron activity to FCL
        #     if runtime_data.training_neuron_list_img:
        #         for cortical_area in runtime_data.v1_members:
        #             if runtime_data.training_neuron_list_img[cortical_area]:
        #                 # print("Before FCL injection:", candidate_list_counter(runtime_data.fire_candidate_list),
        #                 # len(runtime_data.training_neuron_list_img[cortical_area]))
        #                 runtime_data.fire_candidate_list[cortical_area]. \
        #                     update(runtime_data.training_neuron_list_img[cortical_area])
        #                 # print("After FCL injection:", candidate_list_counter(runtime_data.fire_candidate_list))0

        # @staticmethod
        # def mnist_feeder(num, seq, mnist_type):
        #     runtime_data.labeled_image = ['', num]
        #
        #     # todo: define a function to return mnist image (should already have one)
        #     image = {}
        #
        #     runtime_data.training_neuron_list_img = retina(image=image, polarize=True)

    class Controller:
        """

        This module manages all IPU related modules

        todo: figure how the exposure counter can work in synchrony with the burst engine
        todo: convert IPU library to a plug-in based architecture
        """

        def __init__(self):
            print("IPU controller initialized")
            runtime_data.last_ipu_activity = datetime.now()

        def proximity_controller(self):
            while not runtime_data.exit_condition:
                try:
                    self.source.Lidar.get_and_translate()
                except Exception as e:
                    traceback.print_exc()
                finally:
                    runtime_data.last_ipu_activity = datetime.now()

        def ipu_handler(self, ipu_data):
            """
            Decodes the message received from the ipu router and distribute the sub-messages to corresponding IPU modules

            expected ipu_data structure:

            ipu_data = {
                "capabilities": {},
                "network": {},
                "data": {
                    "direct_stimulation": {
                        "cortical_area_id": {voxel},
                        "cortical_area_id": sensor_data,
                        "cortical_area_id": sensor_data
                        ...
                        },
                    "sensory_data": {
                        "sensor type": sensor data,
                        "sensor type": sensor data,
                        "sensor type": sensor data,
                        ...
                    }
                }
            """

            if type(ipu_data) == dict:
                if "direct_stimulation" in ipu_data["data"]:
                    if ipu_data["data"]["direct_stimulation"] is not None:
                        try:
                            print(">>> >> >> > > >> >>>>>>> Stimulation data is being processed....")
                            self.Source.Stimulation.\
                                stimulation_injector(stimulation_data=ipu_data["data"]["direct_stimulation"])
                            print(">>> >> >> > > >> >>>>>>> Stimulation data was processed....")
                        except:
                            print("ERROR while processing Stimulation IPU", ipu_data["data"]["direct_stimulation"])

                if "sensory_data" in ipu_data["data"]:
                    for sensor_type in ipu_data["data"]["sensory_data"]:
                        # Ultrasonic / Lidar Handler
                        # todo: need a more consistent naming convention when it comes to lidar vs ultrasonic vs proximity
                        # todo: find a way to generalize the handling of all IPU data instead of using all the if statements

                        if 'ultrasonic' in sensor_type and \
                                ipu_data["data"]["sensory_data"][sensor_type] is not None:
                            try:
                                self.Source.Lidar.translate(proximity_data=
                                                            ipu_data["data"]["sensory_data"][sensor_type])
                            except:
                                print("ERROR while processing lidar function")

                        # Infrared Handler
                        if 'ir' in sensor_type and ipu_data["data"]["sensory_data"][sensor_type] is not None:
                            try:
                                print("+_+_+ipu_data[sensor_type]: ", ipu_data[sensor_type])
                                self.Source.Infrared.convert_ir_to_fire_list(ir_data=ipu_data[
                                    "data"]["sensory_data"][sensor_type])
                            except:
                                print("ERROR while processing Infrared IPU")

                        if 'battery' in sensor_type and ipu_data["data"]["sensory_data"][sensor_type] is not None:
                            try:
                                self.Source.Battery.translate(sensor_data=ipu_data["data"]["sensory_data"]
                                [sensor_type])
                            except:
                                print("ERROR while processing Battery IPU")

                else:
                    print("ERROR: IPU handler encountered non-compliant data")

        class Source:
            class Stimulation:
                """
                This module facilitates the translation of cortical stimulation information to actual stimulation within connectome.

                Stimulation data received will have the following data structure:
                { 'stimulation': {
                        'motor_opu': [[x1, y1, z1], [x2, y2, z2], .....],
                        'IR_opu': [[x1, y1, z1], [x2, y2, z2], .....],
                        ...
                        ...
                    }
                }

                """
                @staticmethod
                def fake_cortical_stimulation(input_instruction, burst_count):
                    """
                    It fakes cortical stimulation for the purpose of testing

                    The following data format is used for input_instruction as the function input:

                    input_instructions receives a dictionary as input with keys as the name of the ipu cortical name and the value
                    being a list of block locations that needs to be activated in the block-ref format e.g. xBlock-yBlock-zBlock.

                    Note: all of the blocks outlined in the data structure will be activated at the same time during the same
                    burst.

                    input_instruction_example = {
                        ir_ipu: ["0-0-0", "1-0-0"],
                        proximity_ipu: ["0-0-0", "0-0-3", "0-0-10", "0-0-20"]
                        led_opu: ["5-0-0"]
                    }

                    # todo: Currently we can only inject data from the first index on each burst. change it so it goes thru all
                    """
                    neuron_list = []

                    for cortical_area_ in input_instruction[burst_count]:
                        if cortical_area_ in runtime_data.block_dic:
                            for block_ref in input_instruction[burst_count][cortical_area_]:
                                if block_ref in runtime_data.block_dic[cortical_area_]:
                                    for neuron in runtime_data.block_dic[cortical_area_][block_ref]:
                                        neuron_list.append(neuron)
                                else:
                                    print("Warning: Block ref %s was not found for %s" % (block_ref, cortical_area_))
                            # print("neuron list:", cortical_area_, neuron_list)
                            runtime_data.fcl_queue.put({cortical_area_: set(neuron_list)})
                            neuron_list = []
                        else:
                            print("Warning: Cortical area %s not found within the block_dic" % cortical_area_)

                @staticmethod
                def stimulation_injector(stimulation_data):
                    for cortical_area in stimulation_data:
                        print("stimulating...", cortical_area)
                        neuron_list = set()
                        for voxel in stimulation_data[cortical_area]:
                            print("FEAGI received direct stimulation; processing...", voxel)
                            if type(voxel) is list:
                                voxel = block_reference_builder(voxel)
                            in_the_block = neurons_in_the_block(cortical_area=cortical_area, block_ref=voxel)
                            for neuron in in_the_block:
                                neuron_list.add(neuron)
                        runtime_data.fcl_queue.put({cortical_area: neuron_list})
                        print(">>> >> >> > > >> >>>>>>>  Stimulation has been injected in FCL!")


                # @staticmethod
                # def godot_injector(stimulation_data):
                #     for cortical_area in stimulation_data:
                #         print("stimulating...", cortical_area)
                #         neuron_list = set()
                #         for voxel in stimulation_data[cortical_area]:
                #             voxel = block_ref_2_id(voxel)
                #             relative_coords = \
                #                 runtime_data.genome['blueprint'][cortical_area]['neuron_params'].get('relative_coordinate')
                #             cortical_block_ref = [voxel[0] - relative_coords[0],
                #                                   voxel[1] - relative_coords[1],
                #                                   voxel[2] - relative_coords[2]]
                #             print("FEAGI received stimulation from Godot and processing...", cortical_block_ref)
                #             in_the_block = neurons_in_the_block(cortical_area=cortical_area,
                #                                                 block_ref=block_reference_builder(cortical_block_ref))
                #             for neuron in in_the_block:
                #                 neuron_list.add(neuron)
                #         runtime_data.fcl_queue.put({cortical_area: neuron_list})
                #         print(">>> >> >> > > >> >>>>>>> Stimulation data from Godot has been injected in FCL!")

            class Battery:
                """
                This module will provide the methods to receive information about embodiment battery level and have it passed along to
                the artificial brain.

                Battery level is broken down into 10% increments and be represented in the form of a range in a single cortical block
                with the dimensions of 1x1x10 where x represents the battery index, y is unused, and z reflects the range. In the event
                that the embodiment consists of multiple battery backs the x axis will be used to capture it e.g. 4x2x5 for the case of
                four battery packs.
                """

                @staticmethod
                def translate(sensor_data):
                    """
                    Translates battery related data to neuronal activity

                    todo: place the battery data format here

                    sensor_data = {



                    }

                    """

                    print("Translating Battery data...")

                    cortical_area = 'i__bat'
                    if sensor_data is not None:
                        for sensor in sensor_data:
                            print("----------------->>>> Battery data:", sensor_data[sensor])
                            detections = range.range_to_coords(cortical_area=cortical_area,
                                                               range_data=int(float(sensor_data[sensor]) * 100),
                                                               range_min=0,
                                                               range_max=100,
                                                               threshold=10)

                            neurons = range.coords_to_neuron_ids(detections,
                                                                 cortical_area=cortical_area)
                            # TODO: Add proximity feeder function in fcl_injector
                            runtime_data.fcl_queue.put({'battery_ipu': set(neurons)})

            class Infrared:
                @staticmethod
                def convert_ir_to_fire_list(ir_data):
                    """

                    The keys in ir_data correlate to the index id of each Infrared Sensor

                    ir_data = {
                        0: True,
                        1: True,
                        2: False
                    }
                    """
                    fire_list = set()
                    for sensor_idx in ir_data:
                        if ir_data[sensor_idx]:
                            for key in runtime_data.brain['i__inf']:
                                if sensor_idx == runtime_data.brain['i__inf'][key]['soma_location'][0][0]:
                                    fire_list.add(key)

                    runtime_data.fcl_queue.put({'i__inf': fire_list})

            class Lidar:
                @staticmethod
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
                                detections = range.lidar_to_coords(proximity_data[sensor])
                            else:
                                detections = range.sonar_to_coords(proximity_data[sensor])

                            neurons = range.coords_to_neuron_ids(
                                detections, cortical_area='i__pro'
                            )

                            # TODO: Add proximity feeder function in fcl_injector
                            runtime_data.fcl_queue.put({'i__pro': set(neurons)})

            # class FolderMonitor:
            #     """
            #     Source: https://camcairns.github.io/python/2017/09/06/python_watchdog_jobs_queue.html
            #
            #     This class inherits from the Watchdog PatternMatchingEventHandler class. In this code our watchdog will only be
            #     triggered if a file is moved to have a .trigger extension. Once triggered the watchdog places the event object on the
            #     queue, ready to be picked up by the worker thread
            #     """
            #
            #     import string
            #     import time
            #     from queue import Queue
            #     from threading import Thread
            #     from watchdog.observers import Observer
            #     from watchdog.events import PatternMatchingEventHandler
            #     from inf import runtime_data
            #     # from ipu.processor import utf
            #
            #     @staticmethod
            #     def initialize():
            #         runtime_data.watchdog_queue = Queue()
            #         ipu_folder_monitor = Thread(target=folder_mon,
            #                                     args=(runtime_data.working_directory + '/ipu', ['*.png', '*.txt'],
            #                                           runtime_data.watchdog_queue,), name='IPU_folder_monitor',
            #                                     daemon=True)
            #         ipu_folder_monitor.start()
            #         print(">> >> >> Folder monitoring thread has started..")
            #
            #     @staticmethod
            #     def folder_mon(folder_path, pattern, q):
            #         event_handler = FolderWatchdog(queue_=q, patterns=pattern)
            #         observer = Observer()
            #         observer.schedule(event_handler, folder_path, recursive=True)
            #         observer.start()
            #         while not runtime_data.exit_condition:
            #             time.sleep(2)
            #
            #     class FolderWatchdog(PatternMatchingEventHandler):
            #         """ Watches a nominated directory and when a trigger file is
            #             moved to take the ".trigger" extension it proceeds to execute
            #             the commands within that trigger file.
            #         """
            #
            #         def __init__(self, queue_, patterns):
            #             PatternMatchingEventHandler.__init__(self, patterns=patterns)
            #             self.queue = queue_
            #
            #         def process(self, event):
            #             """
            #             event.event_type
            #                 'modified' | 'created' | 'moved' | 'deleted'
            #             event.is_directory
            #                 True | False
            #             event.src_path
            #                 path/to/observed/file
            #             """
            #             self.queue.put(event)
            #
            #         def on_moved(self, event):
            #             print("IPU detected a modification to ", event.src_path)
            #             self.process(event)
            #
            #         def on_created(self, event):
            #             file_path = event.src_path
            #             if str(file_path).endswith('.png'):
            #                 print('IPU detected the presence of a new --PNG-- file @', file_path)
            #                 pass
            #             if str(file_path).endswith('.txt'):
            #                 print('Detected the presence of a new --TXT-- file @', file_path)
            #                 with open(file_path, 'r') as txt_file:
            #                     for _ in txt_file.read():
            #                         print(_)
            #                         runtime_data.fire_candidate_list['utf8_ipu'].update(utf.convert_char_to_fire_list(_))

            # class MNIST:
            #     def __init__(self):
            #         # global mnist_array, mnist_iterator
            #         self.mnist_training_iterator = self.read_mnist_raw(dataset_type="training")
            #         self.mnist_test_iterator = self.read_mnist_raw(dataset_type="testing")
            #         self.kernel = Kernel
            #         self.mnist_array = dict()
            #         self.mnist_array['training'] = []
            #         self.mnist_array['test'] = []
            #         for _ in self.mnist_training_iterator:
            #             self.mnist_array['training'].append(_)
            #         for __ in self.mnist_test_iterator:
            #             self.mnist_array['test'].append(__)
            #         # self.mongo = MongoManagement()
            #         # print(len(mnist_array))
            #
            #     def mnist_plotter(self, mnist_type="training", subplot_dimension=5, desirable_label=6):
            #         counter = 0
            #         x_counter = 0
            #         y_counter = 0
            #         counter_limit = subplot_dimension * subplot_dimension
            #         f, axarr = plt.subplots(subplot_dimension, subplot_dimension)
            #         mnist = self.MNIST()
            #         for entry in mnist.mnist_array[mnist_type]:
            #
            #             label = entry[0]
            #
            #             if label == desirable_label:
            #
            #                 # The rest of columns are pixels
            #                 pixels = entry[1:]
            #
            #                 # Make those columns into a array of 8-bits pixels
            #                 # This array will be of 1D with length 784
            #                 # The pixel intensity values are integers from 0 to 255
            #                 pixels = np.array(pixels, dtype='uint8')
            #
            #                 # Reshape the array into 28 x 28 array (2-dimensional array)
            #                 pixels = pixels.reshape((28, 28))
            #
            #                 # Plot
            #                 axarr[y_counter, x_counter].imshow(pixels)
            #                 # Turn off tick labels
            #                 axarr[y_counter, x_counter].set_yticklabels([])
            #                 axarr[y_counter, x_counter].set_xticklabels([])
            #                 axarr[y_counter, x_counter].axis('off')
            #
            #                 counter += 1
            #                 if counter == counter_limit:
            #                     # plt.tight_layout()
            #                     plt.axis('off')
            #                     # plt.figure(figsize=(1000, 1000))
            #                     plt.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=0.001,
            #                                         hspace=0.001)
            #                     plt.show()
            #                     return
            #
            #                 if x_counter == (subplot_dimension - 1):
            #                     y_counter += 1
            #                     x_counter = 0
            #                 else:
            #                     x_counter += 1
            #
            #     def print_mnist_img(num, seq, mnist_type):
            #         mnist = MNIST()
            #
            #         if runtime_data.parameters['Logs']['print_mnist_img']:
            #             mnist_img = mnist.read_nth_mnist_digit(seq=seq, type=mnist_type, digit=num)
            #             try:
            #                 mnist_img = mnist_img.tolist()
            #                 for row in mnist_img:
            #                     for item in row:
            #                         if item > 50:
            #                             print("O", end='  ')
            #                         else:
            #                             print('   ', end='')
            #                     print('\n')
            #             except AttributeError:
            #                 print("ERROR: Attribute error while printing image.", mnist_img)
            #
            #     def print_mnist_img_raw(raw_img):
            #         for row in raw_img:
            #             for item in row:
            #                 if item > 50:
            #                     print("*", end='  ')
            #                 else:
            #                     print('   ', end='')
            #             print('\n')
            #
            #     def read_mnist_labels(dataset="training"):
            #         """
            #         For importing the MNIST data set.  It returns an iterator
            #         of 2-tuples with the first element being the label and the second element
            #         being a numpy.uint8 2D array of pixel data for the given image.
            #         """
            #         path = runtime_data.parameters['Paths']['mnist_path']
            #
            #         if dataset == "training":
            #             fname_lbl = os.path.join(path, 'train-labels.idx1-ubyte')
            #         elif dataset == "testing":
            #             fname_lbl = os.path.join(path, 't10k-labels.idx1-ubyte')
            #         else:
            #             raise Exception(ValueError, "data set must be 'testing' or 'training'")
            #
            #         # Load everything in some numpy arrays
            #         with open(fname_lbl, 'rb') as flbl:
            #             magic, num = struct.unpack(">II", flbl.read(8))
            #             lbl = np.fromfile(flbl, dtype=np.int8)
            #
            #         get_img_lbl = lambda idx: (lbl[idx])
            #
            #         # Create an iterator which returns each image in turn
            #         for i in range(len(lbl)):
            #             yield get_img_lbl(i)
            #
            #     def mnist_direction_matrix_builder_in_mongodb(self):
            #         kernel = Kernel()
            #         mnist_type_options = ['training', 'test']
            #         # kernel_size_options = [3, 5, 7]
            #         kernel_size_options = [5]
            #
            #         for mnist_type in mnist_type_options:
            #             mnist_instance_seq = 0
            #             for entry in self.mnist_array[mnist_type]:
            #                 mnist_instance_label, mnist_instance_data = entry
            #
            #                 for kernel_size in kernel_size_options:
            #                     direction_matrix_ = (kernel.create_direction_matrix2(image=mnist_instance_data,
            #                                                                          kernel_size=int(kernel_size)))
            #
            #                     mnist_data = {
            #                         "mnist_type": mnist_type,
            #                         "mnist_seq": mnist_instance_seq,
            #                         "kernel_size": kernel_size,
            #                         "digit": str(mnist_instance_label),
            #                         "original_image": mnist_instance_data.tolist()
            #                     }
            #
            #                     for direction in direction_matrix_:
            #                         mnist_data[direction] = direction_matrix_[direction]
            #
            #                     runtime_data.mongodb.insert_mnist_entry(mnist_data=mnist_data)
            #                     print("Added to MongoDb: Type=%s  Seq=%s  Kernel_size=%s  Digit=%s" % (
            #                     mnist_type, mnist_instance_seq, kernel_size, mnist_instance_label))
            #                 mnist_instance_seq += 1
            #
            #     def mnist_direction_matrix_builder(self):
            #         template = {
            #             "3": {},
            #             "5": {},
            #             "7": {}
            #         }
            #         kernel = Kernel()
            #         for key in template:
            #             for number in range(0, 10):
            #                 template[key][str(number)] = []
            #
            #         all_of_mnist_training = template
            #
            #         training_processing_start_time = datetime.now()
            #         counter = 0
            #         for kernel_size in all_of_mnist_training:
            #             for digit in all_of_mnist_training[kernel_size]:
            #                 for entry in self.mnist_array['training']:
            #                     counter += 1
            #                     print("Kernel size:", kernel_size, "Digit:", digit, "Training counter: ", counter)
            #                     # if counter == 100:
            #                     #     counter = 0
            #                     #     break
            #                     mnist_instance_label, mnist_instance_data = entry
            #                     if str(mnist_instance_label) == digit:
            #                         direction_matrix_ = (kernel.create_direction_matrix2(image=mnist_instance_data,
            #                                                                              kernel_size=int(kernel_size)))
            #                         # direction_matrix_["original"] = mnist_instance_data
            #                         all_of_mnist_training[kernel_size][digit].append(direction_matrix_)
            #
            #         save_processed_mnist_to_disk(data_type='training', data=all_of_mnist_training)
            #         print("Processed MNIST Training data has been saved to disk.")
            #         print(">> Processing of MNIST Training data set took: ",
            #               datetime.now() - training_processing_start_time)
            #
            #         test_processing_start_time = datetime.now()
            #         counter = 0
            #         all_of_mnist_test = template
            #
            #         for kernel_size in all_of_mnist_test:
            #             for digit in all_of_mnist_test[kernel_size]:
            #                 for entry in self.mnist_array['test']:
            #                     counter += 1
            #                     print("Kernel size:", kernel_size, "Digit:", digit, "Test counter: ", counter)
            #                     # if counter == 100:
            #                     #     counter = 0
            #                     #     break
            #                     mnist_instance_label, mnist_instance_data = entry
            #                     if str(mnist_instance_label) == digit:
            #                         direction_matrix_ = (kernel.create_direction_matrix2(image=mnist_instance_data,
            #                                                                              kernel_size=int(kernel_size)))
            #                         # direction_matrix_["original"] = mnist_instance_data
            #                         all_of_mnist_test[kernel_size][digit].append(direction_matrix_)
            #
            #         save_processed_mnist_to_disk(data_type='test', data=all_of_mnist_test)
            #         print("Processed MNIST Test data has been saved to disk.")
            #         print(">> Processing of MNIST Test data set took: ", datetime.now() - test_processing_start_time)
            #
            #     @staticmethod
            #     def read_mnist_raw(dataset_type):
            #         """
            #         Python function for importing the MNIST data set.  It returns an iterator
            #         of 2-tuples with the first element being the label and the second element
            #         being a numpy.uint8 2D array of pixel data for the given image.
            #         """
            #
            #         path = runtime_data.parameters["Paths"]["mnist_path"]
            #
            #         if dataset_type == "training":
            #             fname_img = os.path.join(path, 'train-images.idx3-ubyte')
            #             fname_lbl = os.path.join(path, 'train-labels.idx1-ubyte')
            #             # fname_img2 = os.path.join(path2, 'train-images.idx3-ubyte')
            #             # fname_lbl2 = os.path.join(path2, 'train-labels.idx1-ubyte')
            #
            #         elif dataset_type == "testing":
            #             fname_img = os.path.join(path, 't10k-images.idx3-ubyte')
            #             fname_lbl = os.path.join(path, 't10k-labels.idx1-ubyte')
            #             # fname_img2 = os.path.join(path2, 't10k-images.idx3-ubyte')
            #             # fname_lbl2 = os.path.join(path2, 't10k-labels.idx1-ubyte')
            #         else:
            #             raise Exception(ValueError, "data set must be 'testing' or 'training'")
            #
            #         import pathlib
            #         print(">>>>>>  >>> >> >> >> >> >> ", pathlib.Path(__file__).parent.absolute(), fname_lbl, path)
            #
            #         # Load everything in some numpy arrays
            #         with open(fname_lbl, 'rb') as flbl:
            #             magic, num = struct.unpack(">II", flbl.read(8))
            #             lbl = np.fromfile(flbl, dtype=np.int8)
            #
            #         with open(fname_img, 'rb') as fimg:
            #             magic, num, rows, cols = struct.unpack(">IIII", fimg.read(16))
            #             img = np.fromfile(fimg, dtype=np.uint8).reshape(len(lbl), rows, cols)
            #
            #         get_img = lambda idx: (lbl[idx], img[idx])
            #
            #         # Create an iterator which returns each image in turn
            #         for i in range(len(lbl)):
            #             yield get_img(i)
            #
            #     def mnist_img_fetcher_mongo(self, num, kernel_size, seq, mnist_type, random_num=False):
            #         """
            #         Reads a number from pre-processed dataset and returns direction matrix data
            #         """
            #
            #         if random_num:
            #             # todo: Need to create and call a MongoDb function to pull a random number
            #             return
            #         else:
            #             return self.mongo.mnist_read_nth_digit(mnist_type=mnist_type, n=seq, kernel_size=kernel_size,
            #                                                    digit=num)
            #             # return self.mongo.mnist_read_single_digit(mnist_type=mnist_type, seq=seq, kernel=kernel_size)
            #
            #     def read_nth_mnist_digit(self, seq, digit, type):
            #         counter = 0
            #         for item in self.mnist_array[type]:
            #             if item[0] == digit:
            #                 counter += 1
            #                 if counter == seq:
            #                     return item[1]
            #
            #     def read_image(self, index, type):
            #         # Reads an image from MNIST matching the index number requested in the function
            #         # global mnist_iterator
            #         tmp = 1
            #         if type == "training":
            #             image_db = self.mnist_training_iterator
            #         elif type == "test":
            #             image_db = self.mnist_test_iterator
            #         else:
            #             print("ERROR: Invalid MNIST type")
            #         for labeledImage in image_db:
            #             tmp += 1
            #             if tmp == index:
            #                 # print(i[1])
            #                 img = labeledImage[1]
            #                 label = labeledImage[0]
            #                 return img, label
            #
            # class Camera:
            #
            # class Microphone:
            #
            # class Keyboard:
            #
            # class Mouse:
            #


        # def initialize():
        #     """
        #     This function will monitor the IPU folder and other possible input devices such as a camera or mic for data and
        #     pass them along to the IPU module to have them converted to neuronal activity that in turn can be passed to cortical
        #     areas via FCL injection.
        #     """
        #     print(
        #         "\n\n\n\n\n**** *** **  Initializing the IPU Controller  **** * * **** ** ** * * *** ** *** *\n\n\n\n ")
        #     # todo: figure it its best to enable devices using the following if statements or using class instantiation within..
        #     # ...ipu_controller function
        #     # Initialize IPU devices
        #     if runtime_data.parameters['IPU']['folder_monitor']:
        #         folder_monitor.initialize()
        #
        #     # IPU feeder thread processes input stimuli and pass it along to the corresponding IPU module.
        #     if runtime_data.parameters['IPU']['mnist']:
        #         mnist_controller_thread = Thread(
        #             target=mnist_controller,
        #             args=(
        #                 runtime_data.watchdog_queue,
        #                 runtime_data.fcl_queue,
        #             ),
        #             name="MNIST_Controller",
        #             daemon=True
        #         )
        #         mnist_controller_thread.start()
        #         print(">> >> MNIST Controller thread has started.")


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

            # runtime_data.last_ipu_activity = datetime.now()



        # def mnist_load_queue(target_queue):
        #     # todo: The following is experimental and needs to be rebuilt
        #     mnist = MNIST()
        #     mnist_img = mnist.mnist_array['training'][5][1]
        #     print_mnist_img_raw(mnist_img)
        #     # Building the list of visual corticothalamic layers associated with vision
        #     visual_cortical_layers = cortical_sub_group_members('t_vision')
        #     fcl_entry = {}
        #     for cortical_layer in visual_cortical_layers:
        #         neuron_list = Image.convert_image_locations_to_neuron_ids(image_locations=mnist_img,
        #                                                                   cortical_area=cortical_layer)
        #         fcl_entry[cortical_layer] = neuron_list
        #     target_queue.put(fcl_entry)
        #     # todo: exiting immediately for test purposes
        #     runtime_data.exit_condition = True
        #     runtime_data.parameters["Switches"]["ready_to_exit_burst"] = True
        #
        # def mnist_controller(watchdoq_queue, fcl_queue):
        #     print("<> <> <> <> <> <> <> <> <>        <> <> <> <> <> <>      <> <> <> <> <> <> <>")
        #     while not runtime_data.exit_condition:
        #         try:
        #             mnist_load_queue(fcl_queue)
        #         except Exception as e:
        #             traceback.print_exc()
        #         finally:
        #             runtime_data.last_ipu_activity = datetime.now()
        #     time.sleep(2)


    # class Processor:
        # class Image:
        #     """
        #     This module acts as an eye. An input processing module responsible for processing visual raw data and activating the
        #     ganglion cells that act as the gateway to the visual cortical pathways.
        #     """
        #     import os
        #     import struct
        #     import numpy as np
        #     from math import floor
        #     from datetime import datetime
        #     from inf import runtime_data, settings
        #     from inf.db_handler import MongoManagement
        #     from inf.disk_ops import save_processed_mnist_to_disk
        #     from evo.neuron import neuron_finder
        #     from evo.neuroembryogenesis import cortical_sub_group_members
        #
        #     def polarize_img(img):
        #         polarized_image = mnist.read_nth_mnist_digit(seq=seq, digit=num, type=mnist_type)
        #
        #         # polarized_image = mnist.mnist_img_fetcher_mongo(num=num,
        #         #                                                 kernel_size=kernel_size,
        #         #                                                 seq=seq,
        #         #                                                 mnist_type=mnist_type,
        #         #                                                 random_num=random_num)
        #         if runtime_data.parameters['Logs']['print_polarized_img']:
        #             image = polarized_image['original_image']
        #             npimage = np.array(image)
        #             for _ in npimage:
        #                 print(_)
        #         return polarized_image
        #
        #     def image_to_neuron_list(image, polarize=False):
        #         """
        #         Plays the role of Ganglionic Layer in retina.
        #
        #         Responsible for converting visual stimuli to neuronal activity so it can be passed along the cortical pathways
        #         Input: Standard Image Data
        #         Output: List of neurons from various Thalamic Visual layers (LGN) that are activated
        #
        #         Output format:
        #             neuron_list =
        #                 {
        #                     "cortical_area_1" : {list of activated neurons},
        #                     "cortical_area_2" : {list of activated neurons},
        #                     "cortical_area_3" : {list of activated neurons}
        #                 }
        #         """
        #
        #         neuron_list = {}
        #         vision_group = cortical_sub_group_members('t_vision')
        #
        #         # Polarize image as an option
        #         if polarize:
        #             polarized_image = polarize_img(image)
        #
        #         # Convert image data to list of neurons
        #         for cortical_area in vision_group:
        #             neuron_list[cortical_area] = set()
        #             cortical_direction_sensitivity = runtime_data.genome['blueprint'][cortical_area][
        #                 'direction_sensitivity']
        #
        #             if polarize:
        #                 for key in polarized_image:
        #                     if key == cortical_direction_sensitivity:
        #                         try:
        #                             # print(np.array2string(np.array(polarized_image[cortical_direction_sensitivity]), max_line_width=np.inf))
        #
        #                             # ipu_vision_array = \
        #                             #     IPU_vision.Image.convert_direction_matrix_to_coordinates(
        #                             #         polarized_image[cortical_direction_sensitivity])
        #
        #                             ipu_vision_array = polarized_image[cortical_direction_sensitivity]
        #                             neuron_id_list = Image.convert_image_locations_to_neuron_ids(ipu_vision_array,
        #                                                                                          cortical_area)
        #                             neuron_list[cortical_area].update(set(neuron_id_list))
        #
        #                             if runtime_data.parameters['Logs']['print_activation_counters']:
        #                                 print("\n Bipolar cell activation count in %s is  %i" % (
        #                                 cortical_area, len(ipu_vision_array)))
        #                                 print("Neuron id count activated in layer %s is %i\n\n" %
        #                                       (cortical_area, len(neuron_id_list)))
        #
        #                         except:
        #                             print("Error on direction selectivity")
        #             else:
        #                 neuron_id_list = Image.convert_image_locations_to_neuron_ids(image, cortical_area)
        #                 neuron_list[cortical_area].update(set(neuron_id_list))
        #
        #         return neuron_list
        #
        #     def print_polarized_image(polarized_image, cortical_area):
        #         if runtime_data.parameters['Logs']['print_polarized_img']:
        #             print("\nPrinting polarized image for ", cortical_area)
        #             cortical_direction_sensitivity = runtime_data.genome['blueprint'][cortical_area][
        #                 'direction_sensitivity']
        #             for row in polarized_image[cortical_direction_sensitivity]:
        #                 print(" ***")
        #                 for item in row:
        #                     print(settings.Bcolors.YELLOW + item + settings.Bcolors.ENDC, end='')
        #                     if item == '':
        #                         print(settings.Bcolors.RED + '.' + settings.Bcolors.ENDC, end='')
        #
        #     class Filter:
        #         @staticmethod
        #         def brightness(image):
        #             new_image = np.zeros(image.shape)
        #             for x in range(image.shape[0]):
        #                 for y in range(image.shape[1]):
        #                     if image[x, y] >= runtime_data.genome["image_color_intensity_tolerance"]:
        #                         new_image[x, y] = image[x, y]
        #                     else:
        #                         new_image[x, y] = 1
        #             return new_image
        #
        #         @staticmethod
        #         def contrast(image, kernel_size):
        #             """This function simulates the effect of Amacrine and Horizontal cells within human Retina"""
        #             if divmod(kernel_size, 2)[1] == 0:
        #                 print("Error: Kernel size should only be Odd number!")
        #                 return
        #             row_index = 0
        #             col_index = 0
        #             new_image = [[] for x in range(np.shape(image)[1])]
        #             for row in image:
        #                 for row_item in row:
        #                     kernel_values = Image.image_read_by_block(image, kernel_size, [row_index, col_index])
        #                     cell_value = Kernel.kernel_contrast(kernel_values=kernel_values, kernel_size=kernel_size)
        #                     new_image[row_index].append(cell_value)
        #                     col_index += 1
        #                 col_index = 0
        #                 row_index += 1
        #             new_image = np.asarray(new_image, dtype=np.int)
        #
        #             # print("Pre-normalized image:\n", new_image)
        #
        #             # Normalize pixel values
        #             image_max_value = np.amax(new_image)
        #             # print("Max value:", image_max_value)
        #             row_index = 0
        #             col_index = 0
        #             normalized_image = [[] for x in range(np.shape(new_image)[1])]
        #             for row in new_image:
        #                 for row_item in row:
        #                     # 255 is the max intensity value that each image cell can be
        #                     normalized_value = floor(row_item * 255 / image_max_value)
        #                     normalized_image[row_index].append(normalized_value)
        #                     col_index += 1
        #                 col_index = 0
        #                 row_index += 1
        #             # print("NNN\n", normalized_image)
        #             # normalized_image = np.asarray(normalized_image, dtype=np.int)
        #             return normalized_image
        #
        #         @staticmethod
        #         def direction(kernel_values, kernel_size, direction_key):
        #             """Function to apply a particular filter to a kernel region of any size"""
        #             # end_result = {}
        #             result = np.zeros((kernel_size, kernel_size))
        #             filter_value = runtime_data.genome["IPU_vision_filters"][str(kernel_size)][direction_key]
        #             for i in range(0, kernel_size):
        #                 for ii in range(0, kernel_size):
        #                     result[i][ii] = kernel_values[i][ii] * filter_value[i][ii]
        #                     ii += 1
        #                 i += 1
        #             # end_result[direction_key] = result
        #             return result
        #
        #         @staticmethod
        #         def monochrome(image):
        #             """This function converts a gray-scale image to monochrome by setting all the pixels below a threshold to
        #             zero and above that threshold to 255."""
        #             row_index = 0
        #             col_index = 0
        #             new_image = [[] for x in range(np.shape(image)[1])]
        #             for row in image:
        #                 for row_item in row:
        #                     if row_item < runtime_data.parameters["InitData"]["image_monochromization_threshold"]:
        #                         new_image[row_index].append(0)
        #                     else:
        #                         new_image[row_index].append(255)
        #                     col_index += 1
        #                 col_index = 0
        #                 row_index += 1
        #             new_image = np.asarray(new_image, dtype=np.int)
        #             return new_image
        #
        #     class Kernel:
        #         @staticmethod
        #         def kernel_sizer(kernel_values):
        #             np.tmp = kernel_values
        #             kernel_size = np.shape(np.tmp)
        #             kernel_size = kernel_size[0]
        #             if divmod(kernel_size, 2)[1] == 0:
        #                 print("Error: Kernel size should only be Odd number!")
        #                 return
        #             return kernel_size
        #
        #         def kernel_direction(self, kernel_values):
        #             """
        #             Apply all filters from the IPU_vision_filters to the kernel and evaluate the best match
        #             Output is the Type of directional cell which will be activated
        #             :param kernel_size:
        #             :param kernel_values:
        #             :return:
        #
        #             The following conditions will estimate the line orientation angle into 4 standard options as following:
        #             1: /        2: \        3: -       4: |       0 : none
        #             Each if condition will perform a simple statistical analysis on the concentration of the pixels
        #             """
        #             # todo: Important >>> Something is wrong with this function returning incorrect values as direction label changes
        #
        #             end_result = {}
        #             kernel_size = self.kernel_sizer(kernel_values)
        #             # print("Kernel size is: ", kernel_size)
        #             for filter_entry in runtime_data.genome["IPU_vision_filters"][str(kernel_size)]:
        #                 end_result[filter_entry] = Filter.direction(kernel_values, kernel_size, filter_entry)
        #
        #             tmpArray = []
        #             # print('this is tmp before all appends', tmpArray)
        #             for entry in end_result:
        #                 # print(entry, "\n", end_result[entry], "\n\n\n")
        #                 sumation = np.sum(end_result[entry])
        #                 # print("Appending: %s Sum: %d \n End_result: \n %s" % (entry, summation,end_result[entry]))
        #                 # tmp = np.append(tmp, [entry, np.sum(end_result[entry])], axis=0)
        #                 tmpArray.append([entry, np.sum(end_result[entry])])
        #                 # print('***', tmpArray)
        #             # print("This is the end result: \n %s" % end_result)
        #             # print('tmp after appends %s' % tmpArray)
        #             maxValue = max(list(zip(*tmpArray))[1])
        #             # print("MaxValue: ", maxValue)
        #             maxValueIndex = list(zip(*tmpArray))[1].index(maxValue)
        #             direction = tmpArray[maxValueIndex][0]
        #             # direction = direction.replace('\\', '\')
        #             # print('max value is %s' % maxValue)
        #             # print('max index is %s' % maxValueIndex)
        #             # print('direction is %s' % direction)
        #             tempSum = 0
        #             for entry in tmpArray:
        #                 tempSum += abs(entry[1])
        #             if tempSum == 0:
        #                 return "^"
        #             return direction
        #
        #         @staticmethod
        #         def kernel_contrast(kernel_values, kernel_size):
        #             filtered_kernel = Filter.direction(kernel_values, kernel_size, 'o')
        #             # contrast_value = np.sum(kernel_values * filtered_kernel)
        #             contrast_value = np.sum(filtered_kernel)
        #             if contrast_value < 0:
        #                 contrast_value = 0
        #             return contrast_value
        #
        #         # todo: This function is super slow............
        #         def create_direction_matrix(self, image, kernel_size, direction_sensitivity=''):
        #             """
        #             Generates a Matrix where each element outlines the direction detected by the Kernel filters against each
        #             corresponding pixel in the image.
        #             """
        #             # print(">>> >>>", kernel_size, type(kernel_size))
        #             if divmod(kernel_size, 2)[1] == 0:
        #                 print("Error: Kernel size should only be Odd number!")
        #                 return
        #             row_index = 0
        #             col_index = 0
        #             direction_matrix = [[] for x in range(np.shape(image)[1])]
        #             for row in image:
        #                 for row_item in row:
        #                     direction = self.kernel_direction(
        #                         Image.image_read_by_block(image, kernel_size, [row_index, col_index]))
        #                     if direction == direction_sensitivity or direction_sensitivity == '':
        #                         direction_matrix[row_index].append(direction)
        #                     else:
        #                         direction_matrix[row_index].append('')
        #                     col_index += 1
        #                 col_index = 0
        #                 row_index += 1
        #             return direction_matrix
        #
        #         def create_direction_matrix2(self, image, kernel_size):
        #             """
        #             Generates a Matrix where each element outlines the direction detected by the Kernel filters against each
        #             corresponding pixel in the image.
        #             """
        #             if divmod(kernel_size, 2)[1] == 0:
        #                 print("Error: Kernel size should only be Odd number!")
        #                 return
        #             direction_sensitivity_options = runtime_data.genome["IPU_vision_filters"][str(kernel_size)]
        #             direction_matrix = {}
        #
        #             row_index = 0
        #             col_index = 0
        #
        #             for direction_sensitivity in direction_sensitivity_options:
        #                 direction_matrix[direction_sensitivity] = []
        #
        #             for row in image:
        #                 for row_item in row:
        #                     image_block = Image.image_read_by_block(image, kernel_size, [row_index, col_index])
        #                     actual_direction = self.kernel_direction(image_block)
        #                     if actual_direction in direction_sensitivity_options:
        #                         direction_matrix[actual_direction].append([row_index, col_index])
        #                     col_index += 1
        #                 col_index = 0
        #                 row_index += 1
        #             return direction_matrix
        #
        #         @staticmethod
        #         def orientation_matrix(raw_image, orientation_key, kernel_size):
        #             """
        #             Function to produce an orientation matrix based on the raw image data
        #             """
        #             return
        #
        #     class Image:
        #         @staticmethod
        #         def resize_image(image):
        #             img = Image.image_read_by_block(image=image)
        #             new_image_dimension = runtime_data.parameters["InitData"]["image_magnification_factor"]
        #             new_size = (new_image_dimension, new_image_dimension)
        #             image = img.Image.resize_image(new_size)
        #             return image
        #
        #         @staticmethod
        #         def convert_image_to_coordinates(image):  # Image is currently assumed to be a 28 x 28 numpy array
        #             """
        #             Function responsible for reading an image and converting the pixel values to coordinates
        #             """
        #             # Note: currently set to function based on Gray scale image
        #             genome = runtime_data.genome
        #
        #             image_locations = []
        #             for x in range(image.shape[0]):
        #                 for y in range(image.shape[1]):
        #                     if image[x, y] >= genome["image_color_intensity_tolerance"]:
        #                         image_locations.append([x, y, 0])
        #
        #             # Image location will be fed to another function to identify the Id of neurons to be activated
        #             return image_locations
        #
        #         @staticmethod
        #         def convert_direction_matrix_to_coordinates(image):
        #             # print("Polarized image type = ", type(image))
        #             image_locations = []
        #             x = 0
        #             y = 0
        #             for row in image:
        #                 for column in row:
        #                     if image[x][y] != '':
        #                         image_locations.append([x, y, 0])
        #                     y += 1
        #                 y = 0
        #                 x += 1
        #             return image_locations
        #
        #         # todo: Cythonize this
        #         @staticmethod
        #         def convert_image_locations_to_neuron_ids_old(image_locations, cortical_area):
        #             """
        #             Queries the connectome for each location and provides the list of Neuron Ids matching the location
        #             :param image_locations:
        #             :return:
        #             """
        #             genome = runtime_data.genome
        #
        #             neuron_id_list = []
        #             for x in range(len(image_locations)):
        #                 # call the function to find neuron candidates for a given location
        #                 tmp = neuron_finder(cortical_area, image_locations[x], genome["location_tolerance"])
        #                 for item in tmp:
        #                     if (item is not None) and (neuron_id_list.count(item) == 0):
        #                         neuron_id_list.append(item)
        #
        #             return neuron_id_list
        #
        #         @staticmethod
        #         def convert_image_locations_to_neuron_ids(image_locations, cortical_area):
        #             """
        #             Queries the connectome for each location and provides the list of Neuron Ids matching the location
        #             :param image_locations:
        #             :return:
        #             """
        #             # print("$$$$$ $  $  $ $  $ $ Image locations:", image_locations)
        #             neuron_id_list = []
        #             for x in range(len(image_locations)):
        #                 # print(">> Image location item:", x)
        #
        #                 block_reference = str(image_locations[x][0]) + '-' + \
        #                                   str(image_locations[x][1]) + '-' + \
        #                                   str(0)
        #                 if block_reference in runtime_data.block_dic[cortical_area]:
        #                     neuron_list = runtime_data.block_dic[cortical_area][block_reference]
        #                     # print(">>..>> Neuron list:", neuron_list)
        #                     # print("XXXXXXXXXX    XXXXXXXXX     XXXXXXXX", cortical_area, block_reference, len(neuron_list))
        #                     for item in neuron_list:
        #                         if (item is not None) and (neuron_id_list.count(item) == 0):
        #                             neuron_id_list.append(item)
        #             # print("+++++++++\n\n\n-----------\n\n\n++++++++\n\n\nYYYYYYYY    YYYYYYYY     YYYYYYY", cortical_area, neuron_id_list)
        #             return neuron_id_list
        #
        #         @staticmethod
        #         def image_read_by_block(image, kernel_size, seed_coordinates):
        #             x = seed_coordinates[0]
        #             y = seed_coordinates[1]
        #             if divmod(kernel_size, 2)[1] == 0:
        #                 print("Error: Kernel size should only be Odd number!")
        #                 return
        #             kernel_values = np.zeros((kernel_size, kernel_size))
        #             scan_length = divmod(kernel_size, 2)[0]
        #             for a in range(0, kernel_size):
        #                 for b in range(0, kernel_size):
        #                     if ((x - scan_length + a >= 0) and (y - scan_length + b >= 0) and (
        #                             x - scan_length + a < np.shape(image)[0])
        #                             and (y - scan_length + b < np.shape(image)[1])):
        #                         kernel_values[a, b] = image[x - scan_length + a, y - scan_length + b]
        #             return kernel_values
        #
        #         # todo: Need to add a method to combine multiple IPU layer data into a single one
        #         #        -Think how to build a direction agnostic representation of an object
        #
        #         @staticmethod
        #         def image_processing():
        #             """
        #             Function to read an image from a file and have it converted to it's fundamental components
        #             """
        #             return
        #
        #         @staticmethod
        #         def image_orientation_detector():
        #             """
        #             Performs higher level analysis to detect the direction of an image
        #             """
        #             # todo: need to figure which processing layer this belongs to. It might need to go thru entire stack
        #             return
        #
        #         @staticmethod
        #         def direction_stats(image_block):
        #             """
        #             Reads direction Kernel data and returns statistics on the percentage of each direction
        #             :param kernel:
        #             :return:
        #             """
        #             # direction_matrix = (image, kernel_size))
        #             # print(image)
        #
        #             direction_matrix = ''
        #             for row in image_block:
        #                 for item in row:
        #                     direction_matrix = direction_matrix + str(item)
        #
        #             # generate a list of all unique Characters present in the image block
        #             unique_chars = []
        #             for item in direction_matrix:
        #                 if unique_chars.count(item) == 0 and item != ' ':
        #                     unique_chars.append(item)
        #             # print('list of unique chars = %s' % unique_chars)
        #
        #             # Count number of occurrences of each unique character
        #             counts = []
        #             for item in unique_chars:
        #                 counts.append([item, direction_matrix.count(item)])
        #
        #             # Calculate the percentage of usage of each word
        #             stats = []
        #             count_total = direction_matrix.__len__() - direction_matrix.count(' ')
        #             for key in range(0, counts.__len__()):
        #                 stats.append([counts[key][0], str(counts[key][1] * 100 / float(count_total)) + ' %'])
        #
        #             return stats

        # class UTF:
        #     from inf import runtime_data
        #
        #     def convert_char_to_fire_list(char):
        #         utf_value = ord(char)
        #         fire_set = set()
        #         for key in runtime_data.brain["utf8_ipu"]:
        #             if utf_value == runtime_data.brain["utf8_ipu"][key]['soma_location'][0][2]:
        #                 fire_set.add(key)
        #         return fire_set

        # class Range:
        #     """
        #     This module processes LIDAR data from a message queue and converts it to neuronal activity
        #     within the proximity cortical area.
        #     """
        #     from math import inf
        #     from evo.neuron import block_reference_builder
        #     from evo.blocks import neurons_in_the_block
        #     from inf import runtime_data
        #
        #     # TODO: Generalize this module to process any data with a linear range e.g. lidar, battery, sonar, temperature, etc.
        #
        #     def range_to_coords(cortical_area, range_data, range_min, range_max, threshold):
        #         """
        #         This multi purpose function converts data in a linear range format to coordinates in target cortical area.
        #
        #         :param range_data: float array of detection distances
        #         :param threshold: threshold for detection distance (int)
        #         :return: list of tuple detection locations (x, y, z)
        #         """
        #         x_max = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries'][0]
        #         y_max = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries'][1]
        #         z_max = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries'][2]
        #         detection_locations = list
        #         if hasattr(range_data, '__iter__'):
        #             for idx, dist in enumerate(range_data):
        #                 if dist != inf:
        #                     dist_map = map_value(dist, range_min, range_max, 0, z_max)
        #                     if dist_map <= threshold:
        #                         x = idx
        #                         y = y_max // 2
        #                         z = dist_map
        #                         detection_locations.append((x, y, int(z)))
        #             return detection_locations
        #         else:
        #             try:
        #                 dist_map = round(map_value(range_data, range_min, range_max, 0, z_max))
        #             except TypeError:
        #                 dist_map = 0
        #                 print("Type Error in sonar_to_coords...")
        #
        #             if dist_map != 0:
        #                 x = x_max // 2
        #                 y = y_max // 2
        #                 z = dist_map
        #                 return [(x, y, z - 1)]
        #
        #     # TODO: make detection threshold part of config
        #     def lidar_to_coords(lidar_data, threshold=5):
        #         """ Converts turtlebot LIDAR data to coordinates in
        #         the proximity cortical area.
        #
        #         :param lidar_data: float array of detection distances
        #         :param threshold: threshold for detection distance (int)
        #         :return: list of tuple detection locations (x, y, z)
        #         """
        #         # turtlebot3 specs/documentation (in meters)
        #         LIDAR_MIN = 0.12
        #         LIDAR_MAX = 3.5
        #
        #         Y_MAX = runtime_data.genome['blueprint'] \
        #             ['proximity_ipu'] \
        #             ['neuron_params'] \
        #             ['block_boundaries'][1]
        #
        #         Z_MAX = runtime_data.genome['blueprint'] \
        #             ['proximity_ipu'] \
        #             ['neuron_params'] \
        #             ['block_boundaries'][2]
        #
        #         detection_locations = []
        #         for idx, dist in enumerate(lidar_data):
        #             if dist != inf:
        #                 dist_map = map_value(dist, LIDAR_MIN, LIDAR_MAX, 0, Z_MAX)
        #                 if dist_map <= threshold:
        #                     x = idx
        #                     y = Y_MAX // 2
        #                     z = dist_map
        #                     detection_locations.append((x, y, int(z)))
        #         return detection_locations
        #
        #     def sonar_to_coords(sonar_data, threshold=10):
        #         """ Converts SONAR data from sensor to coordinates in
        #         the proximity cortical area.
        #
        #         :param sonar_data: detection distance (int)
        #         :param threshold: threshold for detection distance (int)
        #         :return: list containing single tuple detection location (x, y, z)
        #         """
        #         # HC-SR04 datasheet specs (in cm)
        #         # TODO: parameterize min max vals for various sensors/scenarios
        #         # updated these min/max values to reflect those in gazebo simulation
        #         SONAR_MIN = 0
        #         SONAR_MAX = 4
        #
        #         X_MAX = runtime_data.genome['blueprint'] \
        #             ['proximity_ipu'] \
        #             ['neuron_params'] \
        #             ['block_boundaries'][0]
        #
        #         Y_MAX = runtime_data.genome['blueprint'] \
        #             ['proximity_ipu'] \
        #             ['neuron_params'] \
        #             ['block_boundaries'][1]
        #
        #         Z_MAX = runtime_data.genome['blueprint'] \
        #             ['proximity_ipu'] \
        #             ['neuron_params'] \
        #             ['block_boundaries'][2]
        #
        #         try:
        #             dist_map = round(map_value(sonar_data, SONAR_MIN, SONAR_MAX, 0, Z_MAX - 1))
        #         except TypeError:
        #             dist_map = 0
        #             print("Type Error in sonar_to_coords...")
        #
        #         if dist_map != 0:
        #             x = X_MAX // 2
        #             y = Y_MAX // 2
        #             z = dist_map
        #             return [(x, y, z)]
        #
        #     def coords_to_neuron_ids(detection_locations, cortical_area):
        #         """ Converts proximity detection locations to neuron IDs in
        #         the corresponding cortical area block.
        #
        #         :param detection_locations: list of tuple (x, y, z) detections
        #         :param cortical_area: name of cortical area (str)
        #         :return: list of neuron IDs (str)
        #         """
        #         neuron_ids = []
        #         if detection_locations is not None:
        #             for i in range(len(detection_locations)):
        #                 block_ref = block_reference_builder(detection_locations[i])
        #                 block_neurons = neurons_in_the_block(cortical_area, block_ref)
        #                 for neuron in block_neurons:
        #                     if neuron is not None and neuron not in neuron_ids:
        #                         neuron_ids.append(neuron)
        #         return neuron_ids
        #
        #     # todo: Move map value function out of proximity and to a more generic location
        #     def map_value(val, min1, max1, min2, max2):
        #         """ Performs linear transformation to map value from
        #         range 1 [min1, max1] to a value in range 2 [min2, max2].
        #
        #         :param val: value (int/float) being mapped
        #         :param min1: min of range 1
        #         :param max1: max of range 1
        #         :param min2: min of range 2
        #         :param max2: max of range 2
        #         :return: value mapped from range 1 to range 2
        #         """
        #         if val < min1:
        #             return min2
        #         if val > max1:
        #             return max2
        #
        #         mapped_value = (val - min1) * ((max2 - min2) / (max1 - min1)) + min2
        #
        #         if max2 >= mapped_value >= min2:
        #             return mapped_value

        # class Video:
        #
        # class Audio:




