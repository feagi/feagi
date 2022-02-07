
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

from inf import runtime_data








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


