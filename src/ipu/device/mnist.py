"""
Copyright (c) 2019 Mohammad Nadji-Tehrani <m.nadji.tehrani@gmail.com>

This library is responsible for MNIST related operations from reading, plotting, processing, etc.
"""
import os
import struct
import numpy as np
import matplotlib.pyplot as plt
from inf.db_handler import MongoManagement
from inf.disk_ops import save_processed_mnist_to_disk
from inf import runtime_data
from ipu.processor.visual import Kernel
from datetime import datetime


class MNIST:
    def __init__(self):
        # global mnist_array, mnist_iterator
        self.mnist_training_iterator = self.read_mnist_raw(dataset_type="training")
        self.mnist_test_iterator = self.read_mnist_raw(dataset_type="testing")
        self.kernel = Kernel
        self.mnist_array = dict()
        self.mnist_array['training'] = []
        self.mnist_array['test'] = []
        for _ in self.mnist_training_iterator:
            self.mnist_array['training'].append(_)
        for __ in self.mnist_test_iterator:
            self.mnist_array['test'].append(__)
        self.mongo = MongoManagement()
        # print(len(mnist_array))

    def mnist_direction_matrix_builder_in_mongodb(self):
        kernel = Kernel()
        mnist_type_options = ['training', 'test']
        # kernel_size_options = [3, 5, 7]
        kernel_size_options = [5]

        for mnist_type in mnist_type_options:
            mnist_instance_seq = 0
            for entry in self.mnist_array[mnist_type]:
                mnist_instance_label, mnist_instance_data = entry

                for kernel_size in kernel_size_options:
                    direction_matrix_ = (kernel.create_direction_matrix2(image=mnist_instance_data,
                                                                         kernel_size=int(kernel_size)))

                    mnist_data = {
                        "mnist_type": mnist_type,
                        "mnist_seq": mnist_instance_seq,
                        "kernel_size": kernel_size,
                        "digit": str(mnist_instance_label),
                        "original_image": mnist_instance_data.tolist()
                    }

                    for direction in direction_matrix_:
                        mnist_data[direction] = direction_matrix_[direction]

                    self.mongo.insert_mnist_entry(mnist_data=mnist_data)
                    print("Added to MongoDb: Type=%s  Seq=%s  Kernel_size=%s  Digit=%s" % (mnist_type, mnist_instance_seq, kernel_size, mnist_instance_label))
                mnist_instance_seq += 1

    def mnist_direction_matrix_builder(self):
        template = {
            "3": {},
            "5": {},
            "7": {}
        }
        kernel = Kernel()
        for key in template:
            for number in range(0, 10):
                template[key][str(number)] = []

        all_of_mnist_training = template

        training_processing_start_time = datetime.now()
        counter = 0
        for kernel_size in all_of_mnist_training:
            for digit in all_of_mnist_training[kernel_size]:
                for entry in self.mnist_array['training']:
                    counter += 1
                    print("Kernel size:", kernel_size, "Digit:", digit, "Training counter: ", counter)
                    # if counter == 100:
                    #     counter = 0
                    #     break
                    mnist_instance_label, mnist_instance_data = entry
                    if str(mnist_instance_label) == digit:
                        direction_matrix_ = (kernel.create_direction_matrix2(image=mnist_instance_data,
                                                                             kernel_size=int(kernel_size)))
                        # direction_matrix_["original"] = mnist_instance_data
                        all_of_mnist_training[kernel_size][digit].append(direction_matrix_)

        save_processed_mnist_to_disk(data_type='training', data=all_of_mnist_training)
        print("Processed MNIST Training data has been saved to disk.")
        print(">> Processing of MNIST Training data set took: ", datetime.now() - training_processing_start_time)

        test_processing_start_time = datetime.now()
        counter = 0
        all_of_mnist_test = template

        for kernel_size in all_of_mnist_test:
            for digit in all_of_mnist_test[kernel_size]:
                for entry in self.mnist_array['test']:
                    counter += 1
                    print("Kernel size:", kernel_size, "Digit:", digit, "Test counter: ", counter)
                    # if counter == 100:
                    #     counter = 0
                    #     break
                    mnist_instance_label, mnist_instance_data = entry
                    if str(mnist_instance_label) == digit:
                        direction_matrix_ = (kernel.create_direction_matrix2(image=mnist_instance_data,
                                                                                  kernel_size=int(kernel_size)))
                        # direction_matrix_["original"] = mnist_instance_data
                        all_of_mnist_test[kernel_size][digit].append(direction_matrix_)

        save_processed_mnist_to_disk(data_type='test', data=all_of_mnist_test)
        print("Processed MNIST Test data has been saved to disk.")
        print(">> Processing of MNIST Test data set took: ", datetime.now() - test_processing_start_time)

    @staticmethod
    def read_mnist_raw(dataset_type):
        """
        Python function for importing the MNIST data set.  It returns an iterator
        of 2-tuples with the first element being the label and the second element
        being a numpy.uint8 2D array of pixel data for the given image.
        """

        path = runtime_data.parameters["Paths"]["mnist_path"]

        if dataset_type is "training":
                fname_img = os.path.join(path, 'train-images.idx3-ubyte')
                fname_lbl = os.path.join(path, 'train-labels.idx1-ubyte')
                # fname_img2 = os.path.join(path2, 'train-images.idx3-ubyte')
                # fname_lbl2 = os.path.join(path2, 'train-labels.idx1-ubyte')

        elif dataset_type is "testing":
            fname_img = os.path.join(path, 't10k-images.idx3-ubyte')
            fname_lbl = os.path.join(path, 't10k-labels.idx1-ubyte')
            # fname_img2 = os.path.join(path2, 't10k-images.idx3-ubyte')
            # fname_lbl2 = os.path.join(path2, 't10k-labels.idx1-ubyte')
        else:
            raise Exception(ValueError, "data set must be 'testing' or 'training'")

        # Load everything in some numpy arrays
        with open(fname_lbl, 'rb') as flbl:
            magic, num = struct.unpack(">II", flbl.read(8))
            lbl = np.fromfile(flbl, dtype=np.int8)

        with open(fname_img, 'rb') as fimg:
            magic, num, rows, cols = struct.unpack(">IIII", fimg.read(16))
            img = np.fromfile(fimg, dtype=np.uint8).reshape(len(lbl), rows, cols)

        get_img = lambda idx: (lbl[idx], img[idx])

        # Create an iterator which returns each image in turn
        for i in range(len(lbl)):
            yield get_img(i)

    def mnist_img_fetcher_mongo(self, num, kernel_size, seq, mnist_type, random_num=False):
        """
        Reads a number from pre-processed dataset and returns direction matrix data
        """

        if random_num:
            # todo: Need to create and call a MongoDb function to pull a random number
            return
        else:
            return self.mongo.mnist_read_nth_digit(mnist_type=mnist_type, n=seq, kernel_size=kernel_size, digit=num)
            # return self.mongo.mnist_read_single_digit(mnist_type=mnist_type, seq=seq, kernel=kernel_size)

    def read_nth_mnist_digit(self, seq, digit, type):
        counter = 0
        for item in self.mnist_array[type]:
            if item[0] == digit:
                counter += 1
                if counter == seq:
                    return item[1]

    def read_image(self, index, type):
        # Reads an image from MNIST matching the index number requested in the function
        # global mnist_iterator
        tmp = 1
        if type == "training":
            image_db = self.mnist_training_iterator
        elif type == "test":
            image_db = self.mnist_test_iterator
        else:
            print("ERROR: Invalid MNIST type")
        for labeledImage in image_db:
            tmp += 1
            if tmp == index:
                # print(i[1])
                img = labeledImage[1]
                label = labeledImage[0]
                return img, label


def mnist_plotter(mnist_type="training", subplot_dimension=5, desirable_label=6):
    counter = 0
    x_counter = 0
    y_counter = 0
    counter_limit = subplot_dimension * subplot_dimension
    f, axarr = plt.subplots(subplot_dimension, subplot_dimension)
    mnist = MNIST()
    for entry in mnist.mnist_array[mnist_type]:

        label = entry[0]

        if label == desirable_label:

            # The rest of columns are pixels
            pixels = entry[1:]

            # Make those columns into a array of 8-bits pixels
            # This array will be of 1D with length 784
            # The pixel intensity values are integers from 0 to 255
            pixels = np.array(pixels, dtype='uint8')

            # Reshape the array into 28 x 28 array (2-dimensional array)
            pixels = pixels.reshape((28, 28))

            # Plot
            axarr[y_counter, x_counter].imshow(pixels)
            # Turn off tick labels
            axarr[y_counter, x_counter].set_yticklabels([])
            axarr[y_counter, x_counter].set_xticklabels([])
            axarr[y_counter, x_counter].axis('off')

            counter += 1
            if counter == counter_limit:
                # plt.tight_layout()
                plt.axis('off')
                # plt.figure(figsize=(1000, 1000))
                plt.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=0.001, hspace=0.001)
                plt.show()
                return

            if x_counter == (subplot_dimension - 1):
                y_counter += 1
                x_counter = 0
            else:
                x_counter += 1


def print_mnist_img(num, seq, mnist_type):
    mnist = MNIST()

    if runtime_data.parameters['Logs']['print_mnist_img']:
        mnist_img = mnist.read_nth_mnist_digit(seq=seq, type=mnist_type, digit=num)
        try:
            mnist_img = mnist_img.tolist()
            for row in mnist_img:
                for item in row:
                    if item > 50:
                        print("O", end='  ')
                    else:
                        print('   ', end='')
                print('\n')
        except AttributeError:
            print("ERROR: Attribute error while printing image.", mnist_img)


def read_mnist_labels(dataset="training"):
    """
    For importing the MNIST data set.  It returns an iterator
    of 2-tuples with the first element being the label and the second element
    being a numpy.uint8 2D array of pixel data for the given image.
    """
    path = runtime_data.parameters['Paths']['mnist_path']

    if dataset is "training":
        fname_lbl = os.path.join(path, 'train-labels.idx1-ubyte')
    elif dataset is "testing":
        fname_lbl = os.path.join(path, 't10k-labels.idx1-ubyte')
    else:
        raise Exception(ValueError, "data set must be 'testing' or 'training'")

    # Load everything in some numpy arrays
    with open(fname_lbl, 'rb') as flbl:
        magic, num = struct.unpack(">II", flbl.read(8))
        lbl = np.fromfile(flbl, dtype=np.int8)

    get_img_lbl = lambda idx: (lbl[idx])

    # Create an iterator which returns each image in turn
    for i in range(len(lbl)):
        yield get_img_lbl(i)


if __name__ == "__main__":
    from inf import initialize
    initialize.init_parameters('../feagi_configuration.ini')

    mnist_plotter(mnist_type='training', desirable_label=4, subplot_dimension=4)
    mnist_plotter(mnist_type='test', desirable_label=4, subplot_dimension=4)
