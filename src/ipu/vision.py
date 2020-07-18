"""
This module acts as an eye. An input processing module responsible for processing visual raw data and activating the
ganglion cells that act as the gateway to the visual cortical pathways.
"""
import os
import struct
import numpy as np
from math import floor
from datetime import datetime
from inf import runtime_data, settings
from inf.db_handler import MongoManagement
from inf.disk_ops import save_processed_mnist_to_disk
from evo.neuron import neuron_finder


def cortical_sub_group_members(group):
    members = []
    for item in runtime_data.cortical_list:
        if runtime_data.genome['blueprint'][item]['sub_group_id'] == group:
            members.append(item)
    return members




def polarize_img(img):
    polarized_image = mnist.read_nth_mnist_digit(seq=seq, digit=num, type=mnist_type)

    # polarized_image = mnist.mnist_img_fetcher_mongo(num=num,
    #                                                 kernel_size=kernel_size,
    #                                                 seq=seq,
    #                                                 mnist_type=mnist_type,
    #                                                 random_num=random_num)
    if runtime_data.parameters['Logs']['print_polarized_img']:
        image = polarized_image['original_image']
        npimage = np.array(image)
        for _ in npimage:
            print(_)
    return polarized_image


def retina(image, polarize=False):
    """
    Responsible for converting visual stimuli to neuronal activity so it can be passed along the cortical pathways
    Input: Standard Image Data
    Output: List of neurons from various Thalamic Visual layers (LGN) that are activated

    Output format:
        neuron_list =
            {
                "cortical_area_1" : {list of activated neurons},
                "cortical_area_2" : {list of activated neurons},
                "cortical_area_3" : {list of activated neurons}
            }
    """

    neuron_list = {}
    vision_group = cortical_sub_group_members('t_vision')

    # Polarize image as an option
    if polarize:
        polarized_image = polarize_img(image)

    # Convert image data to list of neurons
    for cortical_area in vision_group:
        neuron_list[cortical_area] = set()
        cortical_direction_sensitivity = runtime_data.genome['blueprint'][cortical_area]['direction_sensitivity']

        if polarize:
            for key in polarized_image:
                if key == cortical_direction_sensitivity:
                    try:
                        # print(np.array2string(np.array(polarized_image[cortical_direction_sensitivity]), max_line_width=np.inf))

                        # ipu_vision_array = \
                        #     IPU_vision.Image.convert_direction_matrix_to_coordinates(
                        #         polarized_image[cortical_direction_sensitivity])

                        ipu_vision_array = polarized_image[cortical_direction_sensitivity]
                        neuron_id_list = Image.convert_image_locations_to_neuron_ids(ipu_vision_array, cortical_area)
                        neuron_list[cortical_area].update(set(neuron_id_list))

                        if runtime_data.parameters['Logs']['print_activation_counters']:
                            print("\n Bipolar cell activation count in %s is  %i" % (cortical_area, len(ipu_vision_array)))
                            print("Neuron id count activated in layer %s is %i\n\n" %
                                  (cortical_area, len(neuron_id_list)))

                    except:
                        print("Error on direction selectivity")
        else:
            neuron_id_list = Image.convert_image_locations_to_neuron_ids(image, cortical_area)
            neuron_list[cortical_area].update(set(neuron_id_list))

    return neuron_list


def print_polarized_image(polarized_image, cortical_area):
    if runtime_data.parameters['Logs']['print_polarized_img']:
        print("\nPrinting polarized image for ", cortical_area)
        cortical_direction_sensitivity = runtime_data.genome['blueprint'][cortical_area]['direction_sensitivity']
        for row in polarized_image[cortical_direction_sensitivity]:
            print(" ***")
            for item in row:
                print(settings.Bcolors.YELLOW + item + settings.Bcolors.ENDC, end='')
                if item == '':
                    print(settings.Bcolors.RED + '.' + settings.Bcolors.ENDC, end='')


class Filter:
    @staticmethod
    def brightness(image):
        new_image = np.zeros(image.shape)
        for x in range(image.shape[0]):
            for y in range(image.shape[1]):
                if image[x, y] >= runtime_data.genome["image_color_intensity_tolerance"]:
                    new_image[x, y] = image[x, y]
                else:
                    new_image[x, y] = 1
        return new_image

    @staticmethod
    def contrast(image, kernel_size):
        """This function simulates the effect of Amacrine and Horizontal cells within human Retina"""
        if divmod(kernel_size, 2)[1] == 0:
            print("Error: Kernel size should only be Odd number!")
            return
        row_index = 0
        col_index = 0
        new_image = [[] for x in range(np.shape(image)[1])]
        for row in image:
            for row_item in row:
                kernel_values = Image.image_read_by_block(image, kernel_size, [row_index, col_index])
                cell_value = Kernel.kernel_contrast(kernel_values=kernel_values, kernel_size=kernel_size)
                new_image[row_index].append(cell_value)
                col_index += 1
            col_index = 0
            row_index += 1
        new_image = np.asarray(new_image, dtype=np.int)

        # print("Pre-normalized image:\n", new_image)

        # Normalize pixel values
        image_max_value = np.amax(new_image)
        # print("Max value:", image_max_value)
        row_index = 0
        col_index = 0
        normalized_image = [[] for x in range(np.shape(new_image)[1])]
        for row in new_image:
            for row_item in row:
                # 255 is the max intensity value that each image cell can be
                normalized_value = floor(row_item * 255 / image_max_value)
                normalized_image[row_index].append(normalized_value)
                col_index += 1
            col_index = 0
            row_index += 1
        # print("NNN\n", normalized_image)
        # normalized_image = np.asarray(normalized_image, dtype=np.int)
        return normalized_image

    @staticmethod
    def direction(kernel_values, kernel_size, direction_key):
        """Function to apply a particular filter to a kernel region of any size"""
        # end_result = {}
        result = np.zeros((kernel_size, kernel_size))
        filter_value = runtime_data.genome["IPU_vision_filters"][str(kernel_size)][direction_key]
        for i in range(0, kernel_size):
            for ii in range(0, kernel_size):
                result[i][ii] = kernel_values[i][ii] * filter_value[i][ii]
                ii += 1
            i += 1
        # end_result[direction_key] = result
        return result

    @staticmethod
    def monochrome(image):
        """This function converts a gray-scale image to monochrome by setting all the pixels below a threshold to
        zero and above that threshold to 255."""
        row_index = 0
        col_index = 0
        new_image = [[] for x in range(np.shape(image)[1])]
        for row in image:
            for row_item in row:
                if row_item < runtime_data.parameters["InitData"]["image_monochromization_threshold"]:
                    new_image[row_index].append(0)
                else:
                    new_image[row_index].append(255)
                col_index += 1
            col_index = 0
            row_index += 1
        new_image = np.asarray(new_image, dtype=np.int)
        return new_image


class Kernel:
    @staticmethod
    def kernel_sizer(kernel_values):
        np.tmp = kernel_values
        kernel_size = np.shape(np.tmp)
        kernel_size = kernel_size[0]
        if divmod(kernel_size, 2)[1] == 0:
            print("Error: Kernel size should only be Odd number!")
            return
        return kernel_size

    def kernel_direction(self, kernel_values):
        """
        Apply all filters from the IPU_vision_filters to the kernel and evaluate the best match
        Output is the Type of directional cell which will be activated
        :param kernel_size:
        :param kernel_values:
        :return:

        The following conditions will estimate the line orientation angle into 4 standard options as following:
        1: /        2: \        3: -       4: |       0 : none
        Each if condition will perform a simple statistical analysis on the concentration of the pixels
        """
        # todo: Important >>> Something is wrong with this function returning incorrect values as direction label changes

        end_result = {}
        kernel_size = self.kernel_sizer(kernel_values)
        # print("Kernel size is: ", kernel_size)
        for filter_entry in runtime_data.genome["IPU_vision_filters"][str(kernel_size)]:
            end_result[filter_entry] = Filter.direction(kernel_values, kernel_size, filter_entry)

        tmpArray = []
        # print('this is tmp before all appends', tmpArray)
        for entry in end_result:
            # print(entry, "\n", end_result[entry], "\n\n\n")
            sumation = np.sum(end_result[entry])
            # print("Appending: %s Sum: %d \n End_result: \n %s" % (entry, summation,end_result[entry]))
            # tmp = np.append(tmp, [entry, np.sum(end_result[entry])], axis=0)
            tmpArray.append([entry, np.sum(end_result[entry])])
            # print('***', tmpArray)
        # print("This is the end result: \n %s" % end_result)
        # print('tmp after appends %s' % tmpArray)
        maxValue = max(list(zip(*tmpArray))[1])
        # print("MaxValue: ", maxValue)
        maxValueIndex = list(zip(*tmpArray))[1].index(maxValue)
        direction = tmpArray[maxValueIndex][0]
        # direction = direction.replace('\\', '\')
        # print('max value is %s' % maxValue)
        # print('max index is %s' % maxValueIndex)
        # print('direction is %s' % direction)
        tempSum = 0
        for entry in tmpArray:
            tempSum += abs(entry[1])
        if tempSum == 0:
            return "^"
        return direction

    @staticmethod
    def kernel_contrast(kernel_values, kernel_size):
        filtered_kernel = Filter.direction(kernel_values, kernel_size, 'o')
        # contrast_value = np.sum(kernel_values * filtered_kernel)
        contrast_value = np.sum(filtered_kernel)
        if contrast_value < 0:
            contrast_value = 0
        return contrast_value

    # todo: This function is super slow............
    def create_direction_matrix(self, image, kernel_size, direction_sensitivity=''):
        """
        Generates a Matrix where each element outlines the direction detected by the Kernel filters against each
        corresponding pixel in the image.
        """
        # print(">>> >>>", kernel_size, type(kernel_size))
        if divmod(kernel_size, 2)[1] == 0:
            print("Error: Kernel size should only be Odd number!")
            return
        row_index = 0
        col_index = 0
        direction_matrix = [[] for x in range(np.shape(image)[1])]
        for row in image:
            for row_item in row:
                direction = self.kernel_direction(Image.image_read_by_block(image, kernel_size, [row_index, col_index]))
                if direction == direction_sensitivity or direction_sensitivity == '':
                    direction_matrix[row_index].append(direction)
                else:
                    direction_matrix[row_index].append('')
                col_index += 1
            col_index = 0
            row_index += 1
        return direction_matrix

    def create_direction_matrix2(self, image, kernel_size):
        """
        Generates a Matrix where each element outlines the direction detected by the Kernel filters against each
        corresponding pixel in the image.
        """
        if divmod(kernel_size, 2)[1] == 0:
            print("Error: Kernel size should only be Odd number!")
            return
        direction_sensitivity_options = runtime_data.genome["IPU_vision_filters"][str(kernel_size)]
        direction_matrix = {}

        row_index = 0
        col_index = 0

        for direction_sensitivity in direction_sensitivity_options:
            direction_matrix[direction_sensitivity] = []

        for row in image:
            for row_item in row:
                image_block = Image.image_read_by_block(image, kernel_size, [row_index, col_index])
                actual_direction = self.kernel_direction(image_block)
                if actual_direction in direction_sensitivity_options:
                    direction_matrix[actual_direction].append([row_index, col_index])
                col_index += 1
            col_index = 0
            row_index += 1
        return direction_matrix

    @staticmethod
    def orientation_matrix(raw_image, orientation_key, kernel_size):
        """
        Function to produce an orientation matrix based on the raw image data
        """
        return


class Image:
    @staticmethod
    def resize_image(image):
        img = Image.image_read_by_block(image=image)
        new_image_dimension = runtime_data.parameters["InitData"]["image_magnification_factor"]
        new_size = (new_image_dimension, new_image_dimension)
        image = img.Image.resize_image(new_size)
        return image

    @staticmethod
    def convert_image_to_coordinates(image):   # Image is currently assumed to be a 28 x 28 numpy array
        """
        Function responsible for reading an image and converting the pixel values to coordinates
        """
        # Note: currently set to function based on Gray scale image
        genome = runtime_data.genome

        image_locations = []
        for x in range(image.shape[0]):
            for y in range(image.shape[1]):
                if image[x, y] >= genome["image_color_intensity_tolerance"]:
                    image_locations.append([x, y, 0])

        # Image location will be fed to another function to identify the Id of neurons to be activated
        return image_locations

    @staticmethod
    def convert_direction_matrix_to_coordinates(image):
        # print("Polarized image type = ", type(image))
        image_locations = []
        x = 0
        y = 0
        for row in image:
            for column in row:
                if image[x][y] != '':
                    image_locations.append([x, y, 0])
                y += 1
            y = 0
            x += 1
        return image_locations

    # todo: Cythonize this
    @staticmethod
    def convert_image_locations_to_neuron_ids_old(image_locations, cortical_area):
        """
        Queries the connectome for each location and provides the list of Neuron Ids matching the location
        :param image_locations:
        :return:
        """
        genome = runtime_data.genome

        neuron_id_list = []
        for x in range(len(image_locations)):
                # call the function to find neuron candidates for a given location
                tmp = neuron_finder(cortical_area, image_locations[x], genome["location_tolerance"])
                for item in tmp:
                    if (item is not None) and (neuron_id_list.count(item) == 0):
                        neuron_id_list.append(item)

        return neuron_id_list

    @staticmethod
    def convert_image_locations_to_neuron_ids(image_locations, cortical_area):
        """
        Queries the connectome for each location and provides the list of Neuron Ids matching the location
        :param image_locations:
        :return:
        """
        # print("$$$$$ $  $  $ $  $ $ Image locations:", image_locations)
        neuron_id_list = []
        for x in range(len(image_locations)):
            # print(">> Image location item:", x)

            block_reference = str(image_locations[x][0]) + '-' + \
                              str(image_locations[x][1]) + '-' + \
                              str(0)
            if block_reference in runtime_data.block_dic[cortical_area]:
                neuron_list = runtime_data.block_dic[cortical_area][block_reference]
                # print(">>..>> Neuron list:", neuron_list)
                # print("XXXXXXXXXX    XXXXXXXXX     XXXXXXXX", cortical_area, block_reference, len(neuron_list))
                for item in neuron_list:
                    if (item is not None) and (neuron_id_list.count(item) == 0):
                        neuron_id_list.append(item)
        # print("+++++++++\n\n\n-----------\n\n\n++++++++\n\n\nYYYYYYYY    YYYYYYYY     YYYYYYY", cortical_area, neuron_id_list)
        return neuron_id_list

    @staticmethod
    def image_read_by_block(image, kernel_size, seed_coordinates):
        x = seed_coordinates[0]
        y = seed_coordinates[1]
        if divmod(kernel_size, 2)[1] == 0:
            print("Error: Kernel size should only be Odd number!")
            return
        kernel_values = np.zeros((kernel_size, kernel_size))
        scan_length = divmod(kernel_size, 2)[0]
        for a in range(0, kernel_size):
            for b in range(0, kernel_size):
                if ((x-scan_length+a >= 0) and (y-scan_length+b >= 0) and (x-scan_length+a < np.shape(image)[0])
                        and (y-scan_length+b < np.shape(image)[1])):
                    kernel_values[a, b] = image[x-scan_length+a, y-scan_length+b]
        return kernel_values

    # todo: Need to add a method to combine multiple IPU layer data into a single one
    #        -Think how to build a direction agnostic representation of an object

    @staticmethod
    def image_processing():
        """
        Function to read an image from a file and have it converted to it's fundamental components
        """
        return

    @staticmethod
    def image_orientation_detector():
        """
        Performs higher level analysis to detect the direction of an image
        """
        # todo: need to figure which processing layer this belongs to. It might need to go thru entire stack
        return

    @staticmethod
    def direction_stats(image_block):
        """
        Reads direction Kernel data and returns statistics on the percentage of each direction
        :param kernel:
        :return:
        """
        # direction_matrix = (image, kernel_size))
        # print(image)

        direction_matrix = ''
        for row in image_block:
            for item in row:
                direction_matrix = direction_matrix + str(item)

        # generate a list of all unique Characters present in the image block
        unique_chars = []
        for item in direction_matrix:
            if unique_chars.count(item) == 0 and item != ' ':
                unique_chars.append(item)
        # print('list of unique chars = %s' % unique_chars)

        # Count number of occurrences of each unique character
        counts = []
        for item in unique_chars:
            counts.append([item, direction_matrix.count(item)])

        # Calculate the percentage of usage of each word
        stats = []
        count_total = direction_matrix.__len__() - direction_matrix.count(' ')
        for key in range(0, counts.__len__()):
            stats.append([counts[key][0], str(counts[key][1] * 100 / float(count_total)) + ' %'])

        return stats
