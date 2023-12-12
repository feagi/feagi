import os
import re
import cv2
from feagi_agent import pns_gateway as pns


def scan_the_folder(path_direction):
    """
    Generator to yield image files with specific pattern in filename.
    Useful for filtering training images provided by the user.
    Will ignore the file if it doesn't have #-#-# in the filename or isn't an image.
    """
    folder_path = path_direction
    files = os.listdir(folder_path)
    pattern = re.compile(r'\d+-\d+-\d+\..+')
    image_extensions = ['.jpg', '.jpeg', '.png', '.gif', '.bmp']
    for filename in files:
        if pattern.match(filename) and os.path.splitext(filename)[1].lower() in image_extensions:
            yield filename


def read_single_image(absolute_to_file, filename):
    id_message = dict()
    name_only = os.path.splitext(filename)[0]
    id_message[name_only] = 100
    return cv2.imread(absolute_to_file + filename), id_message


def id_training_with_image(message_to_feagi, image, capabilities, raw_frame=[]):
    # Process for ID training
    raw_frame, name_id = read_single_image(capabilities['image_reader']['path'], image)
    message_to_feagi = pns.append_sensory_data_for_feagi('training', name_id, message_to_feagi)
    # Process ends for the ID training
    return message_to_feagi, image, raw_frame
