import os
import re
import cv2
from feagi_agent import pns_gateway as pns


def gather_all_images(path_direction):
    """
    Generator to yield image files with specific pattern in filename.
    Useful for filtering training images provided by the user.
    Will ignore the file if it doesn't have #-#-# in the filename.
    """
    folder_path = path_direction
    files = os.listdir(folder_path)
    pattern = re.compile(r'\d+-\d+-\d+\..+')

    for filename in files:
        if pattern.match(filename):
            yield filename


def read_single_image(absolute_to_file, filename):
    id_message = dict()
    name_only = os.path.splitext(filename)[0]
    id_message[name_only] = 100
    return cv2.imread(absolute_to_file + filename), id_message


def initalize_the_folder(path):
    return gather_all_images(path)


def id_training_with_image(message_to_feagi, image, capabilities, start_timer,
                           flag_blink,
                           raw_frame=[]):
    # Process for ID training
    raw_frame, name_id = read_single_image(capabilities['image_reader']['path'], image)
    if not flag_blink:
        flag_blink = True
        raw_frame = capabilities['camera']['blink']
        capabilities['camera']['blink'] = []
    else:
        flag_blink = False
        capabilities['camera']['blink'] = raw_frame

    message_to_feagi = pns.append_sensory_data_for_feagi('training', name_id, message_to_feagi)
    # Process ends for the ID training
    return message_to_feagi, start_timer, image, flag_blink, raw_frame
