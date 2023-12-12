import os
import re
import cv2
from time import sleep
from datetime import datetime
from feagi_agent import pns_gateway as pns

start_timer = 0
list_images = None


def image_identity_constructor(filename):
    id_message = dict()
    name_only = os.path.splitext(filename)[0]
    id_message[name_only] = 100
    return id_message


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


def read_single_image(absolute_to_file):
    return cv2.imread(absolute_to_file)


def count_images_in_folder(folder_path):
    """
    Generator to yield image files in the given folder.
    Useful for monitoring any new/delete file during runtime.
    """
    image_extensions = ['.jpg', '.jpeg', '.png', '.gif', '.bmp']
    for file_name in os.listdir(folder_path):
        if os.path.isfile(os.path.join(folder_path, file_name)):
            file_extension = os.path.splitext(file_name)[1].lower()
            if file_extension in image_extensions:
                yield file_name
    return file_name

def initalize_the_folder(path):
    global list_images
    list_images = gather_all_images(path)

def id_training(message_to_feagi, raw_frame=[]):
    global start_timer, list_images
    try:
        if start_timer == 0:
            start_timer = datetime.now()
            image = next(list_images)

        # Process for ID training
        raw_frame = feagi_trainer.read_single_image(capabilities['image_reader']['path'] + image)
        if not flag_blink:
            flag_blink = True
            raw_frame = capabilities['camera']['blink']
            capabilities['camera']['blink'] = []
        else:
            flag_blink = False
            capabilities['camera']['blink'] = raw_frame

        new_dict = image_identity_constructor(image)
        message_to_feagi = pns.append_sensory_data_for_feagi('training',
                                                             new_dict, message_to_feagi)
        if capabilities['image_reader']['pause'] <= int((datetime.now() -
                                                         start_timer).total_seconds()):
            start_timer = 0
        # Process ends for the ID training

        # Post image into vision
        previous_frame_data, rgb = retina.detect_change_edge(raw_frame, capabilities, "00",
                                                             size_list, previous_frame_data,
                                                             rgb)
        capabilities, feagi_settings['feagi_burst_speed'] = \
            retina.vision_progress(capabilities,
                                   feagi_opu_channel,
                                   api_address, feagi_settings, raw_frame)

        message_to_feagi = pns.generate_feagi_data(rgb, msg_counter, datetime.now(),
                                                   message_to_feagi)
        # Vision process ends

        sleep(feagi_settings['feagi_burst_speed'])
        feagi_settings['feagi_burst_speed'] = pns.check_refresh_rate(message_from_feagi,
                                                                     feagi_settings[
                                                                         'feagi_burst_speed'])
        pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings)
        message_to_feagi.clear()
    except StopIteration:
        # Check if you need to loop
        if capabilities['image_reader']['loop']:
            list_images = feagi_trainer.gather_all_images(capabilities['image_reader']['path'])
            image = next(list_images)
        else:
            break  # Exit the loop if you don't need to loop