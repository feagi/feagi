import os
import re
import cv2
from time import sleep


def image_identity_constructor(filename):
    id_message = dict()
    name_only = os.path.splitext(filename)[0]
    id_message[name_only] = 100
    return id_message


def gather_all_images(path_direction):
    """
    There are training images provided by the user.Will ignore the file if it doesn't have #-#-# in
    the filename
    """
    folder_path = path_direction
    files = os.listdir(folder_path)
    pattern = re.compile(r'\d+-\d+-\d+\..+')
    filtered_files = [filename for filename in files if pattern.match(filename)]
    return filtered_files


def read_single_image(absolute_to_file):
    return cv2.imread(absolute_to_file)


def count_images_in_folder(folder_path):
    """
    count the images inside the folder. Useful for monitoring any new/delete file during runtime
    """
    image_extensions = ['.jpg', '.jpeg', '.png', '.gif', '.bmp']
    image_count = 0
    for file_name in os.listdir(folder_path):
        if os.path.isfile(os.path.join(folder_path, file_name)):
            file_extension = os.path.splitext(file_name)[1].lower()
            if file_extension in image_extensions:
                image_count += 1
    return image_count
