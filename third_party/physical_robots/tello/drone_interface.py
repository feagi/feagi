import time
import requests
import numpy as np
import configuration
import feagi_interface as FEAGI
from configuration import *
from datetime import datetime
from djitellopy import Tello

previous_frame_data = dict()


def get_battery(full_data):
    """
    full data should be a raw data of get_current_state().
    This will return the battery using the raw full data
    """
    new_data = dict()
    new_data['battery_charge_level'] = full_data['bat']
    return new_data


def get_ultrasonic(full_data):
    """
    full data should be a raw data of get_current_state().
    This will return the battery using the raw full data
    """
    new_data = dict()
    new_data['ultrasonic'] = full_data['tof'] * 0.01  # convert to meter unit
    if new_data:
        formatted_ultrasonic_data = {
            'ultrasonic': {
                sensor: data for sensor, data in enumerate([new_data['ultrasonic']])
            }
        }
    else:
        formatted_ultrasonic_data = {}
    return formatted_ultrasonic_data


def get_gyro(full_data):
    """
        full data should be a raw data of get_current_state().
        This function will return gyro data only.
        This gyro is 3 axis gyro.
    """
    new_data = dict()
    new_data['gyro'] = dict()
    try:
        new_data['gyro']['0'] = convert_gyro_into_feagi(full_data['pitch'], capabilities['gyro']['resolution'],
                                                        capabilities['acc']['range'])
        new_data['gyro']['1'] = convert_gyro_into_feagi(full_data['roll'], capabilities['gyro']['resolution'],
                                                        capabilities['acc']['range'])
        new_data['gyro']['2'] = convert_gyro_into_feagi(full_data['yaw'], capabilities['gyro']['resolution'],
                                                        capabilities['acc']['range'])
        return new_data
    except Exception as e:
        print("ERROR STARTS WITH: ", e)


def get_accelerator(full_data):
    """
    full data should be a raw data of get_current_state().
    This function will return acc data only.
    """
    new_data = dict()
    new_data['accelerator'] = dict()
    try:
        new_data['accelerator']['0'] = convert_gyro_into_feagi(full_data['agx'], capabilities['acc']['resolution'],
                                                               capabilities['acc']['range'])
        new_data['accelerator']['1'] = convert_gyro_into_feagi(full_data['agy'], capabilities['acc']['resolution'],
                                                               capabilities['acc']['range'])
        new_data['accelerator']['2'] = offset_z(full_data['agz'], capabilities['acc']['resolution'],
                                                capabilities['acc']['range'])
        return new_data
    except Exception as e:
        print("ERROR STARTS WITH: ", e)


def return_resolution(data):
    """
    try return_resolution(tello.get_frame_read()) in your main.
    data should be `tello.get_frame_read()`
    this will return height and width. Update your config with this numbers as well
    """
    frame_read = data
    height, width, _ = frame_read.frame.shape
    return height, width


def ndarray_to_list(array):
    array = array.flatten()
    new_list = (array.tolist())
    return new_list


def list_to_dict(full_list):
    test = dict()
    test['vision'] = dict()
    for x in range(len(full_list)):
        for i in full_list[x]:
            test['vision'][x] = i
    return test['vision']


def full_frame(self):
    frame_read = self.get_frame_read()
    return frame_read.frame


def get_rgb(frame):
    vision_dict = dict()
    frame_row_count = configuration.capabilities['camera']['width']
    frame_col_count = configuration.capabilities['camera']['height']

    x_vision = 0  # row counter
    y_vision = 0  # col counter
    z_vision = 0  # RGB counter

    try:
        previous_frame = previous_frame_data[0]
    except Exception:
        previous_frame = [0, 0]
    frame_len = len(previous_frame)
    try:
        if frame_len == frame_row_count * frame_col_count * 3:  # check to ensure frame length matches the
            # resolution setting
            for index in range(frame_len):
                if previous_frame[index] != frame[index]:
                    if (abs((previous_frame[index] - frame[index])) / 100) > \
                            configuration.capabilities['camera']['deviation_threshold']:
                        dict_key = str(x_vision) + '-' + str(y_vision) + '-' + str(z_vision)
                        vision_dict[dict_key] = frame[index]  # save the value for the changed index to the dict
                z_vision += 1
                if z_vision == 3:
                    z_vision = 0
                    y_vision += 1
                    if y_vision == frame_col_count:
                        y_vision = 0
                        x_vision += 1
        if frame != {}:
            previous_frame_data[0] = frame
    except Exception as e:
        print("Error: Raw data frame does not match frame resolution")
        print("Error due to this: ", e)

    return {'vision': vision_dict}


def start_camera(self):
    self.streamon()


def convert_gyro_into_feagi(value, resolution, range_number):
    new_value = value - (range_number[0])
    return (new_value * resolution) / (range_number[1] - range_number[0])


def offset_z(value, resolution, range_number):
    """"
    Gravity is 9.8 m/s^2 however when the drone is on the table, it should be at zero. This offset will keep it to zero
    if the value is between than 2 and -2, it will be zero.
    """
    new_value = value - (range_number[0])
    new_value = (new_value * resolution) / (range_number[1] - range_number[0])
    if new_value > 2:
        return new_value
    elif new_value < -2:
        return new_value
    else:
        return 0


def main():
    # # # # # # # # # # # # Variables/Dictionaries section # # # # # # # # # # # # # # #
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
    runtime_data = dict()
    msg_counter = 0
    flag = False
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #

    # # # # # # # # # # # # # # # FEAGI registration # # # # # # # # # # # # # # # # # #
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
    feagi_host, api_port = FEAGI.feagi_setting_for_registration()
    runtime_data["feagi_state"] = FEAGI.feagi_registration(feagi_host=feagi_host, api_port=api_port)
    ipu_channel_address = FEAGI.feagi_inbound(runtime_data["feagi_state"]['feagi_inbound_port_gazebo'])
    opu_channel_address = FEAGI.feagi_outbound(network_settings['feagi_host'],
                                               runtime_data["feagi_state"]['feagi_outbound_port'])
    feagi_ipu_channel = FEAGI.pub_initializer(ipu_channel_address)
    feagi_opu_channel = FEAGI.sub_initializer(opu_address=opu_channel_address)
    api_address = FEAGI.feagi_gui_address(feagi_host, api_port)
    stimulation_period_endpoint = FEAGI.feagi_api_burst_engine()
    burst_counter_endpoint = FEAGI.feagi_api_burst_counter()
    network_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
    #                            Initializer section
    tello = Tello()
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
    print("Connecting with Tello drone...")
    tello.connect()
    print("Connected with Tello drone.")
    start_camera(tello)

    while True:
        try:
            # Gather all data from the robot to prepare for FEAGI
            data = tello.get_current_state()
            gyro = get_gyro(data)
            acc = get_accelerator(data)
            sonar = get_ultrasonic(data)
            bat = get_battery(data)
            battery = bat['battery_charge_level']
            data = full_frame(tello)
            data = ndarray_to_list(data)
            # data = [0, 42, 115, 0, 42, 115, 0, 42, 115, 0, 42, 115, 0, 42, 115, 0, 42, 115, 0, 42, 115, 0, 42, 115, 0,
            #         42, 115, 0, 42, 115, 0, 42, 115, 0, 42, 115, 0, 42, 115, 0, 42, 115, 0, 42, 115, 0, 42, 115, 0, 42,
            #         115, 0, 42, 115, 0, 41, 114, 0, 40, 113, 0, 39, 112, 0, 37, 110, 0, 36, 109, 0, 36, 109, 0, 36, 109,
            #         0, 36, 109, 0, 36, 109, 0, 36, 109, 0, 36, 109, 0, 36, 109, 0, 36, 109, 0, 36, 109, 0, 36, 109, 0,
            #         36, 109, 0, 36, 109, 0, 36, 109, 0, 36, 109, 0, 36, 109, 0, 37, 110, 0, 37, 110, 0, 39, 112, 0, 39,
            #         112, 0, 39, 112, 0, 40, 113, 0, 40, 113, 0, 40, 113, 0, 40, 113, 0, 40, 113, 0, 40, 113, 0, 40, 113,
            #         0, 40, 113, 0, 40, 113, 0, 40, 113, 0, 40, 113, 0, 40, 113, 0, 40, 113, 0, 40, 113, 0, 40, 113, 0,
            #         40, 113, 0, 40, 113, 0, 40, 113, 0, 40, 113, 0, 40, 113, 0, 40, 113, 0, 40, 111, 0, 40, 111, 0, 41,
            #         109, 0, 41, 109, 0, 41, 109, 0, 41, 109, 0, 41, 109, 0, 41, 109, 0, 41, 109, 0, 41, 109, 0, 41, 109,
            #         0, 41, 109, 0, 41, 109, 0, 41, 109, 0, 41, 109, 0, 41, 109, 0, 42, 110, 0, 42, 110, 0, 42, 110, 0,
            #         42, 110, 0, 42, 110, 0, 42, 110, 0, 41, 109, 0, 41, 109, 0, 41, 109, 0, 39, 107, 0, 39, 107, 0, 39,
            #         107, 0, 39, 107, 0, 39, 107, 0, 41, 109, 0, 41, 109, 0, 41, 109, 0, 42, 110, 0, 42, 110, 0, 42, 110,
            #         0, 42, 110, 0, 42, 110, 0, 42, 110, 0, 42, 110]
            print("start")
            print("R: ", data[0], "G: ", data[1], "B: ", data[2])
            print("**" * 50)
            rgb = get_rgb(data)
            configuration.message_to_feagi, bat = FEAGI.compose_message_to_feagi(original_message=gyro,
                                                                                 data=configuration.message_to_feagi,
                                                                                 battery=battery)
            configuration.message_to_feagi, bat = FEAGI.compose_message_to_feagi(original_message=acc,
                                                                                 data=configuration.message_to_feagi,
                                                                                 battery=battery)
            configuration.message_to_feagi, bat = FEAGI.compose_message_to_feagi(original_message=sonar,
                                                                                 data=configuration.message_to_feagi,
                                                                                 battery=battery)
            configuration.message_to_feagi, bat = FEAGI.compose_message_to_feagi(original_message=rgb,
                                                                                 data=configuration.message_to_feagi,
                                                                                 battery=battery)
            message_from_feagi = feagi_opu_channel.receive()
            if message_from_feagi is not None:
                # print(message_from_feagi)
                pass

            # Preparing to send data to FEAGI
            configuration.message_to_feagi['timestamp'] = datetime.now()
            configuration.message_to_feagi['counter'] = msg_counter
            feagi_ipu_channel.send(configuration.message_to_feagi)
            configuration.message_to_feagi.clear()
            msg_counter += 1
            flag += 1
            if flag == 10:
                feagi_burst_speed = requests.get(api_address + stimulation_period_endpoint).json()
                feagi_burst_counter = requests.get(api_address + burst_counter_endpoint).json()
                flag = 0
                if msg_counter < feagi_burst_counter:
                    feagi_opu_channel = FEAGI.sub_initializer(opu_address=opu_channel_address)
                    if feagi_burst_speed != network_settings['feagi_burst_speed']:
                        network_settings['feagi_burst_speed'] = feagi_burst_speed
            time.sleep((network_settings['feagi_burst_speed']))

        except KeyboardInterrupt as ke:
            print("ERROR: ", ke)
            tello.end()
            break
    print("exited")


if __name__ == '__main__':
    main()
