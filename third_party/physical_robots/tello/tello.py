import cv2
import time
import requests
import numpy as np
import configuration
from configuration import *
from djitellopy import Tello
from datetime import datetime
import feagi_interface as FEAGI

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


def control_drone(self, direction, cm_distance):
    """
    self: instantiation
    direction: direction of forward, backward, left or right
    cm_distance: the default measurement distance from the current position to the goal
    """
    cm_distance = cm_distance * configuration.capabilities['motor']['power_coefficient']
    try:
        if direction == "l":
            self.send_command_without_return("{} {}".format("left", cm_distance))  ## left cm * 11 (max 100)
        elif direction == "r":
            self.send_command_without_return("{} {}".format("right", cm_distance))
        elif direction == "f":
            self.send_command_without_return("{} {}".format("forward", cm_distance))
        elif direction == "b":
            self.send_command_without_return("{} {}".format("back", cm_distance))
        elif direction == "u":
            self.send_command_without_return("{} {}".format("up", cm_distance))
        elif direction == "d":
            self.send_command_without_return("{} {}".format("down", cm_distance))
    except Exception as e:
        print("ERROR at: ", e)


def misc_control(self, data, battery_level):
    if data == 0:
        try:
            self.send_command_without_return("takeoff")
        except Exception as e:
            print("ERROR AT: ", e)
    if data == 1:
        self.send_command_without_return("land")
    if data == 2:
        try:
            if battery_level >= 50:
                self.send_command_without_return("flip {}".format("f"))
            else:
                print("ERROR! The battery is low. It must be at least above than 51% to be able to flip")
        except Exception as e:
            print("Error at: ", e)
    if data == 3:
        try:
            if battery_level >= 50:
                self.send_command_without_return("flip {}".format("b"))
            else:
                print("ERROR! The battery is low. It must be at least above than 51% to be able to flip")
        except Exception as e:
            print("Error at: ", e)
    if data == 4:
        try:
            if battery_level >= 50:
                self.send_command_without_return("flip {}".format("r"))
            else:
                print("ERROR! The battery is low. It must be at least above than 51% to be able to flip")
        except Exception as e:
            print("Error at: ", e)
    if data == 5:
        try:
            if battery_level >= 50:
                self.send_command_without_return("flip {}".format("l"))
            else:
                print("ERROR! The battery is low. It must be at least above than 51% to be able to flip")
        except Exception as e:
            print("Error at: ", e)


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


def convert_feagi_to_english(feagi):
    """
    convert feagi's data into human readable data
    """
    new_dict = dict()
    if feagi != {}:
        try:
            for i in feagi:
                if i == 0:
                    new_dict['f'] = feagi[i]
                if i == 1:
                    new_dict['b'] = feagi[i]
                if i == 2:
                    new_dict['r'] = feagi[i]
                if i == 3:
                    new_dict['l'] = feagi[i]
                if i == 4:
                    new_dict['u'] = feagi[i]
                if i == 5:
                    new_dict['d'] = feagi[i]
        except Exception as e:
            print("ERROR: ", e)
    return new_dict


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
            print("IN!")
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

    return {'camera': vision_dict}


def start_camera(self):
    """
    self as instantiation only
    """
    self.streamon()


def navigate_to_xyz(self, x=0, y=0, z=0, s=0):
    cmd = 'go {} {} {} {}'.format(x, y, z, s)
    self.send_control_command(cmd)


def convert_data_into_split(data):
    return -1 * (data - 10)


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
            dim = (configuration.capabilities['camera']['width'], configuration.capabilities['camera']['height'])
            data = tello.get_current_state()
            gyro = get_gyro(data)
            acc = get_accelerator(data)
            sonar = get_ultrasonic(data)
            bat = get_battery(data)
            battery = bat['battery_charge_level']
            data = full_frame(tello)
            resized = cv2.resize(data, dim, interpolation=cv2.INTER_AREA)
            data = ndarray_to_list(resized)
            rgb = get_rgb(data)
            print(rgb)
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
                opu_data = FEAGI.opu_processor(message_from_feagi)
                if 'misc' in opu_data:
                    for i in opu_data['misc']:
                        misc_control(tello, i, battery)
                if 'navigation' in opu_data:
                    if opu_data['navigation']:
                        try:
                            data0 = opu_data['navigation'][0] * 10
                        except Exception as e:
                            data0 = 0
                        try:
                            data1 = opu_data['navigation'][1] * 10
                        except Exception as e:
                            data1 = 0
                        try:
                            data2 = opu_data['navigation'][2] * 10
                        except Exception as e:
                            data2 = 0
                        try:
                            speed = opu_data['speed'][0] * 10
                        except Exception as e:
                            speed = 0
                        navigate_to_xyz(tello, data0, data1, data2, speed)

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


if __name__ == '__main__':
    main()
