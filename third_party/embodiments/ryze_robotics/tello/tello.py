import cv2
import time
import requests
import numpy as np
import configuration
from feagi_agent import retina as retina
from configuration import *
from djitellopy import Tello
from datetime import datetime
from feagi_agent import feagi_interface as FEAGI

previous_data_frame = dict()
flag = False


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
        new_data['gyro']['0'] = convert_gyro_into_feagi(full_data['pitch'],
                                                        capabilities['gyro']['resolution'],
                                                        capabilities['acc']['range'])
        new_data['gyro']['1'] = convert_gyro_into_feagi(full_data['roll'],
                                                        capabilities['gyro']['resolution'],
                                                        capabilities['acc']['range'])
        new_data['gyro']['2'] = convert_gyro_into_feagi(full_data['yaw'],
                                                        capabilities['gyro']['resolution'],
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
        new_data['accelerator']['0'] = convert_gyro_into_feagi(full_data['agx'],
                                                               capabilities['acc']['resolution'],
                                                               capabilities['acc']['range'])
        new_data['accelerator']['1'] = convert_gyro_into_feagi(full_data['agy'],
                                                               capabilities['acc']['resolution'],
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
            self.send_command_without_return(
                "{} {}".format("left", cm_distance))  ## left cm * 11 (max 100)
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
    global flag
    if data == 0:
        print("flag: ", flag)
        try:
            if flag == False:
                print("takeoff!")
                self.send_command_without_return("takeoff")
                flag = True
        except Exception as e:
            print("ERROR AT: ", e)
    if data == 1:
        print("flag: ", flag)
        if flag:
            print("landed!")
            self.send_command_without_return("land")
            flag = False
    if data == 2:
        try:
            if battery_level >= 50:
                self.send_command_without_return("flip {}".format("f"))
            else:
                print(
                    "ERROR! The battery is low. It must be at least above than 51% to be able to "
                    "flip")
        except Exception as e:
            print("Error at: ", e)
    if data == 3:
        try:
            if battery_level >= 50:
                self.send_command_without_return("flip {}".format("b"))
            else:
                print(
                    "ERROR! The battery is low. It must be at least above than 51% to be able to "
                    "flip")
        except Exception as e:
            print("Error at: ", e)
    if data == 4:
        try:
            if battery_level >= 50:
                self.send_command_without_return("flip {}".format("r"))
            else:
                print(
                    "ERROR! The battery is low. It must be at least above than 51% to be able to "
                    "flip")
        except Exception as e:
            print("Error at: ", e)
    if data == 5:
        try:
            if battery_level >= 50:
                self.send_command_without_return("flip {}".format("l"))
            else:
                print(
                    "ERROR! The battery is low. It must be at least above than 51% to be able to "
                    "flip")
        except Exception as e:
            print("Error at: ", e)


def full_frame(self):
    frame_read = self.get_frame_read()
    return frame_read.frame


def start_camera(self):
    """
    self as instantiation only
    """
    self.streamon()


def navigate_to_xyz(self, x=0, y=0, z=0, s=0):
    cmd = 'go {} {} {} {}'.format(x, y, z, s)
    self.send_control_command(cmd)


def convert_gyro_into_feagi(value, resolution, range_number):
    new_value = value - (range_number[0])
    return (new_value * resolution) / (range_number[1] - range_number[0])


def offset_z(value, resolution, range_number):
    """"
    Gravity is 9.8 m/s^2 however when the drone is on the table, it should be at zero. This
    offset will keep it to zero if the value is between than 2 and -2, it will be zero.
    """
    new_value = value - (range_number[0])
    new_value = (new_value * resolution) / (range_number[1] - range_number[0])
    if new_value > 2:
        return new_value
    elif new_value < -2:
        return new_value
    else:
        return 0


def main(feagi_auth_url):
    # # # # # # # # # # # # Variables/Dictionaries section # # # # # # # # # # # # # # # - - - -
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
    runtime_data = dict()
    msg_counter = 0
    flag_counter = 0
    checkpoint_total = 5
    flying_flag = False
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - #

    # # # # # # # # # # # # # # # FEAGI registration # # # # # # # # # # # # # # # # # # - - - -
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
    print("Connecting to FEAGI resources...")
    runtime_data["feagi_state"] = FEAGI.feagi_registration(feagi_auth_url=feagi_auth_url,
                                                           feagi_settings=feagi_settings,
                                                           agent_settings=agent_settings,
                                                           capabilities=capabilities)
    api_address = runtime_data['feagi_state']["feagi_url"]
    stimulation_period_endpoint = FEAGI.feagi_api_burst_engine()
    burst_counter_endpoint = FEAGI.feagi_api_burst_counter()

    agent_data_port = str(runtime_data["feagi_state"]['agent_state']['agent_data_port'])
    print("** **", runtime_data["feagi_state"])
    feagi_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])
    ipu_channel_address = FEAGI.feagi_outbound(feagi_settings['feagi_host'], agent_data_port)
    print("IPU_channel_address=", ipu_channel_address)
    opu_channel_address = FEAGI.feagi_outbound(feagi_settings['feagi_host'],
                                               runtime_data["feagi_state"]['feagi_opu_port'])
    feagi_ipu_channel = FEAGI.pub_initializer(ipu_channel_address, bind=False)
    feagi_opu_channel = FEAGI.sub_initializer(opu_address=opu_channel_address)
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - #

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - # Initializer section
    tello = Tello()
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - #
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
            retina_data = retina.frame_split(
                data,
                configuration.capabilities['camera']['retina_width_percent'],
                configuration.capabilities['camera']['retina_height_percent'])
            for i in retina_data:
                if 'C' in i:
                    retina_data[i] = retina.center_data_compression(
                        retina_data[i],
                        capabilities['camera'][
                            "central_vision_compression"])
                else:
                    retina_data[i] = retina.center_data_compression(
                        retina_data[i],
                        capabilities['camera']
                        ['peripheral_vision_compression'])
            rgb = dict()
            rgb['camera'] = dict()
            if previous_data_frame == {}:
                for i in retina_data:
                    previous_name = str(i) + "_prev"
                    previous_data_frame[previous_name] = {}
            for i in retina_data:
                name = i
                if 'prev' not in i:
                    data = retina.ndarray_to_list(retina_data[i])
                    if 'C' in i:
                        previous_name = str(i) + "_prev"
                        rgb_data, previous_data_frame[previous_name] = retina.get_rgb(
                            data,
                            capabilities[
                                'camera'][
                                'central_vision_compression'],
                            previous_data_frame[
                                previous_name],
                            name,
                            configuration.capabilities[
                                'camera'][
                                'deviation_threshold'])
                    else:
                        previous_name = str(i) + "_prev"
                        rgb_data, previous_data_frame[previous_name] = retina.get_rgb(
                            data,
                            capabilities[
                                'camera'][
                                'peripheral_vision_compression']
                            ,
                            previous_data_frame[
                                previous_name],
                            name,
                            configuration.capabilities[
                                'camera'][
                                'deviation_threshold'])
                    for a in rgb_data['camera']:
                        rgb['camera'][a] = rgb_data['camera'][a]
            configuration.message_to_feagi, bat = FEAGI.compose_message_to_feagi(
                original_message=gyro,
                data=configuration.message_to_feagi,
                battery=battery)
            configuration.message_to_feagi, bat = FEAGI.compose_message_to_feagi(
                original_message=acc,
                data=configuration.message_to_feagi,
                battery=battery)
            configuration.message_to_feagi, bat = FEAGI.compose_message_to_feagi(
                original_message=sonar,
                data=configuration.message_to_feagi,
                battery=battery)
            configuration.message_to_feagi, bat = FEAGI.compose_message_to_feagi(
                original_message=rgb,
                data=configuration.message_to_feagi,
                battery=battery)

            message_from_feagi = feagi_opu_channel.receive()
            if message_from_feagi is not None:
                opu_data = FEAGI.opu_processor(message_from_feagi)
                if 'misc' in opu_data:
                    for i in opu_data['misc']:
                        misc_control(tello, i, battery)
                if flying_flag:
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
            flag_counter += 1
            if flag_counter == int(checkpoint_total):
                feagi_burst_speed = requests.get(api_address + stimulation_period_endpoint).json()
                feagi_burst_counter = requests.get(api_address + burst_counter_endpoint).json()
                flag_counter = 0
                if feagi_burst_speed > 1:
                    checkpoint_total = 5
                if feagi_burst_speed < 1:
                    checkpoint_total = 5 / feagi_burst_speed
                if msg_counter < feagi_burst_counter:
                    feagi_opu_channel = FEAGI.sub_initializer(opu_address=opu_channel_address)
                    if feagi_burst_speed != feagi_settings['feagi_burst_speed']:
                        feagi_settings['feagi_burst_speed'] = feagi_burst_speed
                if feagi_burst_speed != feagi_settings['feagi_burst_speed']:
                    feagi_settings['feagi_burst_speed'] = feagi_burst_speed
                    msg_counter = feagi_burst_counter
            time.sleep(feagi_settings['feagi_burst_speed'])
        except KeyboardInterrupt as ke:
            print("ERROR: ", ke)
            tello.end()


if __name__ == '__main__':
    feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
    print("FEAGI AUTH URL ------- ", feagi_auth_url)
    while True:
        try:
            main(feagi_auth_url)
        except Exception as e:
            print(f"Controller run failed", e)
            traceback.print_exc()
            sleep(2)
