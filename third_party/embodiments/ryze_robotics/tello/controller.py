import time
import requests
import configuration
from configuration import *
from djitellopy import Tello
from datetime import datetime
from version import __version__
from feagi_agent import retina
from feagi_agent import pns_gateway as pns
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


def action(obtained_signals, device_list, flying_flag):
    for device in device_list:
        if 'misc' in obtained_signals:
            for i in obtained_signals['misc']:
                misc_control(tello, i, battery)
        if flying_flag:
            if 'navigation' in obtained_signals:
                if obtained_signals['navigation']:
                    try:
                        data0 = obtained_signals['navigation'][0] * 10
                    except Exception as e:
                        data0 = 0
                    try:
                        data1 = obtained_signals['navigation'][1] * 10
                    except Exception as e:
                        data1 = 0
                    try:
                        data2 = obtained_signals['navigation'][2] * 10
                    except Exception as e:
                        data2 = 0
                    try:
                        speed = obtained_signals['speed'][0] * 10
                    except Exception as e:
                        speed = 0
                    navigate_to_xyz(tello, data0, data1, data2, speed)


if __name__ == '__main__':
    feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
    print("FEAGI AUTH URL ------- ", feagi_auth_url)
    runtime_data = dict()

    # # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - - - - - - - - - - - - - - - - #
    feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
        FEAGI.connect_to_feagi(feagi_settings, runtime_data, agent_settings, capabilities,
                               __version__)
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    # # # # # # # # # # # # Variables/Dictionaries section # # # # # # # # # # # # # # # - - - -
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
    msg_counter = 0
    flag_counter = 0
    checkpoint_total = 5
    flying_flag = False
    get_size_for_aptr_cortical = api_address + '/v1/FEAGI/genome/cortical_area?cortical_area=o_aptr'
    raw_aptr = requests.get(get_size_for_aptr_cortical).json()
    aptr_cortical_size = pns.fetch_aptr_size(10, raw_aptr, None)
    rgb = dict()
    rgb['camera'] = dict()
    capabilities['camera']['current_select'] = []
    device_list = pns.generate_OPU_list(capabilities)  # get the OPU sensors
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
            if capabilities['camera']['current_select']:
                capabilities['camera']["central_vision_resolution"] = capabilities['camera'][
                    'current_select']
            if capabilities['camera']['mirror']:
                data = retina.flip_video(data)
            previous_data_frame, rgb['camera'], capabilities['camera']['current_select'] = \
                pns.generate_rgb(data,
                                 capabilities['camera']['central_vision_allocation_percentage'][0],
                                 capabilities['camera']['central_vision_allocation_percentage'][1],
                                 capabilities['camera']["central_vision_resolution"],
                                 capabilities['camera']['peripheral_vision_resolution'],
                                 previous_data_frame,
                                 capabilities['camera']['current_select'],
                                 capabilities['camera']['iso_default'],
                                 capabilities['camera']["aperture_default"])
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
            message_from_feagi = pns.efferent_signaling(feagi_opu_channel)
            if message_from_feagi is not None:
                if aptr_cortical_size is None:
                    aptr_cortical_size = pns.check_aptr(raw_aptr)
                # Update the vres
                capabilities = pns.fetch_resolution_selected(message_from_feagi, capabilities)
                # Update the aptr
                capabilities = pns.fetch_aperture_data(message_from_feagi, capabilities,
                                                       aptr_cortical_size)
                # Update the ISO
                capabilities = pns.fetch_iso_data(message_from_feagi, capabilities,
                                                  aptr_cortical_size)
                obtained_signals = pns.obtain_opu_data(device_list, message_from_feagi)
                action(obtained_signals, device_list, flying_flag)

            # Preparing to send data to FEAGI
            configuration.message_to_feagi['timestamp'] = datetime.now()
            configuration.message_to_feagi['counter'] = msg_counter
            pns.afferent_signaling(message_to_feagi, feagi_ipu_channel, agent_settings)
            configuration.message_to_feagi.clear()
            if message_from_feagi is not None:
                feagi_settings['feagi_burst_speed'] = message_from_feagi['burst_frequency']
            time.sleep(feagi_settings['feagi_burst_speed'])
        except KeyboardInterrupt as ke:
            print("ERROR: ", ke)
            tello.end()
